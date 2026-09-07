#include "services/control.h"

#include <algorithm>
#include <cstdint>
#include <string>

#include "mw/utils/logger.h"
#include "robot_config.h"
#include "utils/topics.h"

namespace project
{
namespace services
{

Control::Control()
{
    const RobotConfig& robot = robotConfig();
    config_.max_v_mps = robot.plan_v_mps;
    config_.max_w_rps = robot.plan_w_rps;
}

void Control::configure()
{
    registerParameters();

    subscribe<types::Trajectory>(
        topics::kPlannerTrajectory, [this](const types::Trajectory& t) { onTrajectory(t); }
    );
    subscribe<types::Pose2D>(topics::kSlamPose, [this](const types::Pose2D& p) { onPose(p); });
    subscribe<types::Odometry>(topics::kSensorsOdometry, [this](const types::Odometry& o) { onOdometry(o); });
    subscribe<types::RobotState>(topics::kRobotState, [this](const types::RobotState& s) { onRobotState(s); });
    subscribe<types::SafetyStop>(topics::kSafetyStop, [this](const types::SafetyStop& s) { onSafetyStop(s); });

    scheduleTimer(50, [this] { tick(); }, "control");
}

void Control::registerParameters()
{
    registerParameter<double>("ctrl_lookahead_m", config_.lookahead_m);
    registerParameter<double>("ctrl_min_lookahead_m", config_.min_lookahead_m);
    registerParameter<double>("ctrl_max_v_mps", config_.max_v_mps);
    registerParameter<double>("ctrl_max_w_rps", config_.max_w_rps);
    registerParameter<double>("ctrl_goal_tolerance_m", config_.goal_tolerance_m);
    registerParameter<int>("ctrl_traj_max_age_ms", traj_max_age_ms_);
    registerParameter<double>("ctrl_recover_distance_m", recover_distance_m_);
    registerParameter<double>("ctrl_recover_speed_mps", recover_speed_mps_);
}

void Control::onTrajectory(const types::Trajectory& trajectory)
{
    trajectory_ = trajectory;
    have_trajectory_ = !trajectory.points.empty();
    done_ = false;
    if (!have_trajectory_)
    {
        publishZero();  // пустая траектория = «стой»
    }
}

void Control::onPose(const types::Pose2D& pose)
{
    pose_ = pose;
    have_pose_ = true;
}

void Control::onOdometry(const types::Odometry& odom)
{
    odom_ = odom;
}

void Control::onRobotState(const types::RobotState& state)
{
    robot_state_ = state;
}

void Control::onSafetyStop(const types::SafetyStop& stop)
{
    if (stop.stop && !safety_.stop)
    {
        LOGW("control: safety stop (%s)", stop.reason.c_str());
    }
    safety_ = stop;
}

void Control::publishZero()
{
    types::CmdVel zero;
    zero.timestamp = now();
    publish<types::CmdVel>(topics::kCmdVel, zero);
}

void Control::tick()
{
    // Откат после бампера: назад на recover_distance_m. Стоп по бамперу при этом не мешает —
    // иначе от препятствия не отъехать.
    if (recover_ticks_left_ > 0)
    {
        --recover_ticks_left_;
        types::CmdVel back;
        back.timestamp = now();
        back.vx = -recover_speed_mps_;
        publish<types::CmdVel>(topics::kCmdVel, back);
        if (recover_ticks_left_ == 0)
        {
            have_trajectory_ = false;  // прежняя вела в препятствие; ждём новую от Planner
            publishZero();
        }
        return;
    }
    // Safety-стоп: ноль каждый тик — перебивает и планировщик, и телеоперацию.
    if (safety_.stop)
    {
        const bool bumper_only = safety_.reason.find("bumper") != std::string::npos &&
                                 safety_.reason.find("cliff") == std::string::npos;
        if (bumper_only && recover_distance_m_ > 0 && have_trajectory_)
        {
            recover_ticks_left_ = std::max(1, static_cast<int>(recover_distance_m_ / recover_speed_mps_ / 0.05));
            LOGW("control: бампер (%s) — откат %.2f м", safety_.reason.c_str(), recover_distance_m_);
        }
        publishZero();
        return;
    }
    // В простое молчим: иначе наши нули забили бы телеоперацию с телефона
    // (controls/cmd_vel общий, у Platform побеждает последняя команда).
    if (!have_trajectory_ || !have_pose_ || done_)
    {
        return;
    }
    const uint64_t age_ms = (now() - trajectory_.timestamp) / 1000;
    if (age_ms > static_cast<uint64_t>(traj_max_age_ms_))
    {
        LOGW("control: траектория старше %d мс — стоп", traj_max_age_ms_);
        have_trajectory_ = false;
        publishZero();
        return;
    }

    const algo::PurePursuit pursuit(config_);
    const algo::PurePursuitOutput out = pursuit.Step(trajectory_, pose_);
    if (out.done)
    {
        done_ = true;
        publishZero();
        return;
    }
    types::CmdVel cmd;
    cmd.timestamp = now();
    cmd.vx = out.v;
    cmd.w = out.w;
    publish<types::CmdVel>(topics::kCmdVel, cmd);
}

}  // namespace services
}  // namespace project
