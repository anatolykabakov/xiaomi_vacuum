#include "services/mission.h"

#include "mw/utils/logger.h"
#include "robot_config.h"
#include "utils/topics.h"

namespace project
{
namespace services
{

Mission::Mission()
{
    const RobotConfig& robot = robotConfig();
    config_.robot_radius_m = robot.radius_m;
    config_.explorer.inflation_radius_m = robot.radius_m + 0.03;
    config_.coverage.inflation_radius_m = robot.radius_m + 0.03;
    config_.coverage.lane_width_m = 2 * robot.radius_m * 0.85;  // перекрытие полос 15 %
    mission_ = algo::CleaningMission(config_);
}

void Mission::configure()
{
    registerParameter<bool>("mission_autostart", autostart_);
    registerParameter<double>("mission_lane_width_m", config_.coverage.lane_width_m);
    registerParameter<double>("mission_dock_tolerance_m", config_.dock_tolerance_m);
    registerParameter<double>("mission_undock_distance_m", config_.undock_distance_m);
    registerParameter<bool>("mission_perimeter_pass", config_.coverage.perimeter_pass);
    registerParameter<int>("mission_max_blocked_retries", config_.max_blocked_retries);

    subscribe<types::OccupancyMap>(topics::kSlamMap, [this](const types::OccupancyMap& m) { onMap(m); });
    subscribe<types::Pose2D>(topics::kSlamPose, [this](const types::Pose2D& p) { onPose(p); });
    subscribe<types::PlannerState>(
        topics::kPlannerState, [this](const types::PlannerState& s) { onPlannerState(s); }
    );
    subscribe<types::MissionCommand>(
        topics::kMissionCommand, [this](const types::MissionCommand& c) { onCommand(c); }
    );

    scheduleTimer(1000, [this] { tick(); }, "mission");
}

void Mission::onMap(const types::OccupancyMap& map)
{
    map_ = map;
    have_map_ = true;
}

void Mission::onPose(const types::Pose2D& pose)
{
    pose_ = pose;
    have_pose_ = true;
}

void Mission::onPlannerState(const types::PlannerState& state)
{
    planner_state_ = state;
}

void Mission::onCommand(const types::MissionCommand& command)
{
    using Cmd = types::MissionCommand::Command;
    switch (command.command)
    {
        case Cmd::kStart:
            if (!have_pose_)
            {
                LOGW("mission: старт без позы — жду slam/pose");
                autostart_ = true;
                return;
            }
            mission_ = algo::CleaningMission(config_);  // параметры могли измениться
            mission_.Start(pose_);
            LOGI("mission: старт уборки, док (%.2f, %.2f)", pose_.x, pose_.y);
            break;
        case Cmd::kStop:
            mission_.Stop();
            {
                types::Goal stop;
                stop.type = types::Goal::Type::kStop;
                stop.timestamp = now();
                publish<types::Goal>(topics::kPlannerGoal, stop);
            }
            LOGI("mission: стоп");
            break;
        case Cmd::kReturn:
            mission_.ReturnToDock();
            LOGI("mission: возврат на док");
            break;
    }
    tick();
}

void Mission::tick()
{
    using State = types::MissionState::State;
    if (autostart_ && mission_.state() == State::kIdle && have_map_ && have_pose_)
    {
        autostart_ = false;
        mission_ = algo::CleaningMission(config_);
        mission_.Start(pose_);
        LOGI("mission: автостарт уборки, док (%.2f, %.2f)", pose_.x, pose_.y);
    }
    if (mission_.state() != State::kIdle && have_map_ && have_pose_)
    {
        const State before = mission_.state();
        types::Goal goal;
        if (mission_.Step(map_, pose_, planner_state_, goal))
        {
            goal.timestamp = now();
            publish<types::Goal>(topics::kPlannerGoal, goal);
        }
        if (mission_.state() != before)
        {
            LOGI("mission: %s", mission_.Progress(map_).detail.c_str());
        }
    }
    types::MissionState state = mission_.Progress(have_map_ ? map_ : types::OccupancyMap{});
    state.timestamp = now();
    publish<types::MissionState>(topics::kMissionState, state);
}

}  // namespace services
}  // namespace project
