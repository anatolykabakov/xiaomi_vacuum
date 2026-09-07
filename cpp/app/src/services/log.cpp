#include "services/log.h"

#include <sstream>

#include <spdlog/spdlog.h>

#include "utils/topics.h"

namespace project
{
namespace services
{

void Log::configure()
{
    subscribe<types::LaserScan>(topics::kSensorsLaser, [this](const types::LaserScan& s) { onLaser(s); });
    subscribe<types::Odometry>(topics::kSensorsOdometry, [this](const types::Odometry& o) { onOdometry(o); });
    subscribe<types::Imu>(topics::kSensorsImu, [this](const types::Imu& i) { onImu(i); });
    subscribe<types::CmdVel>(topics::kCmdVel, [this](const types::CmdVel& c) { onCmdVel(c); });
    subscribe<types::Pose2D>(topics::kSlamPose, [this](const types::Pose2D& p) { onPose(p); });
    subscribe<types::OccupancyMap>(topics::kSlamMap, [this](const types::OccupancyMap& m) { onMap(m); });
    subscribe<types::Trajectory>(
        topics::kPlannerTrajectory, [this](const types::Trajectory& t) { onTrajectory(t); }
    );
    subscribe<types::PlannerState>(
        topics::kPlannerState, [this](const types::PlannerState& s) { onPlannerState(s); }
    );
    subscribe<types::MissionState>(
        topics::kMissionState, [this](const types::MissionState& s) { onMissionState(s); }
    );
    subscribe<types::RobotState>(topics::kRobotState, [this](const types::RobotState& s) { onRobotState(s); });
    subscribe<types::SafetyStop>(topics::kSafetyStop, [this](const types::SafetyStop& s) { onSafetyStop(s); });
}

void Log::onLaser(const types::LaserScan& scan)
{
    std::ostringstream ranges;
    for (const auto& r : scan.ranges) ranges << r << ' ';
    spdlog::debug(
        "LASER: {} {:.5f} {:.5f} {} {}",
        scan.timestamp,
        scan.scan_start,
        scan.scan_resolution,
        scan.ranges.size(),
        ranges.str()
    );
}

void Log::onOdometry(const types::Odometry& odom)
{
    spdlog::debug("ODOM: {} {:.5f} {:.5f} {:.5f}", odom.timestamp, odom.px, odom.py, odom.yaw);
}

void Log::onImu(const types::Imu& imu)
{
    spdlog::debug(
        "IMU: {} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f}",
        imu.timestamp,
        imu.roll,
        imu.pitch,
        imu.yaw,
        imu.vel_roll,
        imu.vel_pitch,
        imu.vel_yaw
    );
}

void Log::onCmdVel(const types::CmdVel& cmd)
{
    spdlog::debug("CMD_VEL: {} {:.5f} {:.5f} {:.5f}", cmd.timestamp, cmd.vx, cmd.vy, cmd.w);
}

void Log::onPose(const types::Pose2D& pose)
{
    spdlog::debug("POSE: {} {:.5f} {:.5f} {:.5f} {:.3f} {:.3f}", pose.timestamp, pose.x, pose.y, pose.yaw, pose.v, pose.w);
}

void Log::onMap(const types::OccupancyMap& map)
{
    spdlog::debug("MAP: {} {}x{} res={:.3f} robot=({:.2f},{:.2f})", map.timestamp, map.width, map.height, map.resolution, map.robot_x, map.robot_y);
}

void Log::onTrajectory(const types::Trajectory& trajectory)
{
    spdlog::debug(
        "TRAJ: {} goal={} points={} t_end={:.2f}",
        trajectory.timestamp,
        trajectory.goal_id,
        trajectory.points.size(),
        trajectory.points.empty() ? 0.0 : trajectory.points.back().t
    );
}

void Log::onPlannerState(const types::PlannerState& state)
{
    spdlog::info(
        "PLANNER: {} state={} goal={} dist={:.2f} {}",
        state.timestamp,
        static_cast<int>(state.state),
        state.goal_id,
        state.distance_to_goal,
        state.detail
    );
}

void Log::onMissionState(const types::MissionState& state)
{
    spdlog::info(
        "MISSION: {} state={} frontiers={} waypoints={}/{} coverage={:.2f} {}",
        state.timestamp,
        static_cast<int>(state.state),
        state.frontiers_left,
        state.waypoints_done,
        state.waypoints_total,
        state.coverage_ratio,
        state.detail
    );
}

void Log::onRobotState(const types::RobotState& state)
{
    spdlog::debug(
        "STATE: {} batt={:.0f} charging={} docked={} bumper={} wall={:.2f} mode={}",
        state.timestamp,
        state.battery_percent,
        state.charging,
        state.docked,
        state.bumper,
        state.wall_distance,
        state.mode
    );
}

void Log::onSafetyStop(const types::SafetyStop& stop)
{
    if (stop.stop) spdlog::warn("SAFETY: {} STOP {}", stop.timestamp, stop.reason);
}

}  // namespace services
}  // namespace project
