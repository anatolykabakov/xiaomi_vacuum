#pragma once

#include <string_view>

#include "mw/middleware/manager.hpp"
#include "types.h"

namespace project
{
namespace services
{

/** Пишет всю шину в /opt/xiaomi_robot.log (spdlog, debug). Теги LASER/ODOM/CMD_VEL
 *  сохранены — на них завязаны скрипты (start.sh status, drive_zmq). */
class Log : public mw::middleware::Service
{
public:
    Log() = default;
    ~Log() override = default;

    std::string_view getName() const override { return "log"; }

protected:
    void configure() override;

private:
    void onLaser(const types::LaserScan& scan);
    void onOdometry(const types::Odometry& odom);
    void onImu(const types::Imu& imu);
    void onCmdVel(const types::CmdVel& cmd);
    void onPose(const types::Pose2D& pose);
    void onMap(const types::OccupancyMap& map);
    void onTrajectory(const types::Trajectory& trajectory);
    void onPlannerState(const types::PlannerState& state);
    void onMissionState(const types::MissionState& state);
    void onRobotState(const types::RobotState& state);
    void onSafetyStop(const types::SafetyStop& stop);
};

}  // namespace services
}  // namespace project
