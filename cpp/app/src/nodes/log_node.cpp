#include "nodes/log_node.h"

#include "types.h"

#include <spdlog/spdlog.h>

#include <sstream>

namespace project
{
namespace nodes
{
void LogNode::configure()
{
    subscribe<types::LaserData>(
        "laser_data",
        [this](const types::LaserData& laser_data) { OnLaserData(laser_data); }
    );
    subscribe<types::OdometryData>(
        "odometry_data",
        [this](const types::OdometryData& odometry_data) { OnOdometryData(odometry_data); }
    );
    subscribe<types::GyroData>(
        "gyro_data",
        [this](const types::GyroData& gyro_data) { OnGyroData(gyro_data); }
    );
    subscribe<types::CmdVel>(
        "cmd_vel",
        [this](const types::CmdVel& cmd_vel) { OnCmdVel(cmd_vel); }
    );
}

void LogNode::OnLaserData(const types::LaserData& scan)
{
    std::ostringstream scan_ranges;
    for (const auto& range : scan.ranges)
    {
        scan_ranges << range << ' ';
    }
    spdlog::debug(
        "LASER: {} {:.5f} {:.5f} {} {}",
        scan.timestamp,
        scan.scan_start,
        scan.scan_resolution,
        scan.ranges.size(),
        scan_ranges.str()
    );
}

void LogNode::OnOdometryData(const types::OdometryData& odom)
{
    spdlog::debug("ODOM: {} {:.5f} {:.5f} {:.5f}", odom.timestamp, odom.px, odom.py, odom.yaw);
}

void LogNode::OnGyroData(const types::GyroData& gyro)
{
    spdlog::debug(
        "GYRO: {} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f}",
        gyro.timestamp,
        gyro.roll,
        gyro.pitch,
        gyro.yaw,
        gyro.vel_roll,
        gyro.vel_pitch,
        gyro.vel_yaw
    );
}

void LogNode::OnCmdVel(const types::CmdVel& cmd_vel)
{
    spdlog::debug(
        "CMD_VEL: {} {:.5f} {:.5f} {:.5f}",
        cmd_vel.timestamp,
        cmd_vel.vx,
        cmd_vel.vy,
        cmd_vel.w
    );
}

}  // namespace nodes
}  // namespace project
