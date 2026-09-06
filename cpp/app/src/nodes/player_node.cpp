#include "nodes/player_node.h"

#include "drivers/player_interface.h"
#include "robot_app.h"
#include "types.h"

#include <cstdint>
#include <cstdio>

namespace project
{
namespace nodes
{

namespace
{

constexpr uint64_t K_PLAYER_TIMER_INTERVAL_MS = 100;

}  // namespace

PlayerNode::PlayerNode(const app::RobotApp::Config& config) : config_(config) {}

void PlayerNode::configure()
{
    client_ = drivers::CreatePlayerClientC(config_.host, config_.player_port);
    subscribe<types::CmdVel>(
        "cmd_vel",
        [this](const types::CmdVel& cmd_vel) { OnCmdVel(cmd_vel); }
    );
    scheduleTimer(K_PLAYER_TIMER_INTERVAL_MS, [this]() { OnTimer(); });
}

void PlayerNode::OnTimer()
{
    if (!client_ || !client_->UpdateRobotState())
    {
        std::fprintf(stderr, "player client read failed\n");
        return;
    }
    types::LaserData scan = client_->GetLaserData();
    scan.timestamp = now();
    types::OdometryData pose = client_->GetOdometryData();
    pose.timestamp = now();
    types::GyroData gyro = client_->GetGyroData();
    gyro.timestamp = now();
    publish<types::LaserData>("laser_data", scan);
    publish<types::OdometryData>("odometry_data", pose);
    publish<types::GyroData>("gyro_data", gyro);
}

void PlayerNode::OnCmdVel(const types::CmdVel& cmd_vel)
{
    if (!client_ || !client_->SetVelocityCommand(cmd_vel.vx, cmd_vel.vy, cmd_vel.w))
    {
        std::fprintf(stderr, "Failed to send velocity command\n");
    }
}

}  // namespace nodes
}  // namespace project
