#pragma once

#include "drivers/player_interface.h"
#include "adas/middleware/manager.hpp"
#include "robot_app.h"
#include "types.h"

namespace project
{
namespace nodes
{

class PlayerNode : public adas::middleware::Service
{
public:
    explicit PlayerNode(const app::RobotApp::Config& config);
    ~PlayerNode() = default;

protected:
    std::string_view getName() const override { return "player"; }
    void configure() override;

private:
    app::RobotApp::Config config_;
    std::unique_ptr<drivers::IPlayerClient> client_;
    void OnTimer();

    void OnCmdVel(const types::CmdVel& cmd_vel);
};

}  // namespace nodes
}  // namespace project
