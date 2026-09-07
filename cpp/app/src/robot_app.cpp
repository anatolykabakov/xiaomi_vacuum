#include "robot_app.h"

#include <memory>
#include <string>

#include "mw/middleware/manager.hpp"
#include "services/control.h"
#include "services/log.h"
#include "services/mission.h"
#include "services/planner.h"
#ifdef XIAOMI_ROBOT_ENABLE_DRIVERS
#include "services/platform.h"
#endif
#include "services/safety.h"
#include "services/slam.h"
#include "services/zmq_bridge.h"
#include "utils/utils.h"

namespace project
{
namespace app
{

RobotApp::RobotApp(const Config& config) : config_(config)
{
    using mw::middleware::Manager;
    middleware_ = std::make_shared<Manager>(Manager::Mode::RealTime);
#ifdef XIAOMI_ROBOT_ENABLE_DRIVERS
    middleware_->registerService<services::Platform>(config);
#endif
    middleware_->registerService<services::Slam>();
    middleware_->registerService<services::Mission>();
    middleware_->registerService<services::Planner>();
    middleware_->registerService<services::Control>();
    middleware_->registerService<services::Safety>();
    middleware_->registerService<services::Log>();
    middleware_->registerService<services::ZmqBridge>(config);
}

RobotApp::~RobotApp() {}

void RobotApp::Start()
{
    middleware_->startAll();
    utils::WaitForSignal();
    middleware_->stopAll();
}

RobotApp::Config RobotApp::Config::LoadFromJson(const std::string& config_path)
{
    auto root = utils::LoadJsonConfig(config_path);
    Config config;
    config.host = root["host"].asString();
    if (root.isMember("player_port"))
    {
        config.player_port = root["player_port"].asInt();
    }
    else if (root.isMember("playerport"))
    {
        config.player_port = root["playerport"].asInt();
    }
    config.zmq_sub_port = root["zmq_sub_port"].asInt();
    config.zmq_pub_port = root["zmq_pub_port"].asInt();
    return config;
}

}  // namespace app
}  // namespace project
