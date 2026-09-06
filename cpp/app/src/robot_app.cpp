#include "robot_app.h"

#include "adas/middleware/manager.hpp"
#include "nodes/log_node.h"
#include "nodes/mapping_node.h"
#ifdef XIAOMI_ROBOT_ENABLE_DRIVERS
#include "nodes/player_node.h"
#endif
#include "nodes/zmq_bridge.h"
#include "utils/utils.h"

#include <memory>
#include <string>

namespace project
{
namespace app
{

RobotApp::RobotApp(const Config& config) : config_(config)
{
    middleware_ =
        std::make_shared<adas::middleware::Manager>(adas::middleware::Manager::Mode::RealTime);
#ifdef XIAOMI_ROBOT_ENABLE_DRIVERS
    middleware_->registerService<nodes::PlayerNode>(config);
#endif
    middleware_->registerService<nodes::MappingNode>();
    middleware_->registerService<nodes::LogNode>();
    middleware_->registerService<nodes::ZmqBridge>(config);
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
