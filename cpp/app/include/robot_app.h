#pragma once

#include <memory>
#include <string>

#include "mw/middleware/manager.hpp"

namespace project
{
namespace app
{

/**
 * Собирает стек робота на менеджере исходного проекта:
 *   Platform → Slam → Planner → Control → Platform, поверх — Safety; Log и ZmqBridge сбоку.
 * Platform есть только при XIAOMI_ROBOT_ENABLE_DRIVERS (нужен libplayerc).
 */
class RobotApp
{
public:
    struct Config
    {
        std::string host{"127.0.0.1"};
        int player_port{6665};
        int zmq_sub_port{9090};
        int zmq_pub_port{9091};

        static Config LoadFromJson(const std::string& config_path);
    };

    explicit RobotApp(const Config& config);
    ~RobotApp();

    void Start();

private:
    std::shared_ptr<mw::middleware::Manager> middleware_;
    Config config_;
};

}  // namespace app
}  // namespace project
