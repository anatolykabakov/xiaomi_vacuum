#pragma once

#include <string>

#include "adas/middleware/manager.hpp"

namespace project
{
namespace app
{

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
    std::shared_ptr<adas::middleware::Manager> middleware_;
    Config config_;
};

}  // namespace app
}  // namespace project
