#pragma once

#include "adas/middleware/manager.hpp"
#include "types.h"

namespace project
{
namespace nodes
{

class LogNode : public adas::middleware::Service
{
public:
    LogNode() = default;
    ~LogNode() = default;

protected:
    std::string_view getName() const override { return "log"; }
    void configure() override;

private:
    void OnLaserData(const types::LaserData& scan);
    void OnOdometryData(const types::OdometryData& odom);
    void OnGyroData(const types::GyroData& gyro);
    void OnCmdVel(const types::CmdVel& cmd_vel);
};

}  // namespace nodes
}  // namespace project
