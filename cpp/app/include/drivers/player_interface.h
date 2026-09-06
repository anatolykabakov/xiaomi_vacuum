#pragma once

#include <memory>
#include <string>

#include "types.h"

namespace project
{
namespace drivers
{

class IPlayerClient
{
public:
    virtual ~IPlayerClient() {}
    virtual bool UpdateRobotState() = 0;
    virtual types::LaserData GetLaserData() const = 0;
    virtual types::IrData GetIrSensorData() const = 0;
    virtual double GetSonarData() const = 0;
    virtual types::BatteryState GetBatteryData() const = 0;
    virtual types::OdometryData GetOdometryData() const = 0;
    virtual types::GyroData GetGyroData() const = 0;
    virtual bool SetVelocityCommand(double px, double py, double az) = 0;
};

std::unique_ptr<IPlayerClient> CreatePlayerClientC(const std::string& host, int port);

}  // namespace drivers
}  // namespace project
