#pragma once

#include <libplayerc++/playerc++.h>
#include <memory>
#include <string>
#include <vector>

#include "player_interface.h"
#include "types.h"

namespace project
{
namespace drivers
{

class PlayerClientCPP : public IPlayerClient
{
public:
    PlayerClientCPP(const std::string& host, int port);
    ~PlayerClientCPP();

    PlayerClientCPP(const PlayerClientCPP&) = delete;
    PlayerClientCPP& operator=(const PlayerClientCPP&) = delete;

    bool UpdateRobotState() override;
    types::LaserData GetLaserData() const override;
    types::IrData GetIrSensorData() const override;
    double GetSonarData() const override;
    types::BatteryState GetBatteryData() const override;
    types::OdometryData GetOdometryData() const override;
    types::GyroData GetGyroData() const override;
    bool SetVelocityCommand(double px, double py, double az) override;

private:
    void Cleanup();

    std::unique_ptr<PlayerCc::PlayerClient> client_;
    std::unique_ptr<PlayerCc::IrProxy> ir_wall_;
    std::unique_ptr<PlayerCc::IrProxy> ir_cliff_;
    std::unique_ptr<PlayerCc::Position2dProxy> base_;
    std::unique_ptr<PlayerCc::LaserProxy> laser_;
    std::unique_ptr<PlayerCc::SonarProxy> sonar_;
    std::unique_ptr<PlayerCc::PowerProxy> power_;
    std::unique_ptr<PlayerCc::Position3dProxy> gyro_;
    bool connected_{false};
};

}  // namespace drivers
}  // namespace project
