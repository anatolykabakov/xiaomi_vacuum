#pragma once

#include <memory>
#include <string>
#include <vector>

#include "player_interface.h"
#include "types.h"
#include <libplayerc/playerc.h>

namespace project
{
namespace drivers
{

class PlayerClientC : public IPlayerClient
{
public:
    PlayerClientC(const std::string& host, int port);
    ~PlayerClientC();

    PlayerClientC(const PlayerClientC&) = delete;
    PlayerClientC& operator=(const PlayerClientC&) = delete;

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

    playerc_client_t* client_{nullptr};
    playerc_ir_t* ir_wall_{nullptr};
    playerc_ir_t* ir_cliff_{nullptr};
    playerc_position2d_t* base_{nullptr};
    playerc_laser_t* laser_{nullptr};
    playerc_sonar_t* sonar_{nullptr};
    playerc_power_t* power_{nullptr};
    playerc_position3d_t* gyro_{nullptr};
    int laser_index_{-1};
    bool connected_{false};
};

}  // namespace drivers
}  // namespace project
