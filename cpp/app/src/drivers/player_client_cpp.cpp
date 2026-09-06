#include "drivers/player_client_cpp.h"

#include <libplayerc++/playerc++.h>
#include <libplayerc/playerc.h>

#include <sys/types.h>
#include <unistd.h>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "types.h"

namespace project
{
namespace drivers
{
namespace
{

constexpr int K_INIT_ATTEMPTS = 5;
constexpr int K_RETRY_DELAY_MS = 150;
enum : unsigned
{
    K_MICROSECONDS_PER_MILLISECOND = 1000
};

}  // namespace

PlayerClientCPP::PlayerClientCPP(const std::string& host, int port)
{
    std::string last_error = "unknown error";
    for (int attempt = 0; attempt < K_INIT_ATTEMPTS; ++attempt)
    {
        Cleanup();
        std::fprintf(stderr, "[playerc++] init attempt %d/%d...\n", attempt + 1, K_INIT_ATTEMPTS);
        // NOLINTBEGIN(misc-include-cleaner)
        try
        {
            client_ = std::unique_ptr<PlayerCc::PlayerClient>(
                new PlayerCc::PlayerClient(host, static_cast<uint32_t>(port), PLAYERC_DATAMODE_PULL)
            );
            ir_wall_ = std::unique_ptr<PlayerCc::IrProxy>(new PlayerCc::IrProxy(client_.get(), 0));
            ir_cliff_ = std::unique_ptr<PlayerCc::IrProxy>(new PlayerCc::IrProxy(client_.get(), 1));
            base_ = std::unique_ptr<PlayerCc::Position2dProxy>(
                new PlayerCc::Position2dProxy(client_.get(), 0)
            );

            bool laser_ok = false;
            for (int laser_idx : {0, 1, 2})
            {
                try
                {
                    laser_ = std::unique_ptr<PlayerCc::LaserProxy>(
                        new PlayerCc::LaserProxy(client_.get(), laser_idx)
                    );
                    std::fprintf(stderr, "[playerc++] laser proxy index=%d selected\n", laser_idx);
                    laser_ok = true;
                    break;
                }
                catch (const PlayerCc::PlayerError& ex)
                {
                    last_error = ex.GetErrorStr();
                    std::fprintf(
                        stderr,
                        "[playerc++] laser proxy index=%d failed: %s\n",
                        laser_idx,
                        ex.GetErrorStr().c_str()
                    );
                }
            }
            if (!laser_ok)
            {
                throw std::runtime_error(
                    std::string("laser proxy init failed for indexes 0..2: ") + last_error
                );
            }

            sonar_ =
                std::unique_ptr<PlayerCc::SonarProxy>(new PlayerCc::SonarProxy(client_.get(), 0));
            power_ =
                std::unique_ptr<PlayerCc::PowerProxy>(new PlayerCc::PowerProxy(client_.get(), 0));
            try
            {
                gyro_ = std::unique_ptr<PlayerCc::Position3dProxy>(
                    new PlayerCc::Position3dProxy(client_.get(), 0)
                );
            }
            catch (const PlayerCc::PlayerError& ex)
            {
                std::fprintf(
                    stderr,
                    "[playerc++] gyro (position3d:0) init failed: %s\n",
                    ex.GetErrorStr().c_str()
                );
            }
            connected_ = true;
            std::fprintf(
                stderr,
                "[playerc++] init attempt %d/%d OK\n",
                attempt + 1,
                K_INIT_ATTEMPTS
            );
            return;
        }
        // NOLINTEND(misc-include-cleaner)
        // NOLINTNEXTLINE(misc-include-cleaner)
        catch (const PlayerCc::PlayerError& ex)
        {
            last_error = ex.GetErrorStr();
            std::fprintf(
                stderr,
                "[playerc++] init attempt %d/%d failed: %s\n",
                attempt + 1,
                K_INIT_ATTEMPTS,
                ex.GetErrorStr().c_str()
            );
        }
        catch (const std::exception& ex)
        {
            last_error = ex.what();
            std::fprintf(
                stderr,
                "[playerc++] init attempt %d/%d failed: %s\n",
                attempt + 1,
                K_INIT_ATTEMPTS,
                ex.what()
            );
        }
        ::usleep(static_cast<useconds_t>(K_RETRY_DELAY_MS) * K_MICROSECONDS_PER_MILLISECOND);
    }

    throw std::runtime_error(std::string("playerc++ init failed: ") + last_error);
}

PlayerClientCPP::~PlayerClientCPP()
{
    Cleanup();
}

bool PlayerClientCPP::UpdateRobotState()
{
    try
    {
        if (!client_)
        {
            return false;
        }
        client_->Read();
        return true;
    }
    // NOLINTNEXTLINE(misc-include-cleaner)
    catch (const PlayerCc::PlayerError&)
    {
        return false;
    }
}

types::LaserData PlayerClientCPP::GetLaserData() const
{
    types::LaserData data;
    if (!laser_ || laser_->GetCount() == 0)
    {
        return data;
    }
    const uint32_t count = laser_->GetCount();
    data.scan_start = laser_->GetMinAngle();
    data.scan_resolution = laser_->GetScanRes();
    data.ranges.resize(static_cast<std::size_t>(count));
    for (uint32_t idx = 0; idx < count; ++idx)
    {
        data.ranges[static_cast<std::size_t>(idx)] = static_cast<float>(laser_->GetRange(idx));
    }
    return data;
}

types::IrData PlayerClientCPP::GetIrSensorData() const
{
    types::IrData data;
    if (ir_wall_ && ir_cliff_ && ir_wall_->GetCount() >= 1 && ir_cliff_->GetCount() >= 4)
    {
        data.wall = ir_wall_->GetRange(0);
        data.cliff0 = ir_cliff_->GetRange(0);
        data.cliff1 = ir_cliff_->GetRange(1);
        data.cliff2 = ir_cliff_->GetRange(2);
        data.cliff3 = ir_cliff_->GetRange(3);
    }
    return data;
}

double PlayerClientCPP::GetSonarData() const
{
    if (!sonar_ || sonar_->GetCount() == 0)
    {
        return 0.0;
    }
    return sonar_->GetScan(0);
}

types::BatteryState PlayerClientCPP::GetBatteryData() const
{
    types::BatteryState s;
    if (power_)
    {
        s.percentage = power_->GetPercent();
        s.charging = (power_->GetCharge() > 0.0);
    }
    return s;
}

types::OdometryData PlayerClientCPP::GetOdometryData() const
{
    types::OdometryData odom;
    if (base_)
    {
        odom.px = base_->GetXPos();
        odom.py = base_->GetYPos();
        odom.vx = base_->GetXSpeed();
        odom.vy = base_->GetYSpeed();
        odom.yaw = base_->GetYaw();
        odom.omega = base_->GetYawSpeed();
    }
    return odom;
}

types::GyroData PlayerClientCPP::GetGyroData() const
{
    types::GyroData gyro;
    if (!gyro_)
    {
        return gyro;
    }
    gyro.roll = gyro_->GetRoll();
    gyro.pitch = gyro_->GetPitch();
    gyro.yaw = gyro_->GetYaw();
    gyro.vel_roll = gyro_->GetRollSpeed();
    gyro.vel_pitch = gyro_->GetPitchSpeed();
    gyro.vel_yaw = gyro_->GetYawSpeed();
    return gyro;
}

bool PlayerClientCPP::SetVelocityCommand(double px, double py, double az)
{
    try
    {
        if (!base_)
        {
            return false;
        }
        base_->SetSpeed(px, py, az);
        return true;
    }
    // NOLINTNEXTLINE(misc-include-cleaner)
    catch (const PlayerCc::PlayerError&)
    {
        return false;
    }
}

void PlayerClientCPP::Cleanup()
{
    if (!connected_)
    {
        return;
    }
    gyro_.reset();
    power_.reset();
    sonar_.reset();
    laser_.reset();
    base_.reset();
    ir_cliff_.reset();
    ir_wall_.reset();
    client_.reset();
    connected_ = false;
}

}  // namespace drivers
}  // namespace project
