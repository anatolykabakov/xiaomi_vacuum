#include "drivers/player_client_c.h"
#include "drivers/player_interface.h"

#include <sys/types.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <libplayerinterface/player.h>

extern "C"
{
#include <libplayerc/playerc.h>
}

#include "types.h"

namespace project
{
namespace drivers
{
namespace
{

constexpr int K_READ_ATTEMPTS = 5;
constexpr int K_MS_RETRY_AFTER_INTERRUPT = 50;
constexpr int K_CONNECT_ATTEMPTS = 5;
constexpr int K_MS_BETWEEN_CONNECT = 150;
enum : unsigned
{
    K_MICROSECONDS_PER_MILLISECOND = 1000
};

std::string LastPlayerError()
{
    const char* err = playerc_error_str();
    if (err == nullptr || err[0] == '\0')
    {
        return std::string("unknown playerc error");
    }
    return std::string(err);
}

void ThrowSubscribeError(const char* proxy_name)
{
    throw std::runtime_error(
        std::string("player proxy subscribe failed for ") + proxy_name + ": " + LastPlayerError()
    );
}

bool ReadWithEintrRetry(playerc_client_t* cli)
{
    for (int attempt = 0; attempt < K_READ_ATTEMPTS; ++attempt)
    {
        if (playerc_client_read(cli) != nullptr)
        {
            return true;
        }
        const std::string err = LastPlayerError();
        if (err.find("Interrupted system call") == std::string::npos)
        {
            return false;
        }
        ::usleep(
            static_cast<useconds_t>(K_MS_RETRY_AFTER_INTERRUPT) * K_MICROSECONDS_PER_MILLISECOND
        );
    }
    return false;
}

}  // namespace

PlayerClientC::PlayerClientC(const std::string& host, int port)
{
    auto create_client = [&]()
    {
        client_ = playerc_client_create(nullptr, host.c_str(), port);
        if (client_ == nullptr)
        {
            throw std::runtime_error("playerc_client_create failed");
        }
    };
    create_client();

    bool connected_ok = false;
    for (int attempt = 0; attempt < K_CONNECT_ATTEMPTS; ++attempt)
    {
        if (attempt > 0)
        {
            playerc_client_destroy(client_);
            client_ = nullptr;
            create_client();
        }
        if (playerc_client_connect(client_) == 0)
        {
            connected_ok = true;
            break;
        }
        ::usleep(static_cast<useconds_t>(K_MS_BETWEEN_CONNECT) * K_MICROSECONDS_PER_MILLISECOND);
    }
    if (!connected_ok)
    {
        throw std::runtime_error("playerc_client_connect failed");
    }
    if (playerc_client_datamode(client_, PLAYERC_DATAMODE_PULL) != 0)
    {
        throw std::runtime_error(
            std::string("playerc_client_datamode(PULL) failed: ") + LastPlayerError()
        );
    }
    if (playerc_client_set_replace_rule(client_, -1, -1, PLAYER_MSGTYPE_DATA, -1, 1) != 0)
    {
        throw std::runtime_error(
            std::string("playerc_client_set_replace_rule failed: ") + LastPlayerError()
        );
    }
    ir_wall_ = playerc_ir_create(client_, 0);
    ir_cliff_ = playerc_ir_create(client_, 1);
    base_ = playerc_position2d_create(client_, 0);
    for (int candidate_index : {0, 1, 2})
    {
        laser_ = playerc_laser_create(client_, candidate_index);
        if (laser_ != nullptr)
        {
            laser_index_ = candidate_index;
            break;
        }
    }
    sonar_ = playerc_sonar_create(client_, 0);
    power_ = playerc_power_create(client_, 0);
    gyro_ = playerc_position3d_create(client_, 0);
    if (!ir_wall_ || !ir_cliff_ || !base_ || !laser_ || !sonar_ || !power_)
    {
        throw std::runtime_error(std::string("player proxy create failed: ") + LastPlayerError());
    }

    if (playerc_ir_subscribe(ir_wall_, PLAYER_OPEN_MODE) != 0)
    {
        ThrowSubscribeError("ir_wall");
    }
    if (playerc_ir_subscribe(ir_cliff_, PLAYER_OPEN_MODE) != 0)
    {
        ThrowSubscribeError("ir_cliff");
    }
    if (playerc_position2d_subscribe(base_, PLAYER_OPEN_MODE) != 0)
    {
        ThrowSubscribeError("position2d");
    }
    if (playerc_laser_subscribe(laser_, PLAYER_OPEN_MODE) != 0)
    {
        ThrowSubscribeError("laser");
    }
    if (playerc_sonar_subscribe(sonar_, PLAYER_OPEN_MODE) != 0)
    {
        ThrowSubscribeError("sonar");
    }
    if (playerc_power_subscribe(power_, PLAYER_OPEN_MODE) != 0)
    {
        ThrowSubscribeError("power");
    }
    if (gyro_ && playerc_position3d_subscribe(gyro_, PLAYER_OPEN_MODE) != 0)
    {
        std::fprintf(
            stderr,
            "player gyro (position3d:0) subscribe failed: %s\n",
            LastPlayerError().c_str()
        );
        playerc_position3d_destroy(gyro_);
        gyro_ = nullptr;
    }
    connected_ = true;
}

PlayerClientC::~PlayerClientC()
{
    Cleanup();
    client_ = nullptr;
    ir_wall_ = nullptr;
    ir_cliff_ = nullptr;
    base_ = nullptr;
    laser_ = nullptr;
    sonar_ = nullptr;
    power_ = nullptr;
    gyro_ = nullptr;
    connected_ = false;
}

bool PlayerClientC::UpdateRobotState()
{
    return connected_ && ReadWithEintrRetry(client_);
}

types::LaserData PlayerClientC::GetLaserData() const
{
    types::LaserData data;
    if (!laser_ || laser_->scan_count <= 0)
    {
        return data;
    }
    if (laser_->scan_res - 0.0 < 1e-9)
    {
        data.scan_resolution = 2 * M_PI / laser_->scan_count;
    }
    else
    {
        data.scan_resolution = laser_->scan_res;
    }
    if (laser_->scan_start - 0.0 < 1e-9)
    {
        data.scan_start = -M_PI;
    }
    else
    {
        data.scan_start = laser_->scan_start;
    }
    data.ranges.resize(laser_->scan_count);
    for (int idx = 0; idx < laser_->scan_count; ++idx)
    {
        data.ranges[idx] = static_cast<float>(laser_->ranges[laser_->scan_count - idx - 1]);
    }
    return data;
}

types::IrData PlayerClientC::GetIrSensorData() const
{
    types::IrData data;
    if (!ir_wall_ || !ir_cliff_)
    {
        return data;
    }
    if (ir_wall_->data.ranges_count == 4 && ir_cliff_->data.ranges_count >= 4)
    {
        data.wall = ir_wall_->data.ranges[0];
        data.cliff0 = ir_cliff_->data.ranges[0];
        data.cliff1 = ir_cliff_->data.ranges[1];
        data.cliff2 = ir_cliff_->data.ranges[2];
        data.cliff3 = ir_cliff_->data.ranges[3];
    }
    return data;
}

double PlayerClientC::GetSonarData() const
{
    if (!sonar_ || sonar_->scan_count != 1)
    {
        return 0.0;
    }
    return sonar_->scan[0];
}

types::BatteryState PlayerClientC::GetBatteryData() const
{
    types::BatteryState s;
    if (!power_)
    {
        return s;
    }
    s.percentage = power_->percent;
    s.charging = (power_->charging != 0);
    return s;
}

types::OdometryData PlayerClientC::GetOdometryData() const
{
    types::OdometryData odom;
    if (!base_)
    {
        return odom;
    }
    odom.px = base_->px;
    odom.py = base_->py;
    odom.vx = base_->vx;
    odom.vy = base_->vy;
    odom.yaw = base_->pa;
    odom.omega = base_->va;
    return odom;
}

types::GyroData PlayerClientC::GetGyroData() const
{
    types::GyroData gyro;
    if (!gyro_)
    {
        return gyro;
    }
    gyro.roll = gyro_->pos_roll;
    gyro.pitch = gyro_->pos_pitch;
    gyro.yaw = gyro_->pos_yaw;
    gyro.vel_roll = gyro_->vel_roll;
    gyro.vel_pitch = gyro_->vel_pitch;
    gyro.vel_yaw = gyro_->vel_yaw;
    return gyro;
}

bool PlayerClientC::SetVelocityCommand(double px, double py, double az)
{
    return base_ && playerc_position2d_set_cmd_vel(base_, px, py, az, 1) == 0;
}

void PlayerClientC::Cleanup()
{
    if (ir_cliff_)
    {
        playerc_ir_unsubscribe(ir_cliff_);
        playerc_ir_destroy(ir_cliff_);
        ir_cliff_ = nullptr;
    }
    if (ir_wall_)
    {
        playerc_ir_unsubscribe(ir_wall_);
        playerc_ir_destroy(ir_wall_);
        ir_wall_ = nullptr;
    }
    if (base_)
    {
        playerc_position2d_unsubscribe(base_);
        playerc_position2d_destroy(base_);
        base_ = nullptr;
    }
    if (laser_)
    {
        playerc_laser_unsubscribe(laser_);
        playerc_laser_destroy(laser_);
        laser_ = nullptr;
    }
    if (sonar_)
    {
        playerc_sonar_unsubscribe(sonar_);
        playerc_sonar_destroy(sonar_);
        sonar_ = nullptr;
    }
    if (power_)
    {
        playerc_power_unsubscribe(power_);
        playerc_power_destroy(power_);
        power_ = nullptr;
    }
    if (gyro_)
    {
        playerc_position3d_unsubscribe(gyro_);
        playerc_position3d_destroy(gyro_);
        gyro_ = nullptr;
    }
    if (client_)
    {
        playerc_client_disconnect(client_);
        playerc_client_destroy(client_);
        client_ = nullptr;
    }
    connected_ = false;
}

std::unique_ptr<IPlayerClient> CreatePlayerClientC(const std::string& host, int port)
{
    return std::unique_ptr<IPlayerClient>(new PlayerClientC(host, port));
}

}  // namespace drivers
}  // namespace project
