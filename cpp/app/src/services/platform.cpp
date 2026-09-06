#include "services/platform.h"

#include <cstdint>
#include <cstdio>

#include "mw/utils/logger.h"
#include "utils/topics.h"

namespace project
{
namespace services
{

Platform::Platform(const app::RobotApp::Config& config) : config_(config) {}

void Platform::configure()
{
    client_ = drivers::CreatePlayerClientC(config_.host, config_.player_port);

    registerParameter<int>("platform_cmd_timeout_ms", cmd_timeout_ms_);

    subscribe<types::CmdVel>(topics::kCmdVel, [this](const types::CmdVel& cmd) { onCmdVel(cmd); });
    subscribe<types::ActuatorCommand>(
        topics::kActuators, [this](const types::ActuatorCommand& cmd) { onActuators(cmd); }
    );

    scheduleTimer(100, [this] { rxCallback(); }, "rx");
    scheduleTimer(100, [this] { txCallback(); }, "tx");
    scheduleTimer(1000, [this] { stateCallback(); }, "state");
}

void Platform::rxCallback()
{
    if (!client_ || !client_->UpdateRobotState())
    {
        std::fprintf(stderr, "platform: player read failed\n");
        return;
    }
    const uint64_t ts = now();

    types::LaserScan scan = client_->GetLaserData();
    scan.timestamp = ts;
    publish<types::LaserScan>(topics::kSensorsLaser, scan);

    types::Odometry odom = client_->GetOdometryData();
    odom.timestamp = ts;
    publish<types::Odometry>(topics::kSensorsOdometry, odom);

    types::Imu imu = client_->GetGyroData();
    imu.timestamp = ts;
    publish<types::Imu>(topics::kSensorsImu, imu);

    last_ir_ = client_->GetIrSensorData();
    last_ir_.timestamp = ts;
    publish<types::IrData>(topics::kSensorsIr, last_ir_);

    last_bumper_ = client_->GetBumperData();
    last_bumper_.timestamp = ts;
    publish<types::Bumper>(topics::kSensorsBumper, last_bumper_);

    types::SonarData sonar;
    sonar.timestamp = ts;
    sonar.data = client_->GetSonarData();
    publish<types::SonarData>(topics::kSensorsSonar, sonar);

    last_battery_ = client_->GetBatteryData();
    last_battery_.timestamp = ts;
}

void Platform::txCallback()
{
    if (!client_)
    {
        return;
    }
    const uint64_t age_ms = have_cmd_ ? (now() - last_cmd_.timestamp) / 1000 : UINT64_MAX;
    if (have_cmd_ && age_ms <= static_cast<uint64_t>(cmd_timeout_ms_))
    {
        client_->SetVelocityCommand(last_cmd_.vx, last_cmd_.vy, last_cmd_.w);
        zero_sent_ = false;
        return;
    }
    // Команда устарела или её не было: держим MCU на нуле. Отправляем каждый тик —
    // так поток команд к шасси непрерывен, а зависший клиент не оставит скорость.
    client_->SetVelocityCommand(0.0, 0.0, 0.0);
    if (!zero_sent_ && have_cmd_)
    {
        LOGW("platform: cmd_vel старше %d мс — стоп по watchdog", cmd_timeout_ms_);
    }
    zero_sent_ = true;
}

void Platform::stateCallback()
{
    types::RobotState state;
    state.timestamp = now();
    state.battery_percent = last_battery_.percentage;
    state.charging = last_battery_.charging;
    state.docked = last_battery_.charging;  // точнее с Player пока не узнать
    state.bumper = last_bumper_.any();
    state.wall_distance = last_ir_.wall;
    state.mode = have_cmd_ && !zero_sent_ ? "auto" : "idle";
    publish<types::RobotState>(topics::kRobotState, state);
}

void Platform::onCmdVel(const types::CmdVel& cmd)
{
    last_cmd_ = cmd;
    if (last_cmd_.timestamp == 0)
    {
        last_cmd_.timestamp = now();
    }
    have_cmd_ = true;
}

void Platform::onActuators(const types::ActuatorCommand& cmd)
{
    // Интерфейс motor:0/1/2 (вентилятор, щётки) у Rockrobo нестандартный для Player,
    // стандартными прокси libplayerc не адресуется — см. docs/PLATFORM.md.
    LOGI(
        "platform: actuators fan=%.2f main_brush=%.2f side_brush=%.2f (не реализовано)",
        cmd.fan,
        cmd.main_brush,
        cmd.side_brush
    );
}

}  // namespace services
}  // namespace project
