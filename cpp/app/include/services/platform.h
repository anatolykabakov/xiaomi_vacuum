#pragma once

#include <memory>
#include <string_view>

#include "mw/middleware/manager.hpp"
#include "drivers/player_interface.h"
#include "robot_app.h"
#include "types.h"

namespace project
{
namespace services
{

/**
 * HAL к роботу через штатный Player (:6665).
 *
 *  rx    100 мс: читает player и публикует sensors/* (лидар, одометрия, IMU, бампер, ИК, сонар)
 *  tx    100 мс: отправляет последний controls/cmd_vel; если команды нет дольше
 *                cmd_timeout_ms — шлёт ноль. Это защита от always_command=1 (keep speed)
 *                на MCU: без неё зависший клиент = уехавший робот.
 *  state 1000 мс: публикует robot/state (батарея, док, бампер, обрыв)
 */
class Platform : public mw::middleware::Service
{
public:
    explicit Platform(const app::RobotApp::Config& config);
    ~Platform() override = default;

    std::string_view getName() const override { return "platform"; }

protected:
    void configure() override;

private:
    void rxCallback();
    void txCallback();
    void stateCallback();
    void onCmdVel(const types::CmdVel& cmd);
    void onActuators(const types::ActuatorCommand& cmd);

    app::RobotApp::Config config_;
    std::unique_ptr<drivers::IPlayerClient> client_;

    types::CmdVel last_cmd_;
    bool have_cmd_{false};
    bool zero_sent_{true};
    int cmd_timeout_ms_{500};

    types::IrData last_ir_;
    types::Bumper last_bumper_;
    types::BatteryState last_battery_;
};

}  // namespace services
}  // namespace project
