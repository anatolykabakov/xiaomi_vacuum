#pragma once

#include <string_view>

#include "mw/middleware/manager.hpp"
#include "types.h"

namespace project
{
namespace services
{

/**
 * Аварийный стоп поверх контроллера — 
 * но с прямым действием: safety/stop, который Control обязан выполнить.
 *
 * Триггеры: бампер (всегда), обрыв по ИК-датчикам пола (по параметру safety_cliff_enabled,
 * порог safety_cliff_max_m — значение дальности выше порога = пола нет).
 * Публикует при смене состояния и как heartbeat каждые 500 мс.
 */
class Safety : public mw::middleware::Service
{
public:
    Safety() = default;
    ~Safety() override = default;

    std::string_view getName() const override { return "safety"; }

protected:
    void configure() override;

private:
    void onBumper(const types::Bumper& bumper);
    void onIr(const types::IrData& ir);
    void onRobotState(const types::RobotState& state);
    void tick();

    types::Bumper bumper_;
    types::IrData ir_;
    bool cliff_enabled_{false};
    double cliff_max_m_{0.08};
    bool last_stop_{false};
    int ticks_since_publish_{0};
};

}  // namespace services
}  // namespace project
