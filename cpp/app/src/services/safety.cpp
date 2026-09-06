#include "services/safety.h"

#include <string>

#include "mw/utils/logger.h"
#include "utils/topics.h"

namespace project
{
namespace services
{

void Safety::configure()
{
    registerParameter<bool>("safety_cliff_enabled", cliff_enabled_);
    registerParameter<double>("safety_cliff_max_m", cliff_max_m_);

    subscribe<types::Bumper>(topics::kSensorsBumper, [this](const types::Bumper& b) { onBumper(b); });
    subscribe<types::IrData>(topics::kSensorsIr, [this](const types::IrData& ir) { onIr(ir); });
    subscribe<types::RobotState>(topics::kRobotState, [this](const types::RobotState& s) { onRobotState(s); });

    scheduleTimer(50, [this] { tick(); }, "safety");
}

void Safety::onBumper(const types::Bumper& bumper)
{
    bumper_ = bumper;
}

void Safety::onIr(const types::IrData& ir)
{
    ir_ = ir;
}

void Safety::onRobotState(const types::RobotState& /*state*/) {}

void Safety::tick()
{
    bool stop = false;
    std::string reason;
    if (bumper_.any())
    {
        stop = true;
        reason = bumper_.left && bumper_.right ? "bumper" : (bumper_.left ? "bumper_left" : "bumper_right");
    }
    if (cliff_enabled_)
    {
        // датчики пола: дальность больше порога — пола под датчиком нет
        const bool cliff = ir_.cliff0 > cliff_max_m_ || ir_.cliff1 > cliff_max_m_ ||
                           ir_.cliff2 > cliff_max_m_ || ir_.cliff3 > cliff_max_m_;
        if (cliff)
        {
            stop = true;
            reason = reason.empty() ? "cliff" : reason + "+cliff";
        }
    }

    ++ticks_since_publish_;
    if (stop != last_stop_ || ticks_since_publish_ >= 10)  // смена состояния или heartbeat 500 мс
    {
        if (stop != last_stop_)
        {
            if (stop) LOGW("safety: STOP (%s)", reason.c_str());
            else LOGI("safety: clear");
        }
        types::SafetyStop msg;
        msg.timestamp = now();
        msg.stop = stop;
        msg.reason = reason;
        publish<types::SafetyStop>(topics::kSafetyStop, msg);
        last_stop_ = stop;
        ticks_since_publish_ = 0;
    }
}

}  // namespace services
}  // namespace project
