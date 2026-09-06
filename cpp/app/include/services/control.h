#pragma once

#include <string_view>

#include "mw/middleware/manager.hpp"
#include "algo/pure_pursuit.h"
#include "types.h"

namespace project
{
namespace services
{

/**
 * Ведение по траектории → controls/cmd_vel.
 *
 * Тик 50 мс. Публикует команду КАЖДЫЙ тик (в т.ч. ноль): Platform кормит MCU последней
 * командой и по таймауту сам обнуляет, поэтому поток команд должен быть непрерывным.
 * Ноль при: safety/stop, нет траектории, траектория старше traj_max_age_ms, цель достигнута.
 * Бампер (safety/stop с причиной bumper): откат назад на recover_distance_m, затем ждём новую
 * траекторию — Planner к этому моменту знает о препятствии от Slam.
 */
class Control : public mw::middleware::Service
{
public:
    Control();
    ~Control() override = default;

    std::string_view getName() const override { return "control"; }

protected:
    void configure() override;

private:
    void onTrajectory(const types::Trajectory& trajectory);
    void onPose(const types::Pose2D& pose);
    void onOdometry(const types::Odometry& odom);
    void onRobotState(const types::RobotState& state);
    void onSafetyStop(const types::SafetyStop& stop);
    void tick();
    void publishZero();
    void registerParameters();

    algo::PurePursuitConfig config_;
    int traj_max_age_ms_{2000};
    double recover_distance_m_{0.10};
    double recover_speed_mps_{0.08};
    int recover_ticks_left_{0};

    types::Trajectory trajectory_;
    types::Pose2D pose_;
    types::Odometry odom_;
    types::RobotState robot_state_;
    types::SafetyStop safety_;
    bool have_trajectory_{false};
    bool have_pose_{false};
    bool done_{false};
};

}  // namespace services
}  // namespace project
