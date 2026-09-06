#pragma once

#include <string_view>

#include "mw/middleware/manager.hpp"
#include "algo/path_planner.h"
#include "types.h"

namespace project
{
namespace services
{

/**
 * Траектория до цели по карте.
 *
 * Вход: slam/map, slam/pose, robot/state, planner/goal. Каждые 500 мс, пока есть цель,
 * проверяет текущую траекторию по актуальной карте и, если она ещё годится, публикует её же
 * (Control следит за возрастом); иначе перепланирует (A* + сглаживание + профиль скорости).
 * Состояние машины — в planner/state.
 */
class Planner : public mw::middleware::Service
{
public:
    Planner();
    ~Planner() override = default;

    std::string_view getName() const override { return "planner"; }

protected:
    void configure() override;

private:
    void onMap(const types::OccupancyMap& map);
    void onPose(const types::Pose2D& pose);
    void onRobotState(const types::RobotState& state);
    void onGoal(const types::Goal& goal);
    void onSafetyStop(const types::SafetyStop& stop);
    void planTick();
    void publishState(types::PlannerState::State state, const std::string& detail);
    void registerParameters();

    algo::PathPlannerConfig config_;
    types::OccupancyMap map_;
    types::Pose2D pose_;
    types::RobotState robot_state_;
    types::Goal goal_;
    types::Trajectory trajectory_;
    bool have_trajectory_{false};
    bool have_map_{false};
    bool have_pose_{false};
    bool have_goal_{false};
    types::PlannerState::State last_state_{types::PlannerState::State::kIdle};
};

}  // namespace services
}  // namespace project
