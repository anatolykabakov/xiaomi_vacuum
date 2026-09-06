#pragma once

#include <string_view>

#include "algo/cleaning_mission.h"
#include "mw/middleware/manager.hpp"
#include "types.h"

namespace project
{
namespace services
{

/**
 * Цикл уборки: выбирает цели планировщику — фронтиры пока помещение не исследовано, затем
 * точки змейки покрытия, затем док. Логика — в algo::CleaningMission, здесь только шина.
 *
 * Вход: slam/map, slam/pose, planner/state, mission/command. Выход: planner/goal, mission/state.
 * Тик 1 с. Параметр mission_autostart запускает уборку сам, как только есть карта и поза.
 */
class Mission : public mw::middleware::Service
{
public:
    Mission();
    ~Mission() override = default;

    std::string_view getName() const override { return "mission"; }

protected:
    void configure() override;

private:
    void onMap(const types::OccupancyMap& map);
    void onPose(const types::Pose2D& pose);
    void onPlannerState(const types::PlannerState& state);
    void onCommand(const types::MissionCommand& command);
    void tick();

    algo::CleaningMissionConfig config_;
    algo::CleaningMission mission_;
    bool autostart_{false};

    types::OccupancyMap map_;
    types::Pose2D pose_;
    types::PlannerState planner_state_;
    bool have_map_{false};
    bool have_pose_{false};
};

}  // namespace services
}  // namespace project
