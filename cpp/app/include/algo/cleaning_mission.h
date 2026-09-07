#pragma once

#include <string>
#include <utility>
#include <vector>

#include "algo/coverage_planner.h"
#include "algo/frontier_explorer.h"
#include "types.h"

namespace project
{
namespace algo
{

struct CleaningMissionConfig
{
    FrontierExplorerConfig explorer;
    CoveragePlannerConfig coverage;
    double waypoint_tolerance_m{0.12};  // путевая точка змейки пройдена (чуть больше допуска планировщика)
    double retarget_radius_m{0.35};     // недостижимая точка змейки сдвигается к ближайшей достижимой
    double dock_tolerance_m{0.25};      // у дока
    int max_blocked_retries{2};         // сколько раз терпим blocked на одной цели
    double robot_radius_m{0.1725};      // для метрики покрытия
    double undock_distance_m{0.30};     // первый манёвр: прямо вперёд от дока (док стоит у стены); 0 — без него
    double frontier_done_radius_m{0.30};  // вокруг цели-фронтира не осталось неизвестного — цель выполнена
};

/**
 * Цикл уборки как чистая машина состояний, без middleware — одинаково работает в сервисе
 * Mission на роботе и в симуляторе через биндинги.
 *
 *   Idle → Exploring: сначала съезд с дока прямо вперёд, затем пока есть достижимые фронтиры —
 *                     цель на ближайший; фронтир, вокруг которого всё стало известно, считается пройденным.
 *        → Covering:  фронтиров нет; змейка по свободной области, точки по очереди.
 *        → Returning: змейка пройдена; цель — док (поза старта).
 *        → Done.
 *
 * Вход на каждом шаге: карта, поза, состояние планировщика по текущей цели. Выход — новая цель,
 * если она нужна. Пройденные позы копятся для метрики покрытия.
 */
class CleaningMission
{
public:
    using State = types::MissionState::State;

    explicit CleaningMission(CleaningMissionConfig config = {});

    void Start(const types::Pose2D& dock);
    void Stop();
    /** Досрочно: бросить исследование/покрытие и ехать на док. */
    void ReturnToDock();

    /**
     * Периодический шаг (~1 с). planner_state — последнее состояние планировщика; done/blocked
     * учитываются только если planner_state.goal_id совпадает с текущей целью миссии.
     * \return true, если out_goal нужно отдать планировщику.
     */
    bool Step(
        const types::OccupancyMap& map,
        const types::Pose2D& pose,
        const types::PlannerState& planner_state,
        types::Goal& out_goal
    );

    State state() const { return state_; }
    types::MissionState Progress(const types::OccupancyMap& map) const;
    const std::vector<std::pair<double, double>>& visited() const { return visited_; }
    const std::vector<std::pair<double, double>>& waypoints() const { return waypoints_; }
    const std::vector<std::pair<double, double>>& blacklist() const { return blacklist_; }
    const types::Pose2D& dock() const { return dock_; }

private:
    types::Goal MakeGoal(double x, double y);
    void Transition(State next, const std::string& why);
    static bool HasUnknownNear(const types::OccupancyMap& map, double x, double y, double radius_m);
    /** Ближайшая к (x, y) свободная ячейка вне раздувания в радиусе retarget_radius_m. */
    bool NearestCoverable(const types::OccupancyMap& map, double x, double y, double& out_x, double& out_y) const;

    CleaningMissionConfig config_;
    State state_{State::kIdle};
    types::Pose2D dock_;
    types::Goal current_goal_;
    bool goal_active_{false};
    int blocked_retries_{0};
    uint32_t next_goal_id_{1};
    bool undocked_{false};
    uint32_t undock_goal_id_{0};
    bool retargeted_{false};  // текущую точку змейки уже сдвигали
    std::string detail_;

    std::vector<std::pair<double, double>> blacklist_;
    std::vector<std::pair<double, double>> waypoints_;
    size_t waypoint_index_{0};
    size_t frontiers_left_{0};
    std::vector<std::pair<double, double>> visited_;
};

}  // namespace algo
}  // namespace project
