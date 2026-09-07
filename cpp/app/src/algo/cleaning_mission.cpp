#include "algo/cleaning_mission.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "algo/path_planner.h"

namespace project
{
namespace algo
{

namespace
{
using PS = types::PlannerState::State;

bool InProgress(PS s)
{
    return s == PS::kIdle || s == PS::kNoMap || s == PS::kPlanning || s == PS::kFollowing;
}
}  // namespace

CleaningMission::CleaningMission(CleaningMissionConfig config) : config_(config) {}

void CleaningMission::Start(const types::Pose2D& dock)
{
    dock_ = dock;
    goal_active_ = false;
    blocked_retries_ = 0;
    blacklist_.clear();
    waypoints_.clear();
    waypoint_index_ = 0;
    visited_.clear();
    undocked_ = false;
    undock_goal_id_ = 0;
    Transition(State::kExploring, "старт уборки");
}

void CleaningMission::Stop()
{
    goal_active_ = false;
    Transition(State::kIdle, "стоп по команде");
}

void CleaningMission::ReturnToDock()
{
    if (state_ == State::kIdle || state_ == State::kDone) return;
    goal_active_ = false;
    blocked_retries_ = 0;
    Transition(State::kReturning, "возврат по команде");
}

types::Goal CleaningMission::MakeGoal(double x, double y)
{
    types::Goal g;
    g.type = types::Goal::Type::kPoint;
    g.x = x;
    g.y = y;
    g.id = next_goal_id_++;
    current_goal_ = g;
    goal_active_ = true;
    blocked_retries_ = 0;
    return g;
}

void CleaningMission::Transition(State next, const std::string& why)
{
    state_ = next;
    detail_ = why;
}

bool CleaningMission::NearestCoverable(
    const types::OccupancyMap& map, double x, double y, double& out_x, double& out_y
) const
{
    constexpr uint8_t kFree = 255;
    constexpr uint8_t kCellFree = 1;
    int gx, gy;
    if (!PathPlanner::WorldToGrid(map, x, y, gx, gy)) return false;
    PathPlannerConfig pc;
    pc.inflation_radius_m = config_.coverage.inflation_radius_m;
    const std::vector<uint8_t> grid = PathPlanner(pc).Inflate(map);
    const int w = static_cast<int>(map.width), h = static_cast<int>(map.height);
    const int r = std::max(1, static_cast<int>(std::ceil(config_.retarget_radius_m / map.resolution)));
    double best = std::numeric_limits<double>::infinity();
    for (int dy = -r; dy <= r; ++dy)
        for (int dx = -r; dx <= r; ++dx)
        {
            const int nx = gx + dx, ny = gy + dy;
            if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
            const size_t i = static_cast<size_t>(nx) + static_cast<size_t>(ny) * w;
            if (map.cells[i] != kFree || grid[i] != kCellFree) continue;
            const double d = std::hypot(dx, dy);
            if (d < best)
            {
                best = d;
                PathPlanner::GridToWorld(map, nx, ny, out_x, out_y);
            }
        }
    return std::isfinite(best) && best * map.resolution <= config_.retarget_radius_m;
}

bool CleaningMission::HasUnknownNear(const types::OccupancyMap& map, double x, double y, double radius_m)
{
    constexpr uint8_t kUnknown = 127;
    const int w = static_cast<int>(map.width), h = static_cast<int>(map.height);
    if (map.cells.size() < static_cast<size_t>(w) * h || map.resolution <= 0) return false;
    const int gx = static_cast<int>(x / map.resolution + map.origin_x);
    const int gy = static_cast<int>(y / map.resolution + (static_cast<float>(map.height) - map.origin_y));
    const int r = std::max(1, static_cast<int>(std::ceil(radius_m / map.resolution)));
    for (int dy = -r; dy <= r; ++dy)
        for (int dx = -r; dx <= r; ++dx)
        {
            if (dx * dx + dy * dy > r * r) continue;
            const int nx = gx + dx, ny = gy + dy;
            if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
            if (map.cells[static_cast<size_t>(nx) + static_cast<size_t>(ny) * w] == kUnknown) return true;
        }
    return false;
}

bool CleaningMission::Step(
    const types::OccupancyMap& map,
    const types::Pose2D& pose,
    const types::PlannerState& planner_state,
    types::Goal& out_goal
)
{
    if (state_ == State::kIdle || state_ == State::kDone) return false;
    visited_.emplace_back(pose.x, pose.y);

    // Разбор статуса планировщика по нашей цели. Чужие/старые цели игнорируем.
    bool goal_done = false, goal_blocked = false;
    if (goal_active_ && planner_state.goal_id == current_goal_.id)
    {
        if (planner_state.state == PS::kDone) goal_done = true;
        else if (planner_state.state == PS::kBlocked) goal_blocked = true;
    }
    // Цель считаем пройденной и по геометрии — планировщик мог не успеть сообщить
    if (goal_active_ && std::hypot(current_goal_.x - pose.x, current_goal_.y - pose.y) <= config_.waypoint_tolerance_m)
    {
        goal_done = true;
    }
    if (goal_active_ && state_ == State::kExploring)
    {
        const bool undock_goal = current_goal_.id == undock_goal_id_;
        // съезд с дока — одна попытка: не вышло — всё равно едем дальше, без чёрного списка
        if (undock_goal && goal_blocked)
        {
            goal_active_ = false;
            goal_blocked = false;
        }
        // фронтир мог исчезнуть, пока ехали: вокруг цели всё уже известно — цель выполнена
        if (!undock_goal && goal_active_ && !goal_done &&
            !HasUnknownNear(map, current_goal_.x, current_goal_.y, config_.frontier_done_radius_m))
        {
            goal_done = true;
            goal_blocked = false;
        }
    }
    if (goal_blocked)
    {
        if (++blocked_retries_ <= config_.max_blocked_retries) return false;  // планировщик ещё попробует
        // точка змейки уехала в раздувание (карта уточнилась) — сдвигаем к ближайшей достижимой
        double nx, ny;
        if (state_ == State::kCovering && !retargeted_ && NearestCoverable(map, current_goal_.x, current_goal_.y, nx, ny))
        {
            retargeted_ = true;
            out_goal = MakeGoal(nx, ny);
            detail_ = "точка змейки сдвинута";
            return true;
        }
        blacklist_.emplace_back(current_goal_.x, current_goal_.y);
        goal_active_ = false;
        if (state_ == State::kCovering) ++waypoint_index_;  // точку змейки пропускаем
    }
    if (goal_done)
    {
        goal_active_ = false;
        if (state_ == State::kCovering) ++waypoint_index_;
    }
    if (goal_active_ && InProgress(planner_state.state)) return false;
    if (goal_active_) return false;

    switch (state_)
    {
        case State::kExploring:
        {
            if (config_.undock_distance_m > 0 && !undocked_)
            {
                undocked_ = true;
                out_goal = MakeGoal(
                    dock_.x + config_.undock_distance_m * std::cos(dock_.yaw),
                    dock_.y + config_.undock_distance_m * std::sin(dock_.yaw)
                );
                undock_goal_id_ = out_goal.id;
                detail_ = "съезд с дока";
                return true;
            }
            FrontierExplorer explorer(config_.explorer);
            frontiers_left_ = explorer.Find(map, pose).size();
            types::Goal g;
            if (explorer.NextGoal(map, pose, blacklist_, g))
            {
                out_goal = MakeGoal(g.x, g.y);
                detail_ = "к фронтиру";
                return true;
            }
            // исследовать нечего — планируем покрытие по итоговой карте
            waypoints_ = CoveragePlanner(config_.coverage).Plan(map, pose);
            waypoint_index_ = 0;
            frontiers_left_ = 0;
            if (waypoints_.empty())
            {
                Transition(State::kReturning, "свободной площади для покрытия нет");
                out_goal = MakeGoal(dock_.x, dock_.y);
                return true;
            }
            Transition(State::kCovering, "исследование завершено, покрытие");
            [[fallthrough]];
        }
        case State::kCovering:
        {
            if (waypoint_index_ >= waypoints_.size())
            {
                Transition(State::kReturning, "покрытие завершено, возврат на док");
                out_goal = MakeGoal(dock_.x, dock_.y);
                return true;
            }
            const auto& wp = waypoints_[waypoint_index_];
            retargeted_ = false;
            out_goal = MakeGoal(wp.first, wp.second);
            return true;
        }
        case State::kReturning:
        {
            const double d = std::hypot(dock_.x - pose.x, dock_.y - pose.y);
            if (d <= config_.dock_tolerance_m)
            {
                Transition(State::kDone, "на доке");
                return false;
            }
            if (!blacklist_.empty() && blocked_retries_ == 0 &&
                std::hypot(blacklist_.back().first - dock_.x, blacklist_.back().second - dock_.y) < 1e-6)
            {
                Transition(State::kDone, "док недостижим, остановка");
                return false;
            }
            out_goal = MakeGoal(dock_.x, dock_.y);
            return true;
        }
        default:
            return false;
    }
}

types::MissionState CleaningMission::Progress(const types::OccupancyMap& map) const
{
    types::MissionState s;
    s.state = state_;
    s.frontiers_left = static_cast<uint32_t>(frontiers_left_);
    s.waypoints_total = static_cast<uint32_t>(waypoints_.size());
    s.waypoints_done = static_cast<uint32_t>(std::min(waypoint_index_, waypoints_.size()));
    s.coverage_ratio = CoveragePlanner::CoverageRatio(map, visited_, config_.robot_radius_m);
    s.detail = detail_;
    return s;
}

}  // namespace algo
}  // namespace project
