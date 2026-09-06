#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include "types.h"

namespace project
{
namespace algo
{

struct PathPlannerConfig
{
    double inflation_radius_m{0.20};  // радиус робота 0.1725 + запас
    double robot_radius_m{0.1725};    // ближе к стене центр робота не бывает даже у дока
    double escape_cost{4.0};          // множитель стоимости шага по раздутой ячейке у старта/цели
    double replan_distance_m{0.30};   // робот дальше от траектории — она устарела, перепланировать
    bool through_unknown{true};       // неизвестные ячейки проходимы (с штрафом) — робот исследует
    double unknown_cost{3.0};         // множитель стоимости шага по неизвестному
    double max_v_mps{0.25};
    double max_w_rps{1.0};
    double max_accel_mps2{0.3};
    double goal_tolerance_m{0.10};
    double waypoint_step_m{0.10};  // шаг прореживания пути перед сглаживанием
    int smoothing_iterations{30};
};

/**
 * Глобальный путь по карте занятости (A*, 8-связность) + сглаживание + профиль
 * скорости → types::Trajectory. Чистый алгоритм без middleware, чтобы тестировать
 * без робота.
 */
class PathPlanner
{
public:
    explicit PathPlanner(PathPlannerConfig config = {});

    /** true и заполненная траектория при успехе; false если старт/цель вне карты
     *  или пути нет. */
    bool Plan(
        const types::OccupancyMap& map,
        const types::Pose2D& start,
        const types::Goal& goal,
        types::Trajectory& out
    ) const;

    /** Траектория ещё годится: робот не дальше replan_distance_m от неё и от ближайшей точки до
     *  конца корпус (robot_radius_m) не задевает занятых ячеек. Пока годится — перепланировать не
     *  надо: иначе на мерцающей карте пути у стен скачут между вариантами, а робот дёргается. */
    bool Valid(const types::OccupancyMap& map, const types::Trajectory& trajectory, const types::Pose2D& pose) const;

    /** Преобразования мир↔сетка ровно как в occupancy_mapping:
     *  gx = x/res + origin_x, gy = y/res + (height - origin_y). */
    static bool WorldToGrid(const types::OccupancyMap& map, double x, double y, int& gx, int& gy);
    static void GridToWorld(const types::OccupancyMap& map, int gx, int gy, double& x, double& y);

    /** 0 = препятствие (с учётом раздувания), 1 = свободно, 2 = неизвестно.
     *  Plan() дополнительно помечает 3 = раздутая ячейка у старта/цели, проходимая с штрафом. */
    std::vector<uint8_t> Inflate(const types::OccupancyMap& map) const;

    const PathPlannerConfig& config() const { return config_; }

private:
    bool AStar(
        const std::vector<uint8_t>& grid,
        int width,
        int height,
        int sx,
        int sy,
        int gx,
        int gy,
        std::vector<std::pair<int, int>>& path
    ) const;
    /** Сглаживание с проверкой столкновений: точка, ушедшая в занятую (раздутую)
     *  ячейку, возвращается на исходный A*-путь — иначе срезаются углы у препятствий. */
    void Smooth(
        std::vector<std::pair<double, double>>& pts,
        const types::OccupancyMap& map,
        const std::vector<uint8_t>& grid
    ) const;
    void Profile(
        const std::vector<std::pair<double, double>>& pts,
        double start_yaw,
        double goal_yaw,
        types::Trajectory& out
    ) const;

    PathPlannerConfig config_;
};

}  // namespace algo
}  // namespace project
