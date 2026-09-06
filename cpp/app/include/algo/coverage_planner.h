#pragma once

#include <functional>
#include <utility>
#include <vector>

#include "types.h"

namespace project
{
namespace algo
{

struct CoveragePlannerConfig
{
    double lane_width_m{0.30};        // шаг змейки ≈ ширина щётки/корпуса с перекрытием
    double inflation_radius_m{0.20};  // ячейки ближе к стене недостижимы центром робота
    int min_run_cells{4};             // короче — щель, не проход (концы отрезка ещё на клетку внутрь)
    bool perimeter_pass{true};        // после змейки — обход вдоль стен и препятствий
    double perimeter_step_m{0.30};    // шаг точек обхода по контуру
};

/**
 * Покрытие свободной области карты змейкой (boustrophedon): полосы вдоль X с шагом lane_width,
 * в каждой полосе — отрезки по подряд идущим свободным ячейкам, направление чередуется.
 * Затем обход периметра: точки вдоль границы достижимой области (стены и препятствия), чтобы
 * добрать полосу у стен, куда полосы змейки не попадают.
 * Возвращает путевые точки в порядке обхода; между ними ведёт планировщик, так что препятствия
 * между отрезками обходятся сами.
 */
class CoveragePlanner
{
public:
    explicit CoveragePlanner(CoveragePlannerConfig config = {});

    std::vector<std::pair<double, double>> Plan(const types::OccupancyMap& map, const types::Pose2D& start) const;

    /** Доля свободных ячеек карты, оказавшихся под диском радиуса radius_m хотя бы в одной из
     *  пройденных точек. Метрика качества уборки и критерий в симуляторе. */
    static double CoverageRatio(
        const types::OccupancyMap& map,
        const std::vector<std::pair<double, double>>& visited,
        double radius_m
    );

    const CoveragePlannerConfig& config() const { return config_; }

private:
    void AppendPerimeter(
        const types::OccupancyMap& map,
        const std::function<bool(int, int)>& coverable,
        std::vector<std::pair<double, double>>& waypoints
    ) const;

    CoveragePlannerConfig config_;
};

}  // namespace algo
}  // namespace project
