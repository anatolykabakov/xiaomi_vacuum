#include "algo/coverage_planner.h"

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
constexpr uint8_t kFree = 255;
constexpr uint8_t kCellFree = 1;
}  // namespace

CoveragePlanner::CoveragePlanner(CoveragePlannerConfig config) : config_(config) {}

std::vector<std::pair<double, double>> CoveragePlanner::Plan(
    const types::OccupancyMap& map,
    const types::Pose2D& start
) const
{
    std::vector<std::pair<double, double>> waypoints;
    const int w = static_cast<int>(map.width), h = static_cast<int>(map.height);
    if (map.cells.size() < static_cast<size_t>(w) * h) return waypoints;

    PathPlannerConfig pc;
    pc.inflation_radius_m = config_.inflation_radius_m;
    const std::vector<uint8_t> grid = PathPlanner(pc).Inflate(map);
    const std::function<bool(int, int)> coverable = [&](int x, int y) {
        const size_t i = static_cast<size_t>(x) + static_cast<size_t>(y) * w;
        return map.cells[i] == kFree && grid[i] == kCellFree;
    };

    // полосы: строки сетки с шагом lane_width, начиная с ближайшей к роботу
    const int step = std::max(1, static_cast<int>(std::round(config_.lane_width_m / map.resolution)));
    int sx, sy;
    if (!PathPlanner::WorldToGrid(map, start.x, start.y, sx, sy)) { sx = w / 2; sy = h / 2; }
    int ymin = h, ymax = -1;
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            if (coverable(x, y)) { ymin = std::min(ymin, y); ymax = std::max(ymax, y); break; }
    if (ymax < 0) return waypoints;
    std::vector<int> rows;
    for (int y = sy % step; y < h; y += step)
        if (y >= ymin && y <= ymax) rows.push_back(y);
    // крайние полосы у стен: иначе у первой и последней стены остаётся непройденная кромка
    for (int edge : {ymin, ymax})
    {
        bool near = false;
        for (int y : rows) if (std::abs(y - edge) <= 1) near = true;
        if (!near) rows.push_back(edge);
    }
    std::sort(rows.begin(), rows.end(), [sy](int a, int b) { return std::abs(a - sy) < std::abs(b - sy); });
    // обход полос сверху вниз от ближайшей: сначала все выше, потом все ниже — одним проходом
    std::vector<int> ordered;
    for (int y : rows) if (y <= sy) ordered.push_back(y);
    std::sort(ordered.begin(), ordered.end(), std::greater<int>());
    std::vector<int> below;
    for (int y : rows) if (y > sy) below.push_back(y);
    std::sort(below.begin(), below.end());
    // едем вверх от робота до края, затем возвращаемся вниз через непройденные
    std::vector<int> order = ordered;
    order.insert(order.end(), below.begin(), below.end());

    bool left_to_right = true;
    for (int y : order)
    {
        // отрезки свободных ячеек в строке
        std::vector<std::pair<int, int>> runs;
        int run_start = -1;
        for (int x = 0; x <= w; ++x)
        {
            const bool free = x < w && coverable(x, y);
            if (free && run_start < 0) run_start = x;
            if (!free && run_start >= 0)
            {
                // концы отрезка на клетку внутрь: граница раздувания на живой карте гуляет на клетку,
                // и точка ровно на границе через минуту оказывается недостижимой
                if (x - run_start >= config_.min_run_cells) runs.emplace_back(run_start + 1, x - 2);
                run_start = -1;
            }
        }
        if (runs.empty()) continue;
        if (!left_to_right) std::reverse(runs.begin(), runs.end());
        for (const auto& r : runs)
        {
            const int a = left_to_right ? r.first : r.second;
            const int b = left_to_right ? r.second : r.first;
            double x0, y0, x1, y1;
            PathPlanner::GridToWorld(map, a, y, x0, y0);
            PathPlanner::GridToWorld(map, b, y, x1, y1);
            waypoints.emplace_back(x0, y0);
            waypoints.emplace_back(x1, y1);
        }
        left_to_right = !left_to_right;
    }
    if (config_.perimeter_pass) AppendPerimeter(map, coverable, waypoints);
    return waypoints;
}

void CoveragePlanner::AppendPerimeter(
    const types::OccupancyMap& map,
    const std::function<bool(int, int)>& coverable,
    std::vector<std::pair<double, double>>& waypoints
) const
{
    const int w = static_cast<int>(map.width), h = static_cast<int>(map.height);
    // контур: достижимая ячейка, у которой есть недостижимый сосед (4-связность)
    std::vector<std::pair<int, int>> contour;
    for (int y = 1; y + 1 < h; ++y)
        for (int x = 1; x + 1 < w; ++x)
        {
            if (!coverable(x, y)) continue;
            if (!coverable(x - 1, y) || !coverable(x + 1, y) || !coverable(x, y - 1) || !coverable(x, y + 1))
                contour.emplace_back(x, y);
        }
    if (contour.empty()) return;

    // обход цепочкой ближайших: от конца змейки, всегда к ближайшей непройденной ячейке контура;
    // точку добавляем, когда от прошлой добавленной набежал perimeter_step
    const int step_cells = std::max(1, static_cast<int>(std::round(config_.perimeter_step_m / map.resolution)));
    int cx, cy;
    if (waypoints.empty() || !PathPlanner::WorldToGrid(map, waypoints.back().first, waypoints.back().second, cx, cy))
    {
        cx = contour.front().first;
        cy = contour.front().second;
    }
    std::vector<bool> used(contour.size(), false);
    int last_x = cx, last_y = cy;
    bool first = true;
    for (size_t n = 0; n < contour.size(); ++n)
    {
        size_t best = contour.size();
        long best_d = std::numeric_limits<long>::max();
        for (size_t i = 0; i < contour.size(); ++i)
        {
            if (used[i]) continue;
            const long dx = contour[i].first - cx, dy = contour[i].second - cy;
            const long d = dx * dx + dy * dy;
            if (d < best_d)
            {
                best_d = d;
                best = i;
            }
        }
        if (best == contour.size()) break;
        used[best] = true;
        cx = contour[best].first;
        cy = contour[best].second;
        const long ddx = cx - last_x, ddy = cy - last_y;
        if (first || ddx * ddx + ddy * ddy >= static_cast<long>(step_cells) * step_cells)
        {
            double wx, wy;
            PathPlanner::GridToWorld(map, cx, cy, wx, wy);
            waypoints.emplace_back(wx, wy);
            last_x = cx;
            last_y = cy;
            first = false;
        }
    }
}

double CoveragePlanner::CoverageRatio(
    const types::OccupancyMap& map,
    const std::vector<std::pair<double, double>>& visited,
    double radius_m
)
{
    const int w = static_cast<int>(map.width), h = static_cast<int>(map.height);
    if (map.cells.size() < static_cast<size_t>(w) * h) return 0.0;
    std::vector<uint8_t> covered(static_cast<size_t>(w) * h, 0);
    const int r = std::max(1, static_cast<int>(std::ceil(radius_m / map.resolution)));
    auto stamp = [&](double x, double y) {
        int gx, gy;
        if (!PathPlanner::WorldToGrid(map, x, y, gx, gy)) return;
        for (int dy = -r; dy <= r; ++dy)
            for (int dx = -r; dx <= r; ++dx)
            {
                if (dx * dx + dy * dy > r * r) continue;
                const int nx = gx + dx, ny = gy + dy;
                if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
                covered[static_cast<size_t>(nx) + static_cast<size_t>(ny) * w] = 1;
            }
    };
    // позы приходят выборкой; между соседними точками робот проехал по прямой
    for (size_t i = 0; i < visited.size(); ++i)
    {
        stamp(visited[i].first, visited[i].second);
        if (i == 0) continue;
        const double dx = visited[i].first - visited[i - 1].first;
        const double dy = visited[i].second - visited[i - 1].second;
        const int n = static_cast<int>(std::hypot(dx, dy) / map.resolution);
        for (int k = 1; k < n; ++k)
        {
            const double t = static_cast<double>(k) / n;
            stamp(visited[i - 1].first + t * dx, visited[i - 1].second + t * dy);
        }
    }
    size_t free_total = 0, free_covered = 0;
    for (size_t i = 0; i < covered.size(); ++i)
    {
        if (map.cells[i] != kFree) continue;
        ++free_total;
        if (covered[i]) ++free_covered;
    }
    return free_total ? static_cast<double>(free_covered) / free_total : 0.0;
}

}  // namespace algo
}  // namespace project
