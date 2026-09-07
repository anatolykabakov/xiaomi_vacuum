#include "algo/path_planner.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>

namespace project
{
namespace algo
{

namespace
{

constexpr uint8_t kOccupied = 0;
constexpr uint8_t kUnknown = 127;
constexpr uint8_t kFree = 255;

constexpr uint8_t kCellBlocked = 0;
constexpr uint8_t kCellFree = 1;
constexpr uint8_t kCellUnknown = 2;
constexpr uint8_t kCellEscape = 3;  // раздутая ячейка у старта/цели: проходима, но дорого

double NormalizeAngle(double a)
{
    while (a > M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
}

}  // namespace

PathPlanner::PathPlanner(PathPlannerConfig config) : config_(config) {}

bool PathPlanner::WorldToGrid(const types::OccupancyMap& map, double x, double y, int& gx, int& gy)
{
    gx = static_cast<int>(std::floor(x / map.resolution + map.origin_x));
    gy = static_cast<int>(
        std::floor(y / map.resolution + (static_cast<double>(map.height) - map.origin_y))
    );
    return gx >= 0 && gy >= 0 && gx < static_cast<int>(map.width) &&
           gy < static_cast<int>(map.height);
}

void PathPlanner::GridToWorld(const types::OccupancyMap& map, int gx, int gy, double& x, double& y)
{
    x = (gx + 0.5 - map.origin_x) * map.resolution;
    y = (gy + 0.5 - (static_cast<double>(map.height) - map.origin_y)) * map.resolution;
}

std::vector<uint8_t> PathPlanner::Inflate(const types::OccupancyMap& map) const
{
    const int w = static_cast<int>(map.width);
    const int h = static_cast<int>(map.height);
    std::vector<uint8_t> grid(static_cast<size_t>(w) * h, kCellFree);
    for (size_t i = 0; i < grid.size() && i < map.cells.size(); ++i)
    {
        if (map.cells[i] == kUnknown) grid[i] = kCellUnknown;
    }
    const int r = std::max(1, static_cast<int>(std::ceil(config_.inflation_radius_m / map.resolution)));
    for (int y = 0; y < h; ++y)
    {
        for (int x = 0; x < w; ++x)
        {
            const size_t idx = static_cast<size_t>(x) + static_cast<size_t>(y) * w;
            if (idx >= map.cells.size() || map.cells[idx] != kOccupied) continue;
            for (int dy = -r; dy <= r; ++dy)
            {
                for (int dx = -r; dx <= r; ++dx)
                {
                    if (dx * dx + dy * dy > r * r) continue;
                    const int nx = x + dx, ny = y + dy;
                    if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
                    grid[static_cast<size_t>(nx) + static_cast<size_t>(ny) * w] = kCellBlocked;
                }
            }
        }
    }
    return grid;
}

bool PathPlanner::AStar(
    const std::vector<uint8_t>& grid,
    int width,
    int height,
    int sx,
    int sy,
    int gx,
    int gy,
    std::vector<std::pair<int, int>>& path
) const
{
    auto idx = [width](int x, int y) { return static_cast<size_t>(x) + static_cast<size_t>(y) * width; };
    const size_t n = static_cast<size_t>(width) * height;
    if (grid[idx(gx, gy)] == kCellBlocked) return false;
    if (!config_.through_unknown && grid[idx(gx, gy)] == kCellUnknown) return false;

    std::vector<double> g(n, std::numeric_limits<double>::infinity());
    std::vector<int> parent(n, -1);
    std::vector<bool> closed(n, false);
    using Node = std::pair<double, size_t>;  // f, idx
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;

    auto heur = [gx, gy](int x, int y) { return std::hypot(x - gx, y - gy); };
    g[idx(sx, sy)] = 0;
    open.emplace(heur(sx, sy), idx(sx, sy));

    static const int kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    static const int kDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    const size_t goal_idx = idx(gx, gy);

    while (!open.empty())
    {
        const size_t cur = open.top().second;
        open.pop();
        if (closed[cur]) continue;
        closed[cur] = true;
        if (cur == goal_idx) break;
        const int cx = static_cast<int>(cur % width), cy = static_cast<int>(cur / width);
        for (int k = 0; k < 8; ++k)
        {
            const int nx = cx + kDx[k], ny = cy + kDy[k];
            if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
            const size_t ni = idx(nx, ny);
            const uint8_t cell = grid[ni];
            if (cell == kCellBlocked) continue;
            if (cell == kCellUnknown && !config_.through_unknown) continue;
            double step = (k < 4) ? 1.0 : M_SQRT2;
            if (cell == kCellUnknown) step *= config_.unknown_cost;
            if (cell == kCellEscape) step *= config_.escape_cost;
            const double ng = g[cur] + step;
            if (ng < g[ni])
            {
                g[ni] = ng;
                parent[ni] = static_cast<int>(cur);
                open.emplace(ng + heur(nx, ny), ni);
            }
        }
    }
    if (!closed[goal_idx]) return false;

    path.clear();
    for (int cur = static_cast<int>(goal_idx); cur != -1; cur = parent[static_cast<size_t>(cur)])
    {
        path.emplace_back(cur % width, cur / width);
    }
    std::reverse(path.begin(), path.end());
    return true;
}

void PathPlanner::Smooth(
    std::vector<std::pair<double, double>>& pts,
    const types::OccupancyMap& map,
    const std::vector<uint8_t>& grid
) const
{
    if (pts.size() < 3) return;
    // Градиентное сглаживание с удержанием исходных точек (концы фиксированы).
    const double w_data = 0.5, w_smooth = 0.3;
    const std::vector<std::pair<double, double>> orig = pts;
    auto blocked = [&](const std::pair<double, double>& p) {
        int gx, gy;
        if (!WorldToGrid(map, p.first, p.second, gx, gy)) return true;
        return grid[static_cast<size_t>(gx) + static_cast<size_t>(gy) * map.width] == kCellBlocked;
    };
    for (int it = 0; it < config_.smoothing_iterations; ++it)
    {
        for (size_t i = 1; i + 1 < pts.size(); ++i)
        {
            std::pair<double, double> cand = pts[i];
            cand.first += w_data * (orig[i].first - pts[i].first) +
                          w_smooth * (pts[i - 1].first + pts[i + 1].first - 2 * pts[i].first);
            cand.second += w_data * (orig[i].second - pts[i].second) +
                           w_smooth * (pts[i - 1].second + pts[i + 1].second - 2 * pts[i].second);
            // срезать угол в раздутое препятствие нельзя — остаёмся на A*-пути
            pts[i] = blocked(cand) ? orig[i] : cand;
        }
    }
}

void PathPlanner::Profile(
    const std::vector<std::pair<double, double>>& pts,
    double start_yaw,
    double goal_yaw,
    types::Trajectory& out
) const
{
    out.points.clear();
    if (pts.empty()) return;
    const size_t n = pts.size();

    // курс в каждой точке — направление на следующую
    std::vector<double> yaw(n, start_yaw);
    for (size_t i = 0; i + 1 < n; ++i)
    {
        yaw[i] = std::atan2(pts[i + 1].second - pts[i].second, pts[i + 1].first - pts[i].first);
    }
    if (n >= 2) yaw[n - 1] = std::isnan(goal_yaw) ? yaw[n - 2] : goal_yaw;

    // остаток пути до цели — для торможения
    std::vector<double> remain(n, 0.0);
    for (size_t i = n - 1; i > 0; --i)
    {
        remain[i - 1] = remain[i] + std::hypot(pts[i].first - pts[i - 1].first, pts[i].second - pts[i - 1].second);
    }

    // скорость: лимит, кривизна (w = v*κ ≤ max_w), торможение к цели (v² = 2 a s)
    std::vector<double> v(n, config_.max_v_mps);
    for (size_t i = 1; i + 1 < n; ++i)
    {
        const double ds = std::hypot(pts[i + 1].first - pts[i].first, pts[i + 1].second - pts[i].second);
        const double dyaw = std::fabs(NormalizeAngle(yaw[i + 1] - yaw[i]));
        if (ds > 1e-6 && dyaw > 1e-6)
        {
            const double kappa = dyaw / ds;
            v[i] = std::min(v[i], config_.max_w_rps / kappa);
        }
    }
    for (size_t i = 0; i < n; ++i)
    {
        v[i] = std::min(v[i], std::sqrt(2.0 * config_.max_accel_mps2 * remain[i]));
        v[i] = std::max(v[i], (i + 1 < n) ? 0.05 : 0.0);  // не ноль внутри пути
    }

    double t = 0.0;
    for (size_t i = 0; i < n; ++i)
    {
        types::TrajectoryPoint p;
        p.x = pts[i].first;
        p.y = pts[i].second;
        p.yaw = yaw[i];
        p.v = v[i];
        if (i > 0)
        {
            const double ds = std::hypot(pts[i].first - pts[i - 1].first, pts[i].second - pts[i - 1].second);
            const double v_avg = std::max(0.5 * (v[i] + v[i - 1]), 0.02);
            const double dt = ds / v_avg;
            t += dt;
            p.w = NormalizeAngle(yaw[i] - yaw[i - 1]) / std::max(dt, 1e-3);
        }
        p.t = t;
        out.points.push_back(p);
    }
}

bool PathPlanner::Valid(
    const types::OccupancyMap& map,
    const types::Trajectory& trajectory,
    const types::Pose2D& pose
) const
{
    const auto& pts = trajectory.points;
    if (pts.empty() || map.cells.empty() || map.width == 0 || map.height == 0) return false;

    std::size_t nearest = 0;
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        const double d = std::hypot(pts[i].x - pose.x, pts[i].y - pose.y);
        if (d < best)
        {
            best = d;
            nearest = i;
        }
    }
    if (best > config_.replan_distance_m) return false;

    const int w = static_cast<int>(map.width), h = static_cast<int>(map.height);
    const double rr = config_.robot_radius_m / map.resolution + 0.5;
    const int rri = static_cast<int>(std::ceil(rr));
    for (std::size_t i = nearest; i < pts.size(); ++i)
    {
        int gx, gy;
        if (!WorldToGrid(map, pts[i].x, pts[i].y, gx, gy)) return false;
        for (int dy = -rri; dy <= rri; ++dy)
            for (int dx = -rri; dx <= rri; ++dx)
            {
                if (dx * dx + dy * dy >= rr * rr) continue;
                const int nx = gx + dx, ny = gy + dy;
                if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
                if (map.cells[static_cast<size_t>(nx) + static_cast<size_t>(ny) * w] == kOccupied) return false;
            }
    }
    return true;
}

bool PathPlanner::Plan(
    const types::OccupancyMap& map,
    const types::Pose2D& start,
    const types::Goal& goal,
    types::Trajectory& out
) const
{
    out.points.clear();
    out.goal_id = goal.id;
    if (map.cells.empty() || map.width == 0 || map.height == 0) return false;

    int sx, sy, gx, gy;
    if (!WorldToGrid(map, start.x, start.y, sx, sy)) return false;
    if (!WorldToGrid(map, goal.x, goal.y, gx, gy)) return false;

    std::vector<uint8_t> grid = Inflate(map);
    const int w = static_cast<int>(map.width);
    const int h = static_cast<int>(map.height);
    // Старт и цель могут лежать внутри раздутого препятствия: робот стоит у стены, док стоит у
    // стены. В диске радиуса раздувания вокруг них раздувание снимаем, но только там, где корпус
    // физически помещается (не ближе robot_radius_m к стене); такие ячейки дорогие, чтобы путь
    // сразу уходил от стены, а не шёл вдоль неё.
    const int r = std::max(1, static_cast<int>(std::ceil(config_.inflation_radius_m / map.resolution)));
    // стена может лежать в любом месте занятой ячейки — к радиусу корпуса полклетки запаса
    const double rr = config_.robot_radius_m / map.resolution + 0.5;
    const int rri = static_cast<int>(std::ceil(rr));
    auto fits = [&](int cx, int cy) {
        for (int dy = -rri; dy <= rri; ++dy)
            for (int dx = -rri; dx <= rri; ++dx)
            {
                if (dx * dx + dy * dy >= rr * rr) continue;
                const int nx = cx + dx, ny = cy + dy;
                if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
                if (map.cells[static_cast<size_t>(nx) + static_cast<size_t>(ny) * w] == kOccupied) return false;
            }
        return true;
    };
    auto unblock = [&](int cx, int cy) {
        for (int dy = -r; dy <= r; ++dy)
            for (int dx = -r; dx <= r; ++dx)
            {
                if (dx * dx + dy * dy > r * r) continue;
                const int nx = cx + dx, ny = cy + dy;
                if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
                const size_t i = static_cast<size_t>(nx) + static_cast<size_t>(ny) * w;
                if (grid[i] != kCellBlocked || map.cells[i] == kUnknown || !fits(nx, ny)) continue;
                grid[i] = kCellEscape;
            }
    };
    unblock(sx, sy);
    unblock(gx, gy);
    grid[static_cast<size_t>(sx) + static_cast<size_t>(sy) * w] = kCellFree;

    std::vector<std::pair<int, int>> cells;
    if (!AStar(grid, w, static_cast<int>(map.height), sx, sy, gx, gy, cells)) return false;

    // в метры + прореживание
    std::vector<std::pair<double, double>> pts;
    const size_t step = std::max<size_t>(1, static_cast<size_t>(config_.waypoint_step_m / map.resolution));
    for (size_t i = 0; i < cells.size(); i += step)
    {
        double x, y;
        GridToWorld(map, cells[i].first, cells[i].second, x, y);
        pts.emplace_back(x, y);
    }
    if (cells.size() > 1 && (cells.size() - 1) % step != 0)
    {
        double x, y;
        GridToWorld(map, cells.back().first, cells.back().second, x, y);
        pts.emplace_back(x, y);
    }
    // первая точка — сам робот, последняя — точная цель
    if (!pts.empty()) pts.front() = {start.x, start.y};
    if (!pts.empty()) pts.back() = {goal.x, goal.y};

    Smooth(pts, map, grid);
    Profile(pts, start.yaw, goal.yaw, out);
    return !out.points.empty();
}

}  // namespace algo
}  // namespace project
