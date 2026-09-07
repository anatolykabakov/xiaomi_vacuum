#include "algo/frontier_explorer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

#include "algo/path_planner.h"

namespace project
{
namespace algo
{

namespace
{
constexpr uint8_t kUnknown = 127;
constexpr uint8_t kFree = 255;
constexpr uint8_t kCellFree = 1;  // код Inflate(): 0 = препятствие, 1 = свободно, 2 = неизвестно
}  // namespace

FrontierExplorer::FrontierExplorer(FrontierExplorerConfig config) : config_(config) {}

std::vector<Frontier> FrontierExplorer::Find(const types::OccupancyMap& map, const types::Pose2D& pose) const
{
    std::vector<Frontier> result;
    const int w = static_cast<int>(map.width), h = static_cast<int>(map.height);
    if (map.cells.size() < static_cast<size_t>(w) * h) return result;

    PathPlannerConfig pc;
    pc.inflation_radius_m = config_.inflation_radius_m;
    const std::vector<uint8_t> inflated = PathPlanner(pc).Inflate(map);

    auto at = [&](int x, int y) { return static_cast<size_t>(x) + static_cast<size_t>(y) * w; };
    // фронтир: свободная ячейка, не в раздутом препятствии, с неизвестным соседом (8-связность)
    std::vector<uint8_t> is_frontier(static_cast<size_t>(w) * h, 0);
    for (int y = 1; y + 1 < h; ++y)
    {
        for (int x = 1; x + 1 < w; ++x)
        {
            if (map.cells[at(x, y)] != kFree || inflated[at(x, y)] != kCellFree) continue;
            bool unknown_near = false;
            for (int dy = -1; dy <= 1 && !unknown_near; ++dy)
                for (int dx = -1; dx <= 1; ++dx)
                    if ((dx || dy) && map.cells[at(x + dx, y + dy)] == kUnknown) { unknown_near = true; break; }
            if (unknown_near) is_frontier[at(x, y)] = 1;
        }
    }

    // кластеризация BFS
    std::vector<uint8_t> seen(is_frontier.size(), 0);
    for (int y = 0; y < h; ++y)
    {
        for (int x = 0; x < w; ++x)
        {
            const size_t s = at(x, y);
            if (!is_frontier[s] || seen[s]) continue;
            std::queue<std::pair<int, int>> q;
            q.emplace(x, y);
            seen[s] = 1;
            std::vector<std::pair<int, int>> cluster;
            while (!q.empty())
            {
                auto [cx, cy] = q.front();
                q.pop();
                cluster.emplace_back(cx, cy);
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dx = -1; dx <= 1; ++dx)
                    {
                        const int nx = cx + dx, ny = cy + dy;
                        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
                        const size_t ni = at(nx, ny);
                        if (is_frontier[ni] && !seen[ni]) { seen[ni] = 1; q.emplace(nx, ny); }
                    }
            }
            if (static_cast<int>(cluster.size()) < config_.min_cluster_cells) continue;
            // Целевая точка — ячейка кластера, ближайшая к роботу, но не под ним: центроид
            // протяжённого фронтира может лечь в неизвестное или в препятствие.
            Frontier f;
            f.cells = static_cast<int>(cluster.size());
            f.distance = std::numeric_limits<double>::infinity();
            for (const auto& [cx, cy] : cluster)
            {
                double wx, wy;
                PathPlanner::GridToWorld(map, cx, cy, wx, wy);
                const double d = std::hypot(wx - pose.x, wy - pose.y);
                if (d < config_.min_distance_m || d >= f.distance) continue;
                f.x = wx;
                f.y = wy;
                f.distance = d;
            }
            if (!std::isfinite(f.distance)) continue;
            result.push_back(f);
        }
    }
    std::sort(result.begin(), result.end(), [](const Frontier& a, const Frontier& b) { return a.distance < b.distance; });
    return result;
}

bool FrontierExplorer::NextGoal(
    const types::OccupancyMap& map,
    const types::Pose2D& pose,
    const std::vector<std::pair<double, double>>& blacklist,
    types::Goal& goal
) const
{
    PathPlannerConfig pc;
    pc.inflation_radius_m = config_.inflation_radius_m;
    pc.through_unknown = false;  // к фронтиру едем только по уже известному
    const PathPlanner planner(pc);

    for (const Frontier& f : Find(map, pose))
    {
        bool banned = false;
        for (const auto& b : blacklist)
            if (std::hypot(f.x - b.first, f.y - b.second) < config_.blacklist_radius_m) { banned = true; break; }
        if (banned) continue;

        types::Goal candidate;
        candidate.type = types::Goal::Type::kPoint;
        candidate.x = f.x;
        candidate.y = f.y;
        types::Trajectory probe;
        if (planner.Plan(map, pose, candidate, probe))
        {
            goal = candidate;
            return true;
        }
    }
    return false;
}

}  // namespace algo
}  // namespace project
