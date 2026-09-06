#include "algo/path_planner.h"

#include <cmath>
#include <cstdint>
#include <limits>

#include "gtest/gtest.h"

namespace project
{
namespace
{

constexpr uint8_t kUnknown = 127;
constexpr uint8_t kOccupied = 0;
constexpr uint8_t kFree = 255;

// Карта 100x100 по 5 см (5x5 м), origin в центре — как у OccupancyMapping.
types::OccupancyMap MakeMap(uint8_t fill)
{
    types::OccupancyMap map;
    map.width = 100;
    map.height = 100;
    map.resolution = 0.05F;
    map.origin_x = 50;
    map.origin_y = 50;
    map.cells.assign(100 * 100, fill);
    return map;
}

void SetCell(types::OccupancyMap& map, int gx, int gy, uint8_t v)
{
    map.cells[static_cast<size_t>(gx) + static_cast<size_t>(gy) * map.width] = v;
}

types::Goal PointGoal(double x, double y)
{
    types::Goal g;
    g.type = types::Goal::Type::kPoint;
    g.x = x;
    g.y = y;
    g.id = 1;
    return g;
}

TEST(PathPlannerTest, WorldGridRoundTrip)
{
    const auto map = MakeMap(kFree);
    int gx, gy;
    ASSERT_TRUE(algo::PathPlanner::WorldToGrid(map, 0.0, 0.0, gx, gy));
    EXPECT_EQ(gx, 50);
    EXPECT_EQ(gy, 50);
    double x, y;
    algo::PathPlanner::GridToWorld(map, gx, gy, x, y);
    EXPECT_NEAR(x, 0.025, 1e-9);  // центр ячейки
    EXPECT_NEAR(y, 0.025, 1e-9);
    EXPECT_FALSE(algo::PathPlanner::WorldToGrid(map, 10.0, 0.0, gx, gy));  // вне карты
}

TEST(PathPlannerTest, StraightLineOnFreeMap)
{
    const auto map = MakeMap(kFree);
    algo::PathPlanner planner;
    types::Pose2D start;
    types::Trajectory traj;
    ASSERT_TRUE(planner.Plan(map, start, PointGoal(1.0, 0.0), traj));
    ASSERT_GE(traj.points.size(), 2u);
    EXPECT_NEAR(traj.points.front().x, 0.0, 1e-9);
    EXPECT_NEAR(traj.points.back().x, 1.0, 1e-9);
    EXPECT_NEAR(traj.points.back().y, 0.0, 1e-9);
    // время монотонно, скорости в лимитах, в конце тормозим
    for (size_t i = 1; i < traj.points.size(); ++i)
    {
        EXPECT_GT(traj.points[i].t, traj.points[i - 1].t);
        EXPECT_LE(traj.points[i].v, planner.config().max_v_mps + 1e-9);
    }
    EXPECT_NEAR(traj.points.back().v, 0.0, 1e-9);
}

TEST(PathPlannerTest, GoesAroundWall)
{
    auto map = MakeMap(kFree);
    // стена x = 0.5 м (gx = 60) от y = -1.5 до +1.5 м, проход только по краям
    for (int gy = 20; gy <= 80; ++gy) SetCell(map, 60, gy, kOccupied);
    algo::PathPlanner planner;
    types::Pose2D start;
    types::Trajectory traj;
    ASSERT_TRUE(planner.Plan(map, start, PointGoal(1.0, 0.0), traj));
    // траектория обязана обойти стену: хоть одна точка с |y| > 1.5
    double max_abs_y = 0.0;
    for (const auto& p : traj.points) max_abs_y = std::max(max_abs_y, std::fabs(p.y));
    EXPECT_GT(max_abs_y, 1.5);
    // и ни одна точка не лежит в раздутой стене
    const auto grid = planner.Inflate(map);
    for (const auto& p : traj.points)
    {
        int gx, gy;
        ASSERT_TRUE(algo::PathPlanner::WorldToGrid(map, p.x, p.y, gx, gy));
        EXPECT_NE(grid[static_cast<size_t>(gx) + static_cast<size_t>(gy) * map.width], 0)
            << "точка (" << p.x << "," << p.y << ") в препятствии";
    }
}

TEST(PathPlannerTest, BlockedWhenGoalWalledIn)
{
    auto map = MakeMap(kFree);
    // цель (1.0, 0) в коробке 3x3 ячейки из препятствий вокруг gx=70,gy=50
    for (int dx = -3; dx <= 3; ++dx)
        for (int dy = -3; dy <= 3; ++dy)
            if (std::abs(dx) == 3 || std::abs(dy) == 3) SetCell(map, 70 + dx, 50 + dy, kOccupied);
    algo::PathPlanner planner;
    types::Pose2D start;
    types::Trajectory traj;
    EXPECT_FALSE(planner.Plan(map, start, PointGoal(1.0, 0.0), traj));
}

TEST(PathPlannerTest, UnknownCellsPassableByDefaultAndBlockableByConfig)
{
    const auto map = MakeMap(kUnknown);
    types::Pose2D start;
    types::Trajectory traj;
    algo::PathPlanner through;  // through_unknown = true
    EXPECT_TRUE(through.Plan(map, start, PointGoal(1.0, 0.0), traj));

    algo::PathPlannerConfig cfg;
    cfg.through_unknown = false;
    algo::PathPlanner strict(cfg);
    EXPECT_FALSE(strict.Plan(map, start, PointGoal(1.0, 0.0), traj));
}

}  // namespace
}  // namespace project
