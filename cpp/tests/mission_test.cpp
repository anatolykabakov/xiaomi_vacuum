#include "algo/cleaning_mission.h"
#include "algo/coverage_planner.h"
#include "algo/frontier_explorer.h"
#include "algo/path_planner.h"

#include <cmath>
#include <cstdint>

#include "gtest/gtest.h"

namespace project
{
namespace
{

constexpr uint8_t kUnknown = 127;
constexpr uint8_t kOccupied = 0;
constexpr uint8_t kFree = 255;

// Карта 120x120 по 5 см (6x6 м), origin в центре.
types::OccupancyMap MakeMap(uint8_t fill)
{
    types::OccupancyMap map;
    map.width = 120;
    map.height = 120;
    map.resolution = 0.05F;
    map.origin_x = 60;
    map.origin_y = 60;
    map.cells.assign(120 * 120, fill);
    return map;
}

void FillRect(types::OccupancyMap& map, int x0, int y0, int x1, int y1, uint8_t v)
{
    for (int y = y0; y <= y1; ++y)
        for (int x = x0; x <= x1; ++x)
            map.cells[static_cast<size_t>(x) + static_cast<size_t>(y) * map.width] = v;
}

// Закрытая комната 4x4 м: стены — занято, внутри свободно, снаружи неизвестно.
types::OccupancyMap ClosedRoom()
{
    auto map = MakeMap(kUnknown);
    FillRect(map, 20, 20, 100, 100, kOccupied);
    FillRect(map, 21, 21, 99, 99, kFree);
    return map;
}

algo::CleaningMissionConfig NoUndock()
{
    algo::CleaningMissionConfig c;
    c.undock_distance_m = 0;
    return c;
}

types::PlannerState Done(uint32_t goal_id)
{
    types::PlannerState s;
    s.state = types::PlannerState::State::kDone;
    s.goal_id = goal_id;
    return s;
}

types::PlannerState Following(uint32_t goal_id)
{
    types::PlannerState s;
    s.state = types::PlannerState::State::kFollowing;
    s.goal_id = goal_id;
    return s;
}

TEST(FrontierExplorerTest, FindsFrontierBetweenFreeAndUnknown)
{
    // половина комнаты известна: слева свободно, справа неизвестно, стен нет
    auto map = MakeMap(kUnknown);
    FillRect(map, 10, 10, 60, 110, kFree);
    algo::FrontierExplorer explorer;
    types::Pose2D pose;
    pose.x = -1.0;
    const auto frontiers = explorer.Find(map, pose);
    ASSERT_FALSE(frontiers.empty());
    // фронтир — у границы x≈0 (gx=60)
    EXPECT_NEAR(frontiers.front().x, 0.0, 0.3);
    types::Goal goal;
    EXPECT_TRUE(explorer.NextGoal(map, pose, {}, goal));
}

TEST(FrontierExplorerTest, NothingToExploreInClosedRoom)
{
    const auto map = ClosedRoom();
    algo::FrontierExplorer explorer;
    types::Pose2D pose;
    types::Goal goal;
    EXPECT_FALSE(explorer.NextGoal(map, pose, {}, goal));
}

TEST(CoveragePlannerTest, LanesCoverFreeArea)
{
    const auto map = ClosedRoom();
    algo::CoveragePlanner planner;
    types::Pose2D start;
    const auto wps = planner.Plan(map, start);
    ASSERT_GE(wps.size(), 4u);
    // все путевые точки — в свободных ячейках
    for (const auto& wp : wps)
    {
        int gx, gy;
        ASSERT_TRUE(algo::PathPlanner::WorldToGrid(map, wp.first, wp.second, gx, gy));
        EXPECT_EQ(map.cells[static_cast<size_t>(gx) + static_cast<size_t>(gy) * map.width], kFree);
    }
    // если пройти отрезки змейки, покрыта почти вся свободная область (кроме полосы у стен)
    std::vector<std::pair<double, double>> visited;
    for (size_t i = 0; i + 1 < wps.size(); i += 2)
    {
        const int n = 40;
        for (int k = 0; k <= n; ++k)
        {
            const double t = static_cast<double>(k) / n;
            visited.emplace_back(wps[i].first + t * (wps[i + 1].first - wps[i].first),
                                 wps[i].second + t * (wps[i + 1].second - wps[i].second));
        }
    }
    const double ratio = algo::CoveragePlanner::CoverageRatio(map, visited, 0.1725);
    EXPECT_GT(ratio, 0.85) << "покрыто " << ratio;
}

TEST(PathPlannerValidTest, TrajectoryValidUntilWallAppearsOrRobotStrays)
{
    auto map = ClosedRoom();
    algo::PathPlanner planner;
    types::Pose2D start;
    types::Goal goal;
    goal.x = 1.0;
    types::Trajectory traj;
    ASSERT_TRUE(planner.Plan(map, start, goal, traj));
    EXPECT_TRUE(planner.Valid(map, traj, start));
    types::Pose2D far;
    far.y = 1.0;
    EXPECT_FALSE(planner.Valid(map, traj, far));  // робот далеко от траектории
    int gx, gy;
    ASSERT_TRUE(algo::PathPlanner::WorldToGrid(map, 0.5, 0.0, gx, gy));
    map.cells[static_cast<size_t>(gx) + static_cast<size_t>(gy) * map.width] = kOccupied;
    EXPECT_FALSE(planner.Valid(map, traj, start));  // на пути появилась стена
}

TEST(CleaningMissionTest, ClosedRoomGoesCoverThenReturnThenDone)
{
    const auto map = ClosedRoom();
    algo::CleaningMission mission(NoUndock());
    types::Pose2D pose;  // док в центре комнаты
    mission.Start(pose);
    EXPECT_EQ(mission.state(), types::MissionState::State::kExploring);

    // фронтиров нет → сразу покрытие, первая точка змейки
    types::Goal goal;
    ASSERT_TRUE(mission.Step(map, pose, types::PlannerState{}, goal));
    EXPECT_EQ(mission.state(), types::MissionState::State::kCovering);
    const uint32_t total = mission.Progress(map).waypoints_total;
    ASSERT_GT(total, 0u);

    // «доезжаем» до каждой точки: планировщик сообщает done по её id, поза — в точке
    int guard = 0;
    while (mission.state() == types::MissionState::State::kCovering && guard++ < 1000)
    {
        pose.x = goal.x;
        pose.y = goal.y;
        types::Goal next;
        if (mission.Step(map, pose, Done(goal.id), next)) goal = next;
    }
    EXPECT_EQ(mission.state(), types::MissionState::State::kReturning);
    EXPECT_NEAR(goal.x, 0.0, 1e-9);  // цель — док
    EXPECT_NEAR(goal.y, 0.0, 1e-9);
    EXPECT_EQ(mission.Progress(map).waypoints_done, total);

    // едем на док
    pose.x = 0.0;
    pose.y = 0.0;
    types::Goal unused;
    mission.Step(map, pose, Done(goal.id), unused);
    EXPECT_EQ(mission.state(), types::MissionState::State::kDone);
    EXPECT_GT(mission.Progress(map).coverage_ratio, 0.5);
}

TEST(CleaningMissionTest, UndocksStraightAheadFirst)
{
    const auto map = ClosedRoom();
    algo::CleaningMission mission;  // undock_distance_m по умолчанию 0.30
    types::Pose2D pose;
    pose.yaw = M_PI / 2;  // док смотрит вдоль +y
    mission.Start(pose);
    types::Goal goal;
    ASSERT_TRUE(mission.Step(map, pose, types::PlannerState{}, goal));
    EXPECT_EQ(mission.state(), types::MissionState::State::kExploring);
    EXPECT_NEAR(goal.x, 0.0, 1e-6);
    EXPECT_NEAR(goal.y, 0.30, 1e-6);
    // съехали — фронтиров в закрытой комнате нет, дальше покрытие
    pose.y = 0.30;
    types::Goal next;
    ASSERT_TRUE(mission.Step(map, pose, Done(goal.id), next));
    EXPECT_EQ(mission.state(), types::MissionState::State::kCovering);
}

TEST(CleaningMissionTest, FrontierGoalDoneWhenNothingUnknownAround)
{
    // комната известна наполовину: цель-фронтир на границе, потом граница «уезжает»
    auto map = MakeMap(kUnknown);
    FillRect(map, 10, 10, 60, 110, kFree);
    algo::CleaningMission mission(NoUndock());
    types::Pose2D pose;
    pose.x = -1.0;
    mission.Start(pose);
    types::Goal goal;
    ASSERT_TRUE(mission.Step(map, pose, types::PlannerState{}, goal));
    EXPECT_EQ(mission.state(), types::MissionState::State::kExploring);
    // вокруг цели всё стало свободным, робот далеко, планировщик молчит — цель выполнена, новая цель
    FillRect(map, 10, 10, 110, 110, kFree);
    types::Goal next;
    ASSERT_TRUE(mission.Step(map, pose, Following(goal.id), next));
    EXPECT_NE(next.id, goal.id);
    EXPECT_TRUE(mission.blacklist().empty());
}

TEST(CleaningMissionTest, IgnoresDoneOfForeignGoalAndWaitsWhileFollowing)
{
    const auto map = ClosedRoom();
    algo::CleaningMission mission(NoUndock());
    types::Pose2D pose;
    mission.Start(pose);
    types::Goal goal;
    ASSERT_TRUE(mission.Step(map, pose, types::PlannerState{}, goal));
    // done от чужой цели и following по нашей — новой цели быть не должно
    types::Goal next;
    EXPECT_FALSE(mission.Step(map, pose, Done(goal.id + 100), next));
    EXPECT_FALSE(mission.Step(map, pose, Following(goal.id), next));
}

TEST(CleaningMissionTest, StopAndReturnCommands)
{
    const auto map = ClosedRoom();
    algo::CleaningMission mission(NoUndock());
    types::Pose2D pose;
    mission.Start(pose);
    types::Goal goal;
    mission.Step(map, pose, types::PlannerState{}, goal);
    mission.ReturnToDock();
    EXPECT_EQ(mission.state(), types::MissionState::State::kReturning);
    mission.Stop();
    EXPECT_EQ(mission.state(), types::MissionState::State::kIdle);
    EXPECT_FALSE(mission.Step(map, pose, types::PlannerState{}, goal));
}

}  // namespace
}  // namespace project
