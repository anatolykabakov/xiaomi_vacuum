
#include "algo/occupancy_mapping.h"

#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include <Eigen/Dense>

namespace project
{
namespace
{

// Mirrors constants in occupacy_mapping.cpp (binary viz: occupied=black=0).
constexpr uint8_t kUnknown = 127;
constexpr uint8_t kOccupied = 0;
constexpr uint8_t kFree = 255;

constexpr double kLidarOffsetX = -0.1;
constexpr double kLidarOffsetY = 0.0;
constexpr double kLidarOffsetYaw = 97.6 * M_PI / 180.0;

Eigen::Matrix3d MapToGrid(const types::OccupancyMap& map)
{
    Eigen::Matrix3d grid;
    grid << 1.0 / static_cast<double>(map.resolution), 0.0, static_cast<double>(map.origin_x), 0.0,
        1.0 / static_cast<double>(map.resolution),
        static_cast<double>(map.height) - static_cast<double>(map.origin_y), 0.0, 0.0, 1.0;
    return grid;
}

Eigen::Matrix3d LidarToBase()
{
    Eigen::Matrix3d lidar;
    lidar << std::cos(kLidarOffsetYaw), -std::sin(kLidarOffsetYaw), kLidarOffsetX,
        std::sin(kLidarOffsetYaw), std::cos(kLidarOffsetYaw), kLidarOffsetY, 0.0, 0.0, 1.0;
    return lidar;
}

void ExpectEndFlags(int x0, int y0, int x1, int y1)
{
    const auto cells = algo::Bresenham(x0, y0, x1, y1);
    ASSERT_FALSE(cells.empty());
    EXPECT_EQ(cells.back(), std::make_pair(x1, y1));
}

}  // namespace

TEST(BresenhamTest, DiagonalLowOctant)
{
    const auto cells = algo::Bresenham(0, 0, 4, 2);
    const std::vector<std::pair<int, int>> expect = {{0, 0}, {1, 0}, {2, 1}, {3, 1}, {4, 2}};
    EXPECT_EQ(cells, expect);
}

TEST(BresenhamTest, SinglePixel)
{
    const auto cells = algo::Bresenham(3, 5, 3, 5);
    ASSERT_EQ(cells.size(), 1u);
    EXPECT_EQ(cells[0], std::make_pair(3, 5));
}

TEST(BresenhamTest, Horizontal)
{
    const auto cells = algo::Bresenham(0, 0, 3, 0);
    const std::vector<std::pair<int, int>> expect = {{0, 0}, {1, 0}, {2, 0}, {3, 0}};
    EXPECT_EQ(cells, expect);
}

TEST(BresenhamTest, Vertical)
{
    const auto cells = algo::Bresenham(2, 1, 2, 4);
    const std::vector<std::pair<int, int>> expect = {{2, 1}, {2, 2}, {2, 3}, {2, 4}};
    EXPECT_EQ(cells, expect);
}

TEST(BresenhamTest, SteepSlope)
{
    const auto cells = algo::Bresenham(0, 0, 2, 5);
    const std::vector<std::pair<int, int>> expect =
        {{0, 0}, {0, 1}, {1, 2}, {1, 3}, {2, 4}, {2, 5}};
    EXPECT_EQ(cells, expect);
}

TEST(BresenhamTest, EndCallbackFlag)
{
    ExpectEndFlags(0, 0, 4, 2);
    ExpectEndFlags(1, 1, 1, 1);
}

TEST(OccupancyMappingTest, EmptyScanLeavesMapUnknown)
{
    algo::OccupancyMapping mapping;
    const types::OccupancyMap map = mapping.UpdateMap();
    ASSERT_FALSE(map.cells.empty());
    EXPECT_EQ(map.cells[0], kUnknown);
}

TEST(OccupancyMappingTest, OneBeamMarksRayAndHit)
{
    algo::OccupancyMapping mapping;
    const types::OccupancyMap layout = mapping.UpdateMap();

    types::LaserData scan;
    scan.scan_start = 0.0;
    scan.scan_resolution = 0.0;
    scan.ranges = {1.0F};
    types::OdometryData odom;
    odom.px = 0.0;
    odom.py = 0.0;
    odom.yaw = 0.0;
    mapping.UpdateScan(scan);
    mapping.UpdateOdom(odom);
    const types::OccupancyMap map = mapping.UpdateMap();

    // Same grid transform as occupacy_mapping.cpp.
    const Eigen::Matrix3d lidar_on_grid = MapToGrid(layout) * odom.matrix() * LidarToBase();
    const int x0 = static_cast<int>(lidar_on_grid(0, 2));
    const int y0 = static_cast<int>(lidar_on_grid(1, 2));
    const Eigen::Matrix<int, 3, Eigen::Dynamic> endpoints =
        (lidar_on_grid * scan.matrix()).cast<int>();
    ASSERT_EQ(endpoints.cols(), 1);
    const int x1 = endpoints(0, 0);
    const int y1 = endpoints(1, 0);

    const uint32_t w = map.width;
    const uint32_t h = map.height;
    const std::vector<std::pair<int, int>> cells = algo::Bresenham(x0, y0, x1, y1);
    for (const auto& cell : cells)
    {
        const int px = cell.first;
        const int py = cell.second;
        if (px < 0 || py < 0 || px >= static_cast<int>(w) || py >= static_cast<int>(h))
        {
            continue;
        }
        const std::size_t idx = static_cast<std::size_t>(px) + static_cast<std::size_t>(py) * w;
        if (px == x1 && py == y1)
        {
            EXPECT_EQ(map.cells[idx], kOccupied);
        }
        else
        {
            EXPECT_EQ(map.cells[idx], kFree);
        }
    }
}

TEST(OccupancyMappingTest, ShortRangeBeamDoesNotUpdateMap)
{
    algo::OccupancyMapping mapping;
    types::LaserData scan;
    scan.scan_start = 0.0;
    scan.scan_resolution = 0.0;
    scan.ranges = {0.05F};
    types::OdometryData odom;
    odom.px = 0.0;
    odom.py = 0.0;
    odom.yaw = 0.0;
    mapping.UpdateScan(scan);
    mapping.UpdateOdom(odom);
    const types::OccupancyMap map = mapping.UpdateMap();
    EXPECT_EQ(map.cells[0], kUnknown);
}

}  // namespace project
