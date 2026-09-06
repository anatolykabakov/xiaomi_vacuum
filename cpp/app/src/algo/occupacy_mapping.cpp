
#include "algo/occupancy_mapping.h"

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace project
{
namespace algo
{

constexpr float kMinRangeM = 0.1F;
constexpr float kMaxRangeM = 10.0F;
constexpr uint8_t kUnknown = 127;
constexpr uint8_t kOccupied = 0;
constexpr uint8_t kFree = 255;
constexpr double kLidarOffsetX = -0.1;
constexpr double kLidarOffsetY = 0.0;
constexpr double kLidarOffsetYaw = 97.6 * M_PI / 180.0;

std::vector<std::pair<int, int>> Bresenham(int x0, int y0, int x1, int y1)
{
    std::vector<std::pair<int, int>> line;
    if (x0 == x1 && y0 == y1)
    {
        line.push_back({x0, y0});
        return line;
    }

    int x = x0;
    int y = y0;
    const int sx = x0 < x1 ? 1 : -1;
    const int sy = y0 < y1 ? 1 : -1;
    const int dx = std::abs(x1 - x0);
    const int dy = std::abs(y1 - y0);

    if (dx > dy)
    {
        int err = dx / 2;
        while (x != x1)
        {
            line.push_back({x, y});
            err -= dy;
            if (err < 0)
            {
                y += sy;
                err += dx;
            }
            x += sx;
        }
    }
    else
    {
        int err = dy / 2;
        while (y != y1)
        {
            line.push_back({x, y});
            err -= dx;
            if (err < 0)
            {
                x += sx;
                err += dy;
            }
            y += sy;
        }
    }
    line.push_back({x1, y1});
    return line;
}

OccupancyMapping::OccupancyMapping()
{
    map_.width = 300;
    map_.height = 300;
    map_.resolution = 0.05F;
    map_.origin_x = static_cast<float>(map_.width) / 2;
    map_.origin_y = static_cast<float>(map_.height) / 2;
    map_.cells.assign(static_cast<size_t>(map_.width) * static_cast<size_t>(map_.height), kUnknown);

    map_to_grid_ << 1 / map_.resolution, 0, map_.origin_x, 0, 1 / map_.resolution,
        static_cast<float>(map_.height) - map_.origin_y, 0, 0, 1;

    lidar_to_base_ << std::cos(kLidarOffsetYaw), -std::sin(kLidarOffsetYaw), kLidarOffsetX,
        std::sin(kLidarOffsetYaw), std::cos(kLidarOffsetYaw), kLidarOffsetY, 0, 0, 1;
}

OccupancyMapping::~OccupancyMapping() {}

types::OccupancyMap OccupancyMapping::UpdateMap()
{
    if (scan_.ranges.empty())
    {
        return map_;
    }
    const Eigen::Matrix<double, 3, 3> lidar_on_grid =
        map_to_grid_ * odom_.matrix() * lidar_to_base_;
    const int lidar_gx = static_cast<int>(lidar_on_grid(0, 2));
    const int lidar_gy = static_cast<int>(lidar_on_grid(1, 2));
    const Eigen::Matrix<int, 3, Eigen::Dynamic> xyz = (lidar_on_grid * scan_.matrix()).cast<int>();
    for (int i = 0; i < xyz.cols(); ++i)
    {
        const float range = scan_.ranges[static_cast<size_t>(i)];
        if (range < kMinRangeM || range > kMaxRangeM)
        {
            continue;
        }

        const int end_x = xyz(0, i);
        const int end_y = xyz(1, i);
        if (end_x < 0 || end_y < 0 || end_x >= static_cast<int>(map_.width) ||
            end_y >= static_cast<int>(map_.height))
        {
            continue;
        }
        const auto line = Bresenham(lidar_gx, lidar_gy, end_x, end_y);
        for (const auto& cell : line)
        {
            const int px = cell.first;
            const int py = cell.second;
            if (px < 0 || py < 0 || px >= static_cast<int>(map_.width) ||
                py >= static_cast<int>(map_.height))
            {
                continue;
            }
            const size_t idx = static_cast<size_t>(px) + static_cast<size_t>(py) * map_.width;
            if (px == end_x && py == end_y)
            {
                map_.cells[idx] = kOccupied;
            }
            else
            {
                map_.cells[idx] = kFree;
            }
        }
    }
    return map_;
}

void OccupancyMapping::UpdateScan(const types::LaserData& scan)
{
    scan_ = scan;
}

void OccupancyMapping::UpdateOdom(const types::OdometryData& odom)
{
    odom_ = odom;
}

}  // namespace algo
}  // namespace project
