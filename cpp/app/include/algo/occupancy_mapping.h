#pragma once

#include "types.h"
#include <Eigen/Dense>
#include <utility>
#include <vector>

namespace project
{
namespace algo
{

std::vector<std::pair<int, int>> Bresenham(int x0, int y0, int x1, int y1);

class OccupancyMapping
{
public:
    OccupancyMapping();
    ~OccupancyMapping();

    types::OccupancyMap UpdateMap();
    void UpdateScan(const types::LaserData& scan);
    void UpdateOdom(const types::OdometryData& odom);

private:
    types::OccupancyMap map_;
    types::LaserData scan_;
    types::OdometryData odom_;

    Eigen::Matrix<double, 3, 3> map_to_grid_;
    Eigen::Matrix<double, 3, 3> lidar_to_base_;
};
}  // namespace algo
}  // namespace project
