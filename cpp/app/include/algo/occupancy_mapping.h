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
    void UpdateScan(const types::LaserScan& scan);
    void UpdateOdom(const types::Odometry& odom);
    /** Препятствие, которого лидар не видит (сработал бампер): ячейка в кадре карты остаётся
     *  занятой, лучи её не стирают. */
    void MarkObstacle(double x, double y);

private:
    types::OccupancyMap map_;
    types::LaserScan scan_;
    types::Odometry odom_;
    std::vector<std::size_t> bump_cells_;

    Eigen::Matrix<double, 3, 3> map_to_grid_;
    Eigen::Matrix<double, 3, 3> lidar_to_base_;
};
}  // namespace algo
}  // namespace project
