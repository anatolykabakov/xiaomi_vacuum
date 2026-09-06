#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

#include <Eigen/Dense>

namespace project
{
namespace types
{

struct IrData
{
    uint64_t timestamp{0};
    double wall{0};
    double cliff0{0};
    double cliff1{0};
    double cliff2{0};
    double cliff3{0};
};

struct BatteryState
{
    uint64_t timestamp{0};
    double percentage{0};
    bool charging{false};
};

struct OdometryData
{
    uint64_t timestamp{0};
    double px{0};
    double py{0};
    double vx{0};
    double vy{0};
    double yaw{0};
    double omega{0};

    Eigen::Matrix<double, 3, 3> matrix()
    {
        Eigen::Matrix<double, 3, 3> matrix;
        matrix << std::cos(yaw), -std::sin(yaw), px, std::sin(yaw), std::cos(yaw), py, 0, 0, 1;
        return matrix;
    }
};

struct LaserData
{
    uint64_t timestamp{0};
    double scan_start{0};       // in radians
    double scan_resolution{0};  // in radians
    std::vector<float> ranges;

    Eigen::Matrix<double, 3, Eigen::Dynamic> matrix()
    {
        Eigen::Matrix<double, 3, Eigen::Dynamic> points(3, ranges.size());
        if (ranges.empty())
        {
            return points;
        }
        for (std::size_t i = 0; i < ranges.size(); i++)
        {
            double angle = scan_start + i * scan_resolution;
            double x = ranges[i] * std::cos(angle);
            double y = ranges[i] * std::sin(angle);
            points.block<3, 1>(0, i) = Eigen::Vector3d(x, y, 1).transpose();
        }
        return points;
    }
};

/** Player position3d:0 (gyro:::position3d:0 on Rockrobo). */
struct GyroData
{
    uint64_t timestamp{0};
    double roll{0};
    double pitch{0};
    double yaw{0};
    double vel_roll{0};
    double vel_pitch{0};
    double vel_yaw{0};
};

struct SonarData
{
    uint64_t timestamp{0};
    double data{0};
};

struct BatteryData
{
    uint64_t timestamp{0};
    double percentage{0};
    bool charging{false};
};

struct CmdVel
{
    uint64_t timestamp{0};
    double vx{0};
    double vy{0};
    double w{0};
};

struct OccupancyMap
{
    uint64_t timestamp{0};
    uint32_t width{300};
    uint32_t height{300};
    float resolution{0.05F};
    float origin_x{0};
    float origin_y{0};
    std::vector<uint8_t> cells;
    float robot_x{0};
    float robot_y{0};
    float robot_yaw{0};
};
}  // namespace types
}  // namespace project
