#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>

// Типы сообщений внутренней шины — структуры C++ (на ARM дешевле protobuf).
// Protobuf — только на границе ZMQ (см. proto/ и services/zmq_bridge).
namespace project
{
namespace types
{

// ---------------------------------------------------------------- сенсоры (Platform)

/** Player ir:0 (wall) и ir:1 (cliff x4). Дальности в метрах. */
struct IrData
{
    uint64_t timestamp{0};
    double wall{0};
    double cliff0{0};
    double cliff1{0};
    double cliff2{0};
    double cliff3{0};
};

/** Player bumper:0. */
struct Bumper
{
    uint64_t timestamp{0};
    bool left{false};
    bool right{false};

    bool any() const { return left || right; }
};

struct SonarData
{
    uint64_t timestamp{0};
    double data{0};
};

struct BatteryState
{
    uint64_t timestamp{0};
    double percentage{0};
    bool charging{false};
};

/** Player position2d:0 — колёсная одометрия. */
struct Odometry
{
    uint64_t timestamp{0};
    double px{0};
    double py{0};
    double vx{0};
    double vy{0};
    double yaw{0};
    double omega{0};

    Eigen::Matrix<double, 3, 3> matrix() const
    {
        Eigen::Matrix<double, 3, 3> matrix;
        matrix << std::cos(yaw), -std::sin(yaw), px, std::sin(yaw), std::cos(yaw), py, 0, 0, 1;
        return matrix;
    }
};

/** Player laser:0 — один оборот LDS. */
struct LaserScan
{
    uint64_t timestamp{0};
    double scan_start{0};       // rad
    double scan_resolution{0};  // rad
    std::vector<float> ranges;  // m, 0 = нет отражения

    Eigen::Matrix<double, 3, Eigen::Dynamic> matrix() const
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

/** Player position3d:0 (gyro:::position3d:0 на Rockrobo). */
struct Imu
{
    uint64_t timestamp{0};
    double roll{0};
    double pitch{0};
    double yaw{0};
    double vel_roll{0};
    double vel_pitch{0};
    double vel_yaw{0};
};

/** Сводное состояние робота. */
struct RobotState
{
    uint64_t timestamp{0};
    double battery_percent{0};
    bool charging{false};
    bool docked{false};
    bool bumper{false};
    bool cliff{false};
    double wall_distance{0};
    std::string mode;   // idle | manual | auto
    std::string error;  // пусто, если всё в порядке
};

// ---------------------------------------------------------------- SLAM

/** Поза робота в кадре карты + текущая скорость. */
struct Pose2D
{
    uint64_t timestamp{0};
    double x{0};
    double y{0};
    double yaw{0};
    double v{0};  // м/с вдоль курса
    double w{0};  // рад/с
};

/** Карта проходимости: 0 = занято, 127 = неизвестно, 255 = свободно.
 *  origin_x/origin_y — в ячейках: gx = x/res + origin_x, gy = y/res + (height - origin_y),
 *  индекс = gx + gy * width (см. algo/occupancy_mapping и algo/path_planner). */
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

// ---------------------------------------------------------------- Planner

struct Goal
{
    enum class Type
    {
        kPoint,  // доехать в (x, y[, yaw])
        kDock,   // вернуться на базу (точка дока = поза старта)
        kStop    // отменить текущую цель
    };
    uint64_t timestamp{0};
    Type type{Type::kPoint};
    double x{0};
    double y{0};
    double yaw{std::numeric_limits<double>::quiet_NaN()};  // NaN = курс в конце не важен
    uint32_t id{0};
};

struct TrajectoryPoint
{
    double x{0};
    double y{0};
    double yaw{0};
    double v{0};  // м/с
    double w{0};  // рад/с
    double t{0};  // с от начала траектории
};

/** Одна траектория на оба канала: у дифференциального привода поперечное и продольное
 *  движение неразделимы. */
struct Trajectory
{
    uint64_t timestamp{0};
    uint32_t goal_id{0};
    std::vector<TrajectoryPoint> points;
};

struct PlannerState
{
    enum class State
    {
        kIdle,
        kNoMap,
        kPlanning,
        kFollowing,
        kBlocked,
        kDone
    };
    uint64_t timestamp{0};
    State state{State::kIdle};
    uint32_t goal_id{0};
    double distance_to_goal{0};
    std::string detail;
};

// ---------------------------------------------------------------- Mission

/** mission/command: старт, стоп или досрочный возврат на базу. */
struct MissionCommand
{
    enum class Command
    {
        kStart,
        kStop,
        kReturn
    };
    uint64_t timestamp{0};
    Command command{Command::kStart};
};

/** mission/state: фаза уборки и прогресс. */
struct MissionState
{
    enum class State
    {
        kIdle,
        kExploring,
        kCovering,
        kReturning,
        kDone
    };
    uint64_t timestamp{0};
    State state{State::kIdle};
    uint32_t frontiers_left{0};
    uint32_t waypoints_total{0};
    uint32_t waypoints_done{0};
    double coverage_ratio{0};  // доля свободной площади, пройденной роботом
    std::string detail;
};

// ---------------------------------------------------------------- Control / Safety

/** controls/cmd_vel: линейная и угловая скорость шасси. */
struct CmdVel
{
    uint64_t timestamp{0};
    double vx{0};
    double vy{0};
    double w{0};
};

/** Вентилятор и щётки, 0..1. Player-интерфейс motor:* у Rockrobo нестандартный,
 *  поэтому Platform команду принимает и логирует, но не исполняет. */
struct ActuatorCommand
{
    uint64_t timestamp{0};
    double fan{0};
    double main_brush{0};
    double side_brush{0};
};

/** safety/stop: аварийный стоп, Control обязан выполнить нулём. */
struct SafetyStop
{
    uint64_t timestamp{0};
    bool stop{false};
    std::string reason;
};

}  // namespace types
}  // namespace project
