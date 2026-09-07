#pragma once

#include <cmath>

// Геометрия и лимиты робота. Значения из штатного
// /opt/rockrobo/cleaner/conf/ruby_chassis.cfg (robot_radius, odometry_parameter,
// max_speed, max_yaw_speed) и pose лидара из секции ruby_laser.
namespace project
{

struct RobotConfig
{
    double radius_m{0.1725};      // robot_radius
    double wheel_base_m{0.239};   // 2 * 0.1195 (YRight/YLeft из odometry_parameter)
    double max_v_mps{0.5};        // max_speed шасси
    double max_w_rps{3.14};       // max_yaw_speed шасси
    double max_accel_mps2{0.3};   // наш лимит разгона/торможения для профиля скорости

    // Штатный лимит без RoboController: MCU держит ~2–3 см/с вперёд (см. docs/PLATFORM.md),
    // поэтому планировщик по умолчанию просит скромно.
    double plan_v_mps{0.25};
    double plan_w_rps{1.0};

    // pose LDS относительно центра робота: [x y z roll pitch yaw] = [-0.100 0 0 0 0 97.6°]
    double lidar_offset_x_m{-0.1};
    double lidar_offset_y_m{0.0};
    double lidar_offset_yaw_rad{97.6 * M_PI / 180.0};
};

inline const RobotConfig& robotConfig()
{
    static const RobotConfig config;
    return config;
}

}  // namespace project
