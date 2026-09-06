#pragma once

#include <memory>
#include <string_view>

#include "mw/middleware/manager.hpp"
#include "algo/occupancy_mapping.h"
#include "types.h"

namespace project
{
namespace services
{

/**
 * Карта проходимости и поза.
 *
 * Поза: x/y из колёсной одометрии, курс при наличии IMU — гироскопический, привязанный
 * к начальному курсу одометрии (колёса на ковре проскальзывают, гироскоп нет).
 * Карта: OccupancyMapping по лидару. Карта и поза строятся из одного скорректированного
 * состояния, чтобы не расходиться.
 */
class Slam : public mw::middleware::Service
{
public:
    Slam();
    ~Slam() override = default;

    std::string_view getName() const override { return "slam"; }

protected:
    void configure() override;

private:
    void onLaser(const types::LaserScan& scan);
    void onOdometry(const types::Odometry& odom);
    void onImu(const types::Imu& imu);
    void onBumper(const types::Bumper& bumper);
    void mapTick();

    std::unique_ptr<algo::OccupancyMapping> mapping_;
    types::Imu last_imu_;
    bool have_imu_{false};
    bool yaw_anchored_{false};
    double yaw_anchor_{0};  // odom_yaw0 - imu_yaw0
    bool use_imu_yaw_{true};
    int map_period_ms_{1000};
    types::Pose2D last_pose_;
    bool have_pose_{false};
};

}  // namespace services
}  // namespace project
