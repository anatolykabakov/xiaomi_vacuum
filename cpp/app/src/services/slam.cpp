#include "services/slam.h"

#include <cmath>

#include "robot_config.h"
#include "utils/topics.h"

namespace project
{
namespace services
{

namespace
{
double NormalizeAngle(double a)
{
    while (a > M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
}
}  // namespace

Slam::Slam() : mapping_(new algo::OccupancyMapping()) {}

void Slam::configure()
{
    registerParameter<bool>("slam_use_imu_yaw", use_imu_yaw_);

    subscribe<types::LaserScan>(topics::kSensorsLaser, [this](const types::LaserScan& s) { onLaser(s); });
    subscribe<types::Odometry>(topics::kSensorsOdometry, [this](const types::Odometry& o) { onOdometry(o); });
    subscribe<types::Imu>(topics::kSensorsImu, [this](const types::Imu& i) { onImu(i); });
    subscribe<types::Bumper>(topics::kSensorsBumper, [this](const types::Bumper& b) { onBumper(b); });

    scheduleTimer(map_period_ms_, [this] { mapTick(); }, "map");
}

void Slam::onLaser(const types::LaserScan& scan)
{
    mapping_->UpdateScan(scan);
}

void Slam::onImu(const types::Imu& imu)
{
    last_imu_ = imu;
    have_imu_ = true;
}

void Slam::onBumper(const types::Bumper& bumper)
{
    // Бампер задел то, чего лидар не видит (низкий порог, ножка стула): ставим препятствие
    // у кромки корпуса — планировщик обведёт его после отката.
    if (!bumper.any() || !have_pose_) return;
    const double side = bumper.left == bumper.right ? 0.0 : (bumper.left ? 0.5 : -0.5);
    const double a = last_pose_.yaw + side;
    const double r = robotConfig().radius_m + 0.03;
    mapping_->MarkObstacle(last_pose_.x + r * std::cos(a), last_pose_.y + r * std::sin(a));
}

void Slam::onOdometry(const types::Odometry& odom)
{
    // Курс от гироскопа, привязанный к начальному курсу одометрии: колёса на ковре
    // проскальзывают, гироскоп — нет. x/y пока из колёс; карта и поза строятся из одного
    // и того же скорректированного состояния, чтобы не расходиться.
    types::Odometry fused = odom;
    if (use_imu_yaw_ && have_imu_)
    {
        if (!yaw_anchored_)
        {
            yaw_anchor_ = NormalizeAngle(odom.yaw - last_imu_.yaw);
            yaw_anchored_ = true;
        }
        fused.yaw = NormalizeAngle(last_imu_.yaw + yaw_anchor_);
        fused.omega = last_imu_.vel_yaw;
    }
    mapping_->UpdateOdom(fused);

    types::Pose2D pose;
    pose.timestamp = odom.timestamp != 0 ? odom.timestamp : now();
    pose.x = fused.px;
    pose.y = fused.py;
    pose.yaw = fused.yaw;
    pose.v = std::hypot(fused.vx, fused.vy) * (fused.vx < 0 ? -1.0 : 1.0);
    pose.w = fused.omega;
    last_pose_ = pose;
    have_pose_ = true;
    publish<types::Pose2D>(topics::kSlamPose, pose);
}

void Slam::mapTick()
{
    types::OccupancyMap map = mapping_->UpdateMap();
    map.timestamp = now();
    publish<types::OccupancyMap>(topics::kSlamMap, map);
}

}  // namespace services
}  // namespace project
