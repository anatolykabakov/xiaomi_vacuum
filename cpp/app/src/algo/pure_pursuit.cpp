#include "algo/pure_pursuit.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace project
{
namespace algo
{

PurePursuit::PurePursuit(PurePursuitConfig config) : config_(config) {}

PurePursuitOutput PurePursuit::Step(const types::Trajectory& trajectory, const types::Pose2D& pose) const
{
    PurePursuitOutput out;
    const auto& pts = trajectory.points;
    if (pts.empty())
    {
        out.done = true;
        return out;
    }

    // ближайшая точка
    std::size_t nearest = 0;
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        const double d = std::hypot(pts[i].x - pose.x, pts[i].y - pose.y);
        if (d < best)
        {
            best = d;
            nearest = i;
        }
    }
    out.nearest_index = nearest;
    out.distance_to_goal = std::hypot(pts.back().x - pose.x, pts.back().y - pose.y);
    if (out.distance_to_goal <= config_.goal_tolerance_m)
    {
        out.done = true;
        return out;
    }

    // точка прицела: первая после ближайшей на расстоянии lookahead от робота
    // lookahead растёт со скоростью, но в пределах [min_lookahead, lookahead]
    const double ld = std::clamp(
        config_.lookahead_gain_s * std::fabs(pose.v), config_.min_lookahead_m, config_.lookahead_m
    );
    std::size_t target = pts.size() - 1;
    for (std::size_t i = nearest; i < pts.size(); ++i)
    {
        if (std::hypot(pts[i].x - pose.x, pts[i].y - pose.y) >= ld)
        {
            target = i;
            break;
        }
    }

    // цель в кадре робота
    const double dx = pts[target].x - pose.x;
    const double dy = pts[target].y - pose.y;
    const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
    const double lx = c * dx + s * dy;
    const double ly = -s * dx + c * dy;
    const double dist = std::hypot(lx, ly);
    const double heading_err = std::atan2(ly, lx);

    // цель сильно сбоку или сзади — развернуться на месте
    if (std::fabs(heading_err) > config_.turn_in_place_angle_rad)
    {
        out.v = 0.0;
        out.w = (heading_err > 0 ? 1.0 : -1.0) * std::min(config_.max_w_rps, 0.6);
        return out;
    }

    const double kappa = (dist > 1e-6) ? 2.0 * ly / (dist * dist) : 0.0;
    double v = std::min(config_.max_v_mps, pts[target].v > 0 ? pts[target].v : config_.max_v_mps);
    // притормаживаем к цели и при большой кривизне
    v = std::min(v, std::max(0.05, out.distance_to_goal));
    if (std::fabs(kappa) > 1e-6) v = std::min(v, config_.max_w_rps / std::fabs(kappa));
    double w = v * kappa;
    w = std::clamp(w, -config_.max_w_rps, config_.max_w_rps);

    out.v = v;
    out.w = w;
    return out;
}

}  // namespace algo
}  // namespace project
