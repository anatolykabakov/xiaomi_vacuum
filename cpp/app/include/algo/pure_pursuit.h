#pragma once

#include <cstddef>

#include "types.h"

namespace project
{
namespace algo
{

struct PurePursuitConfig
{
    double lookahead_m{0.30};
    double min_lookahead_m{0.15};
    double lookahead_gain_s{0.5};  // lookahead = max(min, gain * v)
    double max_v_mps{0.25};
    double max_w_rps{1.0};
    double goal_tolerance_m{0.10};
    double turn_in_place_angle_rad{1.2};  // цель сильно позади/сбоку — сначала развернуться
};

struct PurePursuitOutput
{
    double v{0};
    double w{0};
    bool done{false};
    std::size_t nearest_index{0};
    double distance_to_goal{0};
};

/** Ведение по траектории для дифференциального привода. Чистый алгоритм. */
class PurePursuit
{
public:
    explicit PurePursuit(PurePursuitConfig config = {});

    PurePursuitOutput Step(const types::Trajectory& trajectory, const types::Pose2D& pose) const;

    const PurePursuitConfig& config() const { return config_; }

private:
    PurePursuitConfig config_;
};

}  // namespace algo
}  // namespace project
