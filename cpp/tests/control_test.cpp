#include "algo/pure_pursuit.h"

#include <cmath>

#include "gtest/gtest.h"

namespace project
{
namespace
{

// прямая траектория вдоль +x длиной len, шаг 0.1 м, скорость v
types::Trajectory StraightX(double len, double v)
{
    types::Trajectory t;
    for (double x = 0.0; x <= len + 1e-9; x += 0.1)
    {
        types::TrajectoryPoint p;
        p.x = x;
        p.v = v;
        p.t = x / v;
        t.points.push_back(p);
    }
    return t;
}

TEST(PurePursuitTest, EmptyTrajectoryIsDone)
{
    algo::PurePursuit pp;
    const auto out = pp.Step(types::Trajectory{}, types::Pose2D{});
    EXPECT_TRUE(out.done);
    EXPECT_DOUBLE_EQ(out.v, 0.0);
    EXPECT_DOUBLE_EQ(out.w, 0.0);
}

TEST(PurePursuitTest, OnLineDrivesStraight)
{
    algo::PurePursuit pp;
    const auto traj = StraightX(2.0, 0.2);
    types::Pose2D pose;  // в начале, курс вдоль x
    const auto out = pp.Step(traj, pose);
    EXPECT_FALSE(out.done);
    EXPECT_GT(out.v, 0.0);
    EXPECT_NEAR(out.w, 0.0, 1e-6);
}

TEST(PurePursuitTest, OffsetLeftTurnsLeft)
{
    algo::PurePursuit pp;
    const auto traj = StraightX(2.0, 0.2);
    types::Pose2D pose;
    pose.y = -0.1;  // робот правее линии → поворачивать влево (w > 0)
    const auto out = pp.Step(traj, pose);
    EXPECT_FALSE(out.done);
    EXPECT_GT(out.w, 0.0);
    pose.y = +0.1;
    EXPECT_LT(pp.Step(traj, pose).w, 0.0);
}

TEST(PurePursuitTest, FacingBackwardsTurnsInPlace)
{
    algo::PurePursuit pp;
    const auto traj = StraightX(2.0, 0.2);
    types::Pose2D pose;
    pose.yaw = M_PI;  // смотрит назад
    const auto out = pp.Step(traj, pose);
    EXPECT_DOUBLE_EQ(out.v, 0.0);
    EXPECT_NE(out.w, 0.0);
}

TEST(PurePursuitTest, DoneWithinGoalTolerance)
{
    algo::PurePursuitConfig cfg;
    cfg.goal_tolerance_m = 0.1;
    algo::PurePursuit pp(cfg);
    const auto traj = StraightX(1.0, 0.2);
    types::Pose2D pose;
    pose.x = 0.95;
    const auto out = pp.Step(traj, pose);
    EXPECT_TRUE(out.done);
    EXPECT_DOUBLE_EQ(out.v, 0.0);
}

TEST(PurePursuitTest, RespectsLimits)
{
    algo::PurePursuitConfig cfg;
    cfg.max_v_mps = 0.15;
    cfg.max_w_rps = 0.5;
    algo::PurePursuit pp(cfg);
    const auto traj = StraightX(2.0, 1.0);  // траектория просит 1.0 м/с
    types::Pose2D pose;
    pose.y = -0.3;
    const auto out = pp.Step(traj, pose);
    EXPECT_LE(out.v, cfg.max_v_mps + 1e-9);
    EXPECT_LE(std::fabs(out.w), cfg.max_w_rps + 1e-9);
}

}  // namespace
}  // namespace project
