#include "services/planner.h"

#include <cmath>
#include <string>

#include "mw/utils/logger.h"
#include "robot_config.h"
#include "utils/topics.h"

namespace project
{
namespace services
{

Planner::Planner()
{
    const RobotConfig& robot = robotConfig();
    config_.inflation_radius_m = robot.radius_m + 0.03;
    config_.robot_radius_m = robot.radius_m;
    config_.max_v_mps = robot.plan_v_mps;
    config_.max_w_rps = robot.plan_w_rps;
    config_.max_accel_mps2 = robot.max_accel_mps2;
}

void Planner::configure()
{
    registerParameters();

    subscribe<types::OccupancyMap>(topics::kSlamMap, [this](const types::OccupancyMap& m) { onMap(m); });
    subscribe<types::Pose2D>(topics::kSlamPose, [this](const types::Pose2D& p) { onPose(p); });
    subscribe<types::RobotState>(topics::kRobotState, [this](const types::RobotState& s) { onRobotState(s); });
    subscribe<types::Goal>(topics::kPlannerGoal, [this](const types::Goal& g) { onGoal(g); });
    subscribe<types::SafetyStop>(topics::kSafetyStop, [this](const types::SafetyStop& s) { onSafetyStop(s); });

    scheduleTimer(500, [this] { planTick(); }, "plan");
}

void Planner::registerParameters()
{
    registerParameter<double>("plan_inflation_radius_m", config_.inflation_radius_m);
    registerParameter<bool>("plan_through_unknown", config_.through_unknown);
    registerParameter<double>("plan_unknown_cost", config_.unknown_cost);
    registerParameter<double>("plan_max_v_mps", config_.max_v_mps);
    registerParameter<double>("plan_max_w_rps", config_.max_w_rps);
    registerParameter<double>("plan_goal_tolerance_m", config_.goal_tolerance_m);
    registerParameter<double>("plan_replan_distance_m", config_.replan_distance_m);
}

void Planner::onMap(const types::OccupancyMap& map)
{
    map_ = map;
    have_map_ = true;
}

void Planner::onPose(const types::Pose2D& pose)
{
    pose_ = pose;
    have_pose_ = true;
}

void Planner::onRobotState(const types::RobotState& state)
{
    robot_state_ = state;
}

void Planner::onSafetyStop(const types::SafetyStop& stop)
{
    // бампер: прежняя траектория вела в препятствие — после отката планируем заново по карте с ним
    if (stop.stop && stop.reason.find("bumper") != std::string::npos) have_trajectory_ = false;
}

void Planner::onGoal(const types::Goal& goal)
{
    if (goal.type == types::Goal::Type::kStop)
    {
        have_goal_ = false;
        have_trajectory_ = false;
        types::Trajectory empty;
        empty.timestamp = now();
        empty.goal_id = goal.id;
        publish<types::Trajectory>(topics::kPlannerTrajectory, empty);
        publishState(types::PlannerState::State::kIdle, "стоп по команде");
        return;
    }
    goal_ = goal;
    have_goal_ = true;
    have_trajectory_ = false;
    LOGI("planner: новая цель id=%u (%.2f, %.2f)", goal.id, goal.x, goal.y);
    planTick();
}

void Planner::planTick()
{
    using State = types::PlannerState::State;
    if (!have_goal_)
    {
        if (last_state_ != State::kIdle) publishState(State::kIdle, "");
        return;
    }
    if (!have_map_)
    {
        publishState(State::kNoMap, "нет карты");
        return;
    }
    if (!have_pose_)
    {
        publishState(State::kNoMap, "нет позы");
        return;
    }

    const double dist = std::hypot(goal_.x - pose_.x, goal_.y - pose_.y);
    if (dist <= config_.goal_tolerance_m)
    {
        have_goal_ = false;
        have_trajectory_ = false;
        types::Trajectory empty;
        empty.timestamp = now();
        empty.goal_id = goal_.id;
        publish<types::Trajectory>(topics::kPlannerTrajectory, empty);
        publishState(State::kDone, "цель достигнута");
        return;
    }

    algo::PathPlanner planner(config_);
    // прежняя траектория годится — едем по ней, не дёргая робота новым вариантом пути
    if (have_trajectory_ && trajectory_.goal_id == goal_.id && planner.Valid(map_, trajectory_, pose_))
    {
        trajectory_.timestamp = now();
        publish<types::Trajectory>(topics::kPlannerTrajectory, trajectory_);
        if (last_state_ != State::kFollowing) publishState(State::kFollowing, "по прежней траектории");
        return;
    }
    types::Trajectory trajectory;
    trajectory.timestamp = now();
    if (!planner.Plan(map_, pose_, goal_, trajectory))
    {
        have_trajectory_ = false;
        publishState(State::kBlocked, "пути по карте нет");
        return;
    }
    trajectory_ = trajectory;
    have_trajectory_ = true;
    publish<types::Trajectory>(topics::kPlannerTrajectory, trajectory);
    publishState(State::kFollowing, std::to_string(trajectory.points.size()) + " точек");
}

void Planner::publishState(types::PlannerState::State state, const std::string& detail)
{
    types::PlannerState s;
    s.timestamp = now();
    s.state = state;
    s.goal_id = goal_.id;
    s.distance_to_goal = have_pose_ ? std::hypot(goal_.x - pose_.x, goal_.y - pose_.y) : 0.0;
    s.detail = detail;
    publish<types::PlannerState>(topics::kPlannerState, s);
    last_state_ = state;
}

}  // namespace services
}  // namespace project
