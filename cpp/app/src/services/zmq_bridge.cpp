#include "services/zmq_bridge.h"

#include <cmath>
#include <limits>
#include <string>

#include "mw/utils/logger.h"
#include "cmd_vel.pb.h"
#include "map.pb.h"
#include "robot.pb.h"
#include "utils/topics.h"
#include "zmq_message.pb.h"

namespace project
{
namespace services
{

namespace
{

std::string BindAll(int port)
{
    return "tcp://*:" + std::to_string(port);
}

void ToProto(const types::OccupancyMap& src, xiaomi::robot::OccupancyMap* dst)
{
    dst->Clear();
    dst->set_timestamp(src.timestamp);
    dst->set_width(src.width);
    dst->set_height(src.height);
    dst->set_resolution(src.resolution);
    dst->set_origin_x(src.origin_x);
    dst->set_origin_y(src.origin_y);
    dst->set_robot_x(src.robot_x);
    dst->set_robot_y(src.robot_y);
    dst->set_robot_yaw(src.robot_yaw);
    if (!src.cells.empty())
    {
        dst->set_cells(reinterpret_cast<const char*>(src.cells.data()), src.cells.size());
    }
}

void ToProto(const types::Pose2D& src, xiaomi::robot::Pose2D* dst)
{
    dst->set_timestamp(src.timestamp);
    dst->set_x(src.x);
    dst->set_y(src.y);
    dst->set_yaw(src.yaw);
    dst->set_v(src.v);
    dst->set_w(src.w);
}

void ToProto(const types::RobotState& src, xiaomi::robot::RobotState* dst)
{
    dst->set_timestamp(src.timestamp);
    dst->set_battery_percent(src.battery_percent);
    dst->set_charging(src.charging);
    dst->set_docked(src.docked);
    dst->set_bumper(src.bumper);
    dst->set_cliff(src.cliff);
    dst->set_wall_distance(src.wall_distance);
    dst->set_mode(src.mode);
    dst->set_error(src.error);
}

void ToProto(const types::Trajectory& src, xiaomi::robot::Trajectory* dst)
{
    dst->set_timestamp(src.timestamp);
    dst->set_goal_id(src.goal_id);
    for (const auto& p : src.points)
    {
        auto* q = dst->add_points();
        q->set_x(p.x);
        q->set_y(p.y);
        q->set_yaw(p.yaw);
        q->set_v(p.v);
        q->set_w(p.w);
        q->set_t(p.t);
    }
}

void ToProto(const types::PlannerState& src, xiaomi::robot::PlannerState* dst)
{
    dst->set_timestamp(src.timestamp);
    dst->set_state(static_cast<xiaomi::robot::PlannerState_State>(static_cast<int>(src.state)));
    dst->set_goal_id(src.goal_id);
    dst->set_distance_to_goal(src.distance_to_goal);
    dst->set_detail(src.detail);
}

void ToProto(const types::MissionState& src, xiaomi::robot::MissionState* dst)
{
    dst->set_timestamp(src.timestamp);
    dst->set_state(static_cast<xiaomi::robot::MissionState_State>(static_cast<int>(src.state)));
    dst->set_frontiers_left(src.frontiers_left);
    dst->set_waypoints_total(src.waypoints_total);
    dst->set_waypoints_done(src.waypoints_done);
    dst->set_coverage_ratio(src.coverage_ratio);
    dst->set_detail(src.detail);
}

}  // namespace

ZmqBridge::ZmqBridge(const app::RobotApp::Config& config) : config_(config) {}

void ZmqBridge::configure()
{
    context_.reset(new zmq::context_t(1));
    publisher_.reset(new zmq::socket_t(*context_, zmq::socket_type::pub));
    publisher_->bind(BindAll(config_.zmq_pub_port));

    subscriber_.reset(new zmq::socket_t(*context_, zmq::socket_type::sub));
    subscriber_->bind(BindAll(config_.zmq_sub_port));
    subscriber_->set(zmq::sockopt::subscribe, "");
    subscriber_->set(zmq::sockopt::rcvtimeo, 10);

    subscribe<types::OccupancyMap>(topics::kSlamMap, [this](const types::OccupancyMap& m) { onMap(m); });
    subscribe<types::Pose2D>(topics::kSlamPose, [this](const types::Pose2D& p) { onPose(p); });
    subscribe<types::RobotState>(topics::kRobotState, [this](const types::RobotState& s) { onRobotState(s); });
    subscribe<types::Trajectory>(
        topics::kPlannerTrajectory, [this](const types::Trajectory& t) { onTrajectory(t); }
    );
    subscribe<types::PlannerState>(
        topics::kPlannerState, [this](const types::PlannerState& s) { onPlannerState(s); }
    );
    subscribe<types::MissionState>(
        topics::kMissionState, [this](const types::MissionState& s) { onMissionState(s); }
    );

    scheduleTimer(2, [this] { onReceiveTimer(); }, "recv");
}

void ZmqBridge::reset() {}

void ZmqBridge::send(const std::string& wire)
{
    zmq::message_t msg(wire.data(), wire.size());
    if (!publisher_->send(msg, zmq::send_flags::dontwait))
    {
        LOGW("zmq_bridge: PUB send dropped (EAGAIN?)");
    }
}

void ZmqBridge::onReceiveTimer()
{
    if (!subscriber_) return;
    zmq::message_t msg;
    if (!subscriber_->recv(msg, zmq::recv_flags::dontwait)) return;

    xiaomi::robot::ZmqMessage in;
    if (!in.ParseFromArray(msg.data(), static_cast<int>(msg.size())))
    {
        LOGW("zmq_bridge: не распарсилось %zu байт", msg.size());
        return;
    }
    if (in.has_cmd_vel())
    {
        // телеоперация в обход планировщика; побеждает последняя команда в Platform
        types::CmdVel cmd;
        cmd.timestamp = now();
        cmd.vx = in.cmd_vel().vx();
        cmd.vy = in.cmd_vel().vy();
        cmd.w = in.cmd_vel().w();
        publish<types::CmdVel>(topics::kCmdVel, cmd);
    }
    if (in.has_mission_command())
    {
        types::MissionCommand cmd;
        cmd.timestamp = now();
        switch (in.mission_command().command())
        {
            case xiaomi::robot::MissionCommand::STOP: cmd.command = types::MissionCommand::Command::kStop; break;
            case xiaomi::robot::MissionCommand::RETURN: cmd.command = types::MissionCommand::Command::kReturn; break;
            default: cmd.command = types::MissionCommand::Command::kStart; break;
        }
        publish<types::MissionCommand>(topics::kMissionCommand, cmd);
    }
    if (in.has_goal())
    {
        const auto& g = in.goal();
        types::Goal goal;
        goal.timestamp = now();
        switch (g.type())
        {
            case xiaomi::robot::Goal::DOCK: goal.type = types::Goal::Type::kDock; break;
            case xiaomi::robot::Goal::STOP: goal.type = types::Goal::Type::kStop; break;
            default: goal.type = types::Goal::Type::kPoint; break;
        }
        goal.x = g.x();
        goal.y = g.y();
        goal.yaw = g.has_yaw() ? g.yaw() : std::numeric_limits<double>::quiet_NaN();
        goal.id = g.id();
        publish<types::Goal>(topics::kPlannerGoal, goal);
    }
}

void ZmqBridge::onMap(const types::OccupancyMap& map)
{
    xiaomi::robot::ZmqMessage out;
    out.set_timestamp(now());
    out.set_topic(topics::kSlamMap);
    ToProto(map, out.mutable_occupancy_map());
    std::string wire;
    if (out.SerializeToString(&wire)) send(wire);
}

void ZmqBridge::onPose(const types::Pose2D& pose)
{
    xiaomi::robot::ZmqMessage out;
    out.set_timestamp(now());
    out.set_topic(topics::kSlamPose);
    ToProto(pose, out.mutable_pose());
    std::string wire;
    if (out.SerializeToString(&wire)) send(wire);
}

void ZmqBridge::onRobotState(const types::RobotState& state)
{
    xiaomi::robot::ZmqMessage out;
    out.set_timestamp(now());
    out.set_topic(topics::kRobotState);
    ToProto(state, out.mutable_robot_state());
    std::string wire;
    if (out.SerializeToString(&wire)) send(wire);
}

void ZmqBridge::onTrajectory(const types::Trajectory& trajectory)
{
    xiaomi::robot::ZmqMessage out;
    out.set_timestamp(now());
    out.set_topic(topics::kPlannerTrajectory);
    ToProto(trajectory, out.mutable_trajectory());
    std::string wire;
    if (out.SerializeToString(&wire)) send(wire);
}

void ZmqBridge::onPlannerState(const types::PlannerState& state)
{
    xiaomi::robot::ZmqMessage out;
    out.set_timestamp(now());
    out.set_topic(topics::kPlannerState);
    ToProto(state, out.mutable_planner_state());
    std::string wire;
    if (out.SerializeToString(&wire)) send(wire);
}

void ZmqBridge::onMissionState(const types::MissionState& state)
{
    xiaomi::robot::ZmqMessage out;
    out.set_timestamp(now());
    out.set_topic(topics::kMissionState);
    ToProto(state, out.mutable_mission_state());
    std::string wire;
    if (out.SerializeToString(&wire)) send(wire);
}

}  // namespace services
}  // namespace project
