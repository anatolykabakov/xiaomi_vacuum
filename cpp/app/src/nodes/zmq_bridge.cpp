#include "nodes/zmq_bridge.h"

#include "cmd_vel.pb.h"
#include "map.pb.h"
#include "zmq_message.pb.h"

#include <sstream>
#include <string>

namespace project
{
namespace nodes
{

namespace
{

bool ToProtoOccupancy(const types::OccupancyMap& src, xiaomi::robot::OccupancyMap* dst)
{
    if (!dst)
    {
        return false;
    }
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
    return true;
}

/** Listen on all interfaces so a phone / PC on the LAN can connect (Player still uses config host).
 */
std::string makeBindAllInterfaces(int port)
{
    return std::string("tcp://*:" + std::to_string(port));
}

}  // namespace

ZmqBridge::ZmqBridge(const app::RobotApp::Config& config) : config_(config) {}

void ZmqBridge::configure()
{
    context_.reset(new zmq::context_t(1));
    publisher_.reset(new zmq::socket_t(*context_, zmq::socket_type::pub));
    publisher_->bind(makeBindAllInterfaces(config_.zmq_pub_port));

    subscriber_.reset(new zmq::socket_t(*context_, zmq::socket_type::sub));
    subscriber_->bind(makeBindAllInterfaces(config_.zmq_sub_port));
    subscriber_->set(zmq::sockopt::subscribe, "");
    subscriber_->set(zmq::sockopt::rcvtimeo, 10);

    subscribe<types::OccupancyMap>(
        "occupancy_map",
        [this](const types::OccupancyMap& map) { OnOccupancyMap(map); }
    );

    scheduleTimer(2, [this]() { OnReceiveTimer(); });
}

void ZmqBridge::reset()
{
    // publisher_.reset();
    // subscriber_.reset();
    // context_.reset();
}

void ZmqBridge::OnReceiveTimer()
{
    if (!subscriber_) return;

    zmq::message_t msg;
    if (!subscriber_->recv(msg, zmq::recv_flags::dontwait))
    {
        return;
    }

    xiaomi::robot::ZmqMessage zmq_msg;
    if (!zmq_msg.ParseFromArray(msg.data(), msg.size()))
    {
        return;
    }
    if (zmq_msg.has_cmd_vel())
    {
        const xiaomi::robot::CmdVel& proto = zmq_msg.cmd_vel();
        types::CmdVel cmd{};
        cmd.timestamp = zmq_msg.timestamp();
        cmd.vx = proto.vx();
        cmd.vy = proto.vy();
        cmd.w = proto.w();
        publish<types::CmdVel>(zmq_msg.topic(), cmd);
    }
}

void ZmqBridge::OnOccupancyMap(const types::OccupancyMap& map)
{
    xiaomi::robot::OccupancyMap proto_map;
    if (!ToProtoOccupancy(map, &proto_map))
    {
        return;
    }
    xiaomi::robot::ZmqMessage msg;
    msg.set_timestamp(now());
    msg.set_topic("occupancy_map");
    *msg.mutable_occupancy_map() = proto_map;

    std::string wire;
    if (!msg.SerializeToString(&wire))
    {
        return;
    }
    zmq::message_t zmq_msg(wire.data(), wire.size());
    const auto sent = publisher_->send(zmq_msg, zmq::send_flags::dontwait);
    if (!sent)
    {
        LOGE("ZmqBridge: map PUB send dropped (EAGAIN?)");
    }
}

}  // namespace nodes
}  // namespace project
