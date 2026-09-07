#pragma once

#include <memory>
#include <string_view>

#include "mw/middleware/manager.hpp"
#include "robot_app.h"
#include "types.h"

#include <zmq.hpp>

namespace project
{
namespace services
{

/**
 * Граница с внешним миром (телефон, ПК). Единственное место
 * с protobuf: внутри шины — структуры types::*.
 *
 *  PUB tcp://*:zmq_pub_port  → slam/map, slam/pose, robot/state, planner/trajectory, planner/state
 *  SUB tcp://*:zmq_sub_port  ← ZmqMessage{cmd_vel} → controls/cmd_vel (телеоперация в обход планировщика)
 *                            ← ZmqMessage{goal}    → planner/goal
 * SUB опрашивается таймером каждые 2 мс.
 */
class ZmqBridge : public mw::middleware::Service
{
public:
    explicit ZmqBridge(const app::RobotApp::Config& config);
    ~ZmqBridge() override = default;

    std::string_view getName() const override { return "zmq_bridge"; }

protected:
    void configure() override;
    void reset() override;

private:
    void onReceiveTimer();
    void onMap(const types::OccupancyMap& map);
    void onPose(const types::Pose2D& pose);
    void onRobotState(const types::RobotState& state);
    void onTrajectory(const types::Trajectory& trajectory);
    void onPlannerState(const types::PlannerState& state);
    void onMissionState(const types::MissionState& state);
    void send(const std::string& wire);

    std::unique_ptr<zmq::context_t> context_;
    std::unique_ptr<zmq::socket_t> publisher_;
    std::unique_ptr<zmq::socket_t> subscriber_;
    app::RobotApp::Config config_;
};

}  // namespace services
}  // namespace project
