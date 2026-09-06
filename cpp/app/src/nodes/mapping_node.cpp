#include "nodes/mapping_node.h"

namespace project
{
namespace nodes
{

MappingNode::MappingNode()
{
    occupancy_mapping_.reset(new algo::OccupancyMapping());
}

void MappingNode::configure()
{
    subscribe<types::LaserData>(
        "laser_data",
        [this](const types::LaserData& laser_data) { OnLaserData(laser_data); }
    );
    subscribe<types::OdometryData>(
        "odometry_data",
        [this](const types::OdometryData& odometry_data) { OnOdometryData(odometry_data); }
    );

    scheduleTimer(1000, [this]() { OnTimer(); });
}

void MappingNode::OnLaserData(const types::LaserData& laser_data)
{
    occupancy_mapping_->UpdateScan(laser_data);
}

void MappingNode::OnOdometryData(const types::OdometryData& odometry_data)
{
    occupancy_mapping_->UpdateOdom(odometry_data);
}

void MappingNode::OnTimer()
{
    types::OccupancyMap occupancy_map = occupancy_mapping_->UpdateMap();
    occupancy_map.timestamp = now();
    publish<types::OccupancyMap>("occupancy_map", occupancy_map);
}
}  // namespace nodes
}  // namespace project
