#pragma once

#include <memory>

#include "algo/occupancy_mapping.h"
#include "adas/middleware/manager.hpp"
#include "types.h"

namespace project
{
namespace nodes
{

class MappingNode : public adas::middleware::Service
{
public:
    MappingNode();
    ~MappingNode() = default;

protected:
    std::string_view getName() const override { return "mapping"; }
    void configure() override;

private:
    std::unique_ptr<algo::OccupancyMapping> occupancy_mapping_;

    void OnLaserData(const types::LaserData& laser_data);
    void OnOdometryData(const types::OdometryData& odometry_data);
    void OnTimer();
};
}  // namespace nodes
}  // namespace project
