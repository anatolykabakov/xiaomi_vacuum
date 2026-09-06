#include "algo/cleaning_mission.h"
#include "algo/coverage_planner.h"
#include "algo/frontier_explorer.h"
#include "algo/occupancy_mapping.h"
#include "algo/path_planner.h"
#include "algo/pure_pursuit.h"
#include "robot_config.h"
#include "types.h"

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
namespace types = project::types;
namespace algo = project::algo;

namespace
{

py::array_t<uint8_t> CellsAsArray(const types::OccupancyMap& map)
{
    py::array_t<uint8_t> out({static_cast<py::ssize_t>(map.height), static_cast<py::ssize_t>(map.width)});
    auto buf = out.mutable_unchecked<2>();
    for (py::ssize_t y = 0; y < buf.shape(0); ++y)
        for (py::ssize_t x = 0; x < buf.shape(1); ++x)
        {
            const size_t i = static_cast<size_t>(x) + static_cast<size_t>(y) * map.width;
            buf(y, x) = i < map.cells.size() ? map.cells[i] : 127;
        }
    return out;
}

void CellsFromArray(types::OccupancyMap& map, py::array_t<uint8_t, py::array::c_style | py::array::forcecast> arr)
{
    if (arr.ndim() != 2) throw std::invalid_argument("cells: ожидается массив (height, width)");
    map.height = static_cast<uint32_t>(arr.shape(0));
    map.width = static_cast<uint32_t>(arr.shape(1));
    map.cells.assign(arr.data(), arr.data() + arr.size());
}

py::array_t<uint8_t> GridAsArray(const std::vector<uint8_t>& grid, const types::OccupancyMap& map)
{
    py::array_t<uint8_t> out({static_cast<py::ssize_t>(map.height), static_cast<py::ssize_t>(map.width)});
    std::copy(grid.begin(), grid.end(), out.mutable_data());
    return out;
}

}  // namespace

PYBIND11_MODULE(pyxiaomi_robot, m)
{
    m.doc() = "Алгоритмы робота-уборщика: картирование, исследование, покрытие, планирование, ведение";

    // ------------------------------------------------------------------ types
    auto t = m.def_submodule("types", "Сообщения внутренней шины");

    py::class_<types::Odometry>(t, "Odometry")
        .def(py::init<>())
        .def_readwrite("timestamp", &types::Odometry::timestamp)
        .def_readwrite("px", &types::Odometry::px)
        .def_readwrite("py", &types::Odometry::py)
        .def_readwrite("vx", &types::Odometry::vx)
        .def_readwrite("vy", &types::Odometry::vy)
        .def_readwrite("yaw", &types::Odometry::yaw)
        .def_readwrite("omega", &types::Odometry::omega);

    py::class_<types::LaserScan>(t, "LaserScan")
        .def(py::init<>())
        .def_readwrite("timestamp", &types::LaserScan::timestamp)
        .def_readwrite("scan_start", &types::LaserScan::scan_start)
        .def_readwrite("scan_resolution", &types::LaserScan::scan_resolution)
        .def_readwrite("ranges", &types::LaserScan::ranges);

    py::class_<types::Imu>(t, "Imu")
        .def(py::init<>())
        .def_readwrite("timestamp", &types::Imu::timestamp)
        .def_readwrite("roll", &types::Imu::roll)
        .def_readwrite("pitch", &types::Imu::pitch)
        .def_readwrite("yaw", &types::Imu::yaw)
        .def_readwrite("vel_roll", &types::Imu::vel_roll)
        .def_readwrite("vel_pitch", &types::Imu::vel_pitch)
        .def_readwrite("vel_yaw", &types::Imu::vel_yaw);

    py::class_<types::Pose2D>(t, "Pose2D")
        .def(py::init<>())
        .def(py::init([](double x, double y, double yaw) {
                 types::Pose2D p;
                 p.x = x;
                 p.y = y;
                 p.yaw = yaw;
                 return p;
             }),
             py::arg("x"), py::arg("y"), py::arg("yaw") = 0.0)
        .def_readwrite("timestamp", &types::Pose2D::timestamp)
        .def_readwrite("x", &types::Pose2D::x)
        .def_readwrite("y", &types::Pose2D::y)
        .def_readwrite("yaw", &types::Pose2D::yaw)
        .def_readwrite("v", &types::Pose2D::v)
        .def_readwrite("w", &types::Pose2D::w)
        .def("__repr__", [](const types::Pose2D& p) {
            return "Pose2D(x=" + std::to_string(p.x) + ", y=" + std::to_string(p.y) + ", yaw=" + std::to_string(p.yaw) + ")";
        });

    py::class_<types::OccupancyMap>(t, "OccupancyMap")
        .def(py::init<>())
        .def_readwrite("timestamp", &types::OccupancyMap::timestamp)
        .def_readwrite("width", &types::OccupancyMap::width)
        .def_readwrite("height", &types::OccupancyMap::height)
        .def_readwrite("resolution", &types::OccupancyMap::resolution)
        .def_readwrite("origin_x", &types::OccupancyMap::origin_x)
        .def_readwrite("origin_y", &types::OccupancyMap::origin_y)
        .def_readwrite("robot_x", &types::OccupancyMap::robot_x)
        .def_readwrite("robot_y", &types::OccupancyMap::robot_y)
        .def_readwrite("robot_yaw", &types::OccupancyMap::robot_yaw)
        .def_property(
            "cells", &CellsAsArray, &CellsFromArray,
            "Ячейки как numpy uint8 (height, width): 0 занято, 127 неизвестно, 255 свободно"
        );

    py::class_<types::Goal> goal(t, "Goal");
    py::enum_<types::Goal::Type>(goal, "Type")
        .value("POINT", types::Goal::Type::kPoint)
        .value("DOCK", types::Goal::Type::kDock)
        .value("STOP", types::Goal::Type::kStop);
    goal.def(py::init<>())
        .def(py::init([](double x, double y) {
                 types::Goal g;
                 g.x = x;
                 g.y = y;
                 return g;
             }),
             py::arg("x"), py::arg("y"))
        .def_readwrite("timestamp", &types::Goal::timestamp)
        .def_readwrite("type", &types::Goal::type)
        .def_readwrite("x", &types::Goal::x)
        .def_readwrite("y", &types::Goal::y)
        .def_readwrite("yaw", &types::Goal::yaw)
        .def_readwrite("id", &types::Goal::id)
        .def("__repr__", [](const types::Goal& g) {
            return "Goal(id=" + std::to_string(g.id) + ", x=" + std::to_string(g.x) + ", y=" + std::to_string(g.y) + ")";
        });

    py::class_<types::TrajectoryPoint>(t, "TrajectoryPoint")
        .def(py::init<>())
        .def_readwrite("x", &types::TrajectoryPoint::x)
        .def_readwrite("y", &types::TrajectoryPoint::y)
        .def_readwrite("yaw", &types::TrajectoryPoint::yaw)
        .def_readwrite("v", &types::TrajectoryPoint::v)
        .def_readwrite("w", &types::TrajectoryPoint::w)
        .def_readwrite("t", &types::TrajectoryPoint::t);

    py::class_<types::Trajectory>(t, "Trajectory")
        .def(py::init<>())
        .def_readwrite("timestamp", &types::Trajectory::timestamp)
        .def_readwrite("goal_id", &types::Trajectory::goal_id)
        .def_readwrite("points", &types::Trajectory::points)
        .def("xy", [](const types::Trajectory& tr) {
            std::vector<std::pair<double, double>> out;
            for (const auto& p : tr.points) out.emplace_back(p.x, p.y);
            return out;
        });

    py::class_<types::PlannerState> ps(t, "PlannerState");
    py::enum_<types::PlannerState::State>(ps, "State")
        .value("IDLE", types::PlannerState::State::kIdle)
        .value("NO_MAP", types::PlannerState::State::kNoMap)
        .value("PLANNING", types::PlannerState::State::kPlanning)
        .value("FOLLOWING", types::PlannerState::State::kFollowing)
        .value("BLOCKED", types::PlannerState::State::kBlocked)
        .value("DONE", types::PlannerState::State::kDone);
    ps.def(py::init<>())
        .def(py::init([](types::PlannerState::State s, uint32_t goal_id) {
                 types::PlannerState st;
                 st.state = s;
                 st.goal_id = goal_id;
                 return st;
             }),
             py::arg("state"), py::arg("goal_id") = 0)
        .def_readwrite("timestamp", &types::PlannerState::timestamp)
        .def_readwrite("state", &types::PlannerState::state)
        .def_readwrite("goal_id", &types::PlannerState::goal_id)
        .def_readwrite("distance_to_goal", &types::PlannerState::distance_to_goal)
        .def_readwrite("detail", &types::PlannerState::detail);

    py::class_<types::MissionState> ms(t, "MissionState");
    py::enum_<types::MissionState::State>(ms, "State")
        .value("IDLE", types::MissionState::State::kIdle)
        .value("EXPLORING", types::MissionState::State::kExploring)
        .value("COVERING", types::MissionState::State::kCovering)
        .value("RETURNING", types::MissionState::State::kReturning)
        .value("DONE", types::MissionState::State::kDone);
    ms.def(py::init<>())
        .def_readwrite("timestamp", &types::MissionState::timestamp)
        .def_readwrite("state", &types::MissionState::state)
        .def_readwrite("frontiers_left", &types::MissionState::frontiers_left)
        .def_readwrite("waypoints_total", &types::MissionState::waypoints_total)
        .def_readwrite("waypoints_done", &types::MissionState::waypoints_done)
        .def_readwrite("coverage_ratio", &types::MissionState::coverage_ratio)
        .def_readwrite("detail", &types::MissionState::detail);

    py::class_<types::CmdVel>(t, "CmdVel")
        .def(py::init<>())
        .def_readwrite("timestamp", &types::CmdVel::timestamp)
        .def_readwrite("vx", &types::CmdVel::vx)
        .def_readwrite("vy", &types::CmdVel::vy)
        .def_readwrite("w", &types::CmdVel::w);

    // ------------------------------------------------------------------ robot config
    py::class_<project::RobotConfig>(m, "RobotConfig")
        .def_readonly("radius_m", &project::RobotConfig::radius_m)
        .def_readonly("wheel_base_m", &project::RobotConfig::wheel_base_m)
        .def_readonly("max_v_mps", &project::RobotConfig::max_v_mps)
        .def_readonly("max_w_rps", &project::RobotConfig::max_w_rps)
        .def_readonly("max_accel_mps2", &project::RobotConfig::max_accel_mps2)
        .def_readonly("plan_v_mps", &project::RobotConfig::plan_v_mps)
        .def_readonly("plan_w_rps", &project::RobotConfig::plan_w_rps)
        .def_readonly("lidar_offset_x_m", &project::RobotConfig::lidar_offset_x_m)
        .def_readonly("lidar_offset_y_m", &project::RobotConfig::lidar_offset_y_m)
        .def_readonly("lidar_offset_yaw_rad", &project::RobotConfig::lidar_offset_yaw_rad);
    m.def("robot_config", &project::robotConfig, py::return_value_policy::reference, "Геометрия и лимиты робота");

    // ------------------------------------------------------------------ algo
    auto a = m.def_submodule("algo", "Алгоритмы");

    py::class_<algo::OccupancyMapping>(a, "OccupancyMapping")
        .def(py::init<>())
        .def("update_scan", &algo::OccupancyMapping::UpdateScan, py::arg("scan"))
        .def("update_odom", &algo::OccupancyMapping::UpdateOdom, py::arg("odom"))
        .def("update_map", &algo::OccupancyMapping::UpdateMap, "Вписать последний скан в карту и вернуть её")
        .def("mark_obstacle", &algo::OccupancyMapping::MarkObstacle, py::arg("x"), py::arg("y"),
             "Препятствие по бамперу: ячейка остаётся занятой, лучи её не стирают");

    py::class_<algo::PathPlannerConfig>(a, "PathPlannerConfig")
        .def(py::init<>())
        .def_readwrite("inflation_radius_m", &algo::PathPlannerConfig::inflation_radius_m)
        .def_readwrite("robot_radius_m", &algo::PathPlannerConfig::robot_radius_m)
        .def_readwrite("escape_cost", &algo::PathPlannerConfig::escape_cost)
        .def_readwrite("replan_distance_m", &algo::PathPlannerConfig::replan_distance_m)
        .def_readwrite("through_unknown", &algo::PathPlannerConfig::through_unknown)
        .def_readwrite("unknown_cost", &algo::PathPlannerConfig::unknown_cost)
        .def_readwrite("max_v_mps", &algo::PathPlannerConfig::max_v_mps)
        .def_readwrite("max_w_rps", &algo::PathPlannerConfig::max_w_rps)
        .def_readwrite("max_accel_mps2", &algo::PathPlannerConfig::max_accel_mps2)
        .def_readwrite("goal_tolerance_m", &algo::PathPlannerConfig::goal_tolerance_m)
        .def_readwrite("waypoint_step_m", &algo::PathPlannerConfig::waypoint_step_m)
        .def_readwrite("smoothing_iterations", &algo::PathPlannerConfig::smoothing_iterations);

    py::class_<algo::PathPlanner>(a, "PathPlanner")
        .def(py::init<algo::PathPlannerConfig>(), py::arg("config") = algo::PathPlannerConfig{})
        .def(
            "plan",
            [](const algo::PathPlanner& self, const types::OccupancyMap& map, const types::Pose2D& start,
               const types::Goal& goal) -> py::object {
                types::Trajectory out;
                if (!self.Plan(map, start, goal, out)) return py::none();
                return py::cast(out);
            },
            py::arg("map"), py::arg("start"), py::arg("goal"), "Траектория до цели или None, если пути нет"
        )
        .def("valid", &algo::PathPlanner::Valid, py::arg("map"), py::arg("trajectory"), py::arg("pose"),
             "Траектория ещё годится: робот рядом с ней и корпус не задевает занятых ячеек")
        .def(
            "inflate",
            [](const algo::PathPlanner& self, const types::OccupancyMap& map) { return GridAsArray(self.Inflate(map), map); },
            py::arg("map"), "Сетка (height, width): 0 препятствие с раздуванием, 1 свободно, 2 неизвестно"
        )
        .def_property_readonly("config", &algo::PathPlanner::config)
        .def_static(
            "world_to_grid",
            [](const types::OccupancyMap& map, double x, double y) -> py::object {
                int gx, gy;
                if (!algo::PathPlanner::WorldToGrid(map, x, y, gx, gy)) return py::none();
                return py::make_tuple(gx, gy);
            },
            py::arg("map"), py::arg("x"), py::arg("y")
        )
        .def_static(
            "grid_to_world",
            [](const types::OccupancyMap& map, int gx, int gy) {
                double x, y;
                algo::PathPlanner::GridToWorld(map, gx, gy, x, y);
                return py::make_tuple(x, y);
            },
            py::arg("map"), py::arg("gx"), py::arg("gy")
        );

    py::class_<algo::PurePursuitConfig>(a, "PurePursuitConfig")
        .def(py::init<>())
        .def_readwrite("lookahead_m", &algo::PurePursuitConfig::lookahead_m)
        .def_readwrite("min_lookahead_m", &algo::PurePursuitConfig::min_lookahead_m)
        .def_readwrite("lookahead_gain_s", &algo::PurePursuitConfig::lookahead_gain_s)
        .def_readwrite("max_v_mps", &algo::PurePursuitConfig::max_v_mps)
        .def_readwrite("max_w_rps", &algo::PurePursuitConfig::max_w_rps)
        .def_readwrite("goal_tolerance_m", &algo::PurePursuitConfig::goal_tolerance_m)
        .def_readwrite("turn_in_place_angle_rad", &algo::PurePursuitConfig::turn_in_place_angle_rad);

    py::class_<algo::PurePursuitOutput>(a, "PurePursuitOutput")
        .def_readonly("v", &algo::PurePursuitOutput::v)
        .def_readonly("w", &algo::PurePursuitOutput::w)
        .def_readonly("done", &algo::PurePursuitOutput::done)
        .def_readonly("nearest_index", &algo::PurePursuitOutput::nearest_index)
        .def_readonly("distance_to_goal", &algo::PurePursuitOutput::distance_to_goal);

    py::class_<algo::PurePursuit>(a, "PurePursuit")
        .def(py::init<algo::PurePursuitConfig>(), py::arg("config") = algo::PurePursuitConfig{})
        .def("step", &algo::PurePursuit::Step, py::arg("trajectory"), py::arg("pose"))
        .def_property_readonly("config", &algo::PurePursuit::config);

    py::class_<algo::FrontierExplorerConfig>(a, "FrontierExplorerConfig")
        .def(py::init<>())
        .def_readwrite("min_cluster_cells", &algo::FrontierExplorerConfig::min_cluster_cells)
        .def_readwrite("min_distance_m", &algo::FrontierExplorerConfig::min_distance_m)
        .def_readwrite("inflation_radius_m", &algo::FrontierExplorerConfig::inflation_radius_m)
        .def_readwrite("blacklist_radius_m", &algo::FrontierExplorerConfig::blacklist_radius_m);

    py::class_<algo::Frontier>(a, "Frontier")
        .def_readonly("x", &algo::Frontier::x)
        .def_readonly("y", &algo::Frontier::y)
        .def_readonly("cells", &algo::Frontier::cells)
        .def_readonly("distance", &algo::Frontier::distance);

    py::class_<algo::FrontierExplorer>(a, "FrontierExplorer")
        .def(py::init<algo::FrontierExplorerConfig>(), py::arg("config") = algo::FrontierExplorerConfig{})
        .def("find", &algo::FrontierExplorer::Find, py::arg("map"), py::arg("pose"))
        .def(
            "next_goal",
            [](const algo::FrontierExplorer& self, const types::OccupancyMap& map, const types::Pose2D& pose,
               const std::vector<std::pair<double, double>>& blacklist) -> py::object {
                types::Goal g;
                if (!self.NextGoal(map, pose, blacklist, g)) return py::none();
                return py::cast(g);
            },
            py::arg("map"), py::arg("pose"), py::arg("blacklist") = std::vector<std::pair<double, double>>{}
        );

    py::class_<algo::CoveragePlannerConfig>(a, "CoveragePlannerConfig")
        .def(py::init<>())
        .def_readwrite("lane_width_m", &algo::CoveragePlannerConfig::lane_width_m)
        .def_readwrite("inflation_radius_m", &algo::CoveragePlannerConfig::inflation_radius_m)
        .def_readwrite("min_run_cells", &algo::CoveragePlannerConfig::min_run_cells)
        .def_readwrite("perimeter_pass", &algo::CoveragePlannerConfig::perimeter_pass)
        .def_readwrite("perimeter_step_m", &algo::CoveragePlannerConfig::perimeter_step_m);

    py::class_<algo::CoveragePlanner>(a, "CoveragePlanner")
        .def(py::init<algo::CoveragePlannerConfig>(), py::arg("config") = algo::CoveragePlannerConfig{})
        .def("plan", &algo::CoveragePlanner::Plan, py::arg("map"), py::arg("start"), "Путевые точки змейки [(x, y), ...]")
        .def_static(
            "coverage_ratio", &algo::CoveragePlanner::CoverageRatio, py::arg("map"), py::arg("visited"), py::arg("radius_m")
        );

    py::class_<algo::CleaningMissionConfig>(a, "CleaningMissionConfig")
        .def(py::init<>())
        .def_readwrite("explorer", &algo::CleaningMissionConfig::explorer)
        .def_readwrite("coverage", &algo::CleaningMissionConfig::coverage)
        .def_readwrite("waypoint_tolerance_m", &algo::CleaningMissionConfig::waypoint_tolerance_m)
        .def_readwrite("dock_tolerance_m", &algo::CleaningMissionConfig::dock_tolerance_m)
        .def_readwrite("max_blocked_retries", &algo::CleaningMissionConfig::max_blocked_retries)
        .def_readwrite("robot_radius_m", &algo::CleaningMissionConfig::robot_radius_m)
        .def_readwrite("undock_distance_m", &algo::CleaningMissionConfig::undock_distance_m)
        .def_readwrite("frontier_done_radius_m", &algo::CleaningMissionConfig::frontier_done_radius_m)
        .def_readwrite("retarget_radius_m", &algo::CleaningMissionConfig::retarget_radius_m);

    py::class_<algo::CleaningMission>(a, "CleaningMission")
        .def(py::init<algo::CleaningMissionConfig>(), py::arg("config") = algo::CleaningMissionConfig{})
        .def("start", &algo::CleaningMission::Start, py::arg("dock"))
        .def("stop", &algo::CleaningMission::Stop)
        .def("return_to_dock", &algo::CleaningMission::ReturnToDock)
        .def(
            "step",
            [](algo::CleaningMission& self, const types::OccupancyMap& map, const types::Pose2D& pose,
               const types::PlannerState& planner_state) -> py::object {
                types::Goal g;
                if (!self.Step(map, pose, planner_state, g)) return py::none();
                return py::cast(g);
            },
            py::arg("map"), py::arg("pose"), py::arg("planner_state"),
            "Шаг миссии (~1 с): новая цель для планировщика или None"
        )
        .def_property_readonly("state", &algo::CleaningMission::state)
        .def("progress", &algo::CleaningMission::Progress, py::arg("map"))
        .def_property_readonly("visited", &algo::CleaningMission::visited)
        .def_property_readonly("waypoints", &algo::CleaningMission::waypoints)
        .def_property_readonly("blacklist", &algo::CleaningMission::blacklist)
        .def_property_readonly("dock", &algo::CleaningMission::dock);
}
