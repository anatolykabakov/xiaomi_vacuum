#include "algo/occupancy_mapping.h"
#include "types.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
namespace types = project::types;
namespace algo = project::algo;

PYBIND11_MODULE(pyxiaomi_robot, pyxiaomi_robot)
{
    if (py::hasattr(pyxiaomi_robot, "PyXiaomiRobot"))
    {
        return;
    }

    pyxiaomi_robot.doc() = "Xiaomi robot: occupancy grid mapping (replay logs / tests from Python)";

    auto types = pyxiaomi_robot.def_submodule("types", "Types");

    py::class_<types::OdometryData>(types, "OdometryData")
        .def(py::init<>())
        .def_readwrite("timestamp", &types::OdometryData::timestamp)
        .def_readwrite("px", &types::OdometryData::px)
        .def_readwrite("py", &types::OdometryData::py)
        .def_readwrite("vx", &types::OdometryData::vx)
        .def_readwrite("vy", &types::OdometryData::vy)
        .def_readwrite("yaw", &types::OdometryData::yaw)
        .def_readwrite("omega", &types::OdometryData::omega);

    py::class_<types::LaserData>(types, "LaserData")
        .def(py::init<>())
        .def_readwrite("timestamp", &types::LaserData::timestamp)
        .def_readwrite("scan_start", &types::LaserData::scan_start)
        .def_readwrite("scan_resolution", &types::LaserData::scan_resolution)
        .def_readwrite("ranges", &types::LaserData::ranges);

    py::class_<types::OccupancyMap>(types, "OccupancyMap")
        .def(py::init<>())
        .def_readwrite("timestamp", &types::OccupancyMap::timestamp)
        .def_readwrite("width", &types::OccupancyMap::width)
        .def_readwrite("height", &types::OccupancyMap::height)
        .def_readwrite("resolution", &types::OccupancyMap::resolution)
        .def_readwrite("origin_x", &types::OccupancyMap::origin_x)
        .def_readwrite("origin_y", &types::OccupancyMap::origin_y)
        .def_readwrite("cells", &types::OccupancyMap::cells)
        .def_readwrite("robot_x", &types::OccupancyMap::robot_x)
        .def_readwrite("robot_y", &types::OccupancyMap::robot_y)
        .def_readwrite("robot_yaw", &types::OccupancyMap::robot_yaw);

    auto algo = pyxiaomi_robot.def_submodule("algo", "Algorithm");

    py::class_<algo::OccupancyMapping>(algo, "OccupancyMapping")
        .def(py::init<>())
        .def("update_scan", &algo::OccupancyMapping::UpdateScan, py::arg("scan"))
        .def("update_odom", &algo::OccupancyMapping::UpdateOdom, py::arg("odom"))
        .def("update_map", &algo::OccupancyMapping::UpdateMap);
}
