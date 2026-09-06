import sys

sys.path.insert(0, "build/Debug/cpp/pybind")
import pyxiaomi_robot as xr

if __name__ == "__main__":
    mapping = xr.OccupancyMapping()
    laser = xr.LaserData()
    laser.ranges = [0.5] * 360
    laser.scan_start = -3.14159265
    laser.scan_resolution = 2 * 3.14159265 / 360
    odom = xr.OdometryData()
    odom.px, odom.py, odom.yaw = -0.06, -0.15, 2.2
    mapping.update_scan(laser)
    mapping.update_odom(odom)
    occ = mapping.update_map()
    print(occ.cells)
