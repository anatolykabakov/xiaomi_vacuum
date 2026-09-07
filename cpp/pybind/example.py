"""Пример работы с биндингами: один скан → карта. Запуск из корня репозитория:

    PYTHONPATH=build/Debug/cpp/pybind python3 cpp/pybind/example.py
"""

import math

import pyxiaomi_robot as xr

if __name__ == "__main__":
    mapping = xr.algo.OccupancyMapping()
    laser = xr.types.LaserScan()
    laser.ranges = [0.5] * 360
    laser.scan_start = -math.pi
    laser.scan_resolution = 2 * math.pi / 360
    odom = xr.types.Odometry()
    odom.px, odom.py, odom.yaw = -0.06, -0.15, 2.2
    mapping.update_scan(laser)
    mapping.update_odom(odom)
    occ = mapping.update_map()
    cells = occ.cells  # numpy (height, width): 0 занято, 127 неизвестно, 255 свободно
    print(cells.shape, "занято:", int((cells == 0).sum()), "свободно:", int((cells == 255).sum()))
