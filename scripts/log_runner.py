#! /usr/bin/env python3

import argparse
import math
import re
import sys

import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, "build/Debug/cpp/pybind")
from pyxiaomi_robot import types, algo


def parse_log_payload(line):
    LOG_PAYLOAD_RE = re.compile(
        r"\]\s*\[[^\]]+\]\s*(?P<tag>LASER|ODOM|GYRO|CMD_VEL):\s*(?P<payload>.+)$"
    )
    m = LOG_PAYLOAD_RE.search(line.strip())
    if not m:
        return None, None
    return m.group("tag"), m.group("payload").strip()


def parse_odom(payload) -> types.OdometryData:
    parts = payload.replace(",", " ").split()
    if len(parts) < 4:
        raise ValueError(f"bad ODOM payload: {payload!r}")
    odom = types.OdometryData()
    odom.timestamp = int(float(parts[0]))
    odom.px = float(parts[1])
    odom.py = float(parts[2])
    odom.yaw = float(parts[3])
    return odom


def parse_laser(payload) -> types.LaserData:
    parts = payload.split()
    laser = types.LaserData()
    if len(parts) < 4:
        return laser
    scan_start = float(parts[1])
    scan_res = float(parts[2])
    count = int(float(parts[3]))
    ranges = [float(r) for r in parts[4 : 4 + count]]
    if len(ranges) < count:
        ranges = [float(r) for r in parts[4:]]
    laser.ranges = list(reversed(ranges))
    n = len(laser.ranges)
    laser.scan_start = scan_start if abs(scan_start) > 1e-9 else -math.pi
    laser.scan_resolution = scan_res if abs(scan_res) > 1e-9 else (2 * math.pi / n if n else 0.0)
    return laser


def log_runner(path):
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            tag, msg = parse_log_payload(line)
            if tag is None:
                continue
            yield tag, msg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("log_file", type=str)
    args = parser.parse_args()
    mapping = algo.OccupancyMapping()

    for tag, msg in log_runner(args.log_file):
        if tag == "ODOM":
            mapping.update_odom(parse_odom(msg))
        elif tag == "LASER":
            mapping.update_scan(parse_laser(msg))
            occ = mapping.update_map()
            grid = np.array(occ.cells, dtype=np.uint8).reshape(occ.height, occ.width)
            plt.imshow(grid, cmap="gray", vmin=0, vmax=255)
            plt.pause(0.01)
            plt.clf()


if __name__ == "__main__":
    main()
