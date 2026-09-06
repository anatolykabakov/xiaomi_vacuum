"""
Pure Pursuit: следование по пути (v, ω) для дифф-привода.

Конвенция как в SingleBotLaser2D.Move:
  θ=0° → вперёд по +Y;  Δx = -v·sin(θ), Δy = +v·cos(θ)

См. doc/SIM_EXPLORE_PLAN.md, planner.py
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from planner import path_length


def normalize_angle_deg(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def forward_heading_to_point(pose: np.ndarray, wx: float, wy: float) -> float:
    """
    Курс θ [deg], при котором «вперёд» смотрит на (wx, wy).
    Вперёд = (-sin θ, cos θ) ∥ (dx, dy).
    """
    dx = wx - pose[0]
    dy = wy - pose[1]
    return math.degrees(math.atan2(-dx, dy))


def _segment_lengths(path: list[tuple[float, float]]) -> list[float]:
    if len(path) < 2:
        return []
    return [
        math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
        for i in range(1, len(path))
    ]


def _point_on_path(
    path: list[tuple[float, float]], start_dist: float, ahead: float
) -> tuple[float, float]:
    """Точка на полилинии на расстоянии start_dist + ahead от начала пути."""
    if not path:
        return 0.0, 0.0
    if len(path) == 1:
        return path[0]

    target = start_dist + ahead
    traveled = 0.0

    for i in range(1, len(path)):
        x0, y0 = path[i - 1]
        x1, y1 = path[i]
        seg = math.hypot(x1 - x0, y1 - y0)
        if seg < 1e-9:
            continue
        if traveled + seg >= target:
            t = (target - traveled) / seg
            return x0 + t * (x1 - x0), y0 + t * (y1 - y0)
        traveled += seg

    return path[-1]


def _distance_along_path(path: list[tuple[float, float]], pose: np.ndarray) -> float:
    """Приближение: проекция на ближайшую вершину + сегмент до неё."""
    if not path:
        return 0.0
    if len(path) == 1:
        return 0.0

    best_dist = float("inf")
    best_along = 0.0
    traveled = 0.0

    for i in range(1, len(path)):
        x0, y0 = path[i - 1]
        x1, y1 = path[i]
        seg = math.hypot(x1 - x0, y1 - y0)
        if seg < 1e-9:
            traveled += seg
            continue

        # ближайшая точка на сегменте
        dx, dy = x1 - x0, y1 - y0
        t = ((pose[0] - x0) * dx + (pose[1] - y0) * dy) / (seg * seg)
        t = max(0.0, min(1.0, t))
        px = x0 + t * dx
        py = y0 + t * dy
        d = math.hypot(pose[0] - px, pose[1] - py)
        along = traveled + t * seg

        if d < best_dist:
            best_dist = d
            best_along = along
        traveled += seg

    return best_along


def find_lookahead_point(
    path: list[tuple[float, float]],
    pose: np.ndarray,
    lookahead: float,
) -> tuple[float, float] | None:
    if not path:
        return None
    along = _distance_along_path(path, pose)
    return _point_on_path(path, along, lookahead)


@dataclass
class PurePursuitConfig:
    lookahead: float = 8.0
    goal_tolerance: float = 4.0
    turn_only_angle_deg: float = 35.0
    k_alpha: float = 1.0
    v_max: float = 6.0
    w_max: float = 6.0


class PurePursuitController:
    def __init__(self, config: PurePursuitConfig | None = None):
        self.cfg = config or PurePursuitConfig()
        self.path: list[tuple[float, float]] = []
        self.path_index: int = 0

    def set_path(self, path: list[tuple[float, float]]) -> None:
        self.path = list(path)
        self.path_index = 0

    def goal_reached(self, pose: np.ndarray) -> bool:
        if not self.path:
            return True
        gx, gy = self.path[-1]
        return math.hypot(pose[0] - gx, pose[1] - gy) < self.cfg.goal_tolerance

    def compute(self, pose: np.ndarray) -> tuple[float, float]:
        """Один тик: (v, ω) в единицах bot_param."""
        if not self.path or self.goal_reached(pose):
            return 0.0, 0.0

        gx, gy = self.path[-1]
        dist_goal = math.hypot(pose[0] - gx, pose[1] - gy)
        if dist_goal < self.cfg.lookahead * 1.5:
            lx, ly = gx, gy
        else:
            lookahead_pt = find_lookahead_point(self.path, pose, self.cfg.lookahead)
            if lookahead_pt is None:
                return 0.0, 0.0
            lx, ly = lookahead_pt
        target_h = forward_heading_to_point(pose, lx, ly)
        alpha = normalize_angle_deg(target_h - pose[2])

        w = max(-self.cfg.w_max, min(self.cfg.k_alpha * alpha, self.cfg.w_max))

        if abs(alpha) > self.cfg.turn_only_angle_deg:
            v = 0.0
        else:
            scale = max(0.3, 1.0 - abs(alpha) / 90.0)
            v = self.cfg.v_max * scale

        return v, w


def config_from_bot_param(bot_param: list, **kwargs) -> PurePursuitConfig:
    return PurePursuitConfig(
        v_max=bot_param[4],
        w_max=bot_param[5],
        **kwargs,
    )


def pure_pursuit_control(
    pose: np.ndarray,
    path: list[tuple[float, float]],
    bot_param: list,
    **kwargs,
) -> tuple[float, float]:
    """Разовый вызов без состояния контроллера."""
    ctrl = PurePursuitController(config_from_bot_param(bot_param, **kwargs))
    ctrl.set_path(path)
    return ctrl.compute(pose)


def apply_kinematics(pose: np.ndarray, v: float, w: float, dt: float = 1.0) -> np.ndarray:
    """Та же кинематика, что SingleBotLaser2D.Move (без шума)."""
    th = np.deg2rad(pose[2])
    pose = pose.copy()
    pose[0] -= v * np.sin(th) * dt
    pose[1] += v * np.cos(th) * dt
    pose[2] = (pose[2] + w * dt) % 360
    return pose


def mini_pure_pursuit_demo(steps: int = 35) -> None:
    """Едет по пути из planner.mini_astar_example (сетка 9x5)."""
    from GridMap import GridMap
    from planner import astar

    layout = [
        "#########",
        "#S......#",
        "#...#...#",
        "#......G#",
        "#########",
    ]
    map_param = [0.4, -0.4, 5.0, -5.0]
    gmap = GridMap(map_param, gsize=1.0)
    start = goal = None
    for iy, row in enumerate(layout):
        for ix, ch in enumerate(row):
            if ch == "#":
                gmap.gmap[(ix, iy)] = 2.0
            elif ch in ".SG":
                gmap.gmap[(ix, iy)] = -2.0
                if ch == "S":
                    start = (ix, iy)
                elif ch == "G":
                    goal = (ix, iy)
    gmap.boundary = [0, 8, 0, 4]
    bounds = (0, 9, 0, 5)

    assert start and goal
    grid_path = astar(gmap, start, goal, bounds)
    if not grid_path:
        print("Нет пути для демо.")
        return

    path = [(float(ix), float(iy)) for ix, iy in grid_path]
    pose = np.array([float(start[0]), float(start[1]), 0.0])
    if len(path) > 1:
        pose[2] = forward_heading_to_point(pose, path[1][0], path[1][1])

    # v_max=1.0 — один шаг ≈ одна ячейка сетки (в sim пикселях обычно 6.0)
    ctrl = PurePursuitController(
        PurePursuitConfig(
            lookahead=1.2,
            goal_tolerance=0.4,
            turn_only_angle_deg=20.0,
            v_max=1.0,
            w_max=25.0,
        )
    )
    ctrl.set_path(path)

    print("=== Mini Pure Pursuit ===\n")
    print("Путь:", " → ".join(f"({x:.0f},{y:.0f})" for x, y in path))
    print(f"Длина пути: {path_length(path):.1f}\n")
    print("step   x      y      θ°     v     ω    dist→G")
    print("-" * 52)

    for step in range(steps):
        gx, gy = path[-1]
        dist_g = math.hypot(pose[0] - gx, pose[1] - gy)
        v, w = ctrl.compute(pose)
        print(
            f"{step:3d}  {pose[0]:5.2f} {pose[1]:5.2f} {pose[2]:6.1f} "
            f"{v:5.1f} {w:5.1f} {dist_g:6.2f}"
        )
        if ctrl.goal_reached(pose):
            print("\nЦель достигнута.")
            break
        pose = apply_kinematics(pose, v, w)
    else:
        print("\nЛимит шагов (увеличь steps или lookahead).")


if __name__ == "__main__":
    mini_pure_pursuit_demo()
