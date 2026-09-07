#!/usr/bin/env python3
"""Полный цикл уборки в симуляторе: карта → исследование → покрытие змейкой → возврат на док.

Гоняются те же алгоритмы, что на роботе (через pyxiaomi_robot), с теми же периодами, что у
сервисов: лидар и карта 5 Гц, миссия 1 Гц, планировщик 2 Гц, ведение 20 Гц. Одометрия по
умолчанию точная — проверяется логика поведения, а не дрейф.

    PYTHONPATH=build/Debug/cpp/pybind python3 scripts/sim/run_cycle.py --out /tmp/cycle

Код возврата 0, если карта построена, площадь покрыта и робот вернулся на док (пороги — флаги).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent))

import pyxiaomi_robot as xr  # noqa: E402
from world import World, render_map  # noqa: E402

PS = xr.types.PlannerState.State
MS = xr.types.MissionState.State


class PlannerLoop:
    """Как сервис Planner: цель → траектория; перепланирование только когда прежняя не годится."""

    def __init__(self, config: xr.algo.PathPlannerConfig):
        self.planner = xr.algo.PathPlanner(config)
        self.tolerance = config.goal_tolerance_m
        self.goal: xr.types.Goal | None = None
        self.state = xr.types.PlannerState()
        self.trajectory: xr.types.Trajectory | None = None
        self.plans = 0

    def on_bumper(self) -> None:
        """Как по safety/stop с причиной bumper: прежняя траектория вела в препятствие."""
        self.trajectory = None

    def on_goal(self, goal: xr.types.Goal, occ_map, pose) -> None:
        if goal.type == xr.types.Goal.Type.STOP:
            self.goal = None
            self.trajectory = xr.types.Trajectory()
            self._set(PS.IDLE, 0)
            return
        self.goal = goal
        self.tick(occ_map, pose)

    def _set(self, state, goal_id: int) -> None:
        s = xr.types.PlannerState()
        s.state = state
        s.goal_id = goal_id
        self.state = s

    def tick(self, occ_map, pose) -> None:
        if self.goal is None:
            self._set(PS.IDLE, 0)
            return
        dist = math.hypot(self.goal.x - pose.x, self.goal.y - pose.y)
        if dist <= self.tolerance:
            gid = self.goal.id
            self.goal = None
            self.trajectory = xr.types.Trajectory()
            self._set(PS.DONE, gid)
            return
        if (self.trajectory is not None and len(self.trajectory.points) > 0
                and self.trajectory.goal_id == self.goal.id and self.planner.valid(occ_map, self.trajectory, pose)):
            self._set(PS.FOLLOWING, self.goal.id)  # прежняя траектория годится — едем по ней
            return
        self.plans += 1
        traj = self.planner.plan(occ_map, pose, self.goal)
        if traj is None:
            self._set(PS.BLOCKED, self.goal.id)
            return
        traj.goal_id = self.goal.id
        self.trajectory = traj
        self._set(PS.FOLLOWING, self.goal.id)


class ControlLoop:
    """Как сервис Control: pure pursuit по последней траектории, ноль без траектории и по done,
    по бамперу — откат назад на recover_distance_m и ожидание новой траектории."""

    def __init__(self, config: xr.algo.PurePursuitConfig, dt: float,
                 recover_distance_m: float = 0.10, recover_speed_mps: float = 0.08):
        self.pursuit = xr.algo.PurePursuit(config)
        self.trajectory: xr.types.Trajectory | None = None
        self.done = False
        self.recover_speed = recover_speed_mps
        self.recover_ticks_total = max(1, int(recover_distance_m / recover_speed_mps / dt))
        self.recover_ticks = 0
        self.recoveries = 0

    def on_trajectory(self, trajectory: xr.types.Trajectory | None) -> None:
        if self.recover_ticks > 0:
            return  # во время отката новые траектории не берём: Planner ещё не знает, где встали
        self.trajectory = trajectory if trajectory is not None and len(trajectory.points) > 0 else None
        self.done = False

    def tick(self, pose, bumper: bool = False) -> tuple[float, float]:
        if self.recover_ticks > 0:
            self.recover_ticks -= 1
            if self.recover_ticks == 0:
                self.trajectory = None
                return 0.0, 0.0
            return -self.recover_speed, 0.0
        if bumper:
            if self.trajectory is not None:
                self.recover_ticks = self.recover_ticks_total
                self.recoveries += 1
            return 0.0, 0.0
        if self.trajectory is None or self.done:
            return 0.0, 0.0
        out = self.pursuit.step(self.trajectory, pose)
        if out.done:
            self.done = True
            return 0.0, 0.0
        return out.v, out.w


def build_configs():
    cfg = xr.robot_config()
    plan = xr.algo.PathPlannerConfig()
    plan.inflation_radius_m = cfg.radius_m + 0.03
    plan.max_v_mps = cfg.plan_v_mps
    plan.max_w_rps = cfg.plan_w_rps
    plan.max_accel_mps2 = cfg.max_accel_mps2

    ctrl = xr.algo.PurePursuitConfig()
    ctrl.max_v_mps = cfg.plan_v_mps
    ctrl.max_w_rps = cfg.plan_w_rps

    mission = xr.algo.CleaningMissionConfig()
    mission.robot_radius_m = cfg.radius_m
    mission.explorer.inflation_radius_m = cfg.radius_m + 0.03
    mission.coverage.inflation_radius_m = cfg.radius_m + 0.03
    mission.coverage.lane_width_m = 2 * cfg.radius_m * 0.85
    return plan, ctrl, mission


def main() -> int:
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", default=str(here / "map.png"), help="план помещения, тёмное — стены")
    ap.add_argument("--scale", type=float, default=0.02, help="метров в пикселе плана")
    ap.add_argument("--start", default="150,100", help="старт (док) в пикселях плана, col,row")
    ap.add_argument("--start-yaw", default="auto", help="курс на старте, рад; auto — от ближайшей стены")
    ap.add_argument("--lidar-range", type=float, default=6.0)
    ap.add_argument("--beams", type=int, default=360)
    ap.add_argument("--odom-noise", type=float, default=0.0, help="относительный шум скоростей одометрии")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--dt", type=float, default=0.05, help="шаг ведения, с")
    ap.add_argument("--max-time", type=float, default=3600.0, help="лимит модельного времени, с")
    ap.add_argument("--min-explored", type=float, default=0.90)
    ap.add_argument("--min-coverage", type=float, default=0.85, help="доля площади, доступной роботу с зазором планировщика")
    ap.add_argument("--dock-tolerance", type=float, default=0.30)
    ap.add_argument("--max-bumps", type=int, default=20, help="допустимое число срабатываний бампера")
    ap.add_argument("--out", default="", help="каталог для картинок и summary.json")
    ap.add_argument("--gui", action="store_true", help="показывать карту по ходу (OpenCV)")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    col, row = (float(v) for v in args.start.split(","))
    start_yaw = None if args.start_yaw == "auto" else float(args.start_yaw)
    world = World(
        args.map, args.scale, (col, row), start_yaw=start_yaw, lidar_range_m=args.lidar_range,
        beams=args.beams, odom_noise=args.odom_noise, seed=args.seed,
    )
    if not world.free_of_walls(world.x, world.y):
        print(f"старт {args.start} внутри стены", file=sys.stderr)
        return 2

    plan_cfg, ctrl_cfg, mission_cfg = build_configs()
    mapping = xr.algo.OccupancyMapping()
    mission = xr.algo.CleaningMission(mission_cfg)
    planner = PlannerLoop(plan_cfg)
    control = ControlLoop(ctrl_cfg, args.dt)
    radius = xr.robot_config().radius_m

    dt = args.dt
    scan_period, plan_period, mission_period = 0.2, 0.5, 1.0
    next_scan = next_plan = next_mission = 0.0
    next_report = 0.0
    t = 0.0
    wall_start = time.time()

    # первый скан и карта до старта: миссии нужна карта, чтобы увидеть фронтиры
    mapping.update_odom(world.odometry(t))
    mapping.update_scan(world.scan(t))
    occ_map = mapping.update_map()
    pose = xr.types.Pose2D(0.0, 0.0, world.odom_yaw)
    mission.start(pose)
    if not args.quiet:
        print(f"мир {world.width}x{world.height} px, {world.width * args.scale:.1f}x{world.height * args.scale:.1f} м; "
              f"старт {args.start}, курс {math.degrees(world.yaw):.0f}°")

    while mission.state != MS.DONE and t < args.max_time:
        odom = world.odometry(t)
        pose = xr.types.Pose2D(odom.px, odom.py, odom.yaw)
        pose.v, pose.w = world.v, world.w

        if t >= next_scan:
            next_scan += scan_period
            mapping.update_odom(odom)
            mapping.update_scan(world.scan(t))
            occ_map = mapping.update_map()

        if t >= next_mission:
            next_mission += mission_period
            goal = mission.step(occ_map, pose, planner.state)
            if goal is not None:
                planner.on_goal(goal, occ_map, pose)
                control.on_trajectory(planner.trajectory)
                if not args.quiet:
                    print(f"[{t:7.1f}] {mission.state.name:9} цель #{goal.id} ({goal.x:6.2f}, {goal.y:6.2f}) "
                          f"→ {planner.state.state.name}")

        if t >= next_plan:
            next_plan += plan_period
            before = planner.state.state
            planner.tick(occ_map, pose)
            if planner.goal is not None or before != planner.state.state:
                control.on_trajectory(planner.trajectory)

        v, w = control.tick(pose, world.bumper)
        world.move(v, w, dt)
        if world.bumper:
            # бампер: препятствие у кромки корпуса — в карту; прежняя траектория негодна
            mapping.mark_obstacle(pose.x + (radius + 0.03) * math.cos(pose.yaw),
                                  pose.y + (radius + 0.03) * math.sin(pose.yaw))
            planner.on_bumper()
        t += dt

        if not args.quiet and t >= next_report:
            next_report += 30.0
            p = mission.progress(occ_map)
            print(f"[{t:7.1f}] {p.state.name:9} исследовано {world.explored_ratio(occ_map):.2f} "
                  f"покрыто {p.coverage_ratio:.2f} фронтиров {p.frontiers_left} "
                  f"змейка {p.waypoints_done}/{p.waypoints_total} столкновений {world.collisions} "
                  f"откатов {control.recoveries} планов {planner.plans}")
        if args.gui and int(t / dt) % 10 == 0:
            cv2.imshow("map", render_map(occ_map, mission.visited, mission.waypoints))
            cv2.imshow("world", world.render())
            if cv2.waitKey(1) == 27:
                break

    progress = mission.progress(occ_map)
    dock_dist = math.hypot(world.x - world.start[0], world.y - world.start[1])
    # зазор планировщика в клетках карты: ceil(inflation / res) * res
    clearance = math.ceil(plan_cfg.inflation_radius_m / occ_map.resolution - 1e-9) * occ_map.resolution
    cov_all, cov_coverable = world.true_coverage(clearance)
    summary = {
        "state": progress.state.name,
        "detail": progress.detail,
        "sim_time_s": round(t, 1),
        "wall_time_s": round(time.time() - wall_start, 1),
        "explored_ratio": round(world.explored_ratio(occ_map), 3),
        "coverage_ratio_map": round(progress.coverage_ratio, 3),
        "coverage_ratio_true": round(cov_all, 3),
        "coverage_ratio_coverable": round(cov_coverable, 3),
        "dock_distance_m": round(dock_dist, 3),
        "distance_m": round(world.distance, 1),
        "collisions": world.collisions,
        "recoveries": control.recoveries,
        "waypoints": f"{progress.waypoints_done}/{progress.waypoints_total}",
        "plans": planner.plans,
    }
    checks = {
        "done": progress.state == MS.DONE,
        "explored": summary["explored_ratio"] >= args.min_explored,
        "covered": summary["coverage_ratio_coverable"] >= args.min_coverage,
        "docked": dock_dist <= args.dock_tolerance,
        "bumps_ok": world.collisions <= args.max_bumps,
    }
    summary["checks"] = checks
    print(json.dumps(summary, ensure_ascii=False, indent=2))

    if args.out:
        os.makedirs(args.out, exist_ok=True)
        cv2.imwrite(os.path.join(args.out, "map.png"), render_map(occ_map, mission.visited, mission.waypoints))
        cv2.imwrite(os.path.join(args.out, "world.png"), world.render())
        with open(os.path.join(args.out, "summary.json"), "w", encoding="utf-8") as f:
            json.dump(summary, f, ensure_ascii=False, indent=2)
    if args.gui:
        cv2.waitKey(0)
    return 0 if all(checks.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
