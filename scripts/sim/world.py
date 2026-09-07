"""Плоский мир для симулятора: план помещения из PNG, дифференциальный привод, лидар.

Система координат мира — метры, x вправо, y вверх (строки изображения идут вниз, поэтому
y = (H - row) * scale). Робот стартует в точке дока; кадр одометрии и карты — мир, сдвинутый
так, что док лежит в (0, 0). Лидар стоит там же, где на роботе: смещение (-0.1, 0) м и поворот
97.6°, дальности отдаются в кадре лидара, 0 — нет отражения.
"""

from __future__ import annotations

import math
import random

import cv2
import numpy as np

import pyxiaomi_robot as xr

OCCUPIED_THRESHOLD = 0.6 * 255  # как в utils.Image2Map: темнее — стена


class World:
    def __init__(
        self,
        png: str,
        scale_m_per_px: float = 0.02,
        start_px: tuple[float, float] = (150.0, 100.0),
        start_yaw: float | None = None,
        lidar_range_m: float = 6.0,
        beams: int = 360,
        range_noise_m: float = 0.01,
        odom_noise: float = 0.0,
        seed: int = 0,
    ):
        img = cv2.imread(png, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise FileNotFoundError(png)
        self.occupied = img < OCCUPIED_THRESHOLD
        self.height, self.width = img.shape
        self.scale = scale_m_per_px
        # расстояние до ближайшей стены (в пикселях) — для проверки столкновений корпусом
        free_u8 = (~self.occupied).astype(np.uint8)
        self.dist_px = cv2.distanceTransform(free_u8, cv2.DIST_L2, 5)
        # достижимая площадь: компонента свободных пикселей, содержащая старт (внутренности
        # замкнутых препятствий на плане тоже белые, но робот туда не попадёт)
        _, labels = cv2.connectedComponents(free_u8, connectivity=4)
        self.reachable = labels == labels[int(round(start_px[1])), int(round(start_px[0]))]

        cfg = xr.robot_config()
        self.radius = cfg.radius_m
        self.lidar_dx = cfg.lidar_offset_x_m
        self.lidar_dy = cfg.lidar_offset_y_m
        self.lidar_dyaw = cfg.lidar_offset_yaw_rad
        self.lidar_range = lidar_range_m
        self.beams = beams
        self.scan_start = -math.pi
        self.scan_res = 2 * math.pi / beams
        self.range_noise = range_noise_m
        self.odom_noise = odom_noise
        self.rng = random.Random(seed)

        self.start = np.array([start_px[0] * self.scale, (self.height - start_px[1]) * self.scale])
        if start_yaw is None:
            start_yaw = self._away_from_wall(start_px)
        self.x, self.y, self.yaw = float(self.start[0]), float(self.start[1]), start_yaw
        self.odom_x, self.odom_y, self.odom_yaw = 0.0, 0.0, start_yaw
        self.v, self.w = 0.0, 0.0
        self.collisions = 0
        self.bumper = False  # последний шаг упёрся в стену
        self.distance = 0.0
        self.path: list[tuple[float, float]] = [(self.x, self.y)]

        # выборка вдоль луча с шагом в полпикселя
        self._ray_steps = np.arange(0.0, self.lidar_range, self.scale / 2)

    def _away_from_wall(self, start_px: tuple[float, float]) -> float:
        """Курс от ближайшей стены: так стоит робот на доке (док у стены, робот смотрит в комнату)."""
        c, r = int(round(start_px[0])), int(round(start_px[1]))
        gy, gx = np.gradient(self.dist_px)
        dx, dy = float(gx[r, c]), -float(gy[r, c])  # строки идут вниз → y мира растёт вверх
        if abs(dx) < 1e-9 and abs(dy) < 1e-9:
            return 0.0
        return math.atan2(dy, dx)

    # ------------------------------------------------------------------ кадры
    def to_px(self, x: float, y: float) -> tuple[float, float]:
        return x / self.scale, self.height - y / self.scale

    def to_world(self, col: float, row: float) -> tuple[float, float]:
        return col * self.scale, (self.height - row) * self.scale

    def world_to_map(self, x: float, y: float) -> tuple[float, float]:
        """Мир → кадр карты/одометрии (док в нуле)."""
        return x - self.start[0], y - self.start[1]

    def map_to_world(self, x: float, y: float) -> tuple[float, float]:
        return x + self.start[0], y + self.start[1]

    def true_pose(self) -> xr.types.Pose2D:
        mx, my = self.world_to_map(self.x, self.y)
        return xr.types.Pose2D(mx, my, self.yaw)

    def free_of_walls(self, x: float, y: float) -> bool:
        col, row = self.to_px(x, y)
        c, r = int(round(col)), int(round(row))
        if c < 0 or r < 0 or c >= self.width or r >= self.height:
            return False
        return self.dist_px[r, c] * self.scale > self.radius

    # ------------------------------------------------------------------ динамика
    def move(self, v: float, w: float, dt: float) -> bool:
        """Шаг дифференциального привода. Упёрлись в стену — стоим, считаем столкновение."""
        yaw_mid = self.yaw + 0.5 * w * dt
        nx = self.x + v * math.cos(yaw_mid) * dt
        ny = self.y + v * math.sin(yaw_mid) * dt
        nyaw = (self.yaw + w * dt + math.pi) % (2 * math.pi) - math.pi
        moved = True
        if not self.free_of_walls(nx, ny):
            self.collisions += 1
            nx, ny = self.x, self.y
            moved = False
            v = 0.0
        self.bumper = not moved
        self.distance += math.hypot(nx - self.x, ny - self.y)
        self.x, self.y, self.yaw = nx, ny, nyaw
        self.v, self.w = v, w
        self.path.append((self.x, self.y))
        # одометрия: та же кинематика, при желании — с шумом на скоростях
        ov, ow = v, w
        if self.odom_noise > 0:
            ov *= 1 + self.rng.gauss(0, self.odom_noise)
            ow *= 1 + self.rng.gauss(0, self.odom_noise)
        omid = self.odom_yaw + 0.5 * ow * dt
        self.odom_x += ov * math.cos(omid) * dt
        self.odom_y += ov * math.sin(omid) * dt
        self.odom_yaw = (self.odom_yaw + ow * dt + math.pi) % (2 * math.pi) - math.pi
        return moved

    # ------------------------------------------------------------------ сенсоры
    def odometry(self, t: float) -> xr.types.Odometry:
        o = xr.types.Odometry()
        o.timestamp = int(t * 1e6)
        o.px, o.py, o.yaw = self.odom_x, self.odom_y, self.odom_yaw
        o.vx = self.v * math.cos(self.odom_yaw)
        o.vy = self.v * math.sin(self.odom_yaw)
        o.omega = self.w
        return o

    def imu(self, t: float) -> xr.types.Imu:
        i = xr.types.Imu()
        i.timestamp = int(t * 1e6)
        i.yaw = self.yaw
        i.vel_yaw = self.w
        return i

    def scan(self, t: float) -> xr.types.LaserScan:
        # лидар: положение и курс в мире
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        lx = self.x + c * self.lidar_dx - s * self.lidar_dy
        ly = self.y + s * self.lidar_dx + c * self.lidar_dy
        lyaw = self.yaw + self.lidar_dyaw
        angles = lyaw + self.scan_start + self.scan_res * np.arange(self.beams)
        # точки вдоль всех лучей: (beams, steps)
        px = lx + np.cos(angles)[:, None] * self._ray_steps[None, :]
        py = ly + np.sin(angles)[:, None] * self._ray_steps[None, :]
        cols = np.round(px / self.scale).astype(int)
        rows = np.round(self.height - py / self.scale).astype(int)
        inside = (cols >= 0) & (rows >= 0) & (cols < self.width) & (rows < self.height)
        hit = np.zeros_like(inside)
        hit[inside] = self.occupied[rows[inside], cols[inside]]
        first = np.argmax(hit, axis=1)
        has_hit = hit[np.arange(self.beams), first]
        ranges = np.where(has_hit, self._ray_steps[first], 0.0)
        if self.range_noise > 0:
            noise = np.array([self.rng.gauss(0, self.range_noise) for _ in range(self.beams)])
            ranges = np.where(has_hit, np.maximum(ranges + noise, 0.05), 0.0)
        scan = xr.types.LaserScan()
        scan.timestamp = int(t * 1e6)
        scan.scan_start = self.scan_start
        scan.scan_resolution = self.scan_res
        scan.ranges = [float(r) for r in ranges]
        return scan

    # ------------------------------------------------------------------ метрики
    def explored_ratio(self, occ_map: xr.types.OccupancyMap) -> float:
        """Доля достижимых свободных пикселей плана, отмеченных на карте робота как свободные."""
        rows, cols = np.where(self.reachable)
        wx = cols * self.scale - self.start[0]
        wy = (self.height - rows) * self.scale - self.start[1]
        gx = (wx / occ_map.resolution + occ_map.origin_x).astype(int)
        gy = (wy / occ_map.resolution + (occ_map.height - occ_map.origin_y)).astype(int)
        ok = (gx >= 0) & (gy >= 0) & (gx < occ_map.width) & (gy < occ_map.height)
        cells = occ_map.cells
        known_free = np.zeros(len(rows), dtype=bool)
        known_free[ok] = cells[gy[ok], gx[ok]] == 255
        return float(known_free.mean()) if len(rows) else 0.0

    def true_coverage(self, clearance_m: float) -> tuple[float, float]:
        """Покрытие корпусом робота по пройденному пути: доля всех достижимых свободных пикселей
        и доля тех, что робот вообще может накрыть, держась от стен не ближе clearance_m
        (полоса у стен шириной clearance - radius планировщику недоступна)."""
        covered = np.zeros((self.height, self.width), dtype=np.uint8)
        r_px = max(1, int(round(self.radius / self.scale)))
        for i, (x, y) in enumerate(self.path):
            if i % 2:  # шаг 50 мс при 0.25 м/с — 1.25 см, каждая вторая точка достаточно
                continue
            col, row = self.to_px(x, y)
            cv2.circle(covered, (int(round(col)), int(round(row))), r_px, 1, -1)
        free = self.reachable
        centers = (self.reachable & (self.dist_px * self.scale >= clearance_m)).astype(np.uint8)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r_px + 1, 2 * r_px + 1))
        coverable = cv2.dilate(centers, kernel).astype(bool) & free
        return (
            float(covered[free].mean()) if free.any() else 0.0,
            float(covered[coverable].mean()) if coverable.any() else 0.0,
        )

    def render(self, occ_map: xr.types.OccupancyMap | None = None, upscale: int = 2) -> np.ndarray:
        """План с пройденным путём (зелёный), стартом (синий) и роботом (красный)."""
        img = np.where(self.occupied, 0, 255).astype(np.uint8)
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        img = cv2.resize(img, (self.width * upscale, self.height * upscale), interpolation=cv2.INTER_NEAREST)
        pts = [tuple(int(round(v * upscale)) for v in self.to_px(x, y)) for x, y in self.path[::4]]
        for a, b in zip(pts, pts[1:]):
            cv2.line(img, a, b, (0, 160, 0), 1)
        sc, sr = self.to_px(self.start[0], self.start[1])
        cv2.circle(img, (int(sc * upscale), int(sr * upscale)), 4 * upscale, (255, 0, 0), -1)
        rc, rr = self.to_px(self.x, self.y)
        cv2.circle(img, (int(rc * upscale), int(rr * upscale)), int(self.radius / self.scale * upscale), (0, 0, 255), 1)
        return img


def render_map(occ_map: xr.types.OccupancyMap, path_map: list[tuple[float, float]] | None = None,
               waypoints: list[tuple[float, float]] | None = None, upscale: int = 2) -> np.ndarray:
    """Карта робота: серое — неизвестно, белое — свободно, чёрное — стены; путь зелёный, змейка жёлтая."""
    cells = occ_map.cells
    img = cv2.cvtColor(cells, cv2.COLOR_GRAY2BGR)
    img = cv2.resize(img, (occ_map.width * upscale, occ_map.height * upscale), interpolation=cv2.INTER_NEAREST)

    def to_px(x: float, y: float) -> tuple[int, int]:
        g = xr.algo.PathPlanner.world_to_grid(occ_map, x, y)
        if g is None:
            return -1, -1
        return int(g[0] * upscale), int(g[1] * upscale)

    if waypoints:
        pts = [to_px(x, y) for x, y in waypoints]
        for a, b in zip(pts, pts[1:]):
            cv2.line(img, a, b, (0, 200, 255), 1)
    if path_map:
        pts = [to_px(x, y) for x, y in path_map]
        for a, b in zip(pts, pts[1:]):
            cv2.line(img, a, b, (0, 160, 0), 1)
    cv2.circle(img, to_px(0.0, 0.0), 3 * upscale, (255, 0, 0), -1)
    return img
