import math
from pathlib import Path

import cv2
import numpy as np

import utils
from utils import clusters_ij_to_world, gmap_occ_grid, grid_ij_to_world, pose_to_grid_ij

NEIGHBORS_8 = [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]

P_FREE = 0.35
P_OCC = 0.65


def cell_class(value):
    if value < P_FREE:
        return "free"
    if value > P_OCC:
        return "occupied"
    return "unknown"


def _neighbor_class(grid_map: np.ndarray, i: int, j: int, dx: int, dy: int) -> str | None:
    ni, nj = i + dx, j + dy
    if ni < 0 or nj < 0 or ni >= grid_map.shape[0] or nj >= grid_map.shape[1]:
        return None
    return cell_class(grid_map[ni, nj])


def is_frontier(grid_map: np.ndarray, i: int, j: int) -> bool:
    """Free cell with at least one unknown neighbor (8-connectivity)."""
    if cell_class(grid_map[i, j]) != "free":
        return False
    for dx, dy in NEIGHBORS_8:
        nc = _neighbor_class(grid_map, i, j, dx, dy)
        if nc == "unknown":
            return True
    return False


def find_frontiers(grid_map: np.ndarray) -> set[tuple[int, int]]:
    frontiers = set()
    for i in range(grid_map.shape[0]):
        for j in range(grid_map.shape[1]):
            if is_frontier(grid_map, i, j):
                frontiers.add((i, j))
    return frontiers


def connected_components(frontiers: set[tuple[int, int]]) -> list[list[tuple[int, int]]]:
    if not frontiers:
        return []
    visited: set[tuple[int, int]] = set()
    clusters: list[list[tuple[int, int]]] = []

    for cell in frontiers:
        if cell in visited:
            continue
        visited.add(cell)
        cluster: list[tuple[int, int]] = []
        queue = [cell]
        while queue:
            current = queue.pop(0)
            cluster.append(current)
            for dx, dy in NEIGHBORS_8:
                neighbor = (current[0] + dx, current[1] + dy)
                if neighbor in frontiers and neighbor not in visited:
                    queue.append(neighbor)
                    visited.add(neighbor)
        clusters.append(cluster)
    return clusters


def calc_center(cluster: list[tuple[int, int]]) -> tuple[float, float] | None:
    if not cluster:
        return None
    sum_i = sum(c[0] for c in cluster)
    sum_j = sum(c[1] for c in cluster)
    n = len(cluster)
    return (sum_i / n, sum_j / n)


def select_goal(
    pose: tuple[float, ...], clusters: list[list[tuple[int, int]]]
) -> tuple[float, float] | None:
    """Nearest cluster centroid; pose may be (row, col) or (row, col, theta)."""
    if not clusters:
        return None
    pos = np.array(pose[:2], dtype=float)
    min_dist = math.inf
    best_cluster: list[tuple[int, int]] | None = None
    for cluster in clusters:
        center = calc_center(cluster)
        if center is None:
            continue
        dist = float(np.linalg.norm(pos - np.array(center)))
        if dist < min_dist:
            min_dist = dist
            best_cluster = cluster
    return calc_center(best_cluster) if best_cluster is not None else None


class FrontierExplorer:
    def __init__(self, min_cluster_size: int = 3):
        self.min_cluster_size = min_cluster_size
        self.goal = None
        self.clusters = None

    def update(self, pose, gmap):
        occ, origin = gmap_occ_grid(gmap)
        if occ is None:
            return None, None
        pose_on_grid = pose_to_grid_ij(pose, gmap, origin)

        frontiers = find_frontiers(occ)
        clusters = [c for c in connected_components(frontiers) if len(c) >= self.min_cluster_size]
        self.goal = select_goal(pose_on_grid, clusters)
        self.clusters = clusters
        return (
            grid_ij_to_world(self.goal, origin, gmap.gsize),
            clusters_ij_to_world(self.clusters, origin, gmap.gsize),
        )


def simple_test():
    grid_map = np.array(
        [
            [0.5, 0.5, 0.5, 0.5, 0.5],
            [0.5, 0, 0, 0, 0.5],
            [0.5, 0, 0, 0, 0.5],
            [0.5, 0, 0, 0, 1],
            [0.5, 0.5, 1, 1, 1],
        ]
    )
    frontiers = find_frontiers(grid_map)
    print(frontiers)  # {(1, 2), (2, 1), (3, 1), (1, 1), (2, 3), (3, 3), (3, 2), (1, 3)}
    clusters = connected_components(frontiers)
    print(clusters)  # [[(1, 2), (2, 1), (3, 1), (1, 1), (2, 3), (3, 3), (3, 2), (1, 3)], [(2, 2)]]
    pose = (2, 2, 0.0)
    goal = select_goal(pose, clusters)
    print(goal)  # (2.0, 2.0)


def test_explorer():
    cv2.namedWindow("map", cv2.WINDOW_AUTOSIZE)
    map_path = Path(__file__).resolve().parent / "map.png"
    img_map = utils.Image2Map(map_path)
    pose = (150.0, 100.0, 0.0)
    explorer = FrontierExplorer()
    goal, clusters = explorer.update(pose, img_map)

    cv2.circle(img_map, (goal[0], goal[1]), 5, (0, 0, 255), -1)
    for cluster in clusters:
        for cell in cluster:
            cv2.circle(img_map, (cell[0], cell[1]), 1, (0, 255, 0), -1)

    cv2.imshow("map", img_map)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    simple_test()
    test_explorer()
