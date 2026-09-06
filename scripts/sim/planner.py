from collections import deque

import heapq
import numpy as np

NEIGHBORS_8 = [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]
NEIGHBORS_4 = ((0, 1), (0, -1), (1, 0), (-1, 0))


def h(start, goal):
    return np.linalg.norm(np.array(start) - np.array(goal))


def dfs(history, start, goal):
    if start == goal:
        return [goal]
    return dfs(history, history[start], goal) + [start]


def astart(grid_map, start, goal):
    waypoints = []
    rows, cols = grid_map.shape
    start = (int(start[0]), int(start[1]))
    goal = (int(goal[0]), int(goal[1]))

    heap = []
    visited = set()
    history = {}
    count = 0
    g_score = {start: 0}
    heapq.heappush(heap, (g_score[start] + h(start, goal), count, start))
    count += 1

    while heap:
        _, _, current = heapq.heappop(heap)
        if current in visited:
            continue
        visited.add(current)
        if current == goal:
            waypoints = dfs(history, goal, start)
            break
        for dx, dy in NEIGHBORS_8:
            ni, nj = current[0] + dx, current[1] + dy
            if ni < 0 or nj < 0 or ni >= rows or nj >= cols:
                continue
            neighbor = (ni, nj)
            if grid_map[ni, nj] == 1:
                print("Wall at {} {}".format(neighbor, grid_map[ni, nj]))
                continue
            if neighbor in visited:
                continue

            g = g_score[current] + 1
            if g < g_score.get(neighbor, float("inf")):
                g_score[neighbor] = g
                f = g + h(neighbor, goal)
                heapq.heappush(heap, (f, count, neighbor))
                count += 1
                history[neighbor] = current

    return waypoints


def plan_path(gmap, pose, goal_xy, padding=20):
    """
    Для main: GridMap + pose/goal в world (x, y) → путь [(x, y), ...].
    """
    from frontier import P_FREE, gmap_occ_grid, grid_ij_to_world, pose_to_grid_ij

    if goal_xy is None:
        return []

    occ, origin = gmap_occ_grid(gmap, padding)
    if occ is None:
        return []

    grid = np.where(occ < P_FREE, 0, 1).astype(int)
    start_ij = pose_to_grid_ij(pose, gmap, origin)
    goal_ij = pose_to_grid_ij(goal_xy, gmap, origin)
    path_ij = astart(grid, start_ij, goal_ij)
    return [grid_ij_to_world(p, origin, gmap.gsize) for p in path_ij]


if __name__ == "__main__":
    grid_map = np.array(
        [
            [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
            [1, 0, 0, 0, 0, 0, 1, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 1, 0, 0, 1],
            [1, 0, 0, 1, 0, 0, 1, 0, 0, 1],
            [1, 0, 0, 1, 0, 0, 1, 0, 0, 1],
            [1, 0, 0, 1, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 1, 0, 0, 0, 0, 0, 1],
            [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        ]
    )
    pose = (6, 1)
    goal = (1, 8)
    waypoints = astart(grid_map, pose, goal)
    print(waypoints)
