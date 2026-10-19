"""Stage 1.3: A* from the spawn centre to the first goal centre."""

from __future__ import annotations

import heapq

import numpy as np

from stage1_sim.course import CourseMap

NEIGHBORS = (
    (1, 0, 1.0),
    (-1, 0, 1.0),
    (0, 1, 1.0),
    (0, -1, 1.0),
    (1, 1, 1.41421356237),
    (1, -1, 1.41421356237),
    (-1, 1, 1.41421356237),
    (-1, -1, 1.41421356237),
)


def plan_path(course: CourseMap, start_xy: tuple[float, float], goal_xy: tuple[float, float], robot_radius: float = 0.13) -> np.ndarray:
    """Return world-frame waypoints. Empty array means no free path."""
    blocked = course.inflate(robot_radius)
    si, sj = _nearest_free(course, blocked, *course.world_to_grid(*start_xy))
    gi, gj = _nearest_free(course, blocked, *course.world_to_grid(*goal_xy))
    if si is None or gi is None:
        return np.zeros((0, 2))

    ny, nx = blocked.shape
    gscore = np.full((ny, nx), np.inf)
    parent = np.full((ny, nx, 2), -1, dtype=int)
    gscore[sj, si] = 0.0
    heap = [(float(np.hypot(gi - si, gj - sj)), 0.0, si, sj)]
    closed = np.zeros((ny, nx), dtype=bool)

    while heap:
        _, g, i, j = heapq.heappop(heap)
        if closed[j, i]:
            continue
        closed[j, i] = True
        if i == gi and j == gj:
            return _reconstruct(course, parent, si, sj, gi, gj)
        for di, dj, step in NEIGHBORS:
            ni, nj = i + di, j + dj
            if not (0 <= ni < nx and 0 <= nj < ny) or closed[nj, ni] or blocked[nj, ni]:
                continue
            ng = g + step
            if ng < gscore[nj, ni]:
                gscore[nj, ni] = ng
                parent[nj, ni] = (i, j)
                h = float(np.hypot(gi - ni, gj - nj))
                heapq.heappush(heap, (ng + h, ng, ni, nj))
    return np.zeros((0, 2))


def path_stays_free(course: CourseMap, path: np.ndarray) -> bool:
    if len(path) == 0:
        return False
    for x, y in path:
        i, j = course.world_to_grid(float(x), float(y))
        if course.occupancy[j, i]:
            return False
    return True


def _nearest_free(course: CourseMap, blocked: np.ndarray, i: int, j: int, radius: int = 8) -> tuple[int, int] | tuple[None, None]:
    ny, nx = blocked.shape
    if not blocked[j, i]:
        return i, j
    best = None
    best_d = 1e9
    for dj in range(-radius, radius + 1):
        for di in range(-radius, radius + 1):
            ni, nj = i + di, j + dj
            if 0 <= ni < nx and 0 <= nj < ny and not blocked[nj, ni]:
                d = di * di + dj * dj
                if d < best_d:
                    best_d = d
                    best = (ni, nj)
    return best if best else (None, None)


def _reconstruct(course: CourseMap, parent: np.ndarray, si: int, sj: int, gi: int, gj: int) -> np.ndarray:
    points = []
    i, j = gi, gj
    seen = 0
    while not (i == si and j == sj):
        points.append(course.grid_to_world(i, j))
        pi, pj = parent[j, i]
        if pi < 0 or seen > course.nx * course.ny:
            return np.zeros((0, 2))
        i, j = int(pi), int(pj)
        seen += 1
    points.append(course.grid_to_world(si, sj))
    points.reverse()
    return np.asarray(points, dtype=float)
