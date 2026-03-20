"""Pure-Python helpers for frontier detection and goal filtering."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Iterable, Sequence


@dataclass(frozen=True)
class GridMeta:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float


@dataclass(frozen=True)
class FrontierCluster:
    cells: tuple[int, ...]
    goal_cell: int
    goal_xy: tuple[float, float]
    size: int


def cell_to_index(row: int, col: int, width: int) -> int:
    return (row * width) + col


def index_to_cell(index: int, width: int) -> tuple[int, int]:
    return divmod(index, width)


def index_to_world(meta: GridMeta, index: int) -> tuple[float, float]:
    row, col = index_to_cell(index, meta.width)
    x = meta.origin_x + ((col + 0.5) * meta.resolution)
    y = meta.origin_y + ((row + 0.5) * meta.resolution)
    return (x, y)


def _neighbors4(index: int, meta: GridMeta) -> list[int]:
    row, col = index_to_cell(index, meta.width)
    neighbors = []
    if row > 0:
        neighbors.append(cell_to_index(row - 1, col, meta.width))
    if row < (meta.height - 1):
        neighbors.append(cell_to_index(row + 1, col, meta.width))
    if col > 0:
        neighbors.append(cell_to_index(row, col - 1, meta.width))
    if col < (meta.width - 1):
        neighbors.append(cell_to_index(row, col + 1, meta.width))
    return neighbors


def _neighbors8(index: int, meta: GridMeta) -> list[int]:
    row, col = index_to_cell(index, meta.width)
    neighbors = []
    for row_offset in (-1, 0, 1):
        for col_offset in (-1, 0, 1):
            if row_offset == 0 and col_offset == 0:
                continue
            next_row = row + row_offset
            next_col = col + col_offset
            if 0 <= next_row < meta.height and 0 <= next_col < meta.width:
                neighbors.append(cell_to_index(next_row, next_col, meta.width))
    return neighbors


def _is_free(value: int, occupied_threshold: int) -> bool:
    return 0 <= value < occupied_threshold


def detect_frontier_cells(
    data: Sequence[int], meta: GridMeta, occupied_threshold: int
) -> set[int]:
    frontier_cells = set()
    for index, value in enumerate(data):
        if not _is_free(value, occupied_threshold):
            continue
        if any(data[neighbor] == -1 for neighbor in _neighbors4(index, meta)):
            frontier_cells.add(index)
    return frontier_cells


def cluster_frontier_cells(
    frontier_cells: Iterable[int], meta: GridMeta
) -> list[tuple[int, ...]]:
    remaining = set(frontier_cells)
    clusters: list[tuple[int, ...]] = []

    while remaining:
        start = remaining.pop()
        cluster = [start]
        queue = deque([start])

        while queue:
            current = queue.popleft()
            for neighbor in _neighbors8(current, meta):
                if neighbor not in remaining:
                    continue
                remaining.remove(neighbor)
                cluster.append(neighbor)
                queue.append(neighbor)

        clusters.append(tuple(sorted(cluster)))

    return clusters


def snap_centroid_to_cluster_cell(cluster: Sequence[int], meta: GridMeta) -> int:
    centroid_row = 0.0
    centroid_col = 0.0

    for cell in cluster:
        row, col = index_to_cell(cell, meta.width)
        centroid_row += row
        centroid_col += col

    centroid_row /= len(cluster)
    centroid_col /= len(cluster)

    return min(
        cluster,
        key=lambda cell: (
            (index_to_cell(cell, meta.width)[0] - centroid_row) ** 2
            + (index_to_cell(cell, meta.width)[1] - centroid_col) ** 2,
            cell,
        ),
    )


def select_cluster_goal_cell(
    cluster: Sequence[int],
    meta: GridMeta,
    robot_xy: tuple[float, float],
    min_goal_distance: float,
) -> int:
    """Fallback from the centroid representative to another cell in the same cluster if needed."""

    centroid_cell = snap_centroid_to_cluster_cell(cluster, meta)
    centroid_xy = index_to_world(meta, centroid_cell)

    if euclidean_distance(robot_xy, centroid_xy) >= min_goal_distance:
        return centroid_cell

    eligible_cells = []
    for cell in cluster:
        cell_xy = index_to_world(meta, cell)
        distance = euclidean_distance(robot_xy, cell_xy)
        if distance >= min_goal_distance:
            eligible_cells.append((distance, cell))

    if not eligible_cells:
        return centroid_cell

    return min(eligible_cells, key=lambda item: (item[0], item[1]))[1]


def count_unknown_cells_near_point(
    data: Sequence[int],
    meta: GridMeta,
    center_xy: tuple[float, float],
    radius: float,
) -> int:
    radius_cells = max(0, int(math.ceil(radius / meta.resolution)))
    center_col = int((center_xy[0] - meta.origin_x) / meta.resolution)
    center_row = int((center_xy[1] - meta.origin_y) / meta.resolution)
    count = 0

    for row in range(center_row - radius_cells, center_row + radius_cells + 1):
        if row < 0 or row >= meta.height:
            continue
        for col in range(center_col - radius_cells, center_col + radius_cells + 1):
            if col < 0 or col >= meta.width:
                continue
            index = cell_to_index(row, col, meta.width)
            cell_xy = index_to_world(meta, index)
            if euclidean_distance(center_xy, cell_xy) <= radius and data[index] == -1:
                count += 1

    return count


def extract_frontier_clusters(
    data: Sequence[int],
    meta: GridMeta,
    occupied_threshold: int,
    min_cluster_size: int,
) -> list[FrontierCluster]:
    frontier_cells = detect_frontier_cells(data, meta, occupied_threshold)
    clusters = cluster_frontier_cells(frontier_cells, meta)
    results = []

    for cluster in clusters:
        if len(cluster) < min_cluster_size:
            continue
        goal_cell = snap_centroid_to_cluster_cell(cluster, meta)
        results.append(
            FrontierCluster(
                cells=cluster,
                goal_cell=goal_cell,
                goal_xy=index_to_world(meta, goal_cell),
                size=len(cluster),
            )
        )

    return results


def euclidean_distance(point_a: tuple[float, float], point_b: tuple[float, float]) -> float:
    return math.hypot(point_a[0] - point_b[0], point_a[1] - point_b[1])


def make_goal_key(x: float, y: float, resolution: float = 0.05) -> tuple[int, int]:
    return (
        int(round(x / resolution)),
        int(round(y / resolution)),
    )


class ExpiringBlacklist:
    """Tracks temporarily blocked goal keys with optional payloads."""

    def __init__(self) -> None:
        self._entries: dict[tuple[int, int], tuple[float, object | None]] = {}

    def add(
        self,
        key: tuple[int, int],
        now: float,
        ttl: float,
        payload: object | None = None,
    ) -> None:
        self._entries[key] = (now + ttl, payload)

    def contains(self, key: tuple[int, int], now: float) -> bool:
        self.prune(now)
        return key in self._entries

    def prune(self, now: float) -> None:
        expired = [key for key, (expires_at, _) in self._entries.items() if now >= expires_at]
        for key in expired:
            self._entries.pop(key, None)

    def active_entries(self, now: float) -> dict[tuple[int, int], object | None]:
        self.prune(now)
        return {key: payload for key, (_, payload) in self._entries.items()}
