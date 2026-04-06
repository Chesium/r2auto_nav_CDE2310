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


@dataclass(frozen=True)
class FrontierCellMetrics:
    is_valid: bool
    free_neighbors: int
    occupied_ratio: float
    reachable_unknown_cells: int
    max_unknown_span: int
    clearance_cells: int


@dataclass(frozen=True)
class FreeSpaceComponent:
    cells: tuple[int, ...]
    touches_map_border: bool


@dataclass(frozen=True)
class EncapsulationResult:
    is_enclosed: bool
    touches_map_border: bool
    unknown_hole_count: int
    largest_unknown_hole_size: int
    component_size: int


def cell_to_index(row: int, col: int, width: int) -> int:
    return (row * width) + col


def index_to_cell(index: int, width: int) -> tuple[int, int]:
    return divmod(index, width)


def index_to_world(meta: GridMeta, index: int) -> tuple[float, float]:
    row, col = index_to_cell(index, meta.width)
    x = meta.origin_x + ((col + 0.5) * meta.resolution)
    y = meta.origin_y + ((row + 0.5) * meta.resolution)
    return (x, y)


def world_to_index(meta: GridMeta, x: float, y: float) -> int | None:
    col = int((x - meta.origin_x) / meta.resolution)
    row = int((y - meta.origin_y) / meta.resolution)
    if row < 0 or row >= meta.height or col < 0 or col >= meta.width:
        return None
    return cell_to_index(row, col, meta.width)


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


def _window_cell_indices(center_index: int, meta: GridMeta, radius_cells: int) -> list[int]:
    center_row, center_col = index_to_cell(center_index, meta.width)
    indices = []
    for row in range(center_row - radius_cells, center_row + radius_cells + 1):
        if row < 0 or row >= meta.height:
            continue
        for col in range(center_col - radius_cells, center_col + radius_cells + 1):
            if col < 0 or col >= meta.width:
                continue
            indices.append(cell_to_index(row, col, meta.width))
    return indices


def count_cell_types_in_window(
    data: Sequence[int],
    meta: GridMeta,
    center_index: int,
    radius_cells: int,
    occupied_threshold: int,
) -> tuple[int, int, int]:
    free_count = 0
    occupied_count = 0
    unknown_count = 0

    for index in _window_cell_indices(center_index, meta, radius_cells):
        value = data[index]
        if value == -1:
            unknown_count += 1
        elif _is_free(value, occupied_threshold):
            free_count += 1
        else:
            occupied_count += 1

    return (free_count, occupied_count, unknown_count)


def _reachable_free_component(
    data: Sequence[int],
    meta: GridMeta,
    start_index: int,
    occupied_threshold: int,
    radius_cells: int,
) -> set[int]:
    allowed = set(_window_cell_indices(start_index, meta, radius_cells))
    if start_index not in allowed or not _is_free(data[start_index], occupied_threshold):
        return set()

    reachable = {start_index}
    queue = deque([start_index])

    while queue:
        current = queue.popleft()
        for neighbor in _neighbors8(current, meta):
            if neighbor not in allowed or neighbor in reachable:
                continue
            if not _is_free(data[neighbor], occupied_threshold):
                continue
            reachable.add(neighbor)
            queue.append(neighbor)

    return reachable


def count_reachable_unknown_border_cells(
    data: Sequence[int],
    meta: GridMeta,
    center_index: int,
    occupied_threshold: int,
    radius_cells: int,
) -> int:
    reachable = _reachable_free_component(
        data,
        meta,
        center_index,
        occupied_threshold,
        radius_cells,
    )
    if not reachable:
        return 0

    unknown_border = set()
    for index in reachable:
        for neighbor in _neighbors4(index, meta):
            if data[neighbor] == -1:
                unknown_border.add(neighbor)

    return len(unknown_border)


def estimate_frontier_unknown_span(
    data: Sequence[int],
    meta: GridMeta,
    center_index: int,
    max_span_radius: int,
) -> int:
    row, col = index_to_cell(center_index, meta.width)
    spans = []
    directions = [
        (-1, 0, 0, 1),
        (1, 0, 0, 1),
        (0, -1, 1, 0),
        (0, 1, 1, 0),
    ]

    for delta_row, delta_col, perp_row, perp_col in directions:
        front_row = row + delta_row
        front_col = col + delta_col
        if not (0 <= front_row < meta.height and 0 <= front_col < meta.width):
            continue
        front_index = cell_to_index(front_row, front_col, meta.width)
        if data[front_index] != -1:
            continue

        span = 1
        for direction in (-1, 1):
            for step in range(1, max_span_radius + 1):
                scan_row = front_row + (perp_row * step * direction)
                scan_col = front_col + (perp_col * step * direction)
                if not (0 <= scan_row < meta.height and 0 <= scan_col < meta.width):
                    break
                scan_index = cell_to_index(scan_row, scan_col, meta.width)
                if data[scan_index] != -1:
                    break
                span += 1
        spans.append(span)

    return max(spans, default=0)


def compute_clearance_to_occupied(
    data: Sequence[int],
    meta: GridMeta,
    center_index: int,
    occupied_threshold: int,
    radius_cells: int,
) -> int:
    center_row, center_col = index_to_cell(center_index, meta.width)
    best_distance_sq: int | None = None

    for index in _window_cell_indices(center_index, meta, radius_cells):
        value = data[index]
        if value == -1 or _is_free(value, occupied_threshold):
            continue
        row, col = index_to_cell(index, meta.width)
        distance_sq = ((row - center_row) ** 2) + ((col - center_col) ** 2)
        if best_distance_sq is None or distance_sq < best_distance_sq:
            best_distance_sq = distance_sq

    if best_distance_sq is None:
        return radius_cells + 1

    return int(math.sqrt(best_distance_sq))


def evaluate_frontier_cell(
    data: Sequence[int],
    meta: GridMeta,
    cell_index: int,
    occupied_threshold: int,
    window_radius_cells: int,
    min_frontier_free_neighbors: int,
    max_frontier_occupied_ratio: float,
    min_reachable_unknown_cells: int,
    min_unknown_span: int,
    candidate_clearance_radius: int,
) -> FrontierCellMetrics:
    if not _is_free(data[cell_index], occupied_threshold):
        return FrontierCellMetrics(False, 0, 1.0, 0, 0, 0)

    unknown_neighbors = [neighbor for neighbor in _neighbors4(cell_index, meta) if data[neighbor] == -1]
    if not unknown_neighbors:
        return FrontierCellMetrics(False, 0, 1.0, 0, 0, 0)

    free_neighbors = sum(
        1 for neighbor in _neighbors8(cell_index, meta) if _is_free(data[neighbor], occupied_threshold)
    )
    free_count, occupied_count, _ = count_cell_types_in_window(
        data,
        meta,
        cell_index,
        window_radius_cells,
        occupied_threshold,
    )
    known_count = free_count + occupied_count
    occupied_ratio = occupied_count / known_count if known_count > 0 else 1.0
    reachable_unknown_cells = count_reachable_unknown_border_cells(
        data,
        meta,
        cell_index,
        occupied_threshold,
        window_radius_cells,
    )
    max_unknown_span = estimate_frontier_unknown_span(
        data,
        meta,
        cell_index,
        window_radius_cells,
    )
    clearance_cells = compute_clearance_to_occupied(
        data,
        meta,
        cell_index,
        occupied_threshold,
        window_radius_cells,
    )

    is_valid = (
        free_neighbors >= min_frontier_free_neighbors
        and occupied_ratio <= max_frontier_occupied_ratio
        and reachable_unknown_cells >= min_reachable_unknown_cells
        and max_unknown_span >= min_unknown_span
        and clearance_cells >= candidate_clearance_radius
    )
    return FrontierCellMetrics(
        is_valid=is_valid,
        free_neighbors=free_neighbors,
        occupied_ratio=occupied_ratio,
        reachable_unknown_cells=reachable_unknown_cells,
        max_unknown_span=max_unknown_span,
        clearance_cells=clearance_cells,
    )


def make_cluster_key(
    cluster: Sequence[int],
    meta: GridMeta,
    representative_cell: int,
    position_resolution: float,
) -> tuple[int, int, int]:
    del representative_cell

    centroid_x = 0.0
    centroid_y = 0.0
    for cell in cluster:
        cell_xy = index_to_world(meta, cell)
        centroid_x += cell_xy[0]
        centroid_y += cell_xy[1]

    representative_xy = (
        centroid_x / len(cluster),
        centroid_y / len(cluster),
    )
    max_extent = 0.0
    for cell in cluster:
        cell_xy = index_to_world(meta, cell)
        max_extent = max(max_extent, euclidean_distance(representative_xy, cell_xy))

    return (
        int(round(representative_xy[0] / position_resolution)),
        int(round(representative_xy[1] / position_resolution)),
        int(round(max_extent / meta.resolution)),
    )


def extract_connected_free_component(
    data: Sequence[int],
    meta: GridMeta,
    start_index: int,
    occupied_threshold: int,
) -> FreeSpaceComponent:
    if start_index < 0 or start_index >= len(data):
        return FreeSpaceComponent(cells=(), touches_map_border=False)
    if not _is_free(data[start_index], occupied_threshold):
        return FreeSpaceComponent(cells=(), touches_map_border=False)

    visited = {start_index}
    queue = deque([start_index])
    touches_map_border = False

    while queue:
        current = queue.popleft()
        row, col = index_to_cell(current, meta.width)
        if row == 0 or row == meta.height - 1 or col == 0 or col == meta.width - 1:
            touches_map_border = True

        for neighbor in _neighbors4(current, meta):
            if neighbor in visited or not _is_free(data[neighbor], occupied_threshold):
                continue
            visited.add(neighbor)
            queue.append(neighbor)

    return FreeSpaceComponent(
        cells=tuple(sorted(visited)),
        touches_map_border=touches_map_border,
    )


def cluster_unknown_boundary_cells(
    data: Sequence[int],
    meta: GridMeta,
    component_cells: Sequence[int],
) -> list[tuple[int, ...]]:
    boundary_unknowns = set()
    for cell in component_cells:
        for neighbor in _neighbors4(cell, meta):
            if data[neighbor] == -1:
                boundary_unknowns.add(neighbor)

    clusters: list[tuple[int, ...]] = []
    remaining = set(boundary_unknowns)

    while remaining:
        start = remaining.pop()
        cluster = [start]
        queue = deque([start])

        while queue:
            current = queue.popleft()
            for neighbor in _neighbors4(current, meta):
                if neighbor not in remaining:
                    continue
                remaining.remove(neighbor)
                cluster.append(neighbor)
                queue.append(neighbor)

        clusters.append(tuple(sorted(cluster)))

    return clusters


def evaluate_component_encapsulation(
    data: Sequence[int],
    meta: GridMeta,
    robot_xy: tuple[float, float],
    occupied_threshold: int,
    max_unknown_holes: int,
    max_unknown_hole_cells: int,
) -> EncapsulationResult:
    start_index = world_to_index(meta, robot_xy[0], robot_xy[1])
    if start_index is None:
        return EncapsulationResult(
            is_enclosed=False,
            touches_map_border=True,
            unknown_hole_count=0,
            largest_unknown_hole_size=0,
            component_size=0,
        )

    component = extract_connected_free_component(
        data,
        meta,
        start_index,
        occupied_threshold,
    )
    if not component.cells:
        return EncapsulationResult(
            is_enclosed=False,
            touches_map_border=False,
            unknown_hole_count=0,
            largest_unknown_hole_size=0,
            component_size=0,
        )

    unknown_clusters = cluster_unknown_boundary_cells(data, meta, component.cells)
    largest_unknown_hole_size = max((len(cluster) for cluster in unknown_clusters), default=0)
    unknown_hole_count = len(unknown_clusters)
    is_enclosed = (
        not component.touches_map_border
        and unknown_hole_count <= max_unknown_holes
        and largest_unknown_hole_size <= max_unknown_hole_cells
    )

    return EncapsulationResult(
        is_enclosed=is_enclosed,
        touches_map_border=component.touches_map_border,
        unknown_hole_count=unknown_hole_count,
        largest_unknown_hole_size=largest_unknown_hole_size,
        component_size=len(component.cells),
    )


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
    window_radius_cells: int = 2,
    min_frontier_free_neighbors: int = 3,
    max_frontier_occupied_ratio: float = 0.45,
    min_reachable_unknown_cells: int = 3,
    min_unknown_span: int = 2,
    candidate_clearance_radius: int = 1,
) -> list[FrontierCluster]:
    frontier_cells = detect_frontier_cells(data, meta, occupied_threshold)
    filtered_frontier_cells = {
        cell
        for cell in frontier_cells
        if evaluate_frontier_cell(
            data,
            meta,
            cell,
            occupied_threshold,
            window_radius_cells,
            min_frontier_free_neighbors,
            max_frontier_occupied_ratio,
            min_reachable_unknown_cells,
            min_unknown_span,
            candidate_clearance_radius,
        ).is_valid
    }
    clusters = cluster_frontier_cells(filtered_frontier_cells, meta)
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
        self._entries: dict[tuple[int, ...], tuple[float, object | None]] = {}

    def add(
        self,
        key: tuple[int, ...],
        now: float,
        ttl: float,
        payload: object | None = None,
    ) -> None:
        self._entries[key] = (now + ttl, payload)

    def contains(self, key: tuple[int, ...], now: float) -> bool:
        self.prune(now)
        return key in self._entries

    def prune(self, now: float) -> None:
        expired = [key for key, (expires_at, _) in self._entries.items() if now >= expires_at]
        for key in expired:
            self._entries.pop(key, None)

    def active_entries(self, now: float) -> dict[tuple[int, ...], object | None]:
        self.prune(now)
        return {key: payload for key, (_, payload) in self._entries.items()}
