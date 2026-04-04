from g3g_frontier_exploration.frontier_utils import count_unknown_cells_near_point
from g3g_frontier_exploration.frontier_utils import evaluate_component_encapsulation
from g3g_frontier_exploration.frontier_utils import ExpiringBlacklist
from g3g_frontier_exploration.frontier_utils import evaluate_frontier_cell
from g3g_frontier_exploration.frontier_utils import GridMeta
from g3g_frontier_exploration.frontier_utils import cluster_frontier_cells
from g3g_frontier_exploration.frontier_utils import detect_frontier_cells
from g3g_frontier_exploration.frontier_utils import extract_frontier_clusters
from g3g_frontier_exploration.frontier_utils import make_cluster_key
from g3g_frontier_exploration.frontier_utils import make_goal_key
from g3g_frontier_exploration.frontier_utils import select_cluster_goal_cell
from g3g_frontier_exploration.frontier_utils import snap_centroid_to_cluster_cell


def test_detect_frontier_cells_uses_free_cells_with_unknown_neighbors():
    meta = GridMeta(width=5, height=5, resolution=1.0, origin_x=0.0, origin_y=0.0)
    data = [
        100, 100, 100, 100, 100,
        100, 0, -1, 0, 100,
        100, 0, 0, 0, 100,
        100, 100, 100, 100, 100,
        100, 100, 100, 100, 100,
    ]

    frontiers = detect_frontier_cells(data, meta, occupied_threshold=50)

    assert frontiers == {6, 8, 12}


def test_cluster_frontiers_and_drop_small_clusters():
    meta = GridMeta(width=6, height=5, resolution=1.0, origin_x=0.0, origin_y=0.0)
    data = [
        100, 100, 100, 100, 100, 100,
        100, 0, -1, 100, 0, -1,
        100, 0, 0, 100, 100, 100,
        100, 0, 0, 100, 100, 100,
        100, 100, 100, 100, 100, 100,
    ]

    frontiers = detect_frontier_cells(data, meta, occupied_threshold=50)
    clusters = cluster_frontier_cells(frontiers, meta)
    large_clusters = extract_frontier_clusters(
        data,
        meta,
        occupied_threshold=50,
        min_cluster_size=2,
        window_radius_cells=1,
        min_frontier_free_neighbors=1,
        max_frontier_occupied_ratio=1.0,
        min_reachable_unknown_cells=1,
        min_unknown_span=1,
        candidate_clearance_radius=0,
    )

    assert len(clusters) == 2
    assert sorted(len(cluster) for cluster in clusters) == [1, 2]
    assert len(large_clusters) == 1
    assert large_clusters[0].size == 2


def test_snap_centroid_to_nearest_cluster_cell():
    meta = GridMeta(width=5, height=5, resolution=1.0, origin_x=0.0, origin_y=0.0)
    cluster = (6, 7, 11, 12)

    snapped = snap_centroid_to_cluster_cell(cluster, meta)

    assert snapped == 6


def test_blacklist_entries_expire_and_candidates_become_selectable_again():
    blacklist = ExpiringBlacklist()
    candidate_key = make_goal_key(1.0, 2.0)

    blacklist.add(candidate_key, now=10.0, ttl=5.0, payload=(1.0, 2.0))

    assert blacklist.contains(candidate_key, now=12.0) is True
    assert candidate_key in blacklist.active_entries(now=12.0)
    assert blacklist.contains(candidate_key, now=16.0) is False
    assert blacklist.active_entries(now=16.0) == {}


def test_select_cluster_goal_cell_falls_back_to_another_cell_when_needed():
    meta = GridMeta(width=8, height=6, resolution=1.0, origin_x=0.0, origin_y=0.0)
    cluster = (18, 19, 20, 21)

    selected = select_cluster_goal_cell(
        cluster,
        meta,
        robot_xy=(3.5, 2.5),
        min_goal_distance=1.1,
    )

    assert selected == 21


def test_count_unknown_cells_near_point_counts_only_cells_within_radius():
    meta = GridMeta(width=5, height=5, resolution=1.0, origin_x=0.0, origin_y=0.0)
    data = [
        100, 100, 100, 100, 100,
        100, -1, -1, 0, 100,
        100, -1, 0, 0, 100,
        100, 0, 0, 0, 100,
        100, 100, 100, 100, 100,
    ]

    count = count_unknown_cells_near_point(
        data,
        meta,
        center_xy=(2.5, 2.5),
        radius=1.6,
    )

    assert count == 3


def test_evaluate_frontier_cell_accepts_a_valid_corridor_frontier():
    meta = GridMeta(width=7, height=7, resolution=1.0, origin_x=0.0, origin_y=0.0)
    data = [
        100, 100, 100, 100, 100, 100, 100,
        100, 0, 0, 0, -1, -1, 100,
        100, 0, 0, 0, -1, -1, 100,
        100, 0, 0, 0, -1, -1, 100,
        100, 0, 0, 0, -1, -1, 100,
        100, 0, 0, 0, -1, -1, 100,
        100, 100, 100, 100, 100, 100, 100,
    ]

    metrics = evaluate_frontier_cell(
        data,
        meta,
        cell_index=24,
        occupied_threshold=50,
        window_radius_cells=2,
        min_frontier_free_neighbors=3,
        max_frontier_occupied_ratio=0.45,
        min_reachable_unknown_cells=3,
        min_unknown_span=2,
        candidate_clearance_radius=1,
    )

    assert metrics.is_valid is True
    assert metrics.reachable_unknown_cells >= 3
    assert metrics.max_unknown_span >= 2


def test_extract_frontier_clusters_rejects_single_cell_wall_leaks():
    meta = GridMeta(width=7, height=7, resolution=1.0, origin_x=0.0, origin_y=0.0)
    data = [
        100, 100, 100, 100, 100, 100, 100,
        100, 0, 0, 0, 100, 100, 100,
        100, 0, 0, 0, 100, 100, 100,
        100, 0, 0, 0, -1, 100, 100,
        100, 0, 0, 0, 100, 100, 100,
        100, 0, 0, 0, 100, 100, 100,
        100, 100, 100, 100, 100, 100, 100,
    ]

    clusters = extract_frontier_clusters(
        data,
        meta,
        occupied_threshold=50,
        min_cluster_size=1,
        window_radius_cells=2,
        min_frontier_free_neighbors=3,
        max_frontier_occupied_ratio=0.45,
        min_reachable_unknown_cells=3,
        min_unknown_span=2,
        candidate_clearance_radius=1,
    )

    assert clusters == []


def test_make_cluster_key_groups_nearby_cells_from_the_same_leak_region():
    meta = GridMeta(width=20, height=20, resolution=0.05, origin_x=0.0, origin_y=0.0)
    cluster = (210, 211, 212, 230, 231, 232)

    key_a = make_cluster_key(cluster, meta, representative_cell=211, position_resolution=0.5)
    key_b = make_cluster_key(cluster, meta, representative_cell=212, position_resolution=0.5)

    assert key_a == key_b


def test_evaluate_component_encapsulation_accepts_enclosed_room_without_holes():
    meta = GridMeta(width=7, height=7, resolution=1.0, origin_x=0.0, origin_y=0.0)
    data = [
        100, 100, 100, 100, 100, 100, 100,
        100, 0, 0, 0, 0, 0, 100,
        100, 0, 0, 0, 0, 0, 100,
        100, 0, 0, 0, 0, 0, 100,
        100, 0, 0, 0, 0, 0, 100,
        100, 0, 0, 0, 0, 0, 100,
        100, 100, 100, 100, 100, 100, 100,
    ]

    result = evaluate_component_encapsulation(
        data,
        meta,
        robot_xy=(3.5, 3.5),
        occupied_threshold=45,
        max_unknown_holes=2,
        max_unknown_hole_cells=6,
    )

    assert result.is_enclosed is True
    assert result.touches_map_border is False
    assert result.unknown_hole_count == 0


def test_evaluate_component_encapsulation_allows_single_tiny_unknown_hole():
    meta = GridMeta(width=7, height=7, resolution=1.0, origin_x=0.0, origin_y=0.0)
    data = [
        100, 100, 100, 100, 100, 100, 100,
        100, 0, 0, 0, 0, 0, 100,
        100, 0, 0, 0, 0, 0, 100,
        100, 0, 0, 0, 0, -1, 100,
        100, 0, 0, 0, 0, 0, 100,
        100, 0, 0, 0, 0, 0, 100,
        100, 100, 100, 100, 100, 100, 100,
    ]

    result = evaluate_component_encapsulation(
        data,
        meta,
        robot_xy=(3.5, 3.5),
        occupied_threshold=45,
        max_unknown_holes=2,
        max_unknown_hole_cells=6,
    )

    assert result.is_enclosed is True
    assert result.unknown_hole_count == 1
    assert result.largest_unknown_hole_size == 1


def test_evaluate_component_encapsulation_rejects_doorway_sized_unknown_opening():
    meta = GridMeta(width=7, height=7, resolution=1.0, origin_x=0.0, origin_y=0.0)
    data = [
        100, 100, 100, -1, -1, -1, 100,
        100, 0, 0, 0, 0, 0, 100,
        100, 0, 0, 0, 0, 0, 100,
        100, 0, 0, 0, 0, -1, 100,
        100, 0, 0, 0, 0, -1, 100,
        100, 0, 0, 0, 0, -1, 100,
        100, 100, 100, 100, 100, 100, 100,
    ]

    result = evaluate_component_encapsulation(
        data,
        meta,
        robot_xy=(3.5, 3.5),
        occupied_threshold=45,
        max_unknown_holes=2,
        max_unknown_hole_cells=2,
    )

    assert result.is_enclosed is False
    assert result.unknown_hole_count >= 1
    assert result.largest_unknown_hole_size >= 3


def test_evaluate_component_encapsulation_rejects_component_touching_map_border():
    meta = GridMeta(width=5, height=5, resolution=1.0, origin_x=0.0, origin_y=0.0)
    data = [
        0, 0, 0, 100, 100,
        0, 0, 0, 100, 100,
        0, 0, 0, 100, 100,
        100, 100, 100, 100, 100,
        100, 100, 100, 100, 100,
    ]

    result = evaluate_component_encapsulation(
        data,
        meta,
        robot_xy=(1.5, 1.5),
        occupied_threshold=45,
        max_unknown_holes=2,
        max_unknown_hole_cells=6,
    )

    assert result.is_enclosed is False
    assert result.touches_map_border is True


def test_evaluate_component_encapsulation_rejects_too_many_tiny_holes():
    meta = GridMeta(width=9, height=9, resolution=1.0, origin_x=0.0, origin_y=0.0)
    data = [
        100, 100, 100, 100, 100, 100, 100, 100, 100,
        100, 0, 0, 0, 0, 0, 0, 0, 100,
        100, 0, 0, 0, 0, 0, 0, 0, 100,
        100, 0, -1, 0, 0, 0, -1, 0, 100,
        100, 0, 0, 0, 0, 0, 0, 0, 100,
        100, 0, 0, 0, 0, 0, 0, 0, 100,
        100, 0, -1, 0, 0, 0, -1, 0, 100,
        100, 0, 0, 0, 0, 0, 0, 0, 100,
        100, 100, 100, 100, 100, 100, 100, 100, 100,
    ]

    result = evaluate_component_encapsulation(
        data,
        meta,
        robot_xy=(4.5, 4.5),
        occupied_threshold=45,
        max_unknown_holes=2,
        max_unknown_hole_cells=2,
    )

    assert result.is_enclosed is False
    assert result.unknown_hole_count == 4
