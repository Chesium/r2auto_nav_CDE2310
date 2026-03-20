from g3g_frontier_exploration.frontier_utils import ExpiringBlacklist
from g3g_frontier_exploration.frontier_utils import GridMeta
from g3g_frontier_exploration.frontier_utils import cluster_frontier_cells
from g3g_frontier_exploration.frontier_utils import detect_frontier_cells
from g3g_frontier_exploration.frontier_utils import extract_frontier_clusters
from g3g_frontier_exploration.frontier_utils import make_goal_key
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
