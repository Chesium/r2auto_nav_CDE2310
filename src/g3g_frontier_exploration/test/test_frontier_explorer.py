from types import SimpleNamespace

import pytest

pytest.importorskip("geometry_msgs.msg")

import g3g_frontier_exploration.frontier_explorer as frontier_explorer_module
from g3g_frontier_exploration.frontier_explorer import FrontierExplorer
from g3g_frontier_exploration.frontier_utils import ExpiringBlacklist


def _make_map(data, width, height, resolution=1.0):
    return SimpleNamespace(
        data=data,
        info=SimpleNamespace(
            width=width,
            height=height,
            resolution=resolution,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0),
            ),
        ),
    )


def _make_pose(x, y):
    return SimpleNamespace(
        pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y),
        )
    )


def _build_explorer(current_map):
    explorer = FrontierExplorer.__new__(FrontierExplorer)
    explorer._enabled = True
    explorer._active_goal_pose = None
    explorer._current_map = current_map
    explorer._cluster_blacklist = ExpiringBlacklist()
    explorer._cluster_failure_counts = {}
    explorer._exhausted_clusters = set()
    explorer._encapsulation_cycles = 0
    explorer._no_frontier_cycles = 0
    explorer._active_cluster_key = None
    explorer._active_frontier_xy = None
    explorer._active_unknown_count_before = None
    explorer._active_cluster_cells = None
    explorer._active_goal_start_sec = None
    explorer.encapsulation_check_enabled = True
    explorer.encapsulation_occupied_threshold = 45
    explorer.encapsulation_confirmation_cycles = 2
    explorer.encapsulation_max_unknown_holes = 2
    explorer.encapsulation_max_unknown_hole_cells = 6
    explorer.occupied_threshold = 56
    explorer.frontier_min_cluster_size = 8
    explorer.frontier_window_radius_cells = 2
    explorer.min_frontier_free_neighbors = 3
    explorer.max_frontier_occupied_ratio = 0.45
    explorer.min_reachable_unknown_cells = 3
    explorer.min_frontier_unknown_span = 2
    explorer.candidate_clearance_radius = 1
    explorer.min_goal_distance = 0.8
    explorer.frontier_region_resolution = 0.5
    explorer.information_gain_radius = 0.6
    explorer.completion_patience_cycles = 5
    explorer.invalid_goal_blacklist_ttl_sec = 15.0
    explorer.cluster_blacklist_ttl_sec = 15.0
    explorer._readiness_state = lambda: (True, "ready")
    explorer._now_seconds = lambda: 0.0
    explorer._publish_markers = lambda clusters, active_goal: None
    explorer._completion_publisher = SimpleNamespace(publish=lambda msg: None)
    explorer.info = lambda message: None
    explorer.warn = lambda message: None
    explorer.debug = lambda message: None
    return explorer


def test_planning_tick_completes_when_encapsulation_is_confirmed():
    current_map = _make_map(
        [
            100, 100, 100, 100, 100, 100, 100,
            100, 0, 0, 0, 0, 0, 100,
            100, 0, 0, 0, 0, 0, 100,
            100, 0, 0, 0, 0, 0, 100,
            100, 0, 0, 0, 0, 0, 100,
            100, 0, 0, 0, 0, 0, 100,
            100, 100, 100, 100, 100, 100, 100,
        ],
        width=7,
        height=7,
    )
    explorer = _build_explorer(current_map)
    explorer._encapsulation_cycles = 1
    explorer._lookup_robot_pose = lambda: _make_pose(3.5, 3.5)
    calls = []
    explorer._disable_exploration = lambda message, clear_markers: calls.append(
        (message, clear_markers)
    )
    explorer.goToPose = lambda goal: (_ for _ in ()).throw(
        AssertionError("goToPose should not be called when encapsulation completes")
    )

    explorer._planning_tick()

    assert calls == [
        ("Reachable space is enclosed; treating exploration as complete.", True)
    ]


def test_planning_tick_continues_to_goal_selection_when_not_enclosed(monkeypatch):
    current_map = _make_map(
        [
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
        ],
        width=5,
        height=5,
    )
    explorer = _build_explorer(current_map)
    explorer._lookup_robot_pose = lambda: _make_pose(2.5, 2.5)
    explorer._select_valid_goal_cell = lambda cluster_cells, preferred_cell, map_meta: 6
    explorer._path_length = lambda path: 1.0
    explorer._make_pose_stamped = lambda x, y: SimpleNamespace(
        pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y))
    )
    published_goals = []
    explorer._goal_publisher = SimpleNamespace(publish=lambda msg: published_goals.append(msg))
    go_to_pose_calls = []
    explorer.goToPose = lambda goal: go_to_pose_calls.append(goal) or True
    explorer.getPath = lambda robot_pose, goal_pose, use_start=True: SimpleNamespace(
        poses=[SimpleNamespace(), SimpleNamespace()]
    )

    monkeypatch.setattr(
        frontier_explorer_module,
        "extract_frontier_clusters",
        lambda *args, **kwargs: [SimpleNamespace(cells=(6, 7, 11), size=3)],
    )
    monkeypatch.setattr(
        frontier_explorer_module,
        "select_cluster_goal_cell",
        lambda cluster, meta, robot_xy, min_goal_distance: 6,
    )
    monkeypatch.setattr(
        frontier_explorer_module,
        "make_cluster_key",
        lambda cluster, meta, selected_goal_cell, frontier_region_resolution: (1, 1, 1),
    )
    monkeypatch.setattr(
        frontier_explorer_module,
        "count_unknown_cells_near_point",
        lambda *args, **kwargs: 4,
    )

    explorer._planning_tick()

    assert explorer._encapsulation_cycles == 0
    assert len(go_to_pose_calls) == 1
    assert len(published_goals) == 1
    assert explorer._active_goal_pose is not None


def test_planning_tick_resets_encapsulation_counter_when_condition_clears(monkeypatch):
    current_map = _make_map(
        [
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
        ],
        width=5,
        height=5,
    )
    explorer = _build_explorer(current_map)
    explorer._encapsulation_cycles = 2
    explorer._lookup_robot_pose = lambda: _make_pose(2.5, 2.5)
    explorer._disable_exploration = lambda message, clear_markers: None

    monkeypatch.setattr(
        frontier_explorer_module,
        "extract_frontier_clusters",
        lambda *args, **kwargs: [],
    )

    explorer._planning_tick()

    assert explorer._encapsulation_cycles == 0


def test_complete_exploration_publishes_completion_signal():
    explorer = _build_explorer(_make_map([0], width=1, height=1))
    published = []
    disabled = []
    explorer._completion_publisher = SimpleNamespace(
        publish=lambda msg: published.append(msg.data)
    )
    explorer._disable_exploration = lambda message, clear_markers: disabled.append(
        (message, clear_markers)
    )

    explorer._complete_exploration("Exploration completed successfully.", True)

    assert published == [True]
    assert disabled == [("Exploration completed successfully.", True)]
