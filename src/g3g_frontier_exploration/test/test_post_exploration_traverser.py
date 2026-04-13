from types import SimpleNamespace

import pytest

pytest.importorskip("geometry_msgs.msg")

import g3g_frontier_exploration.post_exploration_traverser as traverser_module
from g3g_frontier_exploration.frontier_utils import ExpiringBlacklist
from g3g_frontier_exploration.frontier_utils import make_goal_key
from g3g_frontier_exploration.post_exploration_traverser import (
    PostExplorationTraverser,
)


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


def _build_traverser(current_map):
    traverser = PostExplorationTraverser.__new__(PostExplorationTraverser)
    traverser._current_map = current_map
    traverser._goal_blacklist = ExpiringBlacklist()
    traverser.occupied_threshold = 56
    traverser.min_goal_distance = 0.5
    traverser.sample_spacing = 1.0
    traverser.revisit_position_resolution = 1.0
    traverser.candidate_clearance_radius = 1
    traverser.clearance_lookup_radius = 3
    traverser.viewpoint_window_radius_cells = 2
    traverser.unknown_view_weight = 2.5
    traverser.occupied_view_weight = 1.0
    traverser.clearance_weight = 0.35
    traverser.path_length_weight = 0.8
    traverser.max_path_evaluations = 10
    traverser.global_frame = "map"
    traverser.heading_search_radius_cells = 3
    traverser._now_seconds = lambda: 0.0
    traverser._goal_yaw_for_cell = lambda map_meta, cell_index, robot_xy: 0.0
    traverser._make_pose_stamped = lambda x, y, yaw: SimpleNamespace(
        pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y))
    )
    traverser.getPath = (
        lambda robot_pose, goal_pose, use_start=True: SimpleNamespace(
            poses=[
                SimpleNamespace(
                    pose=SimpleNamespace(
                        position=SimpleNamespace(
                            x=robot_pose.pose.position.x,
                            y=robot_pose.pose.position.y,
                        )
                    )
                ),
                goal_pose,
            ]
        )
    )
    return traverser


def test_select_goal_pose_skips_blacklisted_candidate(monkeypatch):
    traverser = _build_traverser(_make_map([0] * 25, width=5, height=5))
    robot_pose = _make_pose(0.5, 0.5)
    preferred_cell = 7
    fallback_cell = 18

    traverser._goal_blacklist.add(
        make_goal_key(2.5, 1.5, traverser.revisit_position_resolution),
        0.0,
        100.0,
    )

    monkeypatch.setattr(
        traverser_module,
        "extract_connected_free_component",
        lambda *args, **kwargs: SimpleNamespace(cells=(preferred_cell, fallback_cell)),
    )
    monkeypatch.setattr(
        traverser_module,
        "compute_clearance_to_occupied",
        lambda *args, **kwargs: 4,
    )
    monkeypatch.setattr(
        traverser_module,
        "count_cell_types_in_window",
        lambda data, meta, cell, radius, occupied_threshold: (
            0,
            6 if cell == preferred_cell else 5,
            4 if cell == preferred_cell else 2,
        ),
    )

    goal_pose = traverser._select_goal_pose(robot_pose)

    assert goal_pose is not None
    assert goal_pose.pose.position.x == 3.5
    assert goal_pose.pose.position.y == 3.5


def test_completion_callback_enables_and_pauses_traversal():
    traverser = PostExplorationTraverser.__new__(PostExplorationTraverser)
    traverser.autostart_after_exploration_complete = True
    traverser._enabled = False
    traverser._cancel_active_goal_requested = False
    traverser._frontier_reactivation_counter = 0
    traverser.info = lambda message: None

    traverser._completion_callback(SimpleNamespace(data=True))
    assert traverser._enabled is True

    traverser._completion_callback(SimpleNamespace(data=False))
    assert traverser._cancel_active_goal_requested is True
