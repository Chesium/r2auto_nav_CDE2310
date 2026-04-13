import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("g3g_frontier_exploration")
    default_params_file = os.path.join(package_dir, "config", "frontier_exploration.yaml")

    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_post_traversal = LaunchConfiguration("use_post_traversal")

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Path to the frontier exploration parameters file.",
    )
    declare_autostart = DeclareLaunchArgument(
        "autostart",
        default_value="false",
        description="Start exploring as soon as the explorer is ready.",
    )
    declare_use_post_traversal = DeclareLaunchArgument(
        "use_post_traversal",
        default_value="true",
        description="Start the post-exploration traverser node.",
    )

    frontier_node = Node(
        package="g3g_frontier_exploration",
        executable="frontier_explorer",
        name="frontier_explorer",
        output="screen",
        parameters=[
            params_file,
            {
                "autostart": autostart,
            },
        ],
    )
    post_traversal_node = Node(
        condition=IfCondition(use_post_traversal),
        package="g3g_frontier_exploration",
        executable="post_exploration_traverser",
        name="post_exploration_traverser",
        output="screen",
        parameters=[params_file],
    )

    return LaunchDescription(
        [
            declare_params_file,
            declare_autostart,
            declare_use_post_traversal,
            frontier_node,
            post_traversal_node,
        ]
    )
