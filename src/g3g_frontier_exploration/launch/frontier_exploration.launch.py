import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("g3g_frontier_exploration")
    default_params_file = os.path.join(package_dir, "config", "frontier_exploration.yaml")

    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")

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

    return LaunchDescription(
        [
            declare_params_file,
            declare_autostart,
            frontier_node,
        ]
    )
