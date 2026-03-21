import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    tb3_nav2_dir = get_package_share_directory("turtlebot3_navigation2")
    tb3_slam_dir = get_package_share_directory("turtlebot3_cartographer")

    tb3_nav2_launch_file = os.path.join(tb3_nav2_dir, "launch", "navigation2.launch.py")
    tb3_slam_launch_file = os.path.join(
        tb3_slam_dir, "launch", "cartographer.launch.py"
    )

    g3nav2_dir = get_package_share_directory("g3nav2")
    nav2_params_file = os.path.join(g3nav2_dir, "nav2", "g3nav2.yaml")

    tb3_slam_launch_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb3_slam_launch_file),
        launch_arguments={"use_rviz": "false"}.items(),
    )

    tb3_nav2_launch_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb3_nav2_launch_file),
        launch_arguments={"params_file": nav2_params_file, "map": ""}.items(),
    )

    ld = LaunchDescription()

    ld.add_action(tb3_slam_launch_action)
    ld.add_action(tb3_nav2_launch_action)

    return ld
