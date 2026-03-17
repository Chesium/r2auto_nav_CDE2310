import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    tb3_simulation_launch_file = os.path.join(
        nav2_bringup_dir, "launch", "tb3_simulation_launch.py"
    )

    tb3_simulation_launch_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb3_simulation_launch_file),
        launch_arguments={
            "headless": "False",
            "slam": "True",
            "map": "",
        }.items(),
    )

    ld = LaunchDescription()

    ld.add_action(tb3_simulation_launch_action)

    return ld