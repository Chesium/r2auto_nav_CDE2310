import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# colcon build --packages-select g3gzsim && roset && ros2 launch g3gzsim nav2full_test_launch.py

def generate_launch_description():

    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    tb3_simulation_launch_file = os.path.join(
        nav2_bringup_dir, "launch", "tb3_simulation_launch.py"
    )

    g3gzsim_dir = get_package_share_directory("g3gzsim")
    nav2_params_file = os.path.join(g3gzsim_dir, "nav2", "nav2_params.yaml")
    rviz_config_file = os.path.join(g3gzsim_dir, "rviz", "nav2_default_view.rviz")
    world_file = os.path.join(g3gzsim_dir, "worlds", "tb3_sandbox.sdf.xacro")

    tb3_simulation_launch_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb3_simulation_launch_file),
        launch_arguments={
            "headless": "False",
            "slam": "True",
            "map": "",
            "params_file": nav2_params_file,
            "rviz_config_file": rviz_config_file,
            "world": world_file,
        }.items(),
    )

    ld = LaunchDescription()

    ld.add_action(tb3_simulation_launch_action)

    return ld
