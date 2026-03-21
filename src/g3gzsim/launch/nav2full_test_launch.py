import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# colcon build --packages-select g3gzsim && roset && ros2 launch g3gzsim nav2full_test_launch.py | tee 

def generate_launch_description():

    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    tb3_simulation_launch_file = os.path.join(
        nav2_bringup_dir, "launch", "tb3_simulation_launch.py"
    )

    g3gzsim_dir = get_package_share_directory("g3gzsim")
    nav2_params_file = os.path.join(g3gzsim_dir, "nav2", "nav2_params.yaml")
    rviz_config_file = os.path.join(g3gzsim_dir, "rviz", "nav2_default_view.rviz")
    # world_file = os.path.join(g3gzsim_dir, "worlds", "tb3_sandbox.sdf.xacro")
    world_file = os.path.join(g3gzsim_dir, "worlds", "warehouse_world.sdf")
    robot_file = os.path.join(g3gzsim_dir, "urdf", "tb.sdf.xacro")
    frontier_dir = get_package_share_directory("g3g_frontier_exploration")
    frontier_launch_file = os.path.join(
        frontier_dir, "launch", "frontier_exploration.launch.py"
    )
    default_frontier_params_file = os.path.join(
        frontier_dir, "config", "frontier_exploration.yaml"
    )

    use_frontier_exploration = LaunchConfiguration("use_frontier_exploration")
    frontier_autostart = LaunchConfiguration("frontier_autostart")
    frontier_params_file = LaunchConfiguration("frontier_params_file")

    tb3_simulation_launch_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb3_simulation_launch_file),
        launch_arguments={
            "headless": "False",
            "slam": "True",
            "map": "",
            "params_file": nav2_params_file,
            "rviz_config_file": rviz_config_file,
            "world": world_file,
            "x_pose": "-9",
            "y_pose": "-5",
            "robot_sdf": robot_file,
        }.items(),
    )

    frontier_launch_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(frontier_launch_file),
        condition=IfCondition(use_frontier_exploration),
        launch_arguments={
            "autostart": frontier_autostart,
            "params_file": frontier_params_file,
        }.items(),
    )

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            "use_frontier_exploration",
            default_value="true",
            description="Launch the manual-trigger frontier exploration node.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "frontier_autostart",
            default_value="false",
            description="Start frontier exploration automatically when the explorer is ready.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "frontier_params_file",
            default_value=default_frontier_params_file,
            description="Full path to the frontier exploration parameters file.",
        )
    )
    ld.add_action(tb3_simulation_launch_action)
    ld.add_action(frontier_launch_action)

    return ld
