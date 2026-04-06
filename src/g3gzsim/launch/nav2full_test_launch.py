import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# colcon build --packages-select g3gzsim && roset && ros2 launch g3gzsim nav2full_test_launch.py | tee 

def generate_launch_description():

    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    tb3_simulation_launch_file = os.path.join(
        nav2_bringup_dir, "launch", "tb3_simulation_launch.py"
    )

    g3gzsim_dir = get_package_share_directory("g3gzsim")
    g3gzsim_models_dir = os.path.join(g3gzsim_dir, "models")
    nav2_params_file = os.path.join(g3gzsim_dir, "nav2", "nav2_params.yaml")
    rviz_config_file = os.path.join(g3gzsim_dir, "rviz", "nav2_default_view.rviz")
    default_world_file = os.path.join(g3gzsim_dir, "worlds", "warehouse_world.sdf")
    robot_file = os.path.join(g3gzsim_dir, "urdf", "tb.sdf.xacro")
    frontier_dir = get_package_share_directory("g3g_frontier_exploration")
    frontier_launch_file = os.path.join(
        frontier_dir, "launch", "frontier_exploration.launch.py"
    )
    default_frontier_params_file = os.path.join(
        frontier_dir, "config", "frontier_exploration.yaml"
    )

    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    world_file = LaunchConfiguration("world")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    use_frontier_exploration = LaunchConfiguration("use_frontier_exploration")
    frontier_autostart = LaunchConfiguration("frontier_autostart")
    frontier_params_file = LaunchConfiguration("frontier_params_file")

    tb3_simulation_launch_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb3_simulation_launch_file),
        launch_arguments={
            "headless": headless,
            "use_rviz": use_rviz,
            "slam": "True",
            "map": "",
            "params_file": nav2_params_file,
            "rviz_config_file": rviz_config_file,
            "world": world_file,
            "x_pose": x_pose,
            "y_pose": y_pose,
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
            "headless",
            default_value="False",
            description="Run Gazebo in headless mode (no GUI).",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="True",
            description="Launch RViz.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "world",
            default_value=default_world_file,
            description="Full path to the Gazebo world file.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "x_pose",
            default_value="-9",
            description="Robot spawn X position.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "y_pose",
            default_value="-5",
            description="Robot spawn Y position.",
        )
    )
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
    # Let Gazebo find custom models (e.g. aruco_marker_42)
    existing_gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    ld.add_action(
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=g3gzsim_models_dir + os.pathsep + existing_gz_path
            if existing_gz_path
            else g3gzsim_models_dir,
        )
    )
    ld.add_action(tb3_simulation_launch_action)
    ld.add_action(frontier_launch_action)

    # Bridge RGB camera images from Gazebo to ROS 2
    ld.add_action(
        Node(
            package="ros_gz_image",
            executable="image_bridge",
            arguments=["camera/image_raw"],
            output="screen",
        )
    )
    ld.add_action(
        Node(
            package="ros_gz_image",
            executable="image_bridge",
            arguments=["side_camera/image_raw"],
            output="screen",
        )
    )
    # Bridge camera_info (not handled by image_bridge)
    ld.add_action(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            ],
            output="screen",
        )
    )

    return ld
