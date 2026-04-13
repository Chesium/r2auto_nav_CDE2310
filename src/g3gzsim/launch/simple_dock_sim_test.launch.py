"""Simple ArUco docking testbed (simulation).

Brings up Gazebo with square_room.sdf (marker 42 on east wall), spawns the
TurtleBot3 facing the marker, runs Nav2 for SLAM only, bridges the camera,
and starts simple_aruco_dock.

NO Nav2 docking_server.  NO frontier exploration.  NO FSM.

Test procedure:
    1. Launch this file
    2. Teleop near the marker:  ros2 run teleop_twist_keyboard teleop_twist_keyboard
    3. Trigger docking:         ros2 service call /simple_dock/start std_srvs/srv/Trigger
    4. Watch /aruco_debug for distance feedback
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    tb3_launch = os.path.join(
        nav2_bringup_dir, "launch", "tb3_simulation_launch.py"
    )

    g3gzsim_dir = get_package_share_directory("g3gzsim")
    models_dir = os.path.join(g3gzsim_dir, "models")
    params_file = os.path.join(g3gzsim_dir, "nav2", "nav2_params.yaml")
    rviz_file = os.path.join(g3gzsim_dir, "rviz", "nav2_default_view.rviz")
    world_file = os.path.join(g3gzsim_dir, "worlds", "square_room.sdf")
    robot_file = os.path.join(g3gzsim_dir, "urdf", "tb.sdf.xacro")

    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    target_marker_id = LaunchConfiguration("target_marker_id")
    marker_size = LaunchConfiguration("marker_size")

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument("headless", default_value="False"))
    ld.add_action(DeclareLaunchArgument("use_rviz", default_value="True"))
    ld.add_action(DeclareLaunchArgument("x_pose", default_value="-2.0"))
    ld.add_action(DeclareLaunchArgument("y_pose", default_value="0.0"))
    ld.add_action(DeclareLaunchArgument("target_marker_id", default_value="42"))
    ld.add_action(DeclareLaunchArgument("marker_size", default_value="0.18"))

    # Gazebo model path
    existing = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    ld.add_action(SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=models_dir + os.pathsep + existing if existing else models_dir,
    ))

    # Nav2 + Gazebo + SLAM
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb3_launch),
        launch_arguments={
            "headless": headless,
            "use_rviz": use_rviz,
            "slam": "True",
            "map": "",
            "params_file": params_file,
            "rviz_config_file": rviz_file,
            "world": world_file,
            "x_pose": x_pose,
            "y_pose": y_pose,
            "robot_sdf": robot_file,
        }.items(),
    ))

    # Camera bridges (Gazebo → ROS 2)
    ld.add_action(Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["camera/image_raw"],
        output="screen",
    ))
    ld.add_action(Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        output="screen",
    ))

    # Simple ArUco docking node
    ld.add_action(Node(
        package="g3_visual_servo",
        executable="simple_aruco_dock",
        name="simple_aruco_dock",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "image_topic": "/camera/image_raw",
            "camera_info_topic": "/camera/camera_info",
            "target_marker_id": target_marker_id,
            "marker_size": marker_size,
        }],
    ))

    return ld
