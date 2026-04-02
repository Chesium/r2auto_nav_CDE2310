"""Minimal launch: Gazebo + robot + camera bridges. No Nav2, no SLAM, no frontier."""

import os
import subprocess

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


def generate_launch_description():
    g3gzsim_dir = get_package_share_directory("g3gzsim")
    g3gzsim_models_dir = os.path.join(g3gzsim_dir, "models")
    default_world_file = os.path.join(g3gzsim_dir, "worlds", "square_room.sdf")
    robot_xacro = os.path.join(g3gzsim_dir, "urdf", "tb.sdf.xacro")

    # Process xacro → raw SDF string (same as tb3_simulation_launch does)
    robot_sdf = subprocess.check_output(
        ["xacro", robot_xacro], text=True
    )

    # Official Gazebo-only launch from ros_gz_sim
    gz_sim_launch_file = os.path.join(
        get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
    )

    world_file = LaunchConfiguration("world")

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            "world", default_value=default_world_file, description="SDF world file."
        )
    )

    # Make our models dir visible to Gazebo
    existing_gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    ld.add_action(
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=(
                g3gzsim_models_dir + os.pathsep + existing_gz_path
                if existing_gz_path
                else g3gzsim_models_dir
            ),
        )
    )

    # Launch Gazebo
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch_file),
            launch_arguments={"gz_args": ["-r ", world_file]}.items(),
        )
    )

    # Spawn the robot using the processed SDF string
    ld.add_action(
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name", "tb",
                "-string", robot_sdf,
                "-x", "0.0",
                "-y", "0.0",
                "-z", "0.01",
            ],
            output="screen",
        )
    )

    # Robot state publisher — publish TF from the processed SDF
    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {"use_sim_time": True, "robot_description": robot_sdf},
            ],
            output="screen",
        )
    )

    # Bridge: cmd_vel, odom, scan, imu, joints, clock
    ld.add_action(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
                "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            ],
            parameters=[{"use_sim_time": True}],
            output="screen",
        )
    )

    # Bridge: camera images
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

    # Bridge: camera_info
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
