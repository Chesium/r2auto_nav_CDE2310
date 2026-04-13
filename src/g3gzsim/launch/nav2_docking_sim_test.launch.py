"""Nav2 docking testbed (simulation).

Single-command launcher that brings up Gazebo with square_room.sdf (which
already includes the aruco_marker_42 model on the east wall), spawns the
TurtleBot3 facing the marker, runs Nav2's tb3_simulation_launch with SLAM
enabled (so docking_server comes up active), bridges the camera, and
starts the aruco_dock_pose_publisher so /detected_dock_pose is populated.

Deliberately NOT included:
    - frontier_exploration (we're testing docking in isolation)
    - mission_controller, simple_mission_fsm, aruco_dock_node (old flow)
    - Any other mission-level nodes

Manual test trigger after launch:
    ros2 action send_goal /dock_robot nav2_msgs/action/DockRobot "{
      use_dock_id: false,
      dock_pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}},
      dock_type: 'aruco_dock',
      navigate_to_staging_pose: true
    }" --feedback
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
    tb3_simulation_launch_file = os.path.join(
        nav2_bringup_dir, "launch", "tb3_simulation_launch.py"
    )

    g3gzsim_dir = get_package_share_directory("g3gzsim")
    g3gzsim_models_dir = os.path.join(g3gzsim_dir, "models")
    nav2_params_file = os.path.join(g3gzsim_dir, "nav2", "nav2_params.yaml")
    rviz_config_file = os.path.join(g3gzsim_dir, "rviz", "nav2_default_view.rviz")
    default_world_file = os.path.join(g3gzsim_dir, "worlds", "square_room.sdf")
    robot_file = os.path.join(g3gzsim_dir, "urdf", "tb.sdf.xacro")

    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    world_file = LaunchConfiguration("world")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    target_marker_id = LaunchConfiguration("target_marker_id")
    marker_size = LaunchConfiguration("marker_size")

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

    # The aruco detector: turns camera frames into PoseStamped on
    # /detected_dock_pose for the docking_server to consume. Sim camera
    # topic is /camera/image_raw (not /usb_cam/image_raw like hardware).
    aruco_pose_publisher = Node(
        package="g3_visual_servo",
        executable="aruco_dock_pose_publisher",
        name="aruco_dock_pose_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "image_topic": "/camera/image_raw",
                "camera_info_topic": "/camera/camera_info",
                "dock_pose_topic": "/detected_dock_pose",
                "target_marker_id": target_marker_id,
                "marker_size": marker_size,
                "dictionary": "DICT_4X4_100",
            }
        ],
    )

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            "headless",
            default_value="False",
            description="Run Gazebo headless (no GUI).",
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
            default_value="-2.0",
            description=(
                "Robot spawn X position. Default puts the robot 2 m west "
                "of the origin, facing east (+X) toward the marker."
            ),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "y_pose",
            default_value="0.0",
            description="Robot spawn Y position.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "target_marker_id",
            default_value="42",
            description="ArUco marker id that the docking pose publisher should track.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "marker_size",
            default_value="0.18",
            description=(
                "Physical marker edge length in meters. Matches the "
                "aruco_marker_42 Gazebo model's 0.18 m placard."
            ),
        )
    )

    # Let Gazebo find custom models (aruco_marker_42, etc.)
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

    # Bridge RGB camera images + camera_info from Gazebo to ROS 2
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
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            ],
            output="screen",
        )
    )

    ld.add_action(aruco_pose_publisher)

    return ld
