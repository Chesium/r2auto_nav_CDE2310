"""Nav2 docking testbed (real hardware).

Minimal wrapper that starts only the camera driver + aruco_dock_pose_publisher.
Assumes you've already launched the SLAM + Nav2 stack elsewhere, e.g.:

    ros2 launch g3nav2 g3nav2_bringup_launch.py use_frontier:=False

The use_frontier:=False flag is already wired into the existing
g3nav2_bringup_launch.py at line 87, so no mission-level or exploration
nodes come up.

This launch file intentionally does NOT start:
    - TB3 drivers (use turtlebot3_bringup separately)
    - SLAM / cartographer (comes from g3nav2_bringup_launch.py)
    - Nav2 / docking_server (comes from g3nav2_bringup_launch.py)
    - RViz (comes from g3nav2_bringup_launch.py)
    - Frontier exploration, mission controller, or the legacy aruco_dock_node

Manual test trigger after bringup:
    ros2 action send_goal /dock_robot nav2_msgs/action/DockRobot "{
      use_dock_id: false,
      dock_pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}},
      dock_type: 'aruco_dock',
      navigate_to_staging_pose: true
    }" --feedback

Launch arguments:
    target_marker_id  (default 42)  ArUco ID the publisher should track.
    marker_size       (default 0.165)  Physical marker edge length in meters.
    start_camera      (default true)  Launch usb_cam alongside the detector.
                                       Set false if a camera driver is already
                                       publishing /usb_cam/image_raw.
    video_device      (default /dev/video0)  passed to usb_cam.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    target_marker_id = LaunchConfiguration("target_marker_id")
    marker_size = LaunchConfiguration("marker_size")
    start_camera = LaunchConfiguration("start_camera")
    video_device = LaunchConfiguration("video_device")

    usb_cam_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam",
        namespace="usb_cam",
        output="screen",
        condition=IfCondition(start_camera),
        parameters=[
            {
                "video_device": video_device,
                "image_width": 640,
                "image_height": 480,
                "pixel_format": "yuyv2rgb",
                "framerate": 30.0,
                "camera_name": "usb_cam",
                "frame_id": "usb_cam",
            }
        ],
    )

    aruco_pose_publisher = Node(
        package="g3_visual_servo",
        executable="aruco_dock_pose_publisher",
        name="aruco_dock_pose_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "image_topic": "/usb_cam/image_raw",
                "camera_info_topic": "/usb_cam/camera_info",
                "dock_pose_topic": "/detected_dock_pose",
                "target_marker_id": target_marker_id,
                "marker_size": marker_size,
                "dictionary": "DICT_4X4_100",
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "target_marker_id",
                default_value="42",
                description="ArUco marker id the pose publisher tracks.",
            ),
            DeclareLaunchArgument(
                "marker_size",
                default_value="0.165",
                description=(
                    "Physical marker edge length in meters. Defaults to the "
                    "0.165 m printout matching the existing aruco_dock_node "
                    "MARKER_SIZE constant."
                ),
            ),
            DeclareLaunchArgument(
                "start_camera",
                default_value="true",
                description=(
                    "Launch usb_cam. Set to false if a camera driver is "
                    "already publishing /usb_cam/image_raw."
                ),
            ),
            DeclareLaunchArgument(
                "video_device",
                default_value="/dev/video0",
                description="Video device passed to usb_cam.",
            ),
            usb_cam_node,
            aruco_pose_publisher,
        ]
    )
