from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='g3_visual_servo',
            executable='aruco_dock',
            name='aruco_dock',
            output='screen',
        ),
    ])
