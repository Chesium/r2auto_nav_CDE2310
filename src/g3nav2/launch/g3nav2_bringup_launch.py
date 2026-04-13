import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    # SLAM stuff
    # gives the following two nodes:
    # /cartographer_node
    # /cartographer_occupancy_grid_node
    use_slam = LaunchConfiguration("use_slam")
    declare_slam = DeclareLaunchArgument(
        "use_slam", default_value="True", description="enable cartographer node or not"
    )
    tb3_slam_dir = get_package_share_directory("turtlebot3_cartographer")
    self_package_dir = get_package_share_directory("g3nav2")
    tb3_slam_launch_file = os.path.join(
        tb3_slam_dir, "launch", "cartographer.launch.py"
    )
    tb3_slam_config_dir = os.path.join(self_package_dir,"config")
    tb3_slam_config_name = "turtlebot3_lds_2d.lua"
    tb3_cartographer_launch_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb3_slam_launch_file),
        condition = IfCondition(use_slam),
        launch_arguments={
            "cartographer_config_dir": tb3_slam_config_dir,
            "configuration_basename": tb3_slam_config_name,
            "use_sim_time": "False",
            "resolution":"0.05",
            "publish_period_sec":"1.0",
            "use_rviz": "false",
        }.items(),
    )

    # NAV 2 stuff
    use_nav2 = LaunchConfiguration("use_nav2")
    declare_nav2 = DeclareLaunchArgument(
        "use_nav2", default_value="True", description="enable nav2 nodes or not"
    )
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    nav2_bringup_launch_file = os.path.join(
        nav2_bringup_dir, "launch", "bringup_launch.py"
    )
    nav2_params_file = os.path.join(self_package_dir, "config", "g3_nav2_params.yaml")
    nav2_launch_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_launch_file),
        condition=IfCondition(use_nav2),
        launch_arguments={
            "slam": "False",  # replaced by the cartographer node
            "map": "",  # replaced by the cartographer node
            "use_localization": "False",  # replaced by the cartographer node
            "use_sim_time": "False",
            "params_file": nav2_params_file,
            "autostart": "True",
            "use_composition": "False",  # for easy debug
            "use_respawn": "False",  # for easy shutdown
            "log_level": "info",
        }.items(),
    )

    # RVIZ2 Stuff
    use_rviz = LaunchConfiguration("use_rviz")
    declare_rviz = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="enable rviz nodes or not"
    )
    rviz2_config_file = os.path.join(self_package_dir, "config", "g3_nav2.rviz")
    rviz2_action = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz2_config_file],
        parameters=[{"use_sim_time": False}],
        output="screen",
    )

    # EXPLORATION STUFF
    use_frontier = LaunchConfiguration("use_frontier")
    declare_frontier = DeclareLaunchArgument(
        "use_frontier", default_value="True", description="enable frontier nodes or not"
    )
    use_post_traversal = LaunchConfiguration("use_post_traversal")
    declare_post_traversal = DeclareLaunchArgument(
        "use_post_traversal",
        default_value="True",
        description="enable post-exploration traversal nodes or not",
    )
    frontier_dir = get_package_share_directory("g3g_frontier_exploration")
    frontier_launch_file = os.path.join(
        frontier_dir, "launch", "frontier_exploration.launch.py"
    )
    default_frontier_params_file = os.path.join(
        frontier_dir, "config", "frontier_exploration.yaml"
    )
    frontier_launch_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(frontier_launch_file),
        condition=IfCondition(use_frontier),
        launch_arguments={
            "autostart": "False",
            "params_file": default_frontier_params_file,
            "use_post_traversal": use_post_traversal,
        }.items(),
    )

    ld = LaunchDescription()

    ld.add_action(declare_slam)
    ld.add_action(declare_nav2)
    ld.add_action(declare_rviz)
    ld.add_action(declare_frontier)
    ld.add_action(declare_post_traversal)

    ld.add_action(tb3_cartographer_launch_action)
    ld.add_action(nav2_launch_action)
    ld.add_action(rviz2_action)
    ld.add_action(frontier_launch_action)

    return ld

# colcon build --symlink-install --packages-select g3nav2 && roset
# ros2 launch g3nav2 g3nav2_bringup_launch.py use_slam:=False use_nav2:=False use_rviz:=False use_frontier:=False

# ros2 launch g3nav2 g3nav2_bringup_launch.py use_slam:=True  use_nav2:=False use_rviz:=False use_frontier:=False
# ros2 launch g3nav2 g3nav2_bringup_launch.py use_slam:=False use_nav2:=False use_rviz:=True  use_frontier:=False
# ros2 launch g3nav2 g3nav2_bringup_launch.py use_slam:=False use_nav2:=True  use_rviz:=False use_frontier:=False
# ros2 launch g3nav2 g3nav2_bringup_launch.py use_slam:=False use_nav2:=False use_rviz:=False use_frontier:=True

# ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 4}"

# ./scripts/g3nav2_tmux.sh
# tmux kill-session -t g3nav2

# ros2 service call /exploration/set_enabled std_srvs/srv/SetBool "{data: true}"
