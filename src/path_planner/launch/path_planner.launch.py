from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("path_planner"))
    return LaunchDescription([
        DeclareLaunchArgument("map_path", default_value=EnvironmentVariable("HDMAP_PATH", default_value="")),
        DeclareLaunchArgument("checkpoint_file", default_value=str(share / "checkpoint/checkpoints.csv")),
        DeclareLaunchArgument("planner_config", default_value=str(share / "config/planner.yaml")),
        Node(
            package="path_planner",
            executable="path_planner_node",
            name="path_planner",
            output="screen",
            parameters=[LaunchConfiguration("planner_config"), {
                "map_path": LaunchConfiguration("map_path"),
                "checkpoint_file": LaunchConfiguration("checkpoint_file"),
                "use_sim_time": False,
            }],
        ),
    ])
