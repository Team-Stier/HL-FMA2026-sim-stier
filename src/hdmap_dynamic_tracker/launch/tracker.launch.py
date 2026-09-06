from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("hdmap_dynamic_tracker")) / "config/tracker.yaml"
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "map_path",
            default_value=EnvironmentVariable("HDMAP_PATH", default_value=""),
        ),
        DeclareLaunchArgument("tracker_config", default_value=default_config),
        Node(
            package="hdmap_dynamic_tracker",
            executable="hdmap_dynamic_tracker_node",
            name="hdmap_dynamic_tracker",
            output="screen",
            parameters=[
                LaunchConfiguration("tracker_config"),
                {"map_path": LaunchConfiguration("map_path"), "use_sim_time": False},
            ],
        ),
    ])
