from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("visualization"))
    visualizer = Node(package="visualization", executable="visualizer_node", output="screen",
                      parameters=[{"map_path": LaunchConfiguration("map_path"), "use_sim_time": False}])
    rviz = Node(package="rviz2", executable="rviz2", output="screen",
                arguments=["-d", str(share / "rviz/default.rviz")], parameters=[{"use_sim_time": False}])
    tf = Node(package="tf_broadcasting", executable="tf_broadcasting_node", output="screen", parameters=[{"use_sim_time": False}])
    return LaunchDescription([
        DeclareLaunchArgument("map_path", default_value=""),
        RegisterEventHandler(OnProcessExit(target_action=visualizer, on_exit=[EmitEvent(event=Shutdown(reason="Visualizer exited"))])),
        RegisterEventHandler(OnProcessExit(target_action=rviz, on_exit=[EmitEvent(event=Shutdown(reason="RViz exited"))])),
        RegisterEventHandler(OnProcessExit(target_action=tf, on_exit=[EmitEvent(event=Shutdown(reason="TF exited"))])),
        tf, visualizer, rviz,
    ])
