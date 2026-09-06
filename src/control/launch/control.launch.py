from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    config = Path(get_package_share_directory('control')) / 'config' / 'control.yaml'
    return LaunchDescription([
        Node(package='control', executable='mpc_control_node', name='mpc_control',
             output='screen', parameters=[str(config)])
    ])
