from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default = Path(get_package_share_directory('gbplanner_local')) / 'config/adaptive.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(default)),
        Node(package='gbplanner_local', executable='local_planner_node',
             parameters=[LaunchConfiguration('config')], output='screen'),
    ])
