"""Ground planner and optional trajectory output for an external robot backend."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('gbplanner_local')) / 'config/ground.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(config)),
        DeclareLaunchArgument('execution', default_value='false'),
        Node(package='gbplanner_local', executable='local_planner_node',
             parameters=[LaunchConfiguration('config')], output='screen'),
        Node(package='gbplanner_execution', executable='path_follower_node.py',
             parameters=[{'robot_type': 'kGround', 'speed': .7, 'yaw_rate': 1.5}],
             condition=IfCondition(LaunchConfiguration('execution')), output='screen'),
    ])
