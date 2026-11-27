"""Planner free-frustum augmentation with an explicitly separate mapper input."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package = Path(get_package_share_directory('gbplanner_local'))
    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='voxblox', choices=['voxblox', 'occupancy']),
        DeclareLaunchArgument('config', default_value=str(package / 'config/free_space.yaml'),
                              description='Independent free-frustum models and matching mapper frame allowlist'),
        Node(package='gbplanner_local', executable='local_planner_node', name='gbplanner_local',
             parameters=[str(package / 'config/local.yaml'), config], output='screen'),
        Node(package='gbplanner_voxblox', executable='voxblox_mapper_node', name='voxblox_mapper',
             parameters=[config], condition=LaunchConfigurationEquals('backend', 'voxblox'), output='screen'),
        Node(package='gbplanner_local', executable='occupancy_mapper_node', name='occupancy_mapper',
             parameters=[config], condition=LaunchConfigurationEquals('backend', 'occupancy'), output='screen'),
    ])
