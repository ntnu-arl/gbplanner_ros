from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('gbplanner_local')) / 'config/multi_sensor.yaml'
    return LaunchDescription([
        DeclareLaunchArgument(
            'backend', default_value='occupancy', choices=['occupancy', 'voxblox'],
            description='Map backend shared by the front and rear lidar inputs'),
        Node(
            package='gbplanner_local', executable='local_planner_node',
            name='gbplanner_local', parameters=[str(config)], output='screen'),
        Node(
            package='gbplanner_local', executable='occupancy_mapper_node',
            name='occupancy_mapper', parameters=[str(config)], output='screen',
            condition=LaunchConfigurationEquals('backend', 'occupancy')),
        Node(
            package='gbplanner_voxblox', executable='voxblox_mapper_node',
            name='voxblox_mapper', parameters=[str(config)], output='screen',
            condition=LaunchConfigurationEquals('backend', 'voxblox')),
    ])
