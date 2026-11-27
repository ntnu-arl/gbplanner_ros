"""Interactive search waypoints, planner and trajectory execution interface."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    local = Path(get_package_share_directory('gbplanner_local'))
    execution = Path(get_package_share_directory('gbplanner_execution'))
    return LaunchDescription([
        DeclareLaunchArgument('planner_config', default_value=str(local / 'config/search.yaml')),
        DeclareLaunchArgument('execution_config', default_value=str(execution / 'config/execution.yaml')),
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(package='gbplanner_local', executable='local_planner_node', name='gbplanner_local',
             parameters=[LaunchConfiguration('planner_config')], output='screen'),
        Node(package='gbplanner_local', executable='control_node.py', name='gbplanner_control',
             parameters=[LaunchConfiguration('planner_config')], output='screen'),
        Node(package='gbplanner_execution', executable='path_follower_node.py',
             name='gbplanner_path_follower',
             parameters=[LaunchConfiguration('execution_config'), LaunchConfiguration('planner_config')],
             output='screen'),
        Node(package='gbplanner_local', executable='search_waypoints_node.py',
             name='gbplanner_search_waypoints', parameters=[LaunchConfiguration('planner_config')],
             output='screen'),
        Node(package='gbplanner_local', executable='visualization_node.py',
             name='gbplanner_visualization', output='screen'),
        Node(package='rviz2', executable='rviz2', name='search_rviz',
             arguments=['-d', str(local / 'config/search.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
