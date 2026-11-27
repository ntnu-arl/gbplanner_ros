from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('gbplanner_local')) / 'config/gbplanner.rviz'
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('semantic_markers', default_value='true'),
        DeclareLaunchArgument('frame_id', default_value='map'),
        Node(package='gbplanner_local', executable='visualization_node.py',
             parameters=[{'frame_id': LaunchConfiguration('frame_id')}], output='screen'),
        Node(package='gbplanner_local', executable='semantic_waypoint_node.py',
             parameters=[{'frame_id': LaunchConfiguration('frame_id')}],
             condition=IfCondition(LaunchConfiguration('semantic_markers')), output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(config)],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
