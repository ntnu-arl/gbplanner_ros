"""Gate planning and PCI; provide odometry, voxel_map and a trajectory consumer."""
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
        DeclareLaunchArgument('config', default_value=str(local / 'config/gate.yaml')),
        DeclareLaunchArgument('execution', default_value='true'),
        Node(package='gbplanner_local', executable='local_planner_node',
             name='gbplanner_local', parameters=[LaunchConfiguration('config')], output='screen'),
        Node(package='gbplanner_local', executable='control_node.py',
             name='gbplanner_control', parameters=[LaunchConfiguration('config')], output='screen'),
        Node(package='gbplanner_execution', executable='path_follower_node.py',
             name='gbplanner_path_follower', condition=IfCondition(LaunchConfiguration('execution')),
             parameters=[str(execution / 'config/execution.yaml'), LaunchConfiguration('config')],
             output='screen'),
    ])
