"""Ground execution bridge and SMB velocity tracker for an external base."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('gbplanner_tracking')) / 'config/smb.yaml'
    remaps = [('odometry', LaunchConfiguration('odometry')),
              ('cmd_vel', LaunchConfiguration('cmd_vel'))]
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(config)),
        DeclareLaunchArgument('odometry', default_value='odometry'),
        DeclareLaunchArgument('cmd_vel', default_value='cmd_vel'),
        DeclareLaunchArgument('execution', default_value='true'),
        Node(package='gbplanner_tracking', executable='pure_pursuit_node',
             parameters=[LaunchConfiguration('config')], remappings=remaps,
             output='screen'),
        Node(package='gbplanner_execution', executable='path_follower_node.py',
             parameters=[LaunchConfiguration('config')], remappings=remaps,
             condition=IfCondition(LaunchConfiguration('execution')), output='screen'),
    ])
