"""Four-wheel SMB physics and optional native ROS 2 path tracking."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('gbplanner_smb'))
    tracking = Path(get_package_share_directory('gbplanner_tracking'))
    return LaunchDescription([
        DeclareLaunchArgument('tracking', default_value='true'),
        DeclareLaunchArgument('execution', default_value='true'),
        DeclareLaunchArgument('description', default_value='true'),
        DeclareLaunchArgument('config', default_value=str(share / 'config/smb.yaml')),
        Node(package='gbplanner_smb', executable='smb_node',
             parameters=[LaunchConfiguration('config')], output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/description.launch.py')),
            condition=IfCondition(LaunchConfiguration('description'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(tracking / 'launch/smb_tracking.launch.py')),
            condition=IfCondition(LaunchConfiguration('tracking')),
            launch_arguments={'execution': LaunchConfiguration('execution')}.items()),
    ])
