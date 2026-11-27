"""Original Edgar collision geometry with ROS 2 SMB, lidar and sparse mapping.

The vehicle uses Bullet wheel contacts. This does not run Gazebo or reproduce
its contact solver. PCI starts idle; loading this scene does not command motion.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    smb = Path(get_package_share_directory('gbplanner_smb'))
    assets = Path(get_package_share_directory('planner_gazebo_sim'))
    return LaunchDescription([
        DeclareLaunchArgument('control', default_value='true'),
        DeclareLaunchArgument('description', default_value='true'),
        DeclareLaunchArgument('planner_profile', default_value='',
                              description='Optional planner-only YAML overlay'),
        DeclareLaunchArgument('visualization', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(smb / 'launch/exploration.launch.py')),
            launch_arguments={
                'vehicle_config': str(smb / 'config/edgar_mine.yaml'),
                'exploration_config': str(smb / 'config/edgar_exploration.yaml'),
                'planner_profile': LaunchConfiguration('planner_profile'),
                'collision_mesh': str(assets / 'collision/edgar_mine.triangles'),
                'control': LaunchConfiguration('control'),
                'description': LaunchConfiguration('description'),
                'visualization': LaunchConfiguration('visualization'),
                'rviz': LaunchConfiguration('rviz'),
            }.items()),
    ])
