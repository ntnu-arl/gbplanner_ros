"""Open the native RViz control panel alongside the running planner."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default = str(Path(get_package_share_directory('gbplanner_ui')) / 'config/control.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='rviz2', executable='rviz2', name='gbplanner_rviz',
             arguments=['-d', LaunchConfiguration('config')],
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}], output='screen'),
    ])
