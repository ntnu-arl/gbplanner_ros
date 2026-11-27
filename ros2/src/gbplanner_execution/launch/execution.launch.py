from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = Path(get_package_share_directory('gbplanner_execution'))/'config/execution.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('output_type', default_value='kTopic', choices=['kTopic', 'kAction']),
        DeclareLaunchArgument('output_action', default_value='pci_output_path'),
        DeclareLaunchArgument('output_action_timeout', default_value='0.0'),
        Node(package='gbplanner_execution', executable='path_follower_node.py',
             parameters=[str(config), {
                 'output_type': LaunchConfiguration('output_type'),
                 'output_action': LaunchConfiguration('output_action'),
                 'output_action_timeout': ParameterValue(LaunchConfiguration('output_action_timeout'), value_type=float),
             }], output='screen')])
