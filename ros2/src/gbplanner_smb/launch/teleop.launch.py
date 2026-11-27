"""SMB joystick conversion and priority arbitration for the PCI control chain."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('gbplanner_smb'))
    return LaunchDescription([
        DeclareLaunchArgument('teleop_config', default_value=str(share / 'config/teleop.yaml')),
        DeclareLaunchArgument('joy_adapter', default_value='true'),
        Node(package='gbplanner_smb', executable='velocity_mux_node.py',
             parameters=[LaunchConfiguration('teleop_config')], output='screen'),
        Node(package='gbplanner_smb', executable='joy_teleop_node.py',
             parameters=[LaunchConfiguration('teleop_config')], output='screen',
             condition=IfCondition(LaunchConfiguration('joy_adapter'))),
    ])
