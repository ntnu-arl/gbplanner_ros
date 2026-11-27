"""Original SMB visual model driven by the physical node's wheel JointState."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('gbplanner_smb'))
    description = (share / 'urdf/smb.urdf').read_text(encoding='utf-8')
    return LaunchDescription([
        DeclareLaunchArgument('joint_states_topic', default_value='joint_states'),
        # smb_node currently timestamps physical states with the ROS wall clock.
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='smb_robot_state_publisher', output='screen',
            parameters=[{
                'robot_description': description,
                'publish_frequency': 50.0,
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
            remappings=[('joint_states', LaunchConfiguration('joint_states_topic'))],
        ),
    ])
