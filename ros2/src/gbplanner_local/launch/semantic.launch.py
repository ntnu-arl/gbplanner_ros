"""Add semantic point selection to a running planner and odometry source."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('marker_namespace', default_value='semantics'),
        DeclareLaunchArgument('odom_timeout', default_value='2.0'),
        Node(package='gbplanner_local', executable='semantic_waypoint_node.py',
             name='gbplanner_semantic_waypoint', parameters=[{
                 'frame_id': LaunchConfiguration('frame_id'),
                 'marker_namespace': LaunchConfiguration('marker_namespace'),
                 'odom_timeout': ParameterValue(LaunchConfiguration('odom_timeout'), value_type=float),
             }], output='screen'),
    ])
