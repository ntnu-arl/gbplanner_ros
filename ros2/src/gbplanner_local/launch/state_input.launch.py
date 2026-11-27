"""Select native odometry or one of the upstream stamped-pose inputs.

Launch beside the planner/control/follower. In odom mode no process is added.
The input pose must already describe body_frame; changing that label does not
apply a sensor-to-body extrinsic transform.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='odom', choices=['odom', 'pose', 'pose_stamped'],
                              description='Native odometry, PoseWithCovarianceStamped, or PoseStamped'),
        DeclareLaunchArgument('input_topic', default_value='',
                              description='Empty selects the original pose or pose_stamped topic'),
        DeclareLaunchArgument('output_topic', default_value='odometry'),
        DeclareLaunchArgument('frame_id', default_value='map', description='Required output pose frame'),
        DeclareLaunchArgument('body_frame', default_value='base_link',
                              description='Body whose pose is measured by the selected input'),
        DeclareLaunchArgument('transform_to_frame', default_value='true', choices=['true', 'false'],
                              description='Use acquisition-time TF for input outside frame_id'),
        DeclareLaunchArgument('tf_timeout_sec', default_value='0.2'),
        Node(package='gbplanner_local', executable='state_input_node.py', name='gbplanner_state_input',
             condition=LaunchConfigurationNotEquals('mode', 'odom'), output='screen',
             parameters=[{
                 'mode': ParameterValue(LaunchConfiguration('mode'), value_type=str),
                 'input_topic': ParameterValue(LaunchConfiguration('input_topic'), value_type=str),
                 'output_topic': ParameterValue(LaunchConfiguration('output_topic'), value_type=str),
                 'frame_id': ParameterValue(LaunchConfiguration('frame_id'), value_type=str),
                 'body_frame': ParameterValue(LaunchConfiguration('body_frame'), value_type=str),
                 'transform_to_frame': ParameterValue(LaunchConfiguration('transform_to_frame'), value_type=bool),
                 'tf_timeout_sec': ParameterValue(LaunchConfiguration('tf_timeout_sec'), value_type=float),
             }]),
    ])
