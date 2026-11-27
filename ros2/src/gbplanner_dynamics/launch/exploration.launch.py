from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    dynamics=Path(get_package_share_directory('gbplanner_dynamics'))
    mapping=Path(get_package_share_directory('gbplanner_voxblox'))
    execution=Path(get_package_share_directory('gbplanner_execution'))
    local=Path(get_package_share_directory('gbplanner_local'))
    sensor_config=PythonExpression(["'",str(local/'config/multi_sensor.yaml'),
        "' if '",LaunchConfiguration('lidar_layout'),"' == 'dual' else '",
        str(local/'config/local.yaml'),"'"])
    return LaunchDescription([
        DeclareLaunchArgument('telemetry_file',default_value=''),
        DeclareLaunchArgument('speed',default_value='0.25'),
        DeclareLaunchArgument('lidar_layout',default_value='legacy',choices=['legacy','dual']),
        DeclareLaunchArgument('mission',default_value='false'),
        DeclareLaunchArgument('planner_trigger_lead_time',default_value='0.0'),
        DeclareLaunchArgument('geofence',default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(mapping/'launch/mapping.launch.py')),
          launch_arguments={'global_planning':'true','background':'true','control':'true',
              'geofence':LaunchConfiguration('geofence'),'mission':LaunchConfiguration('mission'),
              'mission_homing_speed':LaunchConfiguration('speed'),
              'sensor_config':sensor_config,
              'planner_trigger_lead_time':LaunchConfiguration('planner_trigger_lead_time')}.items()),
        Node(package='gbplanner_execution',executable='path_follower_node.py',
          parameters=[str(execution/'config/execution.yaml'),{'speed':ParameterValue(LaunchConfiguration('speed'),value_type=float)}],output='screen'),
        Node(package='gbplanner_dynamics',executable='dynamics_node',
          parameters=[str(dynamics/'config/rmf.yaml'),{
              'telemetry_file':ParameterValue(LaunchConfiguration('telemetry_file'),value_type=str),
              'lidar_layout':LaunchConfiguration('lidar_layout')}],output='screen')])
