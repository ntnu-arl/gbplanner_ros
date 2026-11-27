from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    directory=Path(get_package_share_directory('gbplanner_local'))/'launch'
    return LaunchDescription([
        DeclareLaunchArgument('seed',default_value='42'),
        DeclareLaunchArgument('mission',default_value='false'),
        DeclareLaunchArgument('mission_homing_speed',default_value='0.9'),
        DeclareLaunchArgument('planner_trigger_lead_time',default_value='0.0'),
        DeclareLaunchArgument('geofence',default_value='false'),
        DeclareLaunchArgument('visualization',default_value='true'),
        DeclareLaunchArgument('rviz',default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(directory/'global.launch.py')),launch_arguments={'background':'true','geofence':LaunchConfiguration('geofence'),'mission':LaunchConfiguration('mission'),'mission_homing_speed':LaunchConfiguration('mission_homing_speed'),'seed':LaunchConfiguration('seed')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(directory/'visualization.launch.py')),
            condition=IfCondition(LaunchConfiguration('visualization')),
            launch_arguments={'rviz':LaunchConfiguration('rviz')}.items()),
        Node(package='gbplanner_local',executable='control_node.py',name='gbplanner_control',
             parameters=[{'planner_trigger_lead_time':ParameterValue(LaunchConfiguration('planner_trigger_lead_time'),value_type=float),'require_mission_status':ParameterValue(LaunchConfiguration('mission'),value_type=bool)}],output='screen')
    ])
