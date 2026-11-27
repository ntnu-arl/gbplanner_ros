from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    config = Path(get_package_share_directory('gbplanner_local')) / 'config/local.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('background',default_value='false'),
        DeclareLaunchArgument('geofence',default_value='false'),
        DeclareLaunchArgument('seed',default_value='42'),
        DeclareLaunchArgument('mission',default_value='false'),
        DeclareLaunchArgument('mission_homing_speed',default_value='0.9'),
        DeclareLaunchArgument('auto_home',default_value='true'),
        Node(package='gbplanner_local', executable='local_planner_node', name='gbplanner_local',
             parameters=[str(config), {'planning.geofence_checking_enable':ParameterValue(LaunchConfiguration('geofence'),value_type=bool),'global.enable':True,'mission.enabled':ParameterValue(LaunchConfiguration('mission'),value_type=bool),'global.auto_home':ParameterValue(LaunchConfiguration('auto_home'),value_type=bool),
                 'mission.homing_speed':ParameterValue(LaunchConfiguration('mission_homing_speed'),value_type=float),
                 'global.background_enable':ParameterValue(LaunchConfiguration('background'),value_type=bool),
                 'sampling.seed':ParameterValue(LaunchConfiguration('seed'),value_type=int)}], output='screen'),
        Node(package='gbplanner_local', executable='occupancy_mapper_node', name='occupancy_mapper',
             parameters=[{'frame_id':'map','sensor_frame':'lidar','map.resolution':0.25,
                 'map.origin':[-5.0,-5.0,-1.5],'map.dimensions':[40,40,12],
                 'sensor.min_range':0.1,'sensor.max_range':6.0}],output='screen')])
