"""SMB wheel physics, sparse TSDF, ground planning and Action-based tracking.

The control service starts IDLE. Planning or motion begins only after an
explicit PCI request; starting this launch does not start an exploration run.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start_planner(context):
    parameters = [LaunchConfiguration('exploration_config')]
    profile = LaunchConfiguration('planner_profile').perform(context).strip()
    if profile:
        parameters.append(str(Path(profile).expanduser()))
    parameters.append({'use_sim_time': False})
    return [Node(package='gbplanner_local', executable='local_planner_node', name='gbplanner_local',
                 parameters=parameters, output='screen')]


def generate_launch_description():
    smb = Path(get_package_share_directory('gbplanner_smb'))
    tracking = Path(get_package_share_directory('gbplanner_tracking'))
    local = Path(get_package_share_directory('gbplanner_local'))
    exploration_config = LaunchConfiguration('exploration_config')
    return LaunchDescription([
        DeclareLaunchArgument('vehicle_config', default_value=str(smb / 'config/smb.yaml')),
        DeclareLaunchArgument('collision_mesh', default_value='',
                              description='Optional world-coordinate static triangle mesh'),
        DeclareLaunchArgument('exploration_config',
                              default_value=str(smb / 'config/exploration.yaml')),
        DeclareLaunchArgument('planner_profile', default_value='',
                              description='Optional planner-only YAML overlay, such as gbplanner_smb/config/upstream_planning.yaml'),
        DeclareLaunchArgument('tracking_config', default_value=str(tracking / 'config/smb.yaml')),
        DeclareLaunchArgument('control', default_value='true',
                              description='Expose PCI services; no automatic motion at startup'),
        DeclareLaunchArgument('visualization', default_value='false'),
        DeclareLaunchArgument('description', default_value='true'),
        DeclareLaunchArgument('velocity_mux', default_value='true'),
        DeclareLaunchArgument('mesh_period', default_value=PythonExpression([
            "0.5 if '", LaunchConfiguration('visualization'), "' == 'true' else 0.0"])),
        DeclareLaunchArgument('rviz', default_value='false'),
        # SmbNode uses wall-clock integration and publishes stamped odometry/TF;
        # it does not publish /clock. Keep this chain on the same ROS wall clock.
        Node(package='gbplanner_smb', executable='smb_node', name='gbplanner_smb',
             parameters=[LaunchConfiguration('vehicle_config'), {'use_sim_time': False,
                 'collision_mesh': ParameterValue(LaunchConfiguration('collision_mesh'), value_type=str)}],
             output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(smb / 'launch/description.launch.py')),
            condition=IfCondition(LaunchConfiguration('description'))),
        Node(package='gbplanner_voxblox', executable='voxblox_mapper_node', name='voxblox_mapper',
             parameters=[exploration_config, {'use_sim_time': False,
                 'update_mesh_every_n_sec': ParameterValue(LaunchConfiguration('mesh_period'), value_type=float)}], output='screen'),
        OpaqueFunction(function=start_planner),
        Node(package='gbplanner_local', executable='control_node.py', name='gbplanner_control',
             parameters=[exploration_config, {'use_sim_time': False}],
             condition=IfCondition(LaunchConfiguration('control')), output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(smb / 'launch/teleop.launch.py')),
            condition=IfCondition(LaunchConfiguration('velocity_mux'))),
        # Scope the nested 'config' argument so it cannot change the planner,
        # mapper or vehicle parameter-file selections in this launch.
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(tracking / 'launch/smb_tracking.launch.py')),
                launch_arguments={'config': LaunchConfiguration('tracking_config'),
                                  'execution': 'true',
                                  'cmd_vel': PythonExpression(["'external/cmd_vel' if '",
                                      LaunchConfiguration('velocity_mux'), "' == 'true' else 'cmd_vel'"])}.items()),
        ]),
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(local / 'launch/visualization.launch.py')),
                launch_arguments={'rviz': LaunchConfiguration('rviz')}.items()),
        ], condition=IfCondition(LaunchConfiguration('visualization'))),
    ])
