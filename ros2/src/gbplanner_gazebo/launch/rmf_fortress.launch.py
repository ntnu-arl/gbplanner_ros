"""Original RMF airframe, four native motors and OS0 lidar in Edgar mine."""
import importlib.util
import json
import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable, EmitEvent)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    bridge_mode = LaunchConfiguration('bridge').perform(context)
    if bridge_mode not in ('system', 'managed'):
        raise ValueError('bridge must be system or managed')
    bridge_package, bridge_executable = (('gbplanner_gazebo', 'managed_bridge_node')
        if bridge_mode == 'managed' else ('ros_gz_bridge', 'parameter_bridge'))
    share = Path(get_package_share_directory('gbplanner_gazebo'))
    assets = Path(get_package_share_directory('planner_gazebo_sim'))
    module_file = Path(get_package_prefix('gbplanner_gazebo')) / 'lib/gbplanner_gazebo/fortress_rmf_model.py'
    spec = importlib.util.spec_from_file_location('fortress_rmf_model', module_file)
    model = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(model)
    generated = model.generate_model(share / 'models/rmf_obelix')
    temporary = tempfile.TemporaryDirectory(prefix='gbplanner_rmf_fortress_')
    robot = Path(temporary.name) / 'rmf.sdf'
    robot.write_text(generated['sdf'])
    metadata_file = LaunchConfiguration('model_metadata_file').perform(context)
    if metadata_file:
        destination = Path(metadata_file)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(json.dumps(generated['metadata'], indent=2) + '\n')
    sim_time = {'use_sim_time': True}
    resources = os.pathsep.join([str(assets / 'models'),
                                os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')])
    gazebo = ExecuteProcess(cmd=['ign', 'gazebo', '-r', '-s', '--headless-rendering',
                                 '-v', '3', str(share / 'worlds/edgar_fortress.sdf')],
                            output='screen')
    bridges = ['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
               '/rmf/ground_truth@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
               '/rmf/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
               '/rmf/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
               '/rmf/contact/base@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
               '/rmf/command/motor_speed@actuator_msgs/msg/Actuators]ignition.msgs.Actuators']
    bridges.extend(f'/rmf/imu/{name}/raw@sensor_msgs/msg/Imu[ignition.msgs.IMU'
                   for name in ('adis', 'ground_truth', 'os0'))
    contacts = LaunchConfiguration('contacts').perform(context).lower() == 'true'
    parts = ['base', 'rotor_0', 'rotor_1', 'rotor_2', 'rotor_3']
    if contacts:
        bridges.extend(f'/rmf/contact/{part}@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts'
                       for part in parts if part != 'base')
    nodes = [
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resources),
        SetEnvironmentVariable('IGN_PARTITION', 'gbplanner_fortress_' + os.environ.get('ROS_DOMAIN_ID', '0')),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'), gazebo,
        RegisterEventHandler(OnProcessExit(target_action=gazebo,
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo server exited'))])),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[sim_time, {'robot_description': generated['urdf']}], output='screen'),
        Node(package=bridge_package, executable=bridge_executable, parameters=[sim_time],
             arguments=bridges, remappings=[('/rmf/ground_truth', 'gazebo/odometry_raw'),
                                           ('/rmf/scan/points', 'gazebo/points_raw'),
                                           ('/rmf/joint_state', 'joint_states')], output='screen'),
        Node(package='gbplanner_gazebo', executable='fortress_state_node.py',
             parameters=[sim_time, {'sensor_frame': 'lidar', 'model_kind': 'rmf_obelix',
                                   'pointcloud_reliability': 'reliable', 'pointcloud_queue_size': 10,
                                   'contact_diagnostics': contacts, 'contact_prefix': '/rmf/contact',
                                   'contact_parts': parts}], output='screen'),
        Node(package='gbplanner_gazebo', executable='rmf_controller_node',
             parameters=[str(share / 'config/rmf_controller.yaml'), sim_time], output='screen'),
        Node(package='gbplanner_gazebo', executable='rmf_imu_adapter_node',
             parameters=[sim_time, generated['metadata']['imu']['adapter_parameters']], output='screen'),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-world', 'edgar', '-file', str(robot), '-name', 'rmf_obelix',
                        '-x', '0', '-y', '0', '-z', '0.5'], output='screen'),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
            function=lambda context: temporary.cleanup())])),
    ]
    config = str(share / 'config/rmf_exploration.yaml')
    if LaunchConfiguration('mapping').perform(context).lower() == 'true':
        mapper = Node(package='gbplanner_voxblox', executable='voxblox_mapper_node',
                      name='voxblox_mapper', parameters=[config, sim_time,
                          {'diagnostics.snapshot_trace_file':
                           LaunchConfiguration('mapper_trace_file').perform(context)}], output='screen')
        nodes.extend([mapper, RegisterEventHandler(OnProcessExit(target_action=mapper,
            on_exit=[EmitEvent(event=Shutdown(reason='Voxblox mapper exited'))]))])
    if LaunchConfiguration('planning').perform(context).lower() == 'true':
        nodes.extend([
            Node(package='gbplanner_local', executable='local_planner_node', name='gbplanner_local',
                 parameters=[config, sim_time, {'diagnostics.map_trace_file':
                     LaunchConfiguration('map_trace_file').perform(context),
                     'diagnostics.validation_snapshot_directory':
                     LaunchConfiguration('validation_snapshot_directory').perform(context), 'global.auto_home':
                     LaunchConfiguration('auto_home').perform(context).lower() == 'true'}], output='screen'),
            Node(package='gbplanner_local', executable='control_node.py', name='gbplanner_control',
                 parameters=[config, sim_time, {'max_cycles':
                     int(LaunchConfiguration('max_cycles').perform(context))}], output='screen'),
            Node(package='gbplanner_execution', executable='path_follower_node.py',
                 parameters=[config, sim_time, {'trajectory_profile':
                     LaunchConfiguration('trajectory_profile').perform(context)}], output='screen'),
        ])
    if LaunchConfiguration('mission').perform(context).lower() == 'true':
        if any(LaunchConfiguration(name).perform(context).lower() != 'true' for name in ('planning','mapping')):
            raise ValueError('mission requires both planning:=true and mapping:=true')
        mission = Node(package='gbplanner_gazebo', executable='rmf_mission_node.py',
                       parameters=[sim_time, {'auto_start': False}], output='screen')
        nodes.extend([mission, RegisterEventHandler(OnProcessExit(target_action=mission,
            on_exit=[EmitEvent(event=Shutdown(reason='RMF mission node exited'))]))])
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mapping', default_value='true'),
        DeclareLaunchArgument('planning', default_value='false'),
        DeclareLaunchArgument('bridge', default_value='managed',
            description='managed: drain native transport callbacks before shutting down ROS; system: installed parameter_bridge'),
        DeclareLaunchArgument('mission', default_value='false'),
        DeclareLaunchArgument('trajectory_profile', default_value='quintic',
            description='quintic: stop at each vertex; quintic_collinear: pass compatible collinear vertices continuously; upstream: original timed positions'),
        DeclareLaunchArgument('contacts', default_value='false'),
        DeclareLaunchArgument('max_cycles', default_value='100'),
        DeclareLaunchArgument('auto_home', default_value='false'),
        DeclareLaunchArgument('map_trace_file', default_value=''),
        DeclareLaunchArgument('mapper_trace_file', default_value=''),
        DeclareLaunchArgument('validation_snapshot_directory', default_value=''),
        DeclareLaunchArgument('model_metadata_file', default_value=''),
        OpaqueFunction(function=start),
    ])
