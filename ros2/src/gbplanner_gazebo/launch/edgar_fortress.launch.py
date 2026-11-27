"""Native Fortress SMB and lidar; Edgar defaults with configurable scene inputs."""
import importlib.util
import json
import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable, EmitEvent, LogInfo)
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
    smb = Path(get_package_share_directory('gbplanner_smb'))
    assets = Path(get_package_share_directory('planner_gazebo_sim'))
    tracking = Path(get_package_share_directory('gbplanner_tracking'))
    model_file = Path(get_package_prefix('gbplanner_gazebo')) / 'lib/gbplanner_gazebo/fortress_model.py'
    spec = importlib.util.spec_from_file_location('fortress_model', model_file)
    model = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(model)
    renderer_mode = LaunchConfiguration('renderer').perform(context)
    if renderer_mode not in ('inherit', 'project'):
        raise ValueError('renderer must be inherit or project')
    renderer_environment = {}
    renderer_message = 'Fortress renderer: inherited process environment'
    if renderer_mode == 'project':
        prefix_text = LaunchConfiguration('renderer_prefix').perform(context).strip()
        if not prefix_text or not Path(prefix_text).is_absolute():
            raise ValueError('renderer:=project requires an absolute renderer_prefix')
        renderer_prefix = Path(prefix_text).resolve()
        renderer_file = model_file.with_name('fortress_renderer.py')
        renderer_spec = importlib.util.spec_from_file_location('fortress_renderer', renderer_file)
        renderer = importlib.util.module_from_spec(renderer_spec)
        renderer_spec.loader.exec_module(renderer)
        resolution = int(LaunchConfiguration('gpu_rays_min_cubemap_size').perform(context))
        # Overrides apply to the Gazebo server alone, not the launch process,
        # ROS bridge, mapper or planner. Inherit mode also preserves wrappers.
        renderer_environment = renderer.renderer_environment(
            renderer_prefix, resolution, context.environment)
        renderer_message = (f'Fortress renderer: {renderer_prefix}; minimum cubemap {resolution}; '
                            'original output ray counts and angles retained')
    contacts = LaunchConfiguration('contacts').perform(context).lower() == 'true'
    sensor_profile = LaunchConfiguration('sensor_profile').perform(context)
    if sensor_profile not in ('compact', 'upstream'):
        raise ValueError('sensor_profile must be compact or upstream')
    cameras = LaunchConfiguration('cameras').perform(context).lower() == 'true'
    if cameras and sensor_profile != 'upstream':
        raise ValueError('cameras:=true requires sensor_profile:=upstream')
    lateral_slip = float(LaunchConfiguration('lateral_slip_compliance').perform(context))
    separation_multiplier = float(LaunchConfiguration('wheel_separation_multiplier').perform(context))
    controllers_source = share / 'config/controllers.yaml'
    controllers_text = model.generate_controller_configuration(str(controllers_source),
        wheel_separation_multiplier=separation_multiplier)
    temporary = tempfile.TemporaryDirectory(prefix='gbplanner_fortress_')
    controllers = controllers_source
    if controllers_text != controllers_source.read_text():
        controllers = Path(temporary.name) / 'controllers.yaml'
        controllers.write_text(controllers_text)
    description = model.generate_robot_description(str(smb / 'urdf/smb.urdf'),
        str(controllers), str(smb), contact_diagnostics=contacts,
        lateral_slip_compliance=lateral_slip, sensor_profile=sensor_profile,
        gazebo_share=str(share), cameras_enabled=cameras)
    robot = Path(temporary.name) / 'smb.urdf'
    robot.write_text(description)
    sim_time = {'use_sim_time': True}
    resources = os.pathsep.join([str(assets / 'models'), str(smb.parent),
                                os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')])
    plugins = os.pathsep.join([str(Path(get_package_prefix('gz_ros2_control')) / 'lib'),
                              os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')])
    gazebo = ExecuteProcess(cmd=['ign', 'gazebo', '-r', '-s', '--headless-rendering',
                                 '-v', '3', LaunchConfiguration('world_file').perform(context)],
                            additional_env=renderer_environment, output='screen')
    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=['-world', LaunchConfiguration('world_name').perform(context),
                            '-file', str(robot), '-name', 'smb',
                            '-x', LaunchConfiguration('spawn_x').perform(context),
                            '-y', LaunchConfiguration('spawn_y').perform(context),
                            '-z', LaunchConfiguration('spawn_z').perform(context),
                            '-Y', LaunchConfiguration('spawn_yaw').perform(context)], output='screen')
    joint_state = Node(package='controller_manager', executable='spawner',
                       arguments=['joint_state_broadcaster', '--controller-manager-timeout', '60'],
                       parameters=[sim_time], output='screen')
    drive = Node(package='controller_manager', executable='spawner',
                 arguments=['smb_drive', '--controller-manager-timeout', '60'],
                 parameters=[sim_time], output='screen')
    bridges = ['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
               '/smb/ground_truth@nav_msgs/msg/Odometry[ignition.msgs.Odometry']
    bridge_remappings = [('/smb/ground_truth', 'gazebo/odometry_raw')]
    state_parameters = {'contact_diagnostics': contacts, 'lateral_slip_compliance': lateral_slip}
    if sensor_profile == 'upstream':
        state_parameters['sensor_names'] = ['velodyne', 'os1']
        for name, frame, prefix in (('velodyne', 'velodyne', '/smb/velodyne/scan'),
                                    ('os1', 'lidar_front', '/smb/os1/scan')):
            raw = 'gazebo/' + name + '_points_raw'
            bridges.append(prefix + '/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked')
            bridge_remappings.append((prefix + '/points', raw))
            state_parameters.update({f'sensor.{name}.frame': frame,
                                     f'sensor.{name}.input': raw,
                                     f'sensor.{name}.output': 'points/' + name,
                                     f'sensor.{name}.output_min_range': 0.2 if name == 'velodyne' else 0.7,
                                     f'sensor.{name}.output_max_range': 130.0 if name == 'velodyne' else 100.0})
        bridges.append('/smb/os1/imu/raw@sensor_msgs/msg/Imu[ignition.msgs.IMU')
        if cameras:
            for camera in ('rgb_cam_front', 'rgb_cam_left', 'rgb_cam_right'):
                bridges.extend((f'/{camera}/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
                                f'/{camera}/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'))
                bridge_remappings.append((f'/{camera}/camera_info', f'gazebo/{camera}_camera_info_raw'))
    else:
        bridges.append('/smb/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked')
        bridge_remappings.append(('/smb/scan/points', 'gazebo/points_raw'))
    if contacts:
        bridges.extend(f'/smb/contact/{part}@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts'
                       for part in ('base', 'lf', 'rf', 'lh', 'rh'))
    nodes = [
        LogInfo(msg=renderer_message),
        LogInfo(msg=f'SMB drive wheel_separation_multiplier: {separation_multiplier:.17g}'),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resources),
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', plugins),
        SetEnvironmentVariable('IGN_PARTITION', 'gbplanner_fortress_' + os.environ.get('ROS_DOMAIN_ID', '0')),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        gazebo,
        RegisterEventHandler(OnProcessExit(target_action=gazebo,
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo server exited'))])),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[sim_time, {'robot_description': description}], output='screen'),
        Node(package=bridge_package, executable=bridge_executable, parameters=[sim_time],
             arguments=bridges,
             remappings=bridge_remappings, output='screen'),
        Node(package='gbplanner_gazebo', executable='fortress_state_node.py',
             parameters=[sim_time, state_parameters], output='screen'),
        spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[joint_state])),
        RegisterEventHandler(OnProcessExit(target_action=joint_state, on_exit=[drive])),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
            function=lambda context: temporary.cleanup())])),
    ]
    if sensor_profile == 'upstream':
        nodes.append(Node(package='gbplanner_gazebo', executable='rmf_imu_adapter_node',
            parameters=[sim_time, str(share / 'config/smb_imu.yaml')], output='screen'))
        if cameras:
            nodes.append(Node(package='gbplanner_gazebo', executable='smb_camera_info_node.py',
                parameters=[sim_time], output='screen'))
    exploration = LaunchConfiguration('exploration_config').perform(context)
    sensing_parameters = ([str(smb / 'config/upstream_sensors.yaml')]
                          if sensor_profile == 'upstream' else [])
    if LaunchConfiguration('mapping').perform(context).lower() == 'true':
        voxel_capture_directory = LaunchConfiguration('voxel_capture_directory').perform(context)
        voxel_log_all_changes = LaunchConfiguration('voxel_capture_log_all_changes').perform(context).lower()
        if voxel_log_all_changes not in ('true', 'false'):
            raise ValueError('voxel_capture_log_all_changes must be true or false')
        voxel_capture_parameters = {
            'diagnostics.voxel_capture_directory': voxel_capture_directory,
            'diagnostics.voxel_capture_log_all_changes': voxel_log_all_changes == 'true'}
        if voxel_capture_directory:
            indices = json.loads(LaunchConfiguration('voxel_capture_indices').perform(context))
            if (not isinstance(indices, list) or not 3 <= len(indices) <= 3 * 2048 or len(indices) % 3
                    or any(type(value) is not int or abs(value) > 1000000 for value in indices)):
                raise ValueError('voxel_capture_indices requires 1-2048 flattened integer XYZ triples within +/-1000000')
            if len({tuple(indices[i:i+3]) for i in range(0, len(indices), 3)}) != len(indices) // 3:
                raise ValueError('voxel_capture_indices must not contain duplicate XYZ triples')
            voxel_capture_parameters['diagnostics.voxel_capture_indices'] = indices
        mapper = Node(package='gbplanner_voxblox', executable='voxblox_mapper_node',
                          name='voxblox_mapper', parameters=[exploration] + sensing_parameters +
                              [sim_time, voxel_capture_parameters, {'update_mesh_every_n_sec': 0.0,
                                  'diagnostics.snapshot_trace_file': LaunchConfiguration('snapshot_trace_file').perform(context)}], output='screen')
        nodes.extend([mapper, RegisterEventHandler(OnProcessExit(target_action=mapper,
            on_exit=[EmitEvent(event=Shutdown(reason='Voxblox mapper exited'))]))])
    if LaunchConfiguration('planning').perform(context).lower() == 'true':
        maneuvers = LaunchConfiguration('ground_maneuvers').perform(context).lower() == 'true'
        tracker = str(tracking / 'config/smb.yaml')
        planner_parameters = [exploration]
        profile = LaunchConfiguration('planner_profile').perform(context).strip()
        if profile:
            planner_parameters.append(str(Path(profile).expanduser()))
        planner_parameters.extend(sensing_parameters)
        sensor_overrides = ({'sensor.smb_lidar.vertical_fov': 0.5235987755982988}
                            if sensor_profile == 'compact' else {})
        nodes.extend([
            Node(package='gbplanner_local', executable='local_planner_node', name='gbplanner_local',
                 parameters=planner_parameters + [sim_time, sensor_overrides,
                             {'diagnostics.map_trace_file': LaunchConfiguration('map_trace_file').perform(context),
                              'diagnostics.home_snapshot_directory': LaunchConfiguration('home_snapshot_directory').perform(context),
                              'global.auto_home': LaunchConfiguration('auto_home').perform(context).lower() == 'true'}],
                 output='screen'),
            Node(package='gbplanner_local', executable='control_node.py', name='gbplanner_control',
                 parameters=[exploration, sim_time,
                             {'max_cycles': int(LaunchConfiguration('max_cycles').perform(context)),
                              'ground_maneuver.enabled': maneuvers}],
                 output='screen'),
            Node(package='gbplanner_smb', executable='velocity_mux_node.py',
                 parameters=[str(smb / 'config/teleop.yaml'), sim_time], output='screen'),
            Node(package='gbplanner_tracking', executable='pure_pursuit_node',
                 parameters=[tracker, sim_time, {'allow_ground_maneuver': maneuvers}],
                 remappings=[('cmd_vel', 'external/cmd_vel')], output='screen'),
            Node(package='gbplanner_execution', executable='path_follower_node.py',
                 parameters=[tracker, sim_time, {'allow_ground_maneuver': maneuvers}], output='screen'),
        ])
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory('gbplanner_gazebo'))
    smb = Path(get_package_share_directory('gbplanner_smb'))
    return LaunchDescription([
        DeclareLaunchArgument('world_file', default_value=str(share / 'worlds/edgar_fortress.sdf'),
            description='Fortress world SDF; world_name must match its world element'),
        DeclareLaunchArgument('world_name', default_value='edgar'),
        DeclareLaunchArgument('spawn_x', default_value='0'),
        DeclareLaunchArgument('spawn_y', default_value='0'),
        DeclareLaunchArgument('spawn_z', default_value='0.5'),
        DeclareLaunchArgument('spawn_yaw', default_value='0'),
        DeclareLaunchArgument('exploration_config', default_value=str(smb / 'config/edgar_exploration.yaml'),
            description='Complete scene parameters for mapper, planner and PCI'),
        DeclareLaunchArgument('mapping', default_value='true'),
        DeclareLaunchArgument('planning', default_value='false'),
        DeclareLaunchArgument('bridge', default_value='managed',
            description='managed: drain native transport callbacks before shutting down ROS; system: installed parameter_bridge'),
        DeclareLaunchArgument('renderer', default_value='inherit', choices=['inherit', 'project'],
            description='inherit: keep the process environment; project: isolated patched renderer for Gazebo only'),
        DeclareLaunchArgument('renderer_prefix', default_value='',
            description='Absolute isolated rendering 6.6.4 install prefix; required for renderer:=project'),
        DeclareLaunchArgument('gpu_rays_min_cubemap_size', default_value='512',
            choices=['128', '256', '512', '1024'],
            description='Internal cubemap minimum for renderer:=project; output ray counts are unchanged'),
        DeclareLaunchArgument('sensor_profile', default_value='compact',
            description='compact: existing short-range lidar; upstream: original SMB dual lidar head'),
        DeclareLaunchArgument('cameras', default_value='false',
            description='Enable all three original 1440x1080, 20 Hz RGB cameras (upstream only)'),
        DeclareLaunchArgument('planner_profile', default_value='',
            description='Optional planner-only YAML overlay, such as gbplanner_smb/config/upstream_planning.yaml'),
        DeclareLaunchArgument('max_cycles', default_value='100'),
        DeclareLaunchArgument('ground_maneuvers', default_value='false',
            description='Explicit pre-departure SE2 routes for ordinary ground local/search tasks'),
        DeclareLaunchArgument('contacts', default_value='false'),
        DeclareLaunchArgument('auto_home', default_value='true'),
        DeclareLaunchArgument('map_trace_file', default_value=''),
        DeclareLaunchArgument('home_snapshot_directory', default_value=''),
        DeclareLaunchArgument('snapshot_trace_file', default_value=''),
        DeclareLaunchArgument('voxel_capture_directory', default_value=''),
        DeclareLaunchArgument('voxel_capture_indices', default_value='[]'),
        DeclareLaunchArgument('voxel_capture_log_all_changes', default_value='true',
            description='false logs only occupancy/presence transitions; raw scan capture limits still apply'),
        DeclareLaunchArgument('lateral_slip_compliance', default_value='0.0',
            description='Optional native WheelSlip lateral compliance; 0 preserves upstream rigid contact'),
        DeclareLaunchArgument('wheel_separation_multiplier', default_value='1.0',
            description='Positive diff_drive kinematic track scale; 1 uses the nominal wheel separation'),
        OpaqueFunction(function=start),
    ])
