"""Native dual-lidar/IMU scene for independent estimate-driven mapping.

The caller owns calibration, estimate TF, mapper and bounded motion. This
launch retains both original GPU lidars and their physical sensor head. Its
ground-truth bridge is an evaluation stream only; the point-cloud adapter has
neither a truth odometry input nor a production TF output.
"""
from pathlib import Path
import hashlib
import importlib.util
import json
import os
import shutil
import sys
import xml.etree.ElementTree as ET


sys.dont_write_bytecode = True
TRIAL_ROOT = Path(__file__).resolve().parent
WORKSPACE_ROOT = TRIAL_ROOT.parents[1]
WORLD_NAME = 'estimated_mapping_trial'


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def prepare_artifacts(output_dir, gazebo_share, smb_share, model_file,
                      environment, velocity_window=1, imu_seed=1):
    """Reuse wheel/IMU preparation, then restore the complete native sensors.

    Only this new output directory is changed. The legacy generated world
    filename stays flat_world.sdf so the existing analysis can read the run;
    its contents and provenance explicitly identify this mapping scene.
    """
    base_file = TRIAL_ROOT.parent / 'state_estimation_shadow/sensor_trial.launch.py'
    base = load_module('mapping_base_sensor_preparation', base_file)
    files = base.prepare_artifacts(output_dir, gazebo_share, smb_share, model_file,
                                  environment, velocity_window, imu_seed)
    model = load_module('mapping_native_fortress_model', model_file)
    description = model.generate_robot_description(
        str(Path(smb_share) / 'urdf/smb.urdf'), str(files['controllers.yaml']),
        str(smb_share), contact_diagnostics=False, lateral_slip_compliance=0.,
        sensor_profile='upstream', gazebo_share=str(gazebo_share), cameras_enabled=False)
    robot = ET.fromstring(description)
    ET.indent(robot, space='  ')
    # Regeneration must restore sensors without changing masses, collision
    # geometry, fixed mounts or wheel joints from the existing experiment.
    before = ET.parse(files['smb.urdf']).getroot()
    def geometry(tree):
        return [ET.tostring(item, encoding='unicode') for item in tree
                if item.tag in ('link', 'joint')]
    if geometry(before) != geometry(robot):
        raise ValueError('Restoring native lidars changed robot links or joints')
    sensors = list(robot.iter('sensor'))
    if sorted(sensor.get('type') for sensor in sensors) != ['gpu_lidar', 'gpu_lidar', 'imu']:
        raise ValueError('Expected two original GPU lidars and one native IMU')
    lidar_records = []
    for sensor in sensors:
        if sensor.get('type') != 'gpu_lidar':
            continue
        lidar = sensor.find('lidar')
        lidar_records.append({
            'name': sensor.get('name'), 'topic': sensor.findtext('topic'),
            'update_rate_hz': float(sensor.findtext('update_rate')),
            'horizontal_samples': int(lidar.findtext('scan/horizontal/samples')),
            'vertical_samples': int(lidar.findtext('scan/vertical/samples')),
            'horizontal_min_angle': float(lidar.findtext('scan/horizontal/min_angle')),
            'horizontal_max_angle': float(lidar.findtext('scan/horizontal/max_angle')),
            'vertical_min_angle': float(lidar.findtext('scan/vertical/min_angle')),
            'vertical_max_angle': float(lidar.findtext('scan/vertical/max_angle')),
            'ray_min_range_m': float(lidar.findtext('range/min')),
            'ray_max_range_m': float(lidar.findtext('range/max')),
            'native_noise_stddev_m': float(lidar.findtext('noise/stddev')),
        })
    if sorted((row['horizontal_samples'], row['vertical_samples'], row['update_rate_hz'])
              for row in lidar_records) != [(440, 16, 10.), (440, 32, 10.)]:
        raise ValueError('Original dual-lidar sampling configuration changed')
    files['smb.urdf'].write_text('<?xml version="1.0"?>\n' +
                               ET.tostring(robot, encoding='unicode') + '\n')
    world_source = TRIAL_ROOT / 'mapping_world.sdf'
    shutil.copyfile(world_source, files['flat_world.sdf'])
    manifest = json.loads(files['sensor_manifest.json'].read_text())
    manifest.update(
        scope='Independent native-lidar mapping scene; the caller provides estimated map->base_link TF',
        world=WORLD_NAME, world_source=str(world_source),
        rendering_sensors_removed_from_generated_copy=[],
        native_sensor_profile='Original upstream dual GPU lidar and native OS1 IMU; cameras disabled',
        native_lidars=lidar_records,
        pointclouds={
            'velodyne': {'raw': 'gazebo/velodyne_points_raw', 'output': 'points/velodyne',
                         'frame': 'velodyne', 'output_range_m': [.2, 130.]},
            'os1': {'raw': 'gazebo/os1_points_raw', 'output': 'points/os1',
                    'frame': 'lidar_front', 'output_range_m': [.7, 100.]},
            'reliability': 'reliable', 'relay_queue_size': 1000,
            'source_timestamps_preserved': True, 'synthetic_scans': False},
        pointcloud_adapter={
            'implementation': 'Unchanged fortress_state_node.py, with odometry and TF isolated',
            'odometry_input': '/shadow/unused_odometry',
            'odometry_output': '/shadow/unused_state_odometry',
            'tf_output': '/shadow/unused_state_tf',
            'truth_input_connected': False},
        scene={'room_inner_bounds_xy_m': [-2., 8., -3., 4.], 'wall_height_m': 2.,
               'path_xy_m': [[0., 0.], [1.2, 0.], [2.4, .4], [3.2, .9], [4.2, 1.2]],
               'meaning': 'Known flat geometric scene for mapping evaluation, not a map input'},
    )
    manifest['source_sha256'].pop(str(base.TRIAL_ROOT / 'flat_world.sdf'), None)
    for source in (Path(__file__), world_source, model_file.with_name('fortress_state_node.py')):
        manifest['source_sha256'][str(source)] = sha256(source)
    manifest['generated_sha256'] = {name: sha256(path) for name, path in files.items()
                                    if name != 'sensor_manifest.json'}
    files['sensor_manifest.json'].write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    return files


def start(context):
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from launch.actions import ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable, EmitEvent, LogInfo
    from launch.event_handlers import OnProcessExit
    from launch.events import Shutdown
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node

    environment = context.environment
    domain, partition = environment.get('ROS_DOMAIN_ID', ''), environment.get('IGN_PARTITION', '')
    if not domain.isdecimal() or not 1 <= int(domain) <= 231 or not partition:
        raise ValueError('Caller must supply an isolated ROS_DOMAIN_ID and IGN_PARTITION')
    if environment.get('ROS_LOCALHOST_ONLY') != '1':
        raise ValueError('This local simulation requires ROS_LOCALHOST_ONLY=1')
    output = Path(LaunchConfiguration('output_dir').perform(context)).expanduser()
    if not output.is_absolute():
        raise ValueError('output_dir must be an absolute new directory')
    share = Path(get_package_share_directory('gbplanner_gazebo'))
    smb = Path(get_package_share_directory('gbplanner_smb'))
    model_file = Path(get_package_prefix('gbplanner_gazebo')) / 'lib/gbplanner_gazebo/fortress_model.py'
    files = prepare_artifacts(output, share, smb, model_file, environment,
                              int(LaunchConfiguration('velocity_window').perform(context)),
                              int(LaunchConfiguration('imu_seed').perform(context)))
    renderer = load_module('mapping_project_renderer', model_file.with_name('fortress_renderer.py'))
    renderer_prefix = WORKSPACE_ROOT / 'install_renderer'
    renderer_environment = renderer.renderer_environment(renderer_prefix, 512, environment)
    renderer_environment['LIBGL_ALWAYS_SOFTWARE'] = '1'
    manifest = json.loads(files['sensor_manifest.json'].read_text())
    manifest['renderer'] = {'prefix': str(renderer_prefix), 'minimum_cubemap_size': 512,
                            'environment_scope': 'Gazebo process only',
                            'output_ray_configuration_unchanged': True}
    manifest['source_sha256'][str(model_file.with_name('fortress_renderer.py'))] = sha256(
        model_file.with_name('fortress_renderer.py'))
    files['sensor_manifest.json'].write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    sim_time = {'use_sim_time': True}
    resources = os.pathsep.join((str(smb.parent), str(share / 'models'),
                                 environment.get('IGN_GAZEBO_RESOURCE_PATH', '')))
    plugins = os.pathsep.join((str(Path(get_package_prefix('gz_ros2_control')) / 'lib'),
                               environment.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')))
    gazebo = ExecuteProcess(cmd=['ign', 'gazebo', '-r', '-s', '--headless-rendering', '-v', '3',
                                 str(files['flat_world.sdf'])],
                            additional_env=renderer_environment, output='screen')
    spawn = Node(package='ros_gz_sim', executable='create', parameters=[sim_time],
                 arguments=['-world', WORLD_NAME, '-file', str(files['smb.urdf']),
                            '-name', 'smb', '-x', '0', '-y', '0', '-z', '0.5'], output='screen')
    joint_state = Node(package='controller_manager', executable='spawner', parameters=[sim_time],
                       arguments=['joint_state_broadcaster', '--controller-manager-timeout', '60'], output='screen')
    drive = Node(package='controller_manager', executable='spawner', parameters=[sim_time],
                 arguments=['smb_drive', '--controller-manager-timeout', '60'], output='screen')
    def next_on_success(next_action, description):
        def completed(event, context):
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason=description + ' failed'))]
            return [next_action] if next_action is not None else []
        return completed

    return [
        LogInfo(msg='Independent mapping sensors: original dual lidars, project renderer 512; no truth pose TF'),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resources),
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', plugins),
        gazebo,
        RegisterEventHandler(OnProcessExit(target_action=gazebo,
            on_exit=[EmitEvent(event=Shutdown(reason='Mapping-scene Gazebo exited'))])),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[sim_time, {'robot_description': files['smb.urdf'].read_text()}], output='screen'),
        Node(package='gbplanner_gazebo', executable='managed_bridge_node', parameters=[sim_time],
             arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                        '/smb/ground_truth@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                        '/smb/os1/imu/raw@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                        '/smb/velodyne/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
                        '/smb/os1/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'],
             remappings=[('/smb/ground_truth', '/shadow/ground_truth'),
                         ('/smb/velodyne/scan/points', 'gazebo/velodyne_points_raw'),
                         ('/smb/os1/scan/points', 'gazebo/os1_points_raw')], output='screen'),
        Node(package='gbplanner_gazebo', executable='fortress_state_node.py', name='mapping_lidar_adapter',
             parameters=[sim_time, {'sensor_names': ['velodyne', 'os1'],
                'pointcloud_reliability': 'reliable', 'pointcloud_queue_size': 1000,
                'sensor.velodyne.frame': 'velodyne',
                'sensor.velodyne.input': 'gazebo/velodyne_points_raw',
                'sensor.velodyne.output': 'points/velodyne',
                'sensor.velodyne.output_min_range': .2, 'sensor.velodyne.output_max_range': 130.,
                'sensor.os1.frame': 'lidar_front',
                'sensor.os1.input': 'gazebo/os1_points_raw',
                'sensor.os1.output': 'points/os1',
                'sensor.os1.output_min_range': .7, 'sensor.os1.output_max_range': 100.}],
             remappings=[('gazebo/odometry_raw', '/shadow/unused_odometry'),
                         ('odometry', '/shadow/unused_state_odometry'),
                         ('/tf', '/shadow/unused_state_tf'),
                         ('gazebo_state', '/shadow/mapping_sensor_state')], output='screen'),
        Node(package='gbplanner_gazebo', executable='rmf_imu_adapter_node',
             parameters=[str(files['imu.yaml'])], output='screen'),
        spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn,
            on_exit=next_on_success(joint_state, 'SMB spawn'))),
        RegisterEventHandler(OnProcessExit(target_action=joint_state,
            on_exit=next_on_success(drive, 'Joint-state broadcaster startup'))),
        RegisterEventHandler(OnProcessExit(target_action=drive,
            on_exit=next_on_success(None, 'Wheel controller startup'))),
    ]


def generate_launch_description():
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, OpaqueFunction
    return LaunchDescription([
        DeclareLaunchArgument('output_dir', description='Absolute new directory for generated scene and configuration'),
        DeclareLaunchArgument('velocity_window', default_value='1'),
        DeclareLaunchArgument('imu_seed', default_value='1'),
        OpaqueFunction(function=start),
    ])
