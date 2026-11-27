"""Isolated flat-ground SMB wheel/IMU observations for a shadow estimator.

This launch does not create a command publisher, planner, mapper, follower,
estimator or world->base TF. The experiment runner owns bounded motion and
recording. All generated models/configuration stay in the supplied output_dir.
"""
from pathlib import Path
import hashlib
import importlib.util
import json
import os
import shutil
import sys
import xml.etree.ElementTree as ET

import yaml


sys.dont_write_bytecode = True
TRIAL_ROOT = Path(__file__).resolve().parent
TWIST_COVARIANCE_PRIOR = [0.0004, 0.01, 0.01, 0.01, 0.01, 0.04]


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def prepare_artifacts(output_dir, gazebo_share, smb_share, model_file, environment,
                      velocity_window=10, imu_seed=1):
    """Generate trial copies using the existing installed model implementation."""
    output = Path(output_dir).expanduser().resolve()
    gazebo_share, smb_share, model_file = map(Path, (gazebo_share, smb_share, model_file))
    files = {name: output / name for name in
             ('smb.urdf', 'controllers.yaml', 'imu.yaml', 'flat_world.sdf', 'sensor_manifest.json')}
    if any(p.exists() for p in files.values()):
        raise FileExistsError('Use a new output_dir: generated sensor evidence will not be overwritten')
    output.mkdir(parents=True, exist_ok=True)

    controllers_source = gazebo_share / 'config/controllers.yaml'
    controllers = yaml.safe_load(controllers_source.read_text())
    drive = controllers['smb_drive']['ros__parameters']
    if drive['open_loop'] or not drive['position_feedback'] or drive['enable_odom_tf']:
        raise ValueError('Expected feedback wheel odometry without its own dynamic TF')
    drive['twist_covariance_diagonal'] = TWIST_COVARIANCE_PRIOR
    # The default ten-sample mean delays a 50 Hz wheel-speed observation by
    # about 90 ms. The fast-observation trial changes only this generated
    # estimator input setting; feedback wheel physics and limits are retained.
    if velocity_window not in (1, 10):
        raise ValueError('Trial velocity_window must be 1 or 10')
    drive['velocity_rolling_window_size'] = velocity_window
    files['controllers.yaml'].write_text(yaml.safe_dump(controllers, sort_keys=False))

    spec = importlib.util.spec_from_file_location('shadow_fortress_model', model_file)
    model = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(model)
    original_description = model.generate_robot_description(
        str(smb_share / 'urdf/smb.urdf'), str(files['controllers.yaml']), str(smb_share),
        contact_diagnostics=False, lateral_slip_compliance=0., sensor_profile='upstream',
        gazebo_share=str(gazebo_share), cameras_enabled=False)
    robot = ET.fromstring(original_description)
    removed = []
    # Preserve sensor-head links, masses, joints and all collision geometry.
    # Remove only non-IMU sensor definitions in this generated experiment copy.
    for parent in robot.iter():
        for sensor in list(parent):
            if sensor.tag == 'sensor' and sensor.get('type') != 'imu':
                removed.append({'name': sensor.get('name'), 'type': sensor.get('type'),
                                'reference': parent.get('reference')})
                parent.remove(sensor)
    sensors = list(robot.iter('sensor'))
    if len(sensors) != 1 or sensors[0].get('type') != 'imu':
        raise ValueError('The experiment requires exactly one native IMU and no rendered sensors')
    if float(sensors[0].findtext('update_rate')) != 100.:
        raise ValueError('Expected the original 100 Hz OS1 IMU')
    ET.indent(robot, space='  ')
    files['smb.urdf'].write_text('<?xml version="1.0"?>\n' + ET.tostring(robot, encoding='unicode') + '\n')

    imu_source = gazebo_share / 'config/smb_imu.yaml'
    imu = yaml.safe_load(imu_source.read_text())
    params = imu['gbplanner_rmf_imu_adapter']['ros__parameters']
    params['use_sim_time'] = True
    params['random_seed'] = int(imu_seed)
    params['status_service'] = '/shadow/imu_state'
    # The existing adapter has no enabled switch for these three built-in
    # streams. Isolate their topics and provide no bridge/publisher to inputs.
    for name in ('adis', 'ground_truth', 'os0'):
        params[name + '.input_topic'] = '/shadow/unused/' + name + '/raw'
        params[name + '.output_topic'] = '/shadow/unused/' + name + '/imu'
    params['os0.bias_topic'] = ''
    params['os0.calibration_service'] = ''
    # Do not expose the simulator's exact bias as an estimation observation.
    # No calibration service is used to reset the simulator's true bias.
    params['smb_os1.bias_topic'] = ''
    params['smb_os1.calibration_service'] = ''
    files['imu.yaml'].write_text(yaml.safe_dump(imu, sort_keys=False))
    shutil.copyfile(TRIAL_ROOT / 'flat_world.sdf', files['flat_world.sdf'])

    def preserved_geometry(description):
        tree = ET.fromstring(description)
        # These are the complete physical and visual robot links/joints.
        return [ET.tostring(e, encoding='unicode') for e in tree
                if e.tag in ('link', 'joint')]

    # ElementTree indentation changes whitespace; normalize both copies first.
    original_tree = ET.fromstring(original_description)
    ET.indent(original_tree, space='  ')
    normalized_original = ET.tostring(original_tree, encoding='unicode')
    if preserved_geometry(normalized_original) != preserved_geometry(files['smb.urdf'].read_text()):
        raise ValueError('Physical robot links or joints changed while removing sensors')
    manifest = {
        'scope': 'flat-ground sensor-only side experiment; no production state replacement',
        'world': 'estimation_trial', 'physics_engine': 'ignition-physics-dartsim-plugin',
        'initial_model_pose': [0., 0., .5, 0., 0., 0.],
        'ros_domain_id': environment['ROS_DOMAIN_ID'],
        'ign_partition': environment['IGN_PARTITION'],
        'rendering_sensors_removed_from_generated_copy': removed,
        'robot_links_joints_inertias_collision_geometry_unchanged': True,
        'native_sensor_profile': 'upstream SMB sensor head; IMU retained, lidar/cameras not run',
        'wheel_odometry': {'expected_topic': '/smb_drive/odom',
                           'velocity_rolling_window_size': velocity_window,
                           'position_feedback': True, 'open_loop': False,
                           'publish_tf': False,
                           'twist_covariance_diagonal': TWIST_COVARIANCE_PRIOR,
                           'covariance_status': 'declared trial prior; not empirically calibrated'},
        'imu': {'input_topic': '/smb/os1/imu/raw', 'output_topic': '/os1_cloud_node/imu',
                'random_seed': int(imu_seed),
                'frame': 'os1_imu', 'nominal_rate_hz': 100.,
                'exact_bias_topic_disabled': True, 'reset_bias_calibration_service_disabled': True,
                'unused_streams': 'adapter creates adis/os0/ground_truth streams without enabled flags; '
                                  'input/output names isolated under /shadow/unused; no inputs bridged'},
        'truth': {'topic': '/shadow/ground_truth', 'use': 'independent evaluation only',
                  'native_pose_retained': True, 'dynamic_tf_published': False},
        'automatic_command_publishers': 0,
        'planner_mapper_tracker_estimator_started': False,
        'source_sha256': {str(p): sha256(p) for p in
                          (Path(__file__), TRIAL_ROOT / 'flat_world.sdf', model_file,
                           model_file.with_name('fortress_smb_sensors.py'),
                           smb_share / 'urdf/smb.urdf', controllers_source, imu_source)},
        'generated_sha256': {name: sha256(path) for name, path in files.items()
                             if name != 'sensor_manifest.json'},
    }
    files['sensor_manifest.json'].write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    return files


def start(context):
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from launch.actions import ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable, EmitEvent, LogInfo
    from launch.event_handlers import OnProcessExit
    from launch.events import Shutdown
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node

    # Isolation belongs to the caller, so a launch never silently joins the
    # default ROS graph or somebody else's Ignition transport partition.
    environment = context.environment
    domain = environment.get('ROS_DOMAIN_ID', '')
    partition = environment.get('IGN_PARTITION', '')
    if not domain.isdecimal() or not 1 <= int(domain) <= 231 or not partition:
        raise ValueError('Set an isolated nonzero ROS_DOMAIN_ID and a unique IGN_PARTITION before launch')
    if environment.get('ROS_LOCALHOST_ONLY') != '1':
        raise ValueError('This local-only experiment requires ROS_LOCALHOST_ONLY=1')
    output_text = LaunchConfiguration('output_dir').perform(context).strip()
    if not output_text or not Path(output_text).expanduser().is_absolute():
        raise ValueError('output_dir must be an absolute path for trial artifacts')
    share = Path(get_package_share_directory('gbplanner_gazebo'))
    smb = Path(get_package_share_directory('gbplanner_smb'))
    model_file = Path(get_package_prefix('gbplanner_gazebo')) / 'lib/gbplanner_gazebo/fortress_model.py'
    velocity_window = int(LaunchConfiguration('velocity_window').perform(context))
    imu_seed = int(LaunchConfiguration('imu_seed').perform(context))
    files = prepare_artifacts(output_text, share, smb, model_file, environment,
                              velocity_window=velocity_window, imu_seed=imu_seed)
    sim_time = {'use_sim_time': True}
    resources = os.pathsep.join([str(smb.parent), str(share / 'models'),
                                environment.get('IGN_GAZEBO_RESOURCE_PATH', '')])
    plugins = os.pathsep.join([str(Path(get_package_prefix('gz_ros2_control')) / 'lib'),
                              environment.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')])
    gazebo = ExecuteProcess(cmd=['ign', 'gazebo', '-r', '-s', '-v', '3',
                                 str(files['flat_world.sdf'])], output='screen')
    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=['-world', 'estimation_trial', '-file', str(files['smb.urdf']),
                            '-name', 'smb', '-x', '0', '-y', '0', '-z', '0.5'],
                 parameters=[sim_time], output='screen')
    joint_state = Node(package='controller_manager', executable='spawner',
                       arguments=['joint_state_broadcaster', '--controller-manager-timeout', '60'],
                       parameters=[sim_time], output='screen')
    drive = Node(package='controller_manager', executable='spawner',
                 arguments=['smb_drive', '--controller-manager-timeout', '60'],
                 parameters=[sim_time], output='screen')

    def next_on_success(next_action, description):
        def completed(event, context):
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason=description + ' failed'))]
            return [next_action] if next_action is not None else []
        return completed

    return [
        LogInfo(msg='Shadow sensor trial: native wheel feedback and IMU only; no motion commands or production nodes'),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resources),
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', plugins),
        gazebo,
        RegisterEventHandler(OnProcessExit(target_action=gazebo,
            on_exit=[EmitEvent(event=Shutdown(reason='Trial Gazebo server exited'))])),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[sim_time, {'robot_description': files['smb.urdf'].read_text()}], output='screen'),
        Node(package='gbplanner_gazebo', executable='managed_bridge_node', parameters=[sim_time],
             arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                        '/smb/ground_truth@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                        '/smb/os1/imu/raw@sensor_msgs/msg/Imu[ignition.msgs.IMU'],
             remappings=[('/smb/ground_truth', '/shadow/ground_truth')], output='screen'),
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
        DeclareLaunchArgument('output_dir', description='Absolute new output directory for generated trial artifacts'),
        DeclareLaunchArgument('velocity_window', default_value='10',
                              description='Generated wheel-speed averaging window: 10 baseline, 1 immediate'),
        DeclareLaunchArgument('imu_seed', default_value='1', description='Explicit IMU noise seed'),
        OpaqueFunction(function=start),
    ])
