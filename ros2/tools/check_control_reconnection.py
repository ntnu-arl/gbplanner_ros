#!/usr/bin/env python3
"""Check ordinary PCI reconnection with real planner, validator and follower.

Thin service relays delay unchanged native responses while synthetic odometry
moves by 0.2 m. They do not manufacture planning or validation results. The
trajectory consumer interpolates follower commands; this is not robot dynamics.
"""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import time


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def record(pose):
    p, q = pose.position, pose.orientation
    return [p.x, p.y, p.z, q.x, q.y, q.z, q.w]


def recording_follower(output, ros_args):
    """Run the installed follower, recording its unchanged accept input."""
    from ament_index_python.packages import get_package_prefix
    sys.path.insert(0, str(Path(get_package_prefix('gbplanner_execution'))/'lib/gbplanner_execution'))
    import path_follower_node
    native_follower = path_follower_node.TrajectoryFollower

    class RecordingFollower(native_follower):
        def accept(self, request):
            with output.open('a') as stream:
                stream.write(json.dumps({'path': [record(p.pose) for p in request.path],
                    'frames': [p.header.frame_id for p in request.path],
                    'observed_position': list(self.current) if self.current is not None else None,
                    'manual_path': request.manual_path, 'bound_mode': request.bound_mode})+'\n')
            return super().accept(request)

    # The production entry point retains its executor, cleanup and signals.
    # Only the callback input is recorded; admission and execution stay native.
    path_follower_node.TrajectoryFollower = RecordingFollower
    sys.argv = [sys.argv[0], *ros_args]
    path_follower_node.main()
    return 0


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=workspace/'results/control_reconnection/summary.json')
    parser.add_argument('--timeout', type=float, default=75.)
    parser.add_argument('--case', action='append', choices=('preview', 'connected', 'blocked', 'bounded', 'ground_tsdf'),
                        help='Run only this case; repeat to select more than one (default: all)')
    parser.add_argument('--follower-goal-log', type=Path, help=argparse.SUPPRESS)
    args, extra = parser.parse_known_args()
    if args.follower_goal_log is not None:
        return recording_follower(args.follower_goal_log, extra)
    if extra:
        parser.error('unrecognized arguments: '+' '.join(extra))
    selected_cases = set(args.case or ('preview', 'connected', 'blocked', 'bounded', 'ground_tsdf'))
    require(math.isfinite(args.timeout) and args.timeout > 0., 'Invalid timeout')
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100+os.getpid()%80))
    started, fixtures, initialized = time.monotonic(), [], False
    deadline = started+args.timeout
    summary = {'status': 'failed', 'scope': __doc__, 'launches': [], 'checks': {},
               'selected_cases': sorted(selected_cases)}
    try:
        import rclpy
        from ament_index_python.packages import get_package_prefix
        from nav_msgs.msg import Odometry, Path as RosPath
        from planner_msgs.srv import PciTrigger, PlannerSrv, ValidatePath
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rclpy.task import Future
        from sensor_msgs.msg import PointCloud2, PointField
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from trajectory_msgs.msg import MultiDOFJointTrajectory
        sys.path.insert(0, str(Path(get_package_prefix('gbplanner_execution'))/'lib/gbplanner_execution'))
        from execution_demo import decode, sample

        rclpy.init(args=[])
        initialized = True
        durable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        origin, dimensions, resolution = (-.5, -.5, -.25), (70, 40, 10), .05
        parameters = {
            'map.resolution': resolution, 'map.origin': list(origin), 'map.dimensions': list(dimensions),
            'global.enable': False, 'robot.size': [.04, .04, .04],
            'sampling.min': [-.1, -.1, -.1], 'sampling.max': [2.1, .1, .1],
            'planning.max_vertices': 5, 'planning.max_attempts': 6, 'planning.max_edges': 20,
            'planning.graph_mode': False, 'planning.edge_min': .05, 'planning.edge_max': 2.,
            'planning.overshoot': 0., 'planning.path_direction_penalty': 0.,
            'planning.traverse_length_max': 10., 'planning.traverse_time_max': 100.,
            'planning.path_safety_enhance_enable': False,
            'planning.path_interpolation_distance': .5,
            'gain.free': 1., 'sensor.range': .4, 'sensor.horizontal_fov': 1.,
            'sensor.horizontal_resolution': .5, 'sensor.vertical_fov': .2,
            'sensor.vertical_resolution': .2,
        }
        for axis, value in [('x', 2.), ('y', 0.), ('z', 0.), ('heading', 0.)]:
            parameters[f'sampling.{axis}.distribution'] = 'constant'
            parameters[f'sampling.{axis}.constant'] = value

        def make_map(blocked):
            cloud = PointCloud2()
            cloud.header.frame_id, cloud.height = 'map', 1
            cloud.width, cloud.point_step = math.prod(dimensions), 16
            cloud.row_step, cloud.is_dense = cloud.width*16, True
            cloud.fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                            for i, name in enumerate(('x', 'y', 'z'))]
            cloud.fields.append(PointField(name='occupancy', offset=12, datatype=PointField.INT8, count=1))
            data, offset = bytearray(cloud.row_step), 0
            for z in range(dimensions[2]):
                for y in range(dimensions[1]):
                    for x in range(dimensions[0]):
                        point = [origin[i]+(index+.5)*resolution for i, index in enumerate((x, y, z))]
                        # The original x-axis route and both observed endpoints
                        # stay free. Only the new lateral connector crosses this.
                        obstacle = blocked and abs(point[0]) < .1 and .05 < point[1] < .15
                        struct.pack_into('<fffbxxx', data, offset, *point, 100 if obstacle else 0)
                        offset += 16
            cloud.data = bytes(data)
            return cloud

        class Probe(Node):
            def __init__(self, label, blocked=False, validation_moves=0, ground_input=None):
                namespace = f'/control_reconnect_{os.getpid()}_{label}'
                super().__init__('probe', namespace=namespace)
                self.label, self.fixture_namespace = label, namespace
                self.processes, self.service_clients, self.waits = [], {}, []
                self.closed = False
                self.position = tuple(ground_input['position']) if ground_input else (0., 0., 0., 0.)
                self.command = None
                self.ground_input = ground_input
                self.validation_moves = validation_moves
                self.planned, self.plan_calls, self.validations = [], 0, []
                self.planning_metrics, self.planner_readiness = [], None
                self.follower_validations = []
                self.commands, self.trajectories, self.control = [], [], None
                self.control_history, self.execution_history = [], []
                self.goal_log = output.parent/f'{label}_follower_goals.jsonl'
                self.goal_log.write_text('')
                self.maps = self.create_publisher(PointCloud2, 'voxel_map', durable)
                self.odometry = self.create_publisher(Odometry, 'odometry', 10)
                self.create_subscription(RosPath, 'command_path', self.commands.append, durable)
                self.create_subscription(String, 'control_status', self.receive_control, durable)
                self.create_subscription(String, 'execution_status',
                    lambda message:self.execution_history.append(json.loads(message.data)), durable)
                self.create_subscription(String, 'planning_metrics',
                    lambda message:self.planning_metrics.append(json.loads(message.data)), durable)
                self.create_subscription(MultiDOFJointTrajectory, 'command/trajectory', self.consume, 10)
                self.relay_group = ReentrantCallbackGroup()
                self.native_planner = self.create_client(PlannerSrv, 'native_planner', callback_group=self.relay_group)
                self.native_validator = self.create_client(ValidatePath, 'validate_path', callback_group=self.relay_group)
                self.create_service(PlannerSrv, 'planner', self.plan_relay, callback_group=self.relay_group)
                self.create_service(ValidatePath, 'control_validate', self.validation_relay, callback_group=self.relay_group)
                self.create_service(ValidatePath, 'follower_validate', self.follower_validation_relay,
                                    callback_group=self.relay_group)
                self.create_timer(.01, self.tick)
                fixtures.append(self)
                planner_parameters = ground_input['parameters'] if ground_input else parameters
                self.launch('gbplanner_local', 'local_planner_node', planner_parameters, ['planner:=native_planner'])
                self.launch('gbplanner_local', 'control_node.py',
                            {'request_timeout': 5., 'execution_timeout': 15., 'planner_trigger_lead_time': 0.,
                             'robot_type': 'kGround' if ground_input else 'kAerial'},
                            ['validate_path:=control_validate'])
                self.launch('gbplanner_execution', 'path_follower_node.py',
                            {'speed': 1., 'yaw_rate': 1., 'robot_type': 'kGround' if ground_input else 'kAerial'},
                            ['validate_path:=follower_validate'])
                self.until(lambda:self.maps.get_subscription_count() > 0 and
                           self.odometry.get_subscription_count() >= 3 and
                           self.native_planner.service_is_ready() and self.native_validator.service_is_ready(), 'discovery')
                self.maps.publish(ground_input['cloud'] if ground_input else make_map(blocked))
                self.until(lambda:self.status('map_status')['known_cells'] > 0 and
                           self.status('control_state')['known_cells'] > 0 and
                           self.status('execution_state')['output_ready'], 'native readiness')
                self.until(self.planner_has_odometry, 'native planner accepted actual odometry')

            def launch(self, package, executable, values, remappings=()):
                if executable == 'path_follower_node.py':
                    command = [sys.executable, str(Path(__file__).resolve()),
                               '--follower-goal-log', str(self.goal_log)]
                else:
                    command = ['ros2', 'run', package, executable]
                command += ['--ros-args', '-r', '__ns:='+self.fixture_namespace]
                for rule in remappings:
                    command += ['-r', rule]
                for key, value in values.items():
                    command += ['-p', key+':='+json.dumps(value)]
                log_path = output.parent/f'{self.label}_{executable}.log'
                with log_path.open('w') as log:
                    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                               start_new_session=True, env=dict(os.environ))
                self.processes.append((process, log_path))
                summary['launches'].append(command)

            def until(self, predicate, label, seconds=15.):
                limit = min(deadline, time.monotonic()+seconds)
                while not predicate():
                    for process, log in self.processes:
                        if process.poll() is not None:
                            raise RuntimeError(f'{self.label}: exit {process.returncode}; {log}')
                    if time.monotonic() >= limit:
                        raise TimeoutError(self.label+': '+label)
                    rclpy.spin_once(self, timeout_sec=.01)

            def call(self, name, request, kind=Trigger):
                if name not in self.service_clients:
                    self.service_clients[name] = self.create_client(kind, name)
                client = self.service_clients[name]
                self.until(client.service_is_ready, name+' discovery')
                future = client.call_async(request)
                self.until(future.done, name+' response')
                return future.result()

            def status(self, name='control_state'):
                return json.loads(self.call(name, Trigger.Request()).message)

            def planner_has_odometry(self):
                response = self.call('sensor_gain', Trigger.Request())
                data = json.loads(response.message)
                self.planner_readiness = {'success': response.success, 'data': data}
                expected = list(self.position[:3])
                if self.ground_input:
                    parameters = self.ground_input['parameters']
                    expected[2] += parameters['ground.max_ground_height']-parameters['ground.robot_height']
                return response.success and math.dist(data.get('root', [1.e9]*4)[:3], expected)<1.e-8

            async def drift(self):
                distance = .005 if self.ground_input else .2
                self.position = (self.position[0], self.position[1]+distance, self.position[2], self.position[3])
                wait = Future()
                self.waits.append((time.monotonic()+.15, wait))
                await wait  # Keep publishing fresh odometry before releasing the native response.

            async def plan_relay(self, request, response):
                result = await self.native_planner.call_async(request)
                self.plan_calls += 1
                self.planned = copy.deepcopy(list(result.path))
                if result.path:
                    await self.drift()
                return result

            async def validation_relay(self, request, response):
                result = await self.native_validator.call_async(request)
                self.validations.append({'path': [record(p) for p in request.path],
                    'first_pose_is_observation': request.first_pose_is_observation,
                    'valid': result.valid, 'reason': result.reason, 'map_revision': result.map_revision})
                if result.valid and len(self.validations) <= self.validation_moves:
                    await self.drift()
                return result

            async def follower_validation_relay(self, request, response):
                observed = list(self.position)
                result = await self.native_validator.call_async(request)
                entry = {'path': [record(p) for p in request.path],
                    'observed_position': observed,
                    'first_pose_is_observation': request.first_pose_is_observation,
                    'valid': result.valid, 'reason': result.reason, 'map_revision': result.map_revision}
                if self.ground_input and not self.follower_validations:
                    # Same actual initial request, with only its observation
                    # marker cleared: the strict fixed-path slope must remain.
                    strict = copy.deepcopy(request)
                    strict.first_pose_is_observation = False
                    ordinary = await self.native_validator.call_async(strict)
                    entry['unmarked'] = {'valid': ordinary.valid, 'reason': ordinary.reason,
                                         'map_revision': ordinary.map_revision}
                self.follower_validations.append(entry)
                return result

            def consume(self, message):
                points, times = decode(message)
                self.trajectories.append({'points': points, 'times': times})
                self.command = (points, times, time.monotonic(), self.position)

            def receive_control(self, message):
                self.control = json.loads(message.data)
                self.control_history.append(self.control)

            def tick(self):
                if self.command:
                    points, times, start, initial = self.command
                    self.position = sample(points, times, time.monotonic()-start, initial)
                message = Odometry()
                message.header.frame_id, message.child_frame_id = 'map', 'base_link'
                message.header.stamp = self.get_clock().now().to_msg()
                message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = self.position[:3]
                message.pose.pose.orientation.z, message.pose.pose.orientation.w = math.sin(self.position[3]/2), math.cos(self.position[3]/2)
                self.odometry.publish(message)
                for at, future in list(self.waits):
                    if time.monotonic() >= at:
                        self.waits.remove((at, future))
                        if not future.done():
                            future.set_result(None)

            def trigger(self, preview=False):
                request = PciTrigger.Request()
                request.bound_mode, request.not_exe_path, request.vel_max = 3, preview, 1.
                require(self.call('pci_trigger', request, PciTrigger).success, 'PCI rejected local request')
                self.until(lambda:self.control is not None and self.control['state'] in {'DONE', 'ERROR', 'PREVIEW'}, 'PCI result')
                return self.status()

            def result(self, control):
                return {'control': control, 'planned_path': [record(p) for p in self.planned],
                        'follower_goals': [json.loads(line) for line in self.goal_log.read_text().splitlines()],
                        'command_paths': [[record(p.pose) for p in path.poses] for path in self.commands if path.poses],
                        'validations': self.validations, 'follower_validations': self.follower_validations,
                        'control_history': self.control_history, 'execution_history': self.execution_history,
                        'trajectories': self.trajectories,
                        'planning_metrics': self.planning_metrics, 'planner_readiness': self.planner_readiness,
                        'plan_calls': self.plan_calls, 'final_position': list(self.position)}

            def close(self):
                if self.closed:
                    return
                self.closed = True
                exits = []
                for process, log in reversed(self.processes):
                    if process.poll() is None:
                        try:
                            os.killpg(process.pid, signal.SIGINT)
                        except ProcessLookupError:
                            pass
                        try:
                            process.wait(timeout=3.)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid, signal.SIGKILL)
                            process.wait(timeout=1.)
                    exits.append({'log': str(log), 'exit': process.returncode})
                summary.setdefault('process_exits', {})[self.label] = exits
                self.destroy_node()

        def finish(probe):
            probe.close()
            fixtures.remove(probe)

        if 'preview' in selected_cases:
            preview = Probe('preview')
            control = preview.trigger(preview=True)
            result = preview.result(control)
            require(control['state']=='PREVIEW' and result['command_paths'][-1]==result['planned_path'] and
                    not result['trajectories'], 'Preview changed the original planner route or executed motion')
            require(not result['validations'][0]['first_pose_is_observation'], 'Preview marked a planned root as observation')
            summary['preview'] = result
            finish(preview)

        if 'connected' in selected_cases:
            connected = Probe('connected', validation_moves=1)
            control = connected.trigger()
            result = connected.result(control)
            require(control['state']=='DONE' and control['completed_actions']==1 and result['trajectories'],
                    'Ordinary local route failed to execute after source drift')
            route = result['command_paths'][-1]
            require(math.dist(route[0][:3], (0., .4, 0.)) < 1.e-8 and route[1:]==result['planned_path'],
                    'Dispatch did not reconnect from the original route after the second 0.2 m drift')
            require(len(result['validations']) >= 2 and
                    all(v['valid'] and v['first_pose_is_observation'] for v in result['validations'][:2]) and
                    any(e['reason']=='reconnect_before_execution' for e in control['events']),
                    'The changed connector bypassed native validation')
            require(math.dist(result['final_position'][:3], result['planned_path'][-1][:3]) < .05,
                    'Follower did not reach the original endpoint')
            require(result['follower_validations'], 'Follower did not validate its own reconnection')
            initial_follower_validation = result['follower_validations'][0]
            require(initial_follower_validation['valid'] and
                    initial_follower_validation['first_pose_is_observation'] and
                    math.dist(initial_follower_validation['path'][0][:3],
                              initial_follower_validation['observed_position'][:3]) < 1.e-8,
                    'Initial nonmanual follower validation did not identify its inserted observed pose')
            require(len(result['follower_goals']) == 1 and
                    result['follower_goals'][0]['path'] == result['planned_path'],
                    'Controller sent its measured connector instead of the complete original route')
            require(initial_follower_validation['path'][1:] == result['planned_path'],
                    'Follower retained a second measured prefix instead of reconnecting the original route once')
            transferred = [state for state in result['control_history']
                           if state['state']=='EXECUTING' and state.get('follower_tracks_route') is True
                           and state.get('manual_path') is False]
            owners = [state for state in result['execution_history']
                      if state['state']=='EXECUTING' and state.get('active') is True
                      and state.get('manual_path') is False and state.get('validates_remaining_path') is True
                      and type(state.get('validated_revision')) is int and state['validated_revision']>=1]
            handoffs = [(controller, follower) for controller in transferred for follower in owners
                        if controller['plan_ahead']['generation']==follower['generation']]
            require(handoffs,
                    'Controller did not hand active remaining-path validation to the native follower')
            result['validation_handoff'] = {'controller': handoffs[0][0], 'follower': handoffs[0][1]}
            summary['connected'] = result
            finish(connected)

        if 'blocked' in selected_cases:
            blocked = Probe('blocked', blocked=True)
            control = blocked.trigger()
            result = blocked.result(control)
            require(control['state']=='ERROR' and control['reason'].startswith('path_invalidated:') and
                    not result['trajectories'] and not result['validations'][0]['valid'],
                    'An occupied reconnect segment reached the follower')
            original = ValidatePath.Request()
            original.header.frame_id, original.bound_mode, original.path = 'map', 3, blocked.planned
            require(blocked.call('validate_path', original, ValidatePath).valid,
                    'Obstacle also invalidated the original route, so the connector was not isolated')
            summary['blocked_connector'] = result
            finish(blocked)

        if 'bounded' in selected_cases:
            bounded = Probe('bounded', validation_moves=4)
            control = bounded.trigger()
            result = bounded.result(control)
            require(control['state']=='ERROR' and control['reason']=='pre_execution_recheck_limit' and
                    len(result['validations'])==4 and not result['trajectories'],
                    'Repeated source drift did not terminate after three reconnect validations')
            summary['bounded_rechecks'] = result
            finish(bounded)

        if 'ground_tsdf' in selected_cases:
            # Replay authentic SMB/Voxblox TSDF bytes, not a manufactured distance
            # field. The new planner generates its own local path on that map.
            import yaml
            source = workspace/'results/smb_autonomy/run4'
            archive = json.loads(source.with_suffix('.json').read_text())
            metadata = json.loads(source.with_name(source.name+'_map.json').read_text())
            map_bytes = source.with_name(source.name+'_map.bin').read_bytes()
            require(len(map_bytes)==metadata['data_bytes']==metadata['height']*metadata['row_step'],
                    'Recorded TSDF metadata and bytes differ')
            cloud = PointCloud2()
            cloud.header.frame_id = metadata['frame_id']
            # Zero stamp explicitly identifies static replay; native validation is
            # still against the exact accepted occupancy and TSDF observations.
            for key in ('width', 'height', 'point_step', 'row_step', 'is_bigendian', 'is_dense'):
                setattr(cloud, key, metadata[key])
            cloud.fields = [PointField(**field) for field in metadata['fields']]
            cloud.data = map_bytes
            config = workspace/'src/gbplanner_smb/config/exploration.yaml'
            ground_parameters = yaml.safe_load(config.read_text())['gbplanner_local']['ros__parameters']
            ground_parameters.update({'global.enable': False, 'planning.graph_mode': False,
                'planning.max_vertices': 2, 'planning.max_attempts': 10,
                'planning.edge_min': .05, 'planning.edge_max': .8,
                'planning.path_direction_penalty': 0., 'planning.path_length_penalty': 0.,
                'planning.cluster_vertices_for_gain': False, 'gain.free': 1.,
                'sampling.min': [-.1, -.1, -.1], 'sampling.max': [.7, .1, .1]})
            for axis, value in [('x', .6), ('y', 0.), ('z', 0.), ('heading', 0.)]:
                ground_parameters[f'sampling.{axis}.distribution'] = 'constant'
                ground_parameters[f'sampling.{axis}.constant'] = value
            ground_input = {'cloud': cloud, 'parameters': ground_parameters,
                            'position': archive['checks']['local_gain_exploration']['start']}
            ground = Probe('ground_tsdf', ground_input=ground_input)
            control = ground.trigger()
            result = ground.result(control)
            result['recorded_map'] = {'source': str(source), 'sha256': hashlib.sha256(map_bytes).hexdigest(),
                'recorded_stamp': metadata['stamp'], 'cells': metadata['width']*metadata['height'],
                'source_pose': ground_input['position'], 'config': str(config),
                'parameters': ground_parameters,
                'scope': 'recorded physical TSDF replay; new native local plan; 5 mm synthetic lateral drift; no dynamics'}
            summary['ground_tsdf'] = result
            require(control['state']=='DONE' and control['completed_actions']==1 and result['trajectories'],
                    'Ground TSDF route did not complete through the native follower')
            initial = result['follower_validations'][0]
            require(initial['first_pose_is_observation'] and initial['valid'] and
                    not initial['unmarked']['valid'] and initial['unmarked']['reason']=='terrain_slope_exceeded',
                    'TSDF fixture did not isolate the observed initial height seam from strict fixed geometry')
            retained, planned = initial['path'][1:], result['planned_path']
            require(len(retained)==len(planned), 'Ground follower changed retained waypoint count')
            # The native follower represents each pose as XYZ/yaw, then
            # reconstructs its quaternion for ValidatePath. Positions must
            # remain exact; only that trigonometric roundoff gets tolerance.
            xyz_exact = all(actual[:3]==expected[:3] for actual, expected in zip(retained, planned))
            quaternion_max_difference = max((abs(a-b) for actual, expected in zip(retained, planned)
                                             for a, b in zip(actual[3:], expected[3:])), default=0.)
            result['retained_pose_comparison'] = {'xyz_exact': xyz_exact,
                'quaternion_max_component_difference': quaternion_max_difference,
                'quaternion_absolute_tolerance': 1.e-12}
            require(len(result['follower_goals'])==1 and result['follower_goals'][0]['path']==result['planned_path']
                    and xyz_exact and quaternion_max_difference<=1.e-12
                    and math.dist(initial['path'][0][:3], initial['observed_position'][:3])<1.e-8,
                    'Ground follower did not have exactly one authentic observed prefix')
            require(math.dist(result['final_position'][:3], result['planned_path'][-1][:3])<.05,
                    'Ground follower missed the unchanged original endpoint')
            finish(ground)
        case_checks = {
            'preview': ['preview_preserves_native_route'],
            'connected': ['ordinary_local_reconnects_0p2m_drift', 'dispatch_reconnects_second_0p2m_drift',
                'retained_poses_and_endpoint_preserved', 'real_follower_completes',
                'follower_initial_observation_identified', 'follower_receives_original_route',
                'follower_has_one_observed_prefix', 'native_follower_owns_active_path_validation'],
            'blocked': ['occupied_connector_rejected_by_native_validator'],
            'bounded': ['repeated_reconnection_bounded'],
            'ground_tsdf': ['ground_tsdf_initial_height_seam', 'ground_fixed_geometry_stays_strict',
                'ground_follower_has_one_observed_prefix'],
        }
        summary['checks'] = {check: True for case in sorted(selected_cases) for check in case_checks[case]}
        summary['status'] = 'passed'
    except Exception as error:
        summary['error'] = f'{type(error).__name__}: {error}'
        summary['last_states'] = {p.label: p.result(p.control) for p in fixtures}
    finally:
        for probe in reversed(fixtures):
            try:
                probe.close()
            except Exception as error:
                summary.setdefault('cleanup_errors', []).append(str(error))
        if summary.get('cleanup_errors') or any(row['exit']!=0
                for exits in summary.get('process_exits', {}).values() for row in exits):
            summary['status'] = 'failed'
            summary['cleanup_error'] = 'At least one native process failed to exit cleanly'
        if initialized and rclpy.ok():
            rclpy.shutdown()
        summary['elapsed_s'] = round(time.monotonic()-started, 3)
        output.write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps(summary, ensure_ascii=False))
    return 0 if summary['status']=='passed' else 1


if __name__=='__main__':
    raise SystemExit(main())
