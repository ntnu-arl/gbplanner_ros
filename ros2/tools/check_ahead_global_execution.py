#!/usr/bin/env python3
"""Native PlanAhead -> PCI -> PathFollower global handoff and return.

Reuses the unmodified frontier snapshot produced by check_frontier_switch.py.
Every planning response and action result comes from native project nodes.
A test-only timed-trajectory consumer publishes synthetic measured odometry;
this is a kinematic integration check, not a dynamics or hardware experiment.
"""
import argparse
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
import traceback


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def xyz(pose):
    return [pose.position.x, pose.position.y, pose.position.z]


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--local-resume', action='store_true',
                        help='Exercise a resumed local Action after measured global arrival before homing')
    parser.add_argument('--snapshot', type=Path, default=workspace/'results/frontier_switch/native_frontier.gbmem')
    parser.add_argument('--source-summary', type=Path, default=workspace/'results/frontier_switch/summary.json')
    parser.add_argument('--timeout', type=float, default=90.)
    args = parser.parse_args()
    require(math.isfinite(args.timeout) and args.timeout > 0., 'Timeout must be positive')
    output = (args.output or workspace/('results/ahead_global_resume/summary.json' if
              args.local_resume else 'results/ahead_global_execution/summary.json')).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100+os.getpid()%80))
    started, initialized, probe = time.monotonic(), False, None
    deadline = started+args.timeout
    summary = {'status': 'failed', 'scope': __doc__, 'local_resume_scenario': args.local_resume,
               'domain': os.environ['ROS_DOMAIN_ID'], 'launches': [], 'checks': {}, 'observations': {}}
    try:
        snapshot, source_summary = args.snapshot.resolve(), args.source_summary.resolve()
        provenance = json.loads(source_summary.read_text())
        require(provenance['status']=='passed' and provenance['seed']['metrics']['frontier_exists'],
                'Source check does not document native frontier generation')
        require(Path(provenance['seed']['snapshot']).name==snapshot.name and snapshot.is_file(),
                'Native snapshot differs from its source record')
        snapshot_hash = hashlib.sha256(snapshot.read_bytes()).hexdigest()
        summary['snapshot'] = {'path': str(snapshot), 'sha256': snapshot_hash,
            'source_summary': str(source_summary),
            'source_summary_sha256': hashlib.sha256(source_summary.read_bytes()).hexdigest(),
            'generator': 'check_frontier_switch.py native local_planner_node + gbplanner/save_graph',
            'reuse': 'read existing native graph without modifying its bytes'}

        import rclpy
        from ament_index_python.packages import get_package_prefix
        from nav_msgs.msg import Odometry, Path as RosPath
        from planner_msgs.srv import PciTrigger, PlannerSrv, PlannerStringTrigger
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2, PointField
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from trajectory_msgs.msg import MultiDOFJointTrajectory
        sys.dont_write_bytecode = True
        sys.path.insert(0, str(Path(get_package_prefix('gbplanner_execution'))/'lib/gbplanner_execution'))
        from execution_demo import decode, sample
        if args.local_resume:
            files = [Path(__file__).resolve(), workspace/'src/gbplanner_local/src/local_planner_node.cpp',
                     workspace/'src/gbplanner_local/scripts/control_node.py',
                     workspace/'src/gbplanner_execution/scripts/path_follower_node.py',
                     workspace/'src/gbplanner_local/include/gbplanner_local/global_repositioning.hpp',
                     Path(get_package_prefix('gbplanner_local'))/'lib/gbplanner_local/local_planner_node',
                     Path(get_package_prefix('gbplanner_local'))/'lib/gbplanner_local/control_node.py',
                     Path(get_package_prefix('gbplanner_execution'))/'lib/gbplanner_execution/path_follower_node.py']
            summary['provenance'] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}

        rclpy.init(args=[])
        initialized = True
        durable = QoSProfile(depth=30, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        origin, dimensions, resolution = (-4., -5., -3.), (36, 20, 12), .5
        parameters = {
            'map.resolution': resolution, 'map.origin': list(origin), 'map.dimensions': list(dimensions),
            'global.enable': True, 'global.auto_home': True, 'global.background_enable': False,
            'sampling.min': [-1., -1., -.2], 'sampling.max': [1., 1., .2], 'sampling.seed': 42,
            'planning.max_vertices': 32, 'planning.max_attempts': 1200,
            'planning.edge_min': .35, 'planning.edge_max': 1., 'planning.neighbor_max': 1.5,
            'planning.path_safety_enhance_enable': False,
            'sensor.range': 2., 'sensor.horizontal_fov': 2.*math.pi, 'sensor.vertical_fov': .3,
            'sensor.horizontal_resolution': .3, 'sensor.vertical_resolution': .15,
            'gain.free': .05, 'gain.unknown': 1., 'gain.frontier_threshold': .05,
        }
        summary['parameters'] = parameters
        cloud = PointCloud2()
        cloud.header.frame_id, cloud.height, cloud.width = 'map', 1, math.prod(dimensions)
        cloud.point_step, cloud.row_step, cloud.is_dense = 16, cloud.width*16, True
        cloud.fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                        for i, name in enumerate(('x', 'y', 'z'))]
        cloud.fields.append(PointField(name='occupancy', offset=12, datatype=PointField.INT8, count=1))
        data, offset = bytearray(cloud.row_step), 0
        for z in range(dimensions[2]):
            for y in range(dimensions[1]):
                for x in range(dimensions[0]):
                    point = [origin[i]+(value+.5)*resolution for i, value in enumerate((x, y, z))]
                    struct.pack_into('<fffbxxx', data, offset, *point, -1 if point[0]>=6. else 0)
                    offset += 16
        cloud.data = bytes(data)

        class Probe(Node):
            def __init__(self):
                self.fixture_namespace = f'/ahead_global_execution_{os.getpid()}'
                super().__init__('probe', namespace=self.fixture_namespace)
                self.processes, self.service_clients, self.timeline = [], {}, []
                self.local_metrics, self.global_metrics, self.global_paths = [], [], []
                self.control_events, self.execution_events, self.trajectories = [], [], []
                self.position, self.command, self.closed = (0., 0., 0., 0.), None, False
                self.maps = self.create_publisher(PointCloud2, 'voxel_map', durable)
                self.odometry = self.create_publisher(Odometry, 'odometry', 10)
                for topic, rows in (('planning_metrics', self.local_metrics),
                                    ('global_metrics', self.global_metrics),
                                    ('control_status', self.control_events),
                                    ('execution_status', self.execution_events)):
                    self.create_subscription(String, topic,
                        lambda m, t=topic, r=rows:self.observe(t, r, json.loads(m.data)), durable)
                self.create_subscription(RosPath, 'global_path', self.global_path, durable)
                self.create_subscription(MultiDOFJointTrajectory, 'command/trajectory', self.consume, 10)
                self.timer = self.create_timer(.02, self.tick)

            def observe(self, topic, rows, value):
                rows.append(value)
                self.timeline.append({'received_s': time.monotonic()-started, 'topic': topic, 'data': value})

            def global_path(self, message):
                self.observe('global_path', self.global_paths, [xyz(p.pose) for p in message.poses])

            def launch(self, package, executable, settings):
                command = ['ros2', 'run', package, executable, '--ros-args', '-r', '__ns:='+self.fixture_namespace]
                for key, value in settings.items():
                    command += ['-p', key+':='+json.dumps(value)]
                log_path = output.parent/(executable+'.log')
                with log_path.open('w') as log:
                    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                               start_new_session=True, env=dict(os.environ))
                self.processes.append((process, log_path))
                summary['launches'].append(command)

            def until(self, predicate, label, seconds=20.):
                limit = min(deadline, time.monotonic()+seconds)
                while not predicate():
                    for process, log in self.processes:
                        if process.poll() is not None:
                            raise RuntimeError(f'Node exited {process.returncode}; see {log}')
                    if time.monotonic()>=limit:
                        raise TimeoutError(label)
                    rclpy.spin_once(self, timeout_sec=.02)

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

            def wait_control(self, state, mode, completed, seconds=25.):
                result = {}
                def reached():
                    result.update(self.status())
                    require(result['state'] not in {'ERROR', 'STOPPED'}, 'Unexpected PCI terminal state: '+str(result))
                    return result['state']==state and result['mode']==mode and result['completed_actions']==completed
                self.until(reached, f'{mode}: {state}, completed={completed}', seconds)
                return result

            def consume(self, message):
                points, times = decode(message)
                self.observe('command/trajectory', self.trajectories,
                    {'points': points, 'times': times, 'initial': list(self.position)})
                self.command = (points, times, time.monotonic(), self.position)

            def tick(self):
                if self.command:
                    points, times, begin, initial = self.command
                    self.position = sample(points, times, time.monotonic()-begin, initial)
                message = Odometry()
                message.header.frame_id, message.child_frame_id = 'map', 'base_link'
                message.header.stamp = self.get_clock().now().to_msg()
                message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = self.position[:3]
                message.pose.pose.orientation.z, message.pose.pose.orientation.w = math.sin(self.position[3]/2), math.cos(self.position[3]/2)
                self.odometry.publish(message)

            def publish_map(self):
                before = self.status('map_status')['map_revision']
                # Supported static fixture: zero stamp, fresh observation revision
                # per publication. No graph, plan, action or feedback is fabricated.
                self.maps.publish(cloud)
                self.until(lambda:self.status('map_status')['map_revision']==before+1, 'map accepted')

            def initialize(self):
                self.until(lambda:self.maps.get_subscription_count()>0 and self.odometry.get_subscription_count()>=3,
                           'planner, PCI and follower subscriptions')
                self.publish_map()
                self.until(lambda:self.status('global_status')['vertices']>0 and
                           self.status('sensor_gain')['root']==[0., 0., 0., 0.], 'native home and odometry')
                load = PlannerStringTrigger.Request()
                load.message = str(snapshot)
                require(self.call('gbplanner/load_graph', load, PlannerStringTrigger).success, 'Native graph load failed')
                self.until(lambda:self.status()['known_cells']>0 and self.status('execution_state')['command_subscribers']>0,
                           'PCI and follower ready')

            def prime(self, expected):
                request = PlannerSrv.Request()
                request.header.frame_id, request.bound_mode = 'map', 3
                request.root_pose.orientation.w = 0.
                before = len(self.local_metrics)
                response = self.call('planner', request, PlannerSrv)
                self.until(lambda:len(self.local_metrics)>before, 'prime native metrics')
                metric = self.local_metrics[-1]
                require(response.status==response.K_FORWARD and len(response.path)>1 and
                        metric['gain_evaluated'] and metric['gain']>0. and not metric['frontier_exists'] and
                        metric['auto_global']['count_after']==expected and not metric['auto_global']['triggered'],
                        'Prime fixture did not produce the expected evaluated local round')
                return {'metrics': metric, 'path': [xyz(p) for p in response.path]}

            def close(self):
                if self.closed:
                    return
                self.closed = True
                self.timer.cancel()
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
                summary['process_exits'] = exits
                self.destroy_node()

        probe = Probe()
        probe.launch('gbplanner_local', 'local_planner_node', parameters)
        exploration_actions = 3 if args.local_resume else 2
        probe.launch('gbplanner_local', 'control_node.py',
                     {'planner_trigger_lead_time': 60., 'max_cycles': exploration_actions})
        probe.launch('gbplanner_execution', 'path_follower_node.py', {'speed': .7, 'yaw_rate': 1.})
        probe.initialize()
        initial = probe.status('global_status')
        require(initial['frontiers']>0 and not initial['repositioning_active'], 'Seed must contain inactive native frontiers')
        home = list(probe.position[:3])
        observations = summary['observations']
        observations['initial'] = initial
        observations['prime'] = [probe.prime(1), probe.prime(2)]
        trigger = PciTrigger.Request()
        trigger.bound_mode, trigger.set_auto = 3, True
        require(probe.call('pci_trigger', trigger, PciTrigger).success, 'Automatic PCI trigger refused')
        local_running = probe.wait_control('EXECUTING', 'local', 0)
        require(probe.status('global_status')['low_gain_iterations']==3 and len(probe.local_metrics)==3,
                'PCI local plan did not supply the third native evaluation')
        observations['local_running'] = local_running
        probe.publish_map()
        probe.until(lambda:any(e['reason']=='plan_ahead_global_required' for e in probe.control_events), 'fourth-round PlanAhead latch')
        latched = next(e for e in probe.control_events if e['reason']=='plan_ahead_global_required')
        require(latched['state']=='EXECUTING' and latched['mode']=='local' and latched['completed_actions']==0
                and latched['plan_ahead']['global_required'] and latched['plan_ahead']['continuations']==0
                and not probe.global_metrics and probe.status('global_status')['low_gain_iterations']==0,
                'Ahead result did not defer global planning until the current action finishes')
        observations['ahead_latched'] = latched
        waiting = probe.wait_control('WAITING_OBSERVATION', 'local', 1)
        local_endpoint = list(probe.position[:3])
        before_global = probe.status('execution_state')
        require(before_global['succeeded']==1 and not probe.global_metrics and waiting['plan_ahead']['global_required'],
                'Global planning began before local action success and endpoint observation')
        observations['local_completed'] = {'control': waiting, 'execution': before_global, 'endpoint': local_endpoint}
        probe.publish_map()
        global_running = probe.wait_control('EXECUTING', 'global', 1)
        probe.until(lambda:len(probe.global_metrics)>=1 and any(probe.global_paths), 'native global path')
        global_metric, global_route = probe.global_metrics[0], next(p for p in probe.global_paths if p)
        retained = probe.status('global_status')
        require(global_metric['success'] and global_metric['mode']=='frontier' and global_metric['target']>0
                and len(global_route)>2 and math.dist(global_route[0], local_endpoint)<.05,
                'Global handoff did not produce a complete frontier route from measured endpoint odometry')
        require(retained['repositioning_active'] and retained['repositioning_target']==global_metric['target'],
                'The native global service did not receive an automatic handoff that retains its selected target')
        observations['global_running'] = {'control': global_running, 'global_status': retained,
            'metrics': global_metric, 'route': global_route,
            'target_distance_at_handoff_m': math.dist(local_endpoint, global_route[-1])}
        global_waiting = probe.wait_control('WAITING_OBSERVATION', 'global', 2, seconds=35.)
        execution = probe.status('execution_state')
        require(execution['succeeded']==2 and math.dist(probe.position[:3], global_route[-1])<.05,
                'Native follower did not complete the full global route')
        require(len(probe.trajectories)==2 and math.dist(probe.trajectories[1]['points'][-1][:3], global_route[-1])<1.e-8,
                'The global route was truncated or replaced before follower execution')
        observations['global_completed'] = {'control': global_waiting, 'execution': execution,
                                            'measured_endpoint': list(probe.position[:3])}
        if args.local_resume:
            global_endpoint = list(probe.position[:3])
            before_resume_metrics = len(probe.local_metrics)
            before_resume_observation = global_waiting['observation_revision']
            probe.publish_map()
            resumed = probe.wait_control('EXECUTING', 'local', 2)
            probe.until(lambda:len(probe.local_metrics)>before_resume_metrics and len(probe.trajectories)==3,
                        'native resumed local graph and third trajectory')
            metric = probe.local_metrics[-1]
            after_arrival = probe.status('global_status')
            resume_request = next(e for e in probe.control_events if e['reason']=='local_requested'
                                  and e['completed_actions']==2)
            require(resume_request['observation_revision']>before_resume_observation and
                    resume_request['mode']=='local' and not resume_request['plan_ahead']['global_required'],
                    'Local resume preceded the new endpoint observation or retained a deferred handoff')
            require(metric['success'] and metric['reason']=='ok' and metric['gain_evaluated'] and
                    metric['vertices']>1 and not metric['auto_global']['retained_active'] and
                    metric['auto_global']['retained_target']==-1 and
                    math.dist(metric['planning_root'][:3], global_endpoint)<.05,
                    'Arrival did not produce a new native local graph from measured global endpoint')
            require(not after_arrival['repositioning_active'] and after_arrival['repositioning_target']==-1,
                    'Global target remained active after measured arrival and local resume')
            resume_trajectory = probe.trajectories[2]
            require(math.dist(resume_trajectory['initial'][:3], global_endpoint)<.05 and
                    math.dist(resume_trajectory['points'][0][:3], global_endpoint)<.05,
                    'Resumed local Action did not start from measured global endpoint')
            observations['local_resumed'] = {'control': resumed, 'request': resume_request,
                'global_status': after_arrival, 'metrics': metric,
                'measured_start': global_endpoint, 'trajectory_index': 2}
            resumed_waiting = probe.wait_control('WAITING_OBSERVATION', 'local', 3, seconds=35.)
            resumed_execution = probe.status('execution_state')
            require(resumed_execution['accepted']==3 and resumed_execution['succeeded']==3 and
                    resumed_execution['aborted']==0 and resumed_execution['cancelled']==0 and
                    math.dist(probe.position[:3], resume_trajectory['points'][-1][:3])<.05,
                    'Third native local Action did not complete its generated route')
            require(not resumed_waiting['plan_ahead']['pending'] and
                    not resumed_waiting['plan_ahead']['global_required'] and
                    resumed_waiting['plan_ahead']['continuations']==0,
                    'Cycle limit failed to suppress another speculative continuation')
            observations['resumed_local_completed'] = {'control': resumed_waiting,
                'execution': resumed_execution, 'measured_endpoint': list(probe.position[:3])}
        # After the selected number of exploration actions, a fresh endpoint
        # map starts native homing; the new scenario adds local resume first.
        probe.publish_map()
        probe.wait_control('EXECUTING', 'homing', exploration_actions)
        final = probe.wait_control('DONE', 'homing', exploration_actions+1, seconds=35.)
        execution = probe.status('execution_state')
        require(final['reason']=='returned_home' and not final['auto'] and math.dist(probe.position[:3], home)<.05,
                'Automatic task did not terminate at its original home')
        require(execution['accepted']==exploration_actions+1 and execution['succeeded']==exploration_actions+1 and execution['aborted']==0
                and execution['cancelled']==0 and execution['rejected']==0 and execution['continuations']==0
                and execution['hold_commands']==0 and len(probe.trajectories)==exploration_actions+1,
                'Unexpected native Action count or outcome')
        requested_modes = ['local_requested', 'global_requested'] + (
            ['local_requested'] if args.local_resume else []) + ['homing_requested']
        require([e['reason'] for e in final['events'] if e['reason'] in
                {'local_requested', 'global_requested', 'homing_requested'}]==
                requested_modes, 'PCI repeated or reordered planner tasks')
        global_request = next(e for e in probe.control_events if e['reason']=='global_requested')
        require(global_request['completed_actions']==1 and not global_request['plan_ahead']['global_required'],
                'Global handoff did not consume the latch after the first completed action')
        require(len(probe.local_metrics)==(4 if args.local_resume else 3) and len(probe.global_metrics)==2 and
                [m['mode'] for m in probe.global_metrics]==['frontier', 'homing'],
                'Extra native local/global planning occurred during the handoff')
        require(hashlib.sha256(snapshot.read_bytes()).hexdigest()==snapshot_hash, 'Source snapshot was modified')
        observations['final'] = {'control': final, 'execution': execution, 'measured_endpoint': list(probe.position[:3])}
        summary['checks'] = dict.fromkeys(['native_snapshot_reused_unchanged', 'two_native_prime_rounds',
            'pci_local_is_third_round', 'pci_ahead_is_fourth_round', 'ahead_does_not_splice_current_action',
            'local_action_success_precedes_global_request', 'fresh_endpoint_map_required',
            'global_route_starts_from_measured_odometry', 'automatic_handoff_retains_native_target',
            'native_follower_completes_full_global_route', 'cycle_budget_then_native_homing',
            ('four_actions_succeed_without_cancellation' if args.local_resume else
             'three_actions_succeed_without_cancellation'), 'no_extra_planner_requests'], True)
        if args.local_resume:
            summary['checks'].update(dict.fromkeys(['fresh_global_arrival_observation_precedes_local_resume',
                'measured_arrival_clears_retained_target', 'new_native_local_graph_after_global',
                'third_local_action_starts_from_measured_global_endpoint',
                'third_local_action_completes_before_homing'], True))
        summary['status'] = 'passed'
    except Exception as error:
        summary['error'], summary['traceback'] = str(error), traceback.format_exc()
    finally:
        if probe is not None:
            summary['timeline'] = probe.timeline
            summary['trajectories'] = probe.trajectories
            try:
                probe.close()
                if any(row['exit']!=0 for row in summary.get('process_exits', [])):
                    summary['status'], summary['cleanup_error'] = 'failed', 'Native node exited nonzero'
            except Exception as error:
                summary['status'], summary['cleanup_error'] = 'failed', str(error)
        if initialized and rclpy.ok():
            rclpy.shutdown()
        summary['elapsed_s'] = time.monotonic()-started
        output.write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'status': summary['status'], 'output': str(output),
                      'elapsed_s': summary['elapsed_s'], 'error': summary.get('error')}))
    return 0 if summary['status']=='passed' else 1


if __name__=='__main__':
    raise SystemExit(main())
