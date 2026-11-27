#!/usr/bin/env python3
"""Check evaluated PlanAhead rounds and the native PCI's deferred global handoff.

Real planner, PCI and follower processes consume synthetic occupancy/odometry.
The trajectory consumer interpolates native commands; it is not robot dynamics.
"""
import argparse
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


def xyz(poses):
    return [[p.position.x, p.position.y, p.position.z] for p in poses]


def history(path):
    """Read native GBP2MEM1 history, without modifying or manufacturing a graph."""
    data = path.read_bytes()
    require(data[:8] == b'GBP2MEM1', 'Unexpected native snapshot format')
    cursor = 8

    def take(fmt):
        nonlocal cursor
        result = struct.unpack_from('<'+fmt, data, cursor)
        cursor += struct.calcsize('<'+fmt)
        return result[0] if len(result) == 1 else list(result)

    frame_size = take('I')
    cursor += frame_size+1+8  # frame, ground flag, ground offset
    graph_size = take('I')
    cursor += graph_size
    vertices = take('I')
    cursor += vertices*struct.calcsize('<i4diidddiBBidi')
    edges = take('I')
    cursor += edges*8
    visits = [take('4d') for _ in range(take('I'))]
    odometry = take('i')
    suspended = take('I')
    cursor += suspended*16
    frontiers = take('I')
    cursor += frontiers*4
    require(data[cursor:cursor+4] == b'E1M1', 'Visited marker missing from native snapshot')
    cursor += 4
    marker = take('4d')
    require(cursor == len(data), 'Unexpected trailing native snapshot data')
    return {'visited': visits, 'last_odometry': odometry, 'visited_marker': marker}


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=workspace/'results/ahead_frontier_switch/summary.json')
    parser.add_argument('--timeout', type=float, default=120.)
    args = parser.parse_args()
    require(math.isfinite(args.timeout) and args.timeout > 0., 'Timeout must be positive')
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100+os.getpid()%80))
    started, fixtures, initialized = time.monotonic(), [], False
    deadline = started+args.timeout
    summary = {'status': 'failed', 'scope': __doc__, 'launches': [], 'checks': {}}
    try:
        import rclpy
        from ament_index_python.packages import get_package_prefix
        from nav_msgs.msg import Odometry, Path as RosPath
        from planner_msgs.srv import PlanAhead, PlannerSrv, PlannerStringTrigger, PciTrigger
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2, PointField
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from trajectory_msgs.msg import MultiDOFJointTrajectory
        sys.path.insert(0, str(Path(get_package_prefix('gbplanner_execution'))/'lib/gbplanner_execution'))
        from execution_demo import decode, sample

        rclpy.init(args=[])
        initialized = True
        durable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        origin, dimensions, resolution = (-6., -5., -3.), (40, 20, 12), .5
        common = {
            'map.resolution': resolution, 'map.origin': list(origin), 'map.dimensions': list(dimensions),
            'global.enable': True, 'global.auto_home': True, 'global.background_enable': False,
            'planning.go_home_if_fully_explored': True,
            'sampling.min': [-1., -1., -.2], 'sampling.max': [1., 1., .2], 'sampling.seed': 42,
            'planning.max_vertices': 32, 'planning.max_attempts': 1200,
            'planning.edge_min': .35, 'planning.edge_max': 1., 'planning.neighbor_max': 1.5,
            'planning.path_safety_enhance_enable': False,
            'sensor.range': 2., 'sensor.horizontal_fov': 2.*math.pi, 'sensor.vertical_fov': .3,
            'sensor.horizontal_resolution': .3, 'sensor.vertical_resolution': .15,
            'gain.free': .05, 'gain.unknown': 1., 'gain.frontier_threshold': .05,
        }
        cloud = PointCloud2()
        cloud.header.frame_id = 'map'
        cloud.height, cloud.width, cloud.point_step = 1, math.prod(dimensions), 16
        cloud.row_step, cloud.is_dense = cloud.width*16, True
        cloud.fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                        for i, name in enumerate(('x', 'y', 'z'))]
        cloud.fields.append(PointField(name='occupancy', offset=12, datatype=PointField.INT8, count=1))
        data, offset = bytearray(cloud.row_step), 0
        for z in range(dimensions[2]):
            for y in range(dimensions[1]):
                for x in range(dimensions[0]):
                    point = [origin[i]+(value+.5)*resolution for i, value in enumerate((x, y, z))]
                    struct.pack_into('<fffbxxx', data, offset, *point, 0)
                    offset += 16
        cloud.data = bytes(data)

        class Probe(Node):
            def __init__(self, label, overrides=None, controller=False):
                namespace = f'/ahead_switch_{os.getpid()}_{label}'
                super().__init__('probe', namespace=namespace)
                self.label, self.fixture_namespace, self.processes, self.service_clients = label, namespace, [], {}
                self.position, self.command = (-2., 0., 0., 0.), None
                self.local_metrics, self.global_metrics, self.global_paths = [], [], []
                self.trajectories, self.control_events = [], []
                self.maps = self.create_publisher(PointCloud2, 'voxel_map', durable)
                self.odometry = self.create_publisher(Odometry, 'odometry', 10)
                self.create_subscription(String, 'planning_metrics', lambda m:self.local_metrics.append(self.decode_json('planning_metrics', m.data)), durable)
                self.create_subscription(String, 'global_metrics', lambda m:self.global_metrics.append(self.decode_json('global_metrics', m.data)), durable)
                self.create_subscription(RosPath, 'global_path', self.global_paths.append, durable)
                self.create_subscription(String, 'control_status', lambda m:self.control_events.append(self.decode_json('control_status', m.data)), durable)
                self.create_subscription(MultiDOFJointTrajectory, 'command/trajectory', self.consume, 10)
                self.create_timer(.02, self.tick)
                fixtures.append(self)
                parameters = dict(common)
                parameters.update(overrides or {})
                self.launch('gbplanner_local', 'local_planner_node', parameters)
                if controller:
                    self.launch('gbplanner_local', 'control_node.py', {'planner_trigger_lead_time': 60., 'max_cycles': 20})
                    self.launch('gbplanner_execution', 'path_follower_node.py', {'speed': .4, 'yaw_rate': 1.})

            def launch(self, package, executable, parameters):
                command = ['ros2', 'run', package, executable, '--ros-args', '-r', '__ns:='+self.fixture_namespace]
                for key, value in parameters.items():
                    command += ['-p', key+':='+json.dumps(value)]
                log_path = output.parent/f'{self.label}_{executable}.log'
                with log_path.open('w') as log:
                    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                               start_new_session=True, env=dict(os.environ))
                self.processes.append((process, log_path))
                summary['launches'].append(command)

            def until(self, predicate, label, seconds=25.):
                limit = min(deadline, time.monotonic()+seconds)
                while not predicate():
                    for process, log in self.processes:
                        if process.poll() is not None:
                            raise RuntimeError(f'{self.label} exited {process.returncode}; see {log}')
                    if time.monotonic() >= limit:
                        raise TimeoutError(self.label+': '+label)
                    rclpy.spin_once(self, timeout_sec=.02)

            def call(self, name, request, kind=Trigger):
                if name not in self.service_clients:
                    self.service_clients[name] = self.create_client(kind, name)
                client = self.service_clients[name]
                self.until(client.service_is_ready, name+' discovery')
                future = client.call_async(request)
                self.until(future.done, name+' response')
                return future.result()

            def decode_json(self, source, raw):
                try:
                    return json.loads(raw)
                except json.JSONDecodeError as error:
                    path = output.parent/f'{self.label}_{source.replace("/", "_")}_invalid.txt'
                    path.write_text(raw)
                    summary['invalid_json'] = {'source': source, 'raw_path': str(path), 'offset': error.pos,
                                               'context': raw[max(0, error.pos-80):error.pos+80]}
                    raise

            def status(self, name='global_status'):
                return self.decode_json(name, self.call(name, Trigger.Request()).message)

            def consume(self, message):
                points, times = decode(message)
                self.trajectories.append({'points': points, 'times': times})
                self.command = (points, times, time.monotonic(), self.position)

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

            def publish_map(self):
                self.until(lambda:self.maps.get_subscription_count()>0 and self.odometry.get_subscription_count()>0,
                           'map/odometry discovery')
                before = self.status('map_status')['map_revision']
                # A zero timestamp is the supported static-map fixture mode;
                # each publication still creates a new observation revision.
                self.maps.publish(cloud)
                self.tick()
                self.until(lambda:self.status('map_status')['map_revision']==before+1, 'map acceptance')
                self.until(lambda:self.status()['vertices']>0, 'home initialization')

            def ahead(self, root=(3., 0., 0.)):
                request = PlanAhead.Request()
                request.header.frame_id, request.bound_mode, request.arrival_time = 'map', 3, 1.
                request.root_pose.position.x, request.root_pose.position.y, request.root_pose.position.z = root
                request.root_pose.orientation.w = 1.
                response = self.call('planner_ahead', request, PlanAhead)
                return {'success': response.success, 'reason': response.reason,
                        'global_required': response.global_required, 'homing_required': response.homing_required,
                        'path': xyz(response.path), 'count': self.status()['low_gain_iterations']}

            def snapshot_history(self, label):
                request = PlannerStringTrigger.Request()
                request.message = str(output.parent/f'{self.label}_{label}.gbmem')
                require(self.call('gbplanner/save_graph', request, PlannerStringTrigger).success, 'Native graph save failed')
                return history(Path(request.message))

            def close(self):
                for process, _ in reversed(self.processes):
                    if process.poll() is not None:
                        continue
                    try:
                        os.killpg(process.pid, signal.SIGINT)
                    except ProcessLookupError:
                        pass
                    try:
                        process.wait(timeout=3.)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=1.)
                self.destroy_node()

        # Evaluated but non-executable results must still count. All four calls
        # are real PlanAhead evaluations, with no continuation service involved.
        cases = [('no_gain', {'gain.free': 0.}, 'no_gain'),
                 ('short_path', {'sampling.min': [-.1, -.1, -.05], 'sampling.max': [.1, .1, .05],
                                 'planning.max_vertices': 2, 'planning.edge_min': .01,
                                 'planning.edge_max': .2, 'planning.neighbor_max': .3}, 'best_path_too_short')]
        for label, overrides, reason in cases:
            probe = Probe(label, overrides)
            probe.publish_map()
            before = probe.snapshot_history('before')
            rounds = [probe.ahead() for _ in range(4)]
            require([row['count'] for row in rounds] == [1, 2, 3, 0], label+' did not count every evaluation')
            require(all(not row['success'] and row['reason']==reason and not row['global_required']
                        for row in rounds[:3]), label+' fixture did not reach its expected evaluated result')
            require(rounds[-1]['global_required'] and rounds[-1]['reason']=='plan_ahead_global_required'
                    and not rounds[-1]['path'] and not rounds[-1]['homing_required'], label+' fourth result is wrong')
            after = probe.snapshot_history('after')
            require(before==after, 'Future root changed actual visited/odometry history')
            require(probe.status()['vertices']==1 and not probe.global_metrics,
                    'Discarded local candidate inserted a reference route or ran global planning prematurely')
            summary[label] = {'rounds': rounds, 'history_before': before, 'history_after': after}
            probe.close()
            fixtures.remove(probe)

        probe = Probe('automatic', controller=True)
        probe.publish_map()
        home = list(probe.position[:3])
        # Establish a measured path away from home for the final native return.
        for step in range(1, 9):
            probe.position = (-2.+.5*step, 0., 0., 0.)
            probe.tick()
            probe.until(lambda:math.dist(probe.status('sensor_gain')['root'][:3], probe.position[:3])<1e-9,
                        'measured odometry')
        before = probe.snapshot_history('before_ahead')
        first = probe.ahead()
        require(first['success'] and first['count']==1 and not first['global_required'], 'First ahead round failed')
        hypothetical = PlannerSrv.Request()
        hypothetical.header.frame_id, hypothetical.bound_mode = 'map', 3
        hypothetical.root_pose.position.x, hypothetical.root_pose.orientation.w = 3., 1.
        metric_count = len(probe.local_metrics)
        probe.call('planner', hypothetical, PlannerSrv)
        probe.until(lambda:len(probe.local_metrics)>metric_count, 'hypothetical planning metrics')
        hypothetical_metric = probe.local_metrics[-1]
        distance = hypothetical_metric['direction_distance']
        require((distance is None and hypothetical_metric['direction_distance_status']=='non_finite') or
                (distance is not None and math.isfinite(distance) and hypothetical_metric['direction_distance_status']=='finite'),
                'Direction diagnostic does not state whether its JSON number is available')
        require(probe.status()['low_gain_iterations']==1, 'Ordinary hypothetical root changed the counter')
        second = probe.ahead()
        require(second['success'] and second['count']==2 and not second['global_required'], 'Second ahead round failed')
        require(probe.snapshot_history('after_ahead')==before, 'Successful ahead candidate changed visit history')
        probe.until(lambda:probe.status('control_state')['known_cells']>0 and
                    probe.status('execution_state')['command_subscribers']>0, 'PCI/follower readiness')
        request = PciTrigger.Request()
        request.bound_mode, request.set_auto = 3, True
        require(probe.call('pci_trigger', request, PciTrigger).success, 'Automatic PCI request refused')
        probe.until(lambda:probe.status('control_state')['state'] in {'EXECUTING', 'ERROR'}, 'local execution')
        require(probe.status('control_state')['state']=='EXECUTING' and probe.status()['low_gain_iterations']==3,
                'Actual initial local plan was not the third evaluated round')
        probe.publish_map()  # Observation during execution enables the fourth, ahead round.
        probe.until(lambda:any(s['reason']=='plan_ahead_global_required' for s in probe.control_events), 'ahead global latch')
        latched = next(s for s in probe.control_events if s['reason']=='plan_ahead_global_required')
        require(latched['state']=='EXECUTING' and latched['plan_ahead']['global_required']
                and latched['plan_ahead']['continuations']==0 and not probe.global_metrics,
                'Fourth ahead result committed a route or queried global before current completion')
        require(probe.status()['low_gain_iterations']==0, 'Fourth ahead round did not reset the counter')
        probe.until(lambda:probe.status('control_state')['state'] in {'WAITING_OBSERVATION', 'ERROR'},
                    'current action completion', seconds=35.)
        waiting = probe.status('control_state')
        endpoint = list(probe.position[:3])
        require(waiting['state']=='WAITING_OBSERVATION' and waiting['completed_actions']==1
                and waiting['plan_ahead']['global_required'] and not probe.global_metrics,
                'PCI failed to wait for an endpoint observation before global planning')
        probe.publish_map()
        probe.until(lambda:bool(probe.global_metrics) and any(p.poses for p in probe.global_paths), 'native global return')
        route = xyz([p.pose for p in next(p for p in probe.global_paths if p.poses).poses])
        require(math.dist(route[0], endpoint)<.05 and math.dist(route[-1], home)<1e-9,
                'Global request did not use actual endpoint odometry and original home')
        probe.until(lambda:probe.status('control_state')['state'] in {'DONE', 'ERROR'}, 'return home', seconds=35.)
        completed, execution = probe.status('control_state'), probe.status('execution_state')
        require(completed['state']=='DONE' and completed['mode']=='homing' and not completed['auto']
                and completed['completed_actions']==2 and math.dist(probe.position[:3], home)<.05,
                'Deferred global handoff did not complete a terminal native home route')
        require(execution['continuations']==0 and execution['cancelled']==0 and execution['aborted']==0
                and execution['succeeded']==2, 'Current action was spliced, cancelled or aborted')
        require(sum(e['reason']=='local_requested' for e in completed['events'])==1
                and sum(e['reason']=='global_requested' for e in completed['events'])==1,
                'PCI issued duplicate local/global requests')
        probe.publish_map()
        require(probe.status('control_state')['events']==completed['events'], 'A fresh map restarted terminal homing')
        summary['automatic'] = {'round_composition': ['direct ahead', 'direct ahead', 'PCI local', 'PCI ahead'],
                                'preseed': [first, second], 'hypothetical_metrics': hypothetical_metric,
                                'latched': latched, 'waiting': waiting,
                                'completed': completed, 'execution': execution, 'global_route': route,
                                'global_metrics': probe.global_metrics, 'trajectories': probe.trajectories}
        summary['checks'] = dict.fromkeys(['no_gain_rounds_counted', 'short_path_rounds_counted',
            'hypothetical_planner_root_excluded', 'direction_distance_has_explicit_numeric_status',
            'future_root_does_not_change_visit_history',
            'fourth_ahead_round_not_committed', 'current_action_finishes_before_global',
            'fresh_endpoint_observation_required', 'global_uses_actual_odometry', 'terminal_homing'], True)
        summary['status'] = 'passed'
    except Exception as error:
        summary['error'] = f'{type(error).__name__}: {error}'
        summary['last_control_events'] = {p.label:p.control_events[-5:] for p in fixtures}
        summary['last_local_metrics'] = {p.label:p.local_metrics[-3:] for p in fixtures}
    finally:
        stopped = True
        for probe in reversed(fixtures):
            try:
                probe.close()
            except Exception as error:
                summary.setdefault('cleanup_errors', []).append(str(error))
                stopped = False
        if initialized and rclpy.ok():
            rclpy.shutdown()
        summary['processes_stopped'] = stopped
        summary['elapsed_s'] = round(time.monotonic()-started, 3)
        if not stopped:
            summary['status'] = 'failed'
        output.write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps(summary, ensure_ascii=False))
    return 0 if summary['status']=='passed' else 1


if __name__=='__main__':
    raise SystemExit(main())
