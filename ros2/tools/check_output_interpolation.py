#!/usr/bin/env python3
"""Compare native planner output spacing with two sequential independent fixtures.

Real ROS services consume a synthetic known occupancy map and synthetic odometry.
There is no robot dynamics, trajectory consumer, or fabricated planner/graph reply.
"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def pose_values(pose):
    p, q = pose.position, pose.orientation
    return [p.x, p.y, p.z, q.x, q.y, q.z, q.w]


def path_record(poses):
    points = [pose_values(pose) for pose in poses]
    require(len(points) >= 2, 'Native service returned no usable route')
    require(all(math.isfinite(value) for point in points for value in point), 'Nonfinite native route')
    segments = [math.dist(a[:3], b[:3]) for a, b in zip(points, points[1:])]
    return {'path_xyz_xyzw': points, 'points': len(points), 'length_m': sum(segments),
            'maximum_segment_m': max(segments), 'first_segment_m': segments[0]}


def graph_record(message):
    return {
        'vertices': sorted(({'id': vertex.id, 'pose_xyz_xyzw': pose_values(vertex.pose),
                             'gain_counts': [vertex.num_unknown_voxels, vertex.num_free_voxels,
                                             vertex.num_occupied_voxels],
                             'is_frontier': vertex.is_frontier} for vertex in message.vertices),
                           key=lambda value: value['id']),
        'edges': sorted([[edge.source_id, edge.target_id, edge.weight] for edge in message.edges]),
    }


def compare_paths(baseline, dense, name):
    original, refined = baseline['path_xyz_xyzw'], dense['path_xyz_xyzw']
    require(original[0] == refined[0] and original[-1] == refined[-1],
            name+': exact endpoint positions/orientations changed')
    require(abs(baseline['length_m']-dense['length_m']) < 1.e-9, name+': arc length changed')
    require(baseline['maximum_segment_m'] > .25+1.e-9, name+': baseline does not exercise subdivision')
    require(dense['maximum_segment_m'] <= .25+1.e-9, name+': dense output exceeds spacing')
    # Verify that every existing pose survives in order, and every inserted
    # pose remains on its original segment with that segment's successor yaw.
    segment = 0
    for point in refined[1:]:
        require(segment+1 < len(original), name+': output extends past its endpoint')
        if point == original[segment+1]:
            segment += 1
            continue
        a, b = original[segment], original[segment+1]
        delta = [b[i]-a[i] for i in range(3)]
        norm2 = sum(value*value for value in delta)
        require(norm2 > 0., name+': inserted geometry into a pure-yaw segment')
        fraction = sum((point[i]-a[i])*delta[i] for i in range(3))/norm2
        residual = math.dist(point[:3], [a[i]+fraction*delta[i] for i in range(3)])
        require(0. < fraction < 1. and residual < 1.e-9, name+': skipped a bend or left its original segment')
        require(point[3:] == b[3:], name+': inserted point did not inherit successor orientation')
    require(segment == len(original)-1, name+': original waypoint was omitted')
    return {'baseline_points': baseline['points'], 'dense_points': dense['points'],
            'length_m': dense['length_m'], 'dense_maximum_segment_m': dense['maximum_segment_m'],
            'exact_endpoints_and_original_knots': True, 'same_polyline': True}


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=workspace/'results/output_interpolation/summary.json')
    parser.add_argument('--timeout', type=float, default=120.)
    args = parser.parse_args()
    require(math.isfinite(args.timeout) and args.timeout > 0., 'Timeout must be positive')
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100+os.getpid()%80))
    # Import the existing static-map fixture without making __pycache__ output.
    sys.dont_write_bytecode = True
    started, initialized = time.monotonic(), False
    deadline = started+args.timeout
    probes = []
    summary = {'status': 'failed', 'scope': __doc__, 'fixtures': {}, 'checks': {},
               'domain': os.environ['ROS_DOMAIN_ID'], 'launches': []}
    try:
        import rclpy
        from geometry_msgs.msg import Pose
        from nav_msgs.msg import Odometry, Path as RosPath
        from planner_msgs.msg import Graph
        from planner_msgs.srv import (PciTrigger, PlanAhead, PlannerGlobal, PlannerGoToWaypoint,
                                      PlannerHoming, PlannerSrv)
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        sys.path.insert(0, str(workspace/'src/gbplanner_local/scripts'))
        from local_demo import DIM, ORIGIN, RES, make_map

        rclpy.init(args=[])
        initialized = True
        durable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        common = {
            'map.resolution': RES, 'map.origin': list(ORIGIN), 'map.dimensions': list(DIM),
            'global.enable': True, 'global.auto_home': True, 'global.background_enable': False,
            'planning.auto_global_planner_enable': False, 'planning.go_home_if_fully_explored': False,
            'planning.path_safety_enhance_enable': False, 'planning.adaptive_exploration': False,
            'sampling.seed': 42, 'sampling.min': [-.5, -.5, -.1], 'sampling.max': [4.5, .5, .1],
            'planning.max_vertices': 3, 'planning.max_attempts': 10, 'planning.graph_mode': False,
            'planning.edge_min': .05, 'planning.edge_max': 1.2, 'planning.overshoot': 0.,
            'planning.v_max': .3, 'planning.traverse_length_max': 2., 'planning.traverse_time_max': 100.,
            'planning.path_length_penalty': 0., 'planning.path_direction_penalty': 0.,
            'planning.cluster_vertices_for_gain': False, 'robot.size': [.1, .1, .1],
            'gain.free': 1., 'gain.unknown': 1., 'gain.frontier_threshold': .05,
            'sensor.range': .75, 'sensor.horizontal_fov': 1., 'sensor.horizontal_resolution': 1.,
            'sensor.vertical_fov': .2, 'sensor.vertical_resolution': .2,
        }
        for axis in ('x', 'y', 'z', 'heading'):
            common[f'sampling.{axis}.distribution'] = 'constant'
            common[f'sampling.{axis}.constant'] = 4. if axis == 'x' else 0.
        summary['common_parameters'] = common

        def stamp_ns(message):
            return message.header.stamp.sec*1000000000+message.header.stamp.nanosec

        class Probe(Node):
            def __init__(self, label, spacing):
                namespace = f'/output_interpolation_{os.getpid()}_{label}'
                super().__init__('probe', namespace=namespace)
                self.label, self.namespace_name = label, namespace
                self.processes, self.service_clients = [], {}
                self.local_graphs, self.global_graphs, self.local_metrics = [], [], []
                self.global_metrics, self.waypoint_metrics, self.commands = [], [], []
                self.position = (-2., 0., 0., .3)
                self.closed = False
                self.maps = self.create_publisher(PointCloud2, 'voxel_map', durable)
                self.odometry = self.create_publisher(Odometry, 'odometry', 10)
                self.create_subscription(Graph, 'local_graph', self.local_graphs.append, durable)
                self.create_subscription(Graph, 'global_graph', self.global_graphs.append, durable)
                self.create_subscription(String, 'planning_metrics',
                    lambda message:self.local_metrics.append(self.decode('planning_metrics', message.data)), durable)
                self.create_subscription(String, 'global_metrics',
                    lambda message:self.global_metrics.append(self.decode('global_metrics', message.data)), durable)
                self.create_subscription(String, 'waypoint_metrics',
                    lambda message:self.waypoint_metrics.append(self.decode('waypoint_metrics', message.data)), durable)
                self.create_subscription(RosPath, 'command_path', self.commands.append, durable)
                probes.append(self)
                self.record = {'spacing_m': spacing, 'routes': {}, 'graphs': {}, 'metrics': {}}
                summary['fixtures'][label] = self.record
                parameters = dict(common, **{'planning.path_interpolation_distance': spacing})
                self.launch('local_planner_node', parameters)

            def launch(self, executable, parameters):
                command = ['ros2', 'run', 'gbplanner_local', executable, '--ros-args',
                           '-r', '__ns:='+self.namespace_name]
                for key, value in parameters.items():
                    command += ['-p', key+':='+json.dumps(value)]
                log_path = output.parent/f'{self.label}_{executable}.log'
                with log_path.open('w') as log:
                    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                               start_new_session=True, env=dict(os.environ))
                self.processes.append((process, log_path))
                summary['launches'].append(command)

            def until(self, predicate, label, seconds=10.):
                limit = min(deadline, time.monotonic()+seconds)
                while not predicate():
                    for process, log in self.processes:
                        if process.poll() is not None:
                            raise RuntimeError(f'{self.label}: process exited {process.returncode}; see {log}')
                    if time.monotonic() >= limit:
                        raise TimeoutError(f'{self.label}: {label}')
                    rclpy.spin_once(self, timeout_sec=.02)

            def call(self, name, request, kind=Trigger):
                if name not in self.service_clients:
                    self.service_clients[name] = self.create_client(kind, name)
                client = self.service_clients[name]
                self.until(client.service_is_ready, name+' discovery', 15.)
                future = client.call_async(request)
                self.until(future.done, name+' response')
                response = future.result()
                require(response is not None, name+': empty service result')
                return response

            def decode(self, source, raw):
                try:
                    return json.loads(raw)
                except json.JSONDecodeError:
                    raw_path = output.parent/f'{self.label}_{source}_invalid.txt'
                    raw_path.write_text(raw)
                    self.record['invalid_json'] = {'source': source, 'raw_path': str(raw_path)}
                    raise

            def status(self, name='global_status'):
                return self.decode(name, self.call(name, Trigger.Request()).message)

            def pose(self):
                value = Pose()
                value.position.x, value.position.y, value.position.z = self.position[:3]
                value.orientation.z, value.orientation.w = math.sin(self.position[3]/2), math.cos(self.position[3]/2)
                return value

            def publish_odometry(self):
                message = Odometry()
                message.header.frame_id, message.child_frame_id = 'map', 'base_link'
                message.header.stamp = self.get_clock().now().to_msg()
                message.pose.pose = self.pose()
                self.odometry.publish(message)

            def move_fixture(self, position):
                self.position = tuple(position)
                self.publish_odometry()
                self.until(lambda:math.dist(self.status('sensor_gain').get('root', [1.e6]*4)[:3],
                                             self.position[:3]) < 1.e-10, 'synthetic odometry accepted')

            def initialize(self):
                self.until(lambda:self.maps.get_subscription_count()>0 and
                                  self.odometry.get_subscription_count()>0, 'map/odometry discovery', 15.)
                before = self.status('map_status')['map_revision']
                self.maps.publish(make_map(known=True, wall=False))
                self.until(lambda:self.status('map_status')['map_revision']==before+1, 'map accepted')
                self.move_fixture(self.position)
                self.until(lambda:self.global_graphs and self.global_graphs[-1].vertices, 'home graph')
                self.home = copy.deepcopy(next(vertex.pose for vertex in self.global_graphs[-1].vertices if vertex.id==0))
                require(all(abs(actual-expected)<1.e-12 for actual, expected in
                            zip(pose_values(self.home), pose_values(self.pose()))),
                        'Home pose does not match synthetic input')
                self.record['home_xyz_xyzw'] = pose_values(self.home)

            def save_graphs(self, label, since, local=False):
                self.until(lambda:self.global_graphs and stamp_ns(self.global_graphs[-1])>=since,
                           label+' global graph publication')
                record = {'global': graph_record(self.global_graphs[-1])}
                if local:
                    self.until(lambda:self.local_graphs and stamp_ns(self.local_graphs[-1])>=since,
                               label+' local graph publication')
                    record['local'] = graph_record(self.local_graphs[-1])
                self.record['graphs'][label] = record

            def run_services(self):
                self.initialize()
                request = PlannerSrv.Request()
                request.header.frame_id, request.bound_mode = 'map', 3
                request.root_pose.orientation.w = 0.  # Native current-odometry sentinel.
                before, since = len(self.local_metrics), self.get_clock().now().nanoseconds
                local = self.call('planner', request, PlannerSrv)
                self.until(lambda:len(self.local_metrics)>before, 'local metrics')
                metric = self.local_metrics[-1]
                self.record['metrics']['local'] = metric
                require(metric['success'] and metric['gain']>0. and not metric['refinement']['attempted'],
                        'Local fixture did not plan with positive gain and disabled enhancement')
                require(local.status==PlannerSrv.Response.K_FORWARD, 'Local fixture switched planning modes')
                self.record['routes']['local'] = path_record(local.path)
                self.save_graphs('local', since, local=True)

                request = PlanAhead.Request()
                request.header.frame_id, request.bound_mode, request.arrival_time = 'map', 3, 1.
                request.root_pose = copy.deepcopy(local.path[-1])
                since = self.get_clock().now().nanoseconds
                ahead = self.call('planner_ahead', request, PlanAhead)
                self.record['metrics']['ahead'] = {'success': ahead.success, 'reason': ahead.reason,
                    'homing_required': ahead.homing_required, 'global_required': ahead.global_required}
                require(ahead.success and not ahead.homing_required and not ahead.global_required,
                        'PlanAhead fixture failed: '+ahead.reason)
                self.record['routes']['ahead'] = path_record(ahead.path)
                self.save_graphs('ahead', since, local=True)

                # Select a real node inserted by local/ahead reference paths.
                candidates = [vertex for vertex in self.global_graphs[-1].vertices
                              if vertex.id>0 and not vertex.is_frontier]
                require(candidates, 'Native reference insertion did not create a nonfrontier target')
                target = max(candidates, key=lambda vertex:vertex.pose.position.x)
                require(math.dist(pose_values(target.pose)[:3],pose_values(self.home)[:3])>1.,
                        'Native global target is too close to exercise output spacing')
                self.record['explicit_target'] = {'id': target.id, 'pose_xyz_xyzw': pose_values(target.pose),
                                                  'is_frontier': target.is_frontier}
                request = PlannerGlobal.Request()
                request.id, request.not_check_frontier, request.ignore_time, request.bound_mode = target.id, True, True, 3
                before, since = len(self.global_metrics), self.get_clock().now().nanoseconds
                global_path = self.call('planner_global', request, PlannerGlobal)
                self.until(lambda:len(self.global_metrics)>before, 'global metrics')
                self.record['metrics']['global'] = self.global_metrics[-1]
                require(not global_path.homing and self.global_metrics[-1]['success'], 'Explicit global request did not reach its real graph target')
                self.record['routes']['global'] = path_record(global_path.path)
                self.save_graphs('global', since)

                self.move_fixture((*pose_values(target.pose)[:3], .3))
                request = PlannerHoming.Request()
                request.header.frame_id, request.bound_mode = 'map', 3
                before, since = len(self.global_metrics), self.get_clock().now().nanoseconds
                home = self.call('exploration_homing', request, PlannerHoming)
                self.until(lambda:len(self.global_metrics)>before, 'homing metrics')
                self.record['metrics']['homing'] = self.global_metrics[-1]
                self.record['routes']['homing'] = path_record(home.path)
                require(math.dist(pose_values(home.path[-1])[:3],pose_values(self.home)[:3])<1.e-10,
                        'Native homing endpoint changed')
                self.save_graphs('homing', since)

                request = PlannerGoToWaypoint.Request()
                request.waypoint.header.frame_id, request.bound_mode, request.check_collision = 'map', 3, True
                request.waypoint.pose = copy.deepcopy(self.home)
                before, since = len(self.waypoint_metrics), self.get_clock().now().nanoseconds
                waypoint = self.call('planner_go_to_waypoint', request, PlannerGoToWaypoint)
                self.until(lambda:len(self.waypoint_metrics)>before, 'waypoint metrics')
                self.record['metrics']['waypoint'] = self.waypoint_metrics[-1]
                self.record['routes']['waypoint'] = path_record(waypoint.path)
                require(math.dist(pose_values(waypoint.path[-1])[:3],pose_values(self.home)[:3])<1.e-10,
                        'Native waypoint endpoint changed')
                self.save_graphs('waypoint', since)
                self.record['final_global_status'] = self.status()
                require(self.record['final_global_status']['low_gain_iterations']==0,
                        'Disabled automatic global switching altered its counter')

            def preview(self):
                # Run only after the comparison snapshots. A real PCI preview
                # needs no action server and cannot issue a physical motion.
                self.move_fixture((-2., 0., 0., .3))
                self.launch('control_node.py', {'planner_trigger_lead_time': 0., 'request_timeout': 10.})
                timer = self.create_timer(.05, self.publish_odometry)
                try:
                    self.until(lambda:self.status('control_state').get('known_cells', 0)>0,
                               'PCI receives accepted map', 15.)
                    self.until(lambda:self.odometry.get_subscription_count()>=2, 'PCI odometry discovery')
                    self.publish_odometry()
                    before = len(self.commands)
                    request = PciTrigger.Request()
                    request.bound_mode, request.not_exe_path, request.set_auto = 3, True, False
                    # The trigger itself acknowledges PCI readiness. Rejected
                    # previews are inert; stop retrying at the first accepted one.
                    self.until(lambda:self.call('pci_trigger', request, PciTrigger).success,
                               'PCI preview trigger accepted')
                    self.until(lambda:self.status('control_state')['state'] in {'PREVIEW','ERROR','STOPPED'},
                               'PCI preview completion')
                    state = self.status('control_state')
                    require(state['state']=='PREVIEW' and state['mode']=='local', 'PCI did not finish a local preview')
                    self.until(lambda:len(self.commands)>before and len(self.commands[-1].poses)>2,
                               'PCI command_path preview')
                    record = path_record([point.pose for point in self.commands[-1].poses])
                    require(record['path_xyz_xyzw']==self.record['routes']['local']['path_xyz_xyzw'],
                            'PCI preview truncated or changed the complete local output route')
                    self.record['pci_preview'] = record
                finally:
                    self.destroy_timer(timer)

            def close(self):
                if self.closed:
                    return
                self.closed = True
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
                        try:
                            os.killpg(process.pid, signal.SIGKILL)
                        except ProcessLookupError:
                            pass
                        process.wait(timeout=1.)
                self.destroy_node()

        for label, spacing in (('baseline', 100.), ('dense', .25)):
            probe = Probe(label, spacing)
            try:
                probe.run_services()
                if label=='dense':
                    probe.preview()
            finally:
                probe.close()

        baseline, dense = summary['fixtures']['baseline'], summary['fixtures']['dense']
        for name in ('local', 'ahead', 'global', 'homing', 'waypoint'):
            summary['checks'][name] = compare_paths(baseline['routes'][name], dense['routes'][name], name)
            require(baseline['graphs'][name]==dense['graphs'][name], name+': native graph/gain changed with output spacing')
            summary['checks'][name]['native_graph_identical'] = True
        require(dense['routes']['local']['points']>baseline['routes']['local']['points'],
                'Dense local response did not gain output points')
        keys = ('vertices','edges','gain','gain_evaluations','reused_gains','candidate_path_length_m',
                'traverse_length_m','traverse_time_s','executed_target_id','unpenalized_gain')
        for key in keys:
            require(baseline['metrics']['local'][key]==dense['metrics']['local'][key],
                    'Local selection/accounting changed: '+key)
        summary['checks']['local_gain_and_traversal_unchanged'] = {key:dense['metrics']['local'][key] for key in keys}
        summary['checks']['pci_preview_full_path_m'] = dense['pci_preview']['length_m']
        summary['status'] = 'passed'
    except Exception as error:
        summary['error'] = str(error)
        summary['traceback'] = traceback.format_exc()
    finally:
        cleanup_errors = []
        for probe in reversed(probes):
            try:
                probe.close()
            except Exception as error:
                cleanup_errors.append(str(error))
        if initialized:
            try:
                rclpy.shutdown()
            except Exception as error:
                cleanup_errors.append(str(error))
        if cleanup_errors:
            summary['cleanup_errors'], summary['status'] = cleanup_errors, 'failed'
        summary['wall_time_s'] = time.monotonic()-started
        temporary = output.with_suffix('.tmp')
        temporary.write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
        temporary.replace(output)
        print(json.dumps({'status':summary['status'], 'output':str(output), 'error':summary.get('error'),
                          'wall_time_s':summary['wall_time_s']}, ensure_ascii=False))
    return 0 if summary['status']=='passed' else 1


if __name__=='__main__':
    raise SystemExit(main())
