#!/usr/bin/env python3
"""Check real selected-vertex ray/voxel gain geometry without robot motion.

Two 21-vertex chains evaluate vertex 20 in ray and voxel modes. Two tiny
three-vertex fixtures check that copied and leaf-skipped gains invent no samples.
All geometry comes from actual planner service calls on one synthetic map shape.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback

sys.dont_write_bytecode = True
from check_planning_visualization import close, close_points, require, snapshot_record, stamp_ns, xyz
from check_frontier_clustering import graph_record


COUNT_NAMES = {-1: 'unknown', 0: 'free', 100: 'occupied'}


def marker_map(message, metadata, marker_type):
    require(message.markers and message.markers[0].action == marker_type.DELETEALL,
            'Gain marker snapshot does not start with DELETEALL')
    result = {}
    for index, marker in enumerate(message.markers):
        require(marker.header.frame_id == 'map' and stamp_ns(marker.header) == metadata['stamp_ns'],
                'Gain marker belongs to a different frame or snapshot')
        if index == 0:
            continue
        identity = marker.ns, marker.id
        require(marker.action == marker_type.ADD and identity not in result, 'Invalid or repeated gain marker')
        require(marker.color.a > 0., 'Gain marker is invisible')
        q = marker.pose.orientation
        require(abs(q.x)+abs(q.y)+abs(q.z) < 1.e-8 and abs(abs(q.w)-1.) < 1.e-8,
                'Gain marker unexpectedly rotates captured coordinates')
        if marker.type in (marker_type.LINE_LIST, marker_type.SPHERE_LIST):
            close(xyz(marker.pose.position), [0., 0., 0.], 'Gain marker origin')
        result[identity] = marker
    return result


def verify_markers(metadata, rays_message, samples_message, marker_type):
    rays = marker_map(rays_message, metadata, marker_type)
    samples = marker_map(samples_message, metadata, marker_type)
    geometry = metadata['geometry']
    expected_rays, expected_samples = set(), set()
    if geometry['evaluated'] and geometry['state_valid']:
        for index, sensor in enumerate(geometry['sensors']):
            if sensor['endpoints']:
                identity = 'gain_rays', index
                expected_rays.add(identity)
                require(identity in rays and rays[identity].type == marker_type.LINE_LIST, 'Full endpoint rays are missing')
                expected = [point for endpoint in sensor['endpoints'] for point in (geometry['state'][:3], endpoint)]
                close_points([xyz(point) for point in rays[identity].points], expected, 'Actual full ray/query endpoints')
                require(abs(rays[identity].scale.x-.015) < 1.e-10, 'Gain ray line width differs')
            for kind, (status, name) in enumerate(COUNT_NAMES.items()):
                points = [item['point'] for item in sensor['contributions'] if item['status'] == status]
                if not points:
                    continue
                identity = 'gain_'+name, index*3+kind
                expected_samples.add(identity)
                require(identity in samples and samples[identity].type == marker_type.SPHERE_LIST,
                        'Gain contribution glyphs are missing')
                close_points([xyz(point) for point in samples[identity].points], points,
                             'One glyph per contribution in original order, without expanding weight')
                close(xyz(samples[identity].scale), [.06]*3, 'Gain contribution glyph size')
            minimum, maximum = sensor['minimum'], sensor['maximum']
            if sensor['roi_valid'] and all(a < b for a, b in zip(minimum, maximum)):
                identity = 'gain_roi', index
                expected_samples.add(identity)
                require(identity in samples and samples[identity].type == marker_type.CUBE, 'Voxel gain ROI is missing')
                close(xyz(samples[identity].pose.position), [(a+b)/2 for a, b in zip(minimum, maximum)], 'Gain ROI center')
                close(xyz(samples[identity].scale), [b-a for a, b in zip(minimum, maximum)], 'Gain ROI dimensions')
    require(set(rays) == expected_rays and set(samples) == expected_samples, 'Gain display contains stale or invented geometry')


def counts_from_vertex(vertex):
    return dict(zip(('unknown', 'free', 'occupied'), vertex['counts']))


def verify_gain(metadata, graph, metric, parameters, expected_root, reason):
    geometry = metadata['geometry']
    record = graph_record(graph)
    vertices = {vertex['id']: vertex for vertex in record['vertices']}
    selected = parameters['visualization.gain_vertex_id']
    require(len(vertices) == parameters['planning.max_vertices'] and selected in vertices,
            'Fixed sampling fixture did not create the selected real vertex')
    close(vertices[0]['state'][:3], expected_root, 'Planning graph root')
    require(geometry['enabled'] and geometry['entered'] and geometry['state_valid'] and
            geometry['requested_id'] == selected and geometry['vertex_id'] == selected,
            'Gain diagnostics do not identify the selected native vertex')
    close(geometry['state'][:3], vertices[selected]['state'][:3], 'Selected viewpoint position')
    angle = geometry['state'][3]-vertices[selected]['state'][3]
    require(abs(math.atan2(math.sin(angle), math.cos(angle))) < 1.e-8, 'Selected viewpoint yaw differs')
    require(geometry['reason'] == reason and geometry['policy'] == 'local_workspace_no_gain_zones',
            'Gain capture reason or reward policy differs')
    mode = 'ray' if parameters['planning.use_ray_model_for_volumetric_gain'] else 'voxel'
    require(geometry['mode'] == mode, 'Gain diagnostics use the wrong evaluation mode')
    require(not geometry['partial'] and not geometry['truncated'], 'Small gain fixture unexpectedly truncated its capture')
    coefficients = {name:parameters['gain.'+name] for name in COUNT_NAMES.values()}
    if reason != 'evaluated':
        require(not geometry['evaluated'] and not geometry['complete'] and geometry['sensors'] == [],
                'Copied or skipped vertex fabricated a sensor evaluation')
        require(all(geometry[key] == 0 for key in ('total_endpoints', 'dropped_endpoints', 'total_samples', 'dropped_samples')),
                'Copied or skipped vertex fabricated geometry counts')
        if reason == 'clustered_copy':
            require(geometry['gain_valid'] and geometry['source_id'] == 2,
                    'Copied gain does not identify its actual leaf representative')
            require(geometry['gain']['counts'] == counts_from_vertex(vertices[selected]) == counts_from_vertex(vertices[2]),
                    'Captured copied gain differs from the actual graph assignment')
            require(metric['gain_evaluations'] == 1 and metric['reused_gains'] == 2,
                    'Tiny copied-gain fixture did not exercise one real evaluation and two copies')
        else:
            require(geometry['source_id'] == -1 and all(value == 0 for value in counts_from_vertex(vertices[selected]).values()),
                    'Leaf-only skipped internal vertex received an evaluated gain')
            require(metric['gain_evaluations'] == 1 and metric['reused_gains'] == 0,
                    'Tiny leaf-only fixture did not skip its internal vertices')
        return {'selected_id': selected, 'reason': reason, 'no_fabricated_samples': True}

    require(geometry['evaluated'] and geometry['complete'] and geometry['gain_valid'] and
            geometry['source_id'] == selected, 'Selected vertex lacks a complete real evaluation')
    require(len(geometry['sensors']) == 1, 'Fixture should have one actual legacy sensor')
    accumulated = dict.fromkeys(COUNT_NAMES.values(), 0)
    sensor_checks, total_gain, frontier, endpoint_count, sample_count = [], 0., False, 0, 0
    for sensor in geometry['sensors']:
        result, contributions, endpoints = sensor['result'], sensor['contributions'], sensor['endpoints']
        require(sensor['complete'] and result['mode'] == mode, 'Sensor evaluation is incomplete or mislabeled')
        require(sensor['total_endpoints'] == len(endpoints) and sensor['total_samples'] == len(contributions) and
                sensor['dropped_endpoints'] == sensor['dropped_samples'] == 0, 'Sensor geometry count mismatch')
        endpoint_count += len(endpoints)
        sample_count += len(contributions)
        counts = dict.fromkeys(COUNT_NAMES.values(), 0)
        for contribution in contributions:
            status, weight, point = contribution['status'], contribution['weight'], contribution['point']
            require(status in COUNT_NAMES and isinstance(weight, int) and weight > 0 and
                    len(point) == 3 and all(math.isfinite(value) for value in point), 'Invalid accepted gain contribution')
            require(all(origin <= value < origin+size*resolution for value, origin, size, resolution in
                        zip(point, parameters['map.origin'], parameters['map.dimensions'], [parameters['map.resolution']]*3)),
                    'Accepted reward point lies outside the configured map')
            counts[COUNT_NAMES[status]] += weight
        require(counts == result['counts'], 'Weighted raw contribution sums differ from actual sensor counts')
        expected_gain = sum(counts[name]*coefficients[name] for name in counts)
        require(math.isclose(result['gain'], expected_gain, rel_tol=1.e-10, abs_tol=1.e-8),
                'Sensor gain differs from weighted observed contributions')
        for name in counts:
            accumulated[name] += counts[name]
        total_gain += result['gain']
        frontier = frontier or result['is_frontier']
        if mode == 'ray':
            require(not sensor['roi_valid'] and sensor['raycast_complete'] and result['ray_method'] == 'iterative',
                    'Ray evaluation has incorrect ROI or raycast metadata')
            require(sensor['map_resolution_valid'] and sensor['map_resolution'] == parameters['map.resolution'],
                    'Ray capture omitted its actual map resolution')
            expected_endpoints = []
            vertical = -parameters['sensor.vertical_fov']/2
            yaw, view = geometry['state'][3], geometry['state'][:3]
            while vertical < parameters['sensor.vertical_fov']/2-1.e-10:
                horizontal = -parameters['sensor.horizontal_fov']/2
                while horizontal < parameters['sensor.horizontal_fov']/2-1.e-10:
                    # Preserve SensorParamsBase's original non-spherical Z:
                    # horizontal range is R, while Z is R*sin(vertical).
                    radius = parameters['sensor.range']
                    x, y, z = radius*math.cos(horizontal), radius*math.sin(horizontal), radius*math.sin(vertical)
                    expected_endpoints.append([view[0]+math.cos(yaw)*x-math.sin(yaw)*y,
                                               view[1]+math.sin(yaw)*x+math.cos(yaw)*y, view[2]+z])
                    horizontal += parameters['sensor.horizontal_resolution']
                vertical += parameters['sensor.vertical_resolution']
            close_points(endpoints, expected_endpoints, 'Complete configured sensor endpoints, not obstacle hits')
            require(result['ray_count'] == result['ray_width']*result['ray_height'] == len(endpoints) == 96,
                    'Unexpected native angular ray grid')
            require(result['raycast']['weighted_samples'] >= sum(counts.values()) and
                    result['raycast']['step_increases'] > 0 and any(item['weight'] > 1 for item in contributions),
                    'Fixture did not capture nonuniform weighted samples')
            full_fov = (parameters['sensor.horizontal_fov']/parameters['sensor.horizontal_resolution'])*(
                        parameters['sensor.vertical_fov']/parameters['sensor.vertical_resolution'])*parameters['sensor.range']
            require(result['is_frontier'] == (counts['unknown']*sensor['map_resolution']/full_fov >= parameters['gain.frontier_threshold']),
                    'Sensor frontier classification differs from its actual weighted unknown count')
        else:
            require(sensor['roi_valid'] and not result['is_frontier'] and result['ray_count'] == 0,
                    'Voxel evaluation incorrectly presents ray-mode frontier semantics')
            resolution, view = parameters['planning.exp_gain_voxel_size'], geometry['state'][:3]
            minimum = [max(root+local, origin, coordinate-parameters['sensor.range'])
                       for root, local, origin, coordinate in zip(expected_root, parameters['sampling.min'], parameters['map.origin'], view)]
            maximum = [min(root+local, origin+size*parameters['map.resolution'], coordinate+parameters['sensor.range'])
                       for root, local, origin, size, coordinate in zip(expected_root, parameters['sampling.max'],
                       parameters['map.origin'], parameters['map.dimensions'], view)]
            close(sensor['minimum'], minimum, 'Actual voxel-gain ROI minimum')
            close(sensor['maximum'], maximum, 'Actual voxel-gain ROI maximum')
            require(sensor['step'] == resolution, 'Voxel capture has wrong integration step')
            require(result['visibility_queries'] == len(endpoints) and
                    0 < len(contributions) <= len(endpoints) <= result['fov_voxel_count'] <= result['voxel_count'],
                    'Voxel evaluation counts do not match actual visibility endpoints')
            endpoint_index = {tuple(point): index for index, point in enumerate(endpoints)}
            require(len(endpoint_index) == len(endpoints), 'Voxel visibility endpoint was duplicated')
            previous = -1
            for contribution in contributions:
                require(contribution['weight'] == 1 and tuple(contribution['point']) in endpoint_index,
                        'Voxel contribution is not a unit-weight accepted visibility endpoint')
                index = endpoint_index[tuple(contribution['point'])]
                require(index > previous, 'Voxel contributions do not preserve actual query order')
                previous = index
        sensor_checks.append({'mode': mode, 'weighted_counts': counts, 'gain': result['gain'],
                              'endpoints': len(endpoints), 'contribution_events': len(contributions)})
    require(geometry['total_endpoints'] == endpoint_count <= 4096 and geometry['total_samples'] == sample_count <= 8192 and
            geometry['dropped_endpoints'] == geometry['dropped_samples'] == 0, 'Top-level capture counts or limits differ')
    require(accumulated == geometry['gain']['counts'] == counts_from_vertex(vertices[selected]),
            'Sensor sums, selected gain, and actual graph fields disagree')
    require(math.isclose(total_gain, geometry['gain']['gain'], rel_tol=1.e-10, abs_tol=1.e-8) and
            frontier == geometry['gain']['is_frontier'] == vertices[selected]['frontier'], 'Selected gain or frontier sum differs')
    expected_path_gain = sum(sum(counts_from_vertex(vertex)[name]*coefficients[name] for name in coefficients)
                             for vertex in vertices.values())
    require(math.isclose(metric['gain'], expected_path_gain, rel_tol=1.e-9, abs_tol=1.e-7),
            'Single-chain path score differs from its actual graph gains')
    return {'selected_id': selected, 'reason': reason, 'sensors': sensor_checks,
            'selected_graph_counts_match': True, 'single_chain_path_gain': expected_path_gain}


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=workspace/'results/gain_geometry/summary.json')
    parser.add_argument('--timeout', type=float, default=180.)
    args = parser.parse_args()
    require(math.isfinite(args.timeout) and args.timeout > 0., 'Timeout must be positive')
    output = args.output.resolve()
    require(not output.exists(), 'Refusing to overwrite existing gain results')
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100+os.getpid()%80))
    started, initialized, probe, process = time.monotonic(), False, None, None
    deadline = started+args.timeout
    summary = {'status': 'failed', 'scope': __doc__, 'domain': os.environ['ROS_DOMAIN_ID'], 'fixtures': {}}
    try:
        import rclpy
        from geometry_msgs.msg import Pose
        from nav_msgs.msg import Odometry
        from planner_msgs.msg import Graph
        from planner_msgs.srv import PlanAhead, PlannerSrv
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from visualization_msgs.msg import Marker, MarkerArray
        sys.path.insert(0, str(workspace/'src/gbplanner_local/scripts'))
        from local_demo import DIM, ORIGIN, RES, make_map

        rclpy.init(args=[])
        initialized = True
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        common = {
            'map.resolution': RES, 'map.origin': list(ORIGIN), 'map.dimensions': list(DIM),
            'global.enable': False, 'global.background_enable': False, 'mission.enabled': False,
            'planning.auto_global_planner_enable': False, 'planning.path_safety_enhance_enable': False,
            'planning.adaptive_exploration': False, 'visualization.planning_diagnostics': True,
            'planning.path_interpolation_distance': .5, 'planning.traverse_length_max': 10.,
            'planning.traverse_time_max': 100., 'planning.path_length_penalty': 0., 'planning.path_direction_penalty': 0.,
            'robot.size': [.1, .1, .1], 'sampling.seed': 42, 'sampling.seed_mode': 'fixed',
            'sampling.min': [-1., -2., -1.], 'sampling.max': [5., 2., 1.],
            'planning.graph_mode': False, 'planning.edge_min': .05, 'planning.edge_max': .15,
            'planning.overshoot': 0., 'planning.max_vertices': 21, 'planning.max_attempts': 60,
            'planning.cluster_vertices_for_gain': False, 'planning.leaf_gain_only': False,
            'planning.clustering_radius': 2., 'visualization.gain_vertex_id': 20,
            'sensor.range': 6., 'sensor.horizontal_fov': 2*math.pi, 'sensor.vertical_fov': math.pi/3,
            'sensor.horizontal_resolution': math.pi/12, 'sensor.vertical_resolution': math.pi/12,
            'gain.unknown': 1., 'gain.free': .1, 'gain.occupied': .05, 'gain.frontier_threshold': .1,
            'planning.ray_cast_method': 'iterative', 'planning.nonuniform_ray_cast': True,
            'planning.exp_gain_voxel_size': .25,
        }
        for axis in ('x', 'y', 'z', 'heading'):
            common[f'sampling.{axis}.distribution'] = 'constant'
            common[f'sampling.{axis}.constant'] = 3. if axis == 'x' else 0.

        class Probe(Node):
            def __init__(self, label, record):
                super().__init__('probe', namespace=f'/gain_geometry_{os.getpid()}_{label}')
                self.record, self.rpc_clients = record, {}
                self.geometry, self.graphs, self.rays, self.samples = [], [], [], []
                self.metrics = {'local': [], 'ahead': []}
                self.maps = self.create_publisher(PointCloud2, 'voxel_map', durable)
                self.odometry = self.create_publisher(Odometry, 'odometry', 10)
                self.create_subscription(Graph, 'local_graph', self.graphs.append, durable)
                self.create_subscription(String, 'planning_gain_geometry',
                                         lambda message:self.geometry.append(json.loads(message.data)), durable)
                self.create_subscription(MarkerArray, 'vis/rays', self.rays.append, durable)
                self.create_subscription(MarkerArray, 'vis/volumetric_gains', self.samples.append, durable)
                for label, topic in (('local', 'planning_metrics'), ('ahead', 'plan_ahead_metrics')):
                    self.create_subscription(String, topic,
                        lambda message, values=self.metrics[label]:values.append(json.loads(message.data)), durable)

            def until(self, predicate, label, seconds=20.):
                limit = min(deadline, time.monotonic()+seconds)
                while not predicate():
                    if process.poll() is not None:
                        raise RuntimeError(f'Planner exited {process.returncode}; see {self.record["log"]}')
                    if time.monotonic() >= limit:
                        raise TimeoutError(label)
                    rclpy.spin_once(self, timeout_sec=.02)

            def call(self, name, request, kind=Trigger):
                if name not in self.rpc_clients:
                    self.rpc_clients[name] = self.create_client(kind, name)
                client = self.rpc_clients[name]
                self.until(client.service_is_ready, name+' discovery')
                future = client.call_async(request)
                self.until(future.done, name+' response', 45.)
                response = future.result()
                require(response is not None, name+': empty response')
                return response

            def status(self, name):
                return json.loads(self.call(name, Trigger.Request()).message)

            def publish_map(self, message):
                previous = self.status('map_status')['map_revision']
                self.maps.publish(message)
                self.until(lambda:self.status('map_status')['map_revision'] == previous+1, 'Map accepted')
                return previous+1

            def snapshot(self, label, before, since, revision, request=None, empty=False):
                self.until(lambda:len(self.geometry) > before and self.geometry[-1]['map_revision'] == revision and
                    self.geometry[-1]['stamp_ns'] >= since and (request is None or self.geometry[-1]['request'] == request) and
                    self.rays and self.samples and self.rays[-1].markers and self.samples[-1].markers and
                    self.geometry[-1]['stamp_ns'] == stamp_ns(self.rays[-1].markers[0].header) ==
                    stamp_ns(self.samples[-1].markers[0].header), label+' complete gain snapshot')
                metadata = self.geometry[-1]
                record = {'metadata': metadata, 'rays': snapshot_record(self.rays[-1]), 'samples': snapshot_record(self.samples[-1])}
                self.record['phases'][label] = record
                verify_markers(metadata, self.rays[-1], self.samples[-1], Marker)
                if empty:
                    geometry = metadata['geometry']
                    require(not geometry['evaluated'] and not geometry['state_valid'] and not geometry['gain_valid'] and
                            not geometry['sensors'] and len(self.rays[-1].markers) == len(self.samples[-1].markers) == 1,
                            label+': failed/reset request retained gain geometry')
                    require(all(geometry[key] == 0 for key in ('total_endpoints', 'dropped_endpoints', 'total_samples', 'dropped_samples')),
                            label+': failed/reset request retained sample counters')
                retained = {'metadata': [], 'rays': [], 'samples': []}
                subscriptions = [self.create_subscription(String, 'planning_gain_geometry',
                    lambda message:retained['metadata'].append(json.loads(message.data)), durable),
                    self.create_subscription(MarkerArray, 'vis/rays', retained['rays'].append, durable),
                    self.create_subscription(MarkerArray, 'vis/volumetric_gains', retained['samples'].append, durable)]
                try:
                    self.until(lambda:all(retained.values()), label+' late subscriber')
                    require(retained['metadata'][-1] == metadata and
                            snapshot_record(retained['rays'][-1]) == record['rays'] and
                            snapshot_record(retained['samples'][-1]) == record['samples'],
                            label+': late subscriber received stale or partial gain data')
                    record['late_subscriber_matches'] = True
                finally:
                    for subscription in subscriptions:
                        self.destroy_subscription(subscription)
                return record

        def close_fixture():
            nonlocal probe, process
            errors = []
            if process is not None:
                cleanup = probe.record.setdefault('cleanup', {}) if probe is not None else summary.setdefault('cleanup', {})
                cleanup['forced'] = False
                try:
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGINT)
                        try:
                            process.wait(timeout=5.)
                        except subprocess.TimeoutExpired:
                            cleanup['forced'] = True
                            os.killpg(process.pid, signal.SIGKILL)
                            process.wait(timeout=2.)
                    cleanup['exit_code'] = process.returncode
                    if cleanup['forced'] or process.returncode != 0:
                        errors.append(f'Planner did not exit cleanly: {process.returncode}')
                except Exception as error:
                    errors.append(str(error))
                process = None
            if probe is not None:
                try:
                    probe.destroy_node()
                except Exception as error:
                    errors.append(str(error))
                probe = None
            require(not errors, '; '.join(errors))

        for fixture in ('ray', 'voxel', 'copied', 'skipped'):
            parameters = dict(common)
            parameters['planning.use_ray_model_for_volumetric_gain'] = fixture != 'voxel'
            if fixture in ('copied', 'skipped'):
                parameters.update({'planning.max_vertices': 3, 'planning.edge_max': .8,
                                   'visualization.gain_vertex_id': 1, 'planning.leaf_gain_only': True,
                                   'planning.cluster_vertices_for_gain': fixture == 'copied'})
            record = {'parameters': parameters, 'phases': {}}
            summary['fixtures'][fixture] = record
            probe = Probe(fixture, record)
            log_path = output.with_suffix('.'+fixture+'.planner.log')
            require(not log_path.exists(), 'Refusing to overwrite planner log: '+str(log_path))
            command = ['ros2', 'run', 'gbplanner_local', 'local_planner_node', '--ros-args', '-r', '__ns:='+probe.get_namespace()]
            for key, value in parameters.items():
                command += ['-p', key+':='+json.dumps(value)]
            record['launch'], record['log'] = command, str(log_path)
            with log_path.open('x') as log:
                process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                           start_new_session=True, env=dict(os.environ))
            probe.until(lambda:probe.maps.get_subscription_count() > 0 and probe.odometry.get_subscription_count() > 0,
                        fixture+' map/odometry discovery')
            revision = probe.publish_map(make_map())
            odometry = Odometry()
            odometry.header.frame_id, odometry.child_frame_id = 'map', 'base_link'
            odometry.header.stamp = probe.get_clock().now().to_msg()
            odometry.pose.pose.position.x, odometry.pose.pose.orientation.w = -2., 1.
            probe.odometry.publish(odometry)
            probe.until(lambda:math.dist(probe.status('sensor_gain').get('root', [1.e6]*4)[:3], [-2., 0., 0.]) < 1.e-8,
                        fixture+' odometry accepted')
            for label in (('local', 'ahead') if fixture in ('ray', 'voxel') else ('local',)):
                kind = PlannerSrv if label == 'local' else PlanAhead
                request = kind.Request()
                request.header.frame_id, request.bound_mode = 'map', 3
                root = [-2., 0., 0.] if label == 'local' else [-2.25, 0., 0.]
                if label == 'local':
                    request.root_pose.orientation.w = 0.
                else:
                    request.root_pose = Pose()
                    request.root_pose.position.x, request.root_pose.orientation.w, request.arrival_time = root[0], 1., 1.
                before, metric_before, since = len(probe.geometry), len(probe.metrics[label]), probe.get_clock().now().nanoseconds
                response = probe.call('planner' if label == 'local' else 'planner_ahead', request, kind)
                probe.until(lambda:len(probe.metrics[label]) > metric_before, fixture+' '+label+' metrics')
                metric = probe.metrics[label][-1]
                require(metric['success'] and response.path, fixture+' '+label+' did not produce a native route')
                if label == 'local':
                    require(response.status == PlannerSrv.Response.K_FORWARD, 'Local fixture changed planning modes')
                    phase_metric = metric
                else:
                    require(response.success and not response.global_required and not response.homing_required,
                            'PlanAhead fixture refused continuation')
                    phase_metric = metric['plan']
                phase = probe.snapshot(label, before, since, revision, label)
                probe.until(lambda:probe.graphs and stamp_ns(probe.graphs[-1].header) >= since, fixture+' '+label+' graph')
                phase['graph'], phase['planning_metrics'] = graph_record(probe.graphs[-1]), metric
                reason = {'copied': 'clustered_copy', 'skipped': 'leaf_only_skipped'}.get(fixture, 'evaluated')
                phase['verification'] = verify_gain(phase['metadata'], probe.graphs[-1], phase_metric, parameters, root, reason)

            if fixture == 'ray':
                request = PlannerSrv.Request()
                request.header.frame_id, request.bound_mode, request.root_pose.orientation.w = 'odom', 3, 0.
                before, metric_before, since = len(probe.geometry), len(probe.metrics['local']), probe.get_clock().now().nanoseconds
                response = probe.call('planner', request, PlannerSrv)
                probe.until(lambda:len(probe.metrics['local']) > metric_before, 'Wrong-frame refusal metrics')
                require(not response.path and not probe.metrics['local'][-1]['success'] and
                        probe.metrics['local'][-1]['reason'] == 'wrong_frame', 'Invalid-frame request was not refused')
                probe.snapshot('failed_request', before, since, revision, empty=True)
                empty = make_map()
                data = bytearray(empty.data)
                for offset in range(12, len(data), empty.point_step):
                    data[offset] = 255
                empty.data = bytes(data)
                before, since = len(probe.geometry), probe.get_clock().now().nanoseconds
                revision = probe.publish_map(empty)
                probe.snapshot('empty_map', before, since, revision, 'clear', empty=True)
            close_fixture()
        summary['status'] = 'passed'
    except Exception as error:
        summary['error'], summary['traceback'] = str(error), traceback.format_exc()
        if probe is not None:
            probe.record['last_geometry'] = probe.geometry[-1] if probe.geometry else None
            probe.record['last_rays'] = snapshot_record(probe.rays[-1]) if probe.rays else None
            probe.record['last_samples'] = snapshot_record(probe.samples[-1]) if probe.samples else None
    finally:
        if process is not None or probe is not None:
            try:
                close_fixture()
            except Exception as error:
                summary['cleanup_error'], summary['status'] = str(error), 'failed'
        if initialized:
            try:
                rclpy.shutdown()
            except Exception as error:
                summary['shutdown_error'], summary['status'] = str(error), 'failed'
        summary['wall_time_s'] = time.monotonic()-started
        temporary = output.with_suffix('.tmp')
        temporary.write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
        temporary.replace(output)
        print(json.dumps({'status': summary['status'], 'output': str(output),
                          'error': summary.get('error'), 'wall_time_s': summary['wall_time_s']}, ensure_ascii=False))
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
