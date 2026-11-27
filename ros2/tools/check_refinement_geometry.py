#!/usr/bin/env python3
"""Check native refinement geometry on the existing synthetic doorway map.

Local planning, PlanAhead, and an explicitly refined direct search execute real
planner services. Only synthetic map and odometry messages are supplied; no
trajectory consumer or robot motion is started.
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
import traceback

sys.dont_write_bytecode = True
from check_planning_visualization import close, close_points, require, snapshot_record, stamp_ns, xyz


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def point_cloud_record(message):
    require(message.header.frame_id == 'map' and not message.is_bigendian,
            'Refinement point cloud has an unexpected frame or byte order')
    fields = {field.name: field for field in message.fields}
    require(set(fields) == {'x', 'y', 'z'}, 'Unexpected refinement cloud fields')
    require(all(fields[name].datatype == 7 and fields[name].count == 1 for name in fields),
            'Refinement cloud fields must be scalar float32')
    require(message.row_step >= message.width*message.point_step and
            len(message.data) == message.row_step*message.height, 'Invalid point cloud row layout')
    require(all(fields[name].offset+4 <= message.point_step for name in fields), 'Point cloud field exceeds point stride')
    points = []
    for row in range(message.height):
        for column in range(message.width):
            offset = row*message.row_step+column*message.point_step
            point = [struct.unpack_from('<f', message.data, offset+fields[name].offset)[0] for name in ('x', 'y', 'z')]
            require(all(math.isfinite(value) for value in point), 'Refinement cloud contains nonfinite geometry')
            points.append(point)
    return {'frame': message.header.frame_id, 'stamp_ns': stamp_ns(message.header),
            'width': message.width, 'height': message.height, 'point_step': message.point_step,
            'xyz': points}


def verify_attempts(geometry, safety_extension, occupancy, response_path):
    require(geometry['captured'], 'Refinement geometry was not captured')
    attempts = geometry['attempts']
    require(geometry['attempt_count'] == len(attempts)+geometry['dropped_attempts'], 'Attempt count mismatch')
    require(geometry['plane_count'] == sum(len(attempt['planes']) for attempt in attempts)+geometry['dropped_planes'],
            'Plane count mismatch')
    require(geometry['free_point_count'] == sum(len(attempt['free_points']) for attempt in attempts)+
            geometry['dropped_free_points'], 'Free-point count mismatch')
    require(len(attempts) <= 64 and sum(len(attempt['planes']) for attempt in attempts) <= 1024 and
            sum(len(attempt['free_points']) for attempt in attempts) <= 8192, 'Capture exceeds its geometry limits')
    require(not geometry['truncated'] and all(geometry[field] == 0 for field in
            ('dropped_attempts', 'dropped_planes', 'dropped_free_points')), 'Small doorway fixture unexpectedly truncated geometry')
    require(geometry['path_accepted'], 'Refinement did not accept the service path')
    returned = [xyz(pose.position) for pose in response_path]
    require(len(returned) >= 2, 'Refined service path is empty')
    checks, nonempty, tangent_count = [], 0, 0
    for attempt in attempts:
        start, target, center = attempt['start'], attempt['target'], attempt['center']
        for name in ('start', 'target', 'center', 'adjusted', 'centroid'):
            require(len(attempt[name]) == 3 and all(math.isfinite(value) for value in attempt[name]),
                    'Invalid refinement '+name)
        require(len(attempt['rotation']) == 9 and all(math.isfinite(value) for value in attempt['rotation']),
                'Invalid refinement rotation')
        rotation = [attempt['rotation'][row*3:row*3+3] for row in range(3)]
        columns = [[rotation[row][column] for row in range(3)] for column in range(3)]
        for i in range(3):
            for j in range(3):
                require(abs(dot(columns[i], columns[j])-(1. if i == j else 0.)) < 1.e-8,
                        'Refinement rotation is not orthonormal')
        radius = math.dist(start, target)/2
        if attempt['modify_attempted'] and radius > 1.e-8:
            close(center, [(a+b)/2 for a, b in zip(start, target)], 'Segment midpoint')
            close(columns[0], [(b-a)/(2*radius) for a, b in zip(start, target)], 'Corridor longitudinal direction')
        planes, free_points = attempt['planes'], attempt['free_points']
        for plane in planes:
            normal, offset = plane['normal'], plane['offset']
            require(len(normal) == 3 and all(math.isfinite(value) for value in normal+[offset]) and dot(normal, normal) > 0.,
                    'Invalid raw corridor plane')
            if plane['obstacle']:
                obstacle = plane['tangent']
                require(len(obstacle) == 3 and occupancy(obstacle) == 100, 'Tangent point is not an occupied map sample')
                require(math.isclose(dot(normal, obstacle), offset, rel_tol=1.e-7, abs_tol=1.e-8),
                        'Tangent plane does not pass through its actual obstacle sample')
                tangent_count += 1
        if free_points:
            nonempty += 1
            require(attempt['modify_attempted'] and attempt['modify_succeeded'] and attempt['centroid_valid'],
                    'Nonempty feasible cross-section has incomplete algorithm diagnostics')
            require(len(planes) >= 6, 'Feasible cross-section is missing its box planes')
            half = [radius+safety_extension[0], safety_extension[1], safety_extension[2]]
            for index, plane in enumerate(planes[:6]):
                axis, sign = index//2, -1. if index%2 == 0 else 1.
                expected = [value*sign/half[axis] for value in columns[axis]]
                require(not plane['obstacle'], 'Initial corridor box plane is marked as obstacle tangent')
                close(plane['normal'], expected, 'Unnormalized corridor box normal')
                require(abs(plane['offset']-(1+dot(expected, center))) < 1.e-8, 'World-space box plane offset differs')
            for point in free_points:
                require(len(point) == 3 and all(math.isfinite(value) for value in point), 'Invalid feasible point')
                require(occupancy(point) == 0, 'Feasible point is not in observed free space')
                delta = [value-origin for value, origin in zip(point, center)]
                local = [dot(column, delta) for column in columns]
                require(abs(local[0]-radius) < 1.e-8, 'Feasible point is not on the target cross-section')
                require(all(abs(local[axis]) <= safety_extension[axis]+1.e-8 for axis in (1, 2)),
                        'Feasible point leaves its configured cross-section bounds')
                for plane in planes:
                    require(dot(plane['normal'], point)-plane['offset'] <= -.05+1.e-8,
                            'Feasible point violates the original unnormalized .05 plane margin')
            centroid = [sum(point[axis] for point in free_points)/len(free_points) for axis in range(3)]
            close(attempt['centroid'], centroid, 'Mean of actual feasible cross-section samples')
            close(attempt['adjusted'], centroid, 'Algorithm adjustment versus sample centroid')
        if attempt['locally_accepted']:
            require(any(math.dist(attempt['adjusted'], point) < 1.e-8 for point in returned),
                    'Accepted adjusted waypoint is absent from the returned service path')
        checks.append({'segment_index': attempt['segment_index'], 'free_points': len(free_points),
                       'planes': len(planes), 'locally_accepted': attempt['locally_accepted'], 'reason': attempt['reason']})
    return {'attempts': checks, 'nonempty_cross_sections': nonempty, 'tangent_planes': tangent_count,
            'returned_xyz': returned}


def verify_displays(metadata, cloud_message, marker_message, marker_type):
    cloud = point_cloud_record(cloud_message)
    require(cloud['stamp_ns'] == metadata['stamp_ns'], 'Cloud and JSON refer to different refinement snapshots')
    attempts = metadata['geometry']['attempts']
    expected = [point for attempt in attempts for point in attempt['free_points']]
    # Compare exactly with the prescribed double-to-float32 wire conversion.
    rounded = [[struct.unpack('<f', struct.pack('<f', value))[0] for value in point] for point in expected]
    require(cloud['xyz'] == rounded, 'Point cloud differs from actual feasible samples or their capture order')
    require(marker_message.markers and marker_message.markers[0].action == marker_type.DELETEALL,
            'Refinement markers are not a complete DELETEALL snapshot')
    added = {}
    for index, marker in enumerate(marker_message.markers):
        require(marker.header.frame_id == 'map' and stamp_ns(marker.header) == metadata['stamp_ns'],
                'Refinement marker has wrong frame or snapshot timestamp')
        if index == 0:
            continue
        require(marker.action == marker_type.ADD and marker.color.a > 0., 'Invalid refinement marker action or opacity')
        identity = marker.ns, marker.id
        require(identity not in added, 'Repeated refinement marker identity')
        added[identity] = marker
    identities, tangent_points = set(), []
    planes = [plane for attempt in attempts for plane in attempt['planes']]
    for identifier, plane in enumerate(planes):
        normal, tangent = plane['normal'], plane['tangent']
        norm = math.sqrt(dot(normal, normal))
        require(norm > 0. and math.isfinite(norm), 'Invalid captured plane normal')
        unit = [value/norm for value in normal]
        cube_key, arrow_key = ('refinement_planes', identifier), ('refinement_normals', identifier)
        identities.update((cube_key, arrow_key))
        require(cube_key in added and arrow_key in added, 'Captured plane is missing its patch or normal arrow')
        cube, arrow = added[cube_key], added[arrow_key]
        require(cube.type == marker_type.CUBE and arrow.type == marker_type.ARROW, 'Incorrect plane display marker type')
        close(xyz(cube.pose.position), tangent, 'Plane patch at actual tangent')
        close(xyz(cube.scale), [.02, .75, .75], 'Plane patch dimensions')
        q = cube.pose.orientation
        require(abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) < 1.e-8, 'Plane patch quaternion is not normalized')
        rotated_x = [1-2*(q.y*q.y+q.z*q.z), 2*(q.x*q.y+q.w*q.z), 2*(q.x*q.z-q.w*q.y)]
        close(rotated_x, unit, 'Plane patch normal orientation')
        close_points([xyz(point) for point in arrow.points],
                     [tangent, [value+.35*direction for value, direction in zip(tangent, unit)]],
                     'Raw plane normal shown as a .35 m arrow')
        close(xyz(arrow.scale), [.025, .055, .08], 'Normal arrow dimensions')
        tangent_points.append(tangent)
    if tangent_points:
        identity = 'refinement_tangents', 0
        identities.add(identity)
        require(identity in added and added[identity].type == marker_type.CUBE_LIST, 'Tangent sample list is missing')
        close_points([xyz(point) for point in added[identity].points], tangent_points, 'Actual plane tangent samples')
        close(xyz(added[identity].scale), [.06]*3, 'Tangent sample dimensions')
    for index, attempt in enumerate(attempts):
        if not attempt['centroid_valid']:
            continue
        identity = 'refinement_centroids', index
        identities.add(identity)
        require(identity in added and added[identity].type == marker_type.SPHERE, 'Computed centroid marker is missing')
        close(xyz(added[identity].pose.position), attempt['centroid'], 'Actual feasible-sample centroid')
        close(xyz(added[identity].scale), [.10]*3, 'Centroid marker dimensions')
    require(set(added) == identities, 'Display snapshot contains stale or unrelated refinement geometry')
    return cloud


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=workspace/'results/refinement_geometry/summary.json')
    parser.add_argument('--timeout', type=float, default=150.)
    args = parser.parse_args()
    require(math.isfinite(args.timeout) and args.timeout > 0., 'Timeout must be positive')
    output, log_path = args.output.resolve(), args.output.resolve().with_suffix('.planner.log')
    require(not output.exists() and not log_path.exists(), 'Refusing to overwrite existing results or planner log')
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100+os.getpid()%80))
    started, initialized, probe, process = time.monotonic(), False, None, None
    deadline = started+args.timeout
    summary = {'status': 'failed', 'scope': __doc__, 'domain': os.environ['ROS_DOMAIN_ID'],
               'phases': {}, 'checks': {}, 'cleanup': {}, 'log': str(log_path)}
    try:
        import rclpy
        from geometry_msgs.msg import Pose
        from nav_msgs.msg import Odometry, Path as RosPath
        from planner_msgs.srv import PlanAhead, PlannerSearch, PlannerSrv
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from visualization_msgs.msg import Marker, MarkerArray
        sys.path.insert(0, str(workspace/'src/gbplanner_local/scripts'))
        from local_demo import DIM, ORIGIN, RES, make_map, status_at

        rclpy.init(args=[])
        initialized = True
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        safety = [1., 1.5, 1.]
        parameters = {
            'map.resolution': RES, 'map.origin': list(ORIGIN), 'map.dimensions': list(DIM),
            'global.enable': True, 'global.auto_home': True, 'global.background_enable': False,
            'mission.enabled': False, 'planning.auto_global_planner_enable': False,
            'planning.go_home_if_fully_explored': False, 'visualization.planning_diagnostics': True,
            'planning.path_safety_enhance_enable': True, 'robot.safety_extension': safety,
            'planning.path_interpolation_distance': .5, 'planning.relaxed_corridor_multiplier': 1.,
            'planning.traverse_length_max': 10., 'planning.traverse_time_max': 100.,
            'planning.adaptive_exploration': False, 'robot.size': [.4, .4, .4],
            'sampling.seed': 42, 'sampling.seed_mode': 'fixed',
            'sampling.min': [-3., -3., -1.], 'sampling.max': [3., 3., 1.],
            'planning.max_vertices': 100, 'planning.max_edges': 2000, 'planning.max_attempts': 4000,
            'planning.path_length_penalty': .07, 'planning.path_direction_penalty': 0.,
            'search.refine_path': True,
        }
        summary['parameters'] = parameters
        summary['fixture'] = {'source': 'local_demo.make_map()', 'wall': True, 'unknown_x_min': 1.5,
                              'search_target': [1., 0., 0.], 'cross_section_spacing_m': .1}

        class Probe(Node):
            def __init__(self):
                super().__init__('probe', namespace=f'/refinement_geometry_{os.getpid()}')
                self.rpc_clients = {}
                self.geometry, self.clouds, self.markers = [], [], []
                self.metrics = {'local': [], 'ahead': [], 'search': []}
                self.paths = {'local': [], 'search': []}
                self.maps = self.create_publisher(PointCloud2, 'voxel_map', durable)
                self.odometry = self.create_publisher(Odometry, 'odometry', 10)
                for label, topic in (('local', 'planning_metrics'), ('ahead', 'plan_ahead_metrics'),
                                     ('search', 'search_metrics')):
                    self.create_subscription(String, topic,
                        lambda message, entries=self.metrics[label]:entries.append(json.loads(message.data)), durable)
                for label in self.paths:
                    self.create_subscription(RosPath, label+'_path', self.paths[label].append, durable)
                self.create_subscription(String, 'refinement_geometry',
                                         lambda message:self.geometry.append(json.loads(message.data)), durable)
                self.create_subscription(PointCloud2, 'vis/refinement_corridor', self.clouds.append, durable)
                self.create_subscription(MarkerArray, 'vis/refinement_hyperplanes', self.markers.append, durable)

            def until(self, predicate, label, seconds=20.):
                limit = min(deadline, time.monotonic()+seconds)
                while not predicate():
                    if process.poll() is not None:
                        raise RuntimeError(f'Planner exited {process.returncode}; see {log_path}')
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
                result = future.result()
                require(result is not None, name+': empty response')
                return result

            def status(self, name):
                return json.loads(self.call(name, Trigger.Request()).message)

            def publish_map(self, message):
                previous = self.status('map_status')['map_revision']
                self.maps.publish(message)
                self.until(lambda:self.status('map_status')['map_revision'] == previous+1, 'Map accepted')
                return previous+1

            def snapshot(self, label, before, since, request, revision, response_path=None, refinement=None):
                self.until(lambda:len(self.geometry) > before and self.geometry[-1]['request'] == request and
                    self.geometry[-1]['map_revision'] == revision and self.geometry[-1]['stamp_ns'] >= since and
                    self.clouds and self.markers and self.markers[-1].markers and
                    self.geometry[-1]['stamp_ns'] == stamp_ns(self.clouds[-1].header) ==
                    stamp_ns(self.markers[-1].markers[0].header), label+' complete refinement snapshot')
                metadata, cloud_message, marker_message = self.geometry[-1], self.clouds[-1], self.markers[-1]
                record = {'metadata': metadata, 'markers': snapshot_record(marker_message)}
                summary['phases'][label] = record
                record['cloud'] = verify_displays(metadata, cloud_message, marker_message, Marker)
                if request == 'clear':
                    require(not metadata['response_success'] and not metadata['attempted'] and
                            not metadata['geometry']['captured'] and not metadata['geometry']['path_accepted'] and
                            metadata['geometry']['attempts'] == [] and not record['cloud']['xyz'] and
                            len(marker_message.markers) == 1, 'Clear snapshot retains refinement geometry')
                    require(all(metadata['geometry'][key] == 0 for key in ('attempt_count', 'dropped_attempts',
                            'plane_count', 'dropped_planes', 'free_point_count', 'dropped_free_points')),
                            'Clear snapshot retains geometry counts')
                else:
                    require(metadata['response_success'] and metadata['attempted'], 'Service did not publish its real refinement attempt')
                    require(all(metadata[key] == refinement[key] for key in ('attempted', 'modified', 'reason')),
                            'Refinement geometry disagrees with its service metrics')
                    record['verification'] = verify_attempts(metadata['geometry'], safety, status_at, response_path)
                retained_metadata, retained_clouds, retained_markers = [], [], []
                subscriptions = [self.create_subscription(String, 'refinement_geometry',
                    lambda message:retained_metadata.append(json.loads(message.data)), durable),
                    self.create_subscription(PointCloud2, 'vis/refinement_corridor', retained_clouds.append, durable),
                    self.create_subscription(MarkerArray, 'vis/refinement_hyperplanes', retained_markers.append, durable)]
                try:
                    self.until(lambda:retained_metadata and retained_clouds and retained_markers, label+' late subscriber')
                    require(retained_metadata[-1] == metadata and point_cloud_record(retained_clouds[-1]) == record['cloud'] and
                            snapshot_record(retained_markers[-1]) == record['markers'],
                            label+': late subscriber received a stale or partial refinement snapshot')
                    record['late_subscriber_matches'] = True
                finally:
                    for subscription in subscriptions:
                        self.destroy_subscription(subscription)
                return record

        probe = Probe()
        command = ['ros2', 'run', 'gbplanner_local', 'local_planner_node', '--ros-args',
                   '-r', '__ns:='+probe.get_namespace()]
        for key, value in parameters.items():
            command += ['-p', key+':='+json.dumps(value)]
        summary['launch'] = command
        with log_path.open('x') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True, env=dict(os.environ))
        probe.until(lambda:probe.maps.get_subscription_count() > 0 and probe.odometry.get_subscription_count() > 0,
                    'Map and odometry discovery')
        revision = probe.publish_map(make_map())
        odometry = Odometry()
        odometry.header.frame_id, odometry.child_frame_id = 'map', 'base_link'
        odometry.header.stamp = probe.get_clock().now().to_msg()
        odometry.pose.pose.position.x, odometry.pose.pose.orientation.w = -2., 1.
        probe.odometry.publish(odometry)
        probe.until(lambda:math.dist(probe.status('sensor_gain').get('root', [1.e6]*4)[:3], [-2., 0., 0.]) < 1.e-8,
                    'Synthetic odometry accepted')
        probe.until(lambda:probe.status('global_status')['vertices'] > 0, 'Global home initialized')

        for label in ('local', 'ahead', 'search'):
            kind = {'local': PlannerSrv, 'ahead': PlanAhead, 'search': PlannerSearch}[label]
            request = kind.Request()
            request.header.frame_id, request.bound_mode = 'map', 3
            if label == 'local':
                request.root_pose.orientation.w = 0.
            elif label == 'ahead':
                request.root_pose = Pose()
                request.root_pose.position.x, request.root_pose.orientation.w, request.arrival_time = -1.5, 1., 1.
            else:
                request.use_current_state = True
                request.target.position.x, request.target.orientation.w = 1., 1.
            before, metric_before, since = len(probe.geometry), len(probe.metrics[label]), probe.get_clock().now().nanoseconds
            response = probe.call({'local': 'planner', 'ahead': 'planner_ahead', 'search': 'planner_search'}[label], request, kind)
            probe.until(lambda:len(probe.metrics[label]) > metric_before, label+' metrics')
            metric = probe.metrics[label][-1]
            require(metric['success'] and response.path, label+': fixture returned no successful path: '+str(metric))
            if label == 'local':
                require(response.status == PlannerSrv.Response.K_FORWARD, 'Local fixture changed planning modes')
            elif label == 'ahead':
                require(response.success and not response.homing_required and not response.global_required,
                        'PlanAhead fixture switched to a continuation refusal')
            else:
                require(response.success and metric['reason'] == 'direct' and metric['exact_target'],
                        'Doorway search did not use its deterministic exact direct path')
            refinement = metric['plan']['refinement'] if label == 'ahead' else metric['refinement']
            record = probe.snapshot(label, before, since, label, revision, response.path, refinement)
            record['planning_metrics'] = metric
            path_topic = 'search' if label == 'search' else 'local'
            probe.until(lambda:probe.paths[path_topic] and stamp_ns(probe.paths[path_topic][-1].header) >= since,
                        label+' path publication')
            close_points([xyz(pose.position) for pose in response.path],
                         [xyz(pose.pose.position) for pose in probe.paths[path_topic][-1].poses],
                         label+' service versus path publication')
            if label == 'search':
                require(record['verification']['nonempty_cross_sections'] > 0 and
                        record['verification']['tangent_planes'] > 0,
                        'Doorway search did not exercise occupied-wall planes and feasible cross-section samples')

        empty = make_map()
        data = bytearray(empty.data)
        for offset in range(12, len(data), empty.point_step):
            data[offset] = 255
        empty.data = bytes(data)
        before, since = len(probe.geometry), probe.get_clock().now().nanoseconds
        revision = probe.publish_map(empty)
        probe.snapshot('empty_map', before, since, 'clear', revision)
        summary['checks'] = {'real_local_ahead_and_search_refinement': True, 'plane_and_sample_geometry': True,
                             'sample_centroid_recomputed': True, 'late_subscribers_receive_complete_snapshots': True,
                             'empty_map_clears_refinement': True}
        summary['status'] = 'passed'
    except Exception as error:
        summary['error'], summary['traceback'] = str(error), traceback.format_exc()
        if probe is not None:
            summary['last_geometry'] = probe.geometry[-1] if probe.geometry else None
            summary['last_markers'] = snapshot_record(probe.markers[-1]) if probe.markers else None
            if probe.clouds:
                try:
                    summary['last_cloud'] = point_cloud_record(probe.clouds[-1])
                except Exception as decode_error:
                    summary['last_cloud_error'] = str(decode_error)
    finally:
        cleanup_errors = []
        if process is not None:
            summary['cleanup']['forced'] = False
            try:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=5.)
                    except subprocess.TimeoutExpired:
                        summary['cleanup']['forced'] = True
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=2.)
                summary['cleanup']['exit_code'] = process.returncode
                if summary['cleanup']['forced'] or process.returncode != 0:
                    cleanup_errors.append(f'Planner did not exit cleanly: {process.returncode}')
            except Exception as error:
                cleanup_errors.append(str(error))
        if probe is not None:
            try:
                probe.destroy_node()
            except Exception as error:
                cleanup_errors.append(str(error))
        if initialized:
            try:
                rclpy.shutdown()
            except Exception as error:
                cleanup_errors.append(str(error))
        if cleanup_errors:
            summary['cleanup']['errors'], summary['status'] = cleanup_errors, 'failed'
        summary['wall_time_s'] = time.monotonic()-started
        temporary = output.with_suffix('.tmp')
        temporary.write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
        temporary.replace(output)
        print(json.dumps({'status': summary['status'], 'output': str(output),
                          'error': summary.get('error'), 'wall_time_s': summary['wall_time_s']}, ensure_ascii=False))
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
