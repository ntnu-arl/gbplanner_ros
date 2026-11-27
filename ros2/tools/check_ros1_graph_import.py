#!/usr/bin/env python3
"""Official ROS 1 serialization → converter → native ROS 2 graph services.

Two small synthetic aerial/ground chain graphs exercise deferred map validation,
obstacle suspension, homing and transactional metadata rejection. No controller,
simulator or physical movement is started. genpy runs in a separate process so
its ROS 1 dynamic classes cannot shadow the ROS 2 message packages.
"""
import argparse
import hashlib
import io
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


ROS1_DEFINITION = """Header header
planner_msgs/Vertex[] vertices
planner_msgs/Edge[] edges
================================================================================
MSG: std_msgs/Header
uint32 seq
time stamp
string frame_id
================================================================================
MSG: planner_msgs/Vertex
int32 id
geometry_msgs/Pose pose
int32 num_unknown_voxels
int32 num_occupied_voxels
int32 num_free_voxels
bool is_frontier
================================================================================
MSG: geometry_msgs/Pose
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: planner_msgs/Edge
int32 source_id
int32 target_id
float32 weight
"""
RESOLUTION, ORIGIN, DIMENSIONS = .1, (-3., -1., -1.), (60, 20, 30)


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def generate_fixtures(directory):
    import genpy
    from genpy.dynamic import generate_dynamic
    classes = generate_dynamic('planner_msgs/Graph', ROS1_DEFINITION)
    directory.mkdir(parents=True, exist_ok=True)
    definition_file = directory/'ros1_graph_definition.txt'
    definition_file.write_text(ROS1_DEFINITION)
    manifest = {'generator': 'genpy.dynamic.generate_dynamic + generated Message.serialize(io.BytesIO)',
                'genpy_module': genpy.__file__, 'definition_file': str(definition_file),
                'definition_sha256': hashlib.sha256(ROS1_DEFINITION.encode()).hexdigest(),
                'message_md5': {name: message._md5sum for name, message in classes.items()}, 'fixtures': {}}
    for label, height, frame in (('aerial', 0., ''), ('ground', .8, ''), ('aerial_odom_frame', 0., 'odom')):
        graph = classes['planner_msgs/Graph']()
        graph.header.seq, graph.header.stamp, graph.header.frame_id = 17, genpy.Time(123, 456), frame
        # The upstream graph is held in unordered storage; wire order is not
        # vertex ID order. Keep the IDs/geometry while serializing 2,0,1.
        for index in (2, 0, 1):
            x = (-2., 0., 2.)[index]
            vertex = classes['planner_msgs/Vertex']()
            vertex.id, vertex.pose.position.x, vertex.pose.position.z = index, x, height
            vertex.pose.orientation.w = 1.
            vertex.num_unknown_voxels, vertex.num_occupied_voxels, vertex.num_free_voxels = 17+index, 2, 23
            vertex.is_frontier = index == 2
            graph.vertices.append(vertex)
        for first, second in ((0, 1), (1, 2)):
            edge = classes['planner_msgs/Edge']()
            edge.source_id, edge.target_id, edge.weight = first, second, 2.
            graph.edges.append(edge)
        stream = io.BytesIO()
        graph.serialize(stream)
        data = stream.getvalue()
        path = directory/(label+'.ros1')
        path.write_bytes(data)
        manifest['fixtures'][label] = {'path': str(path), 'sha256': hashlib.sha256(data).hexdigest(),
                                       'bytes': len(data), 'source_frame': frame, 'planning_z': height,
                                       'vertices': 3, 'wire_vertex_order': [2, 0, 1],
                                       'links': [[0, 1, 2.], [1, 2, 2.]], 'source_frontier_id': 2,
                                       'source_frontier_count': 1}
    (directory/'fixture_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')


def map_data(ground, blocked=False):
    data = bytearray(math.prod(DIMENSIONS)*16)
    offset = 0
    for iz in range(DIMENSIONS[2]):
        z = ORIGIN[2]+(iz+.5)*RESOLUTION
        for iy in range(DIMENSIONS[1]):
            y = ORIGIN[1]+(iy+.5)*RESOLUTION
            for ix in range(DIMENSIONS[0]):
                x = ORIGIN[0]+(ix+.5)*RESOLUTION
                occupied = (ground and z < 0.) or (blocked and abs(x) < .2)
                struct.pack_into('<fffbxxx', data, offset, x, y, z, 100 if occupied else 0)
                offset += 16
    return bytes(data)


def graph_record(graph):
    return {'vertices': sorted([{'id': vertex.id,
                                 'position': [vertex.pose.position.x, vertex.pose.position.y, vertex.pose.position.z],
                                 'orientation': [vertex.pose.orientation.x, vertex.pose.orientation.y,
                                                 vertex.pose.orientation.z, vertex.pose.orientation.w],
                                 'frontier': vertex.is_frontier} for vertex in graph.vertices], key=lambda v: v['id']),
            'edges': sorted([[edge.source_id, edge.target_id, edge.weight] for edge in graph.edges])}


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=root/'results/ros1_graph_import/runtime1.json')
    parser.add_argument('--generate-fixtures', type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.generate_fixtures is not None:
        generate_fixtures(args.generate_fixtures.resolve())
        return 0
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    artifacts = output.parent/(output.stem+'_inputs')
    os.environ.setdefault('ROS_DOMAIN_ID', str(100+os.getpid()%80))
    started = time.monotonic()
    report = {'status': 'failed', 'scope': __doc__, 'domain': os.environ['ROS_DOMAIN_ID'],
              'conversions': {}, 'runs': {}, 'checks': {}}
    initialized = False
    try:
        # A separate interpreter prevents genpy's ROS 1 imports from affecting
        # rclpy or geometry_msgs in the native service probe below.
        generation = subprocess.run([sys.executable, str(Path(__file__).resolve()), '--generate-fixtures', str(artifacts)],
                                    capture_output=True, text=True, timeout=30)
        require(generation.returncode == 0, 'Official ROS 1 fixture generation failed: '+generation.stderr)
        report['fixture_manifest'] = json.loads((artifacts/'fixture_manifest.json').read_text())

        def convert(label, source, robot, frame='map', height=None, expected=True):
            destination = artifacts/(label+'.gbmem')
            command = ['ros2', 'run', 'gbplanner_local', 'import_ros1_graph', '--input', str(artifacts/(source+'.ros1')),
                       '--output', str(destination), '--frame', frame, '--robot', robot, '--coordinates', 'planning']
            if height is not None:
                command += ['--planning-height', str(height)]
            before = destination.read_bytes() if destination.exists() else None
            result = subprocess.run(command, capture_output=True, text=True, timeout=20)
            report['conversions'][label] = {'command': command, 'exit_code': result.returncode,
                                             'stdout': result.stdout, 'stderr': result.stderr}
            require((result.returncode == 0) == expected, 'Unexpected conversion result: '+label)
            if expected:
                data = destination.read_bytes()
                require(data[:8] == b'GBP2MEM1', 'Converter did not produce an exploration snapshot')
                report['conversions'][label].update(bytes=len(data), sha256=hashlib.sha256(data).hexdigest())
                # Independently decode the CDR payload through ROS 2's own
                # serializer, rather than the converter's ROS 1 reader.
                from rclpy.serialization import deserialize_message
                from planner_msgs.msg import Graph
                frame_length = struct.unpack_from('<I', data, 8)[0]
                frame_end = 12+frame_length
                require(data[12:frame_end].decode() == frame, 'Converted snapshot frame differs')
                require(data[frame_end] == (1 if robot == 'ground' else 0), 'Converted robot metadata differs')
                if height is not None:
                    require(struct.unpack_from('<d', data, frame_end+1)[0] == height,
                            'Converted planning-height metadata differs')
                cdr_length = struct.unpack_from('<I', data, frame_end+9)[0]
                cdr_start = frame_end+13
                graph = deserialize_message(data[cdr_start:cdr_start+cdr_length], Graph)
                source_fixture = report['fixture_manifest']['fixtures'][source]
                require([vertex.id for vertex in graph.vertices] == [0, 1, 2],
                        'Converter did not normalize unordered vertex IDs')
                vertices = []
                for vertex in graph.vertices:
                    position = [vertex.pose.position.x, vertex.pose.position.y, vertex.pose.position.z]
                    orientation = [vertex.pose.orientation.x, vertex.pose.orientation.y,
                                   vertex.pose.orientation.z, vertex.pose.orientation.w]
                    counts = [vertex.num_unknown_voxels, vertex.num_occupied_voxels, vertex.num_free_voxels]
                    require(position == [(-2., 0., 2.)[vertex.id], 0., source_fixture['planning_z']]
                            and orientation == [0., 0., 0., 1.] and counts == [17+vertex.id, 2, 23],
                            'Converted CDR displaced a pose or voxel-count field')
                    require(not vertex.is_frontier, 'Converted frontier was active without map observations')
                    vertices.append({'id': vertex.id, 'position': position, 'orientation': orientation,
                                     'unknown_occupied_free_counts': counts, 'frontier': vertex.is_frontier})
                report['conversions'][label]['cdr_checked'] = {'frame': graph.header.frame_id, 'vertices': vertices}
            else:
                require((destination.read_bytes() if destination.exists() else None) == before,
                        'Failed conversion changed its destination')
            return destination

        imports = {'aerial': convert('aerial', 'aerial', 'aerial'),
                   'ground': convert('ground', 'ground', 'ground', height=.8)}
        wrong_frame = convert('wrong_frame', 'aerial_odom_frame', 'aerial', frame='odom')
        convert('rejected_frame', 'aerial_odom_frame', 'aerial', expected=False)
        wrong_height = convert('wrong_height', 'ground', 'ground', height=1.)

        import rclpy
        from nav_msgs.msg import Odometry
        from planner_msgs.msg import Graph
        from planner_msgs.srv import PlannerHoming, PlannerStringTrigger, ValidatePath
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2, PointField
        from std_srvs.srv import Trigger
        rclpy.init(args=[])
        initialized = True
        for mode in ('aerial', 'ground'):
            ground = mode == 'ground'
            namespace = f'/ros1_graph_import_{os.getpid()}_{mode}'
            node, process = None, None
            row = report['runs'][mode] = {}
            deadline = time.monotonic()+45.
            try:
                node = Node('probe', namespace=namespace)
                durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
                maps = node.create_publisher(PointCloud2, 'voxel_map', durable)
                odometry = node.create_publisher(Odometry, 'odometry', 10)
                graphs, accepted = [], []
                node.create_subscription(Graph, 'global_graph', graphs.append, durable)
                node.create_subscription(PointCloud2, 'accepted_voxel_map', accepted.append, durable)
                status = node.create_client(Trigger, 'global_status')
                storage_status = node.create_client(Trigger, 'global_graph_storage')
                sensor = node.create_client(Trigger, 'sensor_gain')
                expansion = node.create_client(Trigger, 'expansion_status')
                loader = node.create_client(PlannerStringTrigger, 'gbplanner/load_graph')
                saver = node.create_client(PlannerStringTrigger, 'gbplanner/save_graph')
                homing = node.create_client(PlannerHoming, 'exploration_homing')
                validator = node.create_client(ValidatePath, 'validate_path')
                options = {'map.resolution': RESOLUTION, 'map.origin': list(ORIGIN), 'map.dimensions': list(DIMENSIONS),
                           'robot.type': 'kGround' if ground else 'kAerial', 'robot.size': [.2, .2, .2],
                           'global.enable': True, 'global.auto_home': False, 'global.background_enable': False,
                           'planning.auto_global_planner_enable': False, 'planning.path_safety_enhance_enable': False,
                           'planning.neighbor_radius': .5, 'planning.neighbor_min': .1, 'planning.neighbor_max': .5,
                           'planning.edge_min': .1, 'planning.edge_max': 3., 'planning.overshoot': 0.,
                           'ground.max_ground_height': .8, 'ground.robot_height': .55,
                           'ground.interpolate_projection_distance': False, 'ground.ray_projection_mode': 'voxel_surface',
                           'sensor.range': .5, 'sensor.horizontal_fov': 1., 'sensor.horizontal_resolution': 1.,
                           'sensor.vertical_fov': .2, 'sensor.vertical_resolution': .2}
                launch = ['ros2', 'run', 'gbplanner_local', 'local_planner_node', '--ros-args', '-r', '__ns:='+namespace]
                for key, value in options.items():
                    launch += ['-p', key+':='+json.dumps(value)]
                log_path = output.with_name(output.stem+'_'+mode+'.log')
                row.update(parameters=options, launch=launch, log=str(log_path))
                with log_path.open('w') as log:
                    process = subprocess.Popen(launch, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

                def until(predicate, label):
                    while not predicate():
                        require(process.poll() is None, 'Native planner exited before '+label)
                        if time.monotonic() >= deadline:
                            raise TimeoutError(label)
                        rclpy.spin_once(node, timeout_sec=.02)

                def call(client, request=None):
                    until(client.service_is_ready, 'Service discovery')
                    future = client.call_async(Trigger.Request() if request is None else request)
                    until(future.done, 'Service response')
                    return future.result()

                def read(client):
                    response = call(client)
                    require(response.success, 'Status query failed: '+response.message)
                    return json.loads(response.message)

                def storage(load, path):
                    request = PlannerStringTrigger.Request()
                    request.message = str(path)
                    return call(loader if load else saver, request).success

                def snapshot():
                    current = read(status)
                    until(lambda: graphs and len(graphs[-1].vertices) == current['vertices']
                          and len(graphs[-1].edges) == current['edges'], 'Global graph publication')
                    return {'status': current, 'graph': graph_record(graphs[-1])}

                def publish_map(blocked):
                    data = map_data(ground, blocked)
                    cloud = PointCloud2()
                    cloud.header.frame_id, cloud.header.stamp = 'map', node.get_clock().now().to_msg()
                    cloud.height, cloud.width, cloud.point_step, cloud.row_step = 1, len(data)//16, 16, len(data)
                    cloud.is_dense = True
                    cloud.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                                    for i, name in enumerate(('x', 'y', 'z'))]
                    cloud.fields.append(PointField(name='occupancy', offset=12, datatype=PointField.INT8, count=1))
                    cloud.data = data
                    previous = len(accepted)
                    until(lambda: maps.get_subscription_count() > 0, 'Map subscription')
                    maps.publish(cloud)
                    until(lambda: len(accepted) > previous, 'Accepted map')
                    require(bytes(accepted[-1].data) == data, 'Accepted map differs from published fixture')
                    return hashlib.sha256(data).hexdigest()

                expected_source = [2., 0., .55 if ground else 0.]
                expected_home = [-2., 0., .55 if ground else 0.]

                def publish_odometry():
                    message = Odometry()
                    message.header.frame_id, message.header.stamp = 'map', node.get_clock().now().to_msg()
                    message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = expected_source
                    message.pose.pose.orientation.w = 1.
                    odometry.publish(message)

                def odometry_ready():
                    response = call(sensor)
                    return response.success and math.dist(json.loads(response.message)['root'],
                                                         [2., 0., .8 if ground else 0., 0.]) < 1e-8

                def home_route():
                    request = PlannerHoming.Request()
                    request.header.frame_id, request.bound_mode = 'map', 3
                    return call(homing, request).path

                def verify_route():
                    path = home_route()
                    points = [[p.position.x, p.position.y, p.position.z] for p in path]
                    require(len(points) >= 3 and math.dist(points[0], expected_source) < 1e-8
                            and math.dist(points[-1], expected_home) < 1e-8, 'Homing did not traverse imported graph to home')
                    request = ValidatePath.Request()
                    request.header.frame_id, request.bound_mode, request.path = 'map', 3, path
                    result = call(validator, request)
                    require(result.valid, 'Imported homing path failed validation: '+result.reason)
                    return {'points': points, 'validation': result.reason}

                require(storage(True, imports[mode]), 'Imported exploration snapshot failed to load')
                row['loaded_without_map'] = snapshot()
                row['storage_without_map'] = read(storage_status)
                loaded = row['loaded_without_map']['status']
                require(loaded['vertices'] == 3 and loaded['edges'] == 0 and loaded['suspended_edges'] == 2
                        and loaded['frontiers'] == 0 and row['storage_without_map']['awaiting_map'],
                        'Unobserved imported links or frontier were activated before a map')
                require(not read(expansion)['local_triggered'], 'Loading a graph granted background eligibility')
                row['free_map_sha256'] = publish_map(False)
                row['map_restored'] = snapshot()
                restored = row['map_restored']['status']
                require(restored['vertices'] == 3 and restored['edges'] == 2 and restored['suspended_edges'] == 0,
                        'Free map did not restore both original links')
                until(lambda: odometry.get_subscription_count() > 0, 'Odometry subscription')
                timer = node.create_timer(.05, publish_odometry)
                publish_odometry()
                until(odometry_ready, 'Source odometry acknowledgement')
                row['initial_homing'] = verify_route()
                before = snapshot()
                require(before['status']['vertices'] == 3 and before['status']['edges'] == 2,
                        'Homing or stationary odometry altered the imported topology')

                for label, wrong in ([('frame', wrong_frame)] if not ground else [('planning_height', wrong_height)]):
                    before_error = snapshot()
                    require(not storage(True, wrong), 'Mismatched '+label+' snapshot loaded')
                    after_error = snapshot()
                    row['rejected_'+label] = {'storage_status': read(storage_status), 'before': before_error, 'after': after_error}
                    require(after_error == before_error, 'Failed '+label+' load changed the existing graph')

                row['blocked_map_sha256'] = publish_map(True)
                row['blocked'] = snapshot()
                blocked = row['blocked']['status']
                require(blocked['vertices'] == 3 and blocked['edges'] == 0 and blocked['suspended_edges'] == 2,
                        'Obstacle did not suspend both original links')
                require(not home_route(), 'Homing used imported links through the obstacle')
                publish_map(False)
                row['recovered_homing'] = verify_route()
                recovered = snapshot()
                row['recovered'] = recovered
                require(recovered['graph'] == before['graph'] and recovered['status']['suspended_edges'] == 0,
                        'Clearing the obstacle did not recover original graph geometry and links')
                saved = artifacts/(mode+'_native_saved.gbmem')
                require(storage(False, saved), 'Native re-save failed')
                require(saved.read_bytes()[:8] == b'GBP2MEM1', 'Native save changed format')
                require(storage(True, saved), 'Native saved graph failed to reload')
                row['reloaded'] = snapshot()
                row['reloaded_homing'] = verify_route()
                require(row['reloaded']['graph'] == recovered['graph'], 'Save/load changed graph geometry or links')
                require(not read(expansion)['local_triggered'], 'Storage round trip granted background eligibility')
                row['saved_sha256'] = hashlib.sha256(saved.read_bytes()).hexdigest()
                row['status'] = 'passed'
            finally:
                if process is not None:
                    forced = False
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGINT)
                        try:
                            process.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            forced = True
                            os.killpg(process.pid, signal.SIGTERM)
                            try:
                                process.wait(timeout=5)
                            except subprocess.TimeoutExpired:
                                os.killpg(process.pid, signal.SIGKILL)
                                process.wait(timeout=5)
                    row.update(planner_exit=process.returncode, forced_termination=forced)
                    require(process.returncode == 0 and not forced, 'Native planner did not exit normally')
                if node is not None:
                    node.destroy_node()
        report['checks'] = {'official_independent_ros1_serialization': True, 'aerial_and_ground_services': True,
                            'links_suspended_until_observed_map': True, 'imported_homing_validated': True,
                            'obstacle_suspends_and_clear_map_restores_original_links': True,
                            'frame_mismatch_conversion_rejected': True, 'failed_metadata_load_keeps_graph': True,
                            'native_save_load_preserves_graph': True, 'storage_does_not_enable_background': True}
        report['status'] = 'passed'
    except Exception as error:
        report['error'], report['traceback'] = str(error), traceback.format_exc()
    finally:
        if initialized:
            rclpy.shutdown()
        report['elapsed_s'] = time.monotonic()-started
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'status': report['status'], 'output': str(output), 'error': report.get('error')}))
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
