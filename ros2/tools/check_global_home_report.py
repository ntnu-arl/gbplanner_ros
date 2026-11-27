#!/usr/bin/env python3
"""Check a fresh home report after native aerial current-state attachment.

One real ROS node reloads the production core's U-shaped four-vertex snapshot
before each explicit/automatic global selection. Attaching the measured middle
point creates a shorter return route. No second planning call may repair a
failed first call; there is no controller, motion or simulated vehicle.
"""
import argparse
import hashlib
import heapq
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
import traceback


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def xyz(point):
    return [point.x, point.y, point.z]


def shortest(graph, source, target):
    adjacency = {vertex[0]: [] for vertex in graph['vertices']}
    for a, b, weight in graph['edges']:
        adjacency[a].append((b, weight)); adjacency[b].append((a, weight))
    distances, queue = {source: 0.}, [(0., source)]
    while queue:
        distance, vertex = heapq.heappop(queue)
        if distance > distances[vertex]:
            continue
        if vertex == target:
            return distance
        for neighbor, weight in adjacency[vertex]:
            candidate = distance + weight
            if candidate < distances.get(neighbor, math.inf):
                distances[neighbor] = candidate
                heapq.heappush(queue, (candidate, neighbor))
    return math.inf


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output, seed = args.output.resolve(), args.snapshot.resolve()
    log_path, parameters_path = output.with_suffix('.log'), output.with_suffix('.yaml')
    require(seed.is_file(), 'Production-core seed snapshot is required')
    require(not any(path.exists() for path in (output, log_path, parameters_path)), 'Use a new output name')
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100 + os.getpid() % 80))
    started, deadline = time.monotonic(), time.monotonic() + 30.
    report = {'status': 'failed', 'scope': __doc__, 'cases': {},
              'seed_sha256': hashlib.sha256(seed.read_bytes()).hexdigest()}
    process = node = None
    initialized = False
    try:
        import rclpy
        from nav_msgs.msg import Odometry, Path as RosPath
        from planner_msgs.msg import Graph
        from planner_msgs.srv import PlannerGlobal, PlannerStringTrigger
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import PointCloud2, PointField
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        rclpy.init(args=[]); initialized = True
        namespace = '/global_home_report_' + str(os.getpid())
        node = Node('probe', namespace=namespace)
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        clocks = node.create_publisher(Clock, namespace + '/clock', 10)
        maps = node.create_publisher(PointCloud2, 'voxel_map', durable)
        odometry = node.create_publisher(Odometry, 'odometry', 10)
        metrics, paths, clients = [], [], {}
        node.create_subscription(String, 'global_metrics', lambda m: metrics.append(json.loads(m.data)), durable)
        node.create_subscription(RosPath, 'global_path', paths.append, durable)
        clock = Clock(); clock.clock.sec = 100
        node.create_timer(.02, lambda: clocks.publish(clock))

        def until(predicate, label):
            while not predicate():
                require(process is None or process.poll() is None, 'Planner exited during ' + label)
                if time.monotonic() >= deadline:
                    raise TimeoutError(label)
                rclpy.spin_once(node, timeout_sec=.01)

        def call(name, request, kind=Trigger):
            if name not in clients:
                clients[name] = node.create_client(kind, name)
            client = clients[name]
            until(client.service_is_ready, name + ' discovery')
            future = client.call_async(request)
            until(future.done, name + ' response')
            return future.result()

        def status(name):
            response = call(name, Trigger.Request())
            return json.loads(response.message)

        def snapshot():
            graphs, histories = [], []
            subscriptions = [node.create_subscription(Graph, 'global_graph', graphs.append, durable),
                node.create_subscription(String, 'state_history', lambda m: histories.append(json.loads(m.data)), durable)]
            try:
                until(lambda: bool(graphs and histories), 'Late graph/history snapshot')
                graph = graphs[-1]
                return {'graph': {'vertices': [[v.id, *xyz(v.pose.position), v.is_frontier] for v in graph.vertices],
                                  'edges': [[e.source_id, e.target_id, e.weight] for e in graph.edges]},
                        'history': histories[-1]}
            finally:
                for subscription in subscriptions:
                    node.destroy_subscription(subscription)

        resolution, origin, dimensions = .1, (-2., -2., -1.), (50, 50, 20)
        options = {'use_sim_time': True, 'map.resolution': resolution, 'map.origin': list(origin),
            'map.dimensions': list(dimensions), 'robot.type': 'kAerial', 'robot.size': [.1, .1, .1],
            'global.enable': True, 'global.auto_home': False, 'global.background_enable': False,
            'planning.auto_global_planner_enable': False, 'planning.path_safety_enhance_enable': False,
            'planning.edge_min': .15, 'planning.edge_max': .8, 'planning.neighbor_radius': .8,
            'planning.neighbor_max': .8, 'planning.overshoot': 0., 'planning.graph_mode': True,
            'sensor.range': 2., 'sensor.horizontal_fov': 2 * math.pi, 'sensor.vertical_fov': .5,
            'sensor.horizontal_resolution': .4, 'sensor.vertical_resolution': .25,
            'gain.frontier_threshold': .1, 'gain.unknown': 1., 'gain.free': 0., 'gain.occupied': 0.,
            'mission.enabled': True, 'mission.auto_homing': True, 'mission.time_limit': 22.5,
            'mission.homing_speed': 1., 'mission.auto_landing_enable': False}
        parameters_path.write_text(json.dumps({'/**': {'ros__parameters': options}}, indent=2) + '\n')
        report['parameters'] = options
        command = ['ros2', 'run', 'gbplanner_local', 'local_planner_node', '--ros-args',
                   '-r', '__ns:=' + namespace, '-r', '/clock:=' + namespace + '/clock',
                   '--params-file', str(parameters_path)]
        report['command'] = command
        with log_path.open('x') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        until(lambda: maps.get_subscription_count() and odometry.get_subscription_count(), 'Inputs')
        until(lambda: status('mission_state')['stamp_s'] == 100., 'Paused ROS clock')
        cloud = PointCloud2()
        cloud.header.frame_id, cloud.header.stamp = 'map', clock.clock
        cloud.height, cloud.width, cloud.point_step = 1, math.prod(dimensions), 16
        cloud.row_step, cloud.is_dense = cloud.width * 16, True
        cloud.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                        for i, name in enumerate(('x', 'y', 'z'))]
        cloud.fields.append(PointField(name='occupancy', offset=12, datatype=PointField.INT8, count=1))
        data = bytearray(cloud.row_step); offset = 0
        for z in range(dimensions[2]):
            for y in range(dimensions[1]):
                for x in range(dimensions[0]):
                    point = [base + (index + .5) * resolution for base, index in zip(origin, (x, y, z))]
                    known = -.25 < point[0] < 1.25 and -.25 < point[1] < 1.25 and abs(point[2]) < .25
                    struct.pack_into('<fffbxxx', data, offset, *point, 0 if known else -1); offset += 16
        cloud.data = bytes(data)
        report['scene_sha256'] = hashlib.sha256(data).hexdigest()
        maps.publish(cloud)
        until(lambda: status('map_status')['known_cells'] > 0, 'Known free square')
        odom = Odometry(); odom.header.frame_id, odom.header.stamp = 'map', clock.clock
        odom.pose.pose.position.x, odom.pose.pose.orientation.w = .5, 1.
        odometry.publish(odom)

        def observed_root():
            response = call('sensor_gain', Trigger.Request())
            return response.success and json.loads(response.message)['root'] == [.5, 0., 0., 0.]

        until(observed_root, 'Measured middle point')
        report['mission'] = mission = status('mission_state')
        require(mission['valid'] and mission['remaining_s'] == 22.5 and mission['elapsed_s'] == 0.,
                'Frozen mission budget differs')
        for label, target in (('explicit', 3), ('automatic', 0)):
            storage = PlannerStringTrigger.Request(message=str(seed))
            require(call('gbplanner/load_graph', storage, PlannerStringTrigger).success, 'Seed graph load failed')
            before = snapshot()
            graph = before['graph']
            require(len(graph['vertices']) == 4 and len(graph['edges']) == 3 and
                    sum(v[-1] for v in graph['vertices']) == 1 and graph['vertices'][3][-1],
                    'Seed graph must retain its single real sensor-gain frontier')
            old_home = shortest(graph, 3, 0)
            require(old_home == 3., 'Seed return distance differs from the original U-shaped chain')
            request = PlannerGlobal.Request(id=target, ignore_time=False, not_check_frontier=False, bound_mode=3)
            metric_count, path_count = len(metrics), len(paths)
            response = call('planner_global', request, PlannerGlobal)
            until(lambda: len(metrics) > metric_count and len(paths) > path_count, 'Global outputs')
            route = [xyz(p.position) for p in response.path]
            after = snapshot()
            record = {'before': before, 'after': after, 'path': route, 'metrics': metrics[-1]}
            report['cases'][label] = record
            require(metrics[-1]['success'] and metrics[-1]['target'] == 3 and not response.homing and route,
                    'First global request incorrectly rejected the newly feasible route')
            require(route[0] == [.5, 0., 0.] and route[-1] == [1., 0., 0.], 'Outbound endpoints differ')
            require([xyz(p.pose.position) for p in paths[-1].poses] == route, 'Published route differs')
            outward = sum(math.dist(a, b) for a, b in zip(route, route[1:]))
            home_distance = shortest(after['graph'], 3, 0)
            require(abs(outward - .5) < 1e-8 and home_distance == 1., 'Independent post-connection costs differ')
            require(before['history'] == after['history'], 'Global selection fabricated a visit')
            require(len(after['graph']['vertices']) == 5 and len(after['graph']['edges']) == 5,
                    'Current attachment did not create exactly two short links')
            require(status('mission_state')['remaining_s'] == 22.5, 'ROS mission time advanced')
            record.update(old_return_m=old_home, actual_return_m=home_distance, outbound_m=outward,
                          old_spare_s=22.5-outward-old_home, actual_spare_s=22.5-outward-home_distance)
            require(record['old_spare_s'] < 20. <= record['actual_spare_s'], 'Fixture does not distinguish stale budget')
        report['status'] = 'passed'
    except Exception as error:
        report['error'], report['traceback'] = str(error), traceback.format_exc()
    finally:
        if process is not None:
            forced = False
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try: process.wait(timeout=4.)
                except subprocess.TimeoutExpired:
                    forced = True; os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=2.)
            report['cleanup'] = {'exit_code': process.returncode, 'forced': forced}
            if forced or process.returncode != 0: report['status'] = 'failed'
        if node is not None: node.destroy_node()
        if initialized: rclpy.shutdown()
        report['elapsed_s'] = time.monotonic() - started
        sources = ['src/gbplanner_local/src/global_memory.cpp', 'src/gbplanner_local/src/local_planner_node.cpp',
                   'tools/check_global_home_report.py', 'src/gbplanner_local/src/global_home_report_demo.cpp']
        report['source_sha256'] = {p: hashlib.sha256((workspace/p).read_bytes()).hexdigest() for p in sources}
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'status': report['status'], 'output': str(output), 'elapsed_s': report['elapsed_s'],
                      'error': report.get('error')}))
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
