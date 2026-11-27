#!/usr/bin/env python3
"""Check native ground background sampling with hanging and supported terrain.

Publishes synthetic 3D voxel observations and stationary base odometry, then
uses the real local planner and explicit expansion service. No motion consumer
is present. The timer is disabled so only the requested expansions alter the
background counters; the time budget retains its upstream soft semantics.
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
import time
import traceback


RESOLUTION = .1
ORIGIN = (-4., -3., -1.)
DIMENSIONS = (120, 60, 40)
BASE_ROOT = (-.25, 0., .55)
PLANNING_ROOT = (-.25, 0., .8, 0.)
SAMPLE_OFFSET = (6., 0., 0.)
ROBOT_SIZE = (.3, .3, .3)


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def scene_data(supported):
    """ground_hanging_demo's free corridor with unknown support beyond x=0.

    Extend its positive-X boundary by 2 m to keep the 6 m sample and all five
    terrain rays inside the map. The second scene changes only unknown support
    cells into occupied floor; the complete body corridor remains observed free.
    """
    data = bytearray(math.prod(DIMENSIONS)*16)
    offset = 0
    counts = {'free': 0, 'occupied': 0, 'unknown': 0}
    for iz in range(DIMENSIONS[2]):
        z = ORIGIN[2]+(iz+.5)*RESOLUTION
        for iy in range(DIMENSIONS[1]):
            y = ORIGIN[1]+(iy+.5)*RESOLUTION
            for ix in range(DIMENSIONS[0]):
                x = ORIGIN[0]+(ix+.5)*RESOLUTION
                occupancy = 0 if z >= 0. else 100 if supported or x < 0. else -1
                struct.pack_into('<fffbxxx', data, offset, x, y, z, occupancy)
                counts[{0: 'free', 100: 'occupied', -1: 'unknown'}[occupancy]] += 1
                offset += 16
    return bytes(data), counts


def graph_record(graph):
    return {
        'vertices': sorted([{'id': vertex.id,
                             'position': [vertex.pose.position.x, vertex.pose.position.y, vertex.pose.position.z],
                             'frontier': vertex.is_frontier} for vertex in graph.vertices], key=lambda item: item['id']),
        'edges': sorted([[edge.source_id, edge.target_id, edge.weight] for edge in graph.edges]),
    }


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=workspace/'results/ground_background/native.json')
    parser.add_argument('--time-budget', type=float, default=.03)
    args = parser.parse_args()
    if not math.isfinite(args.time_budget) or args.time_budget <= 0.:
        parser.error('--time-budget must be finite and positive')
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100+os.getpid()%80))
    namespace = f'/ground_background_{os.getpid()}'
    started = time.monotonic()
    deadline = started+60.
    process = node = None
    initialized = False
    summary = {'status': 'failed', 'scope': __doc__, 'checks': {},
               'scene': {'resolution': RESOLUTION, 'origin': ORIGIN, 'dimensions': DIMENSIONS,
                         'base_root': BASE_ROOT, 'planning_root': PLANNING_ROOT,
                         'sample_offset': SAMPLE_OFFSET, 'robot_size': ROBOT_SIZE,
                         'unknown_support_x_min': 0., 'floor_surface_z': 0.,
                         'sparse_radius_m': 5., 'cluster_radius_m': 10.}}
    try:
        import rclpy
        from nav_msgs.msg import Odometry
        from planner_msgs.msg import Graph
        from planner_msgs.srv import PlannerSrv
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2, PointField
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        rclpy.init(args=[])
        initialized = True
        node = Node('probe', namespace=namespace)
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        maps = node.create_publisher(PointCloud2, 'voxel_map', durable)
        odometry = node.create_publisher(Odometry, 'odometry', 10)
        accepted, metrics, graphs = [], [], []
        node.create_subscription(PointCloud2, 'accepted_voxel_map', accepted.append, durable)
        node.create_subscription(String, 'planning_metrics',
                                 lambda message: metrics.append(json.loads(message.data)), durable)
        node.create_subscription(Graph, 'global_graph', graphs.append, durable)
        planner = node.create_client(PlannerSrv, 'gbplanner')
        expand = node.create_client(Trigger, 'expand_global')
        expansion_status = node.create_client(Trigger, 'expansion_status')
        global_status = node.create_client(Trigger, 'global_status')
        sensor_status = node.create_client(Trigger, 'sensor_gain')
        options = {
            'map.resolution': RESOLUTION, 'map.origin': list(ORIGIN), 'map.dimensions': list(DIMENSIONS),
            'robot.type': 'kGround', 'robot.size': list(ROBOT_SIZE),
            'ground.max_ground_height': .8, 'ground.robot_height': .55,
            'ground.max_inclination': .85, 'ground.max_negative_inclination': .85,
            'ground.interpolate_projection_distance': False, 'ground.ray_projection_mode': 'voxel_surface',
            'global.enable': True, 'global.auto_home': True, 'global.background_enable': False,
            'global.background_budget_mode': 'time', 'global.background_time_budget': args.time_budget,
            'planning.auto_global_planner_enable': False, 'planning.graph_mode': False,
            'planning.sampling_budget_mode': 'raw_draws', 'planning.max_attempts': 1,
            'planning.max_vertices': 2, 'planning.max_edges': 100,
            'planning.edge_min': .05, 'planning.edge_max': 1., 'planning.overshoot': 0.,
            'planning.path_safety_enhance_enable': False,
            'sampling.min': [-.2, -1.5, -.1], 'sampling.max': [6.5, 1.5, .3],
            # A genuine local request must enable background work without
            # importing a local frontier or reference path into the test graph.
            'gain.unknown': 0., 'gain.free': 0., 'gain.occupied': 0.,
            'sensor.range': .5, 'sensor.horizontal_fov': 1., 'sensor.horizontal_resolution': 1.,
            'sensor.vertical_fov': .2, 'sensor.vertical_resolution': .2,
        }
        for axis in ('x', 'y', 'z', 'heading'):
            options[f'sampling.{axis}.distribution'] = 'constant'
            options[f'sampling.{axis}.constant'] = 6. if axis == 'x' else 0.
        launch = ['ros2', 'run', 'gbplanner_local', 'local_planner_node', '--ros-args', '-r', '__ns:='+namespace]
        for key, value in options.items():
            launch += ['-p', key+':='+json.dumps(value)]
        summary['parameters'], summary['launch'] = options, launch
        log_path = output.with_suffix('.log')
        summary['log'] = str(log_path)
        with log_path.open('w') as log:
            process = subprocess.Popen(launch, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True, env=dict(os.environ))

        def until(predicate, label):
            while not predicate():
                if process.poll() is not None:
                    raise RuntimeError(f'Native planner exited; see {log_path}')
                if time.monotonic() > deadline:
                    raise TimeoutError(label)
                rclpy.spin_once(node, timeout_sec=.03)

        def call(client, request=None):
            until(client.service_is_ready, 'Service discovery')
            future = client.call_async(Trigger.Request() if request is None else request)
            until(future.done, 'Service response')
            return future.result()

        def read(client):
            response = call(client)
            require(response.success, 'Status service failed: '+response.message)
            return json.loads(response.message)

        def snapshot():
            status = read(global_status)
            until(lambda: graphs and len(graphs[-1].vertices) == status['vertices']
                  and len(graphs[-1].edges) == status['edges'], 'Global graph snapshot')
            return {'status': status, 'graph': graph_record(graphs[-1])}

        def publish_scene(supported):
            data, counts = scene_data(supported)
            cloud = PointCloud2()
            cloud.header.frame_id = 'map'
            cloud.header.stamp = node.get_clock().now().to_msg()
            cloud.height, cloud.width = 1, math.prod(DIMENSIONS)
            cloud.point_step, cloud.row_step = 16, len(data)
            cloud.is_dense, cloud.is_bigendian = True, False
            cloud.fields = [PointField(name=name, offset=index*4, datatype=PointField.FLOAT32, count=1)
                            for index, name in enumerate(('x', 'y', 'z'))]
            cloud.fields.append(PointField(name='occupancy', offset=12, datatype=PointField.INT8, count=1))
            cloud.data = data
            first = len(accepted)
            maps.publish(cloud)
            until(lambda: len(accepted) > first, 'Voxel snapshot acceptance')
            require(bytes(accepted[-1].data) == data, 'Accepted map differs from the published terrain')
            return {'voxel_counts': counts, 'sha256': hashlib.sha256(data).hexdigest()}

        def publish_odometry():
            observed = Odometry()
            observed.header.frame_id = 'map'
            observed.header.stamp = node.get_clock().now().to_msg()
            observed.pose.pose.position.x, observed.pose.pose.position.y, observed.pose.pose.position.z = BASE_ROOT
            observed.pose.pose.orientation.w = 1.
            odometry.publish(observed)

        def odometry_ready():
            response = call(sensor_status)
            if not response.success:
                return False
            state = json.loads(response.message)
            return 'root' in state and math.dist(state['root'], PLANNING_ROOT) < 1e-8

        def explicit_expansion():
            previous = read(expansion_status)
            response = call(expand)
            require(response.success, 'Explicit background expansion was not enabled: '+response.message)
            stats = json.loads(response.message)
            require(stats == read(expansion_status), 'Expansion response and stored status disagree')
            require(not stats['timer_enabled'] and stats['local_triggered'] and stats['budget_mode'] == 'time',
                    'Background mode or local trigger does not match the fixture')
            require(stats['updates'] == previous['updates']+1, 'Expansion ran more than once without a timer')
            require(math.isclose(stats['time_budget_seconds'], args.time_budget), 'Time-budget parameter was not applied')
            require(stats['rounds'] > 0 and stats['sampling_batches'] >= stats['rounds'], 'No complete sampling round ran')
            require(stats['time_budget_reached'] and stats['elapsed_seconds'] >= args.time_budget
                    and stats['stop_reason'] == 'time_budget', 'Expansion did not finish at its soft time budget')
            return stats

        until(lambda: maps.get_subscription_count() > 0 and planner.service_is_ready(), 'Planner discovery')
        require(not call(expand).success, 'Expansion succeeded before map, odometry and local trigger')
        hanging_scene = publish_scene(False)
        until(lambda: odometry.get_subscription_count() > 0, 'Odometry discovery')
        publish_odometry()
        node.create_timer(.1, publish_odometry)
        until(odometry_ready, 'Stationary ground odometry acceptance')
        require(not call(expand).success, 'Expansion succeeded before a genuine local planning request')
        request = PlannerSrv.Request()
        request.header.frame_id, request.bound_mode = 'map', PlannerSrv.Request.K_EXACT_BOUND
        # The all-zero pose is the existing service's live-odometry sentinel.
        request.root_pose.orientation.w = 0.
        first = len(metrics)
        local = call(planner, request)
        until(lambda: len(metrics) > first, 'Local planning metrics')
        metric = metrics[-1]
        summary['local_trigger'] = {'metric': metric, 'response_status': local.status, 'path_points': len(local.path)}
        require(metric['attempts'] == 1 and metric['reason'] in ('no_gain', 'no_feasible_path'),
                'The local trigger did not execute the configured ground sampling request')
        require(read(expansion_status)['local_triggered'], 'Live local request did not enable background expansion')
        before = snapshot()
        # Keep the home vertex unvisited: its planning pose is <1 m from E1's
        # initial world-origin marker. No motion or extra seed graph is needed.
        require(before['status']['vertices'] == 1 and before['status']['edges'] == 0,
                'Local planning unexpectedly enlarged the seed global graph')
        hanging = explicit_expansion()
        after = snapshot()
        summary['hanging'] = {'scene': hanging_scene, 'expansion': hanging, 'before': before, 'after': after}
        require(hanging['accepted_samples'] == hanging['hanging_rejections'] > 0,
                'Free-body hanging samples were not accepted and then rejected as hanging')
        require(hanging['attempts'] == hanging['sampling_batches'] == hanging['accepted_samples'],
                'A hanging rejection incorrectly continued its 1000-draw sampling batch')
        require(hanging['outside_or_occupied'] == 0 and hanging['expansion_attempts'] == 0 and hanging['added'] == 0,
                'Hanging samples were misclassified or reached graph expansion')
        require(after['graph'] == before['graph'] and after['status']['vertices'] == before['status']['vertices'],
                'A hanging sample leaked into the persistent global graph')

        supported_scene = publish_scene(True)
        before = snapshot()
        supported = explicit_expansion()
        after = snapshot()
        summary['supported'] = {'scene': supported_scene, 'expansion': supported, 'before': before, 'after': after}
        require(supported['hanging_rejections'] == 0 and supported['added'] > 0
                and supported['expansion_attempts'] >= supported['added'],
                'The same node did not expand after ground support was observed')
        require(supported['attempts'] == supported['sampling_batches'] == supported['accepted_samples']
                and supported['outside_or_occupied'] == 0, 'Supported free samples did not finish their sampling batch')
        require(after['status']['vertices']-before['status']['vertices'] == supported['added']
                and after['status']['edges'] > before['status']['edges'], 'Supported expansion did not enlarge the actual graph')
        for vertex in after['graph']['vertices']:
            require(abs(vertex['position'][2]-.8) < 1e-8, 'Global ground vertex lost its planning-height convention')
        summary['checks'] = {'real_local_request_enables_background': True, 'timer_disabled': True,
                             'hanging_accepted_before_rejection': True, 'hanging_finishes_sampling_batch': True,
                             'hanging_never_expands_global_graph': True, 'same_node_supported_floor_expands': True,
                             'soft_time_budget_preserved': True, 'service_and_status_metrics_agree': True}
        summary['status'] = 'passed'
    except Exception as error:
        summary['error'], summary['traceback'] = str(error), traceback.format_exc()
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
            summary['planner_exit'], summary['forced_termination'] = process.returncode, forced
            if forced or process.returncode != 0:
                summary['status'] = 'failed'
                summary.setdefault('error', 'Native planner did not exit normally')
        if node is not None:
            node.destroy_node()
        if initialized:
            rclpy.shutdown()
        summary['elapsed_s'] = time.monotonic()-started
        output.write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'status': summary['status'], 'output': str(output),
                      'elapsed_s': summary['elapsed_s'], 'error': summary.get('error')}))
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
