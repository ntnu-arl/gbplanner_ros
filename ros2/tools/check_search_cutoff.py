#!/usr/bin/env python3
"""Check SearchPlanner sampling cutoffs through native ROS 2 services.

Four isolated planners use fixed synthetic occupancy and stationary odometry.
The workflow shares one 54 s deadline, with bounded process-group cleanup.
This checks search queries and path validity, not robot motion.
"""
import argparse
import copy
import json
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


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=workspace / 'results/search_cutoff/native.json')
    parser.add_argument('--domain', type=int, default=80 + os.getpid() % 120)
    args = parser.parse_args()
    if not 0 <= args.domain <= 232:
        parser.error('--domain must be between 0 and 232')
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    namespace = f'/search_cutoff_{os.getpid()}'
    started = time.monotonic()
    deadline = started + 54.
    summary = {'status': 'failed', 'scope': __doc__, 'stage': 'startup', 'checks': {},
               'namespace': namespace, 'domain_id': args.domain,
               'workflow_deadline_seconds': 54., 'total_budget_seconds': 60.,
               'cases': {}, 'last_states': {}}
    processes, logs, cases = [], [], {}
    node = timer = None
    initialized = False
    try:
        import rclpy
        from nav_msgs.msg import Odometry, Path as RosPath
        from planner_msgs.msg import Graph
        from planner_msgs.srv import PlannerSearch, ValidatePath
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        sys.dont_write_bytecode = True
        sys.path.insert(0, str(workspace / 'src/gbplanner_local/scripts'))
        from local_demo import DIM, ORIGIN, RES, xyz
        from search_demo import pose, scene

        rclpy.init(args=[])
        initialized = True
        node = Node('probe', namespace=namespace)
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        common = {
            'map.resolution': RES, 'map.origin': list(ORIGIN), 'map.dimensions': list(DIM),
            'global.enable': False, 'global.background_enable': False,
            'robot.size': [.1, .1, .1], 'planning.overshoot': 0.,
            'planning.path_safety_enhance_enable': False,
            'search.sampling.seed': 42, 'search.max_vertices': 500,
            'search.max_edges': 10000, 'search.max_target_candidates': 5,
        }
        doorway = {
            'planning.edge_min': .12, 'planning.edge_max': .8,
            'planning.neighbor_radius': 1.1, 'planning.neighbor_min': .12,
            'planning.neighbor_max': 1.1,
            'search.sampling.min': [-5., -5., -1.],
            'search.sampling.max': [5., 5., 1.],
            'search.max_attempts': 12000, 'search.target_radius': .8,
        }
        fixed = {
            'planning.graph_mode': False, 'planning.edge_min': .05, 'planning.edge_max': 5.,
            'search.sampling.min': [-.5, -.5, -.1],
            'search.sampling.max': [4.5, .5, .1],
            'search.max_attempts': 21, 'search.empty_graph_cutoff': 7,
            'search.target_radius': .1,
        }
        for axis in ('x', 'y', 'z', 'heading'):
            fixed[f'search.sampling.{axis}.distribution'] = 'constant'
            fixed[f'search.sampling.{axis}.constant'] = 4. if axis == 'x' else 0.
        definitions = (
            ('doorway_cutoff_1', 'doorway', (-2., -2., 0.), (2., -2., 0.),
             {**doorway, 'search.empty_graph_cutoff': 1}),
            ('doorway_cutoff_1500', 'doorway', (-2., -2., 0.), (2., -2., 0.),
             {**doorway, 'search.empty_graph_cutoff': 1500}),
            ('blocked_valid_sample', 'sealed', (-2., -.5, 0.), (2., -.5, 0.), fixed),
            ('rejected_sample', 'sealed', (-2., -.5, 0.), (2., -.5, 0.),
             {**fixed, 'search.sampling.x.constant': 2.}),
        )
        clouds = {kind: scene(kind) for kind in ('doorway', 'sealed')}
        for label, kind, source, target, overrides in definitions:
            prefix = namespace + '/' + label
            options = {**common, **overrides}
            parameters = output.parent / (label + '.yaml')
            parameters.write_text(json.dumps({'/**': {'ros__parameters': options}}, indent=2) + '\n')
            command = ['ros2', 'run', 'gbplanner_local', 'local_planner_node', '--ros-args',
                       '--params-file', str(parameters), '-r', '__ns:=' + prefix]
            log = (output.parent / (label + '.log')).open('w')
            logs.append(log)
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True, env=dict(os.environ))
            processes.append((label, process))
            case = {'source': source, 'target': target, 'kind': kind,
                    'metrics': [], 'accepted': [], 'paths': [], 'graphs': [],
                    'maps': node.create_publisher(PointCloud2, prefix + '/voxel_map', durable),
                    'odometry': node.create_publisher(Odometry, prefix + '/odometry', 10),
                    'planner': node.create_client(PlannerSearch, prefix + '/planner_search'),
                    'validator': node.create_client(ValidatePath, prefix + '/validate_path'),
                    'sensor_state': node.create_client(Trigger, prefix + '/sensor_gain')}
            node.create_subscription(String, prefix + '/search_metrics',
                                     lambda message, values=case['metrics']: values.append(json.loads(message.data)), durable)
            node.create_subscription(PointCloud2, prefix + '/accepted_voxel_map', case['accepted'].append, durable)
            node.create_subscription(RosPath, prefix + '/search_path', case['paths'].append, durable)
            node.create_subscription(Graph, prefix + '/search_graph', case['graphs'].append, durable)
            cases[label] = case
            summary['cases'][label] = {'map': kind, 'source': list(source), 'target': list(target),
                                       'parameters': options, 'command': command}

        def until(predicate, label):
            while True:
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"{summary['stage']}: {label}")
                for name, process in processes:
                    if process.poll() is not None:
                        raise RuntimeError(f'{name} exited with status {process.returncode}; see {name}.log')
                value = predicate()
                if value:
                    return value
                rclpy.spin_once(node, timeout_sec=min(.03, max(0., deadline - time.monotonic())))

        def call(client, request, label):
            until(client.service_is_ready, label + ' discovery')
            pending = client.call_async(request)
            until(pending.done, label + ' response')
            return pending.result()

        def measured_source(label, case):
            result = json.loads(call(case['sensor_state'], Trigger.Request(), label + ' odometry status').message)
            summary['last_states'][label] = result
            return result.get('root') == [*case['source'], 0.]

        def publish_odometry():
            for case in cases.values():
                message = Odometry()
                message.header.frame_id, message.child_frame_id = 'map', 'base_link'
                message.header.stamp = node.get_clock().now().to_msg()
                message.pose.pose = pose(case['source'])
                case['odometry'].publish(message)

        timer = node.create_timer(.05, publish_odometry)
        for label, case in cases.items():
            summary['stage'] = label + ':map_and_odometry'
            until(lambda: case['maps'].get_subscription_count() > 0
                  and case['odometry'].get_subscription_count() > 0
                  and case['planner'].service_is_ready(), 'map, odometry and search discovery')
            cloud = copy.deepcopy(clouds[case['kind']])
            cloud.header.stamp = node.get_clock().now().to_msg()
            case['maps'].publish(cloud)
            until(lambda: bool(case['accepted']), 'accepted occupancy snapshot')
            require(bytes(case['accepted'][-1].data) == bytes(cloud.data), label + ': fixture changed')
            until(lambda: measured_source(label, case), 'stationary source odometry')

            summary['stage'] = label + ':search'
            request = PlannerSearch.Request()
            request.header.frame_id = 'map'
            request.header.stamp = node.get_clock().now().to_msg()
            request.bound_mode = PlannerSearch.Request.K_EXACT_BOUND
            request.use_current_state = True
            request.source, request.target = pose(case['source']), pose(case['target'])
            response = call(case['planner'], request, label + ' PlannerSearch')
            until(lambda: bool(case['metrics']) and bool(case['paths']) and bool(case['graphs']),
                  'search metrics, path and graph')
            metric = case['metrics'][-1]
            path = xyz(response.path)
            result = summary['cases'][label]
            result.update({'metrics': metric, 'path': path, 'response_success': response.success})
            require(metric['success'] == response.success, label + ': service/metric disagreement')
            require(xyz([item.pose for item in case['paths'][-1].poses]) == path,
                    label + ': service/topic path disagreement')
            require(len(case['graphs'][-1].vertices) == metric['vertices'], label + ': graph count mismatch')
            require(metric['seed'] == 42 and metric['map_revision'] == 1, label + ': wrong seed/map')

            if label.startswith('doorway'):
                require(response.success and metric['exact_target'] and len(path) > 2,
                        label + ': no exact detour after the rejected first draw')
                require(path[0] == list(case['source']) and path[-1] == list(case['target'])
                        and any(point[1] > 1. for point in path), label + ': wrong doorway geometry')
                require(metric['attempts'] > metric['sampling_rounds'] > 1
                        and not metric['empty_graph_cutoff_reached'], label + ': wrong sampling counters')
                summary['stage'] = label + ':validate_path'
                check = ValidatePath.Request()
                check.header.frame_id, check.bound_mode, check.path = 'map', request.bound_mode, response.path
                valid = call(case['validator'], check, label + ' ValidatePath')
                require(valid.valid and valid.map_revision == 1, label + ': invalid returned route: ' + valid.reason)
                result['validation'] = {'valid': valid.valid, 'reason': valid.reason,
                                        'map_revision': valid.map_revision}
            elif label == 'blocked_valid_sample':
                require(not response.success and not path and metric['reason'] == 'no_feasible_path'
                        and metric['attempts'] == metric['sampling_rounds'] == 8
                        and metric['vertices'] == 1 and metric['empty_graph_cutoff_reached'],
                        'Valid disconnected samples must stop after strict sampling_rounds > 7')
            else:
                require(not response.success and not path and metric['reason'] == 'no_feasible_path'
                        and metric['attempts'] == 21 and metric['sampling_rounds'] == 0
                        and metric['vertices'] == 1 and not metric['empty_graph_cutoff_reached'],
                        'Occupied samples must exhaust only the independent raw attempt budget')

        summary['stage'] = 'compare_doorway_cutoffs'
        low, normal = (summary['cases'][label] for label in ('doorway_cutoff_1', 'doorway_cutoff_1500'))
        require(low['path'] == normal['path'], 'Empty-graph cutoff changed an already growing graph route')
        for field in ('success', 'reason', 'exact_target', 'vertices', 'edges', 'attempts', 'sampling_rounds',
                      'empty_graph_cutoff_reached', 'collision_rejections', 'target_candidates', 'target_id',
                      'path_length_m', 'endpoint_distance'):
            require(low['metrics'][field] == normal['metrics'][field], 'Cutoff comparison differs at ' + field)
        summary['checks'] = {'rejected_draw_does_not_exhaust_cutoff_1': True,
                             'grown_graph_matches_normal_cutoff': True,
                             'valid_blocked_samples_stop_at_round_8': True,
                             'pure_rejection_uses_raw_budget_with_zero_rounds': True,
                             'native_search_topics_agree': True, 'returned_detours_validate': True}
        summary['status'], summary['stage'] = 'passed', 'completed'
    except Exception as error:
        summary['error'], summary['traceback'] = str(error), traceback.format_exc()
    finally:
        if timer is not None:
            timer.cancel()
        if node is not None:
            node.destroy_node()
        if initialized:
            rclpy.shutdown()
        for _, process in processes:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        grace = min(started + 58., time.monotonic() + 1.)
        for _, process in processes:
            try:
                process.wait(timeout=max(0., grace - time.monotonic()))
            except subprocess.TimeoutExpired:
                pass
        for _, process in processes:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        for _, process in processes:
            try:
                process.wait(timeout=max(0., started + 59. - time.monotonic()))
            except subprocess.TimeoutExpired:
                pass
        for log in logs:
            log.close()
        summary['elapsed_seconds'] = time.monotonic() - started
        temporary = output.with_suffix('.tmp')
        temporary.write_text(json.dumps(summary, indent=2) + '\n')
        temporary.replace(output)
    print(json.dumps({'status': summary['status'], 'stage': summary['stage'],
                      'checks': summary['checks'], 'error': summary.get('error'),
                      'elapsed_seconds': summary['elapsed_seconds'], 'output': str(output)}, indent=2))
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
