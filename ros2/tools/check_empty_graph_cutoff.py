#!/usr/bin/env python3
"""Native local-service cutoff on an occupied wall, then reuse with the wall clear.

Uses synthetic occupancy and explicit source poses. Stationary source odometry
satisfies the live-state validation contract; there is no motion consumer.
The C++ demo separately checks sampling-round boundaries and upstream ordering.
"""
import argparse
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
    parser.add_argument('--output', type=Path, default=workspace/'results/empty_graph_cutoff/native.json')
    parser.add_argument('--sampling-budget-mode', choices=('raw_draws', 'sampling_rounds'), default='raw_draws')
    parser.add_argument('--max-attempts', type=int, default=21)
    parser.add_argument('--max-sampling-rounds', type=int, default=21)
    args = parser.parse_args()
    if args.max_attempts < 1 or args.max_sampling_rounds < 1:
        parser.error('Both sampling budgets must be positive')
    sampling_budget = args.max_sampling_rounds if args.sampling_budget_mode == 'sampling_rounds' else args.max_attempts
    budget_stop_reason = 'sampling_round_limit' if args.sampling_budget_mode == 'sampling_rounds' else 'raw_draw_limit'
    blocked_draws = min(7, sampling_budget)
    cutoff_reached = sampling_budget >= 7
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100+os.getpid()%80))
    namespace = f'/empty_graph_cutoff_{os.getpid()}'
    started = time.monotonic()
    deadline = started+45.
    process = node = None
    initialized = False
    summary = {'status': 'failed', 'scope': __doc__, 'checks': {}}
    try:
        import rclpy
        from geometry_msgs.msg import Pose
        from nav_msgs.msg import Odometry
        from planner_msgs.srv import PlannerSrv, ValidatePath
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        sys.dont_write_bytecode = True
        sys.path.insert(0, str(workspace/'src/gbplanner_local/scripts'))
        from local_demo import DIM, ORIGIN, RES, make_map

        rclpy.init(args=[])
        initialized = True
        node = Node('probe', namespace=namespace)
        durable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        maps = node.create_publisher(PointCloud2, 'voxel_map', durable)
        odometry = node.create_publisher(Odometry, 'odometry', 10)
        accepted, metrics = [], []
        node.create_subscription(PointCloud2, 'accepted_voxel_map', accepted.append, durable)
        node.create_subscription(String, 'planning_metrics',
                                 lambda message: metrics.append(json.loads(message.data)), durable)
        planner = node.create_client(PlannerSrv, 'gbplanner')
        validator = node.create_client(ValidatePath, 'validate_path')
        sensor_state = node.create_client(Trigger, 'sensor_gain')
        options = {
            'map.resolution': RES, 'map.origin': list(ORIGIN), 'map.dimensions': list(DIM),
            'global.enable': False, 'global.background_enable': False,
            'robot.size': [.1, .1, .1], 'planning.graph_mode': False,
            'planning.sampling_budget_mode': args.sampling_budget_mode,
            'planning.max_attempts': args.max_attempts,
            'planning.max_sampling_rounds': args.max_sampling_rounds, 'planning.empty_graph_cutoff': 7,
            'planning.max_vertices': 10, 'planning.max_edges': 100,
            'planning.edge_min': .05, 'planning.edge_max': 5., 'planning.overshoot': 0.,
            'sampling.min': [-.5, -.5, -.1], 'sampling.max': [4.5, .5, .1],
            'gain.free': 1., 'planning.path_direction_penalty': 0.,
            'planning.traverse_length_max': 10., 'planning.traverse_time_max': 100.,
            'planning.path_safety_enhance_enable': False,
            'sensor.range': .5, 'sensor.horizontal_fov': 1., 'sensor.horizontal_resolution': 1.,
            'sensor.vertical_fov': .2, 'sensor.vertical_resolution': .2,
        }
        for axis in ('x', 'y', 'z', 'heading'):
            options[f'sampling.{axis}.distribution'] = 'constant'
            options[f'sampling.{axis}.constant'] = 4. if axis == 'x' else 0.
        launch = ['ros2', 'run', 'gbplanner_local', 'local_planner_node', '--ros-args',
                  '-r', '__ns:='+namespace]
        for key, value in options.items():
            launch += ['-p', key+':='+json.dumps(value)]
        summary['parameters'], summary['launch'] = options, launch
        log_path = output.with_suffix('.log')
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

        def call(client, request):
            until(client.service_is_ready, 'Service discovery')
            pending = client.call_async(request)
            until(pending.done, 'Service response')
            return pending.result()

        until(lambda: maps.get_subscription_count() > 0 and planner.service_is_ready(), 'Map discovery')
        request = PlannerSrv.Request()
        request.header.frame_id, request.bound_mode = 'map', PlannerSrv.Request.K_EXACT_BOUND
        request.root_pose = Pose()
        request.root_pose.position.x, request.root_pose.position.y = -2., 2.
        request.root_pose.orientation.w = 1.
        for label, wall in (('blocked', True), ('clear', False)):
            cloud = make_map(known=True, wall=wall)
            cloud.header.stamp = node.get_clock().now().to_msg()
            count = len(accepted)
            maps.publish(cloud)
            until(lambda: len(accepted) > count, 'Occupancy snapshot acceptance')
            require(bytes(accepted[-1].data) == bytes(cloud.data), 'Map acceptance changed the fixture')
            first = len(metrics)
            response = call(planner, request)
            until(lambda: len(metrics) > first, 'Planning metrics')
            metric = metrics[-1]
            summary[label] = {'metric': metric, 'response_status': response.status,
                              'path': [[p.position.x, p.position.y, p.position.z] for p in response.path]}
            require(metric['empty_graph_cutoff'] == 7, 'ROS parameter did not reach the planner')
            require(metric['sampling_budget_mode'] == args.sampling_budget_mode
                    and metric['max_attempts'] == args.max_attempts
                    and metric['max_sampling_rounds'] == args.max_sampling_rounds,
                    'Selected sampling budget parameters did not reach the planner')
            require(metric['raw_draws'] == metric['attempts'], 'Raw-draw and attempts metrics disagree')
            if wall:
                require(not response.path and metric['reason'] == 'no_feasible_path'
                        and metric['vertices'] == 1 and metric['attempts'] == blocked_draws
                        and metric['sampling_rounds'] == blocked_draws
                        and metric['empty_graph_cutoff_reached'] == cutoff_reached
                        and metric['sampling_stop_reason'] == ('empty_graph_cutoff' if cutoff_reached else budget_stop_reason),
                        'Isolated root did not honor the native cutoff and selected sampling budget')
                require(not metric['gain_evaluated'] and not metric['auto_global']['counted'],
                        'A pre-evaluation build failure counted as a no-frontier evaluation')
            else:
                require(len(response.path) > 1 and metric['success'] and metric['vertices'] == 2
                        and metric['attempts'] == sampling_budget and metric['sampling_rounds'] == sampling_budget
                        and not metric['empty_graph_cutoff_reached']
                        and metric['sampling_stop_reason'] == budget_stop_reason,
                        'A nonempty graph incorrectly lost its remaining sampling budget')
                # An explicit planning root does not replace measured odometry.
                # ValidatePath also checks the current robot pose, so establish
                # the stationary source and wait for the node to acknowledge it.
                until(lambda: odometry.get_subscription_count() > 0, 'Odometry discovery')
                observed = Odometry()
                observed.header.frame_id = 'map'
                observed.header.stamp = node.get_clock().now().to_msg()
                observed.pose.pose = request.root_pose
                odometry.publish(observed)
                until(lambda: json.loads(call(sensor_state, Trigger.Request()).message).get('root')
                      == metric['planning_root'], 'Stationary source odometry acceptance')
                check = ValidatePath.Request()
                check.header.frame_id, check.bound_mode, check.path = 'map', request.bound_mode, response.path
                result = call(validator, check)
                require(result.valid, 'Returned clear-map route failed native validation: '+result.reason)
                summary['clear']['validation'] = result.reason
        summary['checks'] = {'blocked_honors_cutoff_and_budget': True, 'gain_counter_unchanged': True,
                             'selected_sampling_budget_metrics': True, 'raw_draws_match_attempts': True,
                             'same_node_clear_map_uses_full_budget': True, 'returned_path_validated': True}
        if sampling_budget == 21:
            summary['checks']['blocked_ends_at_7_not_21'] = True
        summary['status'] = 'passed'
    except Exception as error:
        summary['error'], summary['traceback'] = str(error), traceback.format_exc()
    finally:
        if process is not None:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
            summary['planner_exit'] = process.returncode
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
