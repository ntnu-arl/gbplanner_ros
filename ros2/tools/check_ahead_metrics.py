#!/usr/bin/env python3
"""Read PlanAhead phase diagnostics from one real planner on a static map."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', '169')
    import rclpy
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from planner_msgs.srv import PlanAhead
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    sys.path.insert(0, str(Path(get_package_prefix('gbplanner_local'))/'lib/gbplanner_local'))
    from global_demo import GlobalDemo
    from local_demo import make_map

    started = time.monotonic()
    result = {'status': 'failed', 'scope': __doc__, 'checks': {}, 'metrics': []}
    node = process = None
    try:
        config = Path(get_package_share_directory('gbplanner_local'))/'config/local.yaml'
        command = ['ros2', 'run', 'gbplanner_local', 'local_planner_node', '--ros-args',
                   '--params-file', str(config), '-p', 'global.enable:=true',
                   '-p', 'planning.max_vertices:=32', '-p', 'gain.free:=0.05',
                   '-p', 'sampling.seed_mode:=sequence']
        result['command'] = command
        with output.with_suffix('.log').open('w') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
        rclpy.init()
        node = GlobalDemo()
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, 'plan_ahead_metrics',
                                 lambda m: result['metrics'].append(json.loads(m.data)), qos)
        client = node.create_client(PlanAhead, 'planner_ahead')
        assert client.wait_for_service(timeout_sec=10.)
        node.publish_map(make_map())
        node.move_odometry([-2., 0., 0.])
        request = PlanAhead.Request()
        request.header.frame_id = 'map'
        request.header.stamp = node.get_clock().now().to_msg()
        request.root_pose.position.x = -1.5
        request.root_pose.orientation.w = 1.
        request.bound_mode, request.arrival_time = 3, 1.
        reply = node.call(client, request)
        node.spin_until(lambda: len(result['metrics']) == 1)
        metrics = result['metrics'][0]
        phase = metrics['plan']
        assert phase is not None and phase['graph_build_completed'] and phase['gain_evaluated'], metrics
        assert metrics['success'] == reply.success and metrics['reason'] == reply.reason, metrics
        assert metrics['map_revision'] == reply.map_revision and metrics['known_cells'] > 0, metrics
        assert metrics['planning_root'] == [-1.5, 0., 0., 0.] and metrics['bound_mode'] == 3, metrics
        assert metrics['effective_seed'] == 42 and metrics['sampling_request_index'] == 0, metrics
        assert 0. < phase['gain_ms'] <= phase['evaluate_ms'] <= metrics['plan_ms'] <= metrics['service_ms'], metrics
        # The refinement helper is still called when its policy is disabled;
        # its measured duration is therefore allowed to be a small positive
        # subset of evaluate_ms.
        assert phase['build_ms'] > 0. and 0. <= phase['refinement_ms'] <= phase['evaluate_ms'], metrics
        result['checks']['completed_phases_and_response_correspond'] = True
        result['checks']['timings_are_nested_and_seed_root_recorded'] = True
        request.header.frame_id = 'wrong_frame'
        rejected = node.call(client, request)
        node.spin_until(lambda: len(result['metrics']) == 2)
        metric = result['metrics'][1]
        assert not rejected.success and metric['reason'] == 'wrong_frame', metric
        assert metric['plan'] is None and metric['effective_seed'] is None and metric['plan_ms'] == 0., metric
        result['checks']['preplanning_rejection_has_no_invented_phase'] = True
        result['status'] = 'passed'
    except Exception as error:
        result['error'] = f'{type(error).__name__}: {error}'
    finally:
        if process is not None:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5.)
            result['planner_exit'] = process.returncode
            if process.returncode != 0:
                result['status'] = 'failed'
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        result['elapsed_s'] = time.monotonic()-started
        output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k: result.get(k) for k in ('status', 'checks', 'elapsed_s', 'error')}))
    return 0 if result['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
