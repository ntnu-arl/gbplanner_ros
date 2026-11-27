#!/usr/bin/env python3
"""One semantic door connection, global route and label persistence check."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile

import rclpy
from nav_msgs.msg import Odometry, Path as RosPath
from planner_msgs.srv import PlannerGlobal, PlannerHoming, PlannerStringTrigger, ValidatePath
from planner_semantic_msgs.msg import SemanticPoint
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_srvs.srv import Trigger

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/gbplanner_local/scripts'))
from local_demo import Demo, xyz
from search_demo import scene, pose, SOURCE, TARGET


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    summary = {'passed': False, 'scope': 'short services; no robot motion'}
    process = demo = None
    with tempfile.TemporaryDirectory(prefix='gbplanner-semantic-') as temporary:
        directory = Path(temporary)
        config = directory / 'params.yaml'
        config.write_text(json.dumps({'gbplanner_local': {'ros__parameters': {
            'global.enable': True, 'search.sampling.min': [-5., -5., -1.],
            'search.sampling.max': [5., 5., 1.], 'search.target_radius': 0.8}}}))
        try:
            with (directory / 'node.log').open('w') as log:
                process = subprocess.Popen(['ros2', 'run', 'gbplanner_local', 'local_planner_node',
                    '--ros-args', '--params-file', str(config)], stdout=log,
                    stderr=subprocess.STDOUT, start_new_session=True)
                rclpy.init()
                demo = Demo()
                odom = Odometry()
                odom.header.frame_id = 'map'; odom.child_frame_id = 'base_link'
                odom.pose.pose = pose(SOURCE)

                def publish_odom():
                    odom.header.stamp = demo.get_clock().now().to_msg()
                    demo.odometry.publish(odom)

                demo.create_timer(0.05, publish_odom)
                clients = {}
                for name, kind in [('semantic_state', Trigger), ('global_status', Trigger),
                                   ('gbplanner/global', PlannerGlobal), ('gbplanner/homing', PlannerHoming),
                                   ('gbplanner/save_graph', PlannerStringTrigger),
                                   ('gbplanner/load_graph', PlannerStringTrigger), ('validate_path', ValidatePath)]:
                    clients[name] = demo.create_client(kind, name)
                    assert clients[name].wait_for_service(timeout_sec=10), name

                def call(name, request):
                    return demo.call(clients[name], request)

                def status(name):
                    return json.loads(call(name, Trigger.Request()).message)

                paths = []
                durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
                demo.create_subscription(RosPath, 'semantic_path', paths.append, durable)
                publisher = demo.create_publisher(SemanticPoint, 'semantic_location', 10)
                demo.publish_map(scene('doorway'))
                demo.spin_until(lambda: status('global_status')['vertices'] == 1)
                demo.spin_until(lambda: publisher.get_subscription_count() > 0)
                target = SemanticPoint()
                target.header.frame_id = 'map'; target.type.value = target.type.K_DOOR
                target.point.x, target.point.y, target.point.z = TARGET
                target.header.stamp = demo.get_clock().now().to_msg()
                publisher.publish(target)
                demo.spin_until(lambda: status('semantic_state')['received'] == 1)
                state = status('semantic_state')
                assert state['success'] and state['target_id'] > 0, state
                demo.spin_until(lambda: bool(paths))
                semantic_path = [item.pose for item in paths[-1].poses]
                assert len(semantic_path) > 2 and xyz(semantic_path)[-1] == list(TARGET), state
                request = PlannerGlobal.Request()
                request.id = state['target_id']; request.ignore_time = True; request.not_check_frontier = True
                route = call('gbplanner/global', request)
                assert len(route.path) > 2 and xyz(route.path)[-1] == list(TARGET)
                check = ValidatePath.Request(); check.header.frame_id = 'map'; check.path = route.path
                checked = call('validate_path', check)
                assert checked.valid, checked.reason
                summary['semantic'] = state
                summary['connection_points'] = len(semantic_path)
                summary['route_points'] = len(route.path)
                summary['route'] = xyz(route.path)
                save = PlannerStringTrigger.Request(); save.message = str(directory / 'graph.bin')
                assert call('gbplanner/save_graph', save).success
                assert call('gbplanner/load_graph', save).success
                restored = status('semantic_state')
                assert restored['targets'] == state['targets'], restored
                homing = PlannerHoming.Request(); homing.header.frame_id = 'map'
                home = call('gbplanner/homing', homing)
                assert home.path and xyz(home.path)[-1] == list(SOURCE)
                # One rejected occupied marker proves the input does not label
                # a nearby search fallback as if it were the actual door.
                target.point.x = 0.; target.point.y = -0.5
                publisher.publish(target)
                demo.spin_until(lambda: status('semantic_state')['received'] == 2)
                rejected = status('semantic_state')
                assert not rejected['success'] and rejected['targets'] == state['targets'], rejected
                summary['occupied_target_reason'] = rejected['reason']
                summary['restored_labels'] = restored['targets']; summary['passed'] = True
        except Exception as error:
            summary['error'] = str(error)
            if (directory / 'node.log').exists():
                print((directory / 'node.log').read_text()[-5000:])
            raise
        finally:
            args.output.write_text(json.dumps(summary, indent=2) + '\n')
            if demo is not None:
                demo.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            if process is not None and process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL); process.wait()
    print(json.dumps(summary))


if __name__ == '__main__':
    main()
