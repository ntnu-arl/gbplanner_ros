#!/usr/bin/env python3
"""Short ROS service run for refinement, gate planning and PCI previews."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from planner_msgs.srv import PciSearch, PlannerGoToWaypoint, PlannerRequestPath, ValidatePath
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/gbplanner_local/scripts'))
from local_demo import Demo, make_map, xyz


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    params = {
        'global.enable': True, 'planning.path_safety_enhance_enable': True,
        'robot.safety_extension': [1.0, 1.0, 1.0],
        'gate.enable': True, 'gate.center': [0.0, 0.0, 0.0],
        'gate.line_search_length': 3.0, 'gate.center_search_radius': 0.1,
        'gate.center_search_step': 0.1,
        'robot.size_extension': [0.0, 2.0, 0.0],
        'robot.size_extension_min': [0.0, 0.0, 0.0],
    }
    config = args.output.parent / 'planning-parameters.yaml'
    config.write_text(json.dumps({'gbplanner_local': {'ros__parameters': params}}, indent=2))
    processes, logs = [], []
    demo = None
    summary = {'passed': False, 'scope': 'short service calls; no simulated motion'}
    try:
        for executable in ['local_planner_node', 'control_node.py', 'search_waypoints_node.py']:
            log = (args.output.parent / (executable + '.log')).open('w')
            logs.append(log)
            command = ['ros2', 'run', 'gbplanner_local', executable]
            if executable == 'local_planner_node':
                command += ['--ros-args', '--params-file', str(config)]
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                              start_new_session=True))
        rclpy.init()
        demo = Demo()
        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = -2.0
        odom.pose.pose.orientation.w = 1.0

        def publish_odometry():
            odom.header.stamp = demo.get_clock().now().to_msg()
            demo.odometry.publish(odom)

        demo.create_timer(0.05, publish_odometry)
        clients = {}
        for name, kind in [('gbplanner/passing_gate', PlannerRequestPath),
                           ('validate_path', ValidatePath), ('control_state', Trigger),
                           ('planner_control_interface/std_srvs/preview_pass_gate', Trigger),
                           ('search_waypoints/status', Trigger), ('pci_search', PciSearch),
                           ('planner_go_to_waypoint', PlannerGoToWaypoint)]:
            clients[name] = demo.create_client(kind, name)
            assert clients[name].wait_for_service(timeout_sec=10), name

        def call(name, request):
            return demo.call(clients[name], request)

        def status(name):
            return json.loads(call(name, Trigger.Request()).message)

        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        gate_metrics = []
        demo.create_subscription(String, 'gate_metrics',
                                 lambda value: gate_metrics.append(json.loads(value.data)), durable)
        demo.publish_map(make_map())
        demo.spin_until(lambda: status('search_waypoints/status')['initialized'])
        demo.spin_until(lambda: status('control_state')['known_cells'] > 0)
        result, metrics = demo.plan()
        assert result.path and metrics['refinement']['attempted'], metrics
        check = ValidatePath.Request()
        check.header.frame_id = 'map'
        check.path = result.path
        validated = call('validate_path', check)
        assert validated.valid, validated.reason
        summary['local'] = {'metrics': metrics, 'path': xyz(result.path), 'valid': validated.valid}

        # The 2 m opening accepts the relaxed 1.4 m box but rejects the 2.4 m
        # extended box. Use the existing occupancy fixture's actual geometry.
        demo.publish_map(make_map(known=True))
        gate = call('gbplanner/passing_gate', PlannerRequestPath.Request())
        demo.spin_until(lambda: bool(gate_metrics))
        assert gate.path and gate.bound.mode == 1, gate_metrics[-1]
        check.path = gate.path
        validated = call('validate_path', check)
        assert validated.valid, validated.reason
        summary['gate'] = {'metrics': gate_metrics[-1], 'path': xyz(gate.path),
                           'bound_mode': gate.bound.mode, 'valid': validated.valid}
        preview = call('planner_control_interface/std_srvs/preview_pass_gate', Trigger.Request())
        assert preview.success, preview.message
        gate_control = status('control_state')
        assert gate_control['state'] == 'PREVIEW', gate_control
        summary['gate_preview'] = gate_control

        waypoint = PlannerGoToWaypoint.Request()
        waypoint.waypoint.header.frame_id = 'map'
        waypoint.waypoint.pose = gate.path[-1]
        waypoint.check_collision = True
        route = call('planner_go_to_waypoint', waypoint)
        assert route.path, 'Global waypoint route missing'
        check.path = route.path
        validated = call('validate_path', check)
        assert validated.valid, validated.reason
        summary['waypoint'] = {'path': xyz(route.path), 'valid': validated.valid}

        target = PoseStamped()
        target.header.frame_id = 'map'
        target.pose.position.x = 1.0
        target.pose.orientation.w = 1.0
        setter = demo.create_publisher(PoseStamped, 'search/target_pose', 10)
        demo.spin_until(lambda: setter.get_subscription_count() > 0)
        setter.publish(target)
        demo.spin_until(lambda: status('search_waypoints/status')['target']['position'] == [1.0, 0.0, 0.0])
        # Let the durable marker output reach the separate PCI subscription.
        for _ in range(3):
            rclpy.spin_once(demo, timeout_sec=0.05)
        request = PciSearch.Request()
        request.header.frame_id = 'map'
        request.use_current_state = True
        request.not_exe_path = True
        request.bound_mode = request.K_EXACT_BOUND
        searched = call('pci_search', request)
        assert searched.success and searched.path, 'PCI search preview failed'
        control = status('control_state')
        assert control['state'] == 'PREVIEW' and control['search']['exact_target'], control
        summary['search_preview'] = {'path': xyz(searched.path), 'control': control,
                                     'markers': status('search_waypoints/status')}
        summary['passed'] = True
    except Exception as error:
        summary['error'] = str(error)
        raise
    finally:
        args.output.write_text(json.dumps(summary, indent=2) + '\n')
        if demo is not None:
            demo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for log in logs:
            log.close()
    print(json.dumps({'passed': summary['passed'], 'output': str(args.output),
                      'refinement': summary['local']['metrics']['refinement'],
                      'gate_bound_mode': summary['gate']['bound_mode']}))


if __name__ == '__main__':
    main()
