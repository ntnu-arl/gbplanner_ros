#!/usr/bin/env python3
"""Load the installed panel in a real RViz2 window on an existing X display.

Observes native ROS graph endpoints for the six clients and SetGoal publisher.
No command request, planner, simulator or robot motion is started here.
"""
import argparse
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time

import rclpy


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    log_path = output.with_suffix('.log')
    if output.exists() or log_path.exists():
        parser.error('Use a new output name to preserve previous results')
    os.environ.setdefault('ROS_DOMAIN_ID', str(100 + os.getpid() % 80))
    rclpy.init(args=[])
    node = rclpy.create_node('rviz_panel_host_probe')
    started = time.monotonic()
    report = {'status': 'failed', 'scope': __doc__, 'log': str(log_path)}
    process = None
    try:
        command = ['ros2', 'launch', 'gbplanner_ui', 'control.launch.py']
        report['command'] = command
        with log_path.open('w') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        prefix = '/planner_control_interface/std_srvs/'
        expected = {prefix + name: ['std_srvs/srv/Trigger'] for name in (
            'automatic_planning', 'stop', 'homing_trigger', 'go_to_waypoint')}
        expected.update({'/pci_global': ['planner_msgs/srv/PciGlobal'],
                         '/pci_initialization_trigger': ['planner_msgs/srv/PciInitialization']})
        while time.monotonic() - started < 35:
            if process.poll() is not None:
                raise RuntimeError('RViz launch exited before discovery')
            rclpy.spin_once(node, timeout_sec=.1)
            panels = [(name, ns) for name, ns in node.get_node_names_and_namespaces()
                      if name.startswith('gbplanner_panel_')]
            if len(panels) != 1:
                continue
            name, namespace = panels[0]
            actual = dict(node.get_client_names_and_types_by_node(name, namespace))
            goals = node.get_publishers_info_by_topic('/move_base_simple/goal')
            matched = [publisher for publisher in goals
                       if publisher.topic_type == 'geometry_msgs/msg/PoseStamped']
            if all(actual.get(key) == value for key, value in expected.items()) and matched:
                report.update(panel_node={'name': name, 'namespace': namespace}, clients=actual,
                    goal_publishers=[{'node': publisher.node_name, 'namespace': publisher.node_namespace,
                                      'type': publisher.topic_type} for publisher in matched])
                report['status'] = 'passed'
                break
        else:
            raise TimeoutError('Panel clients or configured SetGoal publisher did not appear')
    except Exception as error:
        report['error'] = f'{type(error).__name__}: {error}'
    finally:
        forced = False
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                forced = True
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
        report['launch_returncode'] = process.returncode if process is not None else None
        report['forced_termination'] = forced
        log = log_path.read_text() if log_path.exists() else ''
        report['load_errors'] = re.findall(r'^.*(?:Failed to load|could not load|undefined symbol|Segmentation fault|process has died).*$', log, re.M)
        if forced or report['launch_returncode'] != 0 or report['load_errors']:
            report['status'] = 'failed'
            report.setdefault('error', 'RViz load or orderly shutdown failed')
        node.destroy_node()
        rclpy.shutdown()
        report['wall_elapsed_s'] = time.monotonic() - started
        output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report))
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
