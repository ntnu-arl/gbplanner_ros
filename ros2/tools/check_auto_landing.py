#!/usr/bin/env python3
"""Check a one-shot landing-service handoff, not flight or physical touchdown.

Native planner, PCI and follower run separately. The explicit fixture follows
trajectory samples and acknowledges Empty without simulating a descent.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def check_policy():
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/gbplanner_local/scripts'))
    from landing_policy import LandingPolicy
    def message(enabled=True, due=True, clock=True):
        return {'landing': {'configured': enabled, 'enabled': enabled,
                            'clock_valid': clock, 'due': due, 'remaining_s': 0. if due else 2.}}
    disabled = LandingPolicy('kAerial', 1.)
    assert not disabled.observe({}, 1.) and not disabled.enabled
    assert not disabled.observe(message(False), 1.) and not disabled.latched
    ground = LandingPolicy('kGround', 1.)
    assert not ground.observe(message(), 1.) and not ground.enabled and ground.requests == 0
    policy = LandingPolicy('kAerial', 1.)
    assert not policy.observe(message(clock=False), 1.) and not policy.latched
    assert not policy.observe(message(due=False), 1.) and policy.enabled
    assert policy.observe(message(), 2.)
    policy.expect_cancellation(True)
    assert not policy.advance(2.1, True, True, True)
    policy.motion_result(True)
    assert policy.advance(2.2, True, True, True)
    assert not policy.advance(2.3, True, True, True) and policy.requests == 1
    policy.service_result(True, 2.4)
    assert policy.phase == 'ACKNOWLEDGED' and policy.payload()['physical_confirmation'] == 'not_observed'
    assert not policy.observe(message(False), 3.) and not policy.observe(message(), 3.)
    assert policy.latched and policy.requests == 1
    uncertain = LandingPolicy('kAerial', 1.)
    assert uncertain.observe(message(), 1.)
    uncertain.expect_cancellation(True)
    uncertain.motion_result(False)
    assert not uncertain.advance(1.1, True, True, True) and uncertain.requests == 0
    timeout = LandingPolicy('kAerial', 1.)
    assert timeout.observe(message(), 1.)
    timeout.expect_cancellation(False)
    assert timeout.advance(1.1, True, True, True)
    assert not timeout.advance(2.2, True, True, True)
    timeout.service_result(True, 2.3)
    assert timeout.phase == 'UNCONFIRMED' and timeout.late_response_received and timeout.requests == 1
    return ['disabled_and_ground_no_request', 'valid_clock_deadline_required',
            'cancel_confirmation_before_one_request', 'ack_is_not_touchdown',
            'uncertain_cancel_does_not_handoff', 'timeout_has_no_retry_or_late_success']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('results/auto_landing.json'))
    parser.add_argument('--policy-only', action='store_true', help='Check only the ROS-independent latch')
    args = parser.parse_args()
    checks = check_policy()
    if args.policy_only:
        print(json.dumps({'status': 'passed', 'policy_checks': checks}, indent=2))
        return
    os.environ.setdefault('ROS_DOMAIN_ID', str(100+os.getpid() % 80))
    import rclpy
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from planner_msgs.srv import PciToWaypoint, PciTrigger, PciHomingTrigger
    from std_msgs.msg import String
    from std_srvs.srv import Empty, Trigger
    sys.path.insert(0, str(Path(get_package_prefix('gbplanner_execution'))/'lib/gbplanner_execution'))
    from pci_global_demo import GlobalConsumer
    from local_demo import make_map

    class LandingConsumer(GlobalConsumer):
        def __init__(self):
            super().__init__()
            self.land_calls, self.control_events = [], []
            self.land_server = self.create_service(Empty, 'land_srv', self.land)
            self.mission_state = self.create_client(Trigger, 'mission_state')
            self.waypoint = self.create_client(PciToWaypoint, 'pci_to_waypoint')
            durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(String, 'control_status',
                                     lambda msg: self.control_events.append(json.loads(msg.data)), durable)
            assert self.mission_state.wait_for_service(timeout_sec=10)
            assert self.waypoint.wait_for_service(timeout_sec=10)

        def land(self, request, response):
            self.land_calls.append({'received_monotonic_s': time.monotonic(),
                                    'position': list(self.position),
                                    'last_command_points': len(self.commands[-1]['points']) if self.commands else 0})
            return response  # Service transport only; no descent or touchdown fixture.

    args.output = args.output.resolve()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    log_path = args.output.with_suffix('.log')
    output = {'status': 'failed', 'scope': 'native deadline → PCI cancellation → Empty response; '
              'explicit trajectory/odometry fixture, no physical landing', 'policy_checks': checks}
    processes, node = [], None
    started = time.monotonic()
    log = log_path.open('w')
    try:
        config = Path(get_package_share_directory('gbplanner_local'))/'config/local.yaml'
        commands = [
            ['ros2', 'run', 'gbplanner_local', 'local_planner_node', '--ros-args',
             '--params-file', str(config), '-p', 'global.enable:=true',
             '-p', 'mission.enabled:=false', '-p', 'mission.auto_homing:=true',
             '-p', 'mission.auto_landing_enable:=true', '-p', 'mission.time_budget_before_landing:=8.0'],
            ['ros2', 'run', 'gbplanner_local', 'control_node.py', '--ros-args', '-p', 'request_timeout:=3.0'],
            ['ros2', 'run', 'gbplanner_execution', 'path_follower_node.py', '--ros-args', '-p', 'speed:=0.2'],
        ]
        for command in commands:
            processes.append(subprocess.Popen(command, start_new_session=True, stdout=log, stderr=subprocess.STDOUT))
        rclpy.init()
        node = LandingConsumer()
        node.publish_map(make_map())
        node.settle()
        node.spin_until(lambda: node.control_state()['known_cells'] > 0)
        mission = json.loads(node.call(node.mission_state, Trigger.Request()).message)
        output['mission_before'] = mission
        assert not mission['enabled'] and mission['landing']['enabled'] and not mission['auto_homing_effective'], mission
        assert mission['landing']['remaining_s'] > 2., 'Startup consumed the deadline before motion could be checked'
        request = PciToWaypoint.Request()
        request.header.frame_id = 'map'
        request.header.stamp = node.get_clock().now().to_msg()
        request.waypoint.position.x = -4.5
        request.waypoint.orientation.w = 1.
        node.call(node.waypoint, request)
        active = node.wait_control({'EXECUTING', 'ERROR', 'STOPPED'}, seconds=5)
        output['active'] = active
        assert active['state'] == 'EXECUTING' and active['mode'] == 'waypoint_direct', active
        node.spin_until(lambda: node.control_state()['landing']['phase'] in
                        {'ACKNOWLEDGED', 'FAILED', 'UNCONFIRMED'}, seconds=15)
        final, execution = node.control_state(), node.state()
        output.update(control=final, execution=execution)
        assert final['landing']['phase'] == 'ACKNOWLEDGED' and final['state'] == 'STOPPED', final
        assert final['landing']['cancellation_required'] and final['landing']['cancellation_confirmed'], final
        assert final['landing']['physical_confirmation'] == 'not_observed' and final['completed_actions'] == 0, final
        assert execution['cancelled'] == 1 and execution['hold_commands'] >= 1 and not execution['active'], execution
        assert len(node.land_calls) == 1 and node.land_calls[0]['last_command_points'] == 1, node.land_calls
        count = execution['trajectory_commands']
        node.spin_until(lambda: time.monotonic() >= node.land_calls[0]['received_monotonic_s']+.4)
        assert not node.call(node.pci_local, PciTrigger.Request()).success
        assert not node.call(node.pci_home, PciHomingTrigger.Request()).success
        assert len(node.land_calls) == 1 and node.state()['trajectory_commands'] == count
        assert not node.errors, node.errors
        output['status'] = 'passed'
    except Exception as error:
        output['error'] = f'{type(error).__name__}: {error}'
    finally:
        if node is not None:
            output['land_calls'], output['control_events'] = node.land_calls, node.control_events
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2)
        log.close()
        output['processes_stopped'] = all(process.poll() is not None for process in processes)
        output['elapsed_s'] = round(time.monotonic()-started, 3)
        output['log'] = str(log_path)
        args.output.write_text(json.dumps(output, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(output, ensure_ascii=False, indent=2))
    if output['status'] != 'passed':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
