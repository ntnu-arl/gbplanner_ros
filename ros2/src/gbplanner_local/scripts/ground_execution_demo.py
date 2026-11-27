#!/usr/bin/env python3
"""Execute native terrain paths through a timed kinematic trajectory consumer.

The consumer interpolates the received command without projecting or correcting
its Z coordinate. It is an execution fixture, not wheel/ground dynamics.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.clock import Clock
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from planner_msgs.srv import ValidatePath

# Reuse the established action client and MultiDOF command decoder, while
# overriding its aerial fixture's velocity checks and initial height.
sys.path.insert(0, str(Path(get_package_prefix('gbplanner_execution')) / 'lib/gbplanner_execution'))
from execution_demo import Consumer, decode
from trajectory_tools import state_from_pose, wrap
from ground_planning_demo import BASE_HEIGHT, PLANNING_HEIGHT, RESOLUTION, ObservedTerrain, make_terrain, save_route
from local_demo import xyz

SPEED = 0.7
YAW_RATE = 1.5
SAMPLE_PERIOD = 0.05


class GroundConsumer(Consumer):
    def __init__(self, terrain, home):
        self.terrain = terrain
        self.phase = 'initialization'
        self.trace = []
        self.started = time.monotonic()
        super().__init__()
        self.position = (*home, 0.0)

    def command(self, message):
        try:
            points, times = decode(message)
            initial = self.position
            previous, previous_time = initial, 0.0
            max_speed = max_yaw_rate = 0.0
            for point, at in zip(points, times):
                interval = at - previous_time
                speed = math.dist(previous[:3], point[:3]) / interval
                yaw_rate = abs(wrap(point[3] - previous[3])) / interval
                max_speed = max(max_speed, speed)
                max_yaw_rate = max(max_yaw_rate, yaw_rate)
                assert speed <= SPEED + 1e-6, ('command_speed', speed)
                assert yaw_rate <= YAW_RATE + 1e-6, ('command_yaw_rate', yaw_rate)
                previous, previous_time = point, at
            assert abs(times[0] - SAMPLE_PERIOD) < 1e-9
            # This checks the command the separate follower actually published,
            # including its reconnection and interpolation of the planned path.
            terrain_checks = self.terrain.check_path([initial[:3]] + [point[:3] for point in points])
            received = time.monotonic()
            self.commands.append({'phase': self.phase, 'points': points, 'times': times,
                                  'received': received - self.started,
                                  'max_linear_speed_mps': max_speed,
                                  'max_yaw_rate_radps': max_yaw_rate,
                                  'terrain_checks': terrain_checks})
            self.active_command = (points, times, received, initial)
        except (AssertionError, ValueError) as error:
            self.errors.append(repr(error))
            self.follow = False

    def tick(self):
        super().tick()
        if self.terrain is None:
            return
        try:
            ground = self.terrain.support(self.position[0], self.position[1])
            error = abs(self.position[2] - ground - BASE_HEIGHT)
            assert error <= RESOLUTION + 1e-7, ('executed_height', self.position, ground)
            self.trace.append({'time_s': time.monotonic() - self.started,
                               'phase': self.phase, 'state': self.position,
                               'surface_z': ground, 'height_error_m': error})
        except AssertionError as error:
            if not self.errors:
                self.errors.append(repr(error))
            self.follow = False


def save(path, report):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(report, indent=2) + '\n')
    temporary.replace(path)


def execute_path(node, label, path, validator):
    request = ValidatePath.Request()
    request.header.frame_id = 'map'
    request.path = path
    validation = node.call(validator, request)
    assert validation.valid and validation.reason == 'ok', (label, validation.reason)
    states = [state_from_pose(point) for point in path]
    assert math.dist(node.position[:3], states[0][:3]) < 0.05, (node.position, states[0])
    before_commands = len(node.commands)
    before_feedback = len(node.feedback)
    node.phase = label
    start_position = node.position
    start_trace = len(node.trace)
    started = time.monotonic()
    handle = node.send(node.goal(states))
    assert handle.accepted, (label, 'PathFollower rejected planned terrain route', node.state())
    future = handle.get_result_async()
    node.spin_until(lambda: future.done() or bool(node.errors), seconds=45)
    assert not node.errors, node.errors
    result = future.result()
    assert result.status == GoalStatus.STATUS_SUCCEEDED and result.result.success, (label, result.status, node.state())
    finished = time.monotonic()
    assert len(node.commands) == before_commands + 1, (label, 'Unexpected command replacement', node.state())
    assert len(node.feedback) > before_feedback
    assert math.dist(node.position[:3], states[-1][:3]) < 0.05
    assert abs(wrap(node.position[3] - states[-1][3])) < 0.1
    rows = node.trace[start_trace:]
    executed = [start_position[:3]] + [row['state'][:3] for row in rows]
    checks = node.terrain.check_path(executed)
    assert checks['height_span_m'] >= 0.2 - 1e-7, checks
    command = node.commands[-1]
    assert finished - started >= command['times'][-1] - 0.1
    return {'path': [list(point[:3]) for point in states],
            'path_yaws': [point[3] for point in states],
            'action_status': result.status, 'action_success': result.result.success,
            'duration_wall_s': finished - started, 'command_duration_s': command['times'][-1],
            'trajectory_points': len(command['points']), 'feedback_count': len(node.feedback) - before_feedback,
            'final_base_state': node.position, 'executed_samples': len(rows),
            'executed_terrain_checks': checks,
            'validation': {'valid': validation.valid, 'reason': validation.reason,
                           'map_revision': validation.map_revision},
            'executor': node.state()}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=Path('results/ground_execution/summary.json'))
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    report = {'status': 'running', 'scope': 'Native ROS 2 planner and PathFollower with a wall-clock kinematic MultiDOF trajectory consumer; no wheel-ground dynamics',
              'base_height_m': BASE_HEIGHT, 'planning_height_m': PLANNING_HEIGHT,
              'speed_mps': SPEED, 'yaw_rate_radps': YAW_RATE, 'sample_period_s': SAMPLE_PERIOD}
    node = None
    processes, logs = [], []
    try:
        config = Path(get_package_share_directory('gbplanner_local')) / 'config/ground.yaml'
        commands = [
            ['ros2', 'run', 'gbplanner_local', 'local_planner_node', '--ros-args', '--params-file', str(config)],
            ['ros2', 'run', 'gbplanner_execution', 'path_follower_node.py', '--ros-args',
             '-p', 'robot_type:=kGround', '-p', f'speed:={SPEED}', '-p', f'yaw_rate:={YAW_RATE}',
             '-p', f'sample_period:={SAMPLE_PERIOD}', '-p', 'odom_timeout:=2.0',
             '-p', 'execution_grace:=3.0']]
        report['commands_launched'] = commands
        for label, command in zip(('planner', 'follower'), commands):
            log = (args.output.parent / (label + '.log')).open('w')
            logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                              start_new_session=True))
        rclpy.init()
        raw = make_terrain(Clock().now().to_msg())
        terrain = ObservedTerrain(raw)
        home = terrain.base_point(-2.0, 0.0)
        node = GroundConsumer(terrain, home)
        report['home_base_pose'] = home
        report['scene'] = terrain.scene_record()
        node.publish_map(raw)
        node.settle()
        # An actual stamped odometry packet initializes home. The fixture does
        # not move until a trajectory arrives on command/trajectory.
        node.spin_until(lambda: node.status()['vertices'] > 0 and node.state()['known_cells'] > 0)
        validator = node.create_client(ValidatePath, 'validate_path')
        assert validator.wait_for_service(timeout_sec=15)
        planned, local_metrics = node.plan()
        report['local_planning'] = {'metrics': local_metrics, 'path': xyz(planned.path)}
        save(args.output, report)
        assert local_metrics['success'], local_metrics
        report['local'] = execute_path(node, 'local', planned.path, validator)
        save_route(args.output.parent / 'local_path.csv', report['local']['path'], terrain)
        report['global_after_local'] = node.status()
        save(args.output, report)
        # Query homing only after the endpoint is observed in real odometry.
        node.phase = 'homing_planning'
        home_response, homing_metrics = node.global_call(homing=True)
        report['homing_planning'] = {'metrics': homing_metrics, 'path': xyz(home_response.path)}
        assert homing_metrics['success'], homing_metrics
        assert math.dist(xyz(home_response.path)[-1], home) < 1e-8
        report['homing'] = execute_path(node, 'homing', home_response.path, validator)
        save_route(args.output.parent / 'homing_path.csv', report['homing']['path'], terrain)
        assert math.dist(node.position[:3], home) < 0.05
        final = node.state()
        assert final['succeeded'] == 2 and final['aborted'] == 0 and final['hold_commands'] == 0, final
        assert not node.errors, node.errors
        report.update(status='passed', final_executor=final, trajectory_commands=node.commands,
                      executed_trace=node.trace, final_base_state=node.position,
                      global_graph=node.status(), checks={
                          'native_local_and_homing_paths_executed': True,
                          'native_action_success_requires_observed_endpoints': True,
                          'timed_multidof_commands_drive_odometry': True,
                          'consumer_does_not_project_or_correct_height': True,
                          'commands_and_executed_samples_follow_observed_support': True,
                          'native_validate_path_before_both_actions': True,
                          'linear_and_yaw_limits_verified': True,
                          'nonplanar_outward_and_return_motion': True,
                          'returned_to_observed_home': True})
        with (args.output.parent / 'executed_trace.csv').open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['time_s', 'phase', 'x', 'y', 'base_z', 'yaw', 'surface_z', 'height_error_m'])
            for row in node.trace:
                writer.writerow([row['time_s'], row['phase'], *row['state'], row['surface_z'], row['height_error_m']])
        save(args.output, report)
        print(json.dumps({'status': report['status'],
                          'local_duration_s': report['local']['duration_wall_s'],
                          'homing_duration_s': report['homing']['duration_wall_s'],
                          'executed_samples': len(node.trace), 'output': str(args.output)}), flush=True)
    except Exception as error:
        report.update(status='failed', error=repr(error))
        if node is not None:
            report.update(consumer_errors=node.errors, trajectory_commands=node.commands,
                          executed_trace=node.trace, final_base_state=node.position)
            try:
                report['executor_at_error'] = node.state()
            except Exception:
                pass
        save(args.output, report)
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
        for log in logs:
            log.close()


if __name__ == '__main__':
    main()
