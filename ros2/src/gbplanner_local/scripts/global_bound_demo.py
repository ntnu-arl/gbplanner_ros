#!/usr/bin/env python3
"""Exercise native axis-aligned and TF-transformed oriented planning bounds."""
import argparse
import itertools
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from planner_msgs.srv import PlannerDynamicGlobalBound, PlannerSetGlobalBound, ValidatePath
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from global_demo import GlobalDemo
from local_demo import make_map, xyz
from mapping_demo import inspect_snapshot
from rotated_sampling_demo import verify_path

ROOT = (-2.0, 0.0, 0.0)
DEFAULT_MIN = (-5.0, -5.0, -1.5)
DEFAULT_MAX = (5.0, 5.0, 1.5)
TF_YAW = 0.35
TF_TRANSLATION = (-0.5, -0.2, 0.0)
HALF_SIZE = (3.0, 1.6, 1.3)


def point_values(point):
    return [point.x, point.y, point.z]


def assign_point(point, values):
    point.x, point.y, point.z = map(float, values)


def close_vector(actual, expected, tolerance=1e-8):
    assert len(actual) == len(expected)
    assert all(abs(a - b) < tolerance for a, b in zip(actual, expected)), (actual, expected)


def aabb_record(bound):
    return {'minimum': point_values(bound.min_val), 'maximum': point_values(bound.max_val),
            'use_z_val': bound.use_z_val}


def world_point(local):
    c, s = math.cos(TF_YAW), math.sin(TF_YAW)
    return [TF_TRANSLATION[0] + c * local[0] - s * local[1],
            TF_TRANSLATION[1] + s * local[0] + c * local[1],
            TF_TRANSLATION[2] + local[2]]


def local_point(world):
    # Independent scalar inverse transform, not the diagnostic matrix.
    c, s = math.cos(TF_YAW), math.sin(TF_YAW)
    dx, dy, dz = [a - b for a, b in zip(world, TF_TRANSLATION)]
    return [c * dx + s * dy, -s * dx + c * dy, dz]


def verify_robot_box(point):
    for offset in itertools.product((-0.2, 0.2), repeat=3):
        corner = [a + b for a, b in zip(point, offset)]
        local = local_point(corner)
        assert all(abs(value) <= extent + 1e-8 for value, extent in zip(local, HALF_SIZE)), (
            'Robot corner outside oriented bound', corner, local)


def verify_bound_path(path):
    assert len(path) >= 2
    count = 0
    for start, end in zip(path, path[1:]):
        steps = max(1, math.ceil(math.dist(start, end) / 0.05))
        for step in range(steps + 1):
            point = [a + (b - a) * step / steps for a, b in zip(start, end)]
            verify_robot_box(point)
            count += 8
    return count


def save(path, report):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(report, indent=2) + '\n')
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=Path('results/global_bound/summary.json'))
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    report = {'status': 'running', 'root': ROOT, 'phases': [],
              'scope': 'Native ROS 2 bound services, local/global map policy and path validation'}
    node = process = log = None
    try:
        log = (args.output.parent / 'nodes.log').open('w')
        process = subprocess.Popen(
            ['ros2', 'run', 'gbplanner_local', 'local_planner_node', '--ros-args',
             '-p', 'global.enable:=true', '-p', 'sampling.seed:=42'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = GlobalDemo()
        axis = node.create_client(PlannerSetGlobalBound, 'gbplanner/set_global_bound')
        dynamic = node.create_client(PlannerDynamicGlobalBound, 'gbplanner/set_dynamic_global_bound')
        diagnostic = node.create_client(Trigger, 'global_bound')
        validator = node.create_client(ValidatePath, 'validate_path')
        for client in (axis, dynamic, diagnostic, validator):
            assert client.wait_for_service(timeout_sec=15), 'Required bound service missing'
        updates = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, 'map_updates', lambda value: updates.append(json.loads(value.data)), qos)

        def state():
            response = node.call(diagnostic, Trigger.Request())
            assert response.success, response.message
            return json.loads(response.message)

        def query():
            request = PlannerSetGlobalBound.Request()
            request.get_current_bound = True
            response = node.call(axis, request)
            assert response.success
            return aabb_record(response.bound_ret)

        def validate(path):
            request = ValidatePath.Request()
            request.header.frame_id = 'map'
            request.path = path
            response = node.call(validator, request)
            return {'valid': response.valid, 'reason': response.reason, 'map_revision': response.map_revision}

        raw = make_map(known=False, wall=True)
        node.publish_map(raw)
        snapshot = inspect_snapshot(raw)
        node.move_odometry(ROOT)
        node.spin_until(lambda: bool(updates))
        initial = state()
        default_query = query()
        close_vector(default_query['minimum'], DEFAULT_MIN)
        close_vector(default_query['maximum'], DEFAULT_MAX)
        observation_revision = updates[-1]['observation_revision']

        def changed(label, previous_state, previous_map_revision, extra=None):
            next_state = state()
            assert next_state['revision'] > previous_state['revision'], next_state
            node.spin_until(lambda: node.revision()['map_revision'] > previous_map_revision
                            and updates[-1]['map_revision'] == node.revision()['map_revision'])
            assert updates[-1]['observation_revision'] == observation_revision, updates[-1]
            entry = {'phase': label, 'state': next_state, 'world_aabb': query(),
                     'map_status': node.revision(), 'map_update': updates[-1],
                     'global_graph': node.status()}
            if extra:
                entry.update(extra)
            report['phases'].append(entry)
            save(args.output, report)
            return entry

        # The use-Z branch is checked explicitly before constructing the path
        # used for shrink/reset, so that comparison starts at the default bound.
        request = PlannerSetGlobalBound.Request()
        request.bound.use_z_val = True
        assign_point(request.bound.min_val, (-5.0, -5.0, -0.6))
        assign_point(request.bound.max_val, (5.0, 5.0, 0.6))
        revision = node.revision()['map_revision']
        response = node.call(axis, request)
        assert response.success
        close_vector(point_values(response.bound_ret.min_val), (-5.0, -5.0, -0.6))
        close_vector(point_values(response.bound_ret.max_val), (5.0, 5.0, 0.6))
        z_phase = changed('set_xyz', initial, revision)
        request = PlannerSetGlobalBound.Request()
        request.reset_to_default = True
        revision = node.revision()['map_revision']
        assert node.call(axis, request).success
        reset_phase = changed('reset_before_plan', z_phase['state'], revision)
        close_vector(reset_phase['world_aabb']['minimum'], DEFAULT_MIN)
        close_vector(reset_phase['world_aabb']['maximum'], DEFAULT_MAX)
        original, metrics = node.plan()
        assert metrics['success'], metrics
        original_path = xyz(original.path)
        verify_path(original_path, snapshot)
        assert validate(original.path)['valid']
        original_graph = node.status()
        assert original_graph['edges'] > 0, original_graph
        report['original'] = {'path': original_path, 'metrics': metrics,
                              'global_graph': original_graph, 'bound': reset_phase['state']}

        request = PlannerSetGlobalBound.Request()
        request.bound.use_z_val = False
        # Deliberately supply different Z values; XY-only updates must keep the
        # current [-1.5, 1.5] vertical range. The complete robot remains inside.
        assign_point(request.bound.min_val, (-2.22, -0.22, -0.3))
        assign_point(request.bound.max_val, (-1.78, 0.22, 0.3))
        previous = state()
        revision = node.revision()['map_revision']
        response = node.call(axis, request)
        assert response.success
        close_vector(point_values(response.bound_ret.min_val), (-2.22, -0.22, -1.5))
        close_vector(point_values(response.bound_ret.max_val), (-1.78, 0.22, 1.5))
        restricted_validation = validate(original.path)
        assert not restricted_validation['valid'] and restricted_validation['reason'] == 'path_not_free', restricted_validation
        restricted = changed('shrink_xy', previous, revision,
                             {'original_path_validation': restricted_validation})
        assert restricted['global_graph']['suspended_edges'] > 0, restricted['global_graph']
        assert restricted['global_graph']['vertices'] == original_graph['vertices']

        request = PlannerSetGlobalBound.Request()
        request.reset_to_default = True
        revision = node.revision()['map_revision']
        assert node.call(axis, request).success
        restored_validation = validate(original.path)
        assert restored_validation['valid'] and restored_validation['reason'] == 'ok', restored_validation
        restored = changed('restore_default', restricted['state'], revision,
                           {'original_path_validation': restored_validation})
        close_vector(restored['world_aabb']['minimum'], DEFAULT_MIN)
        close_vector(restored['world_aabb']['maximum'], DEFAULT_MAX)
        assert restored['global_graph']['suspended_edges'] == 0, restored['global_graph']
        assert restored['global_graph']['last_restored_edges'] > 0, restored['global_graph']
        assert restored['global_graph']['edges'] == original_graph['edges']
        assert restored['global_graph']['vertices'] == original_graph['vertices']

        broadcaster = StaticTransformBroadcaster(node)
        transform = TransformStamped()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'bound_reference'
        transform.header.stamp = node.get_clock().now().to_msg()
        assign_point(transform.transform.translation, TF_TRANSLATION)
        transform.transform.rotation.z = math.sin(TF_YAW / 2.0)
        transform.transform.rotation.w = math.cos(TF_YAW / 2.0)
        broadcaster.sendTransform(transform)
        ready_at = time.monotonic() + 0.6
        node.spin_until(lambda: time.monotonic() >= ready_at)
        request = PlannerDynamicGlobalBound.Request()
        request.header.frame_id = 'bound_reference'
        request.header.stamp = node.get_clock().now().to_msg()
        assign_point(request.center, (-3.0, -1.6, -1.3))
        assign_point(request.front, (3.0, -1.6, -1.3))
        assign_point(request.left, (-3.0, 1.6, -1.3))
        assign_point(request.up, (-3.0, -1.6, 1.3))
        previous = state()
        revision = node.revision()['map_revision']
        assert node.call(dynamic, request).success
        oriented = changed('tf_oriented_bound', previous, revision)
        close_vector(oriented['state']['center'], TF_TRANSLATION)
        close_vector(oriented['state']['half_size'], HALF_SIZE)
        c, s = math.cos(TF_YAW), math.sin(TF_YAW)
        close_vector(oriented['state']['rotation'], [c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0])
        corners = [world_point(values) for values in itertools.product(
            (-HALF_SIZE[0], HALF_SIZE[0]), (-HALF_SIZE[1], HALF_SIZE[1]), (-HALF_SIZE[2], HALF_SIZE[2]))]
        expected_minimum = [min(corner[i] for corner in corners) for i in range(3)]
        expected_maximum = [max(corner[i] for corner in corners) for i in range(3)]
        close_vector(oriented['world_aabb']['minimum'], expected_minimum)
        close_vector(oriented['world_aabb']['maximum'], expected_maximum)
        assert oriented['world_aabb']['minimum'] != list(DEFAULT_MIN)
        verify_robot_box(ROOT)
        candidate, dynamic_metrics = node.plan()
        assert dynamic_metrics['success'], dynamic_metrics
        candidate_path = xyz(candidate.path)
        verify_path(candidate_path, snapshot)
        corner_checks = verify_bound_path(candidate_path)
        dynamic_validation = validate(candidate.path)
        assert dynamic_validation['valid'] and dynamic_validation['reason'] == 'ok', dynamic_validation
        report['dynamic'] = {'tf_translation': TF_TRANSLATION, 'tf_yaw': TF_YAW,
                             'request_frame': request.header.frame_id,
                             'request_corner': point_values(request.center),
                             'request_front': point_values(request.front),
                             'request_left': point_values(request.left),
                             'request_up': point_values(request.up),
                             'expected_world_corners': corners, 'state': oriented['state'],
                             'world_aabb': oriented['world_aabb'], 'metrics': dynamic_metrics,
                             'path': candidate_path, 'validation': dynamic_validation,
                             'independent_robot_corner_checks': corner_checks}
        # The dynamic interface's own reset also restores the original map box.
        reset = PlannerDynamicGlobalBound.Request()
        reset.reset_to_default = True
        previous = state()
        revision = node.revision()['map_revision']
        assert node.call(dynamic, reset).success
        last = changed('dynamic_reset', previous, revision)
        close_vector(last['world_aabb']['minimum'], DEFAULT_MIN)
        close_vector(last['world_aabb']['maximum'], DEFAULT_MAX)
        report.update(status='passed', checks={
            'query_reports_current_world_aabb': True, 'xyz_updates_vertical_range': True,
            'xy_update_preserves_vertical_range': True, 'robot_box_remains_inside': True,
            'old_path_rejected_after_shrink': True, 'old_path_restored_after_reset': True,
            'global_edges_suspend_and_restore': True, 'policy_updates_preserve_observation_revision': True,
            'dynamic_request_uses_tf_rotation_and_translation': True,
            'new_path_robot_boxes_inside_oriented_bound': True, 'dynamic_reset_restores_defaults': True})
        save(args.output, report)
        print(json.dumps({'status': report['status'], 'original_path_m': metrics['path_length_m'],
                          'dynamic_path_m': dynamic_metrics['path_length_m'],
                          'corner_checks': corner_checks, 'output': str(args.output)}), flush=True)
    except Exception as error:
        report.update(status='failed', error=repr(error))
        save(args.output, report)
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        if log is not None:
            log.close()


if __name__ == '__main__':
    main()
