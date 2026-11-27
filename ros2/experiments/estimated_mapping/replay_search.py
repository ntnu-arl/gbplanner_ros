#!/usr/bin/env python3
"""Replay a saved estimated map through the installed ROS search and validator.

No simulator, estimator or controller is launched. Graph and tree cases use
identical recorded fields, source pose, seed, geometry and resource limits.
The saved snapshot is not asserted to be byte-identical to online revision 31.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[2]
FIELDS = ('x', 'y', 'z', 'occupancy', 'tsdf_distance', 'tsdf_weight')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, data):
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2,
                                    allow_nan=False) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path,
                        default=ROOT/'results/estimated_mapping/run7_multiround')
    parser.add_argument('--output', type=Path,
                        default=ROOT/'results/estimated_mapping/search_budget_replay')
    parser.add_argument('--case', choices=('graph', 'tree', 'first_target', 'refined_target'))
    parser.add_argument('--domain', type=int, default=191)
    parser.add_argument('--probe-relative-path', default='round2/planner_probe.json')
    parser.add_argument('--center-connector', action='store_true',
                        help='Also evaluate fixed-radius C candidates against the recorded endpoint pose')
    parser.add_argument('--via-result', type=Path,
                        help='Validate saved native-supported single-via candidates on the same map')
    args = parser.parse_args()
    run, out = args.run_dir.resolve(), args.output.resolve()
    if not 0 <= args.domain < 232:
        parser.error('domain must be in [0, 231]')
    if args.case is None:
        out.mkdir(parents=True, exist_ok=False)
        cases = {}
        for index, case in enumerate(('graph', 'tree')):
            command = [sys.executable, str(Path(__file__).resolve()),
                       '--run-dir', str(run), '--output', str(out),
                       '--case', case, '--domain', str(args.domain + index),
                       '--probe-relative-path', args.probe_relative_path]
            if args.center_connector:
                command.append('--center-connector')
            if args.via_result:
                command.extend(['--via-result', str(args.via_result.resolve())])
            result = subprocess.run(command, check=False)
            cases[case] = json.loads((out/case/'result.json').read_text())
            if result.returncode:
                break
        summary = {'scope': __doc__, 'cases': cases,
                   'source_sha256': sha(__file__),
                   'status': 'completed' if len(cases) == 2 and all(
                       c['status'] == 'completed' for c in cases.values()) else 'failed'}
        if len(cases) == 2:
            left, right = cases['graph'], cases['tree']
            summary['only_algorithm_parameter_difference'] = {
                k: [left['parameters'].get(k), right['parameters'].get(k)]
                for k in sorted(set(left['parameters']) | set(right['parameters']))
                if left['parameters'].get(k) != right['parameters'].get(k)}
            summary['both_received_identical_saved_fields'] = all(
                c.get('accepted_map_fields_identical') is True for c in cases.values())
        save(out/'comparison.json', summary)
        print(json.dumps({'status': summary['status'], 'output': str(out/'comparison.json')}))
        return 0 if summary['status'] == 'completed' else 1

    case_dir = out/args.case
    case_dir.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_LOG_DIR'] = str(case_dir/'ros_logs')
    os.environ['PYTHONDONTWRITEBYTECODE'] = '1'
    started = time.monotonic()
    deadline = started + 45.
    node = process = logfile = None
    initialized = False
    report = {'status': 'failed', 'case': args.case, 'domain': args.domain,
              'scope': __doc__, 'synthetic_map': False, 'robot_motion': False,
              'snapshot_limit': 'Saved after_execution snapshot is replayed. Its identity '
              'with the online query map is not asserted; matching cell counts alone do not prove it.',
              'source_sha256': sha(__file__)}
    try:
        import numpy as np
        import yaml
        import rclpy
        from ament_index_python.packages import get_package_prefix
        from geometry_msgs.msg import Pose
        from nav_msgs.msg import Odometry, Path as RosPath
        from planner_msgs.msg import Graph
        from planner_msgs.srv import PlannerSearch, ValidatePath
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from rosidl_runtime_py.convert import message_to_ordereddict
        from sensor_msgs.msg import PointCloud2, PointField
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        probe_path = run/args.probe_relative_path
        probe = json.loads(probe_path.read_text())
        source_pose = probe['request']['source_ignored_by_service']
        target_pose = probe['request']['target']
        map_path = run/'after_execution/voxel_map.npz'
        map_metadata_path = run/'after_execution/mapping_summary.json'
        map_metadata = json.loads(map_metadata_path.read_text())['products']['voxel_map']
        archived = np.load(map_path)
        arrays = {k: archived[k] for k in FIELDS}
        count = len(arrays['x'])
        if (set(archived.files) != set(FIELDS) or count != map_metadata['points']
                or sha(map_path) != map_metadata['sha256']):
            raise ValueError('Saved map schema, recorded cell count or archived hash differs')
        dtype = np.dtype({'names': FIELDS,
                          'formats': ['<f4', '<f4', '<f4', 'i1', '<f4', '<f4'],
                          'offsets': [0, 4, 8, 12, 16, 20], 'itemsize': 24})
        packed = np.zeros(count, dtype=dtype)
        for key in FIELDS:
            packed[key] = arrays[key]
        configuration = probe_path.with_suffix('.yaml')
        parameters = yaml.safe_load(configuration.read_text())['gbplanner_local']['ros__parameters']
        parameters.update({'use_sim_time': False, 'map.use_update_envelope': False,
                           'planning.graph_mode': args.case == 'graph'})
        if args.case in ('first_target', 'refined_target'):
            # Existing SearchPlanner stops after candidates.size() exceeds
            # this count. Zero reserves the remaining vertex slots for its
            # existing exact-target expansion after the first nearby sample.
            parameters['search.max_target_candidates'] = 0
        if args.case == 'refined_target':
            parameters['search.refine_path'] = True
        config_path = case_dir/'planner.yaml'
        config_path.write_text(yaml.safe_dump({'gbplanner_local': {'ros__parameters': parameters}},
                                             sort_keys=False))
        prefix = Path(get_package_prefix('gbplanner_local'))
        executable = prefix/'lib/gbplanner_local/local_planner_node'
        report.update(parameters=parameters, source_pose=source_pose, target_pose=target_pose,
                      saved_cells=count, inputs_sha256={str(p): sha(p) for p in
                      (probe_path, map_path, configuration)},
                      installed_executable=str(executable), installed_sha256=sha(executable),
                      original_online_metrics=probe['search_metrics'],
                      saved_map_metadata=map_metadata,
                      request_pose_note='Explicit recorded source replaces live use_current_state; '
                      'both replay cases use these exact position and quaternion fields.')
        rclpy.init(args=[])
        initialized = True
        node = Node('saved_estimated_map_search_replay')
        durable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        accepted, metrics, graphs, paths = [], [], [], []
        maps = node.create_publisher(PointCloud2, '/voxel_map', durable)
        odometry = node.create_publisher(Odometry, '/odometry', 10)
        node.create_subscription(PointCloud2, '/accepted_voxel_map', accepted.append, durable)
        node.create_subscription(String, '/search_metrics',
                                 lambda m: metrics.append(json.loads(m.data)), durable)
        node.create_subscription(Graph, '/search_graph', graphs.append, durable)
        node.create_subscription(RosPath, '/search_path', paths.append, durable)
        search = node.create_client(PlannerSearch, '/planner_search')
        validate = node.create_client(ValidatePath, '/validate_path')
        status = node.create_client(Trigger, '/map_status')
        sensor_state = node.create_client(Trigger, '/sensor_gain')
        command = [str(executable), '--ros-args', '--params-file', str(config_path)]
        logfile = (case_dir/'planner.log').open('w')
        process = subprocess.Popen(command, stdout=logfile, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        report['command'] = command

        def until(predicate, label):
            while not predicate():
                if time.monotonic() >= deadline:
                    raise TimeoutError(label)
                if process.poll() is not None:
                    raise RuntimeError('Planner exited while waiting for ' + label)
                rclpy.spin_once(node, timeout_sec=.02)

        def call(client, request, label):
            until(client.service_is_ready, label+' discovery')
            # Queries are read-only. One established client is retained during
            # bounded retries if an initial FastDDS response match is delayed.
            for attempt in range(3):
                future = client.call_async(request)
                response_deadline = min(deadline, time.monotonic()+8.)
                until(lambda: future.done() or time.monotonic() >= response_deadline,
                      label+' response')
                if future.done():
                    return future.result()
                client.remove_pending_request(future)
                future.cancel()
                report.setdefault('query_response_timeouts', []).append(
                    {'label': label, 'attempt': attempt+1})
            raise TimeoutError(label+' response missing after three bounded attempts')

        def pose(value):
            m = Pose()
            m.position.x, m.position.y, m.position.z = value['position']
            q = value['quaternion_xyzw']
            m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = q
            return m

        def publish_recorded_observation():
            message = Odometry()
            message.header.frame_id = 'map'
            message.child_frame_id = 'base_link'
            message.header.stamp = node.get_clock().now().to_msg()
            message.pose.pose = pose(source_pose)
            odometry.publish(message)

        node.create_timer(.03, publish_recorded_observation)
        report['validation_observation'] = ('Fixed recorded estimated source pose, '
            'republished with fresh replay transport timestamps; no simulated truth or motion.')

        until(lambda: maps.get_subscription_count() and odometry.get_subscription_count()
              and all(c.service_is_ready() for c in (search, validate, status, sensor_state)),
              'planner and map discovery')
        cloud = PointCloud2()
        cloud.header.frame_id = 'map'
        cloud.header.stamp = node.get_clock().now().to_msg()
        cloud.width, cloud.height, cloud.point_step, cloud.row_step = count, 1, 24, 24*count
        cloud.is_dense, cloud.is_bigendian = True, False
        cloud.fields = [PointField(name=name, offset=offset,
                        datatype=PointField.INT8 if name == 'occupancy' else PointField.FLOAT32,
                        count=1) for name, offset in zip(FIELDS, (0,4,8,12,16,20))]
        cloud.data = packed.tobytes()
        maps.publish(cloud)
        until(lambda: bool(accepted), 'accepted saved map')
        received = accepted[-1]
        if (received.width, received.height, received.point_step, received.row_step,
                received.is_bigendian, received.fields) != (
                cloud.width, cloud.height, cloud.point_step, cloud.row_step,
                cloud.is_bigendian, cloud.fields):
            raise AssertionError('Accepted cloud layout changed')
        decoded = np.frombuffer(bytes(received.data), dtype=dtype)
        comparisons = {name: bool(np.array_equal(decoded[name], arrays[name])) for name in FIELDS}
        report['accepted_field_comparison'] = comparisons
        report['accepted_map_fields_identical'] = all(comparisons.values())
        if not report['accepted_map_fields_identical']:
            raise AssertionError('Accepted map differs from saved values')
        report['accepted_map_data_sha256'] = hashlib.sha256(bytes(received.data)).hexdigest()
        report['map_status'] = json.loads(call(status, Trigger.Request(), 'accepted map state').message)
        while True:
            state_response = call(sensor_state, Trigger.Request(), 'recorded observation readiness')
            observed = json.loads(state_response.message)
            if state_response.success and observed.get('odometry_stamp_ns', 0) > 0:
                report['observed_source_state'] = observed
                break
            ready_at = time.monotonic()+.05
            until(lambda: time.monotonic() >= ready_at, 'observation discovery interval')
        request = PlannerSearch.Request()
        request.header.frame_id = 'map'
        request.header.stamp = node.get_clock().now().to_msg()
        request.use_current_state = False
        request.bound_mode = request.K_EXACT_BOUND
        request.source, request.target = pose(source_pose), pose(target_pose)
        before = len(metrics)
        response = call(search, request, 'saved-map search')
        until(lambda: len(metrics) > before and graphs and paths, 'actual search outputs')
        report['search'] = metrics[-1]
        graph, path = graphs[-1], paths[-1]
        report['graph'] = message_to_ordereddict(graph)
        report['path'] = [message_to_ordereddict(p) for p in response.path]
        report['path_topic'] = message_to_ordereddict(path)
        if len(graph.vertices) != report['search']['vertices'] or len(graph.edges) != report['search']['edges']:
            raise AssertionError('Graph counts differ from search metrics')
        if [p.pose for p in path.poses] != list(response.path):
            raise AssertionError('Service and topic paths differ')
        tx, ty = target_pose['position'][:2]
        distances = [(math.hypot(v.pose.position.x-tx, v.pose.position.y-ty), v.id)
                     for v in graph.vertices]
        report['closest_graph_vertex_xy'] = list(min(distances)) if distances else None
        report['target_neighborhood_graph_vertices'] = [
            {'id': v.id,
             'xy_distance_m': math.hypot(v.pose.position.x-tx, v.pose.position.y-ty),
             'planning_pose': message_to_ordereddict(v.pose)}
            for v in graph.vertices
            if math.hypot(v.pose.position.x-tx, v.pose.position.y-ty) < parameters['search.target_radius']]
        if response.path:
            endpoint = response.path[-1].position
            report['returned_endpoint_request_error'] = {
                'xy_m': math.hypot(endpoint.x-tx, endpoint.y-ty),
                'xyz_m': math.dist([endpoint.x, endpoint.y, endpoint.z], target_pose['position']),
                'height_projection_difference_m': endpoint.z-target_pose['position'][2]}
        if args.case == 'refined_target':
            from planned_execution import return_reference, _state
            follower = yaml.safe_load((ROOT/'src/gbplanner_tracking/config/smb.yaml').read_text())[
                'gbplanner_path_follower']['ros__parameters']
            offset = json.loads((run/'run.json').read_text())['turning_center_offset_m']
            report['refinement_contract'] = (
                'Planner refinement may change intermediate XYZ and recomputes tangent yaw, '
                'including terminal yaw. Search target orientation was not an endpoint constraint. '
                'Execution conversion must retain all refined output XYZ and its terminal yaw.')
            report['forward_reference_checks'] = {}
            reference_paths = {'recorded_online_raw': [_state(v) for v in probe['search_response']['path']]}
            if response.path:
                reference_paths['refined_saved_map'] = [_state({
                    'position': [v.position.x, v.position.y, v.position.z],
                    'quaternion_xyzw': [v.orientation.x, v.orientation.y, v.orientation.z, v.orientation.w]})
                    for v in response.path]
            for label, original in reference_paths.items():
                check = {'input_path': original, 'offset_m': offset,
                         'terminal_yaw_rad': original[-1][3]}
                try:
                    base, center, metadata = return_reference(ROOT, original, offset, follower)
                    check.update(success=True, metadata=metadata, base_path=base, center_path=center)
                except ValueError as error:
                    check.update(success=False, error=str(error))
                report['forward_reference_checks'][label] = check
        report['graph_stop_conditions'] = {
            'edge_limit': len(graph.edges) >= parameters['search.max_edges'],
            'vertex_limit': len(graph.vertices) >= parameters['search.max_vertices'],
            'draw_limit': report['search']['attempts'] >= parameters['search.max_attempts']}
        report['validation'] = {}
        for label, poses in [('source', [request.source]), ('target', [request.target]),
                             ('direct_chord', [request.source, request.target]),
                             *([('returned_path', list(response.path))] if response.success else [])]:
            check = ValidatePath.Request()
            check.header.frame_id = 'map'
            check.header.stamp = node.get_clock().now().to_msg()
            check.bound_mode = request.K_EXACT_BOUND
            check.clearance_margin = 0.
            check.path = poses
            reply = call(validate, check, label+' validation')
            report['validation'][label] = message_to_ordereddict(reply)
        if args.center_connector or args.via_result:
            from center_path import construct
            from planned_execution import _state
            follower = yaml.safe_load((ROOT/'src/gbplanner_tracking/config/smb.yaml').read_text())[
                'gbplanner_path_follower']['ros__parameters']
            offset = json.loads((run/'run.json').read_text())['turning_center_offset_m']
            # The real recorded planner terminal orientation is fixed, even if
            # a fresh replay search chooses a different tangent at the goal.
            if args.via_result:
                connector = json.loads(args.via_result.read_text())
                (case_dir/'center_connector').mkdir()
                # Refuse to combine saved candidate geometry with a different
                # observed map. The native request binds its complete fields.
                contract = json.loads(Path(connector['native']['request']).read_text())
                if contract['map']['sha256'] != hashlib.sha256(packed.tobytes()).hexdigest():
                    raise ValueError('Via result map differs from replayed observed fields')
            else:
                connector = construct(ROOT, case_dir/'center_connector', _state(source_pose),
                    _state(probe['search_response']['path'][-1]), offset, follower,
                    parameters, arrays, until)
            connector['ros_validation'] = {}
            connector['selected_family'] = None
            for candidate in connector['feasible']:
                states = candidate['base_reference_xyzyaw']
                checks = {}
                for label, rows, observed_first in (
                        ('full_path', states, False),
                        ('estimated_connection', [_state(source_pose), *states], True)):
                    check = ValidatePath.Request()
                    check.header.frame_id = 'map'
                    check.header.stamp = node.get_clock().now().to_msg()
                    check.bound_mode = request.K_EXACT_BOUND
                    check.first_pose_is_observation = observed_first
                    check.path = [pose({'position': p[:3], 'quaternion_xyzw':
                        [0., 0., math.sin(p[3]/2.), math.cos(p[3]/2.)]}) for p in rows]
                    checks[label] = message_to_ordereddict(call(validate, check,
                        candidate['family']+' '+label+' native ROS validation'))
                connector['ros_validation'][candidate['family']] = checks
                if all(c['valid'] for c in checks.values()) and connector['selected_family'] is None:
                    connector['selected_family'] = candidate['family']
            save(case_dir/'center_connector/result.json', connector)
            report['center_connector'] = {k: connector.get(k) for k in (
                'radius_m', 'start_base_xyzyaw', 'target_base_xyzyaw',
                'selected_family', 'ros_validation', 'mathematically_rejected')}
            report['center_connector']['native_candidates'] = [{
                k: c[k] for k in ('id','success','reason','stage','failure_index')}
                for c in connector['native'].get('candidates', [])]
            if args.via_result:
                report['center_connector']['via_search_counts'] = connector['search_counts']
        report['status'] = 'completed'
    except Exception as error:
        report['error'] = repr(error)
        report['traceback'] = traceback.format_exc()
    finally:
        if process is not None:
            forced = False
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    forced = True
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=2.)
            report['shutdown'] = {'returncode': process.returncode, 'forced': forced}
            if forced or process.returncode != 0:
                report['status'] = 'failed'
        if logfile is not None:
            logfile.close()
        if node is not None:
            node.destroy_node()
        if initialized:
            rclpy.shutdown()
        report['elapsed_wall_s'] = time.monotonic()-started
        save(case_dir/'result.json', report)
    print(json.dumps({k: report.get(k) for k in ('case','status','search','error','shutdown')}))
    return 0 if report['status'] == 'completed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
