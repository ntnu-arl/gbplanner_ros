"""Execute an estimated-map return path through the existing SMB tracker.

All control and validation inputs are estimated physical B poses in map or
their explicit C = B + ell * e(yaw) transforms. Simulator truth is never read.
The default experiment rotates about the present C, replans from the resulting
B, then executes the new path. The explicit center_dubins alternative plans
new curvature-bounded geometry from the actual estimated starting pose. Both
reuse the existing tracker process and control law.
"""
from __future__ import annotations

import hashlib
import importlib.util
import json
import math
from pathlib import Path
import time
import traceback


def _wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def _yaw(values):
    x, y, z, w = values
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(norm) or norm < 1.e-8:
        raise ValueError('Invalid pose quaternion')
    x, y, z, w = (v / norm for v in values)
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def _state(record):
    values = [*record['position'], _yaw(record['quaternion_xyzw'])]
    if len(values) != 4 or not all(math.isfinite(v) for v in values):
        raise ValueError('Expected a finite XYZYaw pose')
    return values


def _load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def alignment_reference(base, target_yaw, offset_m, angle_step=.025):
    """Dense physical-B arc for a fixed-C turn, including both exact ends."""
    if not (math.isfinite(offset_m) and offset_m > 0 and angle_step > 0):
        raise ValueError('Invalid alignment geometry parameters')
    x, y, z, yaw = base
    center = [x+offset_m*math.cos(yaw), y+offset_m*math.sin(yaw), z]
    delta = _wrap(target_yaw-yaw)
    count = max(1, int(math.ceil(abs(delta)/angle_step)))
    path = []
    for i in range(count+1):
        psi = yaw + delta*i/count
        path.append([center[0]-offset_m*math.cos(psi),
                     center[1]-offset_m*math.sin(psi), z, psi])
    return path, [[*center, yaw+delta]], center


def return_reference(root, raw_path, offset_m, follower_parameters):
    """Densify original B poses, preserve XYZ/final yaw, then transform to C."""
    root = Path(root)
    production = root/'src/gbplanner_execution/scripts/trajectory_tools.py'
    consistency_file = root/'experiments/state_estimation_shadow/consistent_path.py'
    interpolation = _load_module(production, 'mapped_return_trajectory_tools')
    consistency = _load_module(consistency_file, 'mapped_return_consistent_path')
    speed = float(follower_parameters['speed'])
    yaw_rate = float(follower_parameters['yaw_rate'])
    dt = float(follower_parameters['sample_period'])
    dense = interpolation.interpolate(raw_path, speed, yaw_rate, dt,
                                      int(follower_parameters.get('max_points', 20000)))
    base, metadata = consistency.consistent_base_poses(dense, offset_m)
    retained = []
    after = 0
    for vertex in raw_path:
        index = next((i for i in range(after, len(base))
                      if tuple(base[i][:3]) == tuple(vertex[:3])), None)
        if index is None:
            raise ValueError('Interpolation lost a planner B vertex')
        retained.append(index)
        after = index
    if abs(_wrap(base[-1][3]-raw_path[-1][3])) > 1.e-12:
        raise ValueError('Return terminal yaw changed')
    center = [[x+offset_m*math.cos(psi), y+offset_m*math.sin(psi), z, psi]
              for x, y, z, psi in base]
    return base, center, {
        'interpolation_module': str(production),
        'interpolation_sha256': hashlib.sha256(production.read_bytes()).hexdigest(),
        'consistency_sha256': hashlib.sha256(consistency_file.read_bytes()).hexdigest(),
        'speed_mps': speed, 'yaw_rate_rad_s': yaw_rate, 'sample_period_s': dt,
        'planner_vertex_indices': retained, 'all_planner_xyz_retained_in_order': True,
        'terminal_yaw_preserved': True, 'consistency': metadata,
    }


def run_planned_execution(node, root, out, trial, initial_probe, launch_process,
                          offset_m, timeout_s, geometry="original"):
    """Return a persisted outcome; the caller spins and owns child processes.

    ``trial`` must already have completed its fixed-route Action and started
    the unchanged C-point tracker. Each stage has separate event/time bounds.
    Success means all recorded stage Actions succeed, with estimated terminal
    pose and a measured stationary tail. The original geometry uses alignment
    and return stages; center_dubins uses one return stage. Truth is scored offline.
    """
    root, out = Path(root), Path(out)
    record_path = out/'planned_execution.json'
    if record_path.exists():
        raise FileExistsError('Planned execution evidence already exists')
    started = time.monotonic()
    result = {
        'status': 'running', 'success': False, 'truth_used': False,
        'planning_geometry': geometry,
        'scope': 'Estimated-map preturn, real replanning and native physical return execution',
        'source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'offset_m': float(offset_m), 'goal_tolerance_xy_m': .04,
        'goal_tolerance_yaw_rad': .1, 'settle_duration_s': 3.,
        'settle_speed_limit_mps': .01, 'settle_yaw_rate_limit_rad_s': .03,
        'initial_search_revision': initial_probe.get('search_map_revision'),
        'requested_target_xy_m': initial_probe['request']['target']['position'][:2],
        'stages': [], 'validation': {},
        'path_frame_id': 'map', 'command_point': 'C', 'map_and_scoring_point': 'B',
        'decision_note': 'All commands use wheel/gyro estimates. Truth is for separate offline scoring only.',
    }
    validation_client = None
    active_handle = None
    active_result_future = None
    active_goal_future = None
    active_stage = None

    def save():
        result['elapsed_wall_s'] = time.monotonic()-started
        record_path.write_text(json.dumps(result, ensure_ascii=False, indent=2,
                                         allow_nan=False)+'\n')

    def now():
        return node.get_clock().now().nanoseconds

    def estimated():
        message = node.mapping_stage.latest_odometry
        if message is None:
            raise RuntimeError('No estimated B odometry')
        p, q = message.pose.pose.position, message.pose.pose.orientation
        stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        age = (now()-stamp)*1.e-9
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if (message.header.frame_id != 'map' or message.child_frame_id != 'base_link'
                or not all(math.isfinite(v) for v in values) or not -.1 <= age <= .5):
            raise RuntimeError('B odometry is not fresh finite map/base_link data')
        v, w = message.twist.twist.linear, message.twist.twist.angular
        state = [p.x, p.y, p.z, _yaw([q.x, q.y, q.z, q.w])]
        return {'stamp_ns': stamp, 'received_sim_ns': now(), 'base_xyzyaw': state,
                'center_xyzyaw': [p.x+offset_m*math.cos(state[3]),
                                  p.y+offset_m*math.sin(state[3]), p.z, state[3]],
                'planar_speed_mps': math.hypot(v.x, v.y), 'yaw_rate_rad_s': w.z}

    def ros_pose(state):
        from geometry_msgs.msg import Pose
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, state[:3])
        pose.orientation.z = math.sin(state[3]/2)
        pose.orientation.w = math.cos(state[3]/2)
        return pose

    def validate(label, states, observed_first=False, required=True):
        from planner_msgs.srv import ValidatePath
        node.spin_until(validation_client.service_is_ready, label+' service')
        request = ValidatePath.Request()
        request.header.frame_id = 'map'
        request.header.stamp = node.get_clock().now().to_msg()
        request.path = [ros_pose(p) for p in states]
        request.bound_mode = int(initial_probe['bound_mode'])
        request.clearance_margin = 0.
        request.clearance_segments = 0
        request.first_pose_is_observation = observed_first
        evidence = {'request_sim_ns': now(), 'base_reference_xyzyaw': states,
                    'first_pose_is_observation': observed_first,
                    'bound_mode': request.bound_mode, 'clearance_margin_m': 0.}
        result['validation'][label] = evidence
        save()
        future = validation_client.call_async(request)
        node.spin_until(future.done, label+' real ValidatePath')
        reply = future.result()
        if reply is None:
            raise RuntimeError(label+' returned no response')
        evidence['response'] = {'valid': bool(reply.valid), 'reason': reply.reason,
                                'map_revision': int(reply.map_revision),
                                'failure_segment': int(reply.failure_segment)}
        evidence['response_sim_ns'] = now()
        save()
        if not reply.valid and required:
            raise RuntimeError(label+' rejected: '+reply.reason)
        return bool(reply.valid)

    def stop_command():
        node.command = (0., 0.)
        node.command_enabled = True
        node.publish_command()

    def wait_motion(predicate, beginning, label):
        node.spin_until(lambda: predicate() or (now()-beginning)*1.e-9 > timeout_s, label)
        if not predicate():
            raise TimeoutError(label+' exceeded simulation-time budget')

    def execute(stage):
        nonlocal active_handle, active_result_future, active_goal_future, active_stage
        from geometry_msgs.msg import PoseStamped
        from planner_msgs.action import PathFollower
        active_stage = stage
        active_handle = active_result_future = active_goal_future = None
        stage['estimate_before_action'] = estimated()
        node.set_phase('planned_'+stage['kind'])
        stage['start_sim_ns'] = now()
        goal = PathFollower.Goal()
        goal.manual_path = True
        goal.vel_max = float(trial.speed)
        goal.bound_mode = int(initial_probe['bound_mode'])
        for state in stage['center_reference_xyzyaw']:
            point = PoseStamped()
            point.header.frame_id = 'map'
            point.header.stamp = node.get_clock().now().to_msg()
            point.pose = ros_pose(state)
            goal.path.append(point)
        stage['feedback_messages'] = 0

        def feedback(message):
            stage['feedback_messages'] += 1
            item = message.feedback
            stage['last_feedback'] = {'remaining_waypoints': int(item.remaining_waypoints),
                                     'dist_to_goal': float(item.dist_to_goal),
                                     'estimated_time_remaining': float(item.estimated_time_remaining)}

        save()
        node.command_enabled = False
        active_goal_future = trial.client.send_goal_async(goal, feedback_callback=feedback)
        wait_motion(active_goal_future.done, stage['start_sim_ns'], stage['kind']+' Action acceptance')
        active_handle = active_goal_future.result()
        stage['action_result'] = {'accepted': bool(active_handle and active_handle.accepted),
                                  'success': False, 'action_status': None}
        if active_handle is None or not active_handle.accepted:
            stage['action_result_sim_ns'] = now()
            node.record('planned_action_result', stamp_ns=now(), stage_kind=stage['kind'],
                        result=stage['action_result'])
            raise RuntimeError(stage['kind']+' Action rejected')
        active_result_future = active_handle.get_result_async()
        wait_motion(active_result_future.done, stage['start_sim_ns'], stage['kind']+' Action completion')
        reply = active_result_future.result()
        if reply is None:
            raise RuntimeError(stage['kind']+' Action returned no result')
        stage['action_result'].update(action_status=int(reply.status), success=bool(reply.result.success))
        stage['action_result_sim_ns'] = now()
        stage['action_duration_sim_s'] = (now()-stage['start_sim_ns'])*1.e-9
        stage['estimate_at_action'] = estimated()
        node.record('planned_action_result', stamp_ns=now(), stage_kind=stage['kind'],
                    result=stage['action_result'])
        stop_command()
        save()
        if reply.status != 4 or not reply.result.success:
            raise RuntimeError(stage['kind']+' Action did not succeed')
        node.set_phase('planned_'+stage['kind']+'_settle')
        stage['settle_start_sim_ns'] = now()
        stage['stopped_start_sim_ns'] = stage['settle_start_sim_ns']
        tail = []
        end = now()+3_000_000_000

        def settled_time():
            observation = estimated()
            if not tail or observation['stamp_ns'] != tail[-1]['stamp_ns']:
                tail.append(observation)
            return now() >= end

        node.spin_until(settled_time, stage['kind']+' three-second stationary tail')
        stage['settled_end_sim_ns'] = now()
        stage['estimate_after_settle'] = estimated()
        final = stage['estimate_after_settle']['base_xyzyaw']
        target = stage['target_base_xyzyaw']
        selected = [r for r in tail if r['stamp_ns'] >= stage['settled_end_sim_ns']-1_000_000_000]
        if len(selected) < 10:
            raise RuntimeError('Insufficient fresh estimates in stationary tail')
        xy_error = math.dist(final[:2], target[:2])
        yaw_error = _wrap(final[3]-target[3])
        speed = max(r['planar_speed_mps'] for r in selected)
        yaw_rate = max(abs(r['yaw_rate_rad_s']) for r in selected)
        stage['estimated_acceptance'] = {
            'position_error_m': xy_error, 'yaw_error_rad': yaw_error,
            'last_second_samples': len(selected), 'last_second_max_planar_speed_mps': speed,
            'last_second_max_abs_yaw_rate_rad_s': yaw_rate,
            'pose_within_tolerance': xy_error <= .04 and abs(yaw_error) <= .1,
            'stationary': speed <= .01 and yaw_rate <= .03,
        }
        stage['success'] = (stage['estimated_acceptance']['pose_within_tolerance']
                            and stage['estimated_acceptance']['stationary'])
        stage['end_sim_ns'] = now()
        save()
        if not stage['success']:
            raise RuntimeError(stage['kind']+' estimated terminal/stop criteria not met')

    try:
        import yaml
        from planner_msgs.srv import ValidatePath
        from planner_probe import run_planner_probe
        if not (math.isfinite(timeout_s) and timeout_s > 0 and math.isfinite(offset_m) and offset_m > 0):
            raise ValueError('Invalid execution bounds')
        if trial.control_point != 'center' or trial.source != 'ekf_slip':
            raise ValueError('Planned return requires the existing estimated C tracker')
        preceding = trial.result()
        if not initial_probe.get('success') or not preceding.get('success') or preceding.get('action_status') != 4:
            raise RuntimeError('Initial physical route and planner probe must succeed first')
        if not trial.ready():
            raise RuntimeError('Existing tracking Action server is unavailable')
        if not node.command_enabled:
            raise RuntimeError('Caller must own stopped commands before planned execution')
        config = yaml.safe_load((root/'src/gbplanner_tracking/config/smb.yaml').read_text())
        follower = config['gbplanner_path_follower']['ros__parameters']
        validation_client = node.create_client(ValidatePath, '/validate_path')
        first_raw = [_state(p) for p in initial_probe['search_response']['path']]
        if geometry == 'center_dubins':
            from center_execution import prepare, stage as make_center_stage
            result['scope'] = 'Independent fixed-center connector on estimated map; existing Action execution'
            node.set_phase('planned_center_construction')
            candidates, evidence = prepare(node, root, out, initial_probe,
                estimated()['base_xyzyaw'], first_raw, offset_m, follower)
            result['center_planning'] = evidence
            chosen = None
            node.set_phase('planned_center_validation')
            for index, candidate in enumerate(candidates):
                base = candidate['base_reference_xyzyaw']
                full_ok = validate('center_'+str(index)+'_full_path', base, required=False)
                connection_ok = validate('center_'+str(index)+'_estimated_connection',
                    [estimated()['base_xyzyaw'], *base], observed_first=True, required=False)
                if full_ok and connection_ok:
                    chosen = candidate
                    result['selected_candidate_index'] = index
                    break
            if chosen is None:
                raise RuntimeError('No fixed-radius candidate passed native terrain and live ROS validation')
            stage = make_center_stage(chosen, estimated(), first_raw,
                                      initial_probe['search_map_revision'])
            if stage['initial_center_gap_m'] > .04 or abs(stage['initial_yaw_gap_rad']) > .1:
                raise RuntimeError('Estimate moved away from the planned start during construction')
            result['stages'].append(stage)
            active_stage = stage
            execute(stage)
        elif geometry == 'original':
            initial_consistent, _, first_sampling = return_reference(root, first_raw, offset_m, follower)
            observation = estimated()
            base, center, anchor = alignment_reference(observation['base_xyzyaw'], initial_consistent[0][3], offset_m)
            alignment = {'kind': 'alignment', 'base_reference_xyzyaw': base,
                         'center_reference_xyzyaw': center, 'center_anchor_xyz': anchor,
                         'target_yaw_rad': center[-1][3], 'target_base_xyzyaw': base[-1],
                         'yaw_change_rad': _wrap(center[-1][3]-base[0][3]),
                         'initial_search_heading_construction': first_sampling,
                         'meaning': 'Fixed estimated C anchor, physical B follows a modeled circular arc; not a fixed-B in-place turn'}
            result['stages'].append(alignment)
            active_stage = alignment
            node.set_phase('planned_alignment_validation')
            validate('alignment_base_arc', base, observed_first=True)
            execute(alignment)

            node.set_phase('planned_return_search')
            result['replanning_start_sim_ns'] = now()
            replan = run_planner_probe(node, root, out/'planned_return_search', launch_process,
                                      node.spin_until, lambda: node.mapping_stage.latest_odometry,
                                      start_planner=False,
                                      target_xy=initial_probe['request']['target']['position'][:2],
                                      search_policy=initial_probe.get('search_policy', 'baseline'))
            result['return_search'] = replan
            result['replanning_end_sim_ns'] = now()
            save()
            if not replan.get('success'):
                raise RuntimeError('Post-turn real search failed: '+str(replan.get('reason')))
            if replan.get('exact_target_reported') is not True:
                raise RuntimeError('Post-turn search reaches only a target neighborhood; '
                                   'requested-goal execution requires an exact-target path')
            raw = [_state(p) for p in replan['search_response']['path']]
            base, center, sampling = return_reference(root, raw, offset_m, follower)
            current = estimated()
            stage = {'kind': 'return', 'raw_planner_base_xyzyaw': raw,
                     'base_reference_xyzyaw': base, 'center_reference_xyzyaw': center,
                     'target_base_xyzyaw': base[-1], 'sampling': sampling,
                     'search_map_revision': replan['search_map_revision'],
                     'initial_center_gap_m': math.dist(current['center_xyzyaw'][:2], center[0][:2]),
                     'initial_yaw_gap_rad': _wrap(center[0][3]-current['base_xyzyaw'][3]),
                     'meaning': 'All new planner B XYZ vertices and terminal yaw retained; intermediate yaw satisfies the fixed-C model'}
            result['stages'].append(stage)
            active_stage = stage
            node.set_phase('planned_return_validation')
            validate('return_dense_base_reference', base)
            validate('return_estimated_base_connection', [estimated()['base_xyzyaw'], *base], observed_first=True)
            execute(stage)
        else:
            raise ValueError('Unknown independent planning geometry')
        result['success'] = True
        result['status'] = 'passed'
        result['reason'] = 'planned_return_executed_with_estimated_terminal_and_stop_acceptance'
        result['planned_path_executed'] = True
    except Exception as error:
        result['status'], result['success'] = 'failed', False
        result['error'] = repr(error)
        result['traceback'] = traceback.format_exc()
        result['failure_phase'] = node.phase
        if active_stage is not None and not active_stage.get('success'):
            active_stage['failure_sim_ns'] = now()
            active_stage['end_sim_ns'] = now()
            if 'start_sim_ns' in active_stage and 'action_result_sim_ns' not in active_stage:
                interrupted = active_stage.setdefault('action_result',
                    {'accepted': None, 'action_status': None, 'success': False})
                interrupted.update(done=False, interrupted=True, error=repr(error))
                node.record('planned_action_result', stamp_ns=now(), stage_kind=active_stage['kind'],
                            result=interrupted)
    finally:
        # The runner shuts down owned processes after failure. Request cancel
        # first, then publish zero; a missed deadline cannot bypass that stop.
        if not result['success']:
            stop_command()
            if active_stage is not None:
                active_stage.setdefault('stopped_start_sim_ns', now())
        if active_goal_future is not None and active_handle is None and active_goal_future.done():
            try:
                active_handle = active_goal_future.result()
            except Exception:
                pass
        if (active_handle is not None and active_handle.accepted
                and (active_result_future is None or not active_result_future.done())):
            try:
                cancellation = active_handle.cancel_goal_async()
                node.spin_until(cancellation.done, 'planned Action cancellation')
                result['cancellation'] = {'response_received': True,
                                          'return_code': int(cancellation.result().return_code)}
            except Exception as error:
                result['cancellation'] = {'response_received': False, 'error': repr(error)}
        stop_command()
        if validation_client is not None:
            node.destroy_client(validation_client)
        result['end_sim_ns'] = now()
        save()
    return result
