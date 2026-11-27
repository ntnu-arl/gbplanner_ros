#!/usr/bin/env python3
"""Reconstruct and explain the recorded native10 crossing execution timing.

Reads saved evidence only. Reconstructed candidates are checked against every
published pose before timing statistics are produced. This is a post-hoc
reconstruction, not a recovered historical Action wire snapshot, map check or
new physical experiment. No ROS import, command publication or simulator run.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/gbplanner_execution/scripts'))
from ground_maneuver import Limits
from ground_route_obligations import construct_route_obligations


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def relative(path):
    try:
        return str(path.resolve().relative_to(ROOT))
    except ValueError:
        return str(path.resolve())


def stats(values):
    if not values:
        return None
    return dict(count=len(values), minimum=min(values), maximum=max(values),
                mean=statistics.mean(values), median=statistics.median(values),
                population_stdev=statistics.pstdev(values))


def source_time(row):
    return row['stamp_ns'] * 1.e-9


def event_time(row):
    return row['ground_maneuver']['motion_time_s']


def endpoint_error(pose, target):
    return dict(xy_m=math.dist(pose[:2], target[:2]),
                height_m=abs(pose[2] - target[2]),
                yaw_rad=abs(math.remainder(pose[3] - target[3], 2. * math.pi)))


def endpoint_observation(history, event):
    """Use the actual source stamp reported by tracking, never interpolating."""
    stamp = event.get('odometry_stamp_s')
    if stamp is None:
        raise ValueError('tracking event has no source odometry stamp')
    rows = [row for row in history['odometry'] if abs(source_time(row) - stamp) < 1.e-8]
    if len(rows) != 1:
        raise ValueError('tracking source odometry is absent or ambiguous')
    row = rows[0]
    return dict(source_stamp_s=source_time(row), pose=row['pose'],
                body_linear_velocity=row['body_linear_velocity'], yaw_rate_radps=row['yaw_rate'])


def plateau_spans(rows, start, end, desired, maximum_gap):
    """Maximal consecutive runs at the declared cap, bounded by one primitive."""
    spans, active = [], []
    for row in rows:
        if not start <= row['sim_s'] < end:
            continue
        saturated = abs(row['angular_z'] - desired) <= 1.e-10
        continuous = not active or row['sim_s'] - active[-1]['sim_s'] <= maximum_gap
        if not saturated or not continuous:
            if active:
                spans.append(active)
            active = []
        if saturated:
            active.append(row)
    if active:
        spans.append(active)
    return spans


def platform_statistics(history, primitives, starts, stop_time, limits):
    # Use one nominal angular acceleration ramp at each end of every maximal
    # saturated command run. This is fixed from declared limits, not selected
    # by inspecting the observed yaw response. Small/fragmented runs are kept
    # as rejected selections, so the resulting sample population is explicit.
    trim = limits.max_yaw_rate / limits.max_yaw_acceleration
    minimum_window = 1.
    maximum_gap = 2. * limits.integration_dt
    accepted, rejected = [], []
    for index, primitive in enumerate(primitives):
        if primitive.kind != 'pivot' or index not in starts:
            continue
        begin = event_time(starts[index])
        end = event_time(starts[index + 1]) if index + 1 in starts else stop_time
        desired = math.copysign(limits.max_yaw_rate, primitive.signed_amount)
        spans = plateau_spans(history['commands'], begin, end, desired, maximum_gap)
        for span in spans:
            lower, upper = span[0]['sim_s'], span[-1]['sim_s']
            a, b = lower + trim, upper - trim
            selection = dict(primitive_index=index, saturated_command_radps=desired,
                             raw_first_receipt_ros_s=lower, raw_last_receipt_ros_s=upper,
                             raw_command_samples=len(span), trim_each_end_s=trim,
                             selected_interval_ros_s=[a, b])
            if b - a < minimum_window:
                rejected.append(dict(selection, reason='trimmed_interval_shorter_than_minimum'))
                continue
            commands = [row for row in history['commands'] if a <= row['sim_s'] <= b]
            limited = [row for row in history['controller_limited_commands']
                       if a <= source_time(row) <= b]
            odometry = [row for row in history['odometry'] if a <= source_time(row) <= b]
            joints = [row for row in history['joints'] if a <= row['sim_s'] <= b]
            if not commands or not limited or len(odometry) < 2:
                raise ValueError('selected command plateau lacks source evidence')
            if any(abs(row['angular_z'] - desired) > 1.e-10 for row in commands + limited):
                raise ValueError('selected plateau is not saturated in both command streams')
            stamps = [source_time(row) for row in odometry]
            if any(right <= left for left, right in zip(stamps, stamps[1:])):
                raise ValueError('source odometry timestamps do not strictly increase')
            deltas = [math.remainder(right['pose'][3] - left['pose'][3], 2. * math.pi)
                      for left, right in zip(odometry, odometry[1:])]
            gaps = [right - left for left, right in zip(stamps, stamps[1:])]
            pose_rates = [delta / dt for delta, dt in zip(deltas, gaps)]
            duration = stamps[-1] - stamps[0]
            pose_average = math.fsum(deltas) / duration
            yaw_rates = [row['yaw_rate'] for row in odometry]
            trapezoid = math.fsum(.5 * (left + right) * dt
                                 for left, right, dt in zip(yaw_rates, yaw_rates[1:], gaps))
            wheel = None
            if joints:
                expected_names = ['LF_wheel_joint', 'RF_wheel_joint',
                                  'LH_wheel_joint', 'RH_wheel_joint']
                if any(row['names'] != expected_names for row in joints):
                    raise ValueError('wheel joint ordering differs from declared model')
                means = [statistics.mean(row['velocity'][i] for row in joints) for i in range(4)]
                # Nominal values from the hashed controllers.yaml. This is a
                # wheel-kinematics calculation, never substituted for body yaw.
                nominal_yaw = ((means[1] + means[3]) - (means[0] + means[2])) * .19 / (2. * .6604)
                wheel = dict(samples=len(joints), joint_names=expected_names,
                             mean_joint_velocity_radps=means, nominal_radius_m=.19,
                             nominal_separation_m=.6604, nominal_body_yaw_rate_radps=nominal_yaw,
                             measured_body_to_nominal_wheel_yaw_ratio=pose_average / nominal_yaw)
            accepted.append(dict(selection,
                command_yaw_rate_radps=stats([row['angular_z'] for row in commands]),
                controller_limited_yaw_rate_radps=stats([row['angular_z'] for row in limited]),
                source_odometry=dict(first_stamp_s=stamps[0], last_stamp_s=stamps[-1],
                    duration_s=duration, stamp_gap_s=stats(gaps),
                    twist_yaw_rate_radps=stats(yaw_rates),
                    pose_difference_yaw_rate_radps=stats(pose_rates),
                    pose_unwrapped_yaw_delta_rad=math.fsum(deltas),
                    pose_difference_mean_yaw_rate_radps=pose_average,
                    twist_trapezoid_yaw_delta_rad=trapezoid,
                    twist_integral_minus_pose_delta_rad=trapezoid - math.fsum(deltas),
                    pose_yaw_to_command_ratio=pose_average / desired,
                    twist_mean_yaw_to_command_ratio=statistics.mean(yaw_rates) / desired),
                wheel_kinematics=wheel))
    return dict(selection_rule='Maximal consecutive same-sign saturated command runs within each pivot; '
                'trim one nominal angular-acceleration ramp from both ends; require >=1 second afterward.',
                command_clock='Observer receipt ROS sim_s; controller-limited and odometry use source stamp_ns.',
                trim_each_end_s=trim, minimum_selected_window_s=minimum_window,
                maximum_command_gap_s=maximum_gap, accepted=accepted, rejected=rejected)


def analyze(run_path, history_path):
    run = json.loads(run_path.read_text())
    history = json.loads(history_path.read_text())
    report = run['automatic_exploration']['control_result']['ground_maneuver']['report']
    if report['task_mode'] != 'local' or not report['committed']:
        raise ValueError('expected committed local candidate report')
    # Original configured ceilings and construction cap. The historical
    # pre-execution remaining wall deadline was not serialized; successful
    # full-pose reconstruction below is the required identity check.
    limits = Limits(max_speed=.3, max_yaw_rate=.4, max_acceleration=.3,
                    max_yaw_acceleration=.6, integration_dt=.05, max_duration=30.,
                    max_path_length=5., max_points=20000, max_wall_time=.1, max_candidates=2)
    start = tuple(report['start'])
    reconstructed = construct_route_obligations(start, report['original_route'],
        offset=.2159, limits=limits, valid_pose=lambda _: True,
        measured_speed=0., measured_yaw_rate=0.)
    if len(reconstructed.candidates) <= report['candidate_index']:
        raise ValueError('recorded candidate could not be reconstructed within original limits')
    candidate = reconstructed.candidates[report['candidate_index']]
    paths = [row for row in history['paths'] if row['phase'] == 'automatic_exploration' and row['path']]
    if len(paths) != 1:
        raise ValueError('recorded local published path is not unique')
    published = paths[0]['path']
    reference = [[s.x, s.y, start[2], s.yaw] for s in candidate.samples]
    if len(reference) != len(published) or len(reference) != report['reference_points']:
        raise ValueError('reference point count differs')
    spatial = [math.dist(a[:3], b[:3]) for a, b in zip(reference, published)]
    angular = [abs(math.remainder(a[3] - b[3], 2. * math.pi)) for a, b in zip(reference, published)]
    duration_error = abs(candidate.duration - report['reference_duration_s'])
    length_error = abs(candidate.path_length - report['base_arc_length_m'])
    tolerance = 1.e-12
    if max(spatial + angular + [duration_error, length_error]) > tolerance:
        raise ValueError('reconstructed candidate differs from recorded publication/report')
    gm_events = [row for row in history['events'] if row.get('layer') == 'tracking'
                 and row['sim_s'] >= paths[0]['sim_s']]
    accepts = [row for row in gm_events if row['reason'] == 'explicit_ground_maneuver_accepted']
    if len(accepts) != 1:
        raise ValueError('local accepted tracking Action is not unique')
    acceptance = accepts[0]
    goal_id = acceptance['goal_id']
    events = [row for row in gm_events if row['goal_id'] == goal_id
              and row['sim_s'] >= acceptance['sim_s']]
    stops = [row for row in events if row['state'] == 'MANEUVER_BRAKING']
    if len(stops) != 1:
        raise ValueError('expected one observed braking transition')
    stop = stops[0]
    stop_time = event_time(stop)
    starts, settling_events = {}, {}
    for row in events:
        index = row['ground_maneuver']['segment_index']
        if row['state'] in ('MANEUVER_RUNNING', 'MANEUVER_SETTLING'):
            starts.setdefault(index, row)
        if row['state'] == 'MANEUVER_SETTLING':
            settling_events.setdefault(index, []).append(row)
    if sorted(starts) != list(range(len(starts))):
        raise ValueError('primitive start history has a gap')
    rows = []
    for index, primitive in enumerate(candidate.primitives):
        row = dict(primitive_index=index, kind=primitive.kind,
            signed_amount=primitive.signed_amount, target_index=primitive.target_index,
            reference_start_s=candidate.samples[primitive.start_index].time,
            reference_end_s=candidate.samples[primitive.end_index].time,
            reference_duration_s=primitive.duration,
            expected_start=[*primitive.expected_start[:2], start[2], primitive.expected_start[2]],
            expected_end=[*primitive.expected_end[:2], start[2], primitive.expected_end[2]],
            actual_status='not_started', actual_start_ros_s=None, actual_end_ros_s=None,
            actual_duration_s=None, actual_minus_reference_s=None, settling_intervals=[])
        if index in starts:
            completed = index + 1 in starts
            end_event = starts[index + 1] if completed else stop
            begin, end = event_time(starts[index]), event_time(end_event)
            end_observation = endpoint_observation(history, end_event)
            row.update(actual_status='completed' if completed else 'interrupted_by_total_timeout',
                actual_start_ros_s=begin, actual_end_ros_s=end,
                actual_duration_s=end - begin,
                actual_minus_reference_s=(end - begin - primitive.duration) if completed else None,
                start_source_odometry=endpoint_observation(history, starts[index]),
                end_source_odometry=end_observation,
                end_pose_error=endpoint_error(end_observation['pose'], row['expected_end']))
            for settling in settling_events.get(index, []):
                began = event_time(settling)
                # A settling episode can resume tracking on the same primitive.
                following = [e for e in events if event_time(e) > began and
                             (e['state'] != 'MANEUVER_SETTLING' or
                              e['ground_maneuver']['segment_index'] != index)]
                ended = min(event_time(following[0]), end) if following else end
                row['settling_intervals'].append(dict(start_ros_s=began, end_ros_s=ended,
                                                       duration_s=ended - began))
            row['settling_duration_s'] = sum(x['duration_s'] for x in row['settling_intervals'])
            row['before_first_settling_s'] = (event_time(settling_events[index][0]) - begin
                                             if index in settling_events else None)
        rows.append(row)
    completed = [row for row in rows if row['actual_status'] == 'completed']
    reference_completed = math.fsum(row['reference_duration_s'] for row in completed)
    actual_completed = math.fsum(row['actual_duration_s'] for row in completed)
    excess = actual_completed - reference_completed
    long_pivots = [row for row in completed if row['kind'] == 'pivot'
                   and row['reference_duration_s'] >= 5.]
    long_excess = math.fsum(row['actual_minus_reference_s'] for row in long_pivots)
    platforms = platform_statistics(history, candidate.primitives, starts, stop_time, limits)
    outcomes = [row for row in history['execution_terminal_outcomes']
                if row['phase'] == 'automatic_exploration']
    if len(outcomes) != 1 or outcomes[0]['outcome']['reason'] != 'ground_maneuver_timeout':
        raise ValueError('expected recorded execution-layer total-duration timeout')
    sources = [Path(__file__), ROOT / 'src/gbplanner_execution/scripts/ground_maneuver.py',
               ROOT / 'src/gbplanner_execution/scripts/ground_return_route.py',
               ROOT / 'src/gbplanner_execution/scripts/ground_route_obligations.py',
               ROOT / 'src/gbplanner_execution/scripts/ground_maneuver_transport.py',
               ROOT / 'src/gbplanner_local/scripts/control_node.py',
               ROOT / 'src/gbplanner_tracking/src/ground_maneuver.cpp',
               ROOT / 'src/gbplanner_tracking/include/gbplanner_tracking/ground_maneuver.hpp',
               ROOT / 'src/gbplanner_tracking/src/pure_pursuit_node.cpp',
               ROOT / 'src/gbplanner_gazebo/config/controllers.yaml']
    return dict(schema_version=1, scope='Read-only post-hoc analysis of native10 saved evidence',
        reconstruction_scope='Reconstructed candidate under original configured limits, matched to every '
            'published reference pose and recorded duration/path length; not a historical Action wire snapshot. '
            'Geometry callback accepts all poses here; no new map or collision validation is claimed.',
        reconstruction=dict(start=start, original_route=report['original_route'], offset_m=.2159,
            limits=asdict(limits), candidate_index=report['candidate_index'],
            construction_status=reconstructed.status,
            wall_budget_scope='Original configured 0.1s cap; historical remaining pre-execution deadline is not serialized.',
            passed=True, matching_tolerance=tolerance, checked_reference_points=len(reference),
            maximum_published_xyz_difference_m=max(spatial),
            maximum_published_wrapped_yaw_difference_rad=max(angular),
            duration_s=candidate.duration, recorded_duration_s=report['reference_duration_s'],
            duration_difference_s=duration_error, base_arc_length_m=candidate.path_length,
            recorded_base_arc_length_m=report['base_arc_length_m'], base_arc_length_difference_m=length_error),
        clocks=dict(segment_clock='tracking ground_maneuver.motion_time_s, in ROS seconds',
                    sensor_clock='source odometry stamp_ns; no pose interpolation',
                    event_scope='Status is emitted on state/primitive/checkpoint changes, not on every control tick'),
        accepted_tracking_goal_id=goal_id, action_accepted_ros_s=event_time(acceptance),
        braking_started_ros_s=stop_time, elapsed_until_braking_s=stop_time - event_time(acceptance),
        approved_duration_s=report['approved_duration_s'], primitives=rows,
        delay_at_last_started_primitive=dict(primitive_index=max(starts),
            completed_reference_duration_s=reference_completed,
            completed_actual_duration_s=actual_completed, excess_s=excess,
            long_pivot_definition='Completed pivot with reconstructed reference duration >=5 seconds',
            long_pivot_indices=[row['primitive_index'] for row in long_pivots],
            long_pivot_reference_duration_s=math.fsum(row['reference_duration_s'] for row in long_pivots),
            long_pivot_actual_duration_s=math.fsum(row['actual_duration_s'] for row in long_pivots),
            long_pivot_excess_s=long_excess, long_pivot_fraction_of_excess=long_excess / excess,
            long_pivot_settling_duration_s=math.fsum(row['settling_duration_s'] for row in long_pivots)),
        saturation_platforms=platforms, execution_outcome=outcomes[0],
        checkpoint_state_at_braking=stop['ground_maneuver'],
        conclusions=[
            'The two long pivots dominate the excess time before primitive 6; settling alone is not the main cause.',
            'Automatically selected saturated-command windows show reduced observed body yaw despite '
            'unchanged limited commands and nominal wheel yaw. This locates the loss after command limiting '
            'and wheel target generation, without by itself identifying a unique contact/model parameter.',
            'This run failed under the present controller and 30s cap. It does not prove the route '
            'infeasible under all admissible controllers.',
            'Next diagnosis should audit command-to-physical-yaw semantics/model and achievable angular '
            'response; any controller or candidate-cost change must retain the existing caps and route obligations.'],
        input_sha256={relative(path): sha(path) for path in (run_path, history_path)},
        source_sha256={relative(path): sha(path) for path in sources})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, default=ROOT / 'results/pittsburgh_port/autonomy10_crossing.json')
    parser.add_argument('--history', type=Path, default=ROOT / 'results/pittsburgh_port/autonomy10_crossing_history.json')
    parser.add_argument('--output', type=Path, default=ROOT / 'results/pittsburgh_port/autonomy10_crossing_timing.json')
    args = parser.parse_args()
    result = analyze(args.run, args.history)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    print(json.dumps(dict(output=str(args.output), reconstruction=result['reconstruction']['passed'],
        checked_reference_points=result['reconstruction']['checked_reference_points'],
        delay=result['delay_at_last_started_primitive'],
        accepted_saturation_platforms=len(result['saturation_platforms']['accepted'])), ensure_ascii=False))


if __name__ == '__main__':
    main()
