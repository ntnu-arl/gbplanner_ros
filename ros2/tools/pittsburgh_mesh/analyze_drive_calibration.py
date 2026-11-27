#!/usr/bin/env python3
"""Analyze the fixed native11 drive calibration using saved evidence only.

No ROS imports, command publication, map mutation or new simulation. Reference
reconstruction is checked against every published pose and serialized primitive;
it is not a historical Action wire capture or a new map/geometry validation.
"""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
from pathlib import Path
from types import SimpleNamespace

from analyze_crossing_timing import (
    ROOT, Limits, construct_route_obligations, endpoint_error,
    endpoint_observation, event_time, platform_statistics, relative, sha,
    source_time, stats,
)


def body_error(row, target):
    """Target minus observed position, in the observed body-heading basis."""
    pose = row['pose']
    dx, dy = target[0] - pose[0], target[1] - pose[1]
    c, s = math.cos(pose[3]), math.sin(pose[3])
    return dict(source_stamp_s=source_time(row), pose=pose,
                target_minus_observed_world_xy_m=[dx, dy],
                longitudinal_m=dx*c + dy*s, lateral_left_m=-dx*s + dy*c,
                error=endpoint_error(pose, target), speed_mps=row['speed_mps'],
                yaw_rate_radps=row['yaw_rate'])


def sampled_polyline_distance(point, reference):
    best = math.inf
    for a, b in zip(reference, reference[1:]):
        d = [b[k] - a[k] for k in range(3)]
        norm = sum(x*x for x in d)
        u = 0. if norm == 0. else max(0., min(1., sum(
            (point[k]-a[k])*d[k] for k in range(3))/norm))
        best = min(best, math.sqrt(sum(
            (point[k]-a[k]-u*d[k])**2 for k in range(3))))
    return best


def analyze(run_path, history_path, plan_path, manifest_path):
    run = json.loads(run_path.read_text())
    history = json.loads(history_path.read_text())
    calibration = json.loads(plan_path.read_text())
    manifest = json.loads(manifest_path.read_text())
    report = run['automatic_exploration']['control_result']['ground_maneuver']['report']
    if report['task_mode'] != 'local' or not report['committed']:
        raise ValueError('expected committed local candidate')
    readback = run['configuration_readback']['drive']
    multiplier = readback['actual']['wheel_separation_multiplier']
    if readback['mismatches'] or multiplier != calibration['wheel_separation_multiplier']:
        raise ValueError('effective drive calibration differs from fixed plan')
    if sha(plan_path) != manifest['plan']['sha256']:
        raise ValueError('calibration plan changed after source freeze')

    limits = Limits(max_speed=.3, max_yaw_rate=.4, max_acceleration=.3,
                    max_yaw_acceleration=.6, integration_dt=.05, max_duration=30.,
                    max_path_length=5., max_points=20000, max_wall_time=.1,
                    max_candidates=2)
    candidate_set = construct_route_obligations(tuple(report['start']),
        report['original_route'], offset=report['control_offset'], limits=limits,
        valid_pose=lambda _: True, measured_speed=0., measured_yaw_rate=0.)
    candidate = candidate_set.candidates[report['candidate_index']]
    # Indices, identities and stop flags must match exactly. Numeric geometry
    # uses the same fixed 1e-12 check as published poses: host/container libm
    # arithmetic can differ at the last representable bit.
    rebuilt_primitives = json.loads(json.dumps([asdict(p) for p in candidate.primitives]))
    if len(rebuilt_primitives) != len(report['primitives']):
        raise ValueError('serialized primitive count differs')
    primitive_differences = []
    for rebuilt, recorded in zip(rebuilt_primitives, report['primitives']):
        if rebuilt.keys() != recorded.keys():
            raise ValueError('serialized primitive fields differ')
        for key in ('kind', 'start_index', 'end_index', 'target_index', 'stop_at_end'):
            if rebuilt[key] != recorded[key]:
                raise ValueError('serialized primitive identity differs: '+key)
        for key in ('signed_amount', 'duration'):
            primitive_differences.append(abs(rebuilt[key]-recorded[key]))
        for key in ('expected_start', 'expected_end'):
            primitive_differences.extend(abs(a-b) for a,b in zip(rebuilt[key], recorded[key]))
    if max(primitive_differences) > 1.e-12:
        raise ValueError('serialized primitive geometry differs from reconstruction')
    path_rows = [r for r in history['paths']
                 if r['phase'] == 'automatic_exploration' and r['path']]
    if len(path_rows) != 1:
        raise ValueError('local published reference is absent or ambiguous')
    reference = [[p.x, p.y, report['start'][2], p.yaw] for p in candidate.samples]
    published = path_rows[0]['path']
    if len(reference) != len(published) or len(reference) != report['reference_points']:
        raise ValueError('published reference count differs')
    spatial = [math.dist(a[:3], b[:3]) for a, b in zip(reference, published)]
    angular = [abs(math.remainder(a[3]-b[3], 2.*math.pi))
               for a, b in zip(reference, published)]
    duration_delta = abs(candidate.duration-report['reference_duration_s'])
    length_delta = abs(candidate.path_length-report['base_arc_length_m'])
    if max(spatial + angular + [duration_delta, length_delta]) > 1.e-12:
        raise ValueError('reconstructed reference differs from publication/report')

    acceptance = [e for e in history['events'] if e.get('layer') == 'tracking'
        and e['phase'] == 'automatic_exploration'
        and e['reason'] == 'explicit_ground_maneuver_accepted']
    if len(acceptance) != 1:
        raise ValueError('expected one local tracking acceptance')
    acceptance = acceptance[0]
    events = [e for e in history['events'] if e.get('layer') == 'tracking'
              and e.get('goal_id') == acceptance['goal_id']]
    stops = [e for e in events if e['state'] == 'MANEUVER_BRAKING']
    if len(stops) != 1:
        raise ValueError('expected one local braking transition')
    stop = stops[0]
    begin, end = event_time(acceptance), event_time(stop)
    starts, settles = {}, {}
    for event in events:
        index = event['ground_maneuver']['segment_index']
        if event['state'] in ('MANEUVER_RUNNING', 'MANEUVER_SETTLING'):
            starts.setdefault(index, event)
        if event['state'] == 'MANEUVER_SETTLING':
            settles.setdefault(index, []).append(event)
    if sorted(starts) != list(range(len(starts))):
        raise ValueError('primitive event history has gaps')
    primitive_rows = []
    for index, primitive in enumerate(report['primitives']):
        target = [*primitive['expected_end'][:2], report['start'][2],
                  primitive['expected_end'][2]]
        row = dict(primitive_index=index, kind=primitive['kind'],
            signed_amount=primitive['signed_amount'], expected_end=target,
            reference_duration_s=primitive['duration'], status='not_started')
        if index in starts:
            completed = index+1 in starts
            end_event = starts[index+1] if completed else stop
            first, last = event_time(starts[index]), event_time(end_event)
            observed = endpoint_observation(history, end_event)
            row.update(status='completed' if completed else 'interrupted_by_cancellation',
                       start_motion_ros_s=first, end_motion_ros_s=last,
                       actual_duration_s=last-first,
                       actual_minus_reference_s=(last-first-primitive['duration']) if completed else None,
                       end_source_observation=observed,
                       end_pose_error=endpoint_error(observed['pose'], target))
            intervals = []
            for settling in settles.get(index, []):
                a = event_time(settling)
                following = [e for e in events if event_time(e) > a and
                    (e['state'] != 'MANEUVER_SETTLING' or
                     e['ground_maneuver']['segment_index'] != index)]
                b = min(event_time(following[0]), last) if following else last
                intervals.append(dict(start_motion_ros_s=a, end_motion_ros_s=b,
                                      duration_s=b-a))
            row['settling_intervals'] = intervals
            row['settling_duration_s'] = math.fsum(i['duration_s'] for i in intervals)
        primitive_rows.append(row)

    platforms = platform_statistics(history,
        [SimpleNamespace(**p) for p in report['primitives']], starts, end, limits)
    for platform in platforms['accepted']:
        wheel = platform['wheel_kinematics']
        if wheel:
            wheel['effective_separation_m'] = wheel['nominal_separation_m']*multiplier
            wheel['effective_body_yaw_rate_radps'] = wheel['nominal_body_yaw_rate_radps']/multiplier
            wheel['effective_yaw_semantics'] = (
                'Wheel-kinematics value under calibrated track; not independent body motion')
            wheel['measured_body_to_effective_wheel_yaw_ratio'] = (
                platform['source_odometry']['pose_difference_mean_yaw_rate_radps'] /
                wheel['effective_body_yaw_rate_radps'])

    odometry = [o for o in history['odometry'] if begin <= source_time(o) <= end]
    if len(odometry) < 2:
        raise ValueError('local source observations missing')
    gaps = [source_time(b)-source_time(a) for a, b in zip(odometry, odometry[1:])]
    if any(dt <= 0. for dt in gaps):
        raise ValueError('source observations not strictly increasing')
    pose_rates = [math.remainder(b['pose'][3]-a['pose'][3], 2.*math.pi)/dt
                  for a, b, dt in zip(odometry, odometry[1:], gaps)]
    twist_accelerations = [(b['yaw_rate']-a['yaw_rate'])/dt
                          for a, b, dt in zip(odometry, odometry[1:], gaps)]
    pose_rate_times = [.5*(source_time(a)+source_time(b)) for a, b in zip(odometry, odometry[1:])]
    pose_accelerations = [(b-a)/(tb-ta) for a,b,ta,tb in zip(
        pose_rates, pose_rates[1:], pose_rate_times, pose_rate_times[1:])]
    commands = [r for r in history['commands'] if begin <= r['sim_s'] <= end]
    limited = [r for r in history['controller_limited_commands'] if begin <= source_time(r) <= end]
    sampled_distance = [sampled_polyline_distance(o['pose'], reference) for o in odometry]

    stalled_index = stop['ground_maneuver']['segment_index']
    primitive = report['primitives'][stalled_index]
    target = [*primitive['expected_end'][:2], report['start'][2], primitive['expected_end'][2]]
    segment_odometry = [o for o in odometry if source_time(o) >= event_time(starts[stalled_index])]
    # Derive the final contiguous stationary-yaw-converged interval using the
    # existing internal stop and pose tolerances. The interval is not selected
    # from an arbitrary point chosen to make the residual look smaller.
    suffix = []
    for observation in reversed(segment_odometry):
        error = endpoint_error(observation['pose'], target)
        stationary = (math.hypot(*observation['body_linear_velocity'][:2]) <= .02
                      and abs(observation['yaw_rate']) <= .03)
        if not (stationary and error['yaw_rad'] <= .02 and error['xy_m'] > .01):
            break
        suffix.append(observation)
    suffix.reverse()
    stall = dict(primitive_index=stalled_index, target=target,
        selection='Final consecutive source observations before cancellation: body XY speed <=0.02, '
                  '|yaw rate|<=0.03, |endpoint yaw error|<=0.02, XY error>0.01.',
        count=len(suffix), includes_cancel_braking=False)
    if suffix:
        body_errors = [body_error(o,target) for o in suffix]
        interval = [source_time(suffix[0]), source_time(suffix[-1])]
        stall.update(source_interval_s=interval, duration_s=interval[1]-interval[0],
            first=body_errors[0], last=body_errors[-1],
            endpoint_xy_error_m=stats([r['error']['xy_m'] for r in body_errors]),
            longitudinal_error_m=stats([r['longitudinal_m'] for r in body_errors]),
            lateral_error_m=stats([r['lateral_left_m'] for r in body_errors]),
            measured_speed_mps=stats([o['speed_mps'] for o in suffix]),
            measured_yaw_rate_radps=stats([o['yaw_rate'] for o in suffix]),
            command_linear_x_mps=stats([r['linear_x'] for r in commands if interval[0]<=r['sim_s']<=interval[1]]),
            command_yaw_rate_radps=stats([r['angular_z'] for r in commands if interval[0]<=r['sim_s']<=interval[1]]))

    outcome = [r for r in history['execution_terminal_outcomes']
               if r['phase'] == 'automatic_exploration']
    if len(outcome) != 1:
        raise ValueError('local execution terminal outcome not unique')
    stopped_stamp = outcome[0]['outcome']['stop_stamp_s']
    stopped_rows = [o for o in history['odometry'] if abs(source_time(o)-stopped_stamp)<1.e-8]
    if len(stopped_rows) != 1:
        raise ValueError('source-stamped terminal stop observation missing')
    source_hashes = {path: dict(before=digest, after=sha(ROOT/path),
                    unchanged=digest==sha(ROOT/path)) for path,digest in manifest['files'].items()}
    return dict(schema_version=1, scope='Read-only native11 fixed-drive-calibration analysis',
        analysis_utc=datetime.now(timezone.utc).isoformat(),
        run_status=run['status'], comparison_scope='New independent mapping and planning run; '
            'not a matched-route A/B performance comparison.',
        fixed_calibration=dict(multiplier=multiplier, training=calibration,
                               native_drive_readback=readback),
        reconstruction=dict(checked_published_points=len(reference), serialized_primitives_match=True,
            numeric_matching_tolerance=1.e-12,
            maximum_primitive_numeric_difference=max(primitive_differences),
            maximum_xyz_difference_m=max(spatial), maximum_yaw_difference_rad=max(angular),
            reference_duration_s=candidate.duration, duration_difference_s=duration_delta,
            base_arc_length_m=candidate.path_length, length_difference_m=length_delta,
            geometry_scope='Complete reference reconstruction matches the saved publication. '
              'No new map/collision validation and no historical Action wire snapshot claimed.'),
        timing=dict(goal_id=acceptance['goal_id'], acceptance_motion_ros_s=begin,
            braking_motion_ros_s=end, braking_receipt_ros_s=stop['sim_s'],
            elapsed_before_braking_s=end-begin, approved_duration_s=report['approved_duration_s'],
            clock_scope='Primitive timing uses tracker motion_time_s; source quantities use odometry stamps.'),
        primitives=primitive_rows, saturation_platforms=platforms,
        physical_observations=dict(source_interval_s=[source_time(odometry[0]),source_time(odometry[-1])],
            count=len(odometry), source_gap_s=stats(gaps),
            pose_difference_yaw_rate_radps=stats(pose_rates),
            pose_difference_yaw_acceleration_radps2=stats(pose_accelerations),
            twist_yaw_rate_radps=stats([o['yaw_rate'] for o in odometry]),
            twist_difference_yaw_acceleration_radps2=stats(twist_accelerations),
            peak_abs_pose_yaw_rate_radps=max(map(abs,pose_rates)),
            peak_abs_twist_yaw_rate_radps=max(abs(o['yaw_rate']) for o in odometry),
            peak_abs_pose_yaw_acceleration_radps2=max(map(abs,pose_accelerations)),
            peak_abs_twist_yaw_acceleration_radps2=max(map(abs,twist_accelerations)),
            command_yaw_rate_radps=stats([r['angular_z'] for r in commands]),
            command_linear_x_mps=stats([r['linear_x'] for r in commands]),
            limited_yaw_rate_radps=stats([r['angular_z'] for r in limited]),
            sampled_nearest_reference_xyz_distance_m=stats(sampled_distance),
            path_distance_scope='Distance from recorded physical source positions to the matched sampled '
              'XYZ polyline only. No interpolation of physical motion, yaw match, ordered progress or map claim.'),
        endpoint_stall=stall, checkpoint_state_at_braking=stop['ground_maneuver'],
        execution_outcome=outcome[0],
        final_stop=dict(source_observation=stopped_rows[0],
                       original_endpoint_error=endpoint_error(stopped_rows[0]['pose'],report['original_endpoint'])),
        cleanup=run['cleanup'], node_exit_failures=run['node_exit_failures'],
        sources=dict(manifest=relative(manifest_path), manifest_sha256=sha(manifest_path),
            comparison_time='After native11 completion, at analysis invocation',
            all_unchanged=all(r['unchanged'] for r in source_hashes.values()), checks=source_hashes,
            analysis_script_sha256=sha(Path(__file__))),
        input_sha256={relative(p):sha(p) for p in (run_path,history_path,plan_path)},
        interpretation=[
            'Task completion and yaw-response calibration are separate outcomes. Native11 remains failed.',
            'Calibrated wheel differential targets and their acceleration are higher; '
              'independent native pose/twist provides the physical response measurement.',
            'Command caps are not proofs of instantaneous physical yaw/acceleration bounds.',
            'The interrupted pivot reaches its yaw neighborhood while a predominantly lateral XY residual '
              'remains outside the unchanged 1 cm internal endpoint requirement.',
            'The data motivates review of pivot-position convergence and model mismatch; '
              'it does not justify relaxing the route, map, pose, stop or duration conditions.'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    base = ROOT/'results/pittsburgh_port'
    parser.add_argument('--run', type=Path, default=base/'autonomy11_drive_calibration.json')
    parser.add_argument('--history', type=Path, default=base/'autonomy11_drive_calibration_history.json')
    parser.add_argument('--plan', type=Path, default=base/'drive_calibration_plan.json')
    parser.add_argument('--manifest', type=Path, default=base/'drive_calibration_source_manifest.json')
    parser.add_argument('--output', type=Path, default=base/'autonomy11_drive_calibration_analysis.json')
    args = parser.parse_args()
    result = analyze(args.run,args.history,args.plan,args.manifest)
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(args.output), status=result['run_status'],
        reconstructed_points=result['reconstruction']['checked_published_points'],
        platform_pose_yaw_radps=[p['source_odometry']['pose_difference_mean_yaw_rate_radps']
                                for p in result['saturation_platforms']['accepted']],
        endpoint_stall=result['endpoint_stall'],
        sources_unchanged=result['sources']['all_unchanged']),ensure_ascii=False))


if __name__ == '__main__':
    main()
