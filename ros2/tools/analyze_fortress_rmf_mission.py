#!/usr/bin/env python3
"""Review a recorded native RMF mission; never start ROS or simulation.

Usage: python3 tools/analyze_fortress_rmf_mission.py results/fortress_rmf/mission3.json
Only the sibling *_review.json and (unless --no-plot) *_trajectory.png are written.
"""
import argparse
from bisect import bisect_left, bisect_right
from collections import Counter
import hashlib
import json
import math
from pathlib import Path


def finite(values):
    return all(isinstance(x, (float, int)) and math.isfinite(x) for x in values)


def source_s(row):
    return row['stamp_ns'] * 1.e-9


def yaw(q):
    x, y, z, w = q
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def angle_difference(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


def inverse_rotate(q, vector):
    x, y, z, w = q
    norm = math.sqrt(sum(a*a for a in q))
    x, y, z, w = (a/norm for a in (x, y, z, w))
    matrix = ((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
              (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
              (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))
    return [sum(matrix[j][i]*vector[j] for j in range(3)) for i in range(3)]


def moments(vectors):
    values = [v for v in vectors if finite(v)]
    if not values:
        return {'count': 0}
    n = len(values)
    mean = [sum(v[i] for v in values)/n for i in range(len(values[0]))]
    norms = [math.hypot(*v) for v in values]
    return {'count': n, 'mean_xyz': mean,
            'stddev_xyz': [math.sqrt(sum((v[i]-mean[i])**2 for v in values)/n)
                           for i in range(len(mean))],
            'norm_mean': sum(norms)/n, 'norm_rms': math.sqrt(sum(v*v for v in norms)/n),
            'norm_min': min(norms), 'norm_max': max(norms)}


class Odometry:
    def __init__(self, rows):
        self.invalid = self.nonmonotonic = 0
        self.rows = []
        previous = -1
        for row in rows:
            if (row.get('stamp_ns', 0) <= 0 or not finite(row.get('position', [])) or
                    len(row.get('position', [])) != 3 or len(row.get('quaternion', [])) != 4 or
                    not finite(row['quaternion']) or math.hypot(*row['quaternion']) < 1.e-8):
                self.invalid += 1
                continue
            if row['stamp_ns'] <= previous:
                self.nonmonotonic += 1
            previous = row['stamp_ns']
            self.rows.append(row)
        # Sorting makes nearest-source matching deterministic; the original
        # ordering defects are retained above and fail the independent check.
        self.rows.sort(key=lambda r: r['stamp_ns'])
        self.times = [source_s(r) for r in self.rows]

    def nearest(self, stamp):
        if not self.rows or stamp is None:
            return None, None
        at = bisect_left(self.times, stamp)
        indexes = [i for i in (at-1, at) if 0 <= i < len(self.rows)]
        index = min(indexes, key=lambda i: abs(self.times[i]-stamp))
        return self.rows[index], abs(self.times[index]-stamp)

    def between(self, start, end):
        return self.rows[bisect_left(self.times, start):bisect_right(self.times, end)]


def travelled(rows):
    return sum(math.dist(a['position'], b['position']) for a, b in zip(rows, rows[1:]))


def file_for(result_path, result, key, suffix):
    # Results written inside the mounted Linux checkout remain portable to macOS.
    return result_path.with_name(Path(result.get(key, result_path.stem+suffix)).name)


def task_events(history, result):
    task = result.get('final', {}).get('task') or (history.get('task') or [{}])[-1]
    events = task.get('events', [])
    return task, sorted(events, key=lambda e: e['ros_s'])


def phase_at(events, stamp):
    at = bisect_right([e['ros_s'] for e in events], stamp)-1
    return events[at]['state'] if at >= 0 else 'STARTUP'


def phase_review(odom, events):
    rows = []
    if not odom.rows:
        return rows
    boundaries = [{'state': 'STARTUP', 'reason': 'before_first_mission_event',
                   'ros_s': min(0., odom.times[0])}, *events]
    for i, event in enumerate(boundaries):
        start = event['ros_s']
        end = boundaries[i+1]['ros_s'] if i+1 < len(boundaries) else odom.times[-1]
        if end < odom.times[0] or start > odom.times[-1]:
            continue
        samples = [r for r in odom.between(start, end) if source_s(r) < end or i+1 == len(boundaries)]
        # Attribute each complete observed increment to its later sample. This
        # keeps phase lengths additive without manufacturing boundary poses.
        increments = [(a, b) for a, b in zip(odom.rows, odom.rows[1:])
                      if start <= source_s(b) < end or
                      (i+1 == len(boundaries) and start <= source_s(b) <= end)]
        rows.append({'phase': event['state'], 'reason': event.get('reason'),
                     'transition_source_s': start, 'next_transition_source_s': end,
                     'duration_source_s': end-start, 'observations': len(samples),
                     'first_odometry_source_s': source_s(samples[0]) if samples else None,
                     'last_odometry_source_s': source_s(samples[-1]) if samples else None,
                     'distance_xyz_m': sum(math.dist(a['position'], b['position']) for a, b in increments),
                     'end_position': samples[-1]['position'] if samples else None})
    return rows


def action_review(history, odom):
    modes = {r['completed_actions']: r.get('mode') for r in history.get('control', [])
             if r.get('state') == 'WAITING_OBSERVATION'}
    done, seen = [], set()
    for event in history.get('execution', []):
        count = event.get('succeeded', 0)
        if event.get('state') != 'DONE' or count in seen or not count:
            continue
        seen.add(count)
        stamp = event.get('odometry_freshness', {}).get('last_accepted_source_stamp_s')
        point, skew = odom.nearest(stamp)
        candidates = [t for t in history.get('trajectories', []) if t.get('samples') and
                      t['wall_s'] <= event['wall_s'] and (stamp is None or source_s(t) <= stamp)]
        command = candidates[-1] if candidates else None
        target = command['samples'][-1]['poses'][0] if command else None
        matched = point is not None and skew <= .025
        error = math.dist(point['position'], target['position']) if matched and target else None
        yaw_error = abs(angle_difference(yaw(point['quaternion']), yaw(target['quaternion']))) if matched and target else None
        motion = odom.between(source_s(command), stamp) if command and stamp is not None else []
        done.append({'succeeded_action': count, 'mode': modes.get(count), 'reason': event.get('reason'),
                     'completion_source_s': stamp, 'odometry_match_skew_s': skew,
                     'completion_odometry_exact_stamp': bool(skew is not None and skew < 1.e-8),
                     'trajectory_origin_source_s': source_s(command) if command else None,
                     'command_samples': len(command['samples']) if command else 0,
                     'command_endpoint': target, 'observed_endpoint': point['position'] if matched else None,
                     'position_error_m': error, 'yaw_error_rad': yaw_error,
                     'body_linear_speed_m_s': math.hypot(*point['body_linear']) if matched else None,
                     'body_angular_speed_rad_s': math.hypot(*point['body_angular']) if matched else None,
                     'endpoint_within_original_limits': error <= .05 and yaw_error <= .1
                         if error is not None else None,
                     'observed_travel_m': travelled(motion), 'observations': len(motion)})
    return {'completed': done, 'mode_counts': dict(Counter(r['mode'] for r in done)),
            'last_execution': (history.get('execution') or [{}])[-1],
            'method': 'Native odometry matched to the follower completion source stamp; '
                      'target is the last actual published trajectory sample, including allocated yaw. '
                      'No reconstruction from old supervised-flight phase labels.',
            'maximum_matching_skew_s': .025, 'position_limit_m': .05, 'yaw_limit_rad': .1}


def continuation_review(history, odom):
    commits, seen = [], set()
    trajectory_records = history.get('trajectories', [])
    # Legacy replacements have distinct origins. Appends retain that origin,
    # so their identity must use separate publication metadata and receipt order.
    trajectories = {r['stamp_ns']: r for r in trajectory_records}
    for event in history.get('execution', []):
        c = event.get('last_continuation')
        if not c or event.get('continuations', 0) in seen:
            continue
        seen.add(event['continuations'])
        append = c.get('strategy') == 'append_timed'
        append_checks = {}
        if append:
            publication_s = c.get('commit_ros_s')
            valid_time = isinstance(publication_s, (int, float)) and math.isfinite(publication_s)
            if not valid_time:
                publication_s = None
            old_matches = [(i, r) for i, r in enumerate(trajectory_records)
                           if r.get('stamp_ns') == c.get('old_trajectory_stamp_ns') and
                           r.get('samples') and len(r['samples']) == c.get('old_sample_count')]
            new_matches = [(i, r) for i, r in enumerate(trajectory_records)
                           if r.get('stamp_ns') == c.get('new_trajectory_stamp_ns') and
                           r.get('samples') and len(r['samples']) == c.get('new_sample_count')]
            # Never guess among repeated identities or silently treat the last
            # row at an origin as both the old and the new command.
            unique = len(old_matches) == len(new_matches) == 1
            old_index, original = old_matches[0] if len(old_matches) == 1 else (None, None)
            new_index, replacement = new_matches[0] if len(new_matches) == 1 else (None, None)
            ordered = unique and old_index < new_index
            prefix_fields = {name: False for name in
                             ('position', 'quaternion', 'velocities', 'accelerations', 'offset_s')}
            growing = bool(original and replacement and
                           len(replacement['samples']) > len(original['samples']))
            if growing:
                pairs = list(zip(original['samples'], replacement['samples']))
                for name in ('velocities', 'accelerations', 'offset_s'):
                    prefix_fields[name] = all(name in a and name in b and a[name] == b[name]
                                              for a, b in pairs)
                for name in ('position', 'quaternion'):
                    prefix_fields[name] = all(a.get('poses') and b.get('poses') and
                        len(a['poses']) == len(b['poses']) and all(
                            name in p and name in q and p[name] == q[name]
                            for p, q in zip(a['poses'], b['poses'])) for a, b in pairs)
            same_origin = bool(original and replacement and
                               original['stamp_ns'] == replacement['stamp_ns'])
            same_frame = bool(original and replacement and
                              original.get('frame_id') == replacement.get('frame_id') and
                              original.get('joint_names') == replacement.get('joint_names'))
            old_count, new_count = (c.get(name) for name in
                ('old_trajectory_command_count', 'new_trajectory_command_count'))
            count_advanced = (isinstance(old_count, int) and not isinstance(old_count, bool) and
                              isinstance(new_count, int) and not isinstance(new_count, bool) and
                              old_count >= 1 and new_count == old_count+1)
            offsets = [s.get('offset_s') for s in replacement['samples']] if replacement else []
            times_ordered = bool(offsets and finite(offsets) and offsets[0] > 0 and
                                 all(b > a for a, b in zip(offsets, offsets[1:])))
            seam = original['samples'][-1] if original else {}
            seam_zero = all(seam.get(name) and all(len(v) == 6 and finite(v) and
                            all(x == 0. for x in v) for v in seam[name])
                            for name in ('velocities', 'accelerations'))
            old_end = (source_s(original)+seam['offset_s']) if original else None
            receipt = replacement.get('sim_s') if replacement else None
            # Observer receipt is a separate observation, not the publication
            # clock or an acknowledgement from the downstream controller.
            received_before_end = (isinstance(receipt, (int, float)) and math.isfinite(receipt)
                                   and old_end is not None and receipt < old_end)
            append_checks = {
                'trajectory_identity_unique': unique,
                'old_trajectory_history_index': old_index, 'new_trajectory_history_index': new_index,
                'old_trajectory_identity_matches': len(old_matches),
                'new_trajectory_identity_matches': len(new_matches),
                'trajectory_receipt_order_confirmed': ordered,
                'trajectory_command_count_advanced': count_advanced,
                'trajectory_origin_preserved': same_origin, 'trajectory_frame_and_joint_preserved': same_frame,
                'old_prefix_fields_equal': prefix_fields,
                'old_prefix_exactly_preserved': growing and all(prefix_fields.values()),
                'old_endpoint_reference_derivatives_zero': seam_zero,
                'new_sample_times_strictly_increasing': times_ordered,
                'new_trajectory_observer_receipt_ros_s': receipt,
                'new_trajectory_observer_receipt_delay_s':
                    receipt-publication_s if valid_time and isinstance(receipt, (int, float)) else None,
                'new_trajectory_observed_before_old_endpoint': received_before_end,
                'append_evidence_verified': bool(valid_time and ordered and count_advanced and
                    same_origin and same_frame and growing and all(prefix_fields.values()) and
                    seam_zero and times_ordered and received_before_end)}
        else:
            original = trajectories.get(c.get('old_trajectory_stamp_ns'))
            replacement = trajectories.get(c.get('new_trajectory_stamp_ns'))
            publication_s = c['new_trajectory_stamp_ns']*1.e-9
        observed, skew = odom.nearest(publication_s)
        speed = math.hypot(*observed['body_linear']) if observed and skew <= .025 else None
        original_target = original['samples'][-1]['poses'][0]['position'] if original else None
        distance = math.dist(observed['position'], original_target) if observed and original_target else None
        elapsed = (publication_s-c['old_trajectory_stamp_ns']*1.e-9
                   if append and publication_s is not None else None if append else
                   (c['new_trajectory_stamp_ns']-c['old_trajectory_stamp_ns'])*1.e-9)
        recorded_duration = original['samples'][-1]['offset_s'] if original else None
        # Actual trajectory duration includes its final sample's dt interval.
        # Compare independent message stamps, native velocity and endpoint,
        # not merely a counter increment or a delayed PCI acknowledgement.
        moving = bool(replacement and original and speed is not None and speed > .05 and
                      distance is not None and distance > .05 and recorded_duration is not None and
                      elapsed is not None and 0 <= elapsed < recorded_duration and
                      math.isclose(recorded_duration, c['old_duration_s'], abs_tol=1.e-8) and
                      c['new_generation'] == c['old_generation']+1 and
                      c['goal_id'] == event.get('goal_id') and
                      (not append or append_checks['append_evidence_verified']))
        commits.append({**c, 'native_odometry_skew_s':skew,'native_body_speed_mps':speed,
            'commit_observation_age_s':publication_s-c['odom_stamp_s'] if publication_s is not None else None,
            'actual_old_trajectory_duration_s':recorded_duration,
            'native_distance_to_old_endpoint_m':distance,'elapsed_at_actual_new_publication_s':elapsed,
            'old_and_new_trajectory_observed':bool(original and replacement),
            'same_goal_id_confirmed':c['goal_id']==event.get('goal_id'),
            'moving_before_old_endpoint':moving, **append_checks})
    return {'commits':commits,'count':len(commits),
            'moving_count':sum(c['moving_before_old_endpoint'] for c in commits),
            'method':'Legacy replacements match distinct trajectory headers. Timed appends match '
                     'origin plus sample count and receipt order, verify the complete old prefix, '
                     'and use separately recorded commit_ros_s. Both match the same Action UUID '
                     'and generation with independently sampled native odometry at publication. '
                     'Observer trajectory receipt is not downstream-controller acceptance.'}


def contact_review(history, odom, events):
    counts, non_support, unknown, examples = Counter(), 0, 0, []
    normal_count = depth_count = 0
    support_times = []
    for record in history.get('contacts', []):
        if not record.get('pairs') and not record.get('positions'):
            continue
        stamp = source_s(record)
        phase = phase_at(events, stamp)
        pose, skew = odom.nearest(stamp)
        normal_count += len(record.get('normals', []))
        depth_count += len(record.get('depths', []))
        if record['part'] != 'base':
            label = 'rotor_contact'
        elif pose is None or skew > .025 or not record.get('positions'):
            label = 'unclassified_missing_synchronous_pose_or_positions'
        else:
            upright = inverse_rotate(pose['quaternion'], [0., 0., 1.])[2] >= math.cos(math.pi/6)
            local = [inverse_rotate(pose['quaternion'], [p[i]-pose['position'][i] for i in range(3)])
                     for p in record['positions']]
            bottom = [upright and abs(p[0]) <= .145 and abs(p[1]) <= .145 and
                      abs(p[2]+.0565) <= .02 for p in local]
            label = 'base_bottom_support' if all(bottom) else 'base_outside_support_or_mixed'
        counts[(phase, record['part'], label)] += 1
        if label == 'base_bottom_support':
            support_times.append(stamp)
        elif label.startswith('unclassified'):
            unknown += 1
        else:
            non_support += 1
        if label != 'base_bottom_support' and len(examples) < 12:
            examples.append({'source_s': stamp, 'task_phase': phase, 'part': record['part'],
                             'classification': label, 'odometry_skew_s': skew, 'pairs': record['pairs'],
                             'positions': record['positions']})
    return {'nonempty_messages': sum(counts.values()), 'non_support_messages': non_support,
            'unclassified_messages': unknown, 'support_messages': len(support_times),
            'first_support_source_s': min(support_times) if support_times else None,
            'last_support_source_s': max(support_times) if support_times else None,
            'normal_samples': normal_count, 'depth_samples': depth_count,
            'by_phase_part_and_geometry': [{'phase': phase, 'part': part, 'classification': label, 'messages': n}
                                         for (phase, part, label), n in counts.items()],
            'non_support_examples': examples,
            'method': 'Every recorded native nonempty contact. World points are transformed with the '
                      'nearest native base pose (maximum 0.025 s skew). All points must lie on the '
                      'original 0.25 x 0.25 x 0.113 m body bottom slab; mixed contacts do not count as support.',
            'limitation': 'Geometric support is not a measured contact-force direction. Empty/missing '
                          'normal or wrench arrays cannot prove load support; absence of a message '
                          'is not proof of zero contact.'}


def actuator_review(history, metadata):
    landed = [r['odometry_stamp_s'] for r in history.get('controller', [])
              if r.get('flight', {}).get('state') == 'LANDED' and r['flight'].get('touchdown_confirmed')]
    landed_at = min(landed) if landed else None
    start = landed_at+.2 if landed_at is not None else None
    commands = [r for r in history.get('motor_commands', []) if start is not None and source_s(r) >= start]
    scale = {m['parameters']['jointName']: float(m['parameters']['rotorVelocitySlowdownSim'])
             for m in metadata.get('motor_parameters', [])}
    joints = []
    for row in history.get('joints', []):
        values = dict(zip(row.get('names', []), row.get('velocity', [])))
        names = [f'rotor_{i}_joint' for i in range(4)]
        if all(name in values and name in scale for name in names):
            joints.append((source_s(row), [abs(values[name])*scale[name] for name in names]))
    stopped = [(stamp, v) for stamp, v in joints if start is not None and stamp >= start]
    return {'landed_confirmation_source_s': landed_at, 'post_touchdown_window_start_s': start,
            'settling_allowance_s': .2, 'zero_command_threshold_rad_s': 1.e-9,
            'measured_stopped_threshold_rad_s': 1.,
            'post_touchdown_command_samples': len(commands), 'post_touchdown_joint_samples': len(stopped),
            'all_four_commands_zero': all(len(r['velocity']) == 4 and finite(r['velocity']) and
                max(abs(v) for v in r['velocity']) <= 1.e-9 for r in commands) if commands else None,
            'all_four_measured_rotors_stopped': all(finite(v) and max(v) <= 1. for _, v in stopped) if stopped else None,
            'maximum_measured_rad_s_per_rotor_after_settle':
                [max(v[i] for _, v in stopped) for i in range(4)] if stopped else None,
            'last_measured_source_s': joints[-1][0] if joints else None,
            'last_measured_rad_s_per_rotor': joints[-1][1] if joints else None,
            'last_motor_command': (history.get('motor_commands') or [None])[-1],
            'joint_to_motor_speed_factors': scale,
            'semantics': 'Commanded zero and measured stopped are separate. Measured rates are '
                         'absolute native joint velocities multiplied by each source motor slowdown factor.'}


def imu_review(result, retained):
    streams = retained.get('streams', {})
    summaries = result.get('imu_summary', {})
    pairs, retained_checks = {}, {}
    for name, groups in streams.items():
        records = [r for rows in groups.values() for r in rows]
        stamps = [r['stamp_ns'] for r in records]
        quaternion_norms = [math.hypot(*r['orientation_xyzw']) for r in records
                            if finite(r['orientation_xyzw'])]
        fields = ('gyro_rad_s', 'accel_m_s2', 'orientation_xyzw', 'gyro_covariance',
                  'accel_covariance', 'orientation_covariance')
        retained_checks[name] = {
            'samples': len(records), 'unique_positive_stamps': bool(stamps) and
                min(stamps) > 0 and len(set(stamps)) == len(stamps),
            'finite': all(finite(r[field]) for r in records for field in fields),
            'frames': sorted({r['frame_id'] for r in records}),
            'orientation_norm_range': [min(quaternion_norms), max(quaternion_norms)] if quaternion_norms else None}
    for name in ('adis', 'ground_truth', 'os0'):
        raw = [r for rows in streams.get(name+'_raw', {}).values() for r in rows]
        output = [r for rows in streams.get(name+'_output', {}).values() for r in rows]
        raw_by_stamp = {r['stamp_ns']: r for r in raw}
        matched = [(raw_by_stamp[r['stamp_ns']], r) for r in output if r['stamp_ns'] in raw_by_stamp]
        differences = {}
        for field in ('gyro_rad_s', 'accel_m_s2'):
            differences[field] = moments([[b-a for a, b in zip(r[field], s[field])]
                                          for r, s in matched if finite(r[field]) and finite(s[field])])
        residuals = []
        for r in raw:
            if (r.get('stationary_from_recent_odometry') and finite(r['accel_m_s2']) and
                    finite(r['orientation_xyzw']) and math.hypot(*r['orientation_xyzw']) > 1.e-8):
                gravity = inverse_rotate(r['orientation_xyzw'], [0., 0., 9.81])
                residuals.append([a-g for a, g in zip(r['accel_m_s2'], gravity)])
        covariance_examples = {}
        for phase, rows in streams.get(name+'_output', {}).items():
            if rows:
                r = rows[-1]
                covariance_examples[phase] = {field: r[field] for field in
                    ('gyro_covariance', 'accel_covariance', 'orientation_covariance')}
        pairs[name] = {'retained_raw': len(raw), 'retained_output': len(output),
                       'exact_stamp_pairs': len(matched), 'output_minus_raw_same_stamp': differences,
                       'stationary_retained_raw_minus_sensor_frame_gravity_m_s2': moments(residuals),
                       'output_covariance_examples': covariance_examples}
    return {'every_received_sample_summaries': summaries, 'retained_pair_analysis': pairs,
            'independent_retained_checks': retained_checks,
            'retention_contract': retained.get('sampling'),
            'physical_interpretation': 'Accelerometer is native specific force in sensor axes: '
                'stationary supported/hovering readings include approximately 9.81 m/s^2, not zero. '
                'Gyro units are rad/s. Full-stream counts/moments are observer accumulations; '
                'raw/output differences here use only exactly equal retained source stamps. '
                'No interpolation, replacement of missing samples, or statistical parity claim.',
            'noise_contract': 'ADIS output adds RotorS white noise, evolving bias and turn-on bias. '
                'Ground-truth accelerometer has zero added noise, while source-default gyro random '
                'walk remains. OS0 follows its Hector reference model and sensor offset.'}


def flight_review(history, task, odom):
    controllers = history.get('controller', [])
    sequence = []
    for row in controllers:
        state = row.get('flight', {}).get('state')
        if not sequence or sequence[-1]['state'] != state:
            sequence.append({'state': state, 'reason': row.get('flight', {}).get('reason'),
                             'source_s': row.get('odometry_stamp_s'), 'position': row.get('position'),
                             'desired_position': row.get('desired_position')})
    taking_off = [r for r in controllers if r.get('flight', {}).get('state') == 'TAKING_OFF']
    takeoff = None
    if taking_off:
        last = taking_off[-1]
        pose, skew = odom.nearest(last['odometry_stamp_s'])
        takeoff = {'last_taking_off_source_s': last['odometry_stamp_s'],
                   'last_desired_position': last.get('desired_position'), 'last_observed_position': last.get('position'),
                   'position_error_m': math.dist(last['desired_position'], last['position']),
                   'native_odometry_skew_s': skew,
                   'native_linear_speed_m_s': math.hypot(*pose['body_linear']) if pose else None,
                   'native_angular_speed_rad_s': math.hypot(*pose['body_angular']) if pose else None,
                   'yaw_error_rad': abs(angle_difference(yaw(pose['quaternion']), last['desired_yaw'])) if pose else None}
    return_events = [r for r in history.get('control', []) if r.get('state') == 'DONE' and
                     r.get('reason') in {'returned_home', 'already_home'}]
    home = task.get('home_position')
    before_land = None
    if return_events:
        event = return_events[0]
        stamp = event.get('odometry_freshness', {}).get('last_accepted_source_stamp_s')
        pose, skew = odom.nearest(stamp)
        if pose and skew <= .025:
            before_land = {'source_s': stamp, 'observed_position': pose['position'], 'odometry_skew_s': skew,
                           'error_to_recorded_hover_home_m': math.dist(pose['position'], home) if home else None,
                           'pci_reason': event['reason']}
    return {'controller_state_transitions': sequence, 'last_takeoff_observation': takeoff,
            'recorded_hover_home': home, 'return_home_before_descent': before_land,
            'last_position': odom.rows[-1]['position'] if odom.rows else None,
            'last_xy_error_to_hover_home_m': math.dist(odom.rows[-1]['position'][:2], home[:2])
                if odom.rows and home else None,
            'home_semantics': 'Mission home_position is its native odometry snapshot on set-home ACK. '
                'Return error is checked before descent; final landed Z is not compared to hover-home Z.'}


def plot(path, history, odom, events, review):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    if not odom.rows:
        return None
    plt.rcParams.update({'font.size': 9, 'axes.spines.top': False, 'axes.spines.right': False})
    fig = plt.figure(figsize=(12, 7.5), layout='constrained')
    grid = fig.add_gridspec(2, 2)
    ax = fig.add_subplot(grid[:, 0], projection='3d')
    labels = {'STARTUP': 'Ground settling', 'IDLE': 'Ground settling', 'WAIT_GROUND': 'Ground settling',
              'TAKEOFF_REQUESTED': 'Native takeoff', 'WAIT_AIRBORNE': 'Native takeoff',
              'EXPLORING': 'Autonomous exploration / return', 'REQUEST_LANDING': 'Native landing',
              'LANDING_REQUESTED': 'Native landing', 'WAIT_LANDED': 'Native landing', 'COMPLETE': 'Landed',
              'STOPPING': 'Failure stop', 'FAILURE_HOLD': 'Failure hold', 'FAILED': 'Failed'}
    phases = [labels.get(phase_at(events, t), 'Map / planning hover') for t in odom.times]
    palette = plt.get_cmap('tab10')
    for i, label in enumerate(dict.fromkeys(phases)):
        points = [r['position'] if phase == label else [math.nan]*3 for r, phase in zip(odom.rows, phases)]
        ax.plot(*zip(*points), label=label, color=palette(i % 10), linewidth=1.6)
    ax.scatter(*odom.rows[0]['position'], marker='o', color='black', s=22)
    ax.scatter(*odom.rows[-1]['position'], marker='x', color='black', s=35)
    # Equal physical scales keep micrometre hover drift from looking like a
    # large horizontal excursion when the only achieved motion was takeoff.
    minima = [min(r['position'][i] for r in odom.rows) for i in range(3)]
    maxima = [max(r['position'][i] for r in odom.rows) for i in range(3)]
    half = max(max(b-a for a, b in zip(minima, maxima))*.55, .25)
    centers = [(a+b)/2 for a, b in zip(minima, maxima)]
    ax.set_xlim(centers[0]-half, centers[0]+half)
    ax.set_ylim(centers[1]-half, centers[1]+half)
    ax.set_zlim(centers[2]-half, centers[2]+half)
    ax.set_box_aspect((1, 1, 1))
    ax.set(xlabel='World X (m)', ylabel='World Y (m)', zlabel='World Z (m)', title='Measured native trajectory')
    ax.legend(fontsize=7, loc='upper left')
    height = fig.add_subplot(grid[0, 1])
    height.plot(odom.times, [r['position'][2] for r in odom.rows], label='Native odometry', color='#0369a1')
    controllers = [r for r in history.get('controller', []) if 'desired_position' in r and r.get('samples', 0) > 0]
    height.plot([r['odometry_stamp_s'] for r in controllers], [r['desired_position'][2] for r in controllers],
                linestyle='--', linewidth=1., color='#b45309', label='Controller desired Z')
    height.set(xlabel='Acquisition time (s)', ylabel='Z (m)', title='Native flight profile')
    height.grid(alpha=.2)
    height.legend(fontsize=8)
    rotor = fig.add_subplot(grid[1, 1])
    scales = review['actuators']['joint_to_motor_speed_factors']
    for i in range(4):
        name = f'rotor_{i}_joint'
        values = []
        for row in history.get('joints', []):
            rates = dict(zip(row.get('names', []), row.get('velocity', [])))
            if name in rates and name in scales:
                values.append((source_s(row), abs(rates[name])*scales[name]))
        if values:
            rotor.plot(*zip(*values), linewidth=.7, label=f'Rotor {i}')
    rotor.set(xlabel='Acquisition time (s)', ylabel='Motor-equivalent speed (rad/s)', title='Native measured joint rates')
    rotor.grid(alpha=.2)
    rotor.legend(ncol=4, fontsize=7)
    fig.suptitle(f'{path.stem}: {review["review_status"]} | Native RMF / original Lee / Gazebo Fortress')
    output = path.with_name(path.stem+'_trajectory.png')
    fig.savefig(output, dpi=170)
    plt.close(fig)
    return str(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('result', type=Path)
    parser.add_argument('--no-plot', action='store_true')
    args = parser.parse_args()
    path = args.result.resolve()
    result = json.loads(path.read_text())
    history_path = file_for(path, result, 'history_file', '_history.json')
    imu_path = file_for(path, result, 'imu_history_file', '_imu_history.json')
    model_path = file_for(path, result, 'model_file', '_model.json')
    history = json.loads(history_path.read_text())
    retained = json.loads(imu_path.read_text()) if imu_path.exists() else {}
    metadata = json.loads(model_path.read_text()) if model_path.exists() else {}
    odom = Odometry(history.get('odometry', []))
    task, events = task_events(history, result)
    phases = phase_review(odom, events)
    actions = action_review(history, odom)
    continuations = continuation_review(history, odom)
    contacts = contact_review(history, odom, events)
    actuators = actuator_review(history, metadata)
    imu = imu_review(result, retained)
    flight = flight_review(history, task, odom)
    source_checks = result.get('checks', {})
    root_checks = task.get('root_checks', [])
    request_counts = task.get('requests', {})
    checks = {'mission_complete': task.get('state') == 'COMPLETE',
              'native_odometry_valid_monotonic': bool(odom.rows) and not (odom.invalid or odom.nonmonotonic),
              'single_start_request': result.get('start_requests') == 1,
              'observer_did_not_publish_trajectory': result.get('observer_trajectory_commands') == 0,
              'single_stage_commands': all(request_counts.get(k) == 1 for k in
                                           ('takeoff', 'set_home', 'exploration', 'landing')),
              'nominal_mission_no_stop_or_hold': all(request_counts.get(k) == 0 for k in ('stop', 'hold')),
              'accepted_extended_root': any(r.get('valid') is True for r in root_checks),
              'local_and_home_actions': actions['mode_counts'].get('local', 0)+continuations['count'] == result.get('requested_local_cycles', 1)
                  and actions['mode_counts'].get('homing', 0) == 1,
              'action_endpoints': all(a['endpoint_within_original_limits'] is True for a in actions['completed'])
                  if actions['completed'] else None,
              'no_recorded_non_support_contact': contacts['non_support_messages'] == 0,
              'all_contact_geometry_classified': contacts['unclassified_messages'] == 0,
              'native_touchdown_confirmed': actuators['landed_confirmation_source_s'] is not None,
              'post_touchdown_native_bottom_contact': contacts['last_support_source_s'] >=
                  actuators['post_touchdown_window_start_s']
                  if contacts['last_support_source_s'] is not None and
                  actuators['post_touchdown_window_start_s'] is not None else None,
              'all_four_motor_commands_zero': actuators['all_four_commands_zero'],
              'all_four_measured_rotors_stopped': actuators['all_four_measured_rotors_stopped'],
              'all_six_imu_streams_valid': len(result.get('imu_summary', {})) == 6 and
                  all(s.get('valid') for s in result.get('imu_summary', {}).values()),
              'retained_imu_values_valid': len(imu['independent_retained_checks']) == 6 and
                  all(s['unique_positive_stamps'] and s['finite'] and len(s['frames']) == 1
                      for s in imu['independent_retained_checks'].values()),
              'sources_unchanged_during_run': source_checks.get('sources_unchanged_during_run'),
              'clean_shutdown': source_checks.get('clean_shutdown')}
    if result.get('require_moving_continuation'):
        checks['native_moving_continuation'] = continuations['moving_count'] > 0
    timed_appends = [c for c in continuations['commits'] if c.get('strategy') == 'append_timed']
    if timed_appends:
        checks['timed_append_evidence_verified'] = all(c['append_evidence_verified'] for c in timed_appends)
    # Failed source runs are never upgraded, even if isolated numerical checks pass.
    status = ('failed' if result.get('status') != 'passed' or any(v is False for v in checks.values())
              else 'incomplete' if any(v is None for v in checks.values()) else 'passed')
    inputs = [path, history_path, imu_path, model_path, path.with_suffix('.log')]
    log_lines = path.with_suffix('.log').read_text(errors='replace').splitlines() if path.with_suffix('.log').exists() else []
    review = {'source_result': path.name, 'original_status': result.get('status'), 'original_error': result.get('error'),
              'review_status': status, 'review_kind': 'Offline independent evidence review; no simulation rerun',
              'analysis_script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'input_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs if p.exists()},
              'continuations': continuations, 'checks': checks, 'failed_checks': [k for k, v in checks.items() if v is False],
              'unobserved_checks': [k for k, v in checks.items() if v is None],
              'task_final_state': task.get('state'), 'task_failure': task.get('failure'),
              'task_reason': task.get('reason'), 'task_requests': task.get('requests'),
              'failure_cleanup': {key: task.get(key) for key in ('stop_result', 'hold_result', 'execution_release')},
              'root_validation': {'attempts': len(root_checks),
                  'accepted': sum(r.get('valid') is True for r in root_checks),
                  'reason_counts': dict(Counter(r.get('reason') for r in root_checks)),
                  'checks': root_checks, 'last_map': result.get('final', {}).get('map'),
                  'semantics': 'current_pose_not_free alone does not distinguish unknown from occupied cells. '
                               'This review does not clear or reinterpret the recorded map.'},
              'node_exit_failures': result.get('node_exit_failures'),
              'log_error_lines': [line for line in log_lines if any(marker in line for marker in
                                  ('AttributeError:', 'RuntimeError:', 'Traceback', 'process has died'))][-12:],
              'phase_time_semantics': 'Mission events use the node ROS clock; sample endpoints and all travel '
                  'use native odometry acquisition stamps. Receipt task_phase tags are not used to relabel old '
                  'supervised runs. Consecutive observed increments are assigned to their later sample phase.',
              'phases': phases, 'odometry': {'samples': len(odom.rows), 'invalid': odom.invalid,
                  'nonmonotonic': odom.nonmonotonic, 'source_start_s': odom.times[0] if odom.times else None,
                  'source_end_s': odom.times[-1] if odom.times else None,
                  'total_observed_distance_m': travelled(odom.rows),
                  'autonomous_exploration_and_return_distance_m': sum(r['distance_xyz_m'] for r in phases if r['phase'] == 'EXPLORING'),
                  'maximum_source_gap_s': max((b-a for a, b in zip(odom.times, odom.times[1:])), default=None)},
              'flight': flight, 'actions': actions, 'contacts': contacts, 'actuators': actuators, 'imu': imu,
              'wall_elapsed_s': result.get('wall_elapsed_s'), 'trajectory_plot': None}
    if not args.no_plot:
        try:
            review['trajectory_plot'] = plot(path, history, odom, events, review)
        except ImportError as error:
            review['plot_unavailable'] = str(error)
    output = path.with_name(path.stem+'_review.json')
    output.write_text(json.dumps(review, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'review': str(output), 'review_status': status, 'task_final_state': task.get('state'),
                      'task_failure': task.get('failure'), 'failed_checks': review['failed_checks'],
                      'odometry': review['odometry'], 'flight': flight, 'trajectory_plot': review['trajectory_plot']},
                     ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if status == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
