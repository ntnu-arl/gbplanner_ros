#!/usr/bin/env python3
"""Compare recorded preturns without feeding truth back into the experiment.

All poses retain their original map origin. Velocity is derived from adjacent
truth poses, not the delayed simulator twist. The first 1.1 s is a common
reporting window, not a claimed settling time of the acceleration-limited run.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE.parent / 'state_estimation_shadow'))
from analyze_trial import interpolate, read_events, sha256, trajectory, wrap
from analyze_execution import initial_stage, score_stage

ELL = .2159


def load(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def direction(yaw):
    return np.column_stack((np.cos(yaw), np.sin(yaw)))


def vector_track(events, kind, key):
    unique = {e['stamp_ns']: e for e in events if e.get('kind') == kind}
    rows = [unique[t] for t in sorted(unique)]
    if len(rows) < 2:
        raise ValueError(f'{kind}: insufficient observations')
    return {'times': np.array([e['stamp_ns']*1.e-9 for e in rows]),
            'values': np.array([e[key] for e in rows], dtype=float)}


class Recording:
    def __init__(self, path, max_gap):
        self.path = Path(path).resolve()
        self.run = load(self.path/'run.json')
        if self.run.get('status') not in ('completed', 'failed'):
            raise ValueError(f'{self.path.name}: still running; do not analyze a live snapshot')
        self.planned = load(self.path/'planned_execution.json')
        if not math.isclose(float(self.planned['offset_m']), ELL, abs_tol=1.e-12):
            raise ValueError('This comparison requires the frozen ell = 0.2159 m')
        if (self.planned['goal_tolerance_xy_m'] != .04 or
                self.planned['goal_tolerance_yaw_rad'] != .1):
            raise ValueError('Original 4 cm / 0.1 rad scoring standard changed')
        self.max_gap = max_gap
        self.events = read_events(self.path/'events.jsonl')
        self.tracks = {k: trajectory(self.events, k, 0) for k in ('truth', 'mapping_input')}
        for k, track in self.tracks.items():
            if track['metadata']['frame_ids'] != ['map'] or track['metadata']['child_frame_ids'] != ['base_link']:
                raise ValueError(f'{k} must already be map/base_link; this script never realigns poses')
        self.wheel = vector_track(self.events, 'wheel_raw', 'linear')
        self.gyro = vector_track(self.events, 'imu_input', 'angular')
        self.alignment = next(s for s in self.planned['stages'] if s['kind'] == 'alignment')
        for key in ('start_sim_ns', 'action_result_sim_ns', 'settled_end_sim_ns'):
            if self.alignment.get(key) is None:
                raise ValueError(f'{self.path.name}: alignment has no {key}; no complete candidate comparison')

    def at(self, track, times):
        values, valid = interpolate(track, np.asarray(times, dtype=float), self.max_gap)
        if not valid.all():
            raise ValueError('Missing synchronous interpolation brackets; never extrapolate')
        return values

    def pose_state(self, seconds):
        truth = self.at(self.tracks['truth'], [seconds])[0]
        estimate = self.at(self.tracks['mapping_input'], [seconds])[0]
        c = truth[:2]+ELL*direction([truth[2]])[0]
        chat = estimate[:2]+ELL*direction([estimate[2]])[0]
        return {'sim_seconds': seconds, 'truth_B_xy_yaw': truth.tolist(),
                'estimated_B_xy_yaw': estimate.tolist(),
                'B_error_world_xy_m': (estimate[:2]-truth[:2]).tolist(),
                'B_error_norm_m': float(np.linalg.norm(estimate[:2]-truth[:2])),
                'C_error_world_xy_m': (chat-c).tolist(),
                'C_error_norm_m': float(np.linalg.norm(chat-c)),
                'yaw_error_rad': float(wrap(estimate[2]-truth[2]))}

    def interval(self, start, end):
        if end <= start:
            raise ValueError('Nonpositive diagnostic interval')
        native = self.tracks['truth']['times']
        # Avoid two floating point representations of the same boundary.
        times = np.r_[start, native[(native > start+1.e-9) & (native < end-1.e-9)], end]
        dt = np.diff(times)
        if np.any(dt > self.max_gap+1.e-9) or np.any(dt <= 0):
            raise ValueError('Truth sample gap in diagnostic interval')
        midpoint = (times[:-1]+times[1:])*.5
        truth = self.at(self.tracks['truth'], times)
        estimate = self.at(self.tracks['mapping_input'], times)
        forward = direction((truth[:-1, 2]+truth[1:, 2])*.5)
        lateral = np.column_stack((-forward[:, 1], forward[:, 0]))
        estimated_forward = direction((estimate[:-1, 2]+estimate[1:, 2])*.5)
        db = np.diff(truth[:, :2], axis=0)
        omega = np.diff(truth[:, 2])/dt
        c = truth[:, :2]+ELL*direction(truth[:, 2])
        chat = estimate[:, :2]+ELL*direction(estimate[:, 2])
        dc = np.diff(c, axis=0)
        cx = np.sum(dc*forward, axis=1)/dt
        cy = np.sum(dc*lateral, axis=1)/dt
        vx = np.sum(db*forward, axis=1)/dt
        vy = np.sum(db*lateral, axis=1)/dt
        wheel = self.at(self.wheel, midpoint)[:, 0]
        gyro = self.at(self.gyro, midpoint)[:, 2]
        iwtrue = np.sum(wheel[:, None]*forward*dt[:, None], axis=0)
        iwest = np.sum(wheel[:, None]*estimated_forward*dt[:, None], axis=0)
        terms = {
            'longitudinal_wheel_minus_true_C_world_xy_m': np.sum((wheel-cx)[:, None]*forward*dt[:, None], axis=0),
            'unmodeled_true_C_lateral_world_xy_m': -np.sum(cy[:, None]*lateral*dt[:, None], axis=0),
            'heading_mismatch_world_xy_m': iwest-iwtrue,
            'EKF_vs_wheel_integral_world_xy_m': chat[-1]-chat[0]-iwest,
        }
        error_change_c = (chat[-1]-c[-1])-(chat[0]-c[0])
        error_change_b = (estimate[-1, :2]-truth[-1, :2])-(estimate[0, :2]-truth[0, :2])
        lever = ELL*((direction([estimate[-1, 2]])-direction([truth[-1, 2]]))-
                     (direction([estimate[0, 2]])-direction([truth[0, 2]])))[0]
        sum_terms = sum(terms.values())
        decomposition = {k: v.tolist() for k, v in terms.items()}
        decomposition.update(sum_world_xy_m=sum_terms.tolist(),
                             observed_C_error_change_world_xy_m=error_change_c.tolist(),
                             observed_B_error_change_world_xy_m=error_change_b.tolist(),
                             yaw_lever_arm_change_world_xy_m=lever.tolist(),
                             closure_error_m=float(np.linalg.norm(sum_terms-error_change_c)),
                             B_closure_error_m=float(np.linalg.norm(error_change_c-lever-error_change_b)))
        integral = lambda v: float(np.sum(v*dt))
        rms = lambda v: float(np.sqrt(np.sum(v*v*dt)/np.sum(dt)))
        c_motion = np.linalg.norm(c-c[0], axis=1)
        error = np.linalg.norm(estimate[:, :2]-truth[:, :2], axis=1)
        report = {
            'start_s': start, 'end_s': end, 'duration_s': end-start, 'intervals': len(dt),
            'start_state': self.pose_state(start), 'end_state': self.pose_state(end),
            'true_C_lateral_integral_body_m': integral(cy),
            'true_C_lateral_absolute_integral_body_m': integral(np.abs(cy)),
            'true_C_lateral_RMS_mps': rms(cy), 'true_C_lateral_peak_abs_mps': float(np.max(np.abs(cy))),
            'continuous_vBy_plus_ell_omega_integral_m': integral(vy+ELL*omega),
            'true_C_displacement_world_xy_m': (c[-1]-c[0]).tolist(),
            'true_C_displacement_norm_m': float(c_motion[-1]),
            'true_C_max_displacement_from_interval_start_m': float(c_motion.max()),
            'estimated_C_displacement_world_xy_m': (chat[-1]-chat[0]).tolist(),
            'wheel_forward_minus_true_B_integral_m': integral(wheel-vx),
            'true_yaw_change_rad': float(truth[-1, 2]-truth[0, 2]),
            'true_yaw_rate_peak_abs_rad_s': float(np.max(np.abs(omega))),
            'gyro_midpoint_integral_minus_true_yaw_change_rad': integral(gyro)-float(truth[-1, 2]-truth[0, 2]),
            'B_estimation_RMSE_m': float(np.sqrt(np.sum(.5*(error[:-1]**2+error[1:]**2)*dt)/np.sum(dt))),
            'decomposition': decomposition,
        }
        curves = {'time_s': midpoint, 'pose_time_s': times, 'dt_s': dt,
                  'yaw_rate_rad_s': omega, 'C_lateral_mps': cy,
                  'C_lateral_cumulative_body_m': np.r_[0., np.cumsum(cy*dt)],
                  'C_displacement_world_xy_m': c-c[0], 'B_estimation_error_m': error,
                  'wheel_vx_mps': wheel, 'true_B_vx_mps': vx, 'true_B_vy_mps': vy,
                  'gyro_rad_s': gyro, 'true_C_vx_mps': cx}
        return report, curves

    def relay_check(self):
        rows = [e for e in self.events if e.get('kind') == 'tracker_command_relay']
        accel = float(self.run.get('alignment_yaw_accel_rad_s2', 0.))
        if not rows:
            return {'status': 'relay_not_enabled' if accel == 0 else 'missing_relay_evidence',
                    'limit_rad_s2': accel, 'record_count': 0, 'passed': accel == 0}
        violations = {'outside_alignment_changed': [], 'linear_x_changed': [],
                      'zero_command_delayed': [], 'slew_limit_exceeded': [], 'angular_limit_exceeded': []}
        resets = [e['stamp_ns'] for e in self.events if e.get('kind') == 'phase'
                  and e.get('name') == 'planned_alignment']
        changed = 0
        prev = None
        for i, row in enumerate(rows):
            before, after = row['input_angular_z'], row['output_angular_z']
            differs = abs(before-after) > 1.e-12
            changed += differs
            if row['phase'] != 'planned_alignment' and differs:
                violations['outside_alignment_changed'].append(i)
            if row['input_linear_x'] != row['output_linear_x']:
                violations['linear_x_changed'].append(i)
            if row['input_linear_x'] == 0. and before == 0. and after != 0.:
                violations['zero_command_delayed'].append(i)
            if abs(after) > .5+1.e-12:
                violations['angular_limit_exceeded'].append(i)
            if row['limit_active']:
                # The relay resets its previous yaw command at entry into
                # planned_alignment; check that first sample as well.
                same_phase = prev is not None and prev['phase'] == row['phase']
                # set_phase() stores Recorder.clock_ns. Action start is read
                # separately from the ROS clock and may be a few ms later.
                previous_stamp = (prev['stamp_ns'] if same_phase else
                                  max(t for t in resets if t <= row['stamp_ns']))
                previous_yaw = prev['output_angular_z'] if same_phase else 0.
                dt = max(0., min(.1, (row['stamp_ns']-previous_stamp)*1.e-9))
                if abs(after-previous_yaw) > accel*dt+1.e-10:
                    violations['slew_limit_exceeded'].append(i)
            prev = row
        return {'status': 'checked', 'limit_rad_s2': accel, 'record_count': len(rows),
                'records_changed': int(changed),
                'phases_changed': sorted({r['phase'] for r in rows if abs(r['input_angular_z']-r['output_angular_z']) > 1.e-12}),
                'maximum_output_abs_angular_z_rad_s': max(abs(r['output_angular_z']) for r in rows),
                'violation_counts': {k: len(v) for k, v in violations.items()},
                'first_violation_indices': {k: v[:5] for k, v in violations.items()},
                'passed': not any(violations.values())}

    def summary(self):
        a = self.alignment['start_sim_ns']*1.e-9
        b = self.alignment['action_result_sim_ns']*1.e-9
        c = self.alignment['settled_end_sim_ns']*1.e-9
        boundary = min(a+1.1, b)
        intervals = {'alignment_control': (a, b), 'alignment_including_settling': (a, c),
                     'alignment_first_1p1s': (a, boundary), 'alignment_settling': (b, c)}
        if boundary < b:
            intervals['alignment_after_1p1s_control'] = (boundary, b)
        ret = next((s for s in self.planned['stages'] if s['kind'] == 'return'), None)
        if ret is not None and all(ret.get(k) is not None for k in ('start_sim_ns', 'action_result_sim_ns', 'settled_end_sim_ns')):
            intervals['return_control'] = (ret['start_sim_ns']*1.e-9, ret['action_result_sim_ns']*1.e-9)
            intervals['return_including_settling'] = (ret['start_sim_ns']*1.e-9, ret['settled_end_sim_ns']*1.e-9)
        summaries = {k: self.interval(*v)[0] for k, v in intervals.items()}
        curves = self.interval(a, c)[1]
        # A descriptive rise time relative to each run's observed peak. It
        # does not declare any later interval free of contact transients.
        peak = float(np.max(np.abs(curves['yaw_rate_rad_s'])))
        ramp = {}
        for fraction in (.1, .5, .9):
            hits = np.flatnonzero(np.abs(curves['yaw_rate_rad_s']) >= fraction*peak)
            ramp[f'first_time_to_{int(fraction*100)}pct_observed_peak_s'] = float(curves['time_s'][hits[0]]-a) if len(hits) else None
        stages = list(self.planned['stages'])
        first = initial_stage(self.run, self.events, self.tracks['truth']['rows'][-1]['stamp_ns'])
        if first:
            stages.insert(0, first)
        scored = []
        for i, stage in enumerate(stages):
            next_start = next((s['start_sim_ns'] for s in stages[i+1:] if s.get('start_sim_ns') is not None), None)
            result, _ = score_stage(stage, self.tracks['truth'], self.tracks['mapping_input'], ELL,
                                    .04, .1, self.max_gap, next_start)
            scored.append(result)
        result = {'run_directory': str(self.path), 'run_status': self.run['status'],
                  'planned_execution_status': self.planned['status'],
                  'inputs_sha256': {n: sha256(self.path/n) for n in ('run.json', 'events.jsonl', 'planned_execution.json', 'tracking_setup.json')},
                  'production_files_unchanged': self.run.get('production_files_unchanged'),
                  'yaw_acceleration_limit_rad_s2': float(self.run.get('alignment_yaw_accel_rad_s2', 0.)),
                  'alignment_start_sim_seconds': a, 'alignment_control_duration_s': b-a,
                  'intervals': summaries, 'descriptive_yaw_rate_rise': ramp,
                  'stages': scored, 'command_relay': self.relay_check()}
        return result, curves


def compact_stage(summary, name):
    stage = next((s for s in summary['stages'] if s['kind'] == name), {})
    return {k: stage.get(k) for k in ('action_protocol_success', 'at_action_result',
                                     'end_of_settling', 'true_goal_success_after_settling')}


def write_csv(path, curves, start):
    columns = ('time_s', 'dt_s', 'yaw_rate_rad_s', 'C_lateral_mps', 'wheel_vx_mps',
               'true_B_vx_mps', 'true_B_vy_mps', 'gyro_rad_s', 'true_C_vx_mps')
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['relative_time_mid_s', *columns[1:]])
        for i in range(len(curves['time_s'])):
            writer.writerow([curves['time_s'][i]-start, *[curves[c][i] for c in columns[1:]]])


def plot(out, summaries, curves):
    os.environ.setdefault('MPLCONFIGDIR', '/tmp/gbplanner-matplotlib')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from plot_fonts import plot_font_family
    family = plot_font_family(ROOT/'docs/course_report/fonts/HeitiSC.ttf')
    plt.rcParams.update({'font.family': family,
                         'font.size': 9, 'legend.fontsize': 7, 'axes.unicode_minus': False,
                         'pdf.fonttype': 42, 'axes.spines.top': False, 'axes.spines.right': False})
    fig, axes = plt.subplots(2, 3, figsize=(11., 6.4), layout='constrained')
    labels = ('原预转向', '限角加速度预转向')
    colors = ('#b65d44', '#237c89')
    for i, (summary, series, label, color) in enumerate(zip(summaries, curves, labels, colors)):
        a = summary['alignment_start_sim_seconds']
        t = series['time_s']-a
        tp = series['pose_time_s']-a
        axes[0, 0].plot(t, series['yaw_rate_rad_s'], label=label, color=color)
        axes[0, 1].plot(t, series['C_lateral_mps']*1000, label=label, color=color)
        axes[0, 2].plot(tp, series['C_lateral_cumulative_body_m']*1000, label=label, color=color)
        xy = series['C_displacement_world_xy_m']*1000
        axes[1, 0].plot(xy[:, 0], xy[:, 1], label=label, color=color)
        axes[1, 0].scatter(xy[-1, 0], xy[-1, 1], color=color, s=15)
        axes[1, 1].plot(tp, series['B_estimation_error_m']*1000, label=label, color=color)
        for name, j in (('initial_fixed', 0), ('alignment', 1), ('return', 2)):
            stage = compact_stage(summary, name)
            error = (stage.get('end_of_settling') or {}).get('position_error_m')
            if error is not None:
                axes[1, 2].bar(j+(i-.5)*.32, error*100, width=.3, color=color,
                               label=label if j == 0 else None)
        for ax in (axes[0, 0], axes[0, 1], axes[0, 2], axes[1, 1]):
            ax.axvline(summary['alignment_control_duration_s'], color=color, ls=':', lw=.6, alpha=.6)
    # Show the full physical aspect ratio in a readable square panel, rather
    # than compressing the nearly vertical displacement into a sliver.
    all_xy = np.vstack([c['C_displacement_world_xy_m']*1000 for c in curves])
    lo, hi = all_xy.min(axis=0), all_xy.max(axis=0)
    center, half = (lo+hi)*.5, max(float(np.max(hi-lo))*.56, 1.)
    axes[1, 0].set(xlim=(center[0]-half, center[0]+half),
                   ylim=(center[1]-half, center[1]+half))
    axes[0, 0].set(title='(a) 实际偏航角速度', xlabel='预转向开始后 / s', ylabel='角速度 / rad/s')
    axes[0, 1].set(title='(b) 实际 C 侧向速度', xlabel='预转向开始后 / s', ylabel='速度 / mm/s')
    axes[0, 2].set(title='(c) C 侧向速度的有符号积分', xlabel='预转向开始后 / s', ylabel='积分 / mm')
    axes[1, 0].set(title='(d) C 相对各自真实起点的位移', xlabel='世界 x 位移 / mm', ylabel='世界 y 位移 / mm', aspect='equal')
    axes[1, 1].set(title='(e) B 的绝对地图估计误差', xlabel='预转向开始后 / s', ylabel='位置误差 / mm')
    axes[1, 2].set(title='(f) 各阶段真实停稳位置误差', xticks=[0, 1, 2], xticklabels=['去程', '预转向', '返回'], ylabel='位置误差 / cm')
    axes[1, 2].axhline(4., ls='--', lw=.8, color='#555555', label='原 4 cm 标准')
    for ax in axes.flat:
        ax.grid(alpha=.18)
        ax.legend(frameon=False)
    fig.suptitle('同一固定中心模型下的转向方式对比', fontsize=12)
    for ext in ('png', 'pdf'):
        fig.savefig(out/('alignment_comparison.'+ext), dpi=220, bbox_inches='tight')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', required=True, type=Path)
    parser.add_argument('--candidate', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--max-gap', default=.1, type=float)
    args = parser.parse_args()
    if not math.isfinite(args.max_gap) or args.max_gap <= 0:
        parser.error('--max-gap must be finite and positive')
    recordings = [Recording(p, args.max_gap) for p in (args.baseline, args.candidate)]
    output = args.output.resolve()
    if any(output == r.path or r.path in output.parents for r in recordings):
        parser.error('--output must be outside both raw run directories')
    data = [r.summary() for r in recordings]
    summaries, curves = list(zip(*data))
    keys = ('controller_source', 'control_point', 'wheel_velocity_window', 'calibration_duration_s',
            'accuracy_profile', 'route_profile', 'path_profile', 'imu_seed', 'turning_center_offset_m')
    parameters = {key: [r.run.get(key) for r in recordings] for key in keys}
    tracker_inputs = [load(r.path/'tracking_setup.json') for r in recordings]
    binary_keys = ('pure_pursuit_node', 'libpure_pursuit.so', 'smb.yaml')
    tracker_hashes = {key: [t['source_sha256'][key] for t in tracker_inputs] for key in binary_keys}
    result = {
        'status': 'analyzed', 'offset_m': ELL, 'goal_tolerance_xy_m': .04,
        'goal_tolerance_yaw_rad': .1, 'maximum_interpolation_gap_s': args.max_gap,
        'coordinate_policy': 'Original map coordinates; no initial-pose or full-trajectory refitting',
        'velocity_source': 'Finite difference of native truth pose at interval midpoint; not delayed truth twist',
        'first_1p1s_meaning': 'A common reporting window, not a settled-speed boundary in the limited run',
        'decomposition_meaning': 'Exact vector accounting conditional on wheel-vx and fixed ell; term norms must not be summed as scalar causal shares',
        'comparison_scope': 'Independent physical executions with the same declared settings, not identical-state replay or a statistical success-rate study',
        'parameters': parameters, 'listed_parameters_equal': all(v[0] == v[1] for v in parameters.values()),
        'tracker_binary_and_source_config_hashes': tracker_hashes,
        'same_tracker_binary_and_source_config': all(v[0] == v[1] for v in tracker_hashes.values()),
        'source_sha256': {str(p.relative_to(ROOT)): sha256(p) for p in (Path(__file__), HERE/'analyze_execution.py', HERE.parent/'state_estimation_shadow/analyze_trial.py')},
        'baseline': summaries[0], 'candidate': summaries[1],
        'endpoint_comparison': {name: {'baseline': compact_stage(summaries[0], name),
                                       'candidate': compact_stage(summaries[1], name)}
                                for name in ('initial_fixed', 'alignment', 'return')},
    }
    a = summaries[0]['intervals']['alignment_including_settling']
    b = summaries[1]['intervals']['alignment_including_settling']
    result['alignment_changes'] = {key: {'baseline': a[key], 'candidate': b[key],
                                         'candidate_minus_baseline': b[key]-a[key]}
                                    for key in ('duration_s', 'true_C_lateral_integral_body_m',
                                                'true_C_lateral_absolute_integral_body_m',
                                                'true_C_max_displacement_from_interval_start_m',
                                                'B_estimation_RMSE_m')}
    output.mkdir(parents=True, exist_ok=True)
    (output/'comparison.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    for name, summary, series in zip(('baseline', 'candidate'), summaries, curves):
        write_csv(output/(name+'_alignment_velocity.csv'), series, summary['alignment_start_sim_seconds'])
    plot(output, summaries, curves)
    print(json.dumps({'output': str(output), 'alignment_changes': result['alignment_changes'],
                      'candidate_relay': summaries[1]['command_relay'],
                      'return': result['endpoint_comparison']['return']}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
