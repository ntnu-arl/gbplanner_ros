#!/usr/bin/env python3
"""Plot one recorded native RMF mission without ROS, simulation, or resampling.

Default input: results/rmf_continuation/native2.json and native2_history.json.
Published commands are plotted as steps at header stamp + time_from_start.
Controller status supplies a bracket, not an invented exact replacement time.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/gbplanner_matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.ticker import FuncFormatter
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
COLORS = {'old': '#c47a16', 'new': '#1769aa', 'home': '#278468',
          'actual': '#202b36', 'muted': '#667380', 'seam': '#7552a0'}


def digest(path):
    hasher = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            hasher.update(chunk)
    return hasher.hexdigest()


def require(condition, message):
    if not condition:
        raise ValueError(message)


def event_time(events, state):
    return next(float(row['ros_s']) for row in events if row['state'] == state)


def trajectory_arrays(record):
    offsets = np.rint([sample['offset_s'] * 1e9 for sample in record['samples']]).astype(np.int64)
    require(np.all(np.diff(offsets) > 0), 'Trajectory offsets are not strictly increasing')
    velocity = np.array([sample['velocities'][0][:3] for sample in record['samples']], dtype=float)
    position = np.array([sample['poses'][0]['position'] for sample in record['samples']], dtype=float)
    return {'record': record, 'offset_ns': offsets,
            'time': (record['stamp_ns'] + offsets) * 1e-9,
            'velocity': velocity, 'speed': np.linalg.norm(velocity, axis=1), 'position': position}


def world_velocity(body, quaternion):
    # The observer stores body-frame twist. Rotate with the same odometry
    # quaternion; norms alone do not require a frame conversion.
    q = quaternion / np.linalg.norm(quaternion, axis=1)[:, None]
    u, w = q[:, :3], q[:, 3:]
    cross = np.cross(u, body)
    return body + 2 * (w * cross + np.cross(u, cross))


def held_command_status(history, trajectories):
    by_stamp = {item['record']['stamp_ns']: item for item in trajectories}
    selected, mismatches, max_error = [], [], 0.
    for row in history['controller']:
        stamp = round(row['trajectory_origin_s'] * 1e9)
        if stamp not in by_stamp or row['samples'] == 0:
            continue
        command = by_stamp[stamp]
        elapsed = max(0, round(row['odometry_stamp_s'] * 1e9) - stamp)
        expected = max(0, int(np.searchsorted(command['offset_ns'], elapsed, side='right')) - 1)
        observed = row['sample_index']
        require(0 <= observed < len(command['offset_ns']), 'Invalid recorded controller sample index')
        if expected != observed:
            mismatches.append({'odom_s': row['odometry_stamp_s'], 'expected': expected, 'observed': observed})
        error = np.max(np.abs(np.array(row['desired_position']) - command['position'][observed]))
        max_error = max(max_error, float(error))
        selected.append({'odom_s': row['odometry_stamp_s'], 'origin_ns': stamp,
                         'sample_index': observed, 'speed': float(command['speed'][observed]),
                         'velocity': command['velocity'][observed].tolist()})
    require(not mismatches, 'Recorded controller indices disagree with piecewise-held scheduling')
    require(max_error < 1e-8, 'Recorded desired positions disagree with selected trajectory samples')
    return selected, {'checked_status_records': len(selected), 'sample_index_mismatches': mismatches,
                      'max_desired_position_difference_m': max_error,
                      'formula': 'max(0, searchsorted(offset_ns, max(0, odom_stamp_ns - header_stamp_ns), right) - 1)',
                      'plot_rule': 'Post-step published samples at header + offset; no polynomial interpolation.'}


def configure_style():
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10.5,
                         'axes.titlesize': 12, 'axes.labelsize': 11,
                         'axes.spines.top': False, 'axes.spines.right': False,
                         'axes.grid': True, 'grid.alpha': .18, 'grid.linewidth': .65,
                         'legend.frameon': False, 'savefig.facecolor': 'white'})


def mission_plot(destination, data, analysis):
    time, position = data['time'], data['position']
    commit = analysis['continuation']['commit']
    seam = commit['new_trajectory_stamp_ns'] * 1e-9
    old_start = commit['old_trajectory_stamp_ns'] * 1e-9
    home_start = data['trajectories'][2]['record']['stamp_ns'] * 1e-9
    landing = analysis['mission']['landing_request_s']
    complete = analysis['mission']['complete_event_s']
    fig = plt.figure(figsize=(13.8, 9.0))
    grid = fig.add_gridspec(3, 2, width_ratios=[.88, 1.45], height_ratios=[1, 1, .66],
                          left=.07, right=.975, bottom=.18, top=.86, wspace=.30, hspace=.43)
    xy, altitude, known, notes = (fig.add_subplot(grid[:, 0]), fig.add_subplot(grid[0, 1]),
                                 fig.add_subplot(grid[1, 1]), fig.add_subplot(grid[2, 1]))
    xy.plot(position[:, 0], position[:, 1], color='#b8c0c7', lw=1.1)
    for start, end, color, label in [(old_start, seam, COLORS['old'], 'Initial exploration'),
                                   (seam, home_start, COLORS['new'], 'Continued exploration'),
                                   (home_start, landing, COLORS['home'], 'Return home')]:
        mask = (time >= start) & (time <= end)
        xy.plot(position[mask, 0], position[mask, 1], color=color, lw=1.65, label=label)
    xy.scatter([position[0, 0]], [position[0, 1]], marker='s', s=58,
               facecolors='white', edgecolors=COLORS['actual'], zorder=5)
    xy.annotate('Home / landing', (0, 0), xytext=(-.25, 1.8),
                arrowprops={'arrowstyle': '-', 'color': COLORS['muted']}, ha='right', fontsize=10)
    cpos = np.array(commit['position'][:3])
    xy.scatter(cpos[0], cpos[1], s=40, color=COLORS['seam'], zorder=6)
    xy.annotate('Moving continuation', cpos[:2], xytext=(-5.7, -.7), fontsize=10,
                arrowprops={'arrowstyle': '-', 'color': COLORS['seam']})
    xy.set(xlabel='World x (m)', ylabel='World y (m)', title='A  Flight path')
    xy.set_aspect('equal', adjustable='box')
    xy.set_xlim(-6.4, .75)
    xy.legend(loc='upper left', bbox_to_anchor=(0, 1.01), fontsize=9.2)

    altitude.plot(time, position[:, 2], color=COLORS['actual'], lw=1.3)
    altitude.axvspan(home_start, landing, color=COLORS['home'], alpha=.09)
    altitude.axvline(seam, color=COLORS['seam'], lw=1, ls='--')
    altitude.axvline(landing, color=COLORS['muted'], lw=1, ls=':')
    altitude.set(xlabel='Simulation time (s)', ylabel='World z (m)',
                 title='B  Altitude profile', xlim=(0, time[-1]))
    altitude.text(.02, .94, f"Maximum altitude: {position[:, 2].max():.3f} m",
                  transform=altitude.transAxes, va='top', fontsize=10)
    altitude.text(.54, .13, 'Return-home phase', color=COLORS['home'], transform=altitude.transAxes, fontsize=10)

    mt = np.array([row['stamp_sec'] + row['stamp_nanosec'] * 1e-9 for row in data['history']['map']])
    mc = np.array([row['known_cells'] for row in data['history']['map']])
    known.step(mt, mc, where='post', color=COLORS['new'], lw=1.7)
    known.scatter(mt[[0, -1]], mc[[0, -1]], color=COLORS['new'], s=24)
    known.yaxis.set_major_formatter(FuncFormatter(lambda value, _: f'{value / 1000:.0f}k'))
    known.set(xlabel='Simulation time (s)', ylabel='Known voxels',
              title='C  Map growth', xlim=(0, time[-1]))
    known.text(.02, .96, f'{mc[0]:,}  →  {mc[-1]:,} known voxels', va='top',
               transform=known.transAxes, fontsize=10)
    notes.axis('off')
    notes.text(0, 1, 'Completed exploration, return and landing', fontsize=12, weight='bold', va='top')
    notes.text(0, .75,
               f"Flight distance: {analysis['mission']['travel_3d_takeoff_to_complete_m']:.2f} m\n"
               f"Completion time: {complete:.3f} s\n"
               f"One trajectory continuation during flight",
               linespacing=1.7, va='top', fontsize=10.6)
    fig.suptitle('Autonomous exploration and return', x=.07, ha='left', y=.965,
                 fontsize=19, weight='bold')
    fig.text(.07, .915, 'Single simulated flight · aerial exploration, mapping and autonomous landing',
             fontsize=11, color=COLORS['muted'])
    fig.text(.07, .085,
             'Trajectory and altitude use recorded odometry. Map growth shows known voxels at their observation times.',
             fontsize=9.1, color=COLORS['muted'], linespacing=1.5, va='top')
    fig.savefig(destination, dpi=180)
    plt.close(fig)


def continuation_plot(destination, data, analysis):
    c = analysis['continuation']
    commit, timing = c['commit'], c['timing']
    seam = timing['new_header_s']
    lower, upper = timing['last_old_controller_odom_s'], timing['first_new_controller_odom_s']
    old, new = data['trajectories'][:2]
    fig, axs = plt.subplots(2, 2, figsize=(14, 10.6),
                            gridspec_kw={'width_ratios': [1.12, 1]}, constrained_layout=False)
    fig.subplots_adjust(left=.075, right=.975, top=.84, bottom=.22, hspace=.48, wspace=.25)

    def panel(ax, window, field, title):
        ax.axvspan(lower, upper, color=COLORS['seam'], alpha=.10, zorder=0)
        observed = data['speed'] if field == 'speed' else data['world_velocity'][:, 1]
        mask = (data['time'] >= window[0]) & (data['time'] <= window[1])
        ax.plot(data['time'][mask], observed[mask], color=COLORS['actual'], lw=1.55, label='Measured (odometry)')
        if field == 'speed':
            ax.scatter([commit['odom_stamp_s']], [commit['measured_speed_mps']], marker='D',
                       s=24, color=COLORS['seam'], zorder=6)
        for command, color, label in [(old, COLORS['old'], 'Old trajectory reference'),
                                       (new, COLORS['new'], 'New trajectory reference')]:
            values = command['speed'] if field == 'speed' else command['velocity'][:, 1]
            # No origin shift, guessed receipt activation or smooth polynomial.
            # Old schedule remains visible as a counterfactual after replacement.
            ax.step(command['time'], values, where='post', color=color, lw=1.3, ls='--', label=label)
            selected = [row for row in data['selected'] if row['origin_ns'] == command['record']['stamp_ns']
                        and window[0] <= row['odom_s'] <= window[1]]
            ax.scatter([row['odom_s'] for row in selected],
                       [row['speed'] if field == 'speed' else row['velocity'][1] for row in selected],
                       marker='o', s=28, facecolors='white', edgecolors=color, lw=1.25, zorder=5)
        ax.set(xlim=window, xlabel='Simulation time (s)', title=title,
               ylabel='3D speed (m/s)' if field == 'speed' else 'World y velocity (m/s)')
        if field == 'speed':
            ax.set_ylim(0, .8)
        else:
            in_window = [observed[mask]]
            for command in (old, new):
                command_mask = (command['time'] >= window[0]) & (command['time'] <= window[1])
                in_window.append(command['velocity'][command_mask, 1])
            values = np.concatenate(in_window)
            pad = max(.05, float(np.ptp(values)) * .1)
            ax.set_ylim(float(values.min()) - pad, float(values.max()) + pad)

    panel(axs[0, 0], (seam - 1, seam + 1), 'speed', 'A  Speed during trajectory continuation')
    axs[0, 0].legend(loc='upper left', fontsize=9)
    panel(axs[0, 1], (seam - .16, seam + .30), 'speed', 'B  Continuation interval')
    axs[0, 1].text(.025, .96, 'Shaded interval: trajectory replacement',
                   va='top', transform=axs[0, 1].transAxes, fontsize=8.8, color=COLORS['muted'])
    panel(axs[1, 0], (seam - 1, seam + 1), 'world_y', 'C  Velocity along the world y-axis')
    notes = axs[1, 1]
    notes.axis('off')
    notes.set_title('D  Flight summary', loc='left', pad=15)
    labels = [
        ('Continuation window', f'{lower:.3f}–{upper:.3f} s'),
        ('Speed at continuation',
         f"{commit['measured_speed_mps']:.3f} m/s (position difference)\n"
         f"{c['odom_twist_speed_at_commit_stamp_mps']:.3f} m/s (odometry twist)"),
        ('Old trajectory time remaining', f"{commit['old_remaining_time_s']:.3f} s"),
        ('New trajectory duration', f"{commit['new_duration_s']:.3f} s"),
        ('Return-home error (3D)', f"{analysis['mission']['return_home_error_3d_m'] * 1000:.2f} mm"),
        ('Mission completion', f"Completed and landed · {analysis['mission']['complete_event_s']:.3f} s"),
    ]
    table = notes.table(cellText=labels, colWidths=[.46, .54], cellLoc='left', bbox=[0, .03, 1, .95])
    table.auto_set_font_size(False)
    table.set_fontsize(9.2)
    for (row, col), cell in table.get_celld().items():
        cell.set_edgecolor('#d7e0e8')
        cell.set_linewidth(.65)
        cell.set_facecolor('#f2f6f9' if row % 2 == 0 else 'white')
        cell.PAD = .055
        if col == 0:
            cell.get_text().set_color(COLORS['muted'])
    fig.suptitle('Trajectory continuation during flight', x=.075, ha='left',
                 y=.967, fontsize=18, weight='bold')
    fig.text(.075, .918, 'Single simulated flight · measured velocity and piecewise-held trajectory references',
             fontsize=11, color=COLORS['muted'])
    fig.text(.075, .12,
             'The old dashed curve after replacement is an unexecuted reference. Shading bounds the recorded replacement interval.\n'
             'Circles: selected reference samples. Diamond: speed from successive positions. Return-home error is measured before descent.',
             fontsize=9.1, linespacing=1.55, color=COLORS['muted'], va='top')
    fig.savefig(destination, dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--summary', type=Path, default=ROOT / 'results/rmf_continuation/native2.json')
    parser.add_argument('--history', type=Path, help='Defaults to SUMMARY_STEM_history.json')
    args = parser.parse_args()
    summary_path = args.summary.resolve()
    history_path = args.history.resolve() if args.history else summary_path.with_name(summary_path.stem + '_history.json')
    summary, history = json.loads(summary_path.read_text()), json.loads(history_path.read_text())
    require(summary['status'] == 'passed', 'Expected the recorded successful mission')
    require(len(summary['continuation_commits']) == 1, 'This plot is scoped to exactly one continuation')
    require(len(history['trajectories']) == 3, 'Expected initial, continuation and homing trajectories')
    commit = summary['continuation_commits'][0]
    trajectories = [trajectory_arrays(record) for record in history['trajectories']]
    require(trajectories[0]['record']['stamp_ns'] == commit['old_trajectory_stamp_ns'] and
            trajectories[1]['record']['stamp_ns'] == commit['new_trajectory_stamp_ns'], 'Commit/trajectory identity mismatch')
    samples = history['odometry']
    time = np.array([row['stamp_ns'] for row in samples], dtype=np.int64) * 1e-9
    position = np.array([row['position'] for row in samples], dtype=float)
    body = np.array([row['body_linear'] for row in samples], dtype=float)
    quaternion = np.array([row['quaternion'] for row in samples], dtype=float)
    require(np.all(np.diff(time) > 0) and np.isfinite(position).all() and np.isfinite(body).all(), 'Invalid odometry series')
    speed = np.linalg.norm(body, axis=1)
    selected, holding = held_command_status(history, trajectories)
    controller_source = ROOT / 'src/gbplanner_gazebo/src/rmf_controller_node.cpp'
    controller_hash = digest(controller_source)
    require(controller_hash == summary['source_sha256_at_start'][str(controller_source.relative_to(ROOT))],
            'Current controller source differs from run provenance; review sample-holding semantics before plotting')
    events = summary['final']['task']['events']
    start, complete = event_time(events, 'TAKEOFF_REQUESTED'), event_time(events, 'COMPLETE')
    flight_mask = (time >= start) & (time <= complete)
    old_header, new_header = (commit[key] * 1e-9 for key in ('old_trajectory_stamp_ns', 'new_trajectory_stamp_ns'))
    old_status = [row for row in selected if row['origin_ns'] == commit['old_trajectory_stamp_ns']]
    new_status = [row for row in selected if row['origin_ns'] == commit['new_trajectory_stamp_ns']]
    last_old, first_new = old_status[-1]['odom_s'], new_status[0]['odom_s']
    attempts = summary['final']['control']['plan_ahead']['attempts']
    require(len(attempts) == 1, 'Expected one plan-ahead attempt')
    seam_index = next(i for i, row in enumerate(samples) if row['stamp_ns'] == round(commit['odom_stamp_s'] * 1e9))
    require(seam_index > 0, 'Missing previous odometry for commit finite difference')
    seam_fd = (position[seam_index] - position[seam_index - 1]) / (time[seam_index] - time[seam_index - 1])
    require(np.max(np.abs(seam_fd - np.array(commit['measured_velocity'][:3]))) < 1e-10 and
            abs(np.linalg.norm(seam_fd) - commit['measured_speed_mps']) < 1e-10,
            'Commit finite-difference speed differs from acquisition-stamped positions')
    follower_source = ROOT / 'src/gbplanner_execution/scripts/path_follower_node.py'
    follower_hash = digest(follower_source)
    require(follower_hash == summary['source_sha256_at_start'][str(follower_source.relative_to(ROOT))],
            'Current follower source differs from run provenance; review measured-speed semantics')
    window_stats = {}
    for width in (.1, .25, .5, 1.):
        mask = np.abs(time - new_header) <= width
        window_stats[str(width)] = {'half_width_s': width, 'samples': int(mask.sum()),
                                   'speed_min_mps': float(speed[mask].min()), 'speed_max_mps': float(speed[mask].max())}
    exec_final, control_final, task_final = (summary['final'][key] for key in ('execution', 'control', 'task'))
    returned_home = next(row for row in events if row['state'] == 'REQUEST_LANDING')
    home_error = float(np.linalg.norm(np.array(returned_home['position']) - np.array(task_final['home_position'])))
    failure_evidence = {
        'execution_counters': {key: exec_final[key] for key in ('rejected', 'cancelled', 'aborted', 'hold_commands', 'continuation_rejections')},
        'continuation_validation_attempts': commit['validation_attempts'],
        'continuation_validation_retries': max(0, commit['validation_attempts'] - 1),
        'plan_ahead_attempts': len(attempts), 'plan_ahead_retry_reasons': [row['retry_reason'] for row in attempts],
        'path_invalidation_recovery': control_final['path_invalidation_recovery'],
        'observed_retry_state_records': [dict(sim_s=row['sim_s'], state=row['state'], reason=row['reason'])
                                         for row in history['control'] if 'retry' in row['reason'] or 'REPLAN' in row['state']],
        'task_failure': task_final['failure'], 'task_stop_requests': task_final['requests']['stop'],
        'task_hold_requests': task_final['requests']['hold'],
        'validation_failures_observed': [row['validation_failure'] for row in history['execution'] if row['validation_failure']],
        'controller_rejected_commands': summary['final']['controller']['rejected_commands'],
        'controller_rejected_odometry': summary['final']['controller']['rejected_odometry'],
        'control_mission_source_rejections': control_final['mission_source_rejections'],
        'first_control_status_reporting_mission_source_rejections_s': next(row['sim_s'] for row in history['control'] if row['mission_source_rejections'] > 0),
        'node_exit_failures': summary['node_exit_failures'], 'node_start_failures': summary['node_start_failures'],
        'callback_errors': summary['callback_errors'],
        'scope': 'Continuation rejection, validation retry, replan recovery, abort/cancel/hold paths were not exercised. '
                 '24 control mission-source message rejections are separately retained; history does not identify each rejection cause.'}
    map_rows = history['map']
    analysis = {
        'status': 'completed', 'scope': 'One existing successful native run; offline analysis only; no new simulation.',
        'sources': {str(path): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in (summary_path, history_path)},
        'controller_semantics_source': {'path': str(controller_source), 'sha256': controller_hash, 'matches_run_source': True},
        'follower_velocity_semantics_source': {'path': str(follower_source), 'sha256': follower_hash, 'matches_run_source': True},
        'checks': summary['checks'], 'record_counts': summary['counts'],
        'mission': {'takeoff_request_s': start, 'landing_request_s': event_time(events, 'LANDING_REQUESTED'),
                    'complete_event_s': complete, 'odometry_first_s': float(time[0]), 'odometry_last_s': float(time[-1]),
                    'travel_3d_takeoff_to_complete_m': float(np.linalg.norm(np.diff(position[flight_mask], axis=0), axis=1).sum()),
                    'travel_xy_takeoff_to_complete_m': float(np.linalg.norm(np.diff(position[flight_mask, :2], axis=0), axis=1).sum()),
                    'maximum_z_m': float(position[:, 2].max()),
                    'maximum_horizontal_distance_from_world_origin_m': float(np.linalg.norm(position[:, :2], axis=1).max()),
                    'return_home_error_3d_m': home_error,
                    'return_home_error_measurement': {'event': returned_home['state'], 'time_s': returned_home['ros_s'],
                                                      'position': returned_home['position'],
                                                      'target': task_final['home_position'],
                                                      'semantics': 'Euclidean 3D position error at returned_home event, before landing descent.'},
                    'final_state': task_final['state'], 'final_reason': task_final['reason'],
                    'native_touchdown_confirmed': summary['final']['controller']['flight']['touchdown_confirmed'],
                    'final_motor_speed_rad_s': summary['final']['controller']['motor_speed_rad_s'],
                    'events': events},
        'mapping': {'first_known_voxels': map_rows[0]['known_cells'], 'last_known_voxels': map_rows[-1]['known_cells'],
                    'increase_known_voxels': map_rows[-1]['known_cells'] - map_rows[0]['known_cells'],
                    'first_revision': map_rows[0]['map_revision'], 'last_revision': map_rows[-1]['map_revision'],
                    'timing': 'Map observation stamp, not observer receipt clock.', 'coverage_fraction': None},
        'continuation': {'commit': commit, 'plan_ahead_attempt': attempts[0],
                         'commit_measured_speed_semantics': 'Norm of adjacent accepted odometry position differences divided by acquisition time difference.',
                         'commit_position_finite_difference_matches': True,
                         'odom_twist_speed_at_commit_stamp_mps': float(speed[seam_index]),
                         'timing': {'old_header_s': old_header, 'old_first_offset_s': trajectories[0]['record']['samples'][0]['offset_s'],
                                    'old_first_sample_s': float(trajectories[0]['time'][0]), 'new_header_s': new_header,
                                    'new_first_offset_s': trajectories[1]['record']['samples'][0]['offset_s'],
                                    'new_first_sample_s': float(trajectories[1]['time'][0]),
                                    'new_observer_receipt_sim_s': trajectories[1]['record']['sim_s'],
                                    'new_observer_receipt_wall_s': trajectories[1]['record']['wall_s'],
                                    'last_old_controller_odom_s': last_old, 'first_new_controller_odom_s': first_new,
                                    'control_ack_s': attempts[0]['ack_ros_s'], 'exact_controller_receipt_s': None},
                         'speed_windows_about_new_header': window_stats,
                         'no_smoothed_continuity_claim': True},
        'trajectories': [{'header_stamp_ns': x['record']['stamp_ns'], 'observer_sim_s': x['record']['sim_s'],
                          'samples': len(x['time']), 'first_offset_s': x['record']['samples'][0]['offset_s'],
                          'duration_s': x['record']['samples'][-1]['offset_s'], 'first_sample_s': float(x['time'][0]),
                          'last_sample_s': float(x['time'][-1])} for x in trajectories],
        'holding_verification': holding, 'failure_and_retry_evidence': failure_evidence,
        'limitations': ['Single run; no statistical reliability or general robustness conclusion.',
                        'Published schedule is not proof of exact controller receipt; controller status brackets replacement.',
                        'No interpolation of held commands; actual odometry points are joined for display.',
                        'Initial simulator settling has a recorded 83.235 m/s twist at 0.010 s; not silently corrected. '
                        'Continuation plots use their explicitly displayed time window.',
                        'Known voxel counts are not a coverage percentage or obstacle-free volume.',
                        'No real rejection snapshot exists for this successful run; failure replay and recovery remain unexercised.']}
    stem = summary_path.with_suffix('')
    destinations = {'analysis': stem.with_name(stem.name + '_analysis.json'),
                    'mission': stem.with_name(stem.name + '_mission.png'),
                    'continuation': stem.with_name(stem.name + '_continuation.png')}
    data = {'history': history, 'time': time, 'position': position, 'speed': speed,
            'world_velocity': world_velocity(body, quaternion), 'trajectories': trajectories, 'selected': selected}
    configure_style()
    mission_plot(destinations['mission'], data, analysis)
    continuation_plot(destinations['continuation'], data, analysis)
    analysis['outputs'] = {key: str(value) for key, value in destinations.items()}
    destinations['analysis'].write_text(json.dumps(analysis, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'status': 'completed', 'outputs': analysis['outputs'],
                      'complete_event_s': complete, 'commit_speed_mps': commit['measured_speed_mps'],
                      'holding_records_checked': holding['checked_status_records']}, indent=2))


if __name__ == '__main__':
    main()
