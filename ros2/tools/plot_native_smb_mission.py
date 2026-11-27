#!/usr/bin/env python3
"""Plot an existing successful native SMB mission from measured odometry.

Requires Python 3, matplotlib and numpy. No ROS or simulator is started.
Example:
  python3 tools/plot_native_smb_mission.py results/smb_dual_autonomy/native25.json \
    --output results/voxel_provenance/native25_trajectory.png

Only history.odometry poses are drawn. Command records provide publication-time
boundaries, never coordinates. Action UUIDs associate terminal outcomes, whose
stop_stamp_s is the native odometry acquisition time. Broad receipt-phase labels
are not used to split automatic exploration into local and homing segments.
"""
import argparse
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np


@dataclass
class Stage:
    label: str
    start: float
    stop: float
    color: str
    style: str = '-'


def require(condition, message):
    if not condition:
        raise ValueError(message)


def terminal_for(history, goal_id):
    """Cross-check the action's terminal record against the separate history."""
    matches = [entry['outcome'] for entry in history['execution_terminal_outcomes']
               if entry['outcome']['goal_id'] == goal_id]
    require(len(matches) == 1, 'Expected one terminal outcome for this Action UUID')
    terminal = matches[0]
    require(terminal['success'] and terminal['stop_confirmed'],
            'Plot requires a successful, confirmed endpoint for every action')
    return terminal


def stages_from_records(result, history):
    stages = []
    if result.get('bootstrap_requested'):
        commands = [entry for entry in history['commands']
                    if entry['phase'] == 'supervised_mapping'
                    and (entry['linear_x'] != 0 or entry['angular_z'] != 0)]
        require(bool(commands), 'No physical bootstrap motion commands found')
        boundary = result['physical_initialization']['post_stop_observation']['boundary']
        stages.append(Stage('Bootstrap (supervised)', commands[0]['sim_s'],
                            boundary['odom_stamp_ns'] / 1e9, '#87919f', '--'))

    search = result['autonomous_search']
    search_terminals = [entry['outcome'] for entry in history['execution_terminal_outcomes']
                        if entry['phase'] == 'autonomous_search']
    require(len(search_terminals) == 1, 'Search must have one unambiguous terminal outcome')
    search_terminal = terminal_for(history, search_terminals[0]['goal_id'])
    stages.append(Stage('Search', search['commanded_path']['sim_s'],
                        search_terminal['stop_stamp_s'], '#dc981a'))

    local_number = 0
    colors = ['#2770b4', '#6656ad', '#ce5863']
    for action in result['automatic_exploration']['actions']:
        terminal = terminal_for(history, action['terminal_outcome']['goal_id'])
        require(terminal['stop_stamp_s'] == action['terminal_outcome']['stop_stamp_s'],
                'Action and history disagree on the terminal source stamp')
        if action['mode'] == 'local':
            local_number += 1
            label, color = f'Local {local_number}', colors[(local_number - 1) % len(colors)]
        elif action['mode'] == 'homing':
            label, color = 'Return home', '#16866c'
        else:
            raise ValueError('Unsupported automatic action mode: ' + action['mode'])
        stages.append(Stage(label, action['commanded_path']['sim_s'],
                            terminal['stop_stamp_s'], color))
    require(stages[-1].label == 'Return home', 'Final action must be return home')
    require(all(a.stop <= b.start for a, b in zip(stages, stages[1:])),
            'Action observation windows overlap or are out of order')
    return stages


def stage_slice(times, stage):
    # Keep the last measured sample at/before publication to bracket the start;
    # never interpolate a pose or append a commanded endpoint.
    begin = max(0, int(np.searchsorted(times, stage.start, side='right')) - 1)
    end = int(np.searchsorted(times, stage.stop, side='right'))
    require(end > begin, 'No odometry samples in stage: ' + stage.label)
    require(abs(times[end - 1] - stage.stop) < 1e-6,
            'Terminal acquisition stamp missing from odometry: ' + stage.label)
    return slice(begin, end)


def style_axes(ax, title, xlabel='World x (m)', ylabel='World y (m)'):
    ax.set_title(title, loc='left', fontsize=12, fontweight='bold', pad=12)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_aspect('equal', adjustable='box')
    ax.set_axisbelow(True)
    ax.grid(color='#e3e7ec', linewidth=.7)
    ax.set_facecolor('#fbfcfe')
    for spine in ax.spines.values():
        spine.set_color('#cbd2dc')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('result', type=Path)
    parser.add_argument('--history', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    history_path = args.history or args.result.with_name(args.result.stem + '_history.json')
    result = json.loads(args.result.read_text())
    history = json.loads(history_path.read_text())
    require(result['status'] == 'passed', 'This plot expects a successful mission')
    require(result['initial_states']['gazebo_state']['odometry_source'] == 'native OdometryPublisher',
            'Expected recorded native Gazebo odometry provenance')
    records = history['odometry']
    times = np.array([entry['stamp_ns'] / 1e9 for entry in records])
    xy = np.array([entry['pose'][:2] for entry in records], dtype=float)
    require(np.isfinite(times).all() and np.isfinite(xy).all(), 'Nonfinite odometry')
    require((np.diff(times) > 0).all(), 'Odometry source stamps must be unique and increasing')
    stages = stages_from_records(result, history)
    slices = [stage_slice(times, stage) for stage in stages]
    home = np.array(result['mission_home']['physical_pose'][:2])
    return_xy = xy[slices[-1]][-1]
    error_m = float(np.linalg.norm(return_xy - home))
    require(abs(error_m - result['automatic_exploration']['planar_home_error_m']) < 1e-8,
            'Measured terminal XY error disagrees with the mission result')

    metadata = {
        'result': str(args.result), 'history': str(history_path),
        'input_sha256': {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                         for path in (args.result, history_path)},
        'trajectory_source': 'history.odometry.pose; native OdometryPublisher; XY projection',
        'phase_boundaries': 'Command receipt sim_s to Action terminal native stop_stamp_s; '
                            'start bracketed by preceding measured odometry. Bootstrap ends at '
                            'post_stop_observation.boundary.odom_stamp_ns. No interpolation.',
        'home_xy_m': home.tolist(), 'return_xy_m': return_xy.tolist(),
        'return_error_m': error_m, 'stages': []}
    for stage, selection in zip(stages, slices):
        points = xy[selection]
        metadata['stages'].append({
            'name': stage.label, 'command_sim_s': stage.start, 'terminal_source_s': stage.stop,
            'first_odom_source_s': float(times[selection][0]),
            'last_odom_source_s': float(times[selection][-1]), 'samples': len(points),
            'measured_xy_travel_m': float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())})

    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10,
                         'axes.labelcolor': '#344256', 'text.color': '#243247',
                         'xtick.color': '#526174', 'ytick.color': '#526174'})
    fig = plt.figure(figsize=(12.8, 9.2), facecolor='white')
    ax = fig.add_axes([.075, .18, .57, .69])
    detail = fig.add_axes([.73, .59, .235, .25])
    arrival = fig.add_axes([.73, .23, .235, .25])
    fig.text(.075, .95, 'SMB autonomous exploration and return', fontsize=19, fontweight='bold')
    fig.text(.075, .916, f'{args.result.stem}  |  Gazebo Fortress / ROS 2  |  '
             f'{sum(s.label.startswith("Local ") for s in stages)} local cycles + return completed',
             color='#526174', fontsize=11)
    style_axes(ax, 'Actual robot trajectory')
    style_axes(detail, 'Departure detail')
    style_axes(arrival, f'Return: {error_m * 100:.2f} cm XY error',
               'x relative to home (cm)', 'y relative to home (cm)')

    mission_range = slice(slices[0].start, slices[-1].stop)
    for chart in (ax, detail):
        chart.plot(xy[mission_range, 0], xy[mission_range, 1], color='#cbd2dc', lw=1.3, zorder=1)
        for stage, selection in zip(stages, slices):
            points = xy[selection]
            chart.plot(points[:, 0], points[:, 1], color=stage.color,
                       linestyle=stage.style, lw=2.3, zorder=3,
                       solid_capstyle='round')
        chart.scatter(*home, s=125, marker='*', c='#152538', edgecolors='white', linewidths=.7, zorder=7)
        chart.scatter(*return_xy, s=55, marker='o', facecolors='white', edgecolors='#16866c',
                      linewidths=1.8, zorder=8)
    for stage, selection in zip(stages, slices):
        if stage.label.startswith('Local '):
            point = xy[selection][-1]
            ax.scatter(*point, s=34, color=stage.color, edgecolors='white', linewidths=.6, zorder=6)
            ax.annotate(stage.label, point, xytext=(-52, 13), textcoords='offset points',
                        fontsize=10, color=stage.color)
    ax.annotate('Home', home, xytext=(12, -27), textcoords='offset points', fontsize=10)
    ax.annotate('Bootstrap turn', xy[slices[0]][np.argmax(xy[slices[0], 0])],
                xytext=(-50, -59), textcoords='offset points', fontsize=9, color='#687486',
                arrowprops={'arrowstyle': '-', 'color': '#87919f', 'lw': .8})
    ax.margins(.12)
    handles = [Line2D([0], [0], color=s.color, linestyle=s.style, lw=2.4, label=s.label)
               for s in stages]
    ax.legend(handles=handles, loc='upper right', frameon=True, facecolor='white',
              edgecolor='#e0e5ec', fontsize=9.5, labelspacing=.65)
    detail.set_xlim(home[0] - .32, home[0] + .91)
    detail.set_ylim(-.13, .57)
    detail.annotate('Home', home, xytext=(5, 12), textcoords='offset points', fontsize=9)
    first_local = next(i for i, stage in enumerate(stages) if stage.label == 'Local 1')
    detail.annotate('Local 1 end', xy[slices[first_local]][-1], xytext=(4, 9),
                    textcoords='offset points', fontsize=9, color=stages[first_local].color)
    returning = (xy[slices[-1]] - home) * 100
    arrival.plot(returning[:, 0], returning[:, 1], color='#16866c', lw=2.3)
    arrival.scatter(0, 0, s=145, marker='*', c='#152538', zorder=5)
    arrival.scatter(*returning[-1], s=70, facecolors='white', edgecolors='#16866c', linewidths=2, zorder=6)
    arrival.annotate('Returned', returning[-1], xytext=(-61, 16), textcoords='offset points',
                     fontsize=9, color='#16866c', arrowprops={'arrowstyle': '-', 'color': '#16866c', 'lw': .8})
    arrival.annotate('Home', (0, 0), xytext=(9, -14), textcoords='offset points', fontsize=9)
    arrival.set_xlim(-5, 3)
    arrival.set_ylim(-3, 4)

    autonomous_distance = sum(s['measured_xy_travel_m'] for s in metadata['stages']
                              if not s['name'].startswith('Bootstrap'))
    fig.text(.075, .115, f'Observed action travel: {autonomous_distance:.2f} m  |  '
             f'Return confirmed at {stages[-1].stop:.2f} s simulation time',
             fontsize=11, fontweight='bold')
    fig.text(.075, .069, 'Lines use native odometry samples only; no commanded path is drawn. '
             'Bootstrap is supervised and excluded from autonomous travel.', fontsize=8.6, color='#647185')
    fig.text(.075, .042, 'Colors span command publication to the confirmed endpoint source stamp. '
             'Gray background retains measured inter-action drift; return error is planar.', fontsize=8.6, color='#647185')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180, metadata={'Description': json.dumps(metadata, ensure_ascii=False)})
    plt.close(fig)
    print(json.dumps({'output': str(args.output), **metadata}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
