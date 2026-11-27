#!/usr/bin/env python3
"""Plot recorded SMB motion and terminal errors; never runs a simulation."""
import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('result', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = args.result.resolve()
    data = json.loads(result.read_text())
    records = list(csv.DictReader(result.with_suffix('.csv').open()))
    output = args.output or result.with_name(result.stem + '_motion.png')
    output.parent.mkdir(parents=True, exist_ok=True)
    checks = data['checks']
    phases = [('local_gain_exploration', 'Gain-selected step', '#2166ac'),
              ('autonomous_homing', 'Return home', '#d97706')]

    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False,
                         'axes.spines.right': False, 'savefig.facecolor': 'white'})
    figure = plt.figure(figsize=(11, 6), layout='constrained')
    grid = figure.add_gridspec(2, 2, width_ratios=(1.2, 1))
    route_axis = figure.add_subplot(grid[:, 0])
    distance_axis = figure.add_subplot(grid[0, 1])
    yaw_axis = figure.add_subplot(grid[1, 1])

    for name, label, color in phases:
        check = checks[name]
        path = check['commanded_path']['path']
        rows = [row for row in records if row['phase'] == name]
        if not rows:
            raise ValueError('No actual odometry for ' + name)
        x, y = [[float(row[key]) for row in rows] for key in ('x_m', 'y_m')]
        route_axis.plot(x, y, color=color, linewidth=2, label=label + ' (actual)')
        route_axis.plot([point[0] for point in path], [point[1] for point in path],
                        '--', color=color, alpha=.5, linewidth=1.3)
        route_axis.scatter(x[-1], y[-1], color=color, s=30, zorder=4)
        relative_time = [float(row['time_s']) - float(rows[0]['time_s']) for row in rows]
        endpoint = path[-1]
        distance_axis.plot(relative_time, [math.hypot(a-endpoint[0], b-endpoint[1])
                                           for a, b in zip(x, y)], color=color, label=label)
        yaw_axis.plot(relative_time, [abs(math.atan2(
            math.sin(float(row['yaw_rad'])-endpoint[3]),
            math.cos(float(row['yaw_rad'])-endpoint[3]))) for row in rows], color=color)

    home = checks['graph_home']['base_pose']
    route_axis.scatter(home[0], home[1], marker='*', s=160, color='#334155',
                       label='Stored home XY', zorder=5)
    route_axis.set(xlabel='Map X (m)', ylabel='Map Y (m)',
                   title='Actual wheel-contact motion; dashed = commanded path')
    route_axis.set_aspect('equal', adjustable='datalim')
    route_axis.grid(alpha=.2)
    route_axis.legend(loc='best', frameon=False, fontsize=9)
    distance_axis.axhline(.04, color='#475569', linestyle=':', label='0.04 m tolerance')
    distance_axis.set(ylabel='Endpoint XY error (m)', title='Error to each commanded endpoint')
    distance_axis.legend(frameon=False, fontsize=8)
    yaw_axis.axhline(.1, color='#475569', linestyle=':', label='0.1 rad tolerance')
    yaw_axis.set(xlabel='Seconds since this phase started', ylabel='Endpoint yaw error (rad)')
    yaw_axis.legend(frameon=False, fontsize=8)
    for axis in (distance_axis, yaw_axis):
        axis.grid(alpha=.2)
    figure.suptitle('SMB: one gain-selected step followed by return home', fontsize=14)
    figure.savefig(output, dpi=200)
    figure.savefig(output.with_suffix('.svg'))
    plt.close(figure)
    print(output.resolve())


if __name__ == '__main__':
    main()
