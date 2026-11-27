#!/usr/bin/env python3
"""Plot native ROS 2 terrain routes and observed support from the actual run."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def profile(route):
    xyz = np.asarray(route, dtype=float)[:, :3]
    distance = np.r_[0., np.cumsum(np.linalg.norm(np.diff(xyz, axis=0), axis=1))]
    return xyz, distance


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path, default=Path('results/ground_planning/summary.json'))
    args = parser.parse_args()
    report = json.loads(args.input.read_text())
    if report['status'] != 'passed':
        raise ValueError('A completed native planning run is required')
    surface = np.asarray(report['scene']['surface_columns'], dtype=float)
    valid = surface[:, 3] == 0
    ground = surface[valid]
    local, s = profile(report['local']['path'])
    home, _ = profile(report['homing']['path'])
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10})
    fig = plt.figure(figsize=(13, 5.8), layout='constrained')
    ax = fig.add_subplot(121, projection='3d')
    # Triangulate only observed columns. Unknown support remains empty.
    ax.plot_trisurf(ground[:, 0], ground[:, 1], ground[:, 2], cmap='terrain',
                    alpha=.42, linewidth=0, antialiased=True, rasterized=True)
    ax.plot(*local.T, color='#116a9e', linewidth=3, label='Local exploration (base_link)')
    ax.plot(*home.T, color='#d16b29', linewidth=1.8, linestyle='--', label='Global homing (base_link)')
    ax.scatter(*local[0], color='#21394c', s=36, label='Home')
    ax.set(xlabel='x (m)', ylabel='y (m)', zlabel='z (m)', title='Observed 3D terrain and native planner output')
    ax.view_init(elev=27, azim=-65)
    ax.set_box_aspect((1.35, 1, .7))
    ax.legend(loc='upper left', fontsize=8)
    ax = fig.add_subplot(122)
    # Recover the occupied-cell top under each returned base pose from the
    # same published columns, rather than the analytic generating surface.
    trace = report.get('executed_trace')
    if trace:
        s = np.asarray([row['time_s'] for row in trace])
        local = np.asarray([row['state'][:3] for row in trace])
        support = [row['surface_z'] for row in trace]
    else:
        support = []
        for point in local:
            nearest = np.argmin(np.sum((ground[:, :2] - point[:2]) ** 2, axis=1))
            support.append(ground[nearest, 2])
    ax.fill_between(s, np.min(ground[:, 2]) - .1, support, color='#becdaf', alpha=.6)
    ax.step(s, support, color='#6d845a', where='mid', linewidth=1.3, label='Observed voxel surface')
    ax.plot(s, local[:, 2], color='#116a9e', marker='.' if not trace else None,
            markersize=5, label='Observed base height' if trace else 'Returned base height')
    if trace:
        turn = next(row['time_s'] for row in trace if row['phase'] == 'homing')
        ax.axvline(turn, color='#d16b29', linestyle='--', linewidth=1.2, label='Start homing action')
    ax.set(xlabel='Elapsed wall time (s)' if trace else 'Distance along exploration path (m)',
           ylabel='World height (m)', title='Executed height follows terrain' if trace else 'Returned route follows changing ground height')
    ax.grid(alpha=.2)
    ax.legend(fontsize=9, loc='lower left')
    title = ('Native PathFollower: two successful ground actions\nTimed kinematic trajectory consumer'
             if trace else 'Ground planning in ROS 2: local exploration and global homing')
    fig.suptitle(title, fontsize=14)
    prefix = args.input.parent / ('terrain_execution' if trace else 'terrain_routes')
    fig.savefig(prefix.with_suffix('.png'), dpi=180)
    fig.savefig(prefix.with_suffix('.pdf'))
    plt.close(fig)
    print(prefix.with_suffix('.png'))


if __name__ == '__main__':
    main()
