#!/usr/bin/env python3
"""Plot the measured ray/voxel gain experiment and its native selected path."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import BoundaryNorm, ListedColormap
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
import numpy as np


def stage_counts(case):
    backward = next(stage['diagnostics'] for stage in case['orientation_comparison']
                    if stage['heading'] == 'back')
    return [stage['sensors'][0]['unknown'] for stage in
            (case['forward'], backward, case['blocked_by_wall'], case['restored'])]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path,
                        default=Path('results/voxel_gain/summary.json'))
    parser.add_argument('--output', type=Path, help='Figure prefix without extension')
    args = parser.parse_args()
    report = json.loads(args.input.read_text())
    if report['status'] != 'passed':
        raise ValueError('A completed voxel-gain experiment is required')
    cases = {case['mode']: case for case in report['cases']}
    voxel = cases['voxel']
    planning = voxel['planning']
    prefix = args.output or args.input.parent / 'voxel_gain'
    prefix.parent.mkdir(parents=True, exist_ok=True)

    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 9,
                         'axes.titleweight': 'bold', 'axes.spines.top': False,
                         'axes.spines.right': False, 'pdf.fonttype': 42})
    fig = plt.figure(figsize=(13.2, 8.1), facecolor='white')
    grid = fig.add_gridspec(2, 2, width_ratios=(1.08, 1),
                           left=.065, right=.975, bottom=.25, top=.84,
                           wspace=.23, hspace=.66)
    map_axis = fig.add_subplot(grid[:, 0])
    ray_axis = fig.add_subplot(grid[0, 1])
    voxel_axis = fig.add_subplot(grid[1, 1])

    values = np.genfromtxt(args.input.parent / voxel['map_slice'],
                          delimiter=',', names=True)
    xs, ys = np.unique(values['x']), np.unique(values['y'])
    occupancy = np.full((len(ys), len(xs)), -1.)
    occupancy[np.searchsorted(ys, values['y']),
              np.searchsorted(xs, values['x'])] = values['status']
    half_cell = report['scene']['resolution_m'] / 2
    colors = {'unknown': '#dce2e8', 'free': '#ffffff', 'occupied': '#344250',
              'path': '#ad3c67', 'camera': '#d48724', 'root': '#274b67',
              'ray': '#367ca4', 'voxel': '#238a79'}
    cmap = ListedColormap([colors['unknown'], colors['free'], colors['occupied']])
    map_axis.imshow(occupancy, origin='lower', interpolation='nearest',
                    extent=[xs[0] - half_cell, xs[-1] + half_cell,
                            ys[0] - half_cell, ys[-1] + half_cell],
                    cmap=cmap, norm=BoundaryNorm([-1.5, -.5, 50, 100.5], 3))

    # Use the marker endpoints recorded from ROS, not a reconstructed FOV.
    # Several 3D edges share the same XY projection; draw each projection once.
    frustum = np.asarray(voxel['sensor_frustum']['points'])
    seen = set()
    for start, end in frustum.reshape(-1, 2, 3):
        if np.linalg.norm(end[:2] - start[:2]) < 1e-9:
            continue
        key = tuple(sorted((tuple(np.round(start[:2], 9)),
                            tuple(np.round(end[:2], 9)))))
        if key in seen:
            continue
        seen.add(key)
        map_axis.plot([start[0], end[0]], [start[1], end[1]],
                      color=colors['camera'], linewidth=1.5,
                      linestyle=(0, (4, 3)), alpha=.85, zorder=3)
    root = np.asarray(report['root'])
    route = np.asarray(planning['path'])
    map_axis.plot(route[:, 0], route[:, 1], color=colors['path'],
                  linewidth=2.5, marker='o', markersize=4, zorder=6)
    map_axis.scatter(root[0], root[1], color=colors['root'],
                     edgecolor='white', linewidth=.8, s=65, zorder=7)
    map_axis.scatter(route[-1, 0], route[-1, 1], color=colors['path'],
                     marker='D', edgecolor='white', linewidth=.6, s=45, zorder=7)
    map_axis.annotate('', xy=root[:2] + [1.25, 0], xytext=root[:2],
                      arrowprops={'arrowstyle': '-|>', 'color': colors['camera'],
                                  'lw': 1.8}, zorder=5)
    map_axis.annotate('Start', xy=root[:2], xytext=(-3.55, -.9),
                      color=colors['root'], fontsize=9,
                      arrowprops={'arrowstyle': '-', 'color': colors['root'], 'lw': .8})
    map_axis.annotate('Selected target', xy=route[-1, :2], xytext=(.6, -1.65),
                      color=colors['path'], fontsize=9,
                      arrowprops={'arrowstyle': '-', 'color': colors['path'], 'lw': .8})
    map_axis.text(3.45, 3.3, 'Unknown\nregion', color='#546371',
                   ha='center', va='center', fontsize=10)
    map_axis.set(xlabel='x (m)', ylabel='y (m)', aspect='equal',
                 xlim=(xs[0] - half_cell, xs[-1] + half_cell),
                 ylim=(ys[0] - half_cell, ys[-1] + half_cell),
                 title='XY map slice and selected voxel-mode path')
    map_axis.set_xticks(np.arange(-4, 5, 2))
    map_axis.set_yticks(np.arange(-4, 5, 2))
    map_axis.legend(handles=[
        Patch(facecolor=colors['unknown'], label='Unknown'),
        Patch(facecolor='white', edgecolor='#aeb8c1', label='Free'),
        Patch(facecolor=colors['occupied'], label='Occupied'),
        Line2D([0], [0], color=colors['path'], lw=2, label='Selected path'),
        Line2D([0], [0], color=colors['camera'], lw=1.5,
               linestyle='--', label='Frustum (XY)'),
        Line2D([0], [0], marker='o', color=colors['root'], lw=0, label='Robot start')],
        loc='upper center', bbox_to_anchor=(.5, -.13), ncol=3,
        frameon=False, fontsize=8, columnspacing=1.3, handlelength=1.6)

    stage_labels = ['Forward\nopen gap', 'Backward\nopen gap',
                    'Forward\nfull wall', 'Forward\ngap restored']
    for mode, axis in [('ray', ray_axis), ('voxel', voxel_axis)]:
        case = cases[mode]
        counts = stage_counts(case)
        sensor = case['forward']['sensors'][0]
        bars = axis.bar(np.arange(4), counts, width=.56,
                        color=colors[mode], zorder=3)
        axis.bar_label(bars, labels=[f'{value:,}' for value in counts], padding=5,
                       color=colors[mode], fontsize=10, weight='bold')
        axis.set(xticks=np.arange(4), xticklabels=stage_labels,
                 ylabel=('Unknown ray hits' if mode == 'ray' else 'Visible unknown samples'),
                 ylim=(0, max(counts) * 1.28))
        if mode == 'ray':
            title = f"Ray model: {sensor['rays']:,} rays per viewpoint"
        else:
            title = (f"Voxel model: {sensor['voxel_count']:,} candidates; "
                     f"{sensor['fov_voxel_count']:,} inside forward FOV")
        axis.set_title(title, fontsize=10, loc='left', pad=12)
        axis.grid(axis='y', color='#d9dfe5', linewidth=.7, zorder=0)
        axis.set_axisbelow(True)
        axis.tick_params(axis='x', length=0)

    metrics = planning['metrics']
    fig.suptitle('Native ROS 2 exploration gain: ray model and per-voxel model',
                 fontsize=16, weight='bold', y=.97)
    fig.text(.5, .918,
             f"Same 3D occupancy map | 0.25 m resolution | "
             f"Selected path: {metrics['path_length_m']:.2f} m | "
             f"ValidatePath: {planning['validation']['reason']}",
             ha='center', color='#526170', fontsize=10)
    fig.text(.065, .105,
             'Count definitions and panel scales differ; bar height is not an accuracy ranking.\n'
             'Ray hits may revisit a voxel; voxel counts refer to sampled positions in the local region.',
             ha='left', va='top', fontsize=9, color='#526170', linespacing=1.6)
    fig.text(.065, .025,
             'The dashed frustum is the published viewing geometry projected onto XY; walls occlude gain queries.',
             ha='left', va='bottom', fontsize=8.5, color='#526170')
    fig.savefig(prefix.with_suffix('.png'), dpi=180)
    fig.savefig(prefix.with_suffix('.pdf'))
    plt.close(fig)
    print(prefix.with_suffix('.png'))


if __name__ == '__main__':
    main()
