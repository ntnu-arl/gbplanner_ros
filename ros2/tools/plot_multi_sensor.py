#!/usr/bin/env python3
"""Plot measured front/rear fusion and planning from multi_sensor_demo output."""
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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend', choices=('occupancy', 'voxblox'), default='occupancy')
    parser.add_argument('--input', type=Path)
    parser.add_argument('--output', type=Path, help='Figure prefix, without extension')
    args = parser.parse_args()
    source = args.input or Path('results/multi_sensor') / args.backend / 'summary.json'
    report = json.loads(source.read_text())
    if report['status'] != 'passed':
        raise ValueError('A completed multi-sensor run is required')
    prefix = args.output or source.parent / 'multi_sensor'
    prefix.parent.mkdir(parents=True, exist_ok=True)
    stages = report['stages']
    body = np.array(report['body_pose'][:3])
    route = np.asarray(report['path'])
    colors = ['#287ea6', '#d47737']
    cmap = ListedColormap(['#d7dbe0', '#ffffff', '#313a46'])
    norm = BoundaryNorm([-1.5, -0.5, 50, 100.5], cmap.N)
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 9})
    fig, axes = plt.subplots(2, 2, figsize=(11.8, 10.1), layout='constrained')

    for index, (axis, stage) in enumerate(zip(axes[0], stages)):
        values = np.genfromtxt(source.parent / stage['map_slice'], delimiter=',', names=True)
        xs, ys = np.unique(values['x']), np.unique(values['y'])
        grid = np.full((len(ys), len(xs)), -1.0)
        grid[np.searchsorted(ys, values['y']), np.searchsorted(xs, values['x'])] = values['status']
        axis.imshow(grid, origin='lower', extent=[xs[0] - .125, xs[-1] + .125,
                                               ys[0] - .125, ys[-1] + .125],
                    interpolation='nearest', cmap=cmap, norm=norm)
        axis.scatter(body[0], body[1], s=45, marker='o', facecolor='#354d69', zorder=5)
        for color, (name, sensor) in zip(colors, report['sensors'].items()):
            origin = body + np.array(sensor['offset'])
            yaw = sensor['yaw']
            axis.arrow(origin[0], origin[1], .9 * np.cos(yaw), .9 * np.sin(yaw),
                       width=.035, head_width=.22, length_includes_head=True,
                       color=color, alpha=1.0 if index == 1 or name == 'front' else .35)
        if index == 1:
            axis.plot(route[:, 0], route[:, 1], color='#b93b62', linewidth=2.1,
                      marker='o', markersize=2.8, zorder=4)
        axis.set(xlabel='x (m)', ylabel='y (m)', aspect='equal',
                 title=('Front lidar only' if index == 0 else 'Front + rear lidar, planned path')
                       + f"\n{stage['mapper']['known_cells']:,} observed cells; +{stage['newly_known_cells']:,} new")
        axis.set_xlim(-5, 5)
        axis.set_ylim(-5, 5)
    axes[0, 0].legend(handles=[Patch(facecolor='#d7dbe0', label='Unknown'),
                              Patch(facecolor='#ffffff', edgecolor='#b8bec7', label='Free'),
                              Patch(facecolor='#313a46', label='Occupied')],
                      loc='upper right', fontsize=8)
    axes[0, 1].legend(handles=[Line2D([0], [0], color=colors[0], lw=2, label='Front lidar'),
                              Line2D([0], [0], color=colors[1], lw=2, label='Rear lidar'),
                              Line2D([0], [0], color='#b93b62', lw=2, label='Selected path')],
                      loc='upper right', fontsize=8)

    axis = axes[1, 0]
    known = [stage['mapper']['known_cells'] for stage in stages]
    new = [stage['newly_known_cells'] for stage in stages]
    previous = [total - addition for total, addition in zip(known, new)]
    axis.bar([0, 1], previous, color='#c3cbd5', label='Previously observed')
    bars = axis.bar([0, 1], new, bottom=previous, color=colors, label='Newly observed')
    for bar, total, addition in zip(bars, known, new):
        axis.text(bar.get_x() + bar.get_width() / 2, total, f'{total:,}\n(+{addition:,})',
                  ha='center', va='bottom')
    axis.set(xticks=[0, 1], xticklabels=['Front scan', 'Rear scan'],
             ylabel='Observed 3D voxels', ylim=(0, max(known) * 1.2),
             title='Each independent scan expands the shared map')
    axis.grid(axis='y', alpha=.2)
    axis.set_axisbelow(True)
    axis.legend(fontsize=8)

    axis = axes[1, 1]
    positions = np.arange(len(stages))
    bottom = np.zeros(len(stages))
    for color, name in zip(colors, report['sensors']):
        gains = [next(s['gain'] for s in stage['sensor_gain']['sensors'] if s['name'] == name)
                 for stage in stages]
        axis.bar(positions, gains, bottom=bottom, color=color, label=name.capitalize())
        bottom += np.asarray(gains)
    for index, stage in enumerate(stages):
        frontier = stage['sensor_gain']['frontier']
        axis.text(index, bottom[index], f"{bottom[index]:.0f}\nfrontier: {str(frontier).lower()}",
                  ha='center', va='bottom')
    axis.set(xticks=positions, xticklabels=['After front', 'After both'],
             ylabel='Information gain at the robot pose',
             ylim=(0, max(1.0, max(bottom)) * 1.25),
             title='Sensor gains add; frontier flags combine with OR')
    axis.grid(axis='y', alpha=.2)
    axis.set_axisbelow(True)
    axis.legend(fontsize=8)
    fig.suptitle(f"Two-lidar mapping and native RRG planning — {report['backend']}\n"
                 f"One cloud per sensor; path length {report['planning']['path_length_m']:.2f} m; "
                 f"ValidatePath: {report['validation']['reason']}", fontsize=13)
    fig.savefig(prefix.with_suffix('.png'), dpi=180)
    fig.savefig(prefix.with_suffix('.pdf'))
    plt.close(fig)
    print(prefix.with_suffix('.png'))


if __name__ == '__main__':
    main()
