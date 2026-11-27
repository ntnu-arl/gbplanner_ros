#!/usr/bin/env python3
"""Plot route choices on one graph after two different odometry histories."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path, default=Path('results/direction/summary.json'))
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    report = json.loads(args.input.read_text())
    if report['status'] != 'passed':
        raise ValueError('A completed direction experiment is required')
    prefix = args.output or args.input.parent / 'direction'
    prefix.parent.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10})
    fig, axes = plt.subplots(1, 2, figsize=(12.0, 5.5), layout='constrained')
    colors = ['#287fa4', '#d17537']
    vertices = {entry['id']: np.asarray(entry['position'])
                for entry in report['graph']['vertices']}
    cloud = np.asarray(list(vertices.values()))
    axis = axes[0]
    for source, target, _ in report['graph']['edges']:
        a, b = vertices[source], vertices[target]
        axis.plot([a[0], b[0]], [a[1], b[1]], color='#d5dae0', alpha=.45, lw=.5, zorder=1)
    axis.scatter(cloud[:, 0], cloud[:, 1], s=12, color='#9aa6b3', zorder=2,
                 label='Identical RRG vertices')
    origin = np.asarray(report['root'])
    for color, run in zip(colors, report['runs']):
        route = np.asarray(run['path'])
        label = 'History toward +x' if run['history'] == 'positive_x' else 'History toward −x'
        axis.plot(route[:, 0], route[:, 1], color=color, lw=2.6, marker='o',
                  markersize=4, zorder=4, label=label)
        heading = run['tracker']['heading']
        axis.arrow(origin[0], origin[1], 1.2 * np.cos(heading), 1.2 * np.sin(heading),
                   width=.025, head_width=.18, color=color, zorder=5,
                   length_includes_head=True)
    axis.arrow(origin[0], origin[1], .8 * np.cos(report['root_yaw']),
               .8 * np.sin(report['root_yaw']), width=.018, head_width=.15,
               color='#39495b', length_includes_head=True, zorder=5)
    axis.text(origin[0] + .08, origin[1] + .85, 'Same robot yaw: π/2', fontsize=8)
    axis.scatter(*origin[:2], s=55, color='#182f47', zorder=6)
    axis.set(title='Same map, root, yaw and sampled graph', xlabel='x (m)', ylabel='y (m)', aspect='equal')
    axis.grid(alpha=.18)
    axis.legend(loc='lower left', fontsize=8)

    axis = axes[1]
    histories = ['History toward +x', 'History toward −x']
    raw = [run['metrics']['unpenalized_gain'] for run in report['runs']]
    selected = [run['metrics']['gain'] for run in report['runs']]
    positions = np.arange(2)
    axis.bar(positions - .18, raw, width=.36, color='#bec7d1', label='Before direction penalty')
    axis.bar(positions + .18, selected, width=.36, color=colors, label='Selected score')
    for index, run in enumerate(report['runs']):
        distance = run['metrics']['direction_distance']
        factor = np.exp(-report['direction_weight'] * distance)
        axis.text(index, max(raw[index], selected[index]) + max(raw) * .025,
                  f'Distance = {distance:.3f}\nScore factor = {factor:.3f}',
                  ha='center', fontsize=9)
    axis.set(title='Selected path score includes the direction penalty',
             xticks=positions, xticklabels=histories, ylabel='Path information-gain score',
             ylim=(0, max(raw) * 1.25))
    axis.grid(axis='y', alpha=.2)
    axis.set_axisbelow(True)
    axis.legend(loc='upper right', fontsize=8)
    fig.suptitle(f"Odometry history changes route choice\n"
                 f"Direction weight {report['direction_weight']:g}; "
                 f"{len(vertices)} shared vertices; both paths pass ValidatePath", fontsize=13)
    fig.savefig(prefix.with_suffix('.png'), dpi=180)
    fig.savefig(prefix.with_suffix('.pdf'))
    plt.close(fig)
    print(prefix.with_suffix('.png'))


if __name__ == '__main__':
    main()
