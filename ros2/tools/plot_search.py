#!/usr/bin/env python3
"""Plot the native PlannerSearch routes, endpoint fallback, and blocked case."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path, default=Path('results/search/summary.json'))
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    report = json.loads(args.input.read_text())
    if report['status'] != 'passed':
        raise ValueError('A completed search experiment is required')
    prefix = args.output or args.input.parent / 'search'
    prefix.parent.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10})
    fig, axes = plt.subplots(2, 2, figsize=(12, 10), layout='constrained')
    titles = {'direct': 'Direct connection from current robot state',
              'doorway': 'RRG search through an offset doorway',
              'target_neighborhood': 'Occupied target: reachable neighborhood',
              'sealed': 'Sealed wall: no reachable target neighborhood'}
    for axis, run in zip(axes.flat, report['runs']):
        metric = run['metrics']
        if run['map'] != 'open':
            spans = [(-5, 1.0), (2.5, 5)] if run['map'] == 'doorway' else [(-5, 5)]
            for lower, upper in spans:
                axis.add_patch(Rectangle((-.25, lower), .5, upper - lower,
                                         facecolor='#485769', edgecolor='none', zorder=2))
        vertices = {v['id']: np.asarray(v['position']) for v in run['graph']['vertices']}
        for source, target, _ in run['graph']['edges']:
            a, b = vertices[source], vertices[target]
            axis.plot([a[0], b[0]], [a[1], b[1]], color='#c4d3dc', lw=.45, alpha=.5, zorder=1)
        cloud = np.asarray(list(vertices.values()))
        if len(cloud):
            axis.scatter(cloud[:, 0], cloud[:, 1], s=5, color='#8fa8b7', alpha=.65, zorder=3)
        target = run['requested_target']
        if not metric['exact_target']:
            axis.add_patch(Circle(target[:2], report['target_radius'], edgecolor='#b25c36',
                                  facecolor='#f8d6bd', alpha=.35, linestyle='--', zorder=3))
        path = np.asarray(run['path'])
        if len(path):
            axis.plot(path[:, 0], path[:, 1], color='#177a86', linewidth=2.6,
                      marker='o', markersize=3.5, label='Validated route', zorder=5)
            if not metric['exact_target']:
                axis.scatter(*path[-1, :2], marker='s', s=65, facecolor='#e3a653',
                             edgecolor='#473c31', zorder=7, label='Reached endpoint')
        axis.scatter(*run['source'][:2], s=65, facecolor='#19495c', edgecolor='white',
                     zorder=7, label='Source')
        axis.scatter(*target[:2], marker='*', s=165, color='#be653d',
                     edgecolor='white', linewidth=.6, zorder=7, label='Requested target')
        detail = (f"{metric['path_length_m']:.2f} m route; {metric['endpoint_distance']:.2f} m endpoint error"
                  if metric['success'] else 'No path returned')
        axis.set_title(titles[run['name']] + '\n' + detail, fontsize=11)
        axis.text(.02, .02, f"{metric['vertices']} vertices · {metric['attempts']} attempts",
                  transform=axis.transAxes, fontsize=8,
                  bbox={'facecolor': 'white', 'edgecolor': 'none', 'alpha': .8})
        axis.set(xlim=(-4.5, 4), ylim=(-4, 4), aspect='equal', xlabel='x (m)', ylabel='y (m)')
        axis.grid(alpha=.17)
        axis.legend(loc='lower right', fontsize=7)
    fig.suptitle('Arbitrary-pose search through the native ROS 2 service\n'
                 'One planner process; synthetic occupancy maps; all successful routes pass ValidatePath', fontsize=13)
    fig.savefig(prefix.with_suffix('.png'), dpi=180)
    fig.savefig(prefix.with_suffix('.pdf'))
    plt.close(fig)
    print(prefix.with_suffix('.png'))


if __name__ == '__main__':
    main()
