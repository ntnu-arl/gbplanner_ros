#!/usr/bin/env python3
"""Show measured adaptive boxes, observed surfaces and native graph paths."""
import argparse
import itertools
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import BoundaryNorm, ListedColormap
import numpy as np


def hull(points):
    points = sorted(set(tuple(point) for point in points))
    def cross(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    lower, upper = [], []
    for point in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    for point in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    return np.asarray(lower[:-1] + upper[:-1] + lower[:1])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path, default=Path('results/adaptive_sampling/summary.json'))
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    report = json.loads(args.input.read_text())
    if report['status'] != 'passed':
        raise ValueError('A completed adaptive sampling run is required')
    prefix = args.output or args.input.parent / 'adaptive_sampling'
    prefix.parent.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 9})
    fig = plt.figure(figsize=(15, 10), layout='constrained')
    cmap = ListedColormap(['#d7dee6', '#ffffff', '#465261'])
    norm = BoundaryNorm([-1.5, -0.5, 50, 100.5], cmap.N)
    resolution = report['grid']['resolution_m']
    for index, case in enumerate(report['cases']):
        top = fig.add_subplot(2, 3, index + 1, projection='3d')
        bottom = fig.add_subplot(2, 3, index + 4)
        surface = np.asarray(case['surface_points'])
        vertices = np.asarray([vertex['world'] for vertex in case['graph']['vertices']])
        box = np.asarray(case['graph']['sampling_box_world_corners'])
        route = np.asarray(case['path'])
        if len(surface):
            # The full surface is retained in JSON; subsample only its display.
            visible = surface[::max(1, len(surface) // 2500)]
            top.scatter(*visible.T, s=2, color='#8f9ba8', alpha=.2, rasterized=True)
        top.scatter(*vertices.T, s=12, color='#2c75a0', label='RRG vertices', depthshade=False)
        for a, b in itertools.combinations(range(8), 2):
            if bin(a ^ b).count('1') == 1:
                top.plot(*np.asarray([box[a], box[b]]).T, color='#328264', lw=1.8)
        if len(route):
            top.plot(*route.T, color='#c06c35', lw=2.4, marker='o', markersize=3, label='Selected path')
        top.scatter(*report['root'], color='#172f46', s=40)
        adaptive = case['metrics']['adaptive']
        if adaptive['applied']:
            name = f"Corridor {chr(ord('A') + index)}: PCA applied"
            subtitle = f"{adaptive['filtered_points']} filtered surface points; {len(vertices)} vertices"
        else:
            name = 'Free space: configured box restored'
            subtitle = f"{adaptive['raw_points']} surface points; {adaptive['reason']}"
        top.set(title=name + '\n' + subtitle, xlabel='x (m)', ylabel='y (m)', zlabel='z (m)')
        top.set_xlim(-6, 6)
        top.set_ylim(-6, 6)
        top.set_zlim(-2.4, 2.4)
        top.set_box_aspect((12, 12, 4.8))
        top.view_init(elev=26, azim=-55)
        if index == 0:
            top.legend(loc='upper left', fontsize=8)

        values = np.genfromtxt(args.input.parent / case['map_slice'], delimiter=',', names=True)
        xs, ys = np.unique(values['x']), np.unique(values['y'])
        grid = np.full((len(ys), len(xs)), -1.0)
        grid[np.searchsorted(ys, values['y']), np.searchsorted(xs, values['x'])] = values['status']
        bottom.imshow(grid, origin='lower', interpolation='nearest', cmap=cmap, norm=norm,
                      extent=[xs[0] - resolution / 2, xs[-1] + resolution / 2,
                              ys[0] - resolution / 2, ys[-1] + resolution / 2])
        outline = hull(box[:, :2])
        bottom.plot(outline[:, 0], outline[:, 1], color='#328264', lw=2, label='Sampling-box XY projection')
        bottom.scatter(vertices[:, 0], vertices[:, 1], s=10, color='#2c75a0', alpha=.8)
        if len(route):
            bottom.plot(route[:, 0], route[:, 1], color='#c06c35', lw=2.5, marker='o', markersize=3)
        bottom.scatter(*report['root'][:2], color='#172f46', s=35, zorder=5)
        dimensions = np.asarray(adaptive['maximum']) - np.asarray(adaptive['minimum'])
        bottom.set(title='Box side lengths: ' + ', '.join(f'{value:.2f}' for value in dimensions) + ' m',
                   xlabel='x (m)', ylabel='y (m)', aspect='equal')
        bottom.set_xlim(-6, 6)
        bottom.set_ylim(-6, 6)
        if index == 0:
            bottom.legend(loc='upper left', fontsize=8)
    fig.suptitle('Adaptive sampling follows observed 3D corridor geometry\n'
                 'Green: actual sampling box; blue: native graph; orange: validated route; gray-blue: unknown space',
                 fontsize=14)
    fig.savefig(prefix.with_suffix('.png'), dpi=180)
    fig.savefig(prefix.with_suffix('.pdf'))
    plt.close(fig)
    print(prefix.with_suffix('.png'))


if __name__ == '__main__':
    main()
