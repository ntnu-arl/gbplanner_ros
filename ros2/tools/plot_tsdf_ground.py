#!/usr/bin/env python3
"""Show fused TSDF terrain, selected ground path and native homing profile."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np


def grid_values(rows, x_name, y_name, value_name):
    xs, ys = np.unique(rows[x_name]), np.unique(rows[y_name])
    values = np.full((len(ys), len(xs)), np.nan)
    values[np.searchsorted(ys, rows[y_name]), np.searchsorted(xs, rows[x_name])] = rows[value_name]
    return xs, ys, values


def route_profile(axis, rows, title, color):
    points = np.column_stack([rows[name] for name in ('x', 'y', 'base_z')])
    distance = np.r_[0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    axis.fill_between(distance, rows['input_surface_z'],
                      min(rows['input_surface_z']) - .12, color='#e6ddd0', alpha=.7)
    axis.plot(distance, rows['input_surface_z'], color='#9b7548', linewidth=1.7,
               label='Input surface')
    axis.plot(distance, rows['tsdf_surface_z'], color='#247b9b', linewidth=1.6,
               linestyle='--', label='Fused TSDF surface')
    axis.plot(distance, rows['projected_surface_z'], color=color,
               marker='o', markersize=3, linewidth=1.2, label='Planner ground projection')
    axis.plot(distance, rows['base_z'], color=color, linewidth=2,
               label='Robot base path')
    axis.set(xlabel='Distance along route (m)', ylabel='Height z (m)', title=title)
    axis.set_xlim(0, distance[-1])
    axis.set_ylim(min(rows['projected_surface_z']) - .06, max(rows['base_z']) + .25)
    axis.grid(alpha=.2)
    axis.legend(fontsize=7.5, loc='upper left', ncol=2, framealpha=.94)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path, default=Path('results/tsdf_ground/summary.json'))
    parser.add_argument('--output', type=Path, help='Figure prefix without extension')
    args = parser.parse_args()
    report = json.loads(args.input.read_text())
    if report['status'] != 'passed':
        raise ValueError('A completed TSDF ground experiment is required')
    directory = args.input.parent
    prefix = args.output or directory / 'tsdf_ground'
    prefix.parent.mkdir(parents=True, exist_ok=True)
    surface = np.genfromtxt(directory / report['map']['surface'], delimiter=',', names=True)
    snapshot = np.genfromtxt(directory / report['map']['snapshot'], delimiter=',', names=True)
    local = np.genfromtxt(directory / 'local_path.csv', delimiter=',', names=True)
    home = np.genfromtxt(directory / 'homing_path.csv', delimiter=',', names=True)
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 9,
                         'axes.titleweight': 'bold', 'axes.spines.top': False,
                         'axes.spines.right': False, 'pdf.fonttype': 42})
    fig, axes = plt.subplots(2, 2, figsize=(13, 10.5),
                             gridspec_kw={'width_ratios': [1, 1.25]})
    fig.subplots_adjust(left=.075, right=.98, bottom=.145, top=.87,
                        hspace=.35, wspace=.28)
    local_color, home_color = '#ad3c67', '#248375'
    resolution = report['scene']['resolution_m']

    xs, ys, heights = grid_values(surface, 'x', 'y', 'tsdf_surface_z')
    axis = axes[0, 0]
    image = axis.imshow(heights, origin='lower', interpolation='nearest', cmap='terrain',
                        extent=[xs[0] - resolution / 2, xs[-1] + resolution / 2,
                                ys[0] - resolution / 2, ys[-1] + resolution / 2])
    axis.plot(local['x'], local['y'], color=local_color, lw=2.5, marker='o', markersize=3)
    axis.plot(home['x'], home['y'], color=home_color, lw=1.5, linestyle='--')
    axis.scatter(local['x'][0], local['y'][0], s=55, color='#253c56',
                  edgecolor='white', linewidth=.8, zorder=6)
    axis.scatter(local['x'][-1], local['y'][-1], s=45, color=local_color,
                  marker='D', edgecolor='white', linewidth=.8, zorder=6)
    axis.set(xlabel='x (m)', ylabel='y (m)', aspect='equal',
              title='Observed surface and actual selected routes')
    axis.legend(handles=[Line2D([0], [0], color=local_color, lw=2, label='Local path'),
                          Line2D([0], [0], color=home_color, lw=1.5,
                                 linestyle='--', label='Homing path'),
                          Line2D([0], [0], marker='o', color='#253c56', lw=0, label='Home')],
                  loc='upper left', fontsize=8)
    fig.colorbar(image, ax=axis, fraction=.045, pad=.03, label='TSDF zero crossing z (m)')

    for axis, rows, key, color in [(axes[0, 1], local, 'local', local_color),
                                  (axes[1, 1], home, 'homing', home_color)]:
        checks = report[key]['terrain_checks']
        route_profile(axis, rows,
                      f"{key.capitalize()} route: {checks['path_length_m']:.2f} m; "
                      f"height change {checks['height_span_m']:.3f} m", color)

    axis = axes[1, 0]
    # Show measured distances in the nearest observed vertical slice. Unknown
    # cells remain blank; the figure does not manufacture values below ground.
    observed = snapshot[snapshot['tsdf_weight'] > 0]
    y = np.unique(observed['y'])[np.argmin(np.abs(np.unique(observed['y'])))]
    section = observed[np.isclose(observed['y'], y)]
    xs, zs, distances = grid_values(section, 'x', 'z', 'tsdf_distance')
    limit = max(abs(np.nanmin(distances)), abs(np.nanmax(distances)))
    image = axis.imshow(distances, origin='lower', interpolation='nearest', cmap='RdBu',
                        vmin=-limit, vmax=limit,
                        extent=[xs[0] - resolution / 2, xs[-1] + resolution / 2,
                                zs[0] - resolution / 2, zs[-1] + resolution / 2], aspect='auto')
    truth_x = np.linspace(xs.min(), xs.max(), 200)
    axis.plot(truth_x, .13 + .12 * np.maximum(0, truth_x + .6),
               color='#222222', lw=1.4, label='Input ground')
    axis.plot(local['x'], local['base_z'], color=local_color, lw=2.2,
               marker='o', markersize=3, label='Local base path (XZ)')
    axis.set(xlabel='x (m)', ylabel='z (m)',
              title=f'Original Voxblox distances at y = {y:.2f} m')
    axis.legend(loc='upper left', fontsize=8)
    fig.colorbar(image, ax=axis, fraction=.045, pad=.03, label='Signed distance (m)')

    fig.suptitle('Native ROS 2 terrain planning from fused TSDF observations',
                  fontsize=16, weight='bold', y=.965)
    fig.text(.5, .925,
              f"{len(report['scans'])} simulated surface scans | "
              f"{report['map']['status']['tsdf_cells']:,} observed TSDF cells | "
              f"{resolution:.2f} m voxels | Local + homing ValidatePath: ok",
              ha='center', color='#526170', fontsize=10)
    fig.text(.075, .045,
              'Input returns describe a flat surface joined to a gentle ramp. '
              'All plotted distance values come from the original Voxblox integrator.\n'
              'Ground projection is reconstructed from the returned base path; '
              f"the configured base clearance is {report['base_height_m']:.2f} m.",
              ha='left', color='#526170', fontsize=9, linespacing=1.6)
    fig.savefig(prefix.with_suffix('.png'), dpi=180)
    fig.savefig(prefix.with_suffix('.pdf'))
    plt.close(fig)
    print(prefix.with_suffix('.png'))


if __name__ == '__main__':
    main()
