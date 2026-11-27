#!/usr/bin/env python3
"""Figures from recorded ROS 2 closed-loop runs; no synthetic measurements."""
import json
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from plot_local_results import grid, rows


def main():
    root = Path(__file__).resolve().parents[1] / 'results'
    directory = root / 'system_experiments'
    summary = json.loads((directory / 'summary.json').read_text())
    assert summary['status'] == 'passed'
    colors = {'local': '#be6428', 'global': '#26794b', 'homing': '#166caa'}
    names = ['seed0', 'seed21', 'seed42']
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10,
                         'axes.spines.top': False, 'axes.spines.right': False})
    fig, axes = plt.subplots(1, 3, figsize=(14, 5.5), layout='constrained')
    for axis, name in zip(axes, names):
        data = json.loads((directory / name / 'summary.json').read_text())
        grid(axis, directory / name / 'final_map.csv',
             f"Seed {data['seed']}: {data['distance_m']:.2f} m total")
        for action in data['actions']:
            path = np.array(action['path'])
            axis.plot(path[:, 0], path[:, 1], '--' if action['mode'] == 'homing' else '-',
                      color=colors[action['mode']], linewidth=2, alpha=.9)
        first = data['actions'][0]['path'][0]
        axis.scatter(first[0], first[1], s=105, marker='*', c='#26794b', zorder=5)
        axis.text(.03, .03, f"Known cells: {data['final_known_cells']:,} / {data['total_cells']:,}\n"
                  f"{len(data['actions'])} successful actions; returned home",
                  transform=axis.transAxes, fontsize=9,
                  bbox=dict(facecolor='white', edgecolor='none', alpha=.9))
    handles = [Patch(color='#ced5dd', label='Unknown'), Patch(color='#303b49', label='Occupied'),
               plt.Line2D([], [], color=colors['local'], label='Local first-edge commands'),
               plt.Line2D([], [], color=colors['homing'], linestyle='--', label='Homing commands'),
               plt.Line2D([], [], color='#26794b', marker='*', linestyle='', label='Home')]
    fig.legend(handles=handles, loc='outside lower center', ncol=5, frameon=False, fontsize=9)
    fig.suptitle('Persistent ROS 2 controller + moving lidar + test kinematic executor\n'
                 '3D command paths projected onto XY; background map slice z = 0.125 m')
    fig.savefig(directory / 'paths.png', dpi=180)
    fig.savefig(directory / 'paths.pdf')
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), layout='constrained')
    run_colors = ['#166caa', '#be6428', '#398258']
    for name, color in zip(names, run_colors):
        history = rows(directory / name / 'history.csv')
        axes[0].plot([float(r['distance_m']) for r in history],
                     [100*float(r['known_fraction']) for r in history], color=color,
                     label=name.replace('seed', 'Seed '), linewidth=1.8)
    axes[0].set(xlabel='Cumulative 3D travel (m)', ylabel='Known / all bounded cells (%)',
                ylim=(50, 100), title='Observation grows while moving')
    axes[0].legend(); axes[0].grid(alpha=.2)
    records = summary['runs']
    x = np.arange(len(records))
    a = np.array([r['local_m'] for r in records])
    b = np.array([r['homing_m'] for r in records])
    assert all(r['global_m'] == 0 for r in records), 'Add global segments to plot before including these results'
    axes[1].bar(x, a, color=colors['local'], label='Local')
    axes[1].bar(x, b, bottom=a, color=colors['homing'], label='Homing')
    for i, r in enumerate(records):
        assert abs(a[i]+b[i]-r['total_m']) < 1e-8
        axes[1].text(i, a[i]+b[i]+.25, f"{r['total_m']:.2f} m", ha='center', fontsize=9)
    axes[1].set(xticks=x, xticklabels=['Seed 0', 'Seed 21', 'Seed 42', '42 repeat'],
                ylabel='3D travel distance (m)', ylim=(0, max(a+b)*1.18),
                title='Separate fresh processes; seed 42 repeated')
    axes[1].legend(loc='upper right'); axes[1].grid(axis='y', alpha=.2)
    fig.suptitle('Three seeds plus one repeat; all runs return home\n'
                 'Known cells include obstacles; this is not reachable-free-space coverage')
    fig.savefig(directory / 'comparison.png', dpi=180)
    fig.savefig(directory / 'comparison.pdf')
    plt.close(fig)
    repeat = summary['seed42_repeat']
    assert repeat['same_action_paths'] and repeat['same_known_cells'] and repeat['distance_difference_m'] < 1e-9
    analysis = {'status': 'passed', 'runs': len(records), 'unique_seeds': 3,
                'known_fraction_percent': {r['run']: 100*r['known_cells']/19200 for r in records},
                'home_errors_m': {r['run']: r['home_error_m'] for r in records},
                'seed42_repeat': repeat,
                'scope': 'Synthetic lidar and kinematic execution; not real-robot accuracy or dynamics'}
    (directory / 'analysis.json').write_text(json.dumps(analysis, indent=2) + '\n')
    print(json.dumps(analysis, indent=2))


if __name__ == '__main__':
    main()
