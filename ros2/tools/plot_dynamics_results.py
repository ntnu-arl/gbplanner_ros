#!/usr/bin/env python3
"""Plot recorded Bullet states without rerunning the simulator or planner."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from plot_local_results import grid
parser = argparse.ArgumentParser()
parser.add_argument('--results', type=Path, default=Path('results/dynamics'))
args = parser.parse_args()
root = args.results
report = json.loads((root / 'summary.json').read_text())
flight = np.genfromtxt(root / 'flight.csv', delimiter=',', names=True)
assert report['status'] == 'passed' and len(flight) > 1
position = np.column_stack([flight[k] for k in ('x', 'y', 'z')])
reference = np.column_stack([flight[k] for k in ('target_x', 'target_y', 'target_z')])
time = flight['time']
error = np.linalg.norm(position - reference, axis=1)
tilt = np.degrees(np.arccos(np.clip(1 - 2 * (flight['qx']**2 + flight['qy']**2), -1, 1)))
speed = np.sqrt(sum(flight[k]**2 for k in ('vx', 'vy', 'vz')))
active = flight['commands'] > 0
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10,
                     'axes.spines.top': False, 'axes.spines.right': False})
fig, axes = plt.subplots(2, 2, figsize=(13, 9), layout='constrained')
ax = axes[0, 0]
grid(ax, root / 'final_map.csv', 'Measured flight and timed position references')
ax.plot(reference[:, 0], reference[:, 1], '--', color='#ce8845', lw=1.4, label='Timed reference')
ax.plot(position[:, 0], position[:, 1], color='#176b9e', lw=1.2, label='Bullet position')
ax.scatter([-2], [0], marker='*', c='#24845a', s=110, label='Home', zorder=5)
moving = report.get('moving_continuations', [])
splice_times = []
for index, replacement in enumerate(moving):
    number = replacement['command']['command_number']
    matches = np.flatnonzero(flight['commands'] >= number)
    if len(matches):
        sample = matches[0];splice_times.append(float(time[sample]))
        ax.scatter(position[sample,0],position[sample,1],marker='D',s=32,color='#bd5733',zorder=6,
                   label='Moving continuation' if index==0 else None)
ax.legend(fontsize=8, loc='lower right')
ax = axes[0, 1]
ax.plot(time, error, color='#176b9e', lw=1)
ax.set(title='Distance to the current timed reference', xlabel='Simulated time (s)', ylabel='3D position error (m)')
ax.text(.02, .97, 'Includes along-track lag; not cross-track clearance', transform=ax.transAxes, va='top', fontsize=8)
for stamp in splice_times:ax.axvline(stamp,color='#bd5733',ls='--',lw=.6,alpha=.6)
ax = axes[1, 0]
ax.plot(time, tilt, color='#974a32', lw=1, label='Tilt from vertical')
ax.set(title='Rigid-body attitude and measured speed', xlabel='Simulated time (s)', ylabel='Tilt (degrees)')
right = ax.twinx()
right.plot(time, speed, color='#176b9e', lw=.8, alpha=.7)
right.set_ylabel('Measured speed (m/s)', color='#176b9e')
ax = axes[1, 1]
for index in range(4):
    ax.plot(time, flight['rotor' + str(index)], lw=.7, alpha=.8, label='Rotor ' + str(index))
ax.set(title='Motor response after lag and saturation', xlabel='Simulated time (s)', ylabel='Rotor angular speed (rad/s)')
ax.legend(fontsize=8, ncol=2)
for ax in axes.flat:
    ax.grid(alpha=.15)
fig.suptitle('Original RotorS Lee controller + Bullet rigid body + live lidar + Voxblox\n'
             f"{report['execution']['succeeded']} completed actions; {report['execution'].get('continuations',0)} continuations; "
             f"{report['physics']['distance_m']:.2f} m flight; zero contact steps")
fig.savefig(root / 'flight.png', dpi=180)
fig.savefig(root / 'flight.pdf')
plt.close(fig)
analysis = {
    'telemetry_samples': len(flight),
    'simulated_seconds': float(time[-1]),
    'flight_distance_m': report['physics']['distance_m'],
    'completed_actions': report['execution']['succeeded'],
    'trajectory_commands': report['execution']['trajectory_commands'],
    'continuations': report['execution'].get('continuations',0),
    'verified_moving_continuations':len(moving),
    'moving_continuation_simulated_times':splice_times,
    'home_error_at_mission_completion_m': report['home_error_m'],
    'distance_to_home_at_last_telemetry_m': float(np.linalg.norm(position[-1] - [-2, 0, 0])),
    'active_tracking_error_rmse_m': float(np.sqrt(np.mean(error[active]**2))),
    'active_tracking_error_p95_m': float(np.percentile(error[active], 95)),
    'tracking_error_maximum_physics_step_m': report['physics']['maximum_tracking_error_m'],
    'maximum_tilt_degrees_physics_step': float(np.degrees(report['physics']['maximum_tilt_rad'])),
    'maximum_measured_speed_mps': float(speed.max()),
    'thrust_newtons_range': [float(flight['thrust'].min()), float(flight['thrust'].max())],
    'rotor_speed_radps_range': [float(min(flight['rotor'+str(i)].min() for i in range(4))),
                              float(max(flight['rotor'+str(i)].max() for i in range(4)))],
    'contact_steps': int(flight['contacts'][-1]),
    'known_cells_initial': report['initial_known_cells'],
    'known_cells_final': report['final_known_cells'],
    'integrated_scans': report['mapper']['integrated_scans'],
    'notes': ['Tracking error is distance to the timed reference, not nearest-path distance.',
              'Mission completion and last telemetry are different observations.',
              'Known cells include occupied cells; this is not traversable coverage.']}
(root / 'analysis.json').write_text(json.dumps(analysis, indent=2) + '\n')
print(json.dumps(analysis, indent=2))
