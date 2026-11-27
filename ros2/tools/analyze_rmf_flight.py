#!/usr/bin/env python3
"""Plot recorded native RMF flight; does not launch simulation."""
import argparse
from bisect import bisect_right
import json
import math
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('result', type=Path)
    args = parser.parse_args()
    path = args.result.resolve()
    result = json.loads(path.read_text())
    history = json.loads(path.with_name(Path(result['history_file']).name).read_text())
    odometry = history['odometry']
    rows = {}
    for phase in dict.fromkeys(r['phase'] for r in odometry):
        samples = [r for r in odometry if r['phase'] == phase]
        segments = [(a,b) for a,b in zip(odometry, odometry[1:]) if b['phase'] == phase]
        rows[phase] = {'samples': len(samples),
            'source_start_s': samples[0]['stamp_ns']*1.e-9,
            'source_end_s': samples[-1]['stamp_ns']*1.e-9,
            'distance_xyz_m': sum(math.dist(a['position'], b['position']) for a,b in segments),
            'end_position': samples[-1]['position']}
    review = {'run_status': result['status'], 'run_error': result.get('error'),
              'source': path.name, 'phases': rows,
              'autonomous_distance_m': rows.get('automatic_exploration', {}).get('distance_xyz_m', 0.),
              'scope': 'Measured native odometry; direct initialization/descent is separate from autonomous motion',
              'contact_observations': result['contact_observations_by_phase'],
              'endpoint_checks': {k:{key:v[key] for key in ('position_error_m','yaw_error_rad','elapsed_sim_s')}
                                  for k,v in result['phases'].items()},
              'latest_controller': result.get('last_controller')}
    path.with_name(path.stem+'_analysis.json').write_text(json.dumps(review, indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    plt.rcParams.update({'font.size': 9, 'axes.spines.top': False, 'axes.spines.right': False})
    fig = plt.figure(figsize=(12,8), layout='constrained')
    grid = fig.add_gridspec(2,2)
    ax = fig.add_subplot(grid[:,0], projection='3d')
    palette = plt.get_cmap('tab10')
    dispatches = [e for e in history.get('control', []) if e['state']=='SENDING']
    times = [e['wall_s'] for e in dispatches]
    labels = []
    for sample in odometry:
        if sample['phase']=='automatic_exploration':
            index = bisect_right(times,sample['wall_s'])-1
            label = ('Autonomous return' if dispatches[index]['mode']=='homing' else f'Local {index+1}') if index>=0 else 'Planning / hover'
        else:
            label = 'Supervised flight / landing'
        labels.append(label)
    for index,label in enumerate(dict.fromkeys(labels)):
        samples = np.array([r['position'] if name==label else [float('nan')]*3
                            for r,name in zip(odometry,labels)])
        color = '#94a3b8' if label=='Supervised flight / landing' else palette(index%10)
        ax.plot(*samples.T, color=color, label=label, lw=1.7)
    ax.set(xlabel='World X (m)', ylabel='World Y (m)', zlabel='World Z (m)', title='Measured 3-D motion')
    ax.legend(fontsize=8, loc='upper left')
    height = fig.add_subplot(grid[0,1])
    t = np.array([r['stamp_ns']*1.e-9 for r in odometry])
    height.plot(t, [r['position'][2] for r in odometry], color='#0369a1', label='Native height')
    for i,trajectory in enumerate(history['trajectories']):
        u = np.linspace(0,1,101)
        blend = 10*u**3-15*u**4+6*u**5
        height.plot(trajectory['origin_sim_s']+u*trajectory['duration_s'],
            trajectory['start'][2]+blend*(trajectory['target'][2]-trajectory['start'][2]),
            '--', color='#e17c05', label='Supervised target' if i==0 else None)
    height.set(xlabel='Source simulation time (s)', ylabel='Z (m)', title='Height and supervised command')
    height.grid(alpha=.2);height.legend()
    motors = fig.add_subplot(grid[1,1])
    for index in range(4):
        samples = [r for r in history['joints'] if len(r['velocity']) == 4]
        motors.plot([r['stamp_ns']*1.e-9 for r in samples],
                    [10*abs(r['velocity'][index]) for r in samples], lw=.7, label=f'Rotor {index}')
    motors.set(xlabel='Source simulation time (s)', ylabel='Equivalent motor speed (rad/s)',
               title='Native joint rate × original slowdown 10')
    motors.grid(alpha=.2);motors.legend(ncol=4,fontsize=8)
    suffix = ' — '+result['error'].split(':',1)[-1][:85] if result.get('error') else ''
    fig.suptitle(f'RMF / original Lee / Gazebo Fortress — {path.stem}: {result["status"]}{suffix}', fontsize=11)
    for extension in ('png','svg'):
        fig.savefig(path.with_name(path.stem+'_flight.'+extension), dpi=170)
    plt.close(fig)
    print(json.dumps({'analysis':str(path.with_name(path.stem+'_analysis.json')),
                      'phases':rows,'autonomous_distance_m':review['autonomous_distance_m']},indent=2))


if __name__ == '__main__':
    main()
