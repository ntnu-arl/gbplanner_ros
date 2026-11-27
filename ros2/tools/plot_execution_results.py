#!/usr/bin/env python3
"""Read actual MultiDOF commands and motion observations; never rerun planner."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from plot_local_results import grid, rows

parser=argparse.ArgumentParser()
parser.add_argument('--results',type=Path,default=Path('results/clean_image/results/execution_system'))
args=parser.parse_args();directory=args.results
data=json.loads((directory/'summary.json').read_text());assert data['status']=='passed'
history=rows(directory/'history.csv')
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.spines.top':False,'axes.spines.right':False})
fig,axes=plt.subplots(2,2,figsize=(12,9),layout='constrained')
grid(axes[0,0],directory/'final_map.csv','Actual motion and timed command paths')
colors={'local':'#b86227','global':'#2e854d','homing':'#176b9e'}
labelled_modes=set()
for command in data['commands']:
    label=command['mode'].capitalize() if command['mode'] not in labelled_modes else None
    labelled_modes.add(command['mode'])
    points=np.array(command['points']);axes[0,0].plot(points[:,0],points[:,1],
        '--' if command['mode']=='homing' else '-',color=colors[command['mode']],lw=1.8,label=label)
axes[0,0].plot([float(r['x']) for r in history],[float(r['y']) for r in history],'.',c='#80398d',ms=1.8,label='Observed poses')
axes[0,0].scatter([-2],[0],marker='*',c='#2e854d',s=100,label='Home');axes[0,0].legend(fontsize=8)
axes[0,1].plot([float(r['distance_m']) for r in history],[100*float(r['known_fraction']) for r in history],color='#176b9e')
axes[0,1].set(title='Observed cells during execution',xlabel='Cumulative 3D motion (m)',ylabel='Known / all bounded cells (%)',ylim=(50,100))
offset=0.;max_speed=max_yaw=0.
for index,command in enumerate(data['commands']):
    points=np.array(command['points']);times=np.array(command['times'])
    delta=np.diff(times);linear=np.linalg.norm(np.diff(points[:,:3],axis=0),axis=1)/delta
    yaw_delta=np.diff(points[:,3]);yaw=np.abs(np.arctan2(np.sin(yaw_delta),np.cos(yaw_delta)))/delta
    if len(linear):
        max_speed=max(max_speed,float(linear.max()));max_yaw=max(max_yaw,float(yaw.max()))
        axes[1,0].plot(offset+times[1:],linear,color=colors[command['mode']],lw=1.)
        axes[1,1].plot(offset+times[1:],yaw,color=colors[command['mode']],lw=1.)
    for axis in axes[1]:axis.axvline(offset,color='#bbc2ca',lw=.6,ls=':')
    offset+=times[-1]
axes[1,0].axhline(1.,color='#be342c',ls='--',label='Configured 1.0 m/s')
axes[1,1].axhline(.15,color='#be342c',ls='--',label='Configured 0.15 rad/s')
axes[1,0].set(title='Finite-difference command speed',xlabel='Concatenated command time (s)',ylabel='Linear rate (m/s)',ylim=(0,1.12))
axes[1,1].set(title='Finite-difference command yaw rate',xlabel='Concatenated command time (s)',ylabel='Yaw rate (rad/s)',ylim=(0,.17))
for axis in axes[1]:axis.legend(fontsize=8);axis.grid(alpha=.2)
fig.suptitle('Separate ROS 2 PCI trajectory backend + dynamic lidar + test topic consumer\n'
             'Position interpolation only; commanded derivatives are zero; no physical dynamics')
fig.savefig(directory/'execution.png',dpi=180);fig.savefig(directory/'execution.pdf');plt.close(fig)
assert max_speed<=1.+1e-8 and max_yaw<=.15+1e-8
analysis={'status':'passed','commands':len(data['commands']),'distance_m':data['distance_m'],
          'home_error_m':data['home_error_m'],'initial_known_cells':data['initial_known_cells'],
          'final_known_cells':data['final_known_cells'],'known_fraction_percent':100*data['final_known_cells']/data['total_cells'],
          'maximum_command_linear_rate_mps':max_speed,'maximum_command_yaw_rate_radps':max_yaw,
          'command_duration_sum_s':offset,'sampling_note':'Wall-clock observation timing varies between runs; results are not paired performance comparisons.'}
(directory/'analysis.json').write_text(json.dumps(analysis,indent=2)+chr(10));print(json.dumps(analysis,indent=2))
