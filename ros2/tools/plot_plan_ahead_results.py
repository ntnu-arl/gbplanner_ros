#!/usr/bin/env python3
"""Plot recorded command splices and measured kinematic motion."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

parser=argparse.ArgumentParser()
parser.add_argument('--results',type=Path,default=Path('results/plan_ahead'))
args=parser.parse_args();root=args.results
report=json.loads((root/'system.json').read_text());assert report['status']=='passed'
rows=list(csv.DictReader((root/'system_data/motion.csv').open()))
t=np.array([float(r['time_s']) for r in rows])
xyz=np.array([[float(r[k]) for k in ['x','y','z']] for r in rows])
speed=np.array([float(r['speed_mps']) for r in rows])
replacements=[r for r in report['command_observations'] if r['replaced_while_moving']]
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,
                     'axes.spines.top':False,'axes.spines.right':False})
fig,axes=plt.subplots(1,2,figsize=(12,4.8),layout='constrained')
ax=axes[0]
for index,command in enumerate(report['commands']):
    points=np.array(command['points'])
    ax.plot(points[:,0],points[:,1],lw=1,alpha=.5,color='#8ba5b0')
ax.plot(xyz[:,0],xyz[:,1],color='#176b9e',lw=2,label='Measured motion')
ax.scatter(xyz[0,0],xyz[0,1],s=110,marker='*',color='#278451',label='Home',zorder=6)
for index,r in enumerate(replacements):
    ax.scatter(r['position'][0],r['position'][1],marker='D',s=55,color='#d06a37',zorder=5,
               label='Moving trajectory replacement' if index==0 else None)
ax.set(xlabel='x (m)',ylabel='y (m)',aspect='equal',
       title=f"Automatic exploration and return\n{report['distance_m']:.2f} m; {len(replacements)} moving splices")
ax.legend(fontsize=8)
ax=axes[1];ax.plot(t,speed,color='#176b9e',lw=1.3,label='Measured speed')
for index,r in enumerate(replacements):
    ax.axvline(r['time_s'],color='#d06a37',ls='--',lw=1,label='Trajectory replacement' if index==0 else None)
ax.set(xlabel='Elapsed time (s)',ylabel='Speed (m/s)',
       title=f"Continuation inside the same action\nHold commands: {report['executor']['hold_commands']}")
ax.legend(fontsize=8)
for ax in axes:ax.grid(alpha=.15)
fig.suptitle('Native ROS 2 plan-ahead: live lidar, mission budget, and atomic trajectory continuation')
fig.savefig(root/'plan_ahead.png',dpi=180);fig.savefig(root/'plan_ahead.pdf')
print(json.dumps({'moving_replacements':len(replacements),'plot':str(root/'plan_ahead.png')}))
