#!/usr/bin/env python3
"""Visualize saved geofence routes and actual action stop observations."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

parser=argparse.ArgumentParser()
parser.add_argument('--results',type=Path,default=Path('results/geofence'))
args=parser.parse_args();root=args.results
report=json.loads((root/'summary.json').read_text());assert report['status']=='passed'
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,
                     'axes.spines.top':False,'axes.spines.right':False})
fig,axes=plt.subplots(1,2,figsize=(12,5),layout='constrained')
ax=axes[0]
ax.add_patch(Rectangle((-.6,-.6),.8,1.2,facecolor='#f2b4a4',edgecolor='#a34628',label='Forbidden XY polygon'))
ax.add_patch(Rectangle((-.8,-.8),1.2,1.6,fill=False,ls='--',edgecolor='#a34628',label='0.2 m footprint padding'))
path=np.array(report['local_path'])
ax.plot(path[:,0],path[:,1],'-o',color='#176b9e',lw=2,ms=4,label='Returned 3D path (XY view)')
ax.scatter(path[0,0],path[0,1],marker='*',color='#278451',s=100,zorder=5)
ax.set(title=f"Local RRG: {report['local_checked_edges']} edges checked\nPath length {report['local_plan']['path_length_m']:.3f} m",
       xlabel='x (m)',ylabel='y (m)',aspect='equal',xlim=(-3,2),ylim=(-3.5,1.5))
ax.legend(loc='upper left',fontsize=8)
ax=axes[1]
ax.add_patch(Rectangle((.35,-1.5),.3,3.,facecolor='#f2b4a4',edgecolor='#a34628',label='Inserted while moving'))
ax.axvline(.15,color='#a34628',ls='--',lw=1,label='Robot-center limit (0.2 m margin)')
ax.plot([-2,2],[0,0],'--',color='#8199a8',lw=1.5,label='Requested route')
stop=report['stopped_position']
ax.plot([-2,stop[0]],[0,stop[1]],color='#176b9e',lw=3,label='Executed before stop')
ax.scatter([stop[0]],[stop[1]],marker='X',s=90,color='#a34628',zorder=5,label='Observed stop')
ax.annotate(f'x = {stop[0]:.3f} m',xy=(stop[0],stop[1]),xytext=(-2.5,.7),arrowprops={'arrowstyle':'->','color':'#444'})
ax.scatter([2],[0],s=70,marker='*',color='#278451',zorder=5,label='Reached after clearing')
ax.set(title='Action invalidation and subsequent execution\nNo new occupancy scan needed for policy change',
       xlabel='x (m)',ylabel='y (m)',aspect='equal',xlim=(-3,3),ylim=(-2,2))
ax.legend(loc='lower left',fontsize=8)
for ax in axes:ax.grid(alpha=.15)
fig.suptitle('ROS 2 geofence integration: planning geometry, live action stop, and route recovery')
fig.savefig(root/'geofence.png',dpi=180)
fig.savefig(root/'geofence.pdf')
print(json.dumps({'local_path_length_m':report['local_plan']['path_length_m'],
                  'checked_graph_edges':report['local_checked_edges'],
                  'stopped_x_m':stop[0],'scope':'Kinematic action consumer, not physical flight'},indent=2))
