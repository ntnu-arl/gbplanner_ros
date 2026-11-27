#!/usr/bin/env python3
"""Compare recorded native graph coordinates with configured sampling bounds."""
import itertools
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root=Path('results/rotated_sampling')
report=json.loads((root/'summary.json').read_text());assert report['status']=='passed'
world=np.array([v['world'] for v in report['vertices']])
local=np.array([v['sampling'] for v in report['vertices']])
path=np.array(report['path']);origin=np.array(report['root'])
yaw,pitch,roll=report['rotations_ypr_rad'];c,s=np.cos,np.sin
rz=np.array([[c(yaw),-s(yaw),0],[s(yaw),c(yaw),0],[0,0,1]])
ry=np.array([[c(pitch),0,s(pitch)],[0,1,0],[-s(pitch),0,c(pitch)]])
rx=np.array([[1,0,0],[0,c(roll),-s(roll)],[0,s(roll),c(roll)]])
rotation=rz@ry@rx
corners=np.array(list(itertools.product(*zip(report['min'],report['max']))))
world_corners=corners@rotation+origin
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':9})
fig=plt.figure(figsize=(13,5.4),layout='constrained')
axes=[fig.add_subplot(121,projection='3d'),fig.add_subplot(122,projection='3d')]
for ax,points,bounds,route,title in zip(axes,[world,local],[world_corners,corners],
                                       [path,(path-origin)@rotation.T],
                                       ['World frame: rotated planning region','Sampling frame: original axis-aligned bounds']):
    ax.scatter(*points.T,s=12,c='#2a75a0',alpha=.75,label='RRG vertices')
    ax.plot(*route.T,c='#c16d36',lw=2.4,label='Selected path')
    for a,b in itertools.combinations(range(8),2):
        if bin(a^b).count('1')==1:ax.plot(*np.array([bounds[a],bounds[b]]).T,c='#8c8c8c',lw=.9,alpha=.75)
    ax.set(title=title,xlabel='x (m)',ylabel='y (m)',zlabel='z (m)')
    ax.set_box_aspect((2.8,2,1));ax.view_init(elev=24,azim=-62)
    ax.legend(fontsize=8,loc='upper left')
fig.suptitle(f"Fixed upstream yaw/pitch/roll = {yaw:.1f}, {pitch:.1f}, {roll:.1f} rad\n"
             f"{len(world)} vertices; {report['vertices_outside_old_world_axis_box']} correctly extend outside the old unrotated box")
fig.savefig(root/'rotated_sampling.png',dpi=180)
fig.savefig(root/'rotated_sampling.pdf')
