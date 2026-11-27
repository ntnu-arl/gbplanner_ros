#!/usr/bin/env python3
"""Plot observed TSDF voxels exported by the ROS2 integration check."""
import argparse
import csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm

parser=argparse.ArgumentParser()
parser.add_argument('--input',type=Path,required=True)
args=parser.parse_args()
with args.input.open() as stream:
    rows=list(csv.DictReader(stream))
a=np.array([[float(row[key]) for key in ('x','y','z','tsdf_distance_m','weight')] for row in rows])
z=min(np.unique(a[:,2]),key=lambda value:abs(value-.125))
a=a[np.isclose(a[:,2],z)]
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10})
fig,axes=plt.subplots(1,3,figsize=(13,4.2),layout='constrained')
points=axes[0].scatter(a[:,0],a[:,1],c=a[:,3],s=19,marker='s',cmap='RdBu',vmin=-.75,vmax=.75)
fig.colorbar(points,ax=axes[0],label='Signed distance (m)')
points=axes[1].scatter(a[:,0],a[:,1],c=a[:,4],s=19,marker='s',cmap='viridis',norm=LogNorm())
fig.colorbar(points,ax=axes[1],label='Integration weight')
for occupied,color,label in [(False,'#edf1f4','Free'),(True,'#303c49','Occupied')]:
    subset=a[(a[:,3]<=.25+1e-6)==occupied]
    axes[2].scatter(subset[:,0],subset[:,1],c=color,s=19,marker='s',label=label)
axes[2].legend(fontsize=8)
for axis,title in zip(axes,['Original TSDF distance','Accumulated measurement weight','Original GBPlanner classification']):
    axis.set(title=title,xlabel='x (m)',ylabel='y (m)',xlim=(-5,5),ylim=(-5,5),aspect='equal')
    axis.set_facecolor('#b7bac0');axis.scatter([-2],[0],c='#a84922',marker='*',s=80)
fig.suptitle(f'First synthetic scan: measured layer at z = {z:.3f} m; gray background = unknown')
fig.savefig(args.input.with_suffix('.png'),dpi=180)
fig.savefig(args.input.with_suffix('.pdf'))
print(f'Plotted {len(a)} observed voxels in one slice; full export has {len(rows)} voxels')
