#!/usr/bin/env python3
"""Render original marching-cubes PLY geometry and measured ESDF values."""
from pathlib import Path
import argparse,csv,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

parser=argparse.ArgumentParser();parser.add_argument('--results',type=Path,default=Path('results/map_products'))
args=parser.parse_args();root=args.results
with (root/'room.ply').open() as stream:
    vertex_count=face_count=0
    for line in stream:
        if line.startswith('element vertex'):vertex_count=int(line.split()[-1])
        if line.startswith('element face'):face_count=int(line.split()[-1])
        if line.strip()=='end_header':break
    vertices=np.array([[float(x) for x in next(stream).split()[:3]] for _ in range(vertex_count)])
    faces=np.array([[int(x) for x in next(stream).split()[1:4]] for _ in range(face_count)])
assert len(vertices)>0 and len(faces)>0
with (root/'esdf.csv').open() as stream:
    values=np.array([[float(row[k]) for k in ('x','y','z','distance')] for row in csv.DictReader(stream)])
z=min(np.unique(values[:,2]),key=lambda v:abs(v-.125));values=values[np.isclose(values[:,2],z)]
report=json.loads((root/'summary.json').read_text())
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10})
fig=plt.figure(figsize=(13,5.5),layout='constrained')
ax=fig.add_subplot(121,projection='3d');triangles=vertices[faces]
# Open the roof/front for viewing; the PLY file retains every extracted triangle.
centers=triangles.mean(axis=1);display=triangles[(centers[:,2]<.9)&(centers[:,1]<4.5)]
collection=Poly3DCollection(display,facecolor='#71abb6',edgecolor='#365a67',linewidth=.1,alpha=.8)
ax.add_collection3d(collection)
ax.set(xlim=(-5,5),ylim=(-5,5),zlim=(-1.5,1.5),xlabel='x (m)',ylabel='y (m)',zlabel='z (m)',
       title=f'Original marching cubes: {face_count:,} triangles\nRoof/front cut away for display')
ax.set_box_aspect((10,10,3));ax.view_init(elev=31,azim=-64)
ax.scatter([-2,2],[0,0],[0,0],marker='*',s=85,c='#b0522a',depthshade=False)
ax=fig.add_subplot(122);ax.set_facecolor('#b7bac0')
points=ax.scatter(values[:,0],values[:,1],c=values[:,3],s=20,marker='s',cmap='RdBu',vmin=-.75,vmax=2)
fig.colorbar(points,ax=ax,label='ESDF signed distance (m)',shrink=.8)
path=np.array(report['esdf_path']);ax.plot(path[:,0],path[:,1],'-o',color='#ed9c15',lw=2,ms=4,label='ESDF planning path')
ax.set(xlabel='x (m)',ylabel='y (m)',title=f'Incremental ESDF at z = {z:.3f} m\nGray = unobserved; configured distance cap = 2 m',
       xlim=(-5,5),ylim=(-5,5),aspect='equal');ax.legend(fontsize=8)
fig.suptitle('Saved scene reused after process restart: surface mesh and distance-based planning')
fig.savefig(root/'map_products.png',dpi=180);fig.savefig(root/'map_products.pdf')
print(json.dumps({'vertices':len(vertices),'triangles':len(faces),'displayed_triangles':len(display),
                  'esdf_slice_voxels':len(values)},indent=2))
