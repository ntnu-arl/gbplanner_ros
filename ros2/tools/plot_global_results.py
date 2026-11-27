#!/usr/bin/env python3
"""Plot recorded global experiment results; no measurements are synthesized."""
import csv
import json
from pathlib import Path
import statistics
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
from plot_local_results import grid,rows

root=Path(__file__).resolve().parents[1]
results=root/'results'
summary=json.loads((results/'global_exploration/summary.json').read_text())
integration=json.loads((results/'global_integration.json').read_text())
bench=rows(results/'global_experiments/benchmark.csv')
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.titlesize':12,
                     'axes.spines.top':False,'axes.spines.right':False})
fig,axes=plt.subplots(1,2,figsize=(12,5.4),layout='constrained')
x=np.arange(-4.875,5,.25);y=x.copy()
cells=np.zeros((40,40))
for iy,yy in enumerate(y):
    for ix,xx in enumerate(x):
        cells[iy,ix]=-1 if xx>=1.5 else 0
        if abs(xx)<.25 and abs(yy)>1:cells[iy,ix]=100
axes[0].imshow(cells,origin='lower',extent=(-5,5,-5,5),cmap=ListedColormap(['#ced5dd','#fafafa','#303b49']),norm=BoundaryNorm([-2,-.5,50,101],3))
path=np.array(integration['fallback_path'])
axes[0].plot(path[:,0],path[:,1],'-o',color='#146a9b',ms=3,label='Global route after local no_gain')
axes[0].scatter(path[0,0],path[0,1],s=65,c='#bf4a35',marker='s',zorder=4,label='Local planning stalled')
axes[0].scatter(path[-1,0],path[-1,1],s=90,c='#2c7d46',marker='*',zorder=4,label='Stored frontier')
axes[0].set(title=f"Retained frontier recovery: {integration['fallback']['distance_m']:.2f} m",xlabel='x (m)',ylabel='y (m)',xlim=(-5,5),ylim=(-5,5),aspect='equal')
axes[0].legend(loc='upper left',fontsize=8)
grid(axes[1],results/'global_exploration/after_homing_map.csv','Moving lidar: exploration and homing')
out=np.array(summary['trajectory']);back=np.array(summary['homing']['path'])
axes[1].plot(out[:,0],out[:,1],'-o',color='#b86227',ms=3,label=f"Exploration {summary['distance_m']:.2f} m")
axes[1].plot(back[:,0],back[:,1],'--o',color='#146a9b',ms=3,label=f"Homing {summary['homing']['distance_m']:.2f} m")
axes[1].scatter(out[0,0],out[0,1],s=110,c='#2c7d46',marker='*',zorder=5,label='Home')
axes[1].legend(loc='upper left',fontsize=8)
fig.suptitle('Global routing and homing: recorded ROS 2 results\nIndependent scenarios; XY paths on the z = 0.125 m map slice')
fig.savefig(results/'global_exploration/global_paths.png',dpi=180)
fig.savefig(results/'global_exploration/global_paths.pdf');plt.close(fig)

recovered=[r for r in bench if r['global_success']=='1']
failed=[r for r in bench if r['global_success']=='0']
fig,axes=plt.subplots(1,2,figsize=(11,4.5),layout='constrained')
labels=['Local plan\nat dead-end','Global\nrecovery','Return\nto home','Blocked return\nrejected']
values=[sum(r['local_dead_end_reason']=='ok' for r in bench),len(recovered),sum(r['homing_success']=='1' for r in bench),sum(r['blocked_homing_reason']=='global_target_unreachable' for r in bench)]
bars=axes[0].bar(range(4),values,color=['#aeb7c3','#146a9b','#2c7d46','#b86227'])
axes[0].bar_label(bars,labels=[f'{v} / {len(bench)}' for v in values],padding=4)
axes[0].set(xticks=range(4),xticklabels=labels,ylim=(0,23),ylabel='Cases',title='Paired trials, seeds 0–19')
axes[1].scatter([int(r['seed']) for r in recovered],[float(r['global_path_m']) for r in recovered],c='#146a9b',label='Global route length',s=35)
axes[1].scatter([int(r['seed']) for r in failed],[0]*len(failed),c='#bf4a35',marker='x',s=60,label='No retained frontier (no path)')
axes[1].set(xlabel='Random seed',ylabel='Path length (m)',title='Recovery varies with stored frontiers',xticks=range(0,20,2),ylim=(-.7,14))
axes[1].legend(fontsize=8,loc='lower right');axes[1].grid(alpha=.2)
fig.suptitle('Prescribed safe odometry to a dead-end; same map and planner settings\nFailed recovery cases remain in the recorded results')
fig.savefig(results/'global_experiments/recovery.png',dpi=180)
fig.savefig(results/'global_experiments/recovery.pdf');plt.close(fig)
stats={'trials':len(bench),'global_recoveries':len(recovered),'failed_seeds':[int(r['seed']) for r in failed],
       'successful_path_mean_m':statistics.mean(float(r['global_path_m']) for r in recovered),
       'successful_query_median_ms':statistics.median(float(r['global_ms']) for r in recovered),
       'failed_trial_diagnostics':[{k:r[k] for k in ['seed','frontier_candidates','frontier_clusters','rejected_near_graph','rejected_near_history','rejected_path','retained_frontiers']} for r in failed]}
(results/'global_experiments/analysis.json').write_text(json.dumps(stats,indent=2)+chr(10))
print(json.dumps(stats,indent=2))
