#!/usr/bin/env python3
"""Plot recorded outputs. Does not generate or replace experiment measurements."""
import argparse
import csv
import json
from pathlib import Path
import statistics
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.patches import Patch

def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))

def grid(axis, path, title):
    data=rows(path)
    xs=sorted({float(r['x']) for r in data});ys=sorted({float(r['y']) for r in data})
    lookup={(float(r['x']),float(r['y'])):int(r['status']) for r in data}
    values=np.array([[lookup[(x,y)] for x in xs] for y in ys])
    cmap=ListedColormap(['#ced5dd','#fafafa','#303b49'])
    norm=BoundaryNorm([-2,-0.5,50,101],cmap.N)
    axis.imshow(values,origin='lower',extent=[xs[0]-.125,xs[-1]+.125,ys[0]-.125,ys[-1]+.125],cmap=cmap,norm=norm)
    axis.set(title=title,xlabel='x (m)',ylabel='y (m)',xlim=(-5,5),ylim=(-5,5))
    axis.set_aspect('equal')

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--results',type=Path,default=Path('results/local_experiments'))
    args=parser.parse_args();directory=args.results
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.titlesize':12,'figure.dpi':140,
                         'axes.spines.top':False,'axes.spines.right':False})
    benchmark=rows(directory/'benchmark.csv');paths=rows(directory/'paths.csv')
    history=rows(directory/'exploration.csv');trajectory=rows(directory/'trajectory.csv')
    summary=json.loads((directory/'experiment_summary.json').read_text())
    colors={'tree':'#b55b24','graph':'#146a9b'}
    fig,axes=plt.subplots(1,3,figsize=(13,4.7),layout='constrained')
    for axis,scene,title in zip(axes,['open','doorway','sealed'],['Open space','Wall with doorway','Sealed wall']):
        grid(axis,directory/f'{scene}_map.csv',title)
        for mode in ['tree','graph']:
            path=[r for r in paths if r['scene']==scene and r['mode']==mode and r['seed']=='0']
            if path:
                axis.plot([float(r['x']) for r in path],[float(r['y']) for r in path],'-o',color=colors[mode],lw=2,ms=3,label=mode)
        axis.scatter([-2],[0],marker='*',s=100,c='#2a803f',zorder=5)
        if scene=='sealed':axis.text(1.8,-3.5,'No gain\n20 / 20 seeds',ha='center',fontsize=10)
    handles=[Patch(color='#ced5dd',label='Unknown'),Patch(color='#303b49',label='Occupied'),
             plt.Line2D([],[],color=colors['tree'],label='Tree path'),plt.Line2D([],[],color=colors['graph'],label='Graph path')]
    fig.legend(handles=handles,loc='outside lower center',ncol=4,frameon=False)
    fig.suptitle('Local planning: measured paths, seed 0 (XY projection; map slice z = 0.125 m)')
    fig.savefig(directory/'planning_paths.png',dpi=180);fig.savefig(directory/'planning_paths.pdf');plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(11,9),layout='constrained')
    for axis,file,title in [(axes[0,0],'exploration_initial_map.csv','Initial observation'),(axes[0,1],'exploration_final_map.csv','After 30 executed edges')]:
        grid(axis,directory/file,title)
    tx=[float(r['x']) for r in trajectory];ty=[float(r['y']) for r in trajectory]
    axes[0,1].plot(tx,ty,'-',color='#b7562a',lw=1.7,label='Executed trajectory')
    axes[0,0].scatter(tx[0],ty[0],marker='*',s=110,c='#2a803f')
    axes[0,1].scatter([tx[0],tx[-1]],[ty[0],ty[-1]],c=['#2a803f','#c12f40'],s=45,zorder=5)
    axes[0,1].legend(loc='lower right',fontsize=9)
    initial=summary['closed_loop']['initial_known_cells']/summary['closed_loop']['total_cells']*100
    fraction=[initial]+[float(r['known_fraction'])*100 for r in history]
    axes[1,0].plot(range(len(fraction)),fraction,'-o',ms=3,color='#146a9b')
    axes[1,0].set(title='Observed fraction of the bounded grid',xlabel='Executed edges',ylabel='Known cells / total cells (%)',ylim=(0,100))
    axes[1,0].annotate(f'{fraction[-1]:.1f}%',(len(fraction)-1,fraction[-1]),xytext=(-35,-18),textcoords='offset points')
    times=[float(r['planning_ms']) for r in history]
    axes[1,1].plot(range(len(times)),times,'-o',ms=3,color='#667940')
    axes[1,1].set(title='Measured local planning time',xlabel='Planning iteration',ylabel='Core computation (ms)',ylim=(0,max(times)*1.2))
    for axis in axes[1]:axis.grid(alpha=.2)
    fig.suptitle('Synthetic exploration: ideal lidar, kinematic first-edge execution\n30-step budget reached; no SLAM, dynamics or global exploration')
    fig.savefig(directory/'exploration.png',dpi=180);fig.savefig(directory/'exploration.pdf');plt.close(fig)
    grouped=[]
    for scene in ['open','doorway','sealed']:
        for mode in ['tree','graph']:
            group=[r for r in benchmark if r['scene']==scene and r['mode']==mode]
            times=[float(r['build_ms'])+float(r['evaluate_ms']) for r in group]
            grouped.append({'scene':scene,'mode':mode,'runs':len(group),'successful_plans':sum(int(r['success']) for r in group),
                'reasons':sorted({r['reason'] for r in group}),'median_planning_ms':statistics.median(times),
                'p90_planning_ms':float(np.quantile(times,.9)),
                'mean_path_length_m':statistics.mean(float(r['path_length_m']) for r in group),
                'mean_score':statistics.mean(float(r['gain']) for r in group),
                'all_output_paths_collision_free':all(int(r['collision_free']) for r in group)})
    (directory/'benchmark_summary.json').write_text(json.dumps(grouped,indent=2)+'\n')
    fig,axes=plt.subplots(1,2,figsize=(10,4.5),layout='constrained')
    for mode,shift in [('tree',-.17),('graph',.17)]:
        groups=[g for g in grouped if g['mode']==mode]
        axes[0].bar(np.arange(3)+shift,[g['median_planning_ms'] for g in groups],width=.32,color=colors[mode],label=mode)
        axes[1].bar(np.arange(2)+shift,[g['mean_path_length_m'] for g in groups[:2]],width=.32,color=colors[mode],label=mode)
    axes[0].set(xticks=range(3),xticklabels=['Open','Doorway','Sealed'],ylabel='Median core computation (ms)',title='20 seeds per condition')
    axes[1].set(xticks=range(2),xticklabels=['Open','Doorway'],ylabel='Mean selected path length (m)',title='Successful planning conditions')
    axes[0].legend(frameon=False);axes[1].legend(frameon=False)
    fig.suptitle('Tree/graph ablation within this port; not a ROS 1 vs ROS 2 performance comparison')
    fig.savefig(directory/'benchmark.png',dpi=180);fig.savefig(directory/'benchmark.pdf');plt.close(fig)
    ros_directory=directory.parent/'ros_exploration'
    if (ros_directory/'summary.json').exists():
        ros_summary=json.loads((ros_directory/'summary.json').read_text())
        ros_history=rows(ros_directory/'history.csv');ros_trajectory=rows(ros_directory/'trajectory.csv')
        fig,axes=plt.subplots(1,3,figsize=(14,4.8),layout='constrained')
        grid(axes[0],ros_directory/'initial_map.csv','Initial lidar map')
        grid(axes[1],ros_directory/'final_map.csv','Map after repeated planning')
        x=[float(r['x']) for r in ros_trajectory];y=[float(r['y']) for r in ros_trajectory]
        axes[1].plot(x,y,'-o',color='#b7562a',lw=1.8,ms=3)
        axes[0].scatter(x[0],y[0],c='#2a803f',marker='*',s=100)
        axes[1].scatter([x[0],x[-1]],[y[0],y[-1]],c=['#2a803f','#c12f40'],s=45,zorder=5)
        fractions=[ros_summary['initial_known_cells']/ros_summary['total_cells']*100]+[float(r['known_fraction'])*100 for r in ros_history]
        axes[2].plot(range(len(fractions)),fractions,'-o',color='#146a9b',ms=4)
        axes[2].set(title='Known grid cells',xlabel='Executed edges',ylabel='Known / total (%)',ylim=(0,100))
        axes[2].grid(alpha=.2)
        axes[2].annotate(f'{fractions[-1]:.1f}%',(len(fractions)-1,fractions[-1]),xytext=(-40,-20),textcoords='offset points')
        fig.suptitle(f"ROS 2 closed loop: moving TF + synthetic point clouds + mapper + local planner\n{ros_summary['executed_steps']} edges, {ros_summary['distance_m']:.2f} m; stopped: {ros_summary['stop_reason']} (kinematic execution)")
        fig.savefig(ros_directory/'ros_exploration.png',dpi=180);fig.savefig(ros_directory/'ros_exploration.pdf');plt.close(fig)
    print(json.dumps({'generated':sorted(p.name for p in directory.glob('*.png')),'known_percent_initial':initial,
                      'known_percent_final':fraction[-1],'groups':grouped},indent=2))

if __name__=='__main__':main()
