#!/usr/bin/env python3
"""Offline scoring of estimated-pose mapping; never alters the map or scene.

The scene defines evaluation masks and plot outlines only. Surface-pointcloud
samples are voxel centres, not reconstructed surface vertices. Linear TSDF
zero crossings are reported separately as discretized field diagnostics.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stats(values):
    a=np.asarray(values,dtype=float)
    if not len(a):return {'count':0}
    return {'count':len(a),'mean_m':float(a.mean()),'median_m':float(np.median(a)),
            'rmse_m':float(np.sqrt(np.mean(a*a))),'std_m':float(a.std()),
            'p05_m':float(np.percentile(a,5)),'p95_m':float(np.percentile(a,95)),
            'p95_absolute_m':float(np.percentile(np.abs(a),95)),
            'min_m':float(a.min()),'max_m':float(a.max())}


def scene_geometry(path):
    world=ET.parse(path).getroot().find('world')
    boxes={}
    for model in world.findall('model'):
        if model.get('name')=='flat_ground':continue
        pose=np.asarray(list(map(float,model.findtext('pose').split())))
        size=np.asarray(list(map(float,model.findtext('link/collision/geometry/box/size').split())))
        if np.any(np.abs(pose[3:])>1e-12):raise ValueError('Scoring assumes axis-aligned boxes')
        boxes[model.get('name')]={'center':pose[:3].tolist(),'size':size.tolist(),
                                 'min':(pose[:3]-size/2).tolist(),'max':(pose[:3]+size/2).tolist()}
    return boxes


def floor_mask(points,boxes):
    # Fixed scene-derived interior window: room faces + 0.5 m inset, remove
    # pillar footprints expanded by 0.5 m. No trajectory- or residual fitting.
    x,y=points[:,0],points[:,1]
    keep=(x>=-1.5)&(x<=7.5)&(y>=-2.5)&(y<=3.5)
    for name,b in boxes.items():
        if not name.endswith('pillar'):continue
        lo,hi=np.array(b['min']),np.array(b['max'])
        keep &= ~((x>=lo[0]-.5)&(x<=hi[0]+.5)&(y>=lo[1]-.5)&(y<=hi[1]+.5))
    return keep


def crossings(xyz,distance,weight,resolution,axis):
    """Interpolate adjacent observed voxel centres with opposite TSDF signs.

    This diagnostic does not extrapolate, smooth or reconstruct missing cells.
    Every returned coordinate belongs to a single adjacent cell pair.
    """
    indices=np.rint(xyz/resolution-.5).astype(np.int64)
    lookup={tuple(index):i for i,index in enumerate(indices)}
    values=[];pair_weights=[]
    for i,index in enumerate(indices):
        nxt=index.copy();nxt[axis]+=1;j=lookup.get(tuple(nxt))
        if j is None or min(weight[i],weight[j])<=1e-4:continue
        d0,d1=float(distance[i]),float(distance[j])
        if not math.isfinite(d0+d1) or d0*d1>=0:continue
        t=d0/(d0-d1)
        values.append(xyz[i]+t*(xyz[j]-xyz[i]));pair_weights.append(min(weight[i],weight[j]))
    return np.asarray(values).reshape(-1,3),np.asarray(pair_weights)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir',type=Path,required=True)
    args=parser.parse_args();out=args.run_dir.resolve();root=Path(__file__).resolve().parents[2]
    run=json.loads((out/'run.json').read_text());mapping=json.loads((out/'mapping_summary.json').read_text())
    probe=json.loads((out/'planner_probe.json').read_text());metrics=json.loads((out/'followup_metrics.json').read_text())
    if run['status']!='completed':raise ValueError('Expected a completed experiment')
    for name in ('voxel_map','surface_pointcloud'):
        if digest(out/(name+'.npz'))!=mapping['products'][name]['sha256']:
            raise ValueError('Saved mapper product differs from recorded hash: '+name)
    scene=out/'generated/flat_world.sdf';boxes=scene_geometry(scene)
    if digest(scene)!=digest(Path(__file__).with_name('mapping_world.sdf')):
        raise ValueError('Recorded scene differs from the scoring scene')
    a=np.load(out/'voxel_map.npz');s=np.load(out/'surface_pointcloud.npz')
    xyz=np.column_stack([a[k] for k in 'xyz']).astype(float)
    surface=np.column_stack([s[k] for k in 'xyz']).astype(float)
    res=float(mapping['final_status']['tsdf_configuration']['resolution'])
    if not math.isclose(res,.1):raise ValueError('Review fixed scoring masks for a new map resolution')
    rows=[json.loads(line) for line in (out/'events.jsonl').open()]
    estimate=[r for r in rows if r['kind']=='mapping_input']
    truth=[r for r in rows if r['kind']=='truth' and r['stamp_ns']>=estimate[0]['stamp_ns']]
    truth_xyz=np.array([r['position'] for r in truth]);estimate_xyz=np.array([r['position'] for r in estimate])
    route=np.asarray(run['path_points']);planned=np.asarray([p['position'] for p in probe['search_response']['path']])
    ground_surface=surface[floor_mask(surface,boxes)&(np.abs(surface[:,2])<=.25)]
    zcross,_=crossings(xyz,a['tsdf_distance'],a['tsdf_weight'],res,2)
    ground_cross=zcross[floor_mask(zcross,boxes)&(np.abs(zcross[:,2])<=.25)]
    # Wall inner faces only. Endpoint inset excludes corners; z excludes the
    # floor and top edges; a +/-0.25 m band rejects unrelated scene surfaces.
    wall_specs={'west_wall':(0,-2.,1,.0), 'east_wall':(0,8.,-1,.0),
                'south_wall':(1,-3.,1,.0), 'north_wall':(1,4.,-1,.0)}
    wall_results={};wall_cross_values=[]
    for name,(axis,face,inside_sign,_) in wall_specs.items():
        other=1-axis;bounds=(-2.5,3.5) if axis==0 else (-1.5,7.5)
        def mask(p):
            return ((np.abs(p[:,axis]-face)<=.25)&(p[:,other]>=bounds[0])&
                    (p[:,other]<=bounds[1])&(p[:,2]>=.3)&(p[:,2]<=1.7))
        selected=surface[mask(surface)]
        zero,_=crossings(xyz,a['tsdf_distance'],a['tsdf_weight'],res,axis)
        selected_zero=zero[mask(zero)]
        centre_residual=inside_sign*(selected[:,axis]-face)
        zero_residual=inside_sign*(selected_zero[:,axis]-face)
        wall_cross_values.append(zero_residual)
        wall_results[name]={'axis':axis,'plane_coordinate_m':face,
             'sign':'positive toward room interior','surface_voxel_centres':stats(centre_residual),
             'linear_tsdf_zero_crossings':stats(zero_residual),
             'selection':{'normal_band_m':.25,'tangent_window_m':bounds,'z_window_m':[.3,1.7]}}
    floor_height=stats(ground_cross[:,2])
    projected_floor=planned[:,2]-.24
    final=mapping['final_status']
    summary={'status':'analyzed','scope':'One independent native dual-lidar estimated-pose mapping run and a planning query',
      'scene_used_for_scoring_only':True,'scene_used_as_map_input':False,
      'truth_used_for_scoring_only':True,'map_changed_by_analysis':False,
      'mapping':{'voxel_resolution_m':res,'saved_known_voxels':len(xyz),
                 'saved_surface_voxel_centres':len(surface),
                 'saved_occupancy_counts':{str(int(k)):int(v) for k,v in zip(*np.unique(a['occupancy'],return_counts=True))},
                 'integrated_scans':final['integrated_scans'],'input_sources':final['input_sources'],
                 'truth_input':mapping['truth_input'],'icp_enabled':mapping['icp_enabled'],'is_slam':mapping['is_slam'],
                 'product_stamps':mapping['products'],
                 'note':'The saved product and planner can see different accepted-map revisions while sensors continue.'},
      'evaluation_definition':{'selection_timing':'Fixed scene-derived analysis windows written before evaluating these metrics; not a preregistered runtime benchmark.',
           'floor_window_xy_m':[-1.5,7.5,-2.5,3.5],'floor_vertical_band_m':[-.25,.25],
           'pillar_exclusion_expansion_m':.5,'tsdf_pair_min_weight':1e-4,
           'surface_cloud_semantics':'Voxel centres with weight>0 and abs(TSDF distance)<0.75*resolution; not exact reconstructed surface points.',
           'zero_crossing_method':'Linear interpolation along one coordinate axis between adjacent observed voxel centres of opposite TSDF signs, no smoothing or extrapolation.',
           'voxel_centre_axis_quantization_scale_m':res/2,
           'zero_crossing_limit':'Interpolated projective TSDF field diagnostic, not exact mesh accuracy; sensor noise, viewing angle, pose error and discretization are mixed.'},
      'floor':{'reference_height_m':0.,'surface_voxel_centres':stats(ground_surface[:,2]),
               'linear_tsdf_zero_crossings':floor_height,
               'planner_projected_floor_at_returned_path':stats(projected_floor),
               'physical_base_height_during_mapping':stats(truth_xyz[:,2]),
               'declared_estimated_base_height_m':mapping['height_m'],
               'declared_minus_median_physical_base_height_m':mapping['height_m']-float(np.median(truth_xyz[:,2])),
               'attribution':'The negative projected floor is preserved. Physical base height matches the declared 0.24 m; fixed chassis height does not explain it. This run cannot uniquely separate renderer/TSDF/pose causes.'},
      'walls':wall_results,
      'state_estimation':metrics['estimates']['ekf_slip'],
      'physical_endpoint':metrics['tracking']['final_truth_endpoint'],
      'planning':{'status':probe['status'],'success':probe['search_response']['success'],
                  'scope':probe['scope'],'target':probe['request']['target']['position'],
                  'path_xyz':planned.tolist(),'path_length_m':float(np.linalg.norm(np.diff(planned,axis=0),axis=1).sum()),
                  'path_vertices':len(planned),'validation':{k:v['response'] for k,v in probe['validation'].items()},
                  'search_metrics':probe['search_metrics'],'path_executed':False,
                  'search_map_revision':probe['search_map_revision'],
                  'known_cells_at_planner':probe['ready_map_status']['data']['known_cells']},
      'scene_boxes':boxes,
      'plot_data':{'desired_path_xy':route.tolist(),'truth_trajectory_xyz':truth_xyz[::5].tolist(),
                   'estimated_trajectory_xyz':estimate_xyz[::5].tolist()},
      'limitations':['This is dead reckoning plus TSDF mapping, without lidar pose correction or loop closure.',
                     'The physical robot executed the predefined outbound route; the returned PlannerSearch path was validated but not executed.',
                     'Only observed selected surface regions are scored; no completeness, collision-free mission or general robustness claim.',
                     'Wall/floor voxel-centre metrics include 0.1 m voxel discretization. Do not present their residuals as exact surface reconstruction accuracy.',
                     'The map and subsequent planner snapshot are not identical-time captures.'],
      'input_sha256':{p.name:digest(p) for p in (out/'voxel_map.npz',out/'surface_pointcloud.npz',out/'events.jsonl',out/'planner_probe.json',out/'mapping_summary.json',scene)},
      'analysis_source_sha256':digest(__file__)}
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from plot_fonts import plot_font_family
    family=plot_font_family(root/'docs/course_report/fonts/HeitiSC.ttf')
    plt.rcParams.update({'font.family':family,'font.size':11,'axes.unicode_minus':False,
                         'axes.spines.top':False,'axes.spines.right':False})
    fig=plt.figure(figsize=(14,8.7));grid=fig.add_gridspec(2,2,width_ratios=[1.45,1.],hspace=.4,wspace=.26)
    ax=fig.add_subplot(grid[:,0]);floor_ax=fig.add_subplot(grid[0,1]);wall_ax=fig.add_subplot(grid[1,1])
    from matplotlib.patches import Rectangle
    free=(a['occupancy']==0)&(xyz[:,2]>=.15)&(xyz[:,2]<=1.25)
    free_xy=np.unique(np.round(xyz[free,:2],3),axis=0)
    ax.scatter(free_xy[:,0],free_xy[:,1],s=5,marker='s',color='#e5ecef',label='已观测自由空间（顶视投影）',rasterized=True)
    obs=surface[(surface[:,2]>=.3)&(surface[:,2]<=1.7)]
    ax.scatter(obs[:,0],obs[:,1],s=5,color='#52646e',alpha=.65,label='表面体素中心（z=0.3–1.7 m）',rasterized=True)
    for i,(name,b) in enumerate(boxes.items()):
        lo,hi=b['min'],b['max'];ax.add_patch(Rectangle(lo[:2],hi[0]-lo[0],hi[1]-lo[1],
            edgecolor='#9a7664',facecolor='none',linestyle='--',linewidth=1.2,label='场景轮廓，仅用于评分' if i==0 else None))
    ax.plot(route[:,0],route[:,1],':',color='#555555',lw=1.5,label='预设行驶路线')
    ax.plot(truth_xyz[:,0],truth_xyz[:,1],color='#007d74',lw=2.1,label='实际底盘轨迹')
    ax.plot(estimate_xyz[:,0],estimate_xyz[:,1],color='#d79a38',lw=1.2,alpha=.9,label='轮速＋IMU 估计轨迹')
    ax.plot(planned[:,0],planned[:,1],color='#b33e55',lw=2.1,linestyle='--',label='实际规划结果（已验证，未执行）')
    ax.scatter(planned[-1,0],planned[-1,1],s=85,marker='*',color='#b33e55',zorder=8)
    ax.set(xlim=(-2.55,8.55),ylim=(-3.55,4.55),xlabel='x / m',ylabel='y / m',title='估计位姿驱动的双雷达地图与规划查询')
    ax.set_aspect('equal');ax.legend(loc='upper center',bbox_to_anchor=(.5,1.20),ncol=2,fontsize=8,framealpha=.94)
    ax.set_title('估计位姿驱动的双雷达地图与规划查询',pad=88)
    floor_ax.hist(ground_cross[:,2]*1000,bins=np.arange(-250,255,5),color='#548d87',alpha=.9)
    floor_ax.axvline(0,color='#555555',ls='--',lw=1.4,label='场景地面 z=0')
    floor_ax.axvline(floor_height['median_m']*1000,color='#b33e55',lw=1.6,label=f"TSDF 中位数 {floor_height['median_m']*1000:.1f} mm")
    floor_ax.set(xlabel='沿 z 线性插值零交叉高度 / mm',ylabel='相邻体素对数',title='地面：观测偏差保留，不平移地图')
    floor_ax.set_xlim(-80,60);floor_ax.legend(fontsize=8)
    labels=['西墙','东墙','南墙','北墙']
    bp=wall_ax.boxplot([1000*x for x in wall_cross_values],tick_labels=labels,showfliers=False,patch_artist=True,
                       medianprops={'color':'#b33e55'},boxprops={'facecolor':'#d9e5e9'})
    wall_ax.axhline(0,color='#555555',ls='--',lw=1.2)
    wall_ax.set(ylabel='沿法向线性 TSDF 零交叉偏差 / mm',title='墙面：正值为向室内偏移')
    wall_ax.set_ylim(-24,23)
    for i,v in enumerate(wall_cross_values):wall_ax.text(i+1,.97,f'n={len(v)}',transform=wall_ax.get_xaxis_transform(),ha='center',va='top',fontsize=8)
    for axis in (ax,floor_ax,wall_ax):axis.grid(alpha=.16);axis.set_axisbelow(True)
    fig.suptitle('轮速／IMU 估计位姿 ＋ 真实双雷达 → Voxblox → GBPlanner',fontsize=18,fontweight='bold',y=.99)
    fig.text(.5,.025,'分辨率 0.1 m；表面点是体素中心，插值零交叉仅为场诊断。场景与真值仅参与评分，未输入建图或规划。',ha='center',fontsize=10,color='#555555')
    fig.subplots_adjust(top=.90,bottom=.12,left=.06,right=.98)
    fig.savefig(out/'mapping_overview.png',dpi=180,facecolor='white');plt.close(fig)
    summary['output_sha256']={'mapping_overview.png':digest(out/'mapping_overview.png')}
    (out/'mapping_analysis.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'floor':summary['floor'],'walls':wall_results,'figure':str(out/'mapping_overview.png')},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
