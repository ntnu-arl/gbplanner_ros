"""Summarize the separately frozen new-route geometry transfer check."""
from pathlib import Path
import json
import math
import numpy as np
from analyze_trial import sha256
from compare_tracking import read_run
from summarize_accuracy_trials import plan_check, production_check, phase_timing, difference
from summarize_center_comparison import strict_base_outcome, recorded_truth_separation


def main():
    root=Path(__file__).resolve().parents[2]/'results/state_estimation_shadow'
    plan_path=root/'kinematic_validation_plan.json'
    plan=json.loads(plan_path.read_text())
    entries=[]
    for expected in plan['runs']:
        entry=read_run(root/expected['directory'],.25)
        entry['key']=expected['directory']
        entry['timing']=phase_timing(entry)
        entry['check']=plan_check(entry,expected,plan)
        entry['path_profile_check']=(entry['run']['path_profile']==expected['path_profile']
                                     ==entry['tracker']['path_profile'])
        entry['production']=production_check(entry)
        entry['separation']=recorded_truth_separation(entry)
        if (entry['check']['status']!='verified' or not entry['path_profile_check']
                or not entry['production']['verified']
                or not entry['separation']['recorded_runtime_separation_checks_pass']):
            raise ValueError('Source/configuration evidence mismatch: '+entry['key'])
        entries.append(entry)
    baseline,candidate=entries
    for field in ('path_points','path_target_yaw','goal_tolerance_xy','goal_tolerance_yaw',
                  'accuracy_profile','wheel_velocity_window','calibration_duration_s','imu_seed'):
        if baseline['run'][field]!=candidate['run'][field]:
            raise ValueError('Not a paired geometry comparison: '+field)
    if baseline['tracker']['overrides']!=candidate['tracker']['overrides']:
        raise ValueError('Controller overrides changed')
    summary={'status':'summarized','scope':'New route/seed transfer check, added after initial geometry studies',
             'plan':{'path':str(plan_path),'sha256':sha256(plan_path),'contents':plan},
             'paired_controller_settings_equal':True,
             'runs':{e['key']:{'outcome':strict_base_outcome(e),'timing':e['timing'],
                     'plan_check':e['check'],'path_profile_matches':e['path_profile_check'],
                     'production':e['production'],'truth_separation':e['separation'],
                     'reference_geometry':e['tracker']['sampling'].get('kinematic_heading')} for e in entries},
             'comparison':difference(candidate,baseline,'Same new route, seed and baseline controller; only intermediate reference-heading construction differs.'),
             'limitations':['One run per variant; not a robustness or statistical superiority claim.',
                            'Nominal seed is shared; callback timing and trajectories can change sampled noise.',
                            'A sharper path tests geometry transfer but does not establish obstacle safety or 3-D feasibility.'],
             'analysis_sha256':sha256(Path(__file__))}
    out=root/'kinematic_validation_comparison';out.mkdir(exist_ok=False)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family':'DejaVu Sans','axes.spines.top':False,'axes.spines.right':False})
    fig,axes=plt.subplots(1,3,figsize=(15,4.7))
    path=baseline['path'];axes[0].plot(path[:,0],path[:,1],'--',color='#999999',label='Desired B path')
    colors=['#b47b43','#167e75'];labels=['Legacy reference','Consistent reference']
    for e,c,label in zip(entries,colors,labels):
        poses=e['poses'];axes[0].plot(poses[:,0],poses[:,1],color=c,label=label,lw=1.8)
    axes[0].set(xlabel='x [m]',ylabel='y [m]',title='New route, IMU seed 23');axes[0].axis('equal');axes[0].legend(fontsize=8)
    metrics=[[],[],[]]
    for e in entries:
        o=e['outcome'];a=o['at_action_result'];f=o['final_settled']
        for values,value in zip(metrics,[a['position_error_m'] if a else math.nan,
                                        f['position_error_m'] if f else math.nan,
                                        o['cross_track_during_action']['rmse_m']]):values.append(100*value)
    x=np.arange(3)
    for i,(c,label) in enumerate(zip(colors,labels)):
        bars=axes[1].bar(x+(i-.5)*.35,[v[i] for v in metrics],.33,color=c,label=label)
        axes[1].bar_label(bars,fmt='%.2f',padding=3,fontsize=8)
    axes[1].set_xticks(x,['At action','Stopped','Cross-track\nRMSE']);axes[1].set(ylabel='B error [cm]',title='Physical base scoring')
    axes[1].set_ylim(top=1.3*max(v for series in metrics for v in series if math.isfinite(v)))
    durations=[e['outcome']['observed_control_duration_sim_s'] for e in entries]
    bars=axes[2].bar(labels,durations,color=colors,width=.55);axes[2].bar_label(bars,fmt='%.2f s',padding=3)
    axes[2].set(ylabel='Control duration [simulation s]',title='Same 10 s calibration in both runs',ylim=(0,max(durations)*1.2))
    for ax in axes:ax.grid(alpha=.2);ax.set_axisbelow(True)
    fig.suptitle('Reference geometry transfer: no controller or estimator retuning',fontweight='bold')
    fig.tight_layout();fig.savefig(out/'validation_comparison.png',dpi=180,facecolor='white');plt.close(fig)
    summary['output_sha256']={'validation_comparison.png':sha256(out/'validation_comparison.png')}
    (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'output':str(out),'comparison':summary['comparison']},indent=2))

if __name__=='__main__':main()
