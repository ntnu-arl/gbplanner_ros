#!/usr/bin/env python3
"""Run fresh ROS 2 process groups for several seeds and a seed-42 repeat."""
import csv
import json
import math
import os
from pathlib import Path
import subprocess

root=Path(__file__).resolve().parents[1];output=root/'results/system_experiments'
output.mkdir(parents=True,exist_ok=True)
rows=[];runs={}
for index,(name,seed) in enumerate([('seed0',0),('seed21',21),('seed42',42),('seed42_repeat',42)]):
    target=output/name;target.mkdir(exist_ok=True)
    environment=dict(os.environ,ROS_DOMAIN_ID=str(55+index))
    with (target/'run.log').open('w') as log:
        run=subprocess.run(['ros2','run','gbplanner_local','system_demo.py','--start-node','--seed',str(seed),'--output',str(target)],
                           env=environment,cwd=root,stdout=log,stderr=subprocess.STDOUT,timeout=210)
    text=(target/'run.log').read_text()
    valid=run.returncode==0 and 'Traceback' not in text and 'process has died' not in text
    if not valid:
        rows.append({'run':name,'seed':seed,'status':'failed','returncode':run.returncode})
        print(name,'FAILED; retained run.log',flush=True);continue
    data=json.loads((target/'summary.json').read_text());runs[name]=data
    distances={mode:sum(math.dist(a,b) for action in data['actions'] if action['mode']==mode for a,b in zip(action['path'],action['path'][1:]))
               for mode in ['local','global','homing']}
    rows.append({'run':name,'seed':seed,'status':data['status'],'returncode':run.returncode,
                 'actions':len(data['actions']),'local_m':distances['local'],'global_m':distances['global'],'homing_m':distances['homing'],
                 'total_m':data['distance_m'],'known_cells':data['final_known_cells'],
                 'home_error_m':data['final_home_error_m'],'background_added':data['background']['total_added']})
    print(name,data['control']['reason'],len(data['actions']),'actions',data['distance_m'],'m',flush=True)
keys=list(dict.fromkeys(k for row in rows for k in row))
with (output/'runs.csv').open('w',newline='') as stream:
    writer=csv.DictWriter(stream,fieldnames=keys);writer.writeheader();writer.writerows(rows)
repeat=None
if 'seed42' in runs and 'seed42_repeat' in runs:
    a,b=runs['seed42'],runs['seed42_repeat']
    repeat={'same_action_paths':a['actions']==b['actions'],'same_known_cells':a['final_known_cells']==b['final_known_cells'],
            'distance_difference_m':abs(a['distance_m']-b['distance_m'])}
summary={'status':'passed' if all(r['status']=='passed' for r in rows) else 'failed','runs':rows,'seed42_repeat':repeat,
         'timing_note':'DDS scheduling may change duplicate scans and background timer counts; compare paths and map outcome separately.'}
(output/'summary.json').write_text(json.dumps(summary,indent=2)+chr(10))
if summary['status']!='passed':raise SystemExit('One or more system experiments failed; see retained logs')
print(json.dumps(repeat,indent=2))
