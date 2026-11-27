#!/usr/bin/env python3
"""Repeat the same timed execution scenario in fresh processes, seed 42."""
import csv
import json
import os
from pathlib import Path
import subprocess

root=Path(__file__).resolve().parents[1]
directory=root/'results/execution_repeats';directory.mkdir(exist_ok=True)
records=[]
for index in range(3):
    run=directory/f'run{index+1}';run.mkdir(exist_ok=True)
    env=dict(os.environ,ROS_DOMAIN_ID=str(65+index))
    with (run/'run.log').open('w') as stream:
        result=subprocess.run(['ros2','run','gbplanner_execution','execution_system_demo.py',
            '--start-node','--output',str(run)],cwd=root,env=env,stdout=stream,stderr=subprocess.STDOUT,timeout=220)
    log=(run/'run.log').read_text()
    assert result.returncode==0 and 'Traceback' not in log and 'process has died' not in log, str(run)
    data=json.loads((run/'summary.json').read_text());assert data['status']=='passed'
    records.append({'run':index+1,'seed':42,'status':data['status'],'commands':len(data['commands']),
        'distance_m':data['distance_m'],'known_cells':data['final_known_cells'],
        'home_error_m':data['home_error_m'],'rejected_scans':data['mapper']['rejected_scans'],
        'action_aborts':data['executor']['aborted']})
    print(json.dumps(records[-1]),flush=True)
with (directory/'runs.csv').open('w',newline='') as stream:
    writer=csv.DictWriter(stream,fieldnames=list(records[0]));writer.writeheader();writer.writerows(records)
summary={'status':'passed','runs':records,'seed':42,
    'distance_range_m':[min(r['distance_m'] for r in records),max(r['distance_m'] for r in records)],
    'known_cells_range':[min(r['known_cells'] for r in records),max(r['known_cells'] for r in records)],
    'interpretation':'Same seed but wall-clock trajectory and asynchronous scans; not a deterministic or paired algorithm comparison.'}
(directory/'summary.json').write_text(json.dumps(summary,indent=2)+chr(10))
print(json.dumps(summary,indent=2))
