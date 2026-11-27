#!/usr/bin/env python3
"""Record fixed upstream mission-time expressions without rewriting originals."""
from pathlib import Path
import hashlib,json
root=Path(__file__).resolve().parents[1]
source=root.parent/'gbplanner/src/rrg.cpp'
text=source.read_text()
functions={}
for name in ['getTimeElapsed','getTimeRemained','isRemainingTimeSufficient','timerCallback']:
    needle='Rrg::'+name+'('
    start=text.index(needle);start=text.rfind('\n',0,start)+1
    brace=text.index('{',start);depth=1;end=brace+1
    while depth:
        if text[end]=='{':depth+=1
        elif text[end]=='}':depth-=1
        end+=1
    body=text[start:end]
    functions[name]={'first_line':text[:start].count('\n')+1,'sha256':hashlib.sha256(body.encode()).hexdigest(),'source':body}
assert text.count('double time_cost = time_to_target + planning_params_.auto_homing_enable')==2
assert 'const double kTimeDelta = 20;' in text
record={'status':'passed','upstream_commit':'7301b5350915cf3a4055072ad559842db37b4ca9',
        'source':'gbplanner/src/rrg.cpp','source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
        'functions':functions,'original_reserve_seconds':20,
        'adaptations':['ROS clock, original node-lifetime start; map reset does not renew budget',
          'Correct automatic-global conditional precedence: (outbound + optional home) / speed',
          'Explicit ignore_time bypasses route feasibility consistently for manual and automatic targets',
          'Stamped battery estimates age, reject invalid/replayed data, expire after configured timeout',
          'Clock rollback latches invalid until node restart',
          'Native K_HOMING is respected by control: execute full route or stop if empty',
          'Aerial automatic landing follows timerCallback: independent node-lifetime deadline, prioritized over automatic homing',
          'PCI confirms trajectory cancellation before one land_srv Empty request; a response is not physical touchdown',
          'The unreachable getBestPath landing branch is not the source of the migrated timer behavior',
          'Enablement is opt-in to preserve earlier deterministic experiments'],
        'verification_scope':'Source hashes and arithmetic boundary/integration tests; not complete ROS 1 execution equivalence'}
output=root/'src/gbplanner_local/mission_provenance.json';output.write_text(json.dumps(record,indent=2)+'\n')
(root/'results/mission_source_verification.json').write_text(json.dumps(record,indent=2)+'\n')
print('Verified 3 mission-time functions, the active landing timer, and both ternary-precedence locations.')
