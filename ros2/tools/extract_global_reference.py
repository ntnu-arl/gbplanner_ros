#!/usr/bin/env python3
"""Extract the original Vertex overload used by upstream background expansion."""
from pathlib import Path
import hashlib
import json
from extract_upstream_reference import block_end

root=Path(__file__).resolve().parents[2]
source=(root/'gbplanner/src/rrg.cpp').read_text()
signature='void Rrg::expandGraph(std::shared_ptr<GraphManager> graph_manager,\n                      Vertex& new_vertex'
start=source.index(signature);original=source[start:block_end(source,start)]
function=original;removed=[]
for marker in ['if (robot_params_.type == RobotType::kGroundRobot) {',
               ' else if (robot_params_.type == RobotType::kGroundRobot) {',
               'if (planning_params_.geofence_checking_enable &&',
               'if (local_exploration_ongoing_) {']:
    while marker in function:
        at=function.index(marker);end=block_end(function,at)
        # Match else-if as a complete branch, not only its inner if statement.
        if marker.startswith('if (robot_params_') and function[max(0,at-6):at].endswith('else '):
            at-=6
        removed.append(function[at:end]);function=function[:at]+function[end:]
assert len(removed)==7,len(removed)
function=function.replace('void Rrg::expandGraph','void ReferenceRrg::expandVertex',1)
target=root/'ros2/src/gbplanner_local/test/upstream_global_expand.inc'
target.write_text('// Copyright (c) 2020 UNR Autonomous Robots Lab. BSD-3-Clause.\n'
                  '// Extracted Vertex overload; see upstream_global_reference.json.\n'+function+'\n')
background_start=source.index('void Rrg::expandGlobalGraphTimerCallback(')
background=source[background_start:block_end(source,background_start)]
manifest={'commit':'7301b5350915cf3a4055072ad559842db37b4ca9',
          'vertex_expand_sha256':hashlib.sha256(original.encode()).hexdigest(),
          'background_callback_sha256':hashlib.sha256(background.encode()).hexdigest(),
          'removed_disabled_branches':removed,
          'other_changes':['Rename class/method to ReferenceRrg::expandVertex'],
          'background_port_adaptations':['Seeded mt19937 replaces rand/random_device',
              'Finite sample-attempt budget replaces wall-clock 0.1 second loop',
              'Aerial fixed-cuboid sampling and occupancy backend only; no ground/geofence',
              'Bounded graph size and explicit per-rejection statistics'],
          'test_scope':'Shared aerial expansion vs extracted original Vertex overload; not complete ROS 1 runtime equivalence'}
target.with_name('upstream_global_reference.json').write_text(json.dumps(manifest,indent=2)+chr(10))
print('Extracted Vertex expansion:',len(removed),'disabled ground/geofence/inclination blocks removed')
