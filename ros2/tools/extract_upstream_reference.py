#!/usr/bin/env python3
"""Extract the upstream aerial expandGraph for a differential test.

Only remove the disabled ground/geofence branches and rename the class. The
remaining statements are upstream text, independent of the port implementation.
The complete ROS 1 source remains untouched. A manifest records each removal.
"""
from pathlib import Path
import hashlib
import json
import tarfile

ROOT=Path(__file__).resolve().parents[2]
SOURCE=ROOT/'gbplanner/src/rrg.cpp'
TARGET=ROOT/'ros2/src/gbplanner_local/test/upstream_expand.inc'

def block_end(text,start):
    opening=text.index('{',start)
    depth=0
    for index in range(opening,len(text)):
        if text[index]=='{': depth+=1
        elif text[index]=='}':
            depth-=1
            if depth==0: return index+1
    raise ValueError('Unbalanced source block')

def main():
    source=SOURCE.read_text()
    archive=Path('/private/tmp/gbplanner2-7301b535.tar.gz')
    if archive.exists():
        with tarfile.open(archive) as tar:
            member=next(m for m in tar.getmembers() if m.name.endswith('/gbplanner/src/rrg.cpp'))
            assert tar.extractfile(member).read()==SOURCE.read_bytes(), 'Upstream source changed'
    start=source.index('void Rrg::expandGraph(std::shared_ptr<GraphManager> graph_manager,\n                      StateVec& new_state')
    function=source[start:block_end(source,start)]
    original_hash=hashlib.sha256(function.encode()).hexdigest()
    removals=[]
    # Remove the three ground-only branches and the two disabled geofence checks.
    markers=[
        '  if (robot_params_.type == RobotType::kGroundRobot) {',
        ' else if (robot_params_.type == RobotType::kGroundRobot) {',
        '  if (planning_params_.geofence_checking_enable &&',
        '          if (planning_params_.geofence_checking_enable &&',
    ]
    for marker in markers:
        while marker in function:
            at=function.index(marker)
            end=block_end(function,at)
            removals.append(function[at:end])
            function=function[:at]+function[end:]
    assert len(removals)==5, f'Expected five disabled branches, got {len(removals)}'
    function=function.replace('void Rrg::expandGraph','void ReferenceRrg::expandGraph',1)
    header='// Extracted from pinned GBPlanner 2 rrg.cpp, aerial branch; see upstream_reference.json.\n'
    header+='// Copyright (c) 2020 UNR Autonomous Robots Lab. BSD-3-Clause; see LICENSE.\n'
    TARGET.write_text(header+function+'\n')
    manifest={'upstream_commit':'7301b5350915cf3a4055072ad559842db37b4ca9',
              'source':'gbplanner/src/rrg.cpp',
              'source_sha256':hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
              'function_sha256_before_adaptation':original_hash,
              'removed_disabled_branches':removals,
              'other_changes':['rename Rrg to ReferenceRrg'],
              'test_limit':'Same occupancy backend and graph library; no ROS 1 runtime, Voxblox or whole-planner equivalence claim.'}
    TARGET.with_name('upstream_reference.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(f'Extracted reference: {TARGET}; {len(removals)} disabled branches removed')

if __name__=='__main__':main()
