#!/usr/bin/env python3
"""Port pure upstream trajectory/clustering functions, retaining their statements."""
from pathlib import Path
import hashlib
import json
import re
from extract_upstream_reference import block_end

ROOT=Path(__file__).resolve().parents[2]
PACKAGE=ROOT/'ros2/src/gbplanner_local'
source=(ROOT/'planner_common/src/trajectory.cpp').read_text()
signatures=[
    'bool Trajectory::compareTwoTrajectories(const PathType&',
    'double Trajectory::computeDistanceBetweenTwoTrajectories(',
    'double Trajectory::computeDistanceBetweenTrajectoryAndDirection(',
    'void Trajectory::shortenPath(',
    'double Trajectory::getPathLength(const PathType&',
    'bool Trajectory::interpolatePath(const PathType&',
    'double Trajectory::computeDTWDistance(',
]
functions=[]
manifest={'commit':'7301b5350915cf3a4055072ad559842db37b4ca9','functions':[]}
for signature in signatures:
    start=source.index(signature);body=source[start:block_end(source,start)]
    manifest['functions'].append({'signature':signature,'sha256':hashlib.sha256(body.encode()).hexdigest()})
    if signature.startswith('double Trajectory::computeDistanceBetweenTwo'):
        old='if (scale_with_length) dist_ret /= (common_length * common_length);'
        new='''if (scale_with_length) {
    // Degenerate constant paths have zero common length; avoid upstream 0/0.
    if (common_length <= 1e-12)
      return dist_ret <= 1e-12 ? 0.0 : std::numeric_limits<double>::infinity();
    dist_ret /= (common_length * common_length);
  }'''
        assert old in body;body=body.replace(old,new)
    if signature.startswith('double Trajectory::computeDistanceBetweenTrajectoryAndDirection'):
        old='if ((path.size() <= 0) || (discrete_length <= 0))'
        new='if (path.empty() || !std::isfinite(heading) || !std::isfinite(discrete_length) || discrete_length <= 0)'
        assert old in body;body=body.replace(old,new)
        old='  int n = (int)(path_length / discrete_length);'
        new='''  const double steps = path_length / discrete_length;
  const double nearest = std::round(steps);
  // Snap roundoff near positive integer step counts; preserve true floor cases.
  const double tolerance = 8. * std::numeric_limits<double>::epsilon() * std::max(1., steps);
  const int n = static_cast<int>(nearest >= 1. && std::abs(steps - nearest) <= tolerance ? nearest : steps);'''
        assert old in body;body=body.replace(old,new)
        old='    Eigen::Vector3d ei = p0 + uvector * n;'
        new='''    // Preserve upstream trajectory.cpp:93: repeat the forward endpoint (*n),
    // not a reference line sampled using *i.
    Eigen::Vector3d ei = p0 + uvector * n;'''
        assert old in body;body=body.replace(old,new)
    functions.append(body)
rrg=(ROOT/'gbplanner/src/rrg.cpp').read_text()
start=rrg.index('std::vector<int> Rrg::performShortestPathsClustering(')
clustering=rrg[start:block_end(rrg,start)]
manifest['functions'].append({'signature':'Rrg::performShortestPathsClustering','sha256':hashlib.sha256(clustering.encode()).hexdigest()})
clustering=re.sub(r'  ROS_INFO_COND\(.*?;', '', clustering, flags=re.S)
clustering=clustering.replace('Rrg::performShortestPathsClustering','FrontierClusterer::cluster')
clustering=clustering.replace('for (int i = 0; i < vertices.size();', 'for (std::size_t i = 0; i < vertices.size();')
clustering=clustering.replace('for (int j = 0; j < cluster_paths.size();', 'for (std::size_t j = 0; j < cluster_paths.size();')
clustering=clustering.replace('for (int j = 0; j < cluster_paths_refine.size();', 'for (std::size_t j = 0; j < cluster_paths_refine.size();')
manifest['adaptations']=['Remove ROS logging from clustering','Rename clustering class/method; wrap in gbplanner2 namespace',
                        'Use Eigen-only PathType; no ROS geometry overloads','Guard zero common length before normalized DTW division',
                        'Use size_t for vector-index loops to remove signedness warnings',
                        'Preserve repeated direction-reference endpoint (*n, not *i)',
                        'Reject nonfinite heading or discretization in direction distance',
                        'Snap direction-reference step counts within 8 machine epsilons times max(1, count) of a positive integer before truncation; preserve true sub-step/floor cases and original repeated endpoints. Prevent translated paths from receiving infinite DTW distance and zero weighted gain due only to roundoff.']
text='// Copyright (c) 2020 UNR Autonomous Robots Lab. BSD-3-Clause.\n'
text+='// Extracted/adapted by tools/port_global_helpers.py; see global_helpers_provenance.json.\n'
text+='#include "gbplanner_local/trajectory.hpp"\n#include <algorithm>\n#include <cmath>\n#include <limits>\nnamespace gbplanner2 {\n'
text+='\n\n'.join(functions)+'\n'+clustering+'\n}\n'
(PACKAGE/'src/trajectory.cpp').write_text(text)
(PACKAGE/'global_helpers_provenance.json').write_text(json.dumps(manifest,indent=2)+'\n')
print('Extracted seven trajectory helpers and original shortest-path clustering')
