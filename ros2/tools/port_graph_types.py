"""Extract graph-only types without the ROS 1 parameter and serialization layer."""
from pathlib import Path
root = Path(__file__).resolve().parents[2]
source = root / 'planner_common/include/planner_common'
target = root / 'ros2/src/gbplanner_graph/include/planner_common'
base = (source / 'graph_base.h').read_text()
base = base[base.index('struct VolumetricGain'):base.index('/* Serialization')]
base = base.replace('geometry_msgs::Point', 'geometry_msgs::msg::Point')
base = base.replace('planner_semantic_msgs::SemanticClass', 'planner_semantic_msgs::msg::SemanticClass')
base = base.replace('::kNone', '::K_NONE')
base = base.replace('num_unknown_surf_voxels(0) {}', 'num_unknown_surf_voxels(0), is_frontier(false) {}')
prefix = '''#pragma once
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <geometry_msgs/msg/point.hpp>
#include <planner_semantic_msgs/msg/semantic_class.hpp>
using StateVec = Eigen::Vector4d;
'''
(target / 'graph_base.h').write_text(prefix + base)
header = (source / 'graph_manager.h').read_text()
for name in ('Edge', 'Graph', 'Vertex'):
    header = header.replace('planner_msgs/' + name + '.h', 'planner_msgs/msg/' + name.lower() + '.hpp')
header = header.replace('#include <tf/transform_datatypes.h>', '#include <memory>')
header = header.replace('#include "planner_common/params.h"', '')
header = header.replace('planner_msgs::Graph', 'planner_msgs::msg::Graph')
header = header.replace('  GraphManager();', '''  GraphManager();
  ~GraphManager();
  GraphManager(const GraphManager&) = delete;
  GraphManager& operator=(const GraphManager&) = delete;''')
header = header.replace('return vertices_map_[id];', 'auto it = vertices_map_.find(id); return it == vertices_map_.end() ? nullptr : it->second;')
(target / 'graph_manager.h').write_text(header)
print('Ported graph types and graph manager declarations')
