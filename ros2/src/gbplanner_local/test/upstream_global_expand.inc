// Copyright (c) 2020 UNR Autonomous Robots Lab. BSD-3-Clause.
// Extracted Vertex overload; see upstream_global_reference.json.
void ReferenceRrg::expandVertex(std::shared_ptr<GraphManager> graph_manager,
                      Vertex& new_vertex, ExpandGraphReport& rep,
                      bool allow_short_edge) {
  // Find nearest neighbour
  StateVec new_state;
  new_state = new_vertex.state;

  Vertex* nearest_vertex = NULL;
  if (!graph_manager->getNearestVertex(&new_state, &nearest_vertex)) {
    rep.status = ExpandGraphStatus::kErrorKdTree;
    return;
  }
  if (nearest_vertex == NULL) {
    rep.status = ExpandGraphStatus::kErrorKdTree;
    return;
  }

  // Check for collision of new connection plus some overshoot distance.
  Eigen::Vector3d origin(nearest_vertex->state[0], nearest_vertex->state[1],
                         nearest_vertex->state[2]);
  Eigen::Vector3d direction(new_state[0] - origin[0], new_state[1] - origin[1],
                            new_state[2] - origin[2]);
  double direction_norm = direction.norm();

  if (direction_norm > planning_params_.edge_length_max) {
    direction = planning_params_.edge_length_max * direction.normalized();
  } else if ((!allow_short_edge) &&
             (direction_norm <= planning_params_.edge_length_min)) {
    // Should not add short edge.
    rep.status = ExpandGraphStatus::kErrorShortEdge;
    return;
  }

  // Recalculate the distance.
  direction_norm = direction.norm();
  new_state[0] = origin[0] + direction[0];
  new_state[1] = origin[1] + direction[1];
  new_state[2] = origin[2] + direction[2];

  

  // Since we are buiding graph,
  // Consider to check the overshoot for both 2 directions except root node.
  Eigen::Vector3d overshoot_vec =
      planning_params_.edge_overshoot * direction.normalized();
  Eigen::Vector3d start_pos = origin + robot_params_.center_offset;
  if (nearest_vertex->id != 0) start_pos = start_pos - overshoot_vec;
  Eigen::Vector3d end_pos =
      origin + robot_params_.center_offset + direction + overshoot_vec;
  

  bool admissible_edge = false;
  int steep_edges = 0;
  std::vector<Eigen::Vector3d> projected_edge;
  if (robot_params_.type == RobotType::kAerialRobot) {
    MapManager::VoxelStatus vs =
        map_manager_->getPathStatus(start_pos, end_pos, robot_box_size_, true);
    if (MapManager::VoxelStatus::kFree == vs) {
      admissible_edge = true;
    }
  }

  if (admissible_edge) {
    Vertex* new_vertex_ptr =
        new Vertex(graph_manager->generateVertexID(), new_state);
    // new_vertex_ptr->id = graph_manager->generateVertexID();
    new_vertex_ptr->state = new_state;
    // Form a tree as the first step.
    new_vertex_ptr->parent = nearest_vertex;
    new_vertex_ptr->distance = nearest_vertex->distance + direction_norm;
    new_vertex_ptr->is_hanging = new_vertex.is_hanging;
    nearest_vertex->children.push_back(new_vertex_ptr);
    graph_manager->addVertex(new_vertex_ptr);
    ++rep.num_vertices_added;
    rep.vertex_added = new_vertex_ptr;
    graph_manager->addEdge(new_vertex_ptr, nearest_vertex, direction_norm);
    ++rep.num_edges_added;
    

    // Form more edges from neighbors if set RRG mode.
    if (planning_params_.rr_mode == RRModeType::kGraph) {
      std::vector<Vertex*> nearest_vertices;
      if (!graph_manager->getNearestVertices(
              &new_state, planning_params_.nearest_range, &nearest_vertices)) {
        rep.status = ExpandGraphStatus::kErrorKdTree;
        return;
      }
      origin << new_vertex_ptr->state[0], new_vertex_ptr->state[1],
          new_vertex_ptr->state[2];
      for (int i = 0; i < nearest_vertices.size(); ++i) {
        direction << nearest_vertices[i]->state[0] - origin[0],
            nearest_vertices[i]->state[1] - origin[1],
            nearest_vertices[i]->state[2] - origin[2];
        double d_norm = direction.norm();

        if ((d_norm > planning_params_.nearest_range_min) &&
            (d_norm < planning_params_.nearest_range_max)) {
          Eigen::Vector3d p_overshoot =
              direction / d_norm * planning_params_.edge_overshoot;
          Eigen::Vector3d p_start =
              origin + robot_params_.center_offset - p_overshoot;
          Eigen::Vector3d p_end =
              origin + robot_params_.center_offset + direction;
          if (nearest_vertices[i]->id != 0) p_end = p_end + p_overshoot;

          bool geofence_pass = true;
          

          if (geofence_pass) {
            admissible_edge = false;
            if (robot_params_.type == RobotType::kAerialRobot) {
              MapManager::VoxelStatus vs = map_manager_->getPathStatus(
                  p_start, p_end, robot_box_size_, true);
              if (MapManager::VoxelStatus::kFree == vs) {
                admissible_edge = true;
              }
            }

            if (admissible_edge) {
              
              graph_manager->addEdge(new_vertex_ptr, nearest_vertices[i],
                                     d_norm);
              ++rep.num_edges_added;
            }
          }
        }
      }
    }

  } else {
    stat_->num_edges_fail++;
    if (stat_->num_edges_fail < 500) {
      std::vector<double> vtmp = {start_pos[0], start_pos[1], start_pos[2],
                                  end_pos[0],   end_pos[1],   end_pos[2]};
      stat_->edges_fail.push_back(vtmp);
    }
    rep.status = ExpandGraphStatus::kErrorCollisionEdge;
    return;
  }
  rep.status = ExpandGraphStatus::kSuccess;
}
