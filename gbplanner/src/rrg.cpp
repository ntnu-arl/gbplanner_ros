#include "gbplanner/rrg.h"

#include <pcl/common/transforms.h>
#include <tf/transform_listener.h>

#define SQ(x) (x * x)

namespace explorer {
namespace gbplanner {

Rrg::Rrg(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
#ifdef USE_OCTOMAP
  map_manager_ = new MapManagerOctomap(nh_, nh_private_);
#else
  map_manager_ =
      new MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>(
          nh_, nh_private_);
#endif
  visualization_ = new Visualization(nh_, nh_private_);

  geofence_manager_.reset(new GeofenceManager());

  // Initialize graphs.
  global_graph_.reset(new GraphManager());

  //
  robot_state_hist_.reset(new RobotStateHistory());

  // Others.
  stat_.reset(new SampleStatistic());
  planner_trigger_time_ = 0;
  current_battery_time_remaining_ = std::numeric_limits<double>::max();
  rostime_start_ = ros::Time::now();
  //
  exploring_direction_ = 0.0;
  periodic_timer_ =
      nh_.createTimer(ros::Duration(kTimerPeriod), &Rrg::timerCallback, this);
  odometry_ready = false;
  last_state_marker_ << 0, 0, 0, 0;
  last_state_marker_global_ << 0, 0, 0, 0;
  robot_backtracking_prev_ = NULL;

  free_cloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("freespace_pointcloud", 10);

  //
  global_graph_update_timer_ =
      nh_.createTimer(ros::Duration(kGlobalGraphUpdateTimerPeriod),
                      &Rrg::expandGlobalGraphTimerCallback, this);

  free_pointcloud_update_timer_ =
      nh_.createTimer(ros::Duration(kFreePointCloudUpdatePeriod),
                      &Rrg::freePointCloudtimerCallback, this);
}

void Rrg::reset() {
  // Reset the local graph.
  local_graph_.reset(new GraphManager());
  local_graph_rep_.reset();

  // Re-initialize data structs.
  stat_.reset(new SampleStatistic());

  // Set state for root/source vertex.
  StateVec root_state;
  if (planning_params_.use_current_state)
    root_state = current_state_;
  else {
    root_state = state_for_planning_;
  }
  stat_->init(root_state);

  // Create a root vertex and add to the graph.
  // Root vertex should be assigned id 0.
  root_vertex_ = new Vertex(local_graph_->generateVertexID(), root_state);
  local_graph_->addVertex(root_vertex_);

  if ((planner_trigger_time_ == 0) && (global_graph_->getNumVertices() == 0)) {
    // First time trigger the planner. Initialize the root for global graph as
    // well.
    Vertex* g_root_vertex =
        new Vertex(global_graph_->generateVertexID(), root_state);
    global_graph_->addVertex(g_root_vertex);
  }

  // First check if this position is free to go.
  MapManager::VoxelStatus voxel_state = map_manager_->getBoxStatus(
      Eigen::Vector3d(root_state[0], root_state[1], root_state[2]) +
          robot_params_.center_offset,
      robot_box_size_, true);
  if (MapManager::VoxelStatus::kFree != voxel_state) {
    switch (voxel_state) {
      case MapManager::VoxelStatus::kFree:
        ROS_INFO("Current box is Free.");
        break;
      case MapManager::VoxelStatus::kOccupied:
        ROS_INFO("Current box contains Occupied voxels.");
        break;
      case MapManager::VoxelStatus::kUnknown:
        ROS_INFO("Current box contains Unknown voxels.");
        break;
    }
    // Assume that even it is not fully free, but safe to clear these voxels.
    ROS_WARN("Starting position is not clear--> clear space around the robot.");
    map_manager_->augmentFreeBox(
        Eigen::Vector3d(root_state[0], root_state[1], root_state[2]) +
            robot_params_.center_offset,
        robot_box_size_);
  }

  // Clear free space before planning.
  if (planning_params_.free_frustum_before_planning) {
    map_manager_->augmentFreeFrustum();
  }
  visualization_->visualizeRobotState(root_vertex_->state, robot_params_);
  visualization_->visualizeSensorFOV(root_vertex_->state, sensor_params_);

  if (planning_params_.type == PlanningModeType::kVerticalExploration) {
    visualization_->visualizeWorkspace(
        root_vertex_->state, global_space_params_, local_vertical_params_);
  } else {
    visualization_->visualizeWorkspace(
        root_vertex_->state, global_space_params_, local_space_params_);
  }
}

void Rrg::clear() {}

bool Rrg::sampleVertex(StateVec& state) {
  bool found = false;
  int while_thres = 1000;  // magic number
  while (!found && while_thres--) {
    random_sampler_.generate(root_vertex_->state, state);
    // Very fast check if the sampled point is inside the planning space.
    // This helps eliminate quickly points outside the sampling space.
    if (state.x() + robot_params_.center_offset.x() <
        global_space_params_.min_val.x() + 0.5 * robot_box_size_.x()) {
      continue;
    } else if (state.y() + robot_params_.center_offset.y() <
               global_space_params_.min_val.y() + 0.5 * robot_box_size_.y()) {
      continue;
    } else if (state.z() + robot_params_.center_offset.z() <
               global_space_params_.min_val.z() + 0.5 * robot_box_size_.z()) {
      continue;
    } else if (state.x() + robot_params_.center_offset.x() >
               global_space_params_.max_val.x() - 0.5 * robot_box_size_.x()) {
      continue;
    } else if (state.y() + robot_params_.center_offset.y() >
               global_space_params_.max_val.y() - 0.5 * robot_box_size_.y()) {
      continue;
    } else if (state.z() + robot_params_.center_offset.z() >
               global_space_params_.max_val.z() - 0.5 * robot_box_size_.z()) {
      continue;
    }

    // Check if in geofence areas.
    if ((planning_params_.geofence_checking_enable) &&
        (GeofenceManager::CoordinateStatus::kViolated ==
         geofence_manager_->getBoxStatus(
             Eigen::Vector2d(state[0] + robot_params_.center_offset[0],
                             state[1] + robot_params_.center_offset[1]),
             Eigen::Vector2d(robot_box_size_[0], robot_box_size_[1]))))
      continue;

    // Check if surrounding area is free.
    if (MapManager::VoxelStatus::kFree ==
        map_manager_->getBoxStatus(
            Eigen::Vector3d(state[0], state[1], state[2]) +
                robot_params_.center_offset,
            robot_box_size_, true)) {
      random_sampler_.pushSample(state, true);  // for debug purpose.
      found = true;
    } else {
      stat_->num_vertices_fail++;
      random_sampler_.pushSample(state, false);
    }
  }
  return found;
}

bool Rrg::sampleVertex(RandomSampler& random_sampler, StateVec& root_state,
                       StateVec& sampled_state) {
  bool found = false;
  int while_thres = 1000;  // magic number.
  while (!found && while_thres--) {
    random_sampler.generate(root_state, sampled_state);
    // Very fast check if the sampled point is inside the planning space.
    // This helps eliminate quickly points outside the sampling space.
    if (sampled_state.x() + robot_params_.center_offset.x() <
        global_space_params_.min_val.x() + 0.5 * robot_box_size_.x()) {
      continue;
    } else if (sampled_state.y() + robot_params_.center_offset.y() <
               global_space_params_.min_val.y() + 0.5 * robot_box_size_.y()) {
      continue;
    } else if (sampled_state.z() + robot_params_.center_offset.z() <
               global_space_params_.min_val.z() + 0.5 * robot_box_size_.z()) {
      continue;
    } else if (sampled_state.x() + robot_params_.center_offset.x() >
               global_space_params_.max_val.x() - 0.5 * robot_box_size_.x()) {
      continue;
    } else if (sampled_state.y() + robot_params_.center_offset.y() >
               global_space_params_.max_val.y() - 0.5 * robot_box_size_.y()) {
      continue;
    } else if (sampled_state.z() + robot_params_.center_offset.z() >
               global_space_params_.max_val.z() - 0.5 * robot_box_size_.z()) {
      continue;
    }
    // Check if surrounding area is free.
    if (MapManager::VoxelStatus::kFree == map_manager_->getBoxStatus(
        Eigen::Vector3d(sampled_state[0], sampled_state[1], sampled_state[2]) +
                robot_params_.center_offset, robot_box_size_, true)) {
      random_sampler.pushSample(sampled_state, true);  // for debug purpose.
      found = true;
    } else {
      stat_->num_vertices_fail++;
      random_sampler.pushSample(sampled_state, false);
    }
  }
  return found;
}

void Rrg::expandTreeStar(std::shared_ptr<GraphManager> graph_manager,
                         StateVec& new_state, ExpandGraphReport& rep) {
  // Find nearest neighbour
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
  } else if (direction_norm <= planning_params_.edge_length_min) {
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

  if (planning_params_.geofence_checking_enable &&
      (GeofenceManager::CoordinateStatus::kViolated ==
       geofence_manager_->getPathStatus(
           Eigen::Vector2d(start_pos[0], start_pos[1]),
           Eigen::Vector2d(end_pos[0], end_pos[1]),
           Eigen::Vector2d(robot_box_size_[0], robot_box_size_[1])))) {
    rep.status = ExpandGraphStatus::kErrorGeofenceViolated;
    return;
  }

  bool connected_to_root = false;
  if (MapManager::VoxelStatus::kFree ==
      map_manager_->getPathStatus(start_pos, end_pos, robot_box_size_, true)) {
    // Obstacle free.
    // Re-wire the shortest one first.
    std::vector<Vertex*> nearest_vertices;
    if (!graph_manager->getNearestVertices(
            &new_state, planning_params_.nearest_range, &nearest_vertices)) {
      rep.status = ExpandGraphStatus::kErrorKdTree;
      return;
    }

    // To save the computation from collision checking, we save feasible list
    // from this first step.
    std::vector<Vertex*> feasible_neigbors;

    Vertex* v_min = nearest_vertex;
    double c_min = v_min->distance + direction_norm;
    origin << new_state[0], new_state[1], new_state[2];

    for (int i = 0; i < nearest_vertices.size(); ++i) {
      direction << nearest_vertices[i]->state[0] - new_state[0],
          nearest_vertices[i]->state[1] - new_state[1],
          nearest_vertices[i]->state[2] - new_state[2];
      double d_norm = direction.norm();
      if (d_norm == 0.0) continue;

      Eigen::Vector3d p_start = origin + robot_params_.center_offset;
      Eigen::Vector3d p_end = origin + robot_params_.center_offset + direction;

      bool geofence_pass = true;
      if (planning_params_.geofence_checking_enable &&
          (GeofenceManager::CoordinateStatus::kViolated ==
           geofence_manager_->getPathStatus(
               Eigen::Vector2d(p_start[0], p_start[1]),
               Eigen::Vector2d(p_end[0], p_end[1]),
               Eigen::Vector2d(robot_box_size_[0], robot_box_size_[1])))) {
        geofence_pass = false;
      }

      if (geofence_pass) {
        if (MapManager::VoxelStatus::kFree ==
            map_manager_->getPathStatus(p_start, p_end, robot_box_size_,
                                        true)) {
          feasible_neigbors.push_back(nearest_vertices[i]);
          double cost_tmp = d_norm + nearest_vertices[i]->distance;
          if (cost_tmp < c_min) {
            v_min = nearest_vertices[i];
            c_min = cost_tmp;
          }
        }
      }
    }
    // Add the vertex with shortest distance to the tree (graph).
    Vertex* new_vertex =
        new Vertex(graph_manager->generateVertexID(), new_state);
    new_vertex->parent = v_min;
    new_vertex->distance = c_min;
    v_min->children.push_back(new_vertex);
    graph_manager->addVertex(new_vertex);
    ++rep.num_vertices_added;
    rep.vertex_added = new_vertex;
    graph_manager->addEdge(new_vertex, v_min, c_min - v_min->distance);
    ++rep.num_edges_added;

    // Rewire neigbor nodes through newly added vertex if found shorter path.
    origin << new_vertex->state[0], new_vertex->state[1], new_vertex->state[2];
    for (auto& near_vertex : feasible_neigbors) {
      direction << near_vertex->state[0] - new_vertex->state[0],
          near_vertex->state[1] - new_vertex->state[1],
          near_vertex->state[2] - new_vertex->state[2];
      double d_norm = direction.norm();
      if (d_norm == 0.0) continue;

      double cost_tmp = d_norm + new_vertex->distance;
      if (near_vertex->distance > cost_tmp) {
        graph_manager->removeEdge(near_vertex, near_vertex->parent);
        near_vertex->distance = cost_tmp;
        near_vertex->parent = new_vertex;
        graph_manager->addEdge(near_vertex, near_vertex->parent, d_norm);
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

void Rrg::expandGraph(std::shared_ptr<GraphManager> graph_manager,
                      StateVec& new_state, ExpandGraphReport& rep,
                      bool allow_short_edge) {
  // Find nearest neighbour
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

  if (planning_params_.geofence_checking_enable &&
      (GeofenceManager::CoordinateStatus::kViolated ==
       geofence_manager_->getPathStatus(
           Eigen::Vector2d(start_pos[0], start_pos[1]),
           Eigen::Vector2d(end_pos[0], end_pos[1]),
           Eigen::Vector2d(robot_box_size_[0], robot_box_size_[1])))) {
    rep.status = ExpandGraphStatus::kErrorGeofenceViolated;
    return;
  }

  bool connected_to_root = false;
  if (MapManager::VoxelStatus::kFree ==
      map_manager_->getPathStatus(start_pos, end_pos, robot_box_size_, true)) {
    Vertex* new_vertex =
        new Vertex(graph_manager->generateVertexID(), new_state);
    // Form a tree as the first step.
    if (nearest_vertex->id == 0) connected_to_root = true;
    new_vertex->parent = nearest_vertex;
    new_vertex->distance = nearest_vertex->distance + direction_norm;
    nearest_vertex->children.push_back(new_vertex);
    graph_manager->addVertex(new_vertex);
    ++rep.num_vertices_added;
    rep.vertex_added = new_vertex;
    graph_manager->addEdge(new_vertex, nearest_vertex, direction_norm);
    ++rep.num_edges_added;
    // Form more edges from neighbors if set RRG mode.
    if (planning_params_.rr_mode == explorer::RRModeType::kGraph) {
      std::vector<Vertex*> nearest_vertices;
      if (!graph_manager->getNearestVertices(
              &new_state, planning_params_.nearest_range, &nearest_vertices)) {
        rep.status = ExpandGraphStatus::kErrorKdTree;
        return;
      }
      origin << new_vertex->state[0],new_vertex->state[1],new_vertex->state[2];
      for (int i = 0; i < nearest_vertices.size(); ++i) {
        if (nearest_vertices[i]->id == 0) connected_to_root = true;
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
          if (planning_params_.geofence_checking_enable &&
              (GeofenceManager::CoordinateStatus::kViolated ==
               geofence_manager_->getPathStatus(
                   Eigen::Vector2d(p_start[0], p_start[1]),
                   Eigen::Vector2d(p_end[0], p_end[1]),
                   Eigen::Vector2d(robot_box_size_[0], robot_box_size_[1])))) {
            geofence_pass = false;
          }

          if (geofence_pass) {
            if (MapManager::VoxelStatus::kFree ==
                map_manager_->getPathStatus(p_start, p_end, robot_box_size_,
                                            true)) {
              graph_manager->addEdge(new_vertex, nearest_vertices[i], d_norm);
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

void Rrg::expandGraphEdges(std::shared_ptr<GraphManager> graph_manager,
                           Vertex* new_vertex, ExpandGraphReport& rep) {
  std::vector<Vertex*> nearest_vertices;
  if (!graph_manager->getNearestVertices(&(new_vertex->state),
                                         planning_params_.nearest_range,
                                         &nearest_vertices)) {
    rep.status = ExpandGraphStatus::kErrorKdTree;
    return;
  }
  Eigen::Vector3d origin;
  origin << new_vertex->state[0], new_vertex->state[1], new_vertex->state[2];
  for (int i = 0; i < nearest_vertices.size(); ++i) {
    Eigen::Vector3d direction;
    direction << nearest_vertices[i]->state[0] - origin[0],
        nearest_vertices[i]->state[1] - origin[1],
        nearest_vertices[i]->state[2] - origin[2];
    double d_norm = direction.norm();
    if ((d_norm > planning_params_.edge_length_min) &&
        (d_norm < planning_params_.edge_length_max)) {
      Eigen::Vector3d p_overshoot =
          direction / d_norm * planning_params_.edge_overshoot;
      Eigen::Vector3d p_start =
          origin + robot_params_.center_offset - p_overshoot;
      Eigen::Vector3d p_end = origin + robot_params_.center_offset + direction;
      if (nearest_vertices[i]->id != 0) p_end = p_end + p_overshoot;
      if (MapManager::VoxelStatus::kFree ==
          map_manager_->getPathStatus(p_start, p_end, robot_box_size_, true)) {
        graph_manager->addEdge(new_vertex, nearest_vertices[i], d_norm);
        ++rep.num_edges_added;
      }
    }
  }
  rep.status = ExpandGraphStatus::kSuccess;
}

void Rrg::expandGraphEdgesBlindly(std::shared_ptr<GraphManager> graph_manager,
                                  Vertex* new_vertex, double radius,
                                  ExpandGraphReport& rep) {
  std::vector<Vertex*> nearest_vertices;
  if (!graph_manager->getNearestVertices(&(new_vertex->state), radius,
                                         &nearest_vertices)) {
    rep.status = ExpandGraphStatus::kNull;
    return;
  }
  Eigen::Vector3d origin;
  origin << new_vertex->state[0], new_vertex->state[1], new_vertex->state[2];
  for (int i = 0; i < nearest_vertices.size(); ++i) {
    Eigen::Vector3d direction;
    direction << nearest_vertices[i]->state[0] - origin[0],
        nearest_vertices[i]->state[1] - origin[1],
        nearest_vertices[i]->state[2] - origin[2];
    double d_norm = direction.norm();
    graph_manager->addEdge(new_vertex, nearest_vertices[i], d_norm);
    ++rep.num_edges_added;
  }
  rep.status = ExpandGraphStatus::kSuccess;
}

Rrg::GraphStatus Rrg::buildGraph() {
  START_TIMER(ttime);
  int loop_count = 0;
  int num_vertices = 1;
  int num_edges = 0;
  if (planning_params_.type == PlanningModeType::kVerticalExploration)
    random_sampler_vertical_.reset();
  else
    random_sampler_.reset();

  while ((loop_count++ < planning_params_.num_loops_max) &&
         (num_vertices < planning_num_vertices_max_) &&
         (num_edges < planning_num_edges_max_)) {
    StateVec new_state;
    if (planning_params_.type == PlanningModeType::kVerticalExploration) {
      if (!sampleVertex(random_sampler_vertical_, root_vertex_->state,
                        new_state))
        continue;
    } else {
      if (!sampleVertex(new_state)) continue;
    }

    ExpandGraphReport rep;
    expandGraph(local_graph_, new_state, rep);
    // expandTreeStar(local_graph_, new_state, rep);
    if (rep.status == ExpandGraphStatus::kSuccess) {
      num_vertices += rep.num_vertices_added;
      num_edges += rep.num_edges_added;
    }

    if ((loop_count >= planning_params_.num_loops_cutoff) &&
        (local_graph_->getNumVertices() <= 1)) {
      break;
    }
  }
  stat_->build_graph_time = GET_ELAPSED_TIME(ttime);

  // Visualize geofence area.
  if (planning_params_.geofence_checking_enable)
    visualization_->visualizeGeofence(geofence_manager_);

  planner_trigger_time_++;
  ROS_INFO("Formed a graph with [%d] vertices and [%d] edges with [%d] loops",
           num_vertices, num_edges, loop_count);

  if (planning_params_.type == PlanningModeType::kVerticalExploration)
    visualization_->visualizeSampler(random_sampler_vertical_);
  else
    visualization_->visualizeSampler(random_sampler_);

  if (local_graph_->getNumVertices() > 1) {
    visualization_->visualizeGraph(local_graph_);
    return Rrg::GraphStatus::OK;
  } else {
    visualization_->visualizeFailedEdges(stat_);
    ROS_INFO("Number of failed samples: [%d] vertices and [%d] edges",
             stat_->num_vertices_fail, stat_->num_edges_fail);
    return Rrg::GraphStatus::ERR_NO_FEASIBLE_PATH;
  }
}

void Rrg::correctYaw() {
  // Choose the heading angle tangent with the moving direction.
  if (planning_params_.yaw_tangent_correction) {
    int num_vertices = local_graph_->getNumVertices();
    for (int id = 1; id < num_vertices; ++id) {
      int pid = local_graph_->getParentIDFromShortestPath(id, local_graph_rep_);
      Vertex* v = local_graph_->getVertex(id);
      Vertex* vp = local_graph_->getVertex(pid);
      Eigen::Vector3d vec(v->state[0] - vp->state[0],
                          v->state[1] - vp->state[1],
                          v->state[2] - vp->state[2]);
      if (planning_params_.planning_backward) vec = -vec;
      v->state[3] = std::atan2(vec[1], vec[0]);
    }
  }
}

Rrg::GraphStatus Rrg::evaluateGraph() {
  START_TIMER(ttime);
  Rrg::GraphStatus gstatus = Rrg::GraphStatus::OK;
  // Dijkstra and mark leaf vertices.
  local_graph_->findShortestPaths(local_graph_rep_);
  local_graph_->findLeafVertices(local_graph_rep_);
  std::vector<Vertex*> leaf_vertices;
  local_graph_->getLeafVertices(leaf_vertices);
  stat_->shortest_path_time = GET_ELAPSED_TIME(ttime);

  // Yaw correction AFTER having shortest paths.
  correctYaw();

  // Gain calculation for each vertex.
  computeExplorationGain();

  // Gain evaluation for valid paths, starting from the leaf to the root.
  START_TIMER(ttime);
  double best_gain = 0;
  int best_path_id = 0;
  int num_leaf_vertices = leaf_vertices.size();
  for (int i = 0; i < num_leaf_vertices; ++i) {
    int id = leaf_vertices[i]->id;
    std::vector<Vertex*> path;
    local_graph_->getShortestPath(id, local_graph_rep_, true, path);
    int path_size = path.size();
    if (path_size > 1) {
      // At least 2 vertices: root + leaf.
      double path_gain = 0;
      double lambda = planning_params_.path_length_penalty;
      for (int ind = 0; ind < path_size; ++ind) {
        path.pop_back();
        Vertex* v_id = path[ind];
        double path_length =
            local_graph_->getShortestDistance(v_id->id, local_graph_rep_);
        path_gain += v_id->vol_gain.gain * exp(-lambda * path_length);
        v_id->vol_gain.accumulative_gain = path_gain;
      }
      // Compare with exploring direction to penalty not-forward paths.
      if (planning_params_.type != PlanningModeType::kVerticalExploration) {
        double lambda2 = planning_params_.path_direction_penalty;
        std::vector<Eigen::Vector3d> path_list;
        local_graph_->getShortestPath(id, local_graph_rep_, true, path_list);
        double fw_ratio =
            Trajectory::computeDistanceBetweenTrajectoryAndDirection(
                path_list, exploring_direction_, 0.2, true);
        path_gain *= exp(-lambda2 * fw_ratio);
      }

      if (path_gain > best_gain) {
        best_gain = path_gain;
        best_path_id = id;
      }
    }
  }

  // Visualization at the end.
  visualization_->visualizeShortestPaths(local_graph_, local_graph_rep_);
  if (best_gain > 0) {
    visualization_->visualizeBestPaths(local_graph_, local_graph_rep_, 10,
                                       best_path_id);
  }

  if (best_gain > 0) {
    // create a branch
    std::vector<int> path;
    local_graph_->getShortestPath(best_path_id, local_graph_rep_, false, path);
    for (int i = 0; i < (path.size() - 1); ++i) {
      local_graph_->getVertex(path[i])->parent =
          local_graph_->getVertex(path[i + 1]);
    }
    best_vertex_ = local_graph_->getVertex(path[0]);
    //
    ROS_INFO("Best path: with gain [%f] and ID [%d] ", best_gain, best_path_id);
    gstatus = Rrg::GraphStatus::OK;

    addFrontiers(best_path_id);
    visualization_->visualizeGlobalGraph(global_graph_);
  } else {
    gstatus = Rrg::GraphStatus::NO_GAIN;
  }
  stat_->evaluate_graph_time = GET_ELAPSED_TIME(ttime);
  stat_->printTime();
  return gstatus;
}

bool Rrg::modifyPath(pcl::PointCloud<pcl::PointXYZ>* obstacle_pcl,
                     Eigen::Vector3d& p0, Eigen::Vector3d& p1,
                     Eigen::Vector3d& p1_mod) {
  p1_mod = p1;
  Eigen::Vector3d p_center;
  p_center = (p0 + p1) / 2.0;
  Eigen::Vector3d p_dir;
  p_dir = (p1 - p0);
  double radius = p_dir.norm() / 2.0;
  Eigen::Vector3d x_axis(1.0, 0.0, 0.0);
  Eigen::Quaternion<double> quat_W2S;
  Eigen::Vector3d p_dir_norm = p_dir.normalized();
  double yaw_angle = std::atan2(p_dir_norm.y(), p_dir_norm.x());
  double pitch_angle = -std::atan2(p_dir_norm.z(),
                        std::sqrt(p_dir_norm.x() * p_dir_norm.x() +
                        p_dir_norm.y() * p_dir_norm.y()));
  quat_W2S = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(pitch_angle, Eigen::Vector3d::UnitY());

  pcl::PointCloud<pcl::PointXYZ>* pcl_tf(new pcl::PointCloud<pcl::PointXYZ>());
  Eigen::Translation<double, 3> trans_W2S(p_center);
  Eigen::Transform<double, 3, Eigen::Affine> tf_W2S(trans_W2S * quat_W2S);
  pcl::transformPointCloud(*obstacle_pcl, *pcl_tf, tf_W2S.inverse());

  // Add a local bounding box
  double kDx = robot_params_.safety_extension[0];
  double kDy = robot_params_.safety_extension[1];
  double kDz = robot_params_.safety_extension[2];

  // 6 rectanges in form:  ax+by+cz = 1
  std::vector<Eigen::Vector3d> u_l;
  std::vector<Eigen::Vector3d> p_l;
  u_l.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
  u_l.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
  u_l.push_back(Eigen::Vector3d(0.0, 1.0, 0.0));
  u_l.push_back(Eigen::Vector3d(0.0, 1.0, 0.0));
  u_l.push_back(Eigen::Vector3d(0.0, 0.0, 1.0));
  u_l.push_back(Eigen::Vector3d(0.0, 0.0, 1.0));
  p_l.push_back(Eigen::Vector3d(-radius - kDx, 0.0, 0.0));
  p_l.push_back(Eigen::Vector3d(radius + kDx, 0.0, 0.0));
  p_l.push_back(Eigen::Vector3d(0.0, -kDy, 0.0));
  p_l.push_back(Eigen::Vector3d(0.0, kDy, 0.0));
  p_l.push_back(Eigen::Vector3d(0.0, 0.0, -kDz));
  p_l.push_back(Eigen::Vector3d(0.0, 0.0, kDz));
  std::vector<Eigen::Vector3d> hyperplane_list;
  std::vector<Eigen::Vector3d> tangent_point_list;
  for (int i = 0; i < 6; ++i) {
    Eigen::Vector3d a_l;
    a_l = u_l[i] / (u_l[i].dot(p_l[i]));
    tangent_point_list.push_back(p_l[i]);
    hyperplane_list.push_back(a_l);
  }

  // Keep points inside the local box only
  pcl::PointCloud<pcl::PointXYZ>* pcl_in_box(
      new pcl::PointCloud<pcl::PointXYZ>());
  for (auto p = pcl_tf->begin(); p != pcl_tf->end(); ++p) {
    // Check all 6 hyperplanes
    const double kDSign = 0.05;  // numeric issue
    double sign;
    int i = 0;
    for (i = 0; i < 6; ++i) {
      sign = p->x * hyperplane_list[i].x() + p->y * hyperplane_list[i].y() +
             p->z * hyperplane_list[i].z() - 1;
      if (sign > kDSign) break;
    }
    if (i == 6) {
      // inside the local box
      pcl_in_box->push_back(*p);
    }
  }
  if (pcl_in_box->size())
    pcl::copyPointCloud(*pcl_in_box, *pcl_tf);
  else {
    // full free space --> keep current vertex.
    return true;
  }

  // Find closest point
  double dist_min_sq = std::numeric_limits<double>::max();
  Eigen::Vector3d p_tangent;
  for (auto p = pcl_tf->begin(); p != pcl_tf->end(); ++p) {
    double dist_t = p->x * p->x + p->y * p->y + p->z * p->z;
    if (dist_t < dist_min_sq) {
      dist_min_sq = dist_t;
      p_tangent << p->x, p->y, p->z;
    }
  }

  const double kDDist = 0.01;  // deal with numeric error.
  if ((dist_min_sq == std::numeric_limits<double>::max()) ||
      (dist_min_sq < kDDist)) {
    // the path is too close to obstacle.
    return false;
  }

  double a = radius, b = radius, c = radius;  // dimensions of the ellipsoid.
  // Check if we need to adjust the sphere to ellipsoid.
  if (dist_min_sq < (radius * radius)) {
    // Reduce other axes
    b = std::sqrt(
        (p_tangent.y() * p_tangent.y() + p_tangent.z() * p_tangent.z()) /
        (1 - p_tangent.x() * p_tangent.x() / (a * a)));
    c = b;  // Set equal b for now; but could increase.???
    // Fit the first hyperplane: x x_l + y y_l + z z_l = 1
    Eigen::Vector3d hyperplane_last =
        Eigen::Vector3d(p_tangent.x() / (a * a), p_tangent.y() / (b * b),
                        p_tangent.z() / (c * c));
    hyperplane_list.push_back(hyperplane_last);
    tangent_point_list.push_back(p_tangent);
  }

  // Increase the ellipsoid and repeat.
  bool stop = false;
  int n_max = 0;  // magic number: max 50 hyperplanes
  while ((!stop) && (n_max < 50)) {
    ++n_max;
    pcl::PointCloud<pcl::PointXYZ>* pcl_reduced(
        new pcl::PointCloud<pcl::PointXYZ>());
    // Also re-scale each dimension followed the dimentions of ellipsoid
    if (hyperplane_list.size()) {
      Eigen::Vector3d hyperplane_last;
      hyperplane_last = hyperplane_list.back();
      // Reduce point: keep points on the same side with zero origin (sign < 0)
      for (auto p = pcl_tf->begin(); p != pcl_tf->end(); ++p) {
        double sign = p->x * hyperplane_last.x() + p->y * hyperplane_last.y() +
                      p->z * hyperplane_last.z() - 1;
        const double kDSign = -0.05;  // numeric issue
        if (sign < kDSign) {
          // same side with the ellipsoid.
          pcl_reduced->push_back(*p);
        }
      }
    } else {
      pcl::copyPointCloud(*pcl_tf, *pcl_reduced);
    }

    Eigen::Vector3d p_tangent1;
    dist_min_sq = std::numeric_limits<double>::max();
    for (auto p = pcl_reduced->begin(); p != pcl_reduced->end(); ++p) {
      // Scale to get next closest point.
      pcl::PointXYZ pv;
      pv.x = p->x / a;
      pv.y = p->y / b;
      pv.z = p->z / c;
      double dist_t = pv.x * pv.x + pv.y * pv.y + pv.z * pv.z;
      if (dist_t < dist_min_sq) {
        dist_min_sq = dist_t;
        p_tangent1 << p->x, p->y, p->z;
      }
    }
    if ((pcl_reduced->size() == 0) ||
        (dist_min_sq == std::numeric_limits<double>::max())) {
      stop = true;
    } else {
      double e_ext = dist_min_sq;
      Eigen::Vector3d hyperplane_new = Eigen::Vector3d(
          p_tangent1.x() / (a * a * e_ext), p_tangent1.y() / (b * b * e_ext),
          p_tangent1.z() / (c * c * e_ext));
      hyperplane_list.push_back(hyperplane_new);
      tangent_point_list.push_back(p_tangent1);
      pcl_tf->clear();
      pcl::copyPointCloud(*pcl_reduced, *pcl_tf);
    }
  }
  if (!stop) {
    // Require too many hyperplanes
    return false;
  }

  // Find the polygon formed from intersections between the bisector plane vs.
  // all hyperplanes, Not sure how to get the closed-form solution, also issue
  // with unknown voxels
  //  --> get average from uniform sampling on the y-z plane (body coordinate)
  std::vector<Eigen::Vector3d> feasible_samples;
  for (double dy = -kDy; dy < kDy; dy += 0.1) {
    for (double dz = -kDz; dz < kDz; dz += 0.1) {
      Eigen::Vector3d p(radius, dy, dz);
      // check if this is inside all hyperplanes.
      const double kDSign = -0.05;  // numeric issue
      double sign;
      int i = 0;
      for (i = 0; i < hyperplane_list.size(); ++i) {
        sign = p.x() * hyperplane_list[i].x() + p.y() * hyperplane_list[i].y() +
               p.z() * hyperplane_list[i].z() - 1;
        if (sign > kDSign) break;
      }
      if (i == hyperplane_list.size()) {
        feasible_samples.push_back(p);
      }
    }
  }

  for (int i = 0; i < hyperplane_list.size(); ++i) {
    tangent_point_list[i] =
        tf_W2S * tangent_point_list[i];  // convert back to world
    Eigen::Matrix4d tf_inv_T = tf_W2S.matrix().inverse().transpose();
    Eigen::Vector4d v_t;
    v_t = tf_inv_T * Eigen::Vector4d(hyperplane_list[i].x(),
                                     hyperplane_list[i].y(),
                                     hyperplane_list[i].z(), -1.0);
    v_t = v_t / (-v_t[3]);
    hyperplane_list[i] << v_t.x(), v_t.y(), v_t.z();
  }

  p1_mod << 0.0, 0.0, 0.0;
  int feasible_count = 0;
  for (int i = 0; i < feasible_samples.size(); ++i) {
    feasible_samples[i] =
        tf_W2S * feasible_samples[i];  // convert back to world
    // check if this is free voxel to deal with occluded area.
    if (map_manager_->getVoxelStatus(feasible_samples[i]) ==
        MapManager::VoxelStatus::kFree) {
      feasible_corridor_pcl_->push_back(pcl::PointXYZ(feasible_samples[i].x(),
                                                      feasible_samples[i].y(),
                                                      feasible_samples[i].z()));
      p1_mod = p1_mod + feasible_samples[i];
      ++feasible_count;
    }
  }

  if (feasible_count) {
    p1_mod = p1_mod / feasible_count;
  } else {
    return false;
  }

  visualization_->visualizeHyperplanes(p_center, hyperplane_list,
                                       tangent_point_list);
  return true;
}

void Rrg::addFrontiers(int best_vertex_id) {
  // Add frontiers to the graph.
  // 1) Check and mark if any vertex is potential frontier and leaf vertices.
  // This should be done in buildGraph step, but for now, put everything here to
  // test --> move to the expandGlobalGraph.
  // 2) Re-update all previous frontiers in graph if they are still
  // frontiers by checking if the are surrounded by normal vertices in local
  // graph, change the status to normal.
  // 3) Sort all the path with frontiers into desending list.
  // 4) For each path, check if the frontier is surrounded by normal vertices
  // or any frontiers. If yes, don't add this path;
  // otherwise, add this path to the global graph.

  ROS_INFO("Global graph: %d vertices, %d edges.",
           global_graph_->getNumVertices(), global_graph_->getNumEdges());
  bool update_global_frontiers = true;
  if (update_global_frontiers) {
    std::vector<Vertex*> global_frontiers;
    int num_vertices = global_graph_->getNumVertices();
    for (int id = 0; id < num_vertices; ++id) {
      if (global_graph_->getVertex(id)->type == VertexType::kFrontier) {
        global_frontiers.push_back(global_graph_->getVertex(id));
      }
    }
    ROS_INFO("Have %d frontiers from global graph.",
             (int)global_frontiers.size());
    for (auto& v : global_frontiers) {
      computeVolumetricGainRayModelNoBound(v->state, v->vol_gain);
      if (!v->vol_gain.is_frontier) v->type = VertexType::kUnvisited;
    }
  }

  // Get all potential frontiers at leaf vertices of newly sampled local graph.
  std::vector<Vertex*> leaf_vertices;
  local_graph_->getLeafVertices(leaf_vertices);
  std::vector<Vertex*> frontier_vertices;
  for (auto& v : leaf_vertices) {
    if (v->type == VertexType::kFrontier) {
      frontier_vertices.push_back(v);
    }
  }
  ROS_INFO("Get %d leaf vertices from newly local graph.",
           (int)leaf_vertices.size());
  ROS_INFO("Get %d frontiers from newly local graph.",
           (int)frontier_vertices.size());

  // Clustering the frontier and add principle path to the global.
  std::vector<int> cluster_ids = performShortestPathsClustering(
      local_graph_, local_graph_rep_, frontier_vertices);
  visualization_->visualizeClusteredPaths(local_graph_, local_graph_rep_,
                                          frontier_vertices, cluster_ids);
  const double kRangeCheck = 1.0;
  const double kUpdateRadius = 3.0;
  for (int i = 0; i < cluster_ids.size(); ++i) {
    Vertex* nearest_vertex = NULL;
    // To add principal path, verify if around that area already have vertices
    // before. Also if the robot already passed that area before.
    if (!global_graph_->getNearestVertexInRange(
            &(local_graph_->getVertex(cluster_ids[i])->state), kRangeCheck,
            &nearest_vertex)) {
      StateVec* nearest_state = NULL;
      if (!robot_state_hist_->getNearestStateInRange(
              &(local_graph_->getVertex(cluster_ids[i])->state), kUpdateRadius,
              &nearest_state)) {
        std::vector<Vertex*> path;
        local_graph_->getShortestPath(cluster_ids[i], local_graph_rep_, true,
                                      path);
        // Only keep frontier for the leaf vertex, the remaining should be
        // cleared to normal.
        for (auto pa = path.begin(); pa != (path.end() - 1); ++pa) {
          (*pa)->type = VertexType::kUnvisited;
        }
        addRefPathToGraph(global_graph_, path);
      }
    }
  }
  visualization_->visualizeRobotStateHistory(robot_state_hist_->state_hist_);
}

void Rrg::freePointCloudtimerCallback(const ros::TimerEvent& event) {
  if (!planning_params_.freespace_cloud_enable) return;

  pcl::PointCloud<pcl::PointXYZ>::Ptr free_cloud_body(
      new pcl::PointCloud<pcl::PointXYZ>);

  std::vector<Eigen::Vector3d> multiray_endpoints_body;
  for (auto sensor_name : free_frustum_params_.sensor_list) {
    StateVec state;
    state[0] = current_state_[0];
    state[1] = current_state_[1];
    state[2] = current_state_[2];
    state[3] = current_state_[3];
    // get frustum endpoints (They are in world frame)
    free_frustum_params_.sensor[sensor_name].getFrustumEndpoints(
        state, multiray_endpoints_body);
    std::vector<Eigen::Vector3d> multiray_endpoints;
    // Check it the full ray till max range is free(for voxblox only, for
    // octomap just convert to world frame)
    map_manager_->getFreeSpacePointCloud(multiray_endpoints_body, state,
                                         free_cloud_body);
    // convert the endpoint to sensor frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr free_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    free_frustum_params_.sensor[sensor_name].convertBodyToSensor(
        free_cloud_body, free_cloud);

    sensor_msgs::PointCloud2 out_cloud;
    pcl::toROSMsg(*free_cloud.get(), out_cloud);
    out_cloud.header.frame_id =
        free_frustum_params_.sensor[sensor_name].frame_id;
    out_cloud.header.stamp = ros::Time::now();
    free_cloud_pub_.publish(out_cloud);
  }
}

void Rrg::expandGlobalGraphTimerCallback(const ros::TimerEvent& event) {
  // Algorithm:
  // Extract unvisited vertices in the global graph.
  // Randomly choose a vertex then group all nearby vertices within a local
  // bounding box. Repeat again until having set of local bounding box covered
  // all unvisited vertices. Random sample a collision free vertex inside a
  // local box, expand the graph, and compute the volumetric gain to check if
  // this is frontier

  ros::Time time_lim;
  START_TIMER(time_lim);

  // ROS_INFO("graphTimerCallback is triggered.");
  if (planner_trigger_time_ == 0) return;

  bool update_global_frontiers = false;
  if (update_global_frontiers) {
    std::vector<Vertex*> global_frontiers;
    int num_vertices = global_graph_->getNumVertices();
    for (int id = 0; id < num_vertices; ++id) {
      if (global_graph_->getVertex(id)->type == VertexType::kFrontier) {
        global_frontiers.push_back(global_graph_->getVertex(id));
      }
    }
    for (auto& v : global_frontiers) {
      // computeVolumetricGainRayModelNoBound(v->state, v->vol_gain);
      computeVolumetricGainRayModel(v->state, v->vol_gain);
      if (!v->vol_gain.is_frontier) v->type = VertexType::kUnvisited;
    }
  }

  std::vector<Vertex*> unvisited_vertices;
  int global_graph_size = global_graph_->getNumVertices();
  for (int id = 0; id < global_graph_size; ++id) {
    if (global_graph_->getVertex(id)->type == VertexType::kUnvisited) {
      unvisited_vertices.push_back(global_graph_->getVertex(id));
    }
  }
  if (unvisited_vertices.empty()) return;

  int num_unvisited_vertices = unvisited_vertices.size();
  const double kLocalBoxRadius = 10;
  const double kLocalBoxRadiusSq = kLocalBoxRadius * kLocalBoxRadius;
  std::vector<Eigen::Vector3d> cluster_centroids;
  std::vector<Vertex*> unvisited_vertices_remain;
  while (true) {
    unvisited_vertices_remain.clear();
    // Randomly pick a vertex
    int ind = rand() % (unvisited_vertices.size());
    // Find all vertices nearby this vertex.
    // Compute the centroid of this cluster.
    Eigen::Vector3d cluster_center(0, 0, 0);
    int num_vertices_in_cluster = 0;
    for (int i = 0; i < unvisited_vertices.size(); ++i) {
      Eigen::Vector3d dist(
          unvisited_vertices[i]->state.x() - unvisited_vertices[ind]->state.x(),
          unvisited_vertices[i]->state.y() - unvisited_vertices[ind]->state.y(),
          unvisited_vertices[i]->state.z() -
              unvisited_vertices[ind]->state.z());
      if (dist.squaredNorm() <= kLocalBoxRadiusSq) {
        cluster_center =
            cluster_center + Eigen::Vector3d(unvisited_vertices[i]->state.x(),
                                             unvisited_vertices[i]->state.y(),
                                             unvisited_vertices[i]->state.z());
        ++num_vertices_in_cluster;
      } else {
        unvisited_vertices_remain.push_back(unvisited_vertices[i]);
      }
    }
    cluster_center = cluster_center / num_vertices_in_cluster;
    cluster_centroids.push_back(cluster_center);
    unvisited_vertices = unvisited_vertices_remain;
    if (unvisited_vertices.empty()) break;
  }
  double time_elapsed = 0;
  int loop_count = 0, loop_count_success = 0;
  int num_vertices = 1;
  int num_edges = 0;
  while (time_elapsed < kGlobalGraphUpdateTimeBudget) {
    time_elapsed = GET_ELAPSED_TIME(time_lim);
    ++loop_count;
    for (int i = 0; i < cluster_centroids.size(); ++i) {
      StateVec centroid_state(cluster_centroids[i].x(),
                              cluster_centroids[i].y(),
                              cluster_centroids[i].z(), 0);
      StateVec new_state;
      if (!sampleVertex(random_sampler_, centroid_state, new_state)) continue;
      // Only expand samples in sparse areas & not yet passed by the robot & not
      // closed to any frontiers
      const double kSparseRadius = 2.0;              // m
      const double kOverlappedFrontierRadius = 3.0;  // m
      std::vector<StateVec*> s_res;
      robot_state_hist_->getNearestStates(&new_state, kSparseRadius, &s_res);
      if (s_res.size()) continue;
      std::vector<Vertex*> v_res;
      global_graph_->getNearestVertices(&new_state, kSparseRadius, &v_res);
      if (v_res.size()) continue;
      std::vector<Vertex*> f_res;
      global_graph_->getNearestVertices(&new_state, kOverlappedFrontierRadius,
                                        &f_res);
      bool frontier_existed = false;
      for (auto v : f_res) {
        if (v->type == VertexType::kFrontier) {
          frontier_existed = true;
          break;
        }
      }
      if (frontier_existed) continue;

      loop_count_success++;
      ExpandGraphReport rep;
      expandGraph(global_graph_, new_state, rep);
      if (rep.status == ExpandGraphStatus::kSuccess) {
        computeVolumetricGainRayModel(rep.vertex_added->state,
                                      rep.vertex_added->vol_gain, false);
        if (rep.vertex_added->vol_gain.is_frontier)
          rep.vertex_added->type = VertexType::kFrontier;
        num_vertices += rep.num_vertices_added;
        num_edges += rep.num_edges_added;
      }
    }
  }

  time_elapsed = GET_ELAPSED_TIME(time_lim);
}

void Rrg::printShortestPath(int id) {
  std::vector<int> id_list;
  local_graph_->getShortestPath(id, local_graph_rep_, false, id_list);
  std::cout << "Path [id,acuumulative_gain] ["
            << local_graph_->getVertex(id)->id << ","
            << local_graph_->getVertex(id)->vol_gain.accumulative_gain << "] ";
  int i = 0;
  while (i < id_list.size()) {
    std::cout << "<-- [" << local_graph_->getVertex(id_list[i])->id << "]";
    ++i;
  }
  std::cout << std::endl;
}

bool Rrg::search(geometry_msgs::Pose source_pose,
                 geometry_msgs::Pose target_pose, bool use_current_state,
                 std::vector<geometry_msgs::Pose>& path_ret) {
  StateVec source;
  if (use_current_state)
    source = current_state_;
  else
    convertPoseMsgToState(source_pose, source);
  StateVec target;
  convertPoseMsgToState(target_pose, target);
  std::shared_ptr<GraphManager> graph_search(new GraphManager());
  RandomSamplingParams sampling_params;
  ROS_WARN("Start searching ...");
  int final_target_id;
  ConnectStatus status = findPathToConnect(
      source, target, graph_search, sampling_params, final_target_id, path_ret);
  if (status == ConnectStatus::kSuccess)
    return true;
  else
    return false;
  // visualization
  visualization_->visualizeGraph(graph_search);
  visualization_->visualizeSampler(random_sampler_to_search_);
}

ConnectStatus Rrg::findPathToConnect(
    StateVec& source, StateVec& target,
    std::shared_ptr<GraphManager> graph_manager, RandomSamplingParams& params,
    int& final_target_id, std::vector<geometry_msgs::Pose>& path_ret) {
  ConnectStatus status;
  path_ret.clear();
  graph_manager->reset();

  ROS_INFO("Search a path from src [%f,%f,%f] to tgt [%f,%f,%f]", source[0],
           source[1], source[2], target[0], target[1], target[2]);

  // Check a corner case if exists a direct collision-free path to connect
  // source and target.
  MapManager::VoxelStatus voxel_state;
  bool try_straight_path = true;
  if (try_straight_path) {
    Eigen::Vector3d src_pos(source[0], source[1], source[2]);
    Eigen::Vector3d tgt_pos(target[0], target[1], target[2]);
    voxel_state = map_manager_->getPathStatus(
        src_pos + robot_params_.center_offset,
        tgt_pos + robot_params_.center_offset, robot_box_size_, true);
    if (voxel_state == MapManager::VoxelStatus::kFree) {
      ROS_INFO("Try straight path...");
      // Add source to the graph.
      Vertex* source_vertex =
          new Vertex(graph_manager->generateVertexID(), source);
      graph_manager->addVertex(source_vertex);
      // Add target to the graph.
      Vertex* target_vertex =
          new Vertex(graph_manager->generateVertexID(), target);
      graph_manager->addVertex(target_vertex);
      graph_manager->addEdge(source_vertex, target_vertex,
                             (tgt_pos - src_pos).norm());
      final_target_id = target_vertex->id;

      geometry_msgs::Pose source_pose;
      convertStateToPoseMsg(source, source_pose);
      path_ret.push_back(source_pose);
      geometry_msgs::Pose target_pose;
      convertStateToPoseMsg(target, target_pose);
      path_ret.push_back(target_pose);

      // Modify heading angle.
      Eigen::Vector3d vec(path_ret[1].position.x - path_ret[0].position.x,
                          path_ret[1].position.y - path_ret[0].position.y,
                          path_ret[1].position.z - path_ret[0].position.z);
      double yaw = std::atan2(vec[1], vec[0]);
      tf::Quaternion quat;
      quat.setEuler(0.0, 0.0, yaw);
      path_ret[1].orientation.x = quat.x();
      path_ret[1].orientation.y = quat.y();
      path_ret[1].orientation.z = quat.z();
      path_ret[1].orientation.w = quat.w();

      status = ConnectStatus::kSuccess;
      return status;
    }
  }

  // Verify source is collision free to go.
  if (params.check_collision_at_source) {
    voxel_state = map_manager_->getBoxStatus(
        Eigen::Vector3d(source[0], source[1], source[2]) +
            robot_params_.center_offset,
        robot_box_size_, true);
    if (MapManager::VoxelStatus::kFree != voxel_state) {
      switch (voxel_state) {
        case MapManager::VoxelStatus::kOccupied:
          ROS_INFO("Source position contains Occupied voxels --> Stop.");
          break;
        case MapManager::VoxelStatus::kUnknown:
          ROS_INFO("Source position contains Unknown voxels  --> Stop.");
          break;
      }
      status = ConnectStatus::kErrorCollisionAtSource;
      return status;
    }
  }
  // Add source to the graph.
  Vertex* source_vertex = new Vertex(graph_manager->generateVertexID(), source);
  graph_manager->addVertex(source_vertex);

  // Start sampling points and add to the graph.
  bool reached_target = false;
  int num_paths_to_target = 0;
  std::vector<Vertex*> target_neigbors;
  int loop_count = 0;
  int num_vertices = 0;
  int num_edges = 0;
  random_sampler_to_search_.reset();
  bool stop_sampling = false;
  while (!stop_sampling) {
    StateVec new_state;
    if (!sampleVertex(random_sampler_to_search_, source, new_state)) continue;
    ExpandGraphReport rep;
    expandGraph(graph_manager, new_state, rep);
    if (rep.status == ExpandGraphStatus::kSuccess) {
      num_vertices += rep.num_vertices_added;
      num_edges += rep.num_edges_added;
      // Check if this state reached the target.
      Eigen::Vector3d radius_vec(new_state[0] - target[0],
                                 new_state[1] - target[1],
                                 new_state[2] - target[2]);
      if (radius_vec.norm() < params.reached_target_radius) {
        target_neigbors.push_back(rep.vertex_added);
        reached_target = true;
        ++num_paths_to_target;
        if (num_paths_to_target > params.num_paths_to_target_max)
          stop_sampling = true;
      }
    }
    if ((loop_count >= params.num_loops_cutoff) &&
        (graph_manager->getNumVertices() <= 1)) {
      stop_sampling = true;
    }

    if ((loop_count++ > params.num_loops_max) ||
        (num_vertices > params.num_vertices_max) ||
        (num_edges > params.num_edges_max))
      stop_sampling = true;
  }
  ROS_INFO("Build a graph with %d vertices and %d edges.",
           graph_manager->getNumVertices(), graph_manager->getNumEdges());

  // Try to add target to graph as well.
  bool added_target = false;
  Vertex* target_vertex = NULL;
  if (reached_target) {
    ROS_WARN("Reached target.");
    // Check if the target voxel is free, then try to add to the graph.
    voxel_state = map_manager_->getBoxStatus(
        Eigen::Vector3d(target[0], target[1], target[2]) +
            robot_params_.center_offset,
        robot_box_size_, true);
    if (voxel_state == MapManager::VoxelStatus::kFree) {
      ExpandGraphReport rep;
      expandGraph(graph_manager, target, rep);
      if (rep.status == ExpandGraphStatus::kSuccess) {
        ROS_INFO("Added target to the graph successfully.");
        num_vertices += rep.num_vertices_added;
        num_edges += rep.num_edges_added;
        added_target = true;
        target_vertex = rep.vertex_added;
      } else {
        ROS_INFO("Cannot expand the graph to connect to the target.");
      }
    } else {
      ROS_INFO("Target is not free, failed to add to the graph.");
    }
  } else {
    ROS_WARN("ConnectStatus::kErrorNoFeasiblePath");
    status = ConnectStatus::kErrorNoFeasiblePath;
    return status;
  }

  // Get shortest path to the goal.
  ShortestPathsReport graph_rep;
  graph_manager->findShortestPaths(graph_rep);

  // Get id list of the shortest path.
  if (!added_target) {
    ROS_INFO("Sorting best path.");
    // Sort all the shortest path that go to target neigbors based on distance
    // in ascending order.
    std::sort(target_neigbors.begin(), target_neigbors.end(),
              [&graph_manager, &graph_rep](const Vertex* a, const Vertex* b) {
                return graph_manager->getShortestDistance(a->id, graph_rep) <
                       graph_manager->getShortestDistance(b->id, graph_rep);
              });
    // Pick the shortest one.
    target_vertex = target_neigbors[0];
  }
  ROS_INFO("Get shortest path [%d] from %d path.", target_vertex->id,
           (int)target_neigbors.size());
  std::vector<int> path_id_list;
  graph_manager->getShortestPath(target_vertex->id, graph_rep, false,
                                 path_id_list);
  final_target_id = target_vertex->id;
  // Convert to the pose message path.
  while (!path_id_list.empty()) {
    geometry_msgs::Pose pose;
    int id = path_id_list.back();
    path_id_list.pop_back();
    convertStateToPoseMsg(graph_manager->getVertex(id)->state, pose);
    path_ret.push_back(pose);
  }

  // Set the heading angle tangent with the moving direction,
  // from the second waypoint; the first waypoint keeps the same direction.
  if (planning_params_.yaw_tangent_correction) {
    for (int i = 0; i < (path_ret.size() - 1); ++i) {
      Eigen::Vector3d vec(path_ret[i + 1].position.x - path_ret[i].position.x,
                          path_ret[i + 1].position.y - path_ret[i].position.y,
                          path_ret[i + 1].position.z - path_ret[i].position.z);
      double yaw = std::atan2(vec[1], vec[0]);
      tf::Quaternion quat;
      quat.setEuler(0.0, 0.0, yaw);
      path_ret[i + 1].orientation.x = quat.x();
      path_ret[i + 1].orientation.y = quat.y();
      path_ret[i + 1].orientation.z = quat.z();
      path_ret[i + 1].orientation.w = quat.w();
    }
  }

  ROS_WARN("Finish searching.");
  status = ConnectStatus::kSuccess;
  visualization_->visualizeBestPaths(graph_manager, graph_rep, 0,
                                     final_target_id);
  return status;
}

bool Rrg::loadParams() {
  // Get the prefix name of the parameters.
  std::string ns = ros::this_node::getName();

  // Load all relevant parameters.
  if (!sensor_params_.loadParams(ns + "/SensorParams")) return false;

  if (!free_frustum_params_.loadParams(ns + "/FreeFrustumParams")) {
    ROS_WARN("No setting for FreeFrustumParams.");
    // return false;
  }

  if (!robot_params_.loadParams(ns + "/RobotParams")) return false;
  if (!global_space_params_.loadParams(ns + "/BoundedSpaceParams/Global"))
    return false;
  if (!local_space_params_.loadParams(ns + "/BoundedSpaceParams/Local"))
    return false;
  if (!local_search_params_.loadParams(ns + "/BoundedSpaceParams/LocalSearch"))
    return false;
  if (!local_vertical_params_.loadParams(
          ns + "/BoundedSpaceParams/LocalVerticalExp")) {
    ROS_WARN("No setting for vertical exploration mode.");
    // return false;
  }

  if (!planning_params_.loadParams(ns + "/PlanningParams")) return false;
  // The sampler doesn't load params automatically.
  // Remember to initialize the sampler in initializeParams() function.
  if (!random_sampler_.loadParams(ns +
                                  "/RandomSamplerParams/SamplerForExploration"))
    return false;
  if (!random_sampler_to_search_.loadParams(
          ns + "/RandomSamplerParams/SamplerForSearching"))
    return false;
  if (!random_sampler_vertical_.loadParams(
          ns + "/RandomSamplerParams/SamplerForVerticalExp")) {
    ROS_WARN("No setting for vertical exploration mode.");
    // return false;
  }

  if (!robot_dynamics_params_.loadParams(ns + "/RobotDynamics")) return false;

  if (!geofence_manager_->loadParams(ns + "/GeofenceParams")) return false;

  // @todo A temporary solution to load the velocity setting.
  planning_params_.v_max = robot_dynamics_params_.v_max;
  planning_params_.v_homing_max = robot_dynamics_params_.v_homing_max;

  // All other relevant const values should be initialized in this call
  // after loading parameters for all fields.
  initializeParams();
  return true;
}

void Rrg::initializeParams() {
  // Compute constant values after loading all parameters to speed up
  // Set sampler params from BoundedSpaceParams if required.
  random_sampler_.setParams(global_space_params_, local_space_params_);
  random_sampler_to_search_.setParams(global_space_params_,
                                      local_search_params_);
  if (random_sampler_vertical_.isReady)
    random_sampler_vertical_.setParams(global_space_params_,
                                       local_vertical_params_);

  // Precompute the robot box for planning.
  robot_params_.getPlanningSize(robot_box_size_);
  planning_num_vertices_max_ = planning_params_.num_vertices_max;
  planning_num_edges_max_ = planning_params_.num_edges_max;

  // Get the global bounding box in the setting as default.
  // Visualize in the beginning for checking.
  global_bound_.setDefault(global_space_params_.min_val,
                           global_space_params_.max_val);
  visualization_->visualizeWorkspace(current_state_, global_space_params_,
                                     local_space_params_);
}

bool Rrg::setGlobalBound(planner_msgs::PlanningBound& bound,
                         bool reset_to_default) {
  if (!reset_to_default) {
    // Make sure current position of the robot and its bounding box is inside
    // the global bound.
    if ((current_state_.x() + robot_params_.center_offset.x() <
         bound.min_val.x + 0.5 * robot_box_size_.x()) ||
        (current_state_.y() + robot_params_.center_offset.y() <
         bound.min_val.y + 0.5 * robot_box_size_.y()) ||
        (current_state_.z() + robot_params_.center_offset.z() <
         bound.min_val.z + 0.5 * robot_box_size_.z()) ||
        (current_state_.x() + robot_params_.center_offset.x() >
         bound.max_val.x - 0.5 * robot_box_size_.x()) ||
        (current_state_.y() + robot_params_.center_offset.y() >
         bound.max_val.y - 0.5 * robot_box_size_.y()) ||
        (current_state_.z() + robot_params_.center_offset.z() >
         bound.max_val.z - 0.5 * robot_box_size_.z())) {
      ROS_WARN(
          "[GlobalBound] Failed to change since robot's position is outside "
          "the global bound.");
      return false;
    }

    Eigen::Vector3d v_min, v_max;
    global_bound_.get(v_min, v_max);
    v_min.x() = bound.min_val.x;
    v_min.y() = bound.min_val.y;
    if (bound.use_z_val) v_min.z() = bound.min_val.z;
    v_max.x() = bound.max_val.x;
    v_max.y() = bound.max_val.y;
    if (!bound.use_z_val) v_max.z() = bound.max_val.z;
    global_bound_.set(v_min, v_max);
    global_space_params_.min_val = v_min;
    global_space_params_.max_val = v_max;
    ROS_WARN(
        "[GlobalBound] Changed successfully: Min [%f, %f, %f], Max [%f, %f, "
        "%f]",
        v_min.x(), v_min.y(), v_min.z(), v_max.x(), v_max.y(), v_max.z());
  } else {
    // reset to an original bounding box.
    global_bound_.reset();
    Eigen::Vector3d v_min, v_max;
    global_bound_.get(v_min, v_max);
    global_space_params_.min_val = v_min;
    global_space_params_.max_val = v_max;
    ROS_WARN(
        "[GlobalBound] Reset to default: Min [%f, %f, %f], Max [%f, %f, %f]",
        v_min.x(), v_min.y(), v_min.z(), v_max.x(), v_max.y(), v_max.z());
  }
  visualization_->visualizeWorkspace(current_state_, global_space_params_,
                                     local_space_params_);
  return true;
}

void Rrg::getGlobalBound(planner_msgs::PlanningBound& bound) {
  global_bound_.get(bound.min_val, bound.max_val);
}

void Rrg::computeExplorationGain(bool only_leaf_vertices) {
  const int id_viz = 20;  // random vertex to show volumetric gain.
  ros::Time tim;
  START_TIMER(tim);
  // Compute gain of all vertices in one function.
  for (const auto& v : local_graph_->vertices_map_) {
    bool viz_en = false;
    if (v.second->id == id_viz) viz_en = true;
    if (planning_params_.use_ray_model_for_volumetric_gain) {
      if ((!only_leaf_vertices) || (v.second->is_leaf_vertex))
        computeVolumetricGainRayModel(v.second->state, v.second->vol_gain,
                                      viz_en);
    } else {
      if ((!only_leaf_vertices) || (v.second->is_leaf_vertex))
        computeVolumetricGain(v.second->state, v.second->vol_gain, viz_en);
    }
    if (v.second->vol_gain.is_frontier) v.second->type = VertexType::kFrontier;
  }
  stat_->compute_exp_gain_time = GET_ELAPSED_TIME(tim);
}

void Rrg::computeVolumetricGain(StateVec& state, VolumetricGain& vgain,
                                bool vis_en) {
  vgain.reset();
  double step_size = planning_params_.exp_gain_voxel_size;
  // Scan winthin a local space and sensor range.
  // Compute the local bound.
  Eigen::Vector3d bound_min;
  Eigen::Vector3d bound_max;
  if (local_space_params_.type == BoundedSpaceType::kSphere) {
    for (int i = 0; i < 3; ++i) {
      bound_min[i] = root_vertex_->state[i] - local_space_params_.radius -
                     local_space_params_.radius_extension;
      bound_max[i] = root_vertex_->state[i] + local_space_params_.radius +
                     local_space_params_.radius_extension;
    }
  } else if (local_space_params_.type == BoundedSpaceType::kCuboid) {
    for (int i = 0; i < 3; ++i) {
      bound_min[i] = root_vertex_->state[i] + local_space_params_.min_val[i] +
                     local_space_params_.min_extension[i];
      bound_max[i] = root_vertex_->state[i] + local_space_params_.max_val[i] +
                     local_space_params_.max_extension[i];
    }
  } else {
    PLANNER_ERROR("Local space is not defined.");
    return;
  }

  // Refine the bound with global bound.
  if (global_space_params_.type == BoundedSpaceType::kSphere) {
    for (int i = 0; i < 3; i++) {
      bound_min[i] =
          std::max(bound_min[i], -global_space_params_.radius -
                                     global_space_params_.radius_extension);
      bound_max[i] =
          std::min(bound_max[i], global_space_params_.radius +
                                     global_space_params_.radius_extension);
    }
  } else if (global_space_params_.type == BoundedSpaceType::kCuboid) {
    for (int i = 0; i < 3; i++) {
      bound_min[i] =
          std::max(bound_min[i], global_space_params_.min_val[i] +
                                     global_space_params_.min_extension[i]);
      bound_max[i] =
          std::min(bound_max[i], global_space_params_.max_val[i] +
                                     global_space_params_.max_extension[i]);
    }
  } else {
    PLANNER_ERROR("Global space is not defined.");
    return;
  }

  std::vector<std::tuple<int, int, int>> gain_log;
  gain_log.clear();
  std::vector<std::pair<Eigen::Vector3d, MapManager::VoxelStatus>> voxel_log;
  voxel_log.clear();
  // Compute for each sensor in the exploration sensor list.
  // However, this would be a problem if those sensors have significant overlap.
  for (int ind = 0; ind < planning_params_.exp_sensor_list.size(); ++ind) {
    std::string sensor_name = planning_params_.exp_sensor_list[ind];
    // Refine the bound within an effective range.
    for (int i = 0; i < 3; i++) {
      bound_min[i] =
          std::max(bound_min[i],
                   state[i] - sensor_params_.sensor[sensor_name].max_range);
      bound_max[i] =
          std::min(bound_max[i],
                   state[i] + sensor_params_.sensor[sensor_name].max_range);
    }

    int num_unknown_voxels = 0, num_free_voxels = 0, num_occupied_voxels = 0;
    // Check all voxels inside local bound.
    Eigen::Vector3d origin(state[0], state[1], state[2]);
    Eigen::Vector3d voxel;
    for (voxel[0] = bound_min[0]; voxel[0] < bound_max[0];
         voxel[0] += step_size) {
      for (voxel[1] = bound_min[1]; voxel[1] < bound_max[1];
           voxel[1] += step_size) {
        for (voxel[2] = bound_min[2]; voxel[2] < bound_max[2];
             voxel[2] += step_size) {
          if (sensor_params_.sensor[sensor_name].isInsideFOV(state, voxel)) {
            MapManager::VoxelStatus vs_ray =
                map_manager_->getRayStatus(origin, voxel, true);
            if (vs_ray != MapManager::VoxelStatus::kOccupied) {
              MapManager::VoxelStatus vs = map_manager_->getVoxelStatus(voxel);
              if (vs == MapManager::VoxelStatus::kUnknown) {
                ++num_unknown_voxels;
              } else if (vs == MapManager::VoxelStatus::kFree) {
                ++num_free_voxels;
              } else if (vs == MapManager::VoxelStatus::kOccupied) {
                ++num_occupied_voxels;
              }
              if (vis_en) voxel_log.push_back(std::make_pair(voxel, vs));
            }
          }
        }
      }
    }
    gain_log.push_back(std::make_tuple(num_unknown_voxels, num_free_voxels,
                                       num_occupied_voxels));
  }

  // Return gain values.
  for (int i = 0; i < gain_log.size(); ++i) {
    int num_unknown_voxels = std::get<0>(gain_log[i]);
    int num_free_voxels = std::get<1>(gain_log[i]);
    int num_occupied_voxels = std::get<2>(gain_log[i]);
    vgain.num_unknown_voxels += num_unknown_voxels;
    vgain.num_free_voxels += num_free_voxels;
    vgain.num_occupied_voxels += num_occupied_voxels;
    vgain.gain += num_unknown_voxels * planning_params_.unknown_voxel_gain +
                  num_free_voxels * planning_params_.free_voxel_gain +
                  num_occupied_voxels * planning_params_.occupied_voxel_gain;
  }

  // Visualize if required.
  if (vis_en) {
    visualization_->visualizeVolumetricGain(bound_min, bound_max, voxel_log,
                                            step_size);
  }
}

void Rrg::computeVolumetricGainRayModel(StateVec& state, VolumetricGain& vgain,
                                        bool vis_en) {
  vgain.reset();

  // Scan winthin a local space and sensor range.
  // Compute the local bound.
  Eigen::Vector3d bound_min;
  Eigen::Vector3d bound_max;
  if (local_space_params_.type == BoundedSpaceType::kSphere) {
    for (int i = 0; i < 3; ++i) {
      bound_min[i] = root_vertex_->state[i] - local_space_params_.radius -
                     local_space_params_.radius_extension;
      bound_max[i] = root_vertex_->state[i] + local_space_params_.radius +
                     local_space_params_.radius_extension;
    }
  } else if (local_space_params_.type == BoundedSpaceType::kCuboid) {
    for (int i = 0; i < 3; ++i) {
      bound_min[i] = root_vertex_->state[i] + local_space_params_.min_val[i] +
                     local_space_params_.min_extension[i];
      bound_max[i] = root_vertex_->state[i] + local_space_params_.max_val[i] +
                     local_space_params_.max_extension[i];
    }
  } else {
    PLANNER_ERROR("Local space is not defined.");
    return;
  }

  // Refine the bound with global bound.
  if (global_space_params_.type == BoundedSpaceType::kSphere) {
    for (int i = 0; i < 3; i++) {
      bound_min[i] =
          std::max(bound_min[i], -global_space_params_.radius -
                                     global_space_params_.radius_extension);
      bound_max[i] =
          std::min(bound_max[i], global_space_params_.radius +
                                     global_space_params_.radius_extension);
    }
  } else if (global_space_params_.type == BoundedSpaceType::kCuboid) {
    for (int i = 0; i < 3; i++) {
      bound_min[i] =
          std::max(bound_min[i], global_space_params_.min_val[i] +
                                     global_space_params_.min_extension[i]);
      bound_max[i] =
          std::min(bound_max[i], global_space_params_.max_val[i] +
                                     global_space_params_.max_extension[i]);
    }
  } else {
    PLANNER_ERROR("Global space is not defined.");
    return;
  }

  std::vector<std::tuple<int, int, int>> gain_log;
  std::vector<std::pair<Eigen::Vector3d, MapManager::VoxelStatus>> voxel_log;
  int raw_unk_voxels_count = 0;
  // Compute for each sensor in the exploration sensor list.
  // However, this would be a problem if those sensors have significant overlap.
  for (int ind = 0; ind < planning_params_.exp_sensor_list.size(); ++ind) {
    std::string sensor_name = planning_params_.exp_sensor_list[ind];
    // Refine the bound within an effective range.
    for (int i = 0; i < 3; i++) {
      bound_min[i] =
          std::max(bound_min[i],
                   state[i] - sensor_params_.sensor[sensor_name].max_range);
      bound_max[i] =
          std::min(bound_max[i],
                   state[i] + sensor_params_.sensor[sensor_name].max_range);
    }

    Eigen::Vector3d origin(state[0], state[1], state[2]);
    std::tuple<int, int, int> gain_log_tmp;
    std::vector<std::pair<Eigen::Vector3d, MapManager::VoxelStatus>>
        voxel_log_tmp;
    std::vector<Eigen::Vector3d> multiray_endpoints;
    sensor_params_.sensor[sensor_name].getFrustumEndpoints(state,
                                                           multiray_endpoints);
    map_manager_->getScanStatus(origin, multiray_endpoints, gain_log_tmp,
                                voxel_log_tmp);
    int num_unknown_voxels = 0, num_free_voxels = 0, num_occupied_voxels = 0;
    // Have to remove those not belong to the local bound.
    // At the same time check if this is frontier.

    for (auto& vl : voxel_log_tmp) {
      Eigen::Vector3d voxel = vl.first;
      MapManager::VoxelStatus vs = vl.second;
      if (vs == MapManager::VoxelStatus::kUnknown) ++raw_unk_voxels_count;
      int j = 0;
      for (j = 0; j < 3; j++) {
        if ((voxel[j] < bound_min[j]) || (voxel[j] > bound_max[j])) break;
      }
      if (j == 3) {
        // valid voxel.
        if (vs == MapManager::VoxelStatus::kUnknown) {
          ++num_unknown_voxels;
        } else if (vs == MapManager::VoxelStatus::kFree) {
          ++num_free_voxels;
        } else if (vs == MapManager::VoxelStatus::kOccupied) {
          ++num_occupied_voxels;
        } else {
          ROS_ERROR("Unsupported voxel type.");
        }
        if (vis_en) voxel_log.push_back(std::make_pair(voxel, vs));
      }
    }
    gain_log.push_back(std::make_tuple(num_unknown_voxels, num_free_voxels,
                                       num_occupied_voxels));
    if (vis_en) {
      visualization_->visualizeRays(state, multiray_endpoints);
    }

    // Check if it is a potential frontier.
    if (sensor_params_.sensor[sensor_name].isFrontier(
            num_unknown_voxels * map_manager_->getResolution())) {
      vgain.is_frontier = true;  // Event E2
    }
  }

  // Return gain values.
  for (int i = 0; i < gain_log.size(); ++i) {
    int num_unknown_voxels = std::get<0>(gain_log[i]);
    int num_free_voxels = std::get<1>(gain_log[i]);
    int num_occupied_voxels = std::get<2>(gain_log[i]);
    vgain.num_unknown_voxels += num_unknown_voxels;
    vgain.num_free_voxels += num_free_voxels;
    vgain.num_occupied_voxels += num_occupied_voxels;
    vgain.gain += num_unknown_voxels * planning_params_.unknown_voxel_gain +
                  num_free_voxels * planning_params_.free_voxel_gain +
                  num_occupied_voxels * planning_params_.occupied_voxel_gain;
  }

  // Visualize if required.
  if (vis_en) {
    visualization_->visualizeVolumetricGain(bound_min, bound_max, voxel_log,
                                            map_manager_->getResolution());
  }
}

void Rrg::computeVolumetricGainRayModelNoBound(StateVec& state,
                                               VolumetricGain& vgain) {
  vgain.reset();

  // Scan winthin a GLOBAL space and sensor range.
  Eigen::Vector3d bound_min;
  Eigen::Vector3d bound_max;
  if (global_space_params_.type == BoundedSpaceType::kSphere) {
    for (int i = 0; i < 3; i++) {
      bound_min[i] =
          -global_space_params_.radius - global_space_params_.radius_extension;
      bound_max[i] =
          global_space_params_.radius + global_space_params_.radius_extension;
    }
  } else if (global_space_params_.type == BoundedSpaceType::kCuboid) {
    for (int i = 0; i < 3; i++) {
      bound_min[i] = global_space_params_.min_val[i] +
                     global_space_params_.min_extension[i];
      bound_max[i] = global_space_params_.max_val[i] +
                     global_space_params_.max_extension[i];
    }
  } else {
    PLANNER_ERROR("Global space is not defined.");
    return;
  }

  std::vector<std::tuple<int, int, int>> gain_log;
  std::vector<std::pair<Eigen::Vector3d, MapManager::VoxelStatus>> voxel_log;
  // @TODO tung.
  // Compute for each sensor in the exploration sensor list.
  // However, this would be a problem if those sensors have significant overlap.
  for (int ind = 0; ind < planning_params_.exp_sensor_list.size(); ++ind) {
    std::string sensor_name = planning_params_.exp_sensor_list[ind];
    // Refine the bound within an effective range.
    for (int i = 0; i < 3; i++) {
      bound_min[i] =
          std::max(bound_min[i],
                   state[i] - sensor_params_.sensor[sensor_name].max_range);
      bound_max[i] =
          std::min(bound_max[i],
                   state[i] + sensor_params_.sensor[sensor_name].max_range);
    }

    Eigen::Vector3d origin(state[0], state[1], state[2]);
    std::tuple<int, int, int> gain_log_tmp;
    std::vector<std::pair<Eigen::Vector3d, MapManager::VoxelStatus>>
        voxel_log_tmp;
    std::vector<Eigen::Vector3d> multiray_endpoints;
    sensor_params_.sensor[sensor_name].getFrustumEndpoints(state,
                                                           multiray_endpoints);
    map_manager_->getScanStatus(origin, multiray_endpoints, gain_log_tmp,
                                voxel_log_tmp);
    int num_unknown_voxels = 0, num_free_voxels = 0, num_occupied_voxels = 0;
    // Have to remove those not belong to the local bound.
    // At the same time check if this is frontier.

    for (auto& vl : voxel_log_tmp) {
      Eigen::Vector3d voxel = vl.first;
      MapManager::VoxelStatus vs = vl.second;
      int j = 0;
      for (j = 0; j < 3; j++) {
        if ((voxel[j] < bound_min[j]) || (voxel[j] > bound_max[j])) break;
      }
      if (j == 3) {
        // valid voxel.
        if (vs == MapManager::VoxelStatus::kUnknown) {
          ++num_unknown_voxels;
        } else if (vs == MapManager::VoxelStatus::kFree) {
          ++num_free_voxels;
        } else if (vs == MapManager::VoxelStatus::kOccupied) {
          ++num_occupied_voxels;
        } else {
          ROS_ERROR("Unsupported voxel type.");
        }
      }
    }
    gain_log.push_back(std::make_tuple(num_unknown_voxels, num_free_voxels,
                                       num_occupied_voxels));
    // Check if it is a potential frontier.
    if (sensor_params_.sensor[sensor_name].isFrontier(
            num_unknown_voxels * map_manager_->getResolution())) {
      vgain.is_frontier = true;  // Event E2
    }
  }

  // Return gain values.
  for (int i = 0; i < gain_log.size(); ++i) {
    int num_unknown_voxels = std::get<0>(gain_log[i]);
    int num_free_voxels = std::get<1>(gain_log[i]);
    int num_occupied_voxels = std::get<2>(gain_log[i]);
    vgain.num_unknown_voxels += num_unknown_voxels;
    vgain.num_free_voxels += num_free_voxels;
    vgain.num_occupied_voxels += num_occupied_voxels;
    vgain.gain += num_unknown_voxels * planning_params_.unknown_voxel_gain +
                  num_free_voxels * planning_params_.free_voxel_gain +
                  num_occupied_voxels * planning_params_.occupied_voxel_gain;
  }
}

void Rrg::setRootStateForPlanning(const geometry_msgs::Pose& root_pose) {
  // If require plan ahead --> use the end pose from the last best path.
  // Otherwise, use current pose.
  state_for_planning_[0] = root_pose.position.x;
  state_for_planning_[1] = root_pose.position.y;
  state_for_planning_[2] = root_pose.position.z;
  state_for_planning_[3] = tf::getYaw(root_pose.orientation);
  if ((state_for_planning_[0] == 0.0) && (state_for_planning_[1] == 0.0) &&
      (state_for_planning_[2] == 0)) {
    planning_params_.use_current_state = true;
  } else {
    planning_params_.use_current_state = false;
  }
}

bool Rrg::setHomingPos() {
  if (global_graph_->getNumVertices() == 0) {
    ROS_INFO("Global graph is empty: add current state as homing position.");
    Vertex* g_root_vertex =
        new Vertex(global_graph_->generateVertexID(), current_state_);
    global_graph_->addVertex(g_root_vertex);
    return true;
  } else {
    ROS_INFO("Global graph is not empty, can not set current state as homing.");
    return false;
  }
}

std::vector<geometry_msgs::Pose> Rrg::searchHomingPath(
    std::string tgt_frame, const StateVec& current_state) {
  std::vector<geometry_msgs::Pose> ret_path;
  ret_path.clear();

  if (global_graph_->getNumVertices() <= 1) {
    ROS_WARN("[GlobalGraph] Graph is empty, nothing to search for homing.");
    return ret_path;
  }

  StateVec cur_state;
  cur_state << current_state[0], current_state[1], current_state[2],
      current_state[3];
  Vertex* nearest_vertex = NULL;
  if (!global_graph_->getNearestVertex(&cur_state, &nearest_vertex))
    return ret_path;
  if (nearest_vertex == NULL) return ret_path;
  Eigen::Vector3d origin(nearest_vertex->state[0], nearest_vertex->state[1],
                         nearest_vertex->state[2]);
  Eigen::Vector3d direction(cur_state[0] - origin[0], cur_state[1] - origin[1],
                            cur_state[2] - origin[2]);
  double direction_norm = direction.norm();

  Vertex* link_vertex = NULL;
  const double kRadiusLimit = 1.0;
  const double kRadiusDelta = 0.2;
  bool connect_state_to_graph = true;
  if (direction_norm <= kRadiusLimit) {
    // Note: if kRadiusLimit <= edge_length_min it will fail with
    // kErrorShortEdge, dirty fix to check max
    // @TODO: find better way to do this.
    // Blindly add a link/vertex to the graph if small radius.
    Vertex* new_vertex =
        new Vertex(global_graph_->generateVertexID(), cur_state);
    new_vertex->parent = nearest_vertex;
    new_vertex->distance = nearest_vertex->distance + direction_norm;
    nearest_vertex->children.push_back(new_vertex);
    global_graph_->addVertex(new_vertex);
    global_graph_->addEdge(new_vertex, nearest_vertex, direction_norm);
    // Add edges only from this vertex.
    ExpandGraphReport rep;
    expandGraphEdges(global_graph_, new_vertex, rep);
    link_vertex = new_vertex;
  } else {
    ROS_WARN("[GlobalGraph] Try to add current state to the graph.");
    ExpandGraphReport rep;
    expandGraph(global_graph_, cur_state, rep);
    if (rep.status == ExpandGraphStatus::kSuccess) {
      ROS_WARN("[GlobalGraph] Added successfully.");
      link_vertex = rep.vertex_added;
    } else {
      // Not implemented solution for this case yet.
      // Hopefully this one will not happen if the global planner always adds
      // vertices from odometry --> naive backtracking.
      connect_state_to_graph = false;
      ROS_WARN("[GlobalGraph] Can not add current state to graph since: ");
      switch (rep.status) {
        case ExpandGraphStatus::kErrorKdTree:
          ROS_WARN("kErrorKdTree.");
          break;
        case ExpandGraphStatus::kErrorCollisionEdge:
          ROS_WARN("kErrorCollisionEdge.");
          break;
        case ExpandGraphStatus::kErrorShortEdge:
          ROS_WARN("kErrorShortEdge.");
          break;
      }
      ROS_WARN("[GlobalGraph] Failed to find global path.");
    }
  }

  if (connect_state_to_graph) {
    if (!global_graph_->findShortestPaths(global_graph_rep_)) {
      ROS_ERROR("[GlobalGraph] Failed to find shortest path.");
      return ret_path;
    }
    std::vector<int> homing_path_id;
    global_graph_->getShortestPath(link_vertex->id, global_graph_rep_, false,
                                   homing_path_id);
    if (homing_path_id.empty() || homing_path_id.back() != 0) {
      ROS_ERROR("[GlobalGraph] Could not find a path to home.");
      return ret_path;
    }
    int homing_path_id_size = homing_path_id.size();
    for (int i = 0; i < homing_path_id_size; ++i) {
      StateVec state = global_graph_->getVertex(homing_path_id[i])->state;
      tf::Quaternion quat;
      quat.setEuler(0.0, 0.0, state[3]);
      tf::Vector3 origin(state[0], state[1], state[2]);
      tf::Pose poseTF(quat, origin);
      geometry_msgs::Pose pose;
      tf::poseTFToMsg(poseTF, pose);
      ret_path.push_back(pose);
    }

    // Set the heading angle tangent with the moving direction,
    // from the second waypoint; the first waypoint keeps the same direction.
    if (planning_params_.yaw_tangent_correction) {
      bool is_similar = comparePathWithDirectionApprioximately(
          ret_path, tf::getYaw(ret_path[0].orientation));
      for (int i = 0; i < (ret_path.size() - 1); ++i) {
        Eigen::Vector3d vec;
        if ((!planning_params_.homing_backward) || (is_similar)) {
          vec << ret_path[i + 1].position.x - ret_path[i].position.x,
              ret_path[i + 1].position.y - ret_path[i].position.y,
              ret_path[i + 1].position.z - ret_path[i].position.z;
        } else if (planning_params_.homing_backward) {
          vec << ret_path[i].position.x - ret_path[i + 1].position.x,
              ret_path[i].position.y - ret_path[i + 1].position.y,
              ret_path[i].position.z - ret_path[i + 1].position.z;
        }
        double yaw = std::atan2(vec[1], vec[0]);
        tf::Quaternion quat;
        quat.setEuler(0.0, 0.0, yaw);
        ret_path[i + 1].orientation.x = quat.x();
        ret_path[i + 1].orientation.y = quat.y();
        ret_path[i + 1].orientation.z = quat.z();
        ret_path[i + 1].orientation.w = quat.w();
      }
    }
    visualization_->visualizeHomingPaths(global_graph_, global_graph_rep_,
                                         link_vertex->id);
  }
  visualization_->visualizeGlobalGraph(global_graph_);
  return ret_path;
}

std::vector<geometry_msgs::Pose> Rrg::getHomingPath(std::string tgt_frame) {
  // ros::Duration(3.0).sleep(); // sleep to unblock the thread to get and
  // update all latest pose update.
  //ros::spinOnce();

  std::vector<geometry_msgs::Pose> ret_path;
  ret_path = searchHomingPath(tgt_frame, current_state_);
  if (ret_path.size() < 1) return ret_path;

  // Re-assign yaw in beginning (HNI)
  double yaw = current_state_[3];
  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, yaw);
  ret_path[0].orientation.x = quat.x();
  ret_path[0].orientation.y = quat.y();
  ret_path[0].orientation.z = quat.z();
  ret_path[0].orientation.w = quat.w();

  const bool clean_short_edges = true;
  if (clean_short_edges && (robot_params_.type == RobotType::kLeggedRobot)) {
    // check if the path is weird: sudden change in the orientation
    // find segments that need to be improved build the graph
    // to reconnect those segments only, keep the remaining the same
    std::vector<geometry_msgs::Pose> mod_path;
    if (reconnectPathBlindly(ret_path, mod_path)) {
      ret_path = mod_path;
    }
  }

  // Modify the best path.
  if (planning_params_.path_safety_enhance_enable) {
    ros::Time mod_time;
    START_TIMER(mod_time);
    std::vector<geometry_msgs::Pose> mod_path;
    if (improveFreePath(ret_path, mod_path)) {
      ret_path = mod_path;
      // Re-assign yaw angle after modification.
      if (planning_params_.yaw_tangent_correction) {
        bool is_similar = comparePathWithDirectionApprioximately(
            ret_path, tf::getYaw(ret_path[0].orientation));
        for (int i = 0; i < (ret_path.size() - 1); ++i) {
          Eigen::Vector3d vec;
          if ((!planning_params_.homing_backward) || (is_similar)) {
            vec << ret_path[i + 1].position.x - ret_path[i].position.x,
                ret_path[i + 1].position.y - ret_path[i].position.y,
                ret_path[i + 1].position.z - ret_path[i].position.z;
          } else if (planning_params_.homing_backward) {
            vec << ret_path[i].position.x - ret_path[i + 1].position.x,
                ret_path[i].position.y - ret_path[i + 1].position.y,
                ret_path[i].position.z - ret_path[i + 1].position.z;
          }
          double yaw = std::atan2(vec[1], vec[0]);
          tf::Quaternion quat;
          quat.setEuler(0.0, 0.0, yaw);
          ret_path[i + 1].orientation.x = quat.x();
          ret_path[i + 1].orientation.y = quat.y();
          ret_path[i + 1].orientation.z = quat.z();
          ret_path[i + 1].orientation.w = quat.w();
        }
      }
    }

    double dmod_time = GET_ELAPSED_TIME(mod_time);
    ROS_WARN("Compute an aternate path for homing in %f(s)", dmod_time);
    visualization_->visualizeModPath(mod_path);
  }

  visualization_->visualizeRefPath(ret_path);
  return ret_path;
}

bool Rrg::reconnectPathBlindly(std::vector<geometry_msgs::Pose>& ref_path,
                               std::vector<geometry_msgs::Pose>& mod_path) {
  // Divide and conquer/Coarse to fine
  // Interpolate the path into intermidiate nodes and reconnect using graph
  if (ref_path.size() <= 2) return false;

  // Interpolate the whole path to the finer resolution.
  const double kPathResolution = 0.2;
  Trajectory::PathType path_extract;
  Trajectory::extractPathFromTrajectory(ref_path, path_extract);
  Trajectory::PathType path_intp;
  if (!Trajectory::interpolatePath(path_extract, kPathResolution, path_intp))
    return false;

  std::shared_ptr<GraphManager> path_graph;
  path_graph.reset(new GraphManager());

  StateVec root_state(path_intp[0][0], path_intp[0][1], path_intp[0][2], 0);
  Vertex* root_vertex = new Vertex(path_graph->generateVertexID(), root_state);
  path_graph->addVertex(root_vertex);

  // Add all remaining vertices of the path.
  std::vector<Vertex*> vertex_list;
  vertex_list.push_back(root_vertex);
  Vertex* parent_vertex = root_vertex;
  for (int i = 1; i < path_intp.size(); ++i) {
    StateVec new_state(path_intp[i][0], path_intp[i][1], path_intp[i][2], 0);
    Vertex* new_vertex = new Vertex(path_graph->generateVertexID(), new_state);
    new_vertex->parent = parent_vertex;
    Eigen::Vector3d dist(new_state[0] - parent_vertex->state[0],
                         new_state[1] - parent_vertex->state[1],
                         new_state[2] - parent_vertex->state[2]);
    new_vertex->distance = parent_vertex->distance + dist.norm();
    parent_vertex->children.push_back(new_vertex);
    path_graph->addVertex(new_vertex);
    path_graph->addEdge(new_vertex, parent_vertex, dist.norm());
    vertex_list.push_back(new_vertex);
    parent_vertex = new_vertex;
  }

  // Build edges around vertices if possible to get better path.
  const double kBlindConnectionRadius = 0.1;
  int n_vertices = 0;
  int n_edges = 0;
  // Assume the path is verified collision free.
  for (int i = 0; i < vertex_list.size(); ++i) {
    ExpandGraphReport rep;
    expandGraphEdgesBlindly(path_graph, vertex_list[i], kBlindConnectionRadius,
                            rep);
    if (rep.status == ExpandGraphStatus::kSuccess) {
      n_vertices += rep.num_vertices_added;
      n_edges += rep.num_edges_added;
    }
  }

  // Find the shortest path again
  ShortestPathsReport path_graph_rep;
  path_graph_rep.reset();
  path_graph->findShortestPaths(path_graph_rep);
  std::vector<Eigen::Vector3d> shortest_path;
  path_graph->getShortestPath(path_graph->getNumVertices() - 1, path_graph_rep,
                              true, shortest_path);

  // Keep the first orientation, the remaining could be adjusted.
  mod_path.clear();
  mod_path.push_back(ref_path[0]);
  double prev_yaw = tf::getYaw(mod_path[0].orientation);
  Eigen::Vector3d new_node;
  for (int i = 0; i < shortest_path.size() - 1; ++i) {
    double new_yaw = std::atan2(shortest_path[i + 1][1] - shortest_path[i][1],
                                shortest_path[i + 1][0] - shortest_path[i][0]);
    double yaw_diff = new_yaw - prev_yaw;
    truncateYaw(yaw_diff);
    const double kYawEpsilon = 0.01;
    if (std::abs(yaw_diff) > kYawEpsilon) {
      if (i) {
        // already added [0] element.
        StateVec st(shortest_path[i][0], shortest_path[i][1],
                    shortest_path[i][2], prev_yaw);
        geometry_msgs::Pose pose_tmp;
        convertStateToPoseMsg(st, pose_tmp);
        mod_path.push_back(pose_tmp);
      }
      prev_yaw = new_yaw;
    }
  }
  // Add the last pose.
  mod_path.push_back(ref_path.back());
  return true;
}

std::vector<geometry_msgs::Pose> Rrg::getBestPath(std::string tgt_frame,
                                                  int& status) {
  // Decide if need to go home.
  if (planning_params_.auto_homing_enable) {
    status = planner_msgs::planner_srv::Response::kHoming;
    const double kTimeDelta = 20;
    std::vector<geometry_msgs::Pose> homing_path;
    double time_elapsed = 0.0;
    if ((ros::Time::now()).toSec() != 0.0) {
      if (rostime_start_.toSec() == 0.0) rostime_start_ = ros::Time::now();
      time_elapsed = (double)((ros::Time::now() - rostime_start_).toSec());
    }
    double time_budget_remaining =
        planning_params_.time_budget_limit - time_elapsed;
    if (time_budget_remaining <= 0.0) {
      ROS_WARN("RAN OUT OF TIME BUDGET --> STOP HERE.");
      return homing_path;
    }
    if (current_battery_time_remaining_ <= 0.0) {
      ROS_WARN("RAN OUT OF BATTERY --> STOP HERE.");
      return homing_path;
    }
    // Check two conditions whatever which one comes first.
    double time_remaining =
        std::min(time_budget_remaining, current_battery_time_remaining_);
    // homing_path = getHomingPath(tgt_frame);
    // Start searching from current root vertex of spanning local graph.
    Vertex* root_vertex = local_graph_->getVertex(0);
    homing_path = searchHomingPath(tgt_frame, root_vertex->state);
    if (!homing_path.empty()) {
      double homing_len = Trajectory::getPathLength(homing_path);
      double time_to_home = homing_len / planning_params_.v_homing_max;
      ROS_INFO("Time to home: %f; Time remaining: %f", time_to_home,
               time_remaining);

      const double kTimeDelta = 20;
      if (std::abs(time_to_home - time_remaining) < kTimeDelta) {
        ROS_WARN("REACHED TIME LIMIT: HOMING ENGAGED.");
        return homing_path;
      }
    } else {
      // @TODO Issue with global graph, cannot find a homing path.
      ROS_WARN("Can not find a path to return home from here.");
    }
  }

  std::vector<geometry_msgs::Pose> ret;
  int id = best_vertex_->id;
  if (id == 0) return ret;

  // Get potentially exploring direction.
  std::vector<Eigen::Vector3d> best_path_3d;
  local_graph_->getShortestPath(id, local_graph_rep_, true, best_path_3d);
  double best_path_direction =
      Trajectory::estimateDirectionFromPath(best_path_3d);
  constexpr double kDiffAngleForwardThres = 0.5 * (M_PI + M_PI / 3);
  bool res = compareAngles(exploring_direction_, best_path_direction,
                           kDiffAngleForwardThres);
  if (planning_params_.type != PlanningModeType::kVerticalExploration) {
    if (!res) {
      ROS_WARN("Changing exploration direction.[%f --> %f]",
               exploring_direction_, best_path_direction);
      status = planner_msgs::planner_srv::Response::kBackward;
    } else {
      ROS_INFO("Current exploring direction: %f; New path direction: %f",
               exploring_direction_, best_path_direction);
      status = planner_msgs::planner_srv::Response::kForward;
    }
  }

  // Extract path.
  double traverse_length = 0;
  double traverse_time = 0;
  std::vector<StateVec> best_path;
  local_graph_->getShortestPath(id, local_graph_rep_, true, best_path);
  Eigen::Vector3d p0(best_path[0][0], best_path[0][1], best_path[0][2]);
  std::vector<Vertex*> best_path_vertices;
  local_graph_->getShortestPath(best_vertex_->id, local_graph_rep_, true,
                                best_path_vertices);

  const double kLenMin = 1.0;
  const double kLenMinMin = 0.3;
  std::vector<Eigen::Vector3d> path_vec;
  local_graph_->getShortestPath(best_vertex_->id, local_graph_rep_, true,
                                path_vec);
  double total_len = Trajectory::getPathLength(path_vec);
  double len_min_thres = kLenMin;
  if (planning_params_.type != PlanningModeType::kVerticalExploration) {
    len_min_thres = kLenMinMin;
  }
  if (total_len <= len_min_thres) {
    ROS_WARN("Best path is too short.");
    return ret;
  }

  std::vector<Vertex*> ref_vertices;
  for (int i = 0; i < best_path.size(); ++i) {
    Eigen::Vector3d p1(best_path[i][0], best_path[i][1], best_path[i][2]);
    Eigen::Vector3d dir_vec = p1 - p0;

    // ERROR: Re-confirm this is a safe path.
    Eigen::Vector3d p_overshoot =
        dir_vec.normalized() * planning_params_.edge_overshoot;
    Eigen::Vector3d p_start = p0 + robot_params_.center_offset - p_overshoot;
    Eigen::Vector3d p_end =
        p0 + robot_params_.center_offset + dir_vec + p_overshoot;
    if ((dir_vec.norm() > 0) &&
        (MapManager::VoxelStatus::kFree !=
         map_manager_->getPathStatus(p_start, p_end, robot_box_size_, true))) {
      ROS_INFO("Segment [%d] is not clear.", i);
    }

    tf::Quaternion quat;
    quat.setEuler(0.0, 0.0, best_path[i][3]);
    tf::Vector3 origin(best_path[i][0], best_path[i][1], best_path[i][2]);
    tf::Pose poseTF(quat, origin);
    geometry_msgs::Pose pose;
    tf::poseTFToMsg(poseTF, pose);
    ret.push_back(pose);

    // Prepare for homing feature.
    ref_vertices.push_back(best_path_vertices[i]);

    double seg_length = (p1 - p0).norm();
    traverse_length += seg_length;
    traverse_time += seg_length / planning_params_.v_max;
    if ((traverse_length > planning_params_.traverse_length_max) ||
        (traverse_time > planning_params_.traverse_time_max)) {
      break;
    }
    p0 = p1;
  }
  ROS_INFO("Best path:  size = %d, length = %f, time = %f", (int)ret.size(),
           traverse_length, traverse_time);
  // Put this into global graph for homing later.
  bool path_added = false;

  // Modify the best path.
  if (planning_params_.path_safety_enhance_enable) {
    ros::Time mod_time;
    START_TIMER(mod_time);
    std::vector<geometry_msgs::Pose> mod_path;
    if (improveFreePath(ret, mod_path)) {
      ret = mod_path;
      addRefPathToGraph(global_graph_, mod_path);
      path_added = true;
    }
    double dmod_time = GET_ELAPSED_TIME(mod_time);
    ROS_WARN("Compute an aternate path in %f(s)", dmod_time);
    visualization_->visualizeModPath(mod_path);
  }

  if (!path_added) {
    addRefPathToGraph(global_graph_, ref_vertices);
  }

  visualization_->visualizeRefPath(ret);
  return ret;
}

bool Rrg::improveFreePath(const std::vector<geometry_msgs::Pose>& path_orig,
                          std::vector<geometry_msgs::Pose>& path_mod) {
  // Few heuristics to improve the path.
  // a) Shorten path by reducing intermidiate nodes. (be careful with turning
  // cases) Shorten/reduce some very short paths to prevent small motion and
  // sudden change in angles.
  // b) Adjust nodes to its neighbors to improve safety
  // c) Ignore leaf node of the path to prevent the robot to come too close the
  // obstacle

  if (path_orig.empty()) return false;

  // Feature a) Remove short intermidiate vertices.
  std::vector<geometry_msgs::Pose> path_mod1 = path_orig;

  const double kSegmentLenMin = 0.5;
  bool cont_refine = true;
  while (cont_refine) {
    cont_refine = false;
    for (int i = 0; i < (path_mod1.size() - 2); ++i) {
      Eigen::Vector3d p_start(path_mod1[i].position.x, path_mod1[i].position.y,
                              path_mod1[i].position.z);
      Eigen::Vector3d p_int(path_mod1[i + 1].position.x,
                            path_mod1[i + 1].position.y,
                            path_mod1[i + 1].position.z);
      Eigen::Vector3d p_end(path_mod1[i + 2].position.x,
                            path_mod1[i + 2].position.y,
                            path_mod1[i + 2].position.z);
      Eigen::Vector3d segment = p_int - p_start;
      double segment_len = segment.norm();
      // ROS_WARN("Segment length %f.", segment_len);
      if (segment_len < kSegmentLenMin) {
        if ((MapManager::VoxelStatus::kFree ==
             map_manager_->getPathStatus(p_start, p_end, robot_box_size_,
                                         true)) &&
            (!planning_params_.geofence_checking_enable ||
             (GeofenceManager::CoordinateStatus::kOK ==
              geofence_manager_->getPathStatus(
                  Eigen::Vector2d(p_start[0], p_start[1]),
                  Eigen::Vector2d(p_end[0], p_end[1]),
                  Eigen::Vector2d(robot_box_size_[0], robot_box_size_[1]))))) {
          // ignore the intermidiate nore, combine the first to the last node.
          ROS_WARN("Combine nodes to remove short segments.");
          path_mod1.erase(path_mod1.begin() + i + 1);
          cont_refine = true;
          break;
        }
      }
    }
  }

  // Implement (b) first: form a safe corridor along each path from cutting
  // hyperplanes.
  feasible_corridor_pcl_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  geometry_msgs::Pose pose0;
  pose0.position.x = path_mod1[0].position.x;
  pose0.position.y = path_mod1[0].position.y;
  pose0.position.z = path_mod1[0].position.z;
  pose0.orientation.x = path_mod1[0].orientation.x;
  pose0.orientation.y = path_mod1[0].orientation.y;
  pose0.orientation.z = path_mod1[0].orientation.z;
  pose0.orientation.w = path_mod1[0].orientation.w;
  path_mod.push_back(pose0);
  bool mod_success = true;
  for (int i = 1; i < path_mod1.size(); ++i) {
    Eigen::Vector3d p0(path_mod1[i - 1].position.x, path_mod1[i - 1].position.y,
                       path_mod1[i - 1].position.z);
    Eigen::Vector3d p0_mod(path_mod[i - 1].position.x,
                           path_mod[i - 1].position.y,
                           path_mod[i - 1].position.z);
    Eigen::Vector3d p1(path_mod1[i].position.x, path_mod1[i].position.y,
                       path_mod1[i].position.z);
    Eigen::Vector3d p1_parallel = p0_mod + p1 - p0;

    Eigen::Vector3d p2;
    bool do_check_p2 = false;
    if (i < path_mod1.size() - 1) {
      do_check_p2 = true;
      p2 = Eigen::Vector3d(path_mod1[i + 1].position.x,
                           path_mod1[i + 1].position.y,
                           path_mod1[i + 1].position.z);
    }

    Eigen::Vector3d p1_target;
    bool seg_free = true;
    if (MapManager::VoxelStatus::kFree ==
        map_manager_->getPathStatus(p0_mod, p1_parallel, robot_box_size_,
                                    true)) {
      p1_target = p1_parallel;
    } else if (MapManager::VoxelStatus::kFree ==
               map_manager_->getPathStatus(p0_mod, p1, robot_box_size_, true)) {
      p1_target = p1;
    } else {
      seg_free = false;
    }

    Eigen::Vector3d p1_mod;
    geometry_msgs::Pose pose;

    Eigen::Vector3d p_center;
    p_center = (p0_mod + p1_target) / 2.0;
    Eigen::Vector3d p_dir;
    p_dir = (p1 - p0);
    double radius = p_dir.norm() / 2.0;
    // add a local bounding box
    Eigen::Vector3d local_bbx(2 * (radius + robot_params_.safety_extension[0]),
                              2 * robot_params_.safety_extension[1],
                              2 * robot_params_.safety_extension[2]);
    std::vector<Eigen::Vector3d> occupied_voxels;
    std::vector<Eigen::Vector3d> free_voxels;
    map_manager_->extractLocalMapAlongAxis(p_center, p_dir, local_bbx,
                                           occupied_voxels, free_voxels);

    pcl::PointCloud<pcl::PointXYZ>* obstacle_pcl(
        new pcl::PointCloud<pcl::PointXYZ>());
    for (auto& v : occupied_voxels) {
      obstacle_pcl->push_back(pcl::PointXYZ(v.x(), v.y(), v.z()));
    }

    if (seg_free && (modifyPath(obstacle_pcl, p0_mod, p1_target, p1_mod))) {
      // Verify collision-free again with the map & geofence.
      if ((MapManager::VoxelStatus::kFree !=
           map_manager_->getPathStatus(p0_mod, p1_mod, robot_box_size_,
                                       true)) ||
          (planning_params_.geofence_checking_enable &&
           (GeofenceManager::CoordinateStatus::kViolated ==
            geofence_manager_->getPathStatus(
                Eigen::Vector2d(p0_mod[0], p0_mod[1]),
                Eigen::Vector2d(p1_mod[0], p1_mod[1]),
                Eigen::Vector2d(robot_box_size_[0], robot_box_size_[1])))) ||
          (do_check_p2 &&
           ((MapManager::VoxelStatus::kFree !=
             map_manager_->getPathStatus(p1_mod, p2, robot_box_size_, true)) ||
            (planning_params_.geofence_checking_enable &&
             (GeofenceManager::CoordinateStatus::kViolated ==
              geofence_manager_->getPathStatus(
                  Eigen::Vector2d(p1_mod[0], p1_mod[1]),
                  Eigen::Vector2d(p2[0], p2[1]),
                  Eigen::Vector2d(robot_box_size_[0],
                                  robot_box_size_[1]))))))) {
        p1_mod = p1;
        mod_success = false;
        ROS_WARN("Newly modified path is not collision-free.");
        break;  // break to save time @recheck
      }
    } else {
      p1_mod = p1;
      // mod_success = false;
      // break; // break to save time @recheck
    }
    pose.position.x = p1_mod.x();
    pose.position.y = p1_mod.y();
    pose.position.z = p1_mod.z();
    path_mod.push_back(pose);
  }

  // Correct the heading angle tangent with the moving direction again.
  // Re-Assign the first heading
  path_mod[0].orientation.x = path_orig[0].orientation.x;
  path_mod[0].orientation.y = path_orig[0].orientation.y;
  path_mod[0].orientation.z = path_orig[0].orientation.z;
  path_mod[0].orientation.w = path_orig[0].orientation.w;
  if ((mod_success) && (planning_params_.yaw_tangent_correction)) {
    for (int i = 1; i < path_mod.size(); ++i) {
      Eigen::Vector3d vec(path_mod[i].position.x - path_mod[i - 1].position.x,
                          path_mod[i].position.y - path_mod[i - 1].position.y,
                          path_mod[i].position.z - path_mod[i - 1].position.z);
      if (planning_params_.planning_backward) vec = -vec;
      double yawhalf = 0.5 * std::atan2(vec[1], vec[0]);
      path_mod[i].orientation.x = 0.0;
      path_mod[i].orientation.y = 0.0;
      path_mod[i].orientation.z = sin(yawhalf);
      path_mod[i].orientation.w = cos(yawhalf);
    }
  }

  visualization_->visualizePCL(feasible_corridor_pcl_.get());
  return mod_success;
}

bool Rrg::addRefPathToGraph(const std::shared_ptr<GraphManager> graph_manager,
                            const std::vector<Vertex*>& vertices) {
  if (vertices.size() <= 0) return false;

  // The whole path is collision free already and start from root vertex.
  // We only need to link the first vertex to the existing graph.
  // Then add the whole path to the graph.
  // Finally, add more edges along the path to the graph.
  StateVec first_state;
  first_state << vertices[0]->state[0], vertices[0]->state[1],
      vertices[0]->state[2], vertices[0]->state[3];
  Vertex* nearest_vertex = NULL;
  if (!graph_manager->getNearestVertex(&first_state, &nearest_vertex))
    return false;
  if (nearest_vertex == NULL) return false;
  Eigen::Vector3d origin(nearest_vertex->state[0], nearest_vertex->state[1],
                         nearest_vertex->state[2]);
  Eigen::Vector3d direction(first_state[0] - origin[0],
                            first_state[1] - origin[1],
                            first_state[2] - origin[2]);
  double direction_norm = direction.norm();
  Vertex* parent_vertex = NULL;
  const double kDeltaLimit = 0.1;
  const double kRadiusLimit = 0.5;

  // Add root vertex first.
  if (direction_norm <= kDeltaLimit) {
    parent_vertex = nearest_vertex;
  } else if (direction_norm <=
             std::max(kRadiusLimit, planning_params_.edge_length_min)) {
    // @TODO: find better way to do this.
    // Blindly add a link/vertex to the graph.
    Vertex* new_vertex =
        new Vertex(graph_manager->generateVertexID(), first_state);
    new_vertex->parent = nearest_vertex;
    new_vertex->distance = nearest_vertex->distance + direction_norm;
    nearest_vertex->children.push_back(new_vertex);
    graph_manager->addVertex(new_vertex);
    graph_manager->addEdge(new_vertex, nearest_vertex, direction_norm);
    parent_vertex = new_vertex;
  } else {
    ROS_WARN("[GlobalGraph] Try to add current state to the graph.");
    ExpandGraphReport rep;
    expandGraph(graph_manager, first_state, rep);
    if (rep.status == ExpandGraphStatus::kSuccess) {
      ROS_WARN("[GlobalGraph] Added successfully.");
      parent_vertex = rep.vertex_added;
    } else {
      ROS_WARN("[GlobalGraph] Can not add current state to the global graph.");
      return false;
    }
  }

  // Add all remaining vertices of the path.
  std::vector<Vertex*> vertex_list;
  vertex_list.push_back(parent_vertex);
  for (int i = 1; i < vertices.size(); ++i) {
    StateVec new_state;
    new_state << vertices[i]->state[0], vertices[i]->state[1],
        vertices[i]->state[2], vertices[i]->state[3];
    Eigen::Vector3d origin(parent_vertex->state[0], parent_vertex->state[1],
                           parent_vertex->state[2]);
    Eigen::Vector3d direction(new_state[0] - origin[0],
                              new_state[1] - origin[1],
                              new_state[2] - origin[2]);
    double direction_norm = direction.norm();

    Vertex* new_vertex =
        new Vertex(graph_manager->generateVertexID(), new_state);
    new_vertex->type = vertices[i]->type;
    new_vertex->parent = parent_vertex;
    new_vertex->distance = parent_vertex->distance + direction_norm;
    parent_vertex->children.push_back(new_vertex);
    graph_manager->addVertex(new_vertex);
    graph_manager->addEdge(new_vertex, parent_vertex, direction_norm);
    vertex_list.push_back(new_vertex);
    parent_vertex = new_vertex;
  }

  // Build edges around vertices if possible to get better path.
  int n_vertices = 0;
  int n_edges = 0;
  // Assume the path is verified collision free.
  for (int i = 0; i < vertex_list.size(); ++i) {
    int num_vertices_added = 0;
    int num_edges_added = 0;
    ExpandGraphReport rep;
    expandGraphEdges(graph_manager, vertex_list[i], rep);
    if (rep.status == ExpandGraphStatus::kSuccess) {
      n_vertices += num_vertices_added;
      n_edges += num_edges_added;
    } else {
      switch (rep.status) {
        case ExpandGraphStatus::kErrorKdTree:
          ROS_WARN("Can not add this vertex: kErrorKdTree.");
          break;
        case ExpandGraphStatus::kErrorCollisionEdge:
          ROS_WARN("Can not add this vertex: kErrorCollisionEdge.");
          break;
        case ExpandGraphStatus::kErrorShortEdge:
          ROS_WARN("Can not add this vertex: kErrorShortEdge.");
          break;
      }
    }
  }

  const bool path_intp_add = true;
  const double intp_len = 1.0;  // m
  if (path_intp_add) {
    // Add some intermidiate vertices along the path to densify the global
    // graph.
    for (int i = 0; i < (vertex_list.size() - 1); ++i) {
      Eigen::Vector3d start_vertex(vertex_list[i]->state.x(),
                                   vertex_list[i]->state.y(),
                                   vertex_list[i]->state.z());
      Eigen::Vector3d end_vertex(vertex_list[i + 1]->state.x(),
                                 vertex_list[i + 1]->state.y(),
                                 vertex_list[i + 1]->state.z());
      Eigen::Vector3d edge_vec = end_vertex - start_vertex;
      double edge_length = edge_vec.norm();
      if (edge_length <= intp_len) continue;
      edge_vec.normalize();
      int n_intp = (int)std::ceil(edge_length / intp_len);  // segments
      Vertex* prev_vertex = vertex_list[i];
      double acc_len = 0;
      for (int j = 1; j < n_intp; ++j) {
        Eigen::Vector3d new_v;
        new_v = start_vertex + j * intp_len * edge_vec;
        StateVec new_state;
        new_state << new_v[0], new_v[1], new_v[2], vertex_list[i]->state[3];
        Vertex* new_vertex =
            new Vertex(graph_manager->generateVertexID(), new_state);
        graph_manager->addVertex(new_vertex);
        graph_manager->addEdge(new_vertex, prev_vertex, intp_len);
        prev_vertex = new_vertex;
        acc_len += intp_len;
      }
      // Link the last connection
      double last_edge_len = edge_length - acc_len;
      graph_manager->addEdge(prev_vertex, vertex_list[i + 1], last_edge_len);
    }
  }

  return true;
}

bool Rrg::addRefPathToGraph(const std::shared_ptr<GraphManager> graph_manager,
                            const std::vector<geometry_msgs::Pose>& path) {
  if (path.size() <= 0) return false;

  // The whole path is collision free already and start from root vertex.
  // We only need to link the first vertex to the existing graph.
  // Then add the whole path to the graph.
  // Finally, add more edges along the path to the graph.

  StateVec first_state;
  first_state << path[0].position.x, path[0].position.y, path[0].position.z,
      0.0;
  Vertex* nearest_vertex = NULL;
  if (!graph_manager->getNearestVertex(&first_state, &nearest_vertex))
    return false;
  if (nearest_vertex == NULL) return false;
  Eigen::Vector3d origin(nearest_vertex->state[0], nearest_vertex->state[1],
                         nearest_vertex->state[2]);
  Eigen::Vector3d direction(first_state[0] - origin[0],
                            first_state[1] - origin[1],
                            first_state[2] - origin[2]);
  double direction_norm = direction.norm();
  Vertex* parent_vertex = NULL;
  const double kDeltaLimit = 0.1;
  const double kRadiusLimit = 0.5;

  // Add root vertex first.
  if (direction_norm <= kDeltaLimit) {
    parent_vertex = nearest_vertex;
  } else if (direction_norm <=
             std::max(kRadiusLimit, planning_params_.edge_length_min)) {
    // @TODO: find better way to do this.
    // Blindly add a link/vertex to the graph.
    Vertex* new_vertex =
        new Vertex(graph_manager->generateVertexID(), first_state);
    new_vertex->parent = nearest_vertex;
    new_vertex->distance = nearest_vertex->distance + direction_norm;
    nearest_vertex->children.push_back(new_vertex);
    graph_manager->addVertex(new_vertex);
    graph_manager->addEdge(new_vertex, nearest_vertex, direction_norm);
    parent_vertex = new_vertex;
  } else {
    ROS_WARN("[GlobalGraph] Try to add current state to the graph.");
    ExpandGraphReport rep;
    expandGraph(graph_manager, first_state, rep);
    if (rep.status == ExpandGraphStatus::kSuccess) {
      ROS_WARN("[GlobalGraph] Added successfully.");
      parent_vertex = rep.vertex_added;
    } else {
      ROS_WARN("[GlobalGraph] Can not add current state to the global graph.");
      return false;
    }
  }

  // Add all remaining vertices of the path.
  std::vector<Vertex*> vertex_list;
  vertex_list.push_back(parent_vertex);
  for (int i = 1; i < path.size(); ++i) {
    StateVec new_state;
    new_state << path[i].position.x, path[i].position.y, path[i].position.z,
        0.0;
    Eigen::Vector3d origin(parent_vertex->state[0], parent_vertex->state[1],
                           parent_vertex->state[2]);
    Eigen::Vector3d direction(new_state[0] - origin[0],
                              new_state[1] - origin[1],
                              new_state[2] - origin[2]);
    double direction_norm = direction.norm();

    Vertex* new_vertex =
        new Vertex(graph_manager->generateVertexID(), new_state);
    // new_vertex->type = vertices[i]->type;
    new_vertex->parent = parent_vertex;
    new_vertex->distance = parent_vertex->distance + direction_norm;
    parent_vertex->children.push_back(new_vertex);
    graph_manager->addVertex(new_vertex);
    graph_manager->addEdge(new_vertex, parent_vertex, direction_norm);
    vertex_list.push_back(new_vertex);
    parent_vertex = new_vertex;
  }

  // Build edges around vertices if possible to get better path.
  int n_vertices = 0;
  int n_edges = 0;
  // Assume the path is verified collision free.
  for (int i = 0; i < vertex_list.size(); ++i) {
    int num_vertices_added = 0;
    int num_edges_added = 0;
    ExpandGraphReport rep;
    expandGraphEdges(graph_manager, vertex_list[i], rep);
    if (rep.status == ExpandGraphStatus::kSuccess) {
      n_vertices += num_vertices_added;
      n_edges += num_edges_added;
    } else {
      switch (rep.status) {
        case ExpandGraphStatus::kErrorKdTree:
          ROS_WARN("Can not add this vertex: kErrorKdTree.");
          break;
        case ExpandGraphStatus::kErrorCollisionEdge:
          ROS_WARN("Can not add this vertex: kErrorCollisionEdge.");
          break;
        case ExpandGraphStatus::kErrorShortEdge:
          ROS_WARN("Can not add this vertex: kErrorShortEdge.");
          break;
      }
    }
  }

  const bool path_intp_add = true;
  const double intp_len = 1.0;  // m
  if (path_intp_add) {
    // Add some intermidiate vertices along the path to densify the global
    // graph.
    for (int i = 0; i < (vertex_list.size() - 1); ++i) {
      Eigen::Vector3d start_vertex(vertex_list[i]->state.x(),
                                   vertex_list[i]->state.y(),
                                   vertex_list[i]->state.z());
      Eigen::Vector3d end_vertex(vertex_list[i + 1]->state.x(),
                                 vertex_list[i + 1]->state.y(),
                                 vertex_list[i + 1]->state.z());
      Eigen::Vector3d edge_vec = end_vertex - start_vertex;
      double edge_length = edge_vec.norm();
      if (edge_length <= intp_len) continue;
      edge_vec.normalize();
      int n_intp = (int)std::ceil(edge_length / intp_len);  // segments
      Vertex* prev_vertex = vertex_list[i];
      double acc_len = 0;
      for (int j = 1; j < n_intp; ++j) {
        Eigen::Vector3d new_v;
        new_v = start_vertex + j * intp_len * edge_vec;
        StateVec new_state;
        new_state << new_v[0], new_v[1], new_v[2], vertex_list[i]->state[3];
        Vertex* new_vertex =
            new Vertex(graph_manager->generateVertexID(), new_state);
        graph_manager->addVertex(new_vertex);
        graph_manager->addEdge(new_vertex, prev_vertex, intp_len);
        prev_vertex = new_vertex;
        acc_len += intp_len;
      }
      // Link the last connection
      double last_edge_len = edge_length - acc_len;
      graph_manager->addEdge(prev_vertex, vertex_list[i + 1], last_edge_len);
    }
  }
  return true;
}

void Rrg::setState(StateVec& state) {
  if (!odometry_ready) {
    // First time receive the pose/odometry for planning purpose.
    // Reset the octomap
    ROS_WARN("Received the first odometry, reset the map");
    map_manager_->resetMap();
  }
  current_state_ = state;
  odometry_ready = true;
  // Clear free space based on current voxel size.
  if (planner_trigger_time_ < planning_params_.augment_free_voxels_time) {
    map_manager_->augmentFreeBox(
        Eigen::Vector3d(current_state_[0], current_state_[1],
                        current_state_[2]) +
            robot_params_.center_offset,
        robot_box_size_);
  }
  if (robot_backtracking_queue_.size()) {
    if (robot_backtracking_queue_.size() >= backtracking_queue_max_size) {
      robot_backtracking_queue_.pop();
    }
    robot_backtracking_queue_.emplace(current_state_);
  } else {
    robot_backtracking_queue_.emplace(state);
  }
}

void Rrg::timerCallback(const ros::TimerEvent& event) {
  // Re-initialize until get non-zero value.
  if (rostime_start_.toSec() == 0) rostime_start_ = ros::Time::now();

  if (!odometry_ready) {
    ROS_WARN("Planner is waiting for odometry");
    return;
  }

  // Estimate the exploring direction.
  // Use the position direction rather than the heading direction.
  constexpr int kQueueMaxSize = 10;
  constexpr double kAlpha = 0.3;  // favor newest paths.
  constexpr double kMinDist = 0.75;
  if (robot_state_queue_.size()) {
    StateVec& last_state = robot_state_queue_.back();
    Eigen::Vector3d cur_dir(current_state_[0] - last_state[0],
                            current_state_[1] - last_state[1],
                            0.0);  // ignore changes in z-axis
    if (cur_dir.norm() >= kMinDist) {
      double yaw = atan2(cur_dir[1], cur_dir[0]);
      double dyaw = yaw - exploring_direction_;
      truncateYaw(dyaw);
      exploring_direction_ = exploring_direction_ + (1 - kAlpha) * dyaw;
      truncateYaw(exploring_direction_);
      if (robot_state_queue_.size() >= kQueueMaxSize) {
        robot_state_queue_.pop();
      }
      robot_state_queue_.emplace(current_state_);
    }
  } else {
    robot_state_queue_.emplace(current_state_);
  }

  // Enforce edges from dometry.
  bool enforce_vertex_from_odometry = true;
  constexpr double kOdoEnforceLength = 0.5;
  if (enforce_vertex_from_odometry) {
    while (robot_backtracking_queue_.size()) {
      StateVec bt_state = robot_backtracking_queue_.front();
      robot_backtracking_queue_.pop();
      if (robot_backtracking_prev_ == NULL)
        global_graph_->getNearestVertex(&bt_state, &robot_backtracking_prev_);
      if (robot_backtracking_prev_) {
        Eigen::Vector3d cur_dir(
            bt_state[0] - robot_backtracking_prev_->state[0],
            bt_state[1] - robot_backtracking_prev_->state[1],
            bt_state[2] - robot_backtracking_prev_->state[2]);
        double dir_norm = cur_dir.norm();
        if (dir_norm >= kOdoEnforceLength) {
          Vertex* new_vertex =
              new Vertex(global_graph_->generateVertexID(), bt_state);
          new_vertex->parent = robot_backtracking_prev_;
          new_vertex->distance = robot_backtracking_prev_->distance + dir_norm;
          global_graph_->addVertex(new_vertex);
          // could increase the distance to limit shortest paths along these
          // edges for safety purpose since we don't check the collision
          const double kEdgeWeightExtended = 1.0;
          global_graph_->addEdge(new_vertex, robot_backtracking_prev_,
                                 dir_norm * kEdgeWeightExtended);
          robot_backtracking_prev_ = new_vertex;
        }
      }
    }
  }

  // Get position from odometry to add more vertices to the graph for homing.
  Eigen::Vector3d cur_dir(current_state_[0] - last_state_marker_[0],
                          current_state_[1] - last_state_marker_[1],
                          current_state_[2] - last_state_marker_[2]);

  constexpr double kOdoUpdateMinLength = kOdoEnforceLength;
  if (cur_dir.norm() >= kOdoUpdateMinLength) {
    // Add this to the global graph.
    const bool add_odometry_to_global_graph = true;
    if (add_odometry_to_global_graph) {
      StateVec new_state;
      new_state = current_state_;
      ExpandGraphReport rep;
      expandGraph(global_graph_, new_state, rep);
      // ROS_INFO("From odometry, added %d vertices and %d edges",
      // rep.num_vertices_added, rep.num_edges_added);
    }
    last_state_marker_ = current_state_;
  }

  constexpr double kMinLength = 1.0;
  Eigen::Vector3d cur_dir1(current_state_[0] - last_state_marker_global_[0],
                           current_state_[1] - last_state_marker_global_[1],
                           current_state_[2] - last_state_marker_global_[2]);
  if (cur_dir1.norm() >= kMinLength) {
    // Instead of adding all odometry to the graph which will increase the graph
    // significantly. Let's store them in a database, then add later if
    // requires. Get position from odometry to add more vertices to the graph
    // for homing.
    // constexpr double kMinLength = 1.0;
    constexpr double kUpdateRadius = 3.0;
    StateVec* state_add = new StateVec(current_state_[0], current_state_[1],
                                       current_state_[2], current_state_[3]);
    robot_state_hist_->addState(state_add);
    const bool apply_eventE1 = true;
    if (apply_eventE1) {
      global_graph_->updateVertexTypeInRange(current_state_,
                                             kUpdateRadius);  // E1
    }
    last_state_marker_global_ = current_state_;
  }
}

void Rrg::setBoundMode(explorer::BoundModeType bmode) {
  constexpr double kNumVerticesRatio = 1.3;
  constexpr double kNumEdgesRatio = 1.3;
  robot_params_.setBoundMode(bmode);
  // Update the robot size for planning.
  robot_params_.getPlanningSize(robot_box_size_);
  // Change the number of vertices allowed as well. @HACK: any better way to do?
  switch (bmode) {
    case BoundModeType::kExtendedBound:
      planning_num_vertices_max_ = planning_params_.num_vertices_max;
      planning_num_edges_max_ = planning_params_.num_edges_max;
      break;
    case BoundModeType::kRelaxedBound:
      planning_num_vertices_max_ =
          (int)((double)planning_params_.num_vertices_max * kNumVerticesRatio);
      planning_num_edges_max_ =
          (int)((double)planning_params_.num_edges_max * kNumEdgesRatio);
      break;
    case BoundModeType::kMinBound:
      planning_num_vertices_max_ =
          (int)((double)planning_params_.num_vertices_max * kNumVerticesRatio *
                kNumVerticesRatio);
      planning_num_edges_max_ = (int)((double)planning_params_.num_edges_max *
                                      kNumEdgesRatio * kNumEdgesRatio);
      break;
  }
}

bool Rrg::compareAngles(double dir_angle_a, double dir_angle_b, double thres) {
  double dyaw = dir_angle_a - dir_angle_b;
  if (dyaw > M_PI)
    dyaw -= 2 * M_PI;
  else if (dyaw < -M_PI)
    dyaw += 2 * M_PI;

  if (std::abs(dyaw) <= thres) {
    return true;
  } else {
    return false;
  }
}

bool Rrg::comparePathWithDirectionApprioximately(
    const std::vector<geometry_msgs::Pose>& path, double yaw) {
  const double kMinSegmentLen = 2.0;
  const double kYawThres = 0.5 * M_PI;

  if (path.size() <= 1) return true;
  // Get aprpoximate direction.
  Eigen::Vector3d root_pos(path[0].position.x, path[0].position.y,
                           path[0].position.z);
  double path_yaw = 0;
  for (int i = 1; i < path.size(); ++i) {
    Eigen::Vector3d dir_vec;
    dir_vec << path[i].position.x - root_pos.x(),
        path[i].position.y - root_pos.y(),
        0.0;  // ignore z
    if (dir_vec.norm() > kMinSegmentLen) {
      path_yaw = std::atan2(dir_vec.y(), dir_vec.x());
      break;
    }
  }

  double dyaw = path_yaw - yaw;
  if (dyaw > M_PI)
    dyaw -= 2 * M_PI;
  else if (dyaw < -M_PI)
    dyaw += 2 * M_PI;

  const double yaw_thres = 0.5 * M_PI;
  if (std::abs(dyaw) <= kYawThres) {
    return true;
  } else {
    return false;
  }
}

std::vector<int> Rrg::performShortestPathsClustering(
    const std::shared_ptr<GraphManager> graph_manager,
    const ShortestPathsReport& graph_rep, std::vector<Vertex*>& vertices,
    double dist_threshold, double principle_path_min_length,
    bool refinement_enable) {
  // Asumme long paths are principle paths.
  // Go over one by one from the longest ones.
  // Group into clusters based on the normalized DTW distance metric.
  // Refine by choosing closest & valid principle path.

  // Sort into descending order.
  std::sort(vertices.begin(), vertices.end(),
            [&graph_manager, &graph_rep](const Vertex* a, const Vertex* b) {
              return graph_manager->getShortestDistance(a->id, graph_rep) >
                     graph_manager->getShortestDistance(b->id, graph_rep);
            });

  std::vector<std::vector<Eigen::Vector3d>> cluster_paths;
  std::vector<int> cluster_ids;
  for (int i = 0; i < vertices.size(); ++i) {
    std::vector<Eigen::Vector3d> path_cur;
    graph_manager->getShortestPath(vertices[i]->id, graph_rep, true, path_cur);
    bool found_a_neigbor = false;
    for (int j = 0; j < cluster_paths.size(); ++j) {
      if (Trajectory::compareTwoTrajectories(path_cur, cluster_paths[j],
                                             dist_threshold)) {
        vertices[i]->cluster_id = cluster_ids[j];
        found_a_neigbor = true;
        break;
      }
    }
    if (!found_a_neigbor) {
      // Can not find any neigbor, set this as a new principle path.
      cluster_paths.emplace_back(path_cur);
      cluster_ids.push_back(vertices[i]->id);
      vertices[i]->cluster_id = vertices[i]->id;
    }
  }

  ROS_INFO("Cluster %d paths into %d clusters.", (int)vertices.size(),
           (int)cluster_paths.size());

  // Refinement step, remove short path and choose closest cluster.
  if (refinement_enable) {
    // Clean out noisy paths by removing short path.
    std::vector<std::vector<Eigen::Vector3d>> cluster_paths_refine;
    std::vector<int> cluster_ids_refine;
    for (int j = 0; j < cluster_paths.size(); ++j) {
      double path_len = Trajectory::getPathLength(cluster_paths[j]);
      if (path_len >= principle_path_min_length) {
        cluster_paths_refine.push_back(cluster_paths[j]);
        cluster_ids_refine.push_back(cluster_ids[j]);
      }
    }
    // Recheck and choose closest one.
    int cluster_paths_size = cluster_paths.size();
    for (int i = 0; i < vertices.size(); ++i) {
      std::vector<Eigen::Vector3d> path_cur;
      graph_manager->getShortestPath(vertices[i]->id, graph_rep, true,
                                     path_cur);
      double dist_min = std::numeric_limits<double>::infinity();
      for (int j = 0; j < cluster_paths_refine.size(); ++j) {
        double dist_score = Trajectory::computeDistanceBetweenTwoTrajectories(
            path_cur, cluster_paths_refine[j]);
        if (dist_min > dist_score) {
          dist_min = dist_score;
          vertices[i]->cluster_id = cluster_ids_refine[j];
        }
      }
    }
    ROS_INFO("Clustering with refinement %d paths into %d clusters.",
             (int)vertices.size(), (int)cluster_paths_refine.size());
    return cluster_ids_refine;
  } else {
    return cluster_ids;
  }
}

bool Rrg::connectStateToGraph(std::shared_ptr<GraphManager> graph,
                              StateVec& cur_state, Vertex*& v_added,
                              double dist_ignore_collision_check) {
  Vertex* nearest_vertex = NULL;
  if (!graph->getNearestVertex(&cur_state, &nearest_vertex)) return false;
  if (nearest_vertex == NULL) return false;
  Eigen::Vector3d origin(nearest_vertex->state[0], nearest_vertex->state[1],
                         nearest_vertex->state[2]);
  Eigen::Vector3d direction(cur_state[0] - origin[0], cur_state[1] - origin[1],
                            cur_state[2] - origin[2]);
  double direction_norm = direction.norm();
  bool connect_state_to_graph = true;
  const double kDelta = 0.05;
  if (direction_norm <= kDelta) {
    // Add edges only from this vertex.
    ExpandGraphReport rep;
    expandGraphEdges(graph, nearest_vertex, rep);
    v_added = nearest_vertex;
  } else if (direction_norm <= std::max(dist_ignore_collision_check,
                                        planning_params_.edge_length_min)) {
    // Blindly add a link/vertex to the graph if small radius.
    Vertex* new_vertex = new Vertex(graph->generateVertexID(), cur_state);
    new_vertex->parent = nearest_vertex;
    new_vertex->distance = nearest_vertex->distance + direction_norm;
    graph->addVertex(new_vertex);
    graph->addEdge(new_vertex, nearest_vertex, direction_norm);
    // Add edges only from this vertex.
    ExpandGraphReport rep;
    expandGraphEdges(graph, new_vertex, rep);
    v_added = new_vertex;
  } else {
    ROS_WARN("[GlobalGraph] Try to add current state to the graph.");
    ExpandGraphReport rep;
    expandGraph(graph, cur_state, rep);
    if (rep.status == ExpandGraphStatus::kSuccess) {
      ROS_WARN("[GlobalGraph] Added successfully.");
      v_added = rep.vertex_added;
    } else {
      // Not implemented solution for this case yet.
      connect_state_to_graph = false;
      ROS_WARN("[GlobalGraph] Can not add current state to graph since: ");
      switch (rep.status) {
        case ExpandGraphStatus::kErrorKdTree:
          ROS_WARN("kErrorKdTree.");
          break;
        case ExpandGraphStatus::kErrorCollisionEdge:
          ROS_WARN("kErrorCollisionEdge.");
          break;
        case ExpandGraphStatus::kErrorShortEdge:
          ROS_WARN("kErrorShortEdge.");
          break;
      }
    }
  }
  return connect_state_to_graph;
}

double Rrg::getTimeElapsed() {
  double time_elapsed = 0.0;
  if ((ros::Time::now()).toSec() != 0.0) {
    if (rostime_start_.toSec() == 0.0) rostime_start_ = ros::Time::now();
    time_elapsed = (double)((ros::Time::now() - rostime_start_).toSec());
  }
  return time_elapsed;
}

double Rrg::getTimeRemained() {
  double time_budget_remaining =
      planning_params_.time_budget_limit - getTimeElapsed();
  // Check 2 conditions (time budget vs battery) whatever which one comes first.
  return std::min(time_budget_remaining, current_battery_time_remaining_);
}

bool Rrg::isRemainingTimeSufficient(const double& time_cost,
                                    double& time_spare) {
  const double kTimeDelta = 20;  // magic number, extra safety
  time_spare = getTimeRemained() - time_cost;
  if (time_spare < kTimeDelta) return false;
  return true;
}

std::vector<geometry_msgs::Pose> Rrg::runGlobalPlanner(int vertex_id,
                                                       bool not_check_frontier,
                                                       bool ignore_time) {
  // @not_check_frontier: just check if it is feasible (collision-free + time)
  // @ignore_time: don't consider time budget.
  // Check if exists any frontier in the global graph
  // Get the list of current frontiers.
  ros::Duration(3.0).sleep();  // sleep to unblock the thread to get update
  ros::spinOnce();

  START_TIMER(ttime);
  std::vector<geometry_msgs::Pose> ret_path;
  ret_path.clear();

  // Check if the global planner exists
  if (global_graph_->getNumVertices() <= 1) {
    ROS_WARN("[GlobalGraph] Graph is empty, nothing to search.");
    return ret_path;
  }

  // Check if the vertex id exists
  if ((vertex_id < 0) || (vertex_id >= global_graph_->getNumVertices())) {
    ROS_WARN(
        "[GlobalGraph] Vertex ID doesn't exist, plz consider IDs in the range "
        "[0-%d].",
        global_graph_->getNumVertices() - 1);
    return ret_path;
  }

  // Check if the time endurance is still available.
  if (!ignore_time) {
    if (getTimeRemained() <= 0.0) {
      ROS_WARN("[Global] RAN OUT OF TIME --> STOP HERE.");
      return ret_path;
    }
  }

  // Check if exists any frontiers in the graph.
  // Re-update all the frontiers based on the volumetric gain.
  std::vector<Vertex*> global_frontiers;
  int num_vertices = global_graph_->getNumVertices();
  ROS_INFO("Re-check all frontiers.");
  global_frontiers.clear();
  for (int id = 0; id < num_vertices; ++id) {
    if (global_graph_->getVertex(id)->type == VertexType::kFrontier) {
      Vertex* v = global_graph_->getVertex(id);
      computeVolumetricGainRayModelNoBound(v->state, v->vol_gain);
      if (!v->vol_gain.is_frontier)
        v->type = VertexType::kUnvisited;
      else
        global_frontiers.push_back(global_graph_->getVertex(id));
    }
  }
  ROS_INFO("Currently have %d frontiers in the global graph.",
           (int)global_frontiers.size());
  if ((!not_check_frontier) && (global_frontiers.size() <= 0)) {
    ROS_INFO(
        "No frontier exists --> Could call HOMING instead if fully explored.");
    return ret_path;
  }

  // Let's try to add current state to the global graph.
  ROS_WARN("Trying to add new vertex from current position.");
  StateVec cur_state;
  cur_state << current_state_[0], current_state_[1], current_state_[2],
      current_state_[3];
  Vertex* link_vertex = NULL;
  const double kRadiusLimit = 1.5;  // 0.5
  bool connected_to_graph =
      connectStateToGraph(global_graph_, cur_state, link_vertex, kRadiusLimit);

  if (!connected_to_graph) {
    ROS_WARN("Cannot add the state to the global graph.");
    return ret_path;
  }

  ROS_WARN(
      "Added current state to the graph. Start searching for the global path "
      "now.");
  // Get Dijsktra path from home to all.
  if (!global_graph_->findShortestPaths(global_graph_rep_)) {
    ROS_ERROR("[GlobalGraph] Failed to find shortest path.");
    return ret_path;
  }
  // Get Dijsktra path from current to all.
  ShortestPathsReport frontier_graph_rep;
  if (!global_graph_->findShortestPaths(link_vertex->id, frontier_graph_rep)) {
    ROS_ERROR("[GlobalGraph] Failed to find shortest path.");
    return ret_path;
  }
  // Check if the planner should find the best vertex automatically or manually
  double best_gain = -1.0;
  Vertex* best_frontier = NULL;
  if (vertex_id) {
    // Manual mode
    // Just need to check if it is feasible
    std::vector<int> current_to_target_path_id;
    std::vector<int> target_to_home_path_id;
    double current_to_target_distance;
    double target_to_home_distance;
    global_graph_->getShortestPath(vertex_id, frontier_graph_rep, true,
                                   current_to_target_path_id);
    global_graph_->getShortestPath(vertex_id, global_graph_rep_, false,
                                   target_to_home_path_id);
    current_to_target_distance =
        global_graph_->getShortestDistance(vertex_id, frontier_graph_rep);
    target_to_home_distance =
        global_graph_->getShortestDistance(vertex_id, global_graph_rep_);
    // Estimate time
    double time_remaining = getTimeRemained();
    double time_to_target =
        current_to_target_distance / planning_params_.v_homing_max;
    double time_to_home =
        target_to_home_distance / planning_params_.v_homing_max;
    double time_cost = 0;
    if (!ignore_time) {
      ROS_INFO("[Global] Time remaining: %f (sec)", time_remaining);
      time_cost += time_to_target;
      ROS_INFO("[Global] Time to [%3d]: %f (sec)", vertex_id, time_to_target);
      if (planning_params_.auto_homing_enable) {
        time_cost += time_to_home;
        ROS_INFO("[Global] Time to home  : %f (sec)", time_to_home);
      }
    }
    double time_spare = 0;
    if (!isRemainingTimeSufficient(time_cost, time_spare)) {
      ROS_WARN("[Global] Not enough time to go the vertex [%d]", vertex_id);
      ROS_WARN("[Global] Consider another ID or set ignore_time to True");
      return ret_path;
    }
    best_frontier = global_graph_->getVertex(vertex_id);
    best_gain = 1.0;
  } else {
    // Auto mode
    // Get list of feasible frontiers by checking remaining time.
    // Leave the check empty for now since it relate to time budget setting.
    std::vector<Vertex*> feasible_global_frontiers;
    for (auto& f : global_frontiers) {
      if (ignore_time)
        feasible_global_frontiers.push_back(f);
      else {
        // get gain.
        std::vector<int> current_to_frontier_path_id;
        std::vector<int> frontier_to_home_path_id;
        double current_to_frontier_distance;
        double frontier_to_home_distance;

        global_graph_->getShortestPath(f->id, frontier_graph_rep, true,
                                       current_to_frontier_path_id);
        global_graph_->getShortestPath(f->id, global_graph_rep_, false,
                                       frontier_to_home_path_id);
        current_to_frontier_distance =
            global_graph_->getShortestDistance(f->id, frontier_graph_rep);
        frontier_to_home_distance =
            global_graph_->getShortestDistance(f->id, global_graph_rep_);

        double time_remaining = getTimeRemained();
        double time_to_target =
            current_to_frontier_distance / planning_params_.v_homing_max;
        double time_to_home =
            frontier_to_home_distance / planning_params_.v_homing_max;
        double time_cost = time_to_target + planning_params_.auto_homing_enable
                               ? time_to_home
                               : 0;
        double time_spare = 0;
        if (isRemainingTimeSufficient(time_cost, time_spare)) {
          feasible_global_frontiers.push_back(f);
        }
      }
    }
    ROS_INFO("There are %d potential frontiers, get %d feasible frontiers.",
        (int)global_frontiers.size(), (int)feasible_global_frontiers.size());
    if (feasible_global_frontiers.size() <= 0) {
      ROS_WARN("No feasible frontier-->Call HOMING instead if fully explored.");
      return ret_path;
    }

    // Compute exploration gain.
    std::unordered_map<int, double> frontier_exp_gain;
    for (int i = 0; i < feasible_global_frontiers.size(); ++i) {
      Vertex* f = feasible_global_frontiers[i];
      // get gain.
      std::vector<int> current_to_frontier_path_id;
      std::vector<int> frontier_to_home_path_id;
      double current_to_frontier_distance;
      double frontier_to_home_distance;

      global_graph_->getShortestPath(f->id, frontier_graph_rep, true,
                                     current_to_frontier_path_id);
      global_graph_->getShortestPath(f->id, global_graph_rep_, false,
                                     frontier_to_home_path_id);
      current_to_frontier_distance =
          global_graph_->getShortestDistance(f->id, frontier_graph_rep);
      frontier_to_home_distance =
          global_graph_->getShortestDistance(f->id, global_graph_rep_);

      // Duplication from above but easier to understand.
      double time_to_target =
          current_to_frontier_distance / planning_params_.v_homing_max;
      double time_to_home =
          frontier_to_home_distance / planning_params_.v_homing_max;
      double time_cost = time_to_target + planning_params_.auto_homing_enable
                             ? time_to_home
                             : 0;
      double time_spare = 0;
      if (!isRemainingTimeSufficient(time_cost, time_spare)) {
        time_spare = 1;
      }

      const double kGDistancePenalty = 0.01;
      double exp_gain = f->vol_gain.gain *
                        exp(-kGDistancePenalty * current_to_frontier_distance);
      if (!ignore_time) exp_gain *= time_spare;
      frontier_exp_gain[f->id] = exp_gain;
      if (exp_gain > best_gain) {
        best_gain = exp_gain;
        best_frontier = f;
      }
    }

    // Rank from the best one.
    // Sort into descending order.
    std::sort(feasible_global_frontiers.begin(),
              feasible_global_frontiers.end(),
              [&frontier_exp_gain](const Vertex* a, const Vertex* b) {
                return frontier_exp_gain[a->id] > frontier_exp_gain[b->id];
              });
  }

  std::vector<int> current_to_frontier_path_id;
  std::vector<int> frontier_to_home_path_id;
  if (best_gain >= 0) {
    ROS_WARN("Found the best frontier to go is: %d", best_frontier->id);

    global_graph_->getShortestPath(best_frontier->id, frontier_graph_rep, true,
                                   current_to_frontier_path_id);
    global_graph_->getShortestPath(best_frontier->id, global_graph_rep_, false,
                                   frontier_to_home_path_id);
    int current_to_frontier_path_id_size = current_to_frontier_path_id.size();
    for (int i = 0; i < current_to_frontier_path_id_size; ++i) {
      StateVec state =
          global_graph_->getVertex(current_to_frontier_path_id[i])->state;
      tf::Quaternion quat;
      quat.setEuler(0.0, 0.0, state[3]);
      tf::Vector3 origin(state[0], state[1], state[2]);
      tf::Pose poseTF(quat, origin);
      geometry_msgs::Pose pose;
      tf::poseTFToMsg(poseTF, pose);
      ret_path.push_back(pose);
    }
  } else {
    ROS_WARN(
        "Could not find any positive gain (Should not happen) --> Try with "
        "HOMING.");
    return ret_path;
  }

  // Set the heading angle tangent with the moving direction,
  // from the second waypoint; the first waypoint keeps the same direction.
  if (planning_params_.yaw_tangent_correction) {
    for (int i = 0; i < (ret_path.size() - 1); ++i) {
      Eigen::Vector3d vec(ret_path[i + 1].position.x - ret_path[i].position.x,
                          ret_path[i + 1].position.y - ret_path[i].position.y,
                          ret_path[i + 1].position.z - ret_path[i].position.z);
      double yaw = std::atan2(vec[1], vec[0]);
      tf::Quaternion quat;
      quat.setEuler(0.0, 0.0, yaw);
      ret_path[i + 1].orientation.x = quat.x();
      ret_path[i + 1].orientation.y = quat.y();
      ret_path[i + 1].orientation.z = quat.z();
      ret_path[i + 1].orientation.w = quat.w();
    }
  }

  // Modify path if required
  if (planning_params_.path_safety_enhance_enable) {
    ros::Time mod_time;
    START_TIMER(mod_time);
    std::vector<geometry_msgs::Pose> mod_path;
    if (improveFreePath(ret_path, mod_path)) {
      ret_path = mod_path;
    }
    double dmod_time = GET_ELAPSED_TIME(mod_time);
    ROS_WARN("Compute an aternate path for homing in %f(s)", dmod_time);
    visualization_->visualizeModPath(mod_path);
  }

  visualization_->visualizeGlobalPaths(
      global_graph_, current_to_frontier_path_id, frontier_to_home_path_id);

  double dtime = GET_ELAPSED_TIME(ttime);
  ROS_WARN("runGlobalPlanner costs: %f (s)", dtime);

  visualization_->visualizeGlobalGraph(global_graph_);
  visualization_->visualizeRefPath(ret_path);
  return ret_path;
}

void Rrg::addGeofenceAreas(const geometry_msgs::PolygonStamped& polygon_msgs) {
  if ((planning_params_.geofence_checking_enable) && (planner_trigger_time_)) {
    // Check if we need to convert to global coordinate to be compatible
    // with the whole planner.
    if (!polygon_msgs.header.frame_id.compare(
            planning_params_.global_frame_id)) {
      geofence_manager_->addGeofenceArea(polygon_msgs.polygon);
    } else {
      geometry_msgs::Polygon polygon;
      // Look up the tf.
      tf::StampedTransform tf_to_global;
      tf::TransformListener listener;
      try {
        listener.waitForTransform(planning_params_.global_frame_id,
                                  polygon_msgs.header.frame_id, ros::Time(0),
                                  ros::Duration(0.1));  // this should be fast.
        listener.lookupTransform(planning_params_.global_frame_id,
                                 polygon_msgs.header.frame_id, ros::Time(0),
                                 tf_to_global);
        for (int i = 0; i < polygon_msgs.polygon.points.size(); ++i) {
          tf::Vector3 poly_in_global;
          poly_in_global.setValue(polygon_msgs.polygon.points[i].x,
                                  polygon_msgs.polygon.points[i].y,
                                  polygon_msgs.polygon.points[i].z);
          poly_in_global = tf_to_global * poly_in_global;
          geometry_msgs::Point32 p32;
          p32.x = poly_in_global.x();
          p32.y = poly_in_global.y();
          p32.z = poly_in_global.z();
          polygon.points.push_back(p32);
        }
        geofence_manager_->addGeofenceArea(polygon);
      } catch (tf::TransformException ex) {
        ROS_WARN(
            "Could not look up TF from polygon frame [%s] to the global frame "
            "[%s].",
            polygon_msgs.header.frame_id.c_str(),
            planning_params_.global_frame_id.c_str());
      }
    }
  }
}

bool Rrg::setExpMode(planner_msgs::PlanningMode& exp_mode) {
  PlanningModeType pmode = (PlanningModeType)exp_mode.mode;
  if ((pmode == PlanningModeType::kVerticalExploration) &&
      (!random_sampler_vertical_.isReady)) {
    ROS_WARN(
        "No setting for vertical exp mode --> cannot switch to vertical "
        "mode.");
    return false;
  }
  planning_params_.setPlanningMode((PlanningModeType)exp_mode.mode);
  return true;
}

bool Rrg::loadGraph(const std::string& path) {
  global_graph_->loadGraph(path);
  visualization_->visualizeGlobalGraph(global_graph_);
  return true;
}

bool Rrg::saveGraph(const std::string& path) {
  visualization_->visualizeGlobalGraph(global_graph_);
  global_graph_->saveGraph(path);
  return true;
}

void Rrg::setGlobalFrame(std::string frame_id) {
  // Mainly use to set the frame_id for visualization.
  if (!frame_id.empty()) visualization_->setGlobalFrame(frame_id);
}

RobotStateHistory::RobotStateHistory() {
  kd_tree_ = NULL;
  reset();
}

void RobotStateHistory::reset() {
  // Reset kdtree first.
  if (kd_tree_) kd_free(kd_tree_);
  kd_tree_ = kd_create(3);
}

void RobotStateHistory::addState(StateVec* s) {
  kd_insert3(kd_tree_, s->x(), s->y(), s->z(), s);
  state_hist_.push_back(s);
}

bool RobotStateHistory::getNearestState(const StateVec* state,
                                        StateVec** s_res) {
  // it seems kdtree lib  can not deal with empty tree, put a guard check here.
  if (state_hist_.size() == 0) return false;
  kdres* nearest = kd_nearest3(kd_tree_, state->x(), state->y(), state->z());
  if (kd_res_size(nearest) <= 0) {
    kd_res_free(nearest);
    return false;
  }
  *s_res = (StateVec*)kd_res_item_data(nearest);
  kd_res_free(nearest);
  return true;
}

bool RobotStateHistory::getNearestStates(const StateVec* state, double range,
                                         std::vector<StateVec*>* s_res) {
  // Notice that this might include the same vertex in the result.
  // if that vertex is added to the tree before.
  // Use the distance 0 or small threshold to filter out.
  if (state_hist_.size() == 0) return false;
  kdres* neighbors =
      kd_nearest_range3(kd_tree_, state->x(), state->y(), state->z(), range);
  int neighbors_size = kd_res_size(neighbors);
  if (neighbors_size <= 0) return false;
  s_res->clear();
  for (int i = 0; i < neighbors_size; ++i) {
    StateVec* new_neighbor = (StateVec*)kd_res_item_data(neighbors);
    s_res->push_back(new_neighbor);
    if (kd_res_next(neighbors) <= 0) break;
  }
  kd_res_free(neighbors);
  return true;
}

bool RobotStateHistory::getNearestStateInRange(const StateVec* state,
                                               double range, StateVec** s_res) {
  if (state_hist_.size() == 0) return false;
  kdres* nearest = kd_nearest3(kd_tree_, state->x(), state->y(), state->z());
  if (kd_res_size(nearest) <= 0) {
    kd_res_free(nearest);
    return false;
  }
  *s_res = (StateVec*)kd_res_item_data(nearest);
  Eigen::Vector3d dist;
  dist << state->x() - (*s_res)->x(), state->y() - (*s_res)->y(),
      state->z() - (*s_res)->z();
  kd_res_free(nearest);
  if (dist.norm() > range) return false;
  return true;
}

}  // namespace gbplanner
}  // namespace explorer
