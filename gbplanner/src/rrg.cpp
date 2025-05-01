#include "gbplanner/rrg.h"

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/common/transforms.h>
#include <tf/transform_listener.h>

#define SQ(x) (x * x)

// namespace explorer {

Rrg::Rrg(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  map_manager_ =
      new MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>(
          nh_, nh_private_);

  adaptive_obb_ = new AdaptiveObb(map_manager_);

  initializeAttributes();
}

Rrg::Rrg(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
         MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>*
             map_manager)
    : nh_(nh), nh_private_(nh_private), map_manager_(map_manager) {
  adaptive_obb_ = new AdaptiveObb(map_manager_);

  initializeAttributes();
}

void Rrg::initializeAttributes() {
  visualization_ = new Visualization(nh_, nh_private_);

  geofence_manager_.reset(new GeofenceManager());

  manhole_detector_.reset(new ManholeDetector(nh_, nh_private_));

  // Initialize graphs.
  global_graph_.reset(new GraphManager());
  local_graph_.reset(new GraphManager());
  projected_graph_.reset(new GraphManager());

  //
  robot_state_hist_.reset(new RobotStateHistory());

  // Others.
  stat_.reset(new SampleStatistic());
  stat_chrono_.reset(new SampleStatistic());
  planner_trigger_count_ = 0;
  current_battery_time_remaining_ = std::numeric_limits<double>::max();
  rostime_start_ = ros::Time::now();
  //
  add_frontiers_to_global_graph_ = false;
  //
  exploring_direction_ = 0.0;
  periodic_timer_ =
      nh_.createTimer(ros::Duration(kTimerPeriod), &Rrg::timerCallback, this);
  odometry_ready = false;
  last_state_marker_ << 0, 0, 0, 0;
  last_state_marker_global_ << 0, 0, 0, 0;
  robot_backtracking_prev_ = NULL;

  planner_trigger_mode_ = PlannerTriggerModeType::kManual;

  num_low_gain_iters_ = 0;
  auto_global_planner_trig_ = false;
  global_exploration_ongoing_ = false;
  local_exploration_ongoing_ = false;

  Eigen::Vector3d zero_vec = Eigen::Vector3d::Zero();
  inspection_bound_.setCenter(zero_vec, true);
  inspection_bound_.setRotation(zero_vec);

  free_cloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("freespace_pointcloud", 10);

  pci_reset_pub_ =
      nh_.advertise<std_msgs::Bool>("planner_control_interface/msg/reset", 10);

  //
  // global_graph_update_timer_ =
  //     nh_.createTimer(ros::Duration(kGlobalGraphUpdateTimerPeriod),
  //                     &Rrg::expandGlobalGraphTimerCallback, this);

  global_graph_frontier_addition_timer_ = nh_.createTimer(
      ros::Duration(kGlobalGraphFrontierAdditionTimerPeriod),
      &Rrg::expandGlobalGraphFrontierAdditionTimerCallback, this);

  // free_pointcloud_update_timer_ =
  //     nh_.createTimer(ros::Duration(kFreePointCloudUpdatePeriod),
  //                     &Rrg::freePointCloudtimerCallback, this);
  
  camera_annotation_timer_ = nh_.createTimer(ros::Duration(0.5), &Rrg::cameraAnnotationTimerCallback, this);

  semantics_subscriber_ =
      nh_.subscribe("semantic_location", 100, &Rrg::semanticsCallback, this);

  stop_srv_subscriber_ = nh_.subscribe("planner_control_interface/stop_request",
                                       100, &Rrg::stopMsgCallback, this);
  current_compartment_longs_sub_ = nh_.subscribe("current_compartment_longs",
                                       1, &Rrg::currentCompartmentLongsCallback, this);                              

  time_log_pub_ =
      nh_.advertise<std_msgs::Float32MultiArray>("gbp_time_log", 10);

  pci_homing_ = nh_.serviceClient<std_srvs::Trigger>(
      "planner_control_interface/std_srvs/homing_trigger");
  landing_srv_client_ = nh_.serviceClient<std_srvs::Empty>(
      "land_srv");  // The service name should be remapped in the launch file

  reset_timer_srv_ = nh_.advertiseService("gbplanner/reset_timer",
      &Rrg::resetTimerCallback, this);

	pass_manhole_srv_ = nh_.advertiseService("get_manhole_traversal_path", &Rrg::getManholePathCallback, this);
  approve_passing_srv_ = nh_.advertiseService("approve_manhole_traversal", &Rrg::approvePassingCallback, this);
  reset_map_srv_ = nh_.advertiseService("reset_map", &Rrg::resetMapCallback, this);

  listener_ = new tf::TransformListener();

  next_compartment_ << std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max();
}

void Rrg::reset() {
  // Check if the local graph frontiers have been added to the global graph
  if (add_frontiers_to_global_graph_) {
    ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Reset: Adding frontiers to global graph");
    add_frontiers_to_global_graph_ = false;
    addFrontiers(0);  // id given as 0 because it is not used
  }
  // Reset the local graph.
  if (local_graph_ != NULL) local_graph_->reset();
  local_graph_.reset(new GraphManager());
  local_graph_rep_.reset();

  if (projected_graph_ != NULL) projected_graph_->reset();
  projected_graph_.reset(new GraphManager());

  auto_global_planner_trig_ = false;

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
  if (robot_params_.type == RobotType::kGroundRobot) {
    MapManager::VoxelStatus vs;
    Eigen::Vector3d root_pos = root_state.head(3);
    double ground_height = projectSample(root_pos, vs);
    root_state(2) += (planning_params_.max_ground_height - ground_height);
  }

  // Create a root vertex and add to the graph.
  // Root vertex should be assigned id 0.
  root_vertex_ = new GbplannerVertex(local_graph_->generateVertexID(), root_state);
  local_graph_->addVertex(root_vertex_);

  // std::cout << "Current state: " << current_state_(0) << " " << current_state_(1) << " " << current_state_(2) << ", " << current_state_(3) << std::endl;
  // std::cout << "Root state:    " << local_graph_->getVertex(0)->state(0) << " " << local_graph_->getVertex(0)->state(1) << " " << local_graph_->getVertex(0)->state(2) << ", " << local_graph_->getVertex(0)->state(3) << std::endl;

  if ((planner_trigger_count_ == 0) && (global_graph_->getNumVertices() == 0)) {
    // First time trigger the planner. Initialize the root for global graph as
    // well.
    GbplannerVertex* g_root_vertex =
        new GbplannerVertex(global_graph_->generateVertexID(), root_state);
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
        ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Current box is Free.");
        break;
      case MapManager::VoxelStatus::kOccupied:
        ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Current box contains Occupied voxels.");
        break;
      case MapManager::VoxelStatus::kUnknown:
        ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Current box contains Unknown voxels.");
        break;
    }
    // Assume that even it is not fully free, but safe to clear these voxels.
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Starting position is not clear--> clear space around the robot.");
    map_manager_->augmentFreeBox(
        Eigen::Vector3d(root_state[0], root_state[1], root_state[2]) +
            robot_params_.center_offset,
        robot_box_size_);
  }

  // Set robot radius when single point collision checking is used
  map_manager_->setRobotRadius(robot_box_size_.maxCoeff() / 2.0);
  map_manager_->setBoxCheckMethod(planning_params_.box_check_method);
  map_manager_->setLineCheckMethod(planning_params_.line_check_method);

  // Clear free space before planning.
  if (planning_params_.free_frustum_before_planning) {
    map_manager_->augmentFreeFrustum();
  }
  visualization_->visualizeRobotState(root_vertex_->state, robot_params_);
  visualization_->visualizeSensorFOV(root_vertex_->state, sensor_params_);

  if (planning_params_.type == PlanningModeType::kAdaptiveExploration) {
    visualization_->visualizeWorkspace(
        root_vertex_->state, global_space_params_, local_adaptive_params_);
  } else {
    visualization_->visualizeWorkspace(
        root_vertex_->state, global_space_params_, local_space_params_);
  }
  visualization_->visualizeNoGainZones(no_gain_zones_);

  std::vector<double> empty_vec;
  for (int i = 0; i < planning_num_vertices_max_; ++i) {
    empty_vec.push_back(0.0);
  }
  edge_inclinations_.clear();
  for (int i = 0; i < planning_num_vertices_max_; ++i) {
    edge_inclinations_.push_back(empty_vec);
  }
}

bool Rrg::resetMapCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  map_manager_->resetMap();
  res.success = true;
  return true;
}

void Rrg::clear() {}

void Rrg::currentCompartmentLongsCallback(const geometry_msgs::PoseArray &longs)
{
  current_compartment_longs_.clear();

  for(auto p : longs.poses)
  {
    Longitudinal l;
    l.center.x() = p.position.x;
    l.center.y() = p.position.y;
    l.center.z() = p.position.z;
    l.direction.x() = std::cos(p.orientation.w);
    l.direction.y() = std::sin(p.orientation.w);
    l.direction.z() = 0.0;
    l.direction.normalize();
    l.end_point1 = l.center + l.direction * p.orientation.x;  // min_dist
    l.end_point2 = l.center + l.direction * p.orientation.y;  // max_dist
    current_compartment_longs_.push_back(l);

    // voxblox::Mesh mesh;
    // voxblox::VertexIndex v_ind;
    // Eigen::Matrix<voxblox::FloatingPoint, 3, 8> vertex_coords;
    // Eigen::Matrix<voxblox::FloatingPoint, 8, 1> vertex_sdf;
    // double voxel_size_inv = 1.0 / map_manager_->getResolution();
    // voxblox::LongIndex center_ind = voxblox::getGridIndexFromPoint<voxblox::LongIndex>(l.center.cast<voxblox::FloatingPoint>(), voxel_size_inv);
    // voxblox::Point center_voxel_point = voxblox::getCenterPointFromGridIndex(center_ind, map_manager_->getResolution());
    // Eigen::Matrix<voxblox::FloatingPoint, 3, 8> neighbor_table;
    // neighbor_table << 0, 1, 1, 0, 0, 1, 1, 0,
    //                   0, 0, 1, 1, 0, 0, 1, 1,
    //                   0, 0, 0, 0, 1, 1, 1, 1;
    // neighbor_table *= map_manager_->getResolution();
    // for(int i=0; i<8; ++i)
    // {
    //   vertex_coords.col(i) = center_voxel_point + neighbor_table.col(i);
    //   vertex_sdf[i] = map_manager_->getPointDistance(vertex_coords.col(i).cast<double>());
    // }
    // voxblox::MarchingCubes::meshCube(vertex_coords, vertex_sdf, &v_ind, &mesh);
    // std::cout << "Vertices:" << std::endl;
    // for(auto v : mesh.vertices) std::cout << v.transpose() << std::endl;
    // std::cout << "Normals:" << std::endl;
    // for(auto v : mesh.normals) std::cout << v.transpose() << std::endl;
  }



  // getLongsInspectionViewpoints();
}

void Rrg::stopMsgCallback(const std_msgs::Bool& msg) {
  global_exploration_ongoing_ = false;
  auto_global_planner_trig_ = false;
  manhole_traversal_status_ = ManholeTraversalMode::kNone;
}

void Rrg::stopPlanner() {
  global_exploration_ongoing_ = false;
  auto_global_planner_trig_ = false;
  manhole_traversal_status_ = ManholeTraversalMode::kNone;
}

bool Rrg::sampleVertex(GbplannerVertex& vertex) {
  StateVec state;
  bool hanging = false;
  bool found = false;

  int while_thres = 1000;  // magic number
  BoundedSpaceParams reduced_global_space = global_space_params_;
  reduced_global_space.min_val += 0.5 * robot_box_size_;
  reduced_global_space.max_val -= 0.5 * robot_box_size_;
  while (!found && while_thres--) {
    hanging = false;
    random_sampler_.generate(root_vertex_->state, state);
    // Very fast check if the sampled point is inside the planning space.
    // This helps eliminate quickly points outside the sampling space.
    Eigen::Vector3d sample = state.head(3);
    if (!reduced_global_space.isInsideSpace(sample)) continue;

    // Check if in geofence areas.
    if ((planning_params_.geofence_checking_enable) &&
        (GeofenceManager::CoordinateStatus::kViolated ==
         geofence_manager_->getBoxStatus(
             Eigen::Vector2d(state[0] + robot_params_.center_offset[0],
                             state[1] + robot_params_.center_offset[1]),
             Eigen::Vector2d(robot_box_size_[0], robot_box_size_[1]))))
      continue;

    if (robot_params_.type == RobotType::kGroundRobot) {
      Eigen::Vector3d sample;
      sample = Eigen::Vector3d(state[0], state[1], state[2]) +
               robot_params_.center_offset;
      MapManager::VoxelStatus vs;
      double ground_dist = projectSample(sample, vs);
      if (ground_dist < 0.0) continue;
      if (vs == MapManager::VoxelStatus::kUnknown) {
        hanging = true;
      }

      sample[2] -= (ground_dist - planning_params_.max_ground_height);
      state[0] = sample[0];
      state[1] = sample[1];
      state[2] -= (ground_dist - planning_params_.max_ground_height);
    }

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
  vertex.state = state;
  vertex.is_hanging = hanging;
  return found;
}

bool Rrg::sampleVertex(RandomSampler& random_sampler, StateVec& root_state,
                       GbplannerVertex& vertex) {
  StateVec state;
  bool hanging = false;
  bool found = false;

  int while_thres = 1000;  // magic number.
  BoundedSpaceParams reduced_global_space = global_space_params_;
  reduced_global_space.min_val += 0.5 * robot_box_size_;
  reduced_global_space.max_val -= 0.5 * robot_box_size_;
  while (!found && while_thres--) {
    hanging = false;
    random_sampler.generate(root_state, state);
    Eigen::Vector3d sample = state.head(3);
    if (!reduced_global_space.isInsideSpace(sample)) continue;

    if (robot_params_.type == RobotType::kGroundRobot) {
      Eigen::Vector3d sample;
      sample = Eigen::Vector3d(state[0], state[1], state[2]) +
               robot_params_.center_offset;
      MapManager::VoxelStatus vs;
      double ground_dist = projectSample(sample, vs);
      if (vs == MapManager::VoxelStatus::kUnknown) {
        hanging = true;
      }

      sample[2] -= (ground_dist - planning_params_.max_ground_height);
      state[0] = sample[0];
      state[1] = sample[1];
      state[2] -= (ground_dist - planning_params_.max_ground_height);
    }
    // Check if surrounding area is free.
    if (MapManager::VoxelStatus::kFree ==
        map_manager_->getBoxStatus(
            Eigen::Vector3d(state[0], state[1], state[2]) +
                robot_params_.center_offset,
            robot_box_size_, true)) {
      random_sampler.pushSample(state, true);  // for debug purpose.
      found = true;
    } else {
      stat_->num_vertices_fail++;
      random_sampler.pushSample(state, false);
      // hanging = false;
    }
  }
  vertex.state = state;
  vertex.is_hanging = hanging;
  return found;
}

double Rrg::projectSample(Eigen::Vector3d& sample,
                          MapManager::VoxelStatus& voxel_status) {
  double max_proj_len = 5.0;

  float voxel_size = map_manager_->getResolution();

  int unknown_count = 0;
  double central_ray_len = 0.0;
  std::vector<Eigen::Vector3d> extra_samples(5, Eigen::Vector3d::Zero());
  extra_samples[0] = Eigen::Vector3d(0.0, 0.0, 0.0);
  extra_samples[1] = Eigen::Vector3d(0.5, 0.0, 0.5);
  extra_samples[2] = Eigen::Vector3d(-0.5, 0.0, 0.5);
  extra_samples[3] = Eigen::Vector3d(0.0, 0.5, 0.5);
  extra_samples[4] = Eigen::Vector3d(0.0, -0.5, 0.5);
  for (int i = 0; i < extra_samples.size(); ++i) {
    Eigen::Vector3d delta = extra_samples[i];
    Eigen::Vector3d start = sample + delta;
    Eigen::Vector3d end = start - Eigen::Vector3d(0.0, 0.0, max_proj_len);

    if (planning_params_.interpolate_projection_distance) {
      double ground_dist = 0.0;
      Eigen::Vector3d point = start;
      while (ground_dist < max_proj_len) {
        double tsdf_dist = map_manager_->getPointDistance(point);
        if (tsdf_dist < 0.0) {
          // Hit Unknown
          if (i == 0) {
            central_ray_len = (point - start).norm();
          }
          ++unknown_count;
          break;
        } else {
          if (tsdf_dist <= voxel_size) {
            // Hit occupied
            voxel_status = MapManager::VoxelStatus::kOccupied;
            sample(0) = start(0);
            sample(1) = start(1);
            return ground_dist + tsdf_dist;
          } else {
            // Still in free
            ground_dist += tsdf_dist;
            point -= Eigen::Vector3d(0.0, 0.0, tsdf_dist - 0.02);
          }
        }
      }
    } else {
      Eigen::Vector3d end_voxel;
      double tsdf_dist;
      MapManager::VoxelStatus vs =
          map_manager_->getRayStatus(start, end, true, end_voxel, tsdf_dist);

      if (vs == MapManager::VoxelStatus::kOccupied) {
        double ray_len = std::abs(start(2) - end_voxel(2));
        if (i == 0) {
          central_ray_len = ray_len;
        }
        voxel_status = MapManager::VoxelStatus::kOccupied;
        sample(0) = start(0);
        sample(1) = start(1);
        return ray_len;
      } else if (vs == MapManager::VoxelStatus::kUnknown) {
        double ray_len = std::abs(start(2) - end_voxel(2));
        if (i == 0) {
          central_ray_len = ray_len;
        }
        ++unknown_count;
      }
    }
  }

  if (unknown_count >= extra_samples.size()) {
    voxel_status = MapManager::VoxelStatus::kUnknown;
    return central_ray_len;
  }

  voxel_status = MapManager::VoxelStatus::kFree;
  return -1.0;
}

ProjectedEdgeStatus Rrg::getProjectedEdgeStatus(
    const Eigen::Vector3d& start, const Eigen::Vector3d& end,
    const Eigen::Vector3d& box_size, bool stop_at_unknown_voxel,
    std::vector<Eigen::Vector3d>& projected_edge_out, bool is_hanging) {
  double step_size = 2.0 * map_manager_->getResolution();
  double max_inclination = planning_params_.max_inclination;

  std::vector<Eigen::Vector3d> projected_edge;
  Eigen::Vector3d ray = end - start;
  double edge_incl = std::atan2(std::abs(ray(2)), std::abs(ray.head(2).norm()));
  if (edge_incl > max_inclination) {
    return ProjectedEdgeStatus::kSteep;
  }

  double ray_len = ray.norm();
  Eigen::Vector3d ray_normed = ray / ray_len;

  if ((end - start).norm() >= 2 * step_size) {
    Eigen::Vector3d last_point;
    for (double step = 0.0; step < ray_len; step += step_size) {
      Eigen::Vector3d edge_point = start + step * ray_normed;
      last_point = edge_point;
      MapManager::VoxelStatus vs;
      double ground_height = projectSample(edge_point, vs);
      // Only allow intermediate hanging vertices if either of the end vertices
      // are hanging:
      if (vs == MapManager::VoxelStatus::kUnknown || ground_height < 0.0) {
        if (!is_hanging) {
          return ProjectedEdgeStatus::kHanging;
        }
      }
      Eigen::Vector3d projected_edge_pt = edge_point;
      // Get the point on the ground at a heigh of
      // planning_params_.max_ground_height
      projected_edge_pt(2) -=
          (ground_height - planning_params_.max_ground_height);
      projected_edge.push_back(projected_edge_pt);
    }
    if ((last_point - end).norm() < 0.75 * step_size) {
      projected_edge.erase(projected_edge.end());
    }
  } else {
    MapManager::VoxelStatus vs;
    Eigen::Vector3d start_m = start;
    Eigen::Vector3d end_m = end;
    double ground_height = projectSample(start_m, vs);
    // Project start and end points (extra check)
    if (vs == MapManager::VoxelStatus::kUnknown || ground_height < 0.0) {
      if (!is_hanging) {
        return ProjectedEdgeStatus::kHanging;
      }
    }
    ground_height = projectSample(end_m, vs);
    if (vs == MapManager::VoxelStatus::kUnknown || ground_height < 0.0) {
      if (!is_hanging) {
        return ProjectedEdgeStatus::kHanging;
      }
    }
    projected_edge.push_back(start);
  }

  MapManager::VoxelStatus vs;
  Eigen::Vector3d end_m = end;
  double ground_height = projectSample(end_m, vs);
  if (vs == MapManager::VoxelStatus::kUnknown || ground_height < 0.0) {
    if (!is_hanging) {
      return ProjectedEdgeStatus::kHanging;
    }
  }
  projected_edge.push_back(end_m);

  // Check edge segment inclination. Cheaper than collision check
  for (int i = 1; i < projected_edge.size(); ++i) {
    Eigen::Vector3d segment = projected_edge[i] - projected_edge[i - 1];
    double theta =
        std::atan2(std::abs(segment(2)), std::abs(segment.head(2).norm()));
    if (std::abs(theta) > max_inclination) {
      return ProjectedEdgeStatus::kSteep;
    }
  }

  // Check edge collision:
  for (int i = 1; i < projected_edge.size(); ++i) {
    MapManager::VoxelStatus vs =
        map_manager_->getPathStatus(projected_edge[i - 1], projected_edge[i],
                                    box_size, stop_at_unknown_voxel);
    if (vs == MapManager::VoxelStatus::kUnknown) {
      return ProjectedEdgeStatus::kUnknown;
    } else if (vs == MapManager::VoxelStatus::kOccupied) {
      return ProjectedEdgeStatus::kOccipied;
    }
  }

  // Edge is admissible
  projected_edge_out = projected_edge;
  return ProjectedEdgeStatus::kAdmissible;
}

void Rrg::expandGraph(std::shared_ptr<GraphManager> graph_manager,
                      StateVec& new_state, ExpandGraphReport& rep,
                      bool allow_short_edge) {
  // Find nearest neighbour
  // StateVec &new_state = new_vertex->state;
  GbplannerVertex* nearest_vertex = NULL;
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

  if (robot_params_.type == RobotType::kGroundRobot) {
    Eigen::Vector3d new_pos;
    new_pos << new_state[0], new_state[1], new_state[2];
    MapManager::VoxelStatus vs;
    double ground_height = projectSample(new_pos, vs);
    if (vs == MapManager::VoxelStatus::kOccupied) {
      new_pos[2] -= (ground_height - planning_params_.max_ground_height);
    } else {
      rep.status = ExpandGraphStatus::kErrorCollisionEdge;
      return;
    }
    new_state[0] = new_pos[0];
    new_state[1] = new_pos[1];
    new_state[2] = new_pos[2];
  }

  // Since we are buiding graph,
  // Consider to check the overshoot for both directions except root node.
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

  bool admissible_edge = false;
  int steep_edges = 0;
  if (robot_params_.type == RobotType::kAerialRobot) {
    if(unknown_as_free_)
    {
      MapManager::VoxelStatus vs =
          map_manager_->getPathStatus(start_pos, end_pos, robot_box_size_, false);
      if (MapManager::VoxelStatus::kOccupied != vs) {
        admissible_edge = true;
      }
    }
    else
    {
      MapManager::VoxelStatus vs =
          map_manager_->getPathStatus(start_pos, end_pos, robot_box_size_, true);
      if (MapManager::VoxelStatus::kFree == vs) {
        admissible_edge = true;
      }
    }
  } else if (robot_params_.type == RobotType::kGroundRobot) {
    std::vector<Eigen::Vector3d> projected_edge;
    ProjectedEdgeStatus es = getProjectedEdgeStatus(
        start_pos, end_pos, robot_box_size_, true, projected_edge, false);
    if (ProjectedEdgeStatus::kAdmissible == es) {
      admissible_edge = true;
      StateVec strt_st(projected_edge[0](0), projected_edge[0](1),
                       projected_edge[0](2), 0.0);
      GbplannerVertex* strt_vert =
          new GbplannerVertex(projected_graph_->generateVertexID(), strt_st);
      projected_graph_->addVertex(strt_vert);
      GbplannerVertex* prev_vert = strt_vert;
      for (int i = 1; i < projected_edge.size(); ++i) {
        StateVec proj_st(projected_edge[i](0), projected_edge[i](1),
                         projected_edge[i](2), 0.0);
        GbplannerVertex* proj_vert =
            new GbplannerVertex(projected_graph_->generateVertexID(), proj_st);
        projected_graph_->addVertex(proj_vert);
        double edge_len = (proj_vert->state - prev_vert->state).norm();
        projected_graph_->addEdge(proj_vert, prev_vert, edge_len);
        prev_vert = proj_vert;
      }
    } else if (ProjectedEdgeStatus::kSteep == es)
      ++steep_edges;
  }
  if (admissible_edge) {
    GbplannerVertex* new_vertex =
        new GbplannerVertex(graph_manager->generateVertexID(), new_state);
    // Form a tree as the first step.
    new_vertex->parent = nearest_vertex;
    new_vertex->distance = nearest_vertex->distance + direction_norm;
    nearest_vertex->children.push_back(new_vertex);
    graph_manager->addVertex(new_vertex);
    ++rep.num_vertices_added;
    rep.vertex_added = new_vertex;
    graph_manager->addEdge(new_vertex, nearest_vertex, direction_norm);
    ++rep.num_edges_added;
    // Form more edges from neighbors if set RRG mode.
    if (planning_params_.rr_mode == RRModeType::kGraph) {
      std::vector<GbplannerVertex*> nearest_vertices;
      if (!graph_manager->getNearestVertices(
              &new_state, planning_params_.nearest_range, &nearest_vertices)) {
        rep.status = ExpandGraphStatus::kErrorKdTree;
        return;
      }
      origin << new_vertex->state[0], new_vertex->state[1],
          new_vertex->state[2];
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
          if (planning_params_.geofence_checking_enable &&
              (GeofenceManager::CoordinateStatus::kViolated ==
               geofence_manager_->getPathStatus(
                   Eigen::Vector2d(p_start[0], p_start[1]),
                   Eigen::Vector2d(p_end[0], p_end[1]),
                   Eigen::Vector2d(robot_box_size_[0], robot_box_size_[1])))) {
            geofence_pass = false;
          }

          if (geofence_pass) {
            admissible_edge = false;
            if (robot_params_.type == RobotType::kAerialRobot) {
              MapManager::VoxelStatus vs = map_manager_->getPathStatus(
                  p_start, p_end, robot_box_size_, true);
              if (MapManager::VoxelStatus::kFree == vs) {
                admissible_edge = true;
              }
            } else if (robot_params_.type == RobotType::kGroundRobot) {
              std::vector<Eigen::Vector3d> projected_edge;
              ProjectedEdgeStatus es = getProjectedEdgeStatus(
                  p_start, p_end, robot_box_size_, true, projected_edge, false);
              if (ProjectedEdgeStatus::kAdmissible == es) {
                admissible_edge = true;
                StateVec strt_st(projected_edge[0](0), projected_edge[0](1),
                                 projected_edge[0](2), 0.0);
                GbplannerVertex* strt_vert =
                    new GbplannerVertex(projected_graph_->generateVertexID(), strt_st);
                projected_graph_->addVertex(strt_vert);
                GbplannerVertex* prev_vert = strt_vert;
                for (int i = 1; i < projected_edge.size(); ++i) {
                  StateVec proj_st(projected_edge[i](0), projected_edge[i](1),
                                   projected_edge[i](2), 0.0);
                  GbplannerVertex* proj_vert =
                      new GbplannerVertex(projected_graph_->generateVertexID(), proj_st);
                  projected_graph_->addVertex(proj_vert);
                  double edge_len =
                      (proj_vert->state - prev_vert->state).norm();
                  projected_graph_->addEdge(proj_vert, prev_vert, edge_len);
                  prev_vert = proj_vert;
                }
              } else if (ProjectedEdgeStatus::kSteep == es)
                ++steep_edges;
            }
            if (admissible_edge) {
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
                           GbplannerVertex* new_vertex, ExpandGraphReport& rep) {
  std::vector<GbplannerVertex*> nearest_vertices;
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
    // ROS_WARN("Expand edges: d_norm: %f", d_norm);
    if ((d_norm > planning_params_.edge_length_min) &&
        (d_norm < planning_params_.edge_length_max)) {
      // ROS_WARN("Expand edges: d_norm: %f: Within limits", d_norm);
      Eigen::Vector3d p_overshoot =
          direction / d_norm * planning_params_.edge_overshoot;
      Eigen::Vector3d p_start =
          origin + robot_params_.center_offset - p_overshoot;
      Eigen::Vector3d p_end = origin + robot_params_.center_offset + direction;
      if (nearest_vertices[i]->id != 0) p_end = p_end + p_overshoot;

      bool admissible_edge = false;
      std::vector<Eigen::Vector3d> projected_edge;
      if (robot_params_.type == RobotType::kAerialRobot) {
        MapManager::VoxelStatus vs =
            map_manager_->getPathStatus(p_start, p_end, robot_box_size_, false);
        if (MapManager::VoxelStatus::kFree == vs) {
          admissible_edge = true;
        }
      } else if (robot_params_.type == RobotType::kGroundRobot) {
        ProjectedEdgeStatus es = getProjectedEdgeStatus(
            p_start, p_end, robot_box_size_, true, projected_edge, false);
        if (ProjectedEdgeStatus::kAdmissible == es) {
          admissible_edge = true;
        }
      }
      if (admissible_edge) {
        // ROS_WARN("     Admissible edge: len=%f", d_norm);
        graph_manager->addEdge(new_vertex, nearest_vertices[i], d_norm);
        ++rep.num_edges_added;
      }
    }
    // else {
    //   ROS_WARN("Expand edges: d_norm: %f: OUTSIDE limits", d_norm);
    // }
  }
  rep.status = ExpandGraphStatus::kSuccess;
}

void Rrg::expandGraph(std::shared_ptr<GraphManager> graph_manager,
                      GbplannerVertex& new_vertex, ExpandGraphReport& rep,
                      bool allow_short_edge, bool allow_long_edge) {
  // Find nearest neighbour
  StateVec new_state;
  new_state = new_vertex.state;

  GbplannerVertex* nearest_vertex = NULL;
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
    if(!allow_long_edge) direction = planning_params_.edge_length_max * direction.normalized();
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

  if (robot_params_.type == RobotType::kGroundRobot) {
    Eigen::Vector3d new_pos;
    new_pos << new_state[0], new_state[1], new_state[2];
    MapManager::VoxelStatus vs;
    double ground_height = projectSample(new_pos, vs);
    if (vs == MapManager::VoxelStatus::kOccupied) {
      new_pos[2] -= (ground_height - planning_params_.max_ground_height);
    } else {
      rep.status = ExpandGraphStatus::kErrorCollisionEdge;
      return;
    }
    new_state[0] = new_pos[0];
    new_state[1] = new_pos[1];
    new_state[2] = new_pos[2];
  }

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

  bool admissible_edge = false;
  int steep_edges = 0;
  std::vector<Eigen::Vector3d> projected_edge;
  if (robot_params_.type == RobotType::kAerialRobot) {
    if(unknown_as_free_)
    {
      MapManager::VoxelStatus vs =
          map_manager_->getPathStatus(start_pos, end_pos, robot_box_size_, false);
      if (MapManager::VoxelStatus::kOccupied != vs) {
        admissible_edge = true;
      }
    }
    else
    {
      MapManager::VoxelStatus vs =
          map_manager_->getPathStatus(start_pos, end_pos, robot_box_size_, true);
      if (MapManager::VoxelStatus::kFree == vs) {
        admissible_edge = true;
      }
    }
  } else if (robot_params_.type == RobotType::kGroundRobot) {
    bool is_hanging = nearest_vertex->is_hanging || new_vertex.is_hanging;
    ProjectedEdgeStatus es = getProjectedEdgeStatus(
        start_pos, end_pos, robot_box_size_, true, projected_edge, is_hanging);
    if (ProjectedEdgeStatus::kAdmissible == es) {
      admissible_edge = true;

      StateVec strt_st(projected_edge[0](0), projected_edge[0](1),
                       projected_edge[0](2), 0.0);
      GbplannerVertex* strt_vert =
          new GbplannerVertex(projected_graph_->generateVertexID(), strt_st);
      projected_graph_->addVertex(strt_vert);
      GbplannerVertex* prev_vert = strt_vert;
      for (int i = 1; i < projected_edge.size(); ++i) {
        StateVec proj_st(projected_edge[i](0), projected_edge[i](1),
                         projected_edge[i](2), 0.0);
        GbplannerVertex* proj_vert =
            new GbplannerVertex(projected_graph_->generateVertexID(), proj_st);
        projected_graph_->addVertex(proj_vert);
        double edge_len = (proj_vert->state - prev_vert->state).norm();
        projected_graph_->addEdge(proj_vert, prev_vert, edge_len);
        prev_vert = proj_vert;
      }
    } else if (ProjectedEdgeStatus::kSteep == es)
      ++steep_edges;
  }

  if (admissible_edge) {
    GbplannerVertex* new_vertex_ptr =
        new GbplannerVertex(graph_manager->generateVertexID(), new_state);
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
    if (local_exploration_ongoing_) {
      double max_inclination = 0.0;
      double avg_inclination = 0.0;

      for (int i = 1; i < projected_edge.size(); ++i) {
        Eigen::Vector3d edge_segment =
            projected_edge[i] - projected_edge[i - 1];
        double inclination = (std::atan2(std::abs(edge_segment(2)),
                                         edge_segment.head(2).norm()));
        // std::cout << inclination << std::endl;
        if (inclination > max_inclination) max_inclination = inclination;
        avg_inclination += inclination;
      }
      avg_inclination /= projected_edge.size();

      edge_inclinations_[new_vertex_ptr->id][nearest_vertex->id] =
          avg_inclination;
      edge_inclinations_[nearest_vertex->id][new_vertex_ptr->id] =
          avg_inclination;
    }

    // Form more edges from neighbors if set RRG mode.
    if (planning_params_.rr_mode == RRModeType::kGraph) {
      std::vector<GbplannerVertex*> nearest_vertices;
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
          if (planning_params_.geofence_checking_enable &&
              (GeofenceManager::CoordinateStatus::kViolated ==
               geofence_manager_->getPathStatus(
                   Eigen::Vector2d(p_start[0], p_start[1]),
                   Eigen::Vector2d(p_end[0], p_end[1]),
                   Eigen::Vector2d(robot_box_size_[0], robot_box_size_[1])))) {
            geofence_pass = false;
          }

          if (geofence_pass) {
            admissible_edge = false;
            if (robot_params_.type == RobotType::kAerialRobot) {
              MapManager::VoxelStatus vs = map_manager_->getPathStatus(
                  p_start, p_end, robot_box_size_, true);
              if (MapManager::VoxelStatus::kFree == vs) {
                admissible_edge = true;
              }
            } else if (robot_params_.type == RobotType::kGroundRobot) {
              projected_edge.clear();
              bool is_hanging =
                  new_vertex_ptr->is_hanging || nearest_vertices[i]->is_hanging;
              ProjectedEdgeStatus es =
                  getProjectedEdgeStatus(p_start, p_end, robot_box_size_, true,
                                         projected_edge, is_hanging);
              if (ProjectedEdgeStatus::kAdmissible == es) {
                admissible_edge = true;
                StateVec strt_st(projected_edge[0](0), projected_edge[0](1),
                                 projected_edge[0](2), 0.0);
                GbplannerVertex* strt_vert =
                    new GbplannerVertex(projected_graph_->generateVertexID(), strt_st);
                projected_graph_->addVertex(strt_vert);
                GbplannerVertex* prev_vert = strt_vert;
                for (int i = 1; i < projected_edge.size(); ++i) {
                  StateVec proj_st(projected_edge[i](0), projected_edge[i](1),
                                   projected_edge[i](2), 0.0);
                  GbplannerVertex* proj_vert =
                      new GbplannerVertex(projected_graph_->generateVertexID(), proj_st);
                  projected_graph_->addVertex(proj_vert);
                  double edge_len =
                      (proj_vert->state - prev_vert->state).norm();
                  projected_graph_->addEdge(proj_vert, prev_vert, edge_len);
                  prev_vert = proj_vert;
                }
              } else if (ProjectedEdgeStatus::kSteep == es)
                ++steep_edges;
            }

            if (admissible_edge) {
              if (local_exploration_ongoing_) {
                double max_inclination = 0.0;
                double avg_inclination = 0.0;
                for (int i = 1; i < projected_edge.size(); ++i) {
                  Eigen::Vector3d edge_segment =
                      projected_edge[i] - projected_edge[i - 1];
                  double inclination = (std::atan2(
                      std::abs(edge_segment(2)), edge_segment.head(2).norm()));
                  if (inclination > max_inclination)
                    max_inclination = inclination;
                  avg_inclination += inclination;
                }
                avg_inclination /= projected_edge.size();
                edge_inclinations_[new_vertex_ptr->id]
                                  [nearest_vertices[i]->id] = avg_inclination;
                edge_inclinations_[nearest_vertices[i]->id]
                                  [new_vertex_ptr->id] = avg_inclination;
              }
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

void Rrg::expandGraphEdgesBlindly(std::shared_ptr<GraphManager> graph_manager,
                                  GbplannerVertex* new_vertex, double radius,
                                  ExpandGraphReport& rep) {
  std::vector<GbplannerVertex*> nearest_vertices;
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
  int loop_count = 0;
  int num_vertices = 1;
  int num_edges = 0;

  local_exploration_ongoing_ = true;

  if (global_exploration_ongoing_) {
    GbplannerVertex* global_vertex = global_graph_->getVertex(current_global_vertex_id_);
    // Global repositioning stopped in between
    if ((current_state_.head(3) - global_vertex->state.head(3)).norm() > 5.0) {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Global frontier not reached. Triggering global planner again");
      local_exploration_ongoing_ = false;
      return GraphStatus::NOT_OK;
    }
    // Global repositioning complete
    else {
      global_exploration_ongoing_ = false;
    }
  }

  if(planning_params_.only_manhole_traversal) {
    local_exploration_ongoing_ = false;
    return GraphStatus::OK;
  }

  if (planning_params_.type == PlanningModeType::kAdaptiveExploration) {
    // 1. Construct the box
    Eigen::Vector3d min_val, max_val, rotations, mean_val, std_val;
    Eigen::Vector3d pos = root_vertex_->state.head(3);
    // adaptive obb will extend this bounding box
    min_val = adaptive_orig_min_val_;
    max_val = adaptive_orig_max_val_;
    adaptive_obb_->constructBoundingBox(pos, min_val, max_val, rotations,
                                        mean_val, std_val);

    // 2. Update bounding box
    local_adaptive_params_.setBound(min_val, max_val);
    local_adaptive_params_.setRotation(rotations);

    // 3. Update sampler
    random_sampler_adaptive_.setBound(min_val, max_val);
    random_sampler_adaptive_.setRotation(rotations);
    // Note: doesn't affect sampler when kUniform, kConst
    random_sampler_adaptive_.setDistributionParams(mean_val, std_val);
    random_sampler_adaptive_.reset();

  } else {
    random_sampler_.reset();
  }

  if (planning_params_.type == PlanningModeType::kAdaptiveExploration) {
    visualization_->visualizeWorkspace(
        root_vertex_->state, global_space_params_, local_adaptive_params_);
  } else {
    visualization_->visualizeWorkspace(
        root_vertex_->state, global_space_params_, local_space_params_);
  }

  START_TIMER(ttime);
  auto t1 = std::chrono::high_resolution_clock::now();
  auto t2 = t1;

  while ((loop_count++ < planning_params_.num_loops_max) &&
         (num_vertices < planning_num_vertices_max_) &&
         (num_edges < planning_num_edges_max_)) {
    GbplannerVertex new_vertex(-1, StateVec::Zero());

    if (planning_params_.type == PlanningModeType::kAdaptiveExploration) {
      if (!sampleVertex(random_sampler_adaptive_, root_vertex_->state,
                        new_vertex)) {
        continue;
      }
    } else {
      if (!sampleVertex(new_vertex)) {
        continue;
      }
    }

    ExpandGraphReport rep;
    expandGraph(local_graph_, new_vertex, rep);
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
  t2 = std::chrono::high_resolution_clock::now();

  std::shared_ptr<BaseGraph> g = local_graph_->graph_;

  stat_chrono_->build_graph_time =
      std::chrono::duration<double, std::milli>(t2 - t1).count();

  // Visualize geofence area.
  if (planning_params_.geofence_checking_enable)
    visualization_->visualizeGeofence(geofence_manager_);

  planner_trigger_count_++;
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Formed a graph with [%d] vertices and [%d] edges with [%d] loops",
           num_vertices, num_edges, loop_count);

  if (planning_params_.type == PlanningModeType::kAdaptiveExploration)
    visualization_->visualizeSampler(random_sampler_adaptive_);
  else
    visualization_->visualizeSampler(random_sampler_);

  local_exploration_ongoing_ = false;

  if (local_graph_->getNumVertices() > 1) {
    if (robot_params_.type == RobotType::kGroundRobot) {
      visualization_->visualizeGraph(local_graph_);
      visualization_->visualizeProjectedGraph(projected_graph_);
    } else
      visualization_->visualizeGraph(local_graph_);
    local_graph_->findShortestPaths(local_graph_rep_);
    stat_->shortest_path_time = GET_ELAPSED_TIME(ttime);
    // correctYaw();
    return Rrg::GraphStatus::OK;
  } else {
    visualization_->visualizeFailedEdges(stat_);
    ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Number of failed samples: [%d] vertices and [%d] edges",
             stat_->num_vertices_fail, stat_->num_edges_fail);
    return Rrg::GraphStatus::ERR_NO_FEASIBLE_PATH;
  }
}

Rrg::GraphStatus Rrg::buildGridGraph(StateVec state, Eigen::Vector3d robot_size,
                                     Eigen::Vector3d grid_min,
                                     Eigen::Vector3d grid_max,
                                     Eigen::Vector3d grid_res, double heading) {
  // Create vertices based on grid pattern, keep tracking collision-free
  // vertices. Create edges along x, y, and diagonal axes; keep collision-free
  // edges only. Clean unconnected vertices or edges to provide a clean graph.
  // ??? Grid_min <= 0; Grid_max >= 0; grid_res >= 0;

  // Root vertex must be exactly at the state.

  if ((grid_min[0] > 0.0) || (grid_min[1] > 0.0) || (grid_min[2] > 0.0)) {
    return GraphStatus::NOT_OK;
  }

  if ((grid_max[0] < 0.0) || (grid_max[1] < 0.0) || (grid_max[2] < 0.0)) {
    return GraphStatus::NOT_OK;
  }

  if ((grid_res[0] == 0.0) || (grid_res[1] == 0.0) || (grid_res[2] == 0.0)) {
    return GraphStatus::NOT_OK;
  }

  // Add all nodes first.
  int num_nodes[3];
  int root_node_ind[3];
  for (int i = 0; i < 3; ++i) {
    // truncate according to setting grid resolution.
    grid_min[i] = -grid_res[i] * std::ceil(-grid_min[i] / grid_res[i]);
    grid_max[i] = grid_res[i] * std::ceil(grid_max[i] / grid_res[i]);
    num_nodes[i] = (int)((grid_max[i] - grid_min[i]) / grid_res[i]) + 1;
    if (num_nodes[i] == 0) num_nodes[i] = 1;
    root_node_ind[i] = (int)(-grid_min[i] / grid_res[i]);
  }
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Number of nodes [%d][%d][%d].", num_nodes[0], num_nodes[1],
           num_nodes[2]);

  int num_total_nodes = num_nodes[0] * num_nodes[1] * num_nodes[2];
  GbplannerVertex** vertices_mat = new GbplannerVertex*[num_total_nodes];
  bool* collision_free_mat = new bool[num_total_nodes];

  for (int i = 0; i < num_nodes[0]; ++i) {
    for (int j = 0; j < num_nodes[1]; ++j) {
      for (int k = 0; k < num_nodes[2]; ++k) {
        int ind = i * num_nodes[1] * num_nodes[2] + j * num_nodes[2] + k;
        collision_free_mat[ind] = false;
      }
    }
  }
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Initialized matrices.");

  double cos_h = 1.0, sin_h = 0.0;
  if (heading) {
    cos_h = std::cos(heading);
    sin_h = std::sin(heading);
  }
  int root_node_ind_arr = root_node_ind[0] * num_nodes[1] * num_nodes[2] +
                          root_node_ind[1] * num_nodes[2] + root_node_ind[2];

  int free_vertex_count = 0;
  for (int i = 0; i < num_nodes[0]; ++i) {
    for (int j = 0; j < num_nodes[1]; ++j) {
      for (int k = 0; k < num_nodes[2]; ++k) {
        int cur_ind = i * num_nodes[1] * num_nodes[2] + j * num_nodes[2] + k;
        if (cur_ind == root_node_ind_arr) {
          // Force the root node as free; otherwise it is useless to build the
          // graph.
          collision_free_mat[cur_ind] = true;
          vertices_mat[cur_ind] = root_vertex_;
          continue;
        }

        double x_val = grid_min.x() + i * grid_res.x();
        double y_val = grid_min.y() + j * grid_res.y();
        double z_val = grid_min.z() + k * grid_res.z();
        if (heading) {
          double x_val_, y_val_;
          x_val_ = x_val * cos_h - y_val * sin_h;
          y_val_ = x_val * sin_h + y_val * cos_h;
          x_val = x_val_;
          y_val = y_val_;
        }

        x_val += state.x();
        y_val += state.y();
        z_val += state.z();

        StateVec new_state(x_val, y_val, z_val, heading);
        GbplannerVertex* new_vertex =
            new GbplannerVertex(local_graph_->generateVertexID(), new_state);
        local_graph_->addVertex(new_vertex);
        vertices_mat[cur_ind] = new_vertex;

        Eigen::Vector3d voxel(x_val, y_val, z_val);
        if (MapManager::VoxelStatus::kFree ==
            map_manager_->getBoxStatus(voxel, robot_size, true)) {
          collision_free_mat[cur_ind] = true;
          free_vertex_count++;
        }
      }
    }
  }
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Number of free nodes [%d].", free_vertex_count);

  // Quick check for edge shorter than map resolution:
  double map_res = map_manager_->getResolution();
  // Check collision along edges vertical/horizontal
  double dx_len = grid_res.x();
  double dy_len = grid_res.y();
  double dz_len = grid_res.z();
  int free_edge_count = 0;
  for (int i = 0; i < num_nodes[0]; ++i) {
    for (int j = 0; j < num_nodes[1]; ++j) {
      for (int k = 0; k < num_nodes[2]; ++k) {
        // add if it is collision free.
        int vertex_ind = i * num_nodes[1] * num_nodes[2] + j * num_nodes[2] + k;
        if (!collision_free_mat[vertex_ind]) continue;

        Eigen::Vector3d start(vertices_mat[vertex_ind]->state.x(),
                              vertices_mat[vertex_ind]->state.y(),
                              vertices_mat[vertex_ind]->state.z());
        Eigen::Vector3d end;

        // x direction
        if (i < (num_nodes[0] - 1)) {
          int vertex_ind_x =
              (i + 1) * num_nodes[1] * num_nodes[2] + j * num_nodes[2] + k;
          if (collision_free_mat[vertex_ind_x]) {
            end << vertices_mat[vertex_ind_x]->state.x(),
                vertices_mat[vertex_ind_x]->state.y(),
                vertices_mat[vertex_ind_x]->state.z();
            if ((dx_len <= map_res) ||
                (MapManager::VoxelStatus::kFree ==
                 map_manager_->getPathStatus(start, end, robot_size, true))) {
              free_edge_count++;
              local_graph_->addEdge(vertices_mat[vertex_ind],
                                    vertices_mat[vertex_ind_x], dx_len);
            }
          }
        }

        // y direction
        if (j < (num_nodes[1] - 1)) {
          int vertex_ind_y =
              i * num_nodes[1] * num_nodes[2] + (j + 1) * num_nodes[2] + k;
          if (collision_free_mat[vertex_ind_y]) {
            end << vertices_mat[vertex_ind_y]->state.x(),
                vertices_mat[vertex_ind_y]->state.y(),
                vertices_mat[vertex_ind_y]->state.z();
            if ((dy_len <= map_res) ||
                (MapManager::VoxelStatus::kFree ==
                 map_manager_->getPathStatus(start, end, robot_size, true))) {
              free_edge_count++;
              local_graph_->addEdge(vertices_mat[vertex_ind],
                                    vertices_mat[vertex_ind_y], dy_len);
            }
          }
        }

        // z direction
        if (k < (num_nodes[2] - 1)) {
          int vertex_ind_z =
              i * num_nodes[1] * num_nodes[2] + j * num_nodes[2] + k + 1;
          if (collision_free_mat[vertex_ind_z]) {
            end << vertices_mat[vertex_ind_z]->state.x(),
                vertices_mat[vertex_ind_z]->state.y(),
                vertices_mat[vertex_ind_z]->state.z();
            if ((dz_len <= map_res) ||
                (MapManager::VoxelStatus::kFree ==
                 map_manager_->getPathStatus(start, end, robot_size, true))) {
              free_edge_count++;
              local_graph_->addEdge(vertices_mat[vertex_ind],
                                    vertices_mat[vertex_ind_z], dz_len);
            }
          }
        }
      }
    }
  }

  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Free edges: [%d]", free_edge_count);

  // Check collision along diagonal edges
  double diag_len = std::sqrt(dx_len * dx_len + dy_len * dy_len);
  for (int i = 0; i < num_nodes[0]; ++i) {
    for (int j = 0; j < num_nodes[1]; ++j) {
      for (int k = 0; k < num_nodes[2]; ++k) {
        if ((i < (num_nodes[0] - 1)) && (j < (num_nodes[1] - 1))) {
          int vertex_ind_d0 =
              i * num_nodes[1] * num_nodes[2] + j * num_nodes[2] + k;
          int vertex_ind_d2 = (i + 1) * num_nodes[1] * num_nodes[2] +
                              (j + 1) * num_nodes[2] + k;
          if (collision_free_mat[vertex_ind_d0] &&
              collision_free_mat[vertex_ind_d2]) {
            Eigen::Vector3d start(vertices_mat[vertex_ind_d0]->state.x(),
                                  vertices_mat[vertex_ind_d0]->state.y(),
                                  vertices_mat[vertex_ind_d0]->state.z());
            Eigen::Vector3d end(vertices_mat[vertex_ind_d2]->state.x(),
                                vertices_mat[vertex_ind_d2]->state.y(),
                                vertices_mat[vertex_ind_d2]->state.z());
            if ((diag_len <= map_res) ||
                (MapManager::VoxelStatus::kFree ==
                 map_manager_->getPathStatus(start, end, robot_size, true))) {
              free_edge_count++;
              local_graph_->addEdge(vertices_mat[vertex_ind_d0],
                                    vertices_mat[vertex_ind_d2], diag_len);
            }
          }
        }

        if ((i < (num_nodes[0] - 1)) && (j < (num_nodes[1] - 1))) {
          int vertex_ind_d1 =
              i * num_nodes[1] * num_nodes[2] + (j + 1) * num_nodes[2] + k;
          int vertex_ind_d3 =
              (i + 1) * num_nodes[1] * num_nodes[2] + j * num_nodes[2] + k;
          if (collision_free_mat[vertex_ind_d1] &&
              collision_free_mat[vertex_ind_d3]) {
            Eigen::Vector3d start(vertices_mat[vertex_ind_d1]->state.x(),
                                  vertices_mat[vertex_ind_d1]->state.y(),
                                  vertices_mat[vertex_ind_d1]->state.z());
            Eigen::Vector3d end(vertices_mat[vertex_ind_d3]->state.x(),
                                vertices_mat[vertex_ind_d3]->state.y(),
                                vertices_mat[vertex_ind_d3]->state.z());
            if ((diag_len <= map_res) ||
                (MapManager::VoxelStatus::kFree ==
                 map_manager_->getPathStatus(start, end, robot_size, true))) {
              free_edge_count++;
              local_graph_->addEdge(vertices_mat[vertex_ind_d1],
                                    vertices_mat[vertex_ind_d3], diag_len);
            }
          }
        }
      }
    }
  }

  // Add source vertex.
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Grid graph: %d vertices, %d edges", local_graph_->getNumVertices(),
           local_graph_->getNumEdges());
  return GraphStatus::OK;
}

void Rrg::correctYaw() {
  // Choose the heading angle tangent with the moving direction.
  if (planning_params_.yaw_tangent_correction) {
		// If this param is true, do it for all vertices
    int num_vertices = local_graph_->getNumVertices();
    for (int id = 1; id < num_vertices; ++id) {
      int pid = local_graph_->getParentIDFromShortestPath(id, local_graph_rep_);
      GbplannerVertex* v = local_graph_->getVertex(id);
      GbplannerVertex* vp = local_graph_->getVertex(pid);
      Eigen::Vector3d vec(v->state[0] - vp->state[0],
                          v->state[1] - vp->state[1],
                          v->state[2] - vp->state[2]);
      if (planning_params_.planning_backward) vec = -vec;
      v->state[3] = std::atan2(vec[1], vec[0]);
    }
  }
  else {
		// Else
    if(planning_params_.keep_leaf_yaw_only) {
			// do it for all but leaves
			int num_vertices = local_graph_->getNumVertices();
			for (int id = 1; id < num_vertices; ++id) {
				int pid = local_graph_->getParentIDFromShortestPath(id, local_graph_rep_);
				GbplannerVertex* v = local_graph_->getVertex(id);
				if(v->is_leaf_vertex) {
					continue;
				}
				GbplannerVertex* vp = local_graph_->getVertex(pid);
				Eigen::Vector3d vec(v->state[0] - vp->state[0],
														v->state[1] - vp->state[1],
														v->state[2] - vp->state[2]);
				if (planning_params_.planning_backward) vec = -vec;
				v->state[3] = std::atan2(vec[1], vec[0]);
			}
		}
		// Else do it for none
  }
}

Rrg::GraphStatus Rrg::evaluateGraph(bool assisted) {
  Rrg::GraphStatus gstatus = Rrg::GraphStatus::OK;

  if(planning_params_.only_manhole_traversal) {
    auto_global_planner_trig_ = true;
    return Rrg::GraphStatus::NOT_OK;
  }

  START_TIMER(ttime);
  auto t1 = std::chrono::high_resolution_clock::now();
  auto t2 = t1;
  // Dijkstra and mark leaf vertices.
  local_graph_->findShortestPaths(local_graph_rep_);
  local_graph_->findLeafVertices(local_graph_rep_);
  std::vector<GbplannerVertex*> leaf_vertices;
  local_graph_->getLeafVertices(leaf_vertices);
  stat_->shortest_path_time = GET_ELAPSED_TIME(ttime);
  t2 = std::chrono::high_resolution_clock::now();
  stat_chrono_->shortest_path_time =
      std::chrono::duration<double, std::milli>(t2 - t1).count();

  // correctYaw();

  // Gain calculation for each vertex.
  if(assisted)
    computeSemAssistedExplorationGain();
  else
    computeExplorationGain();

  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[Evaluate graph]: Gain Calculation Done");
  add_frontiers_to_global_graph_ = true;
  // Gain evaluation for valid paths, starting from the leaf to the root.
  START_TIMER(ttime);
  t1 = std::chrono::high_resolution_clock::now();
  double best_gain = 0;
  int best_path_id = 0;
  int num_leaf_vertices = leaf_vertices.size();
  bool frontier_exists = false;
  std::vector<int> negative_edge_leafs;
  std::vector<Eigen::Vector3d> inadmissible_negative_edges;
  int path_len_count = 0, path_dir_count = 0;
  double path_len_time = 0.0, path_dir_time = 0.0;
  for (int i = 0; i < num_leaf_vertices; ++i) {
    int id = leaf_vertices[i]->id;
    std::vector<GbplannerVertex*> path;
    local_graph_->getShortestPath(id, local_graph_rep_, true, path);
    int path_size = path.size();
    int num_unknown_voxels = 0;
    if (path_size > 1) {
      // At least 2 vertices: root + leaf.
      double path_gain = 0;
      double lambda = planning_params_.path_length_penalty;
      bool inadmissible_edge = false;
      Timer tc;
      for (int ind = 0; ind < path_size; ++ind) {
        GbplannerVertex* v_id = path[ind];
        double path_length =
            local_graph_->getShortestDistance(v_id->id, local_graph_rep_);
        double vol_gain;
        if(assisted)
        {
          vol_gain =
              v_id->total_gain *
              exp(-v_id->is_hanging * planning_params_.hanging_vertex_penalty);
        }
        else
        {
          vol_gain =
              v_id->vol_gain.gain *
              exp(-v_id->is_hanging * planning_params_.hanging_vertex_penalty);
        }

        if (ind > 0 && robot_params_.type == RobotType::kGroundRobot) {
          double inclination =
              edge_inclinations_[path[ind]->id][path[ind - 1]->id];
          double max_negative_inclination = 0.37;
          if (inclination > max_negative_inclination) {
          }
          Eigen::Vector3d segment =
              path[ind]->state.head(3) - path[ind - 1]->state.head(3);
          if ((path[ind]->state(2) - path[ind - 1]->state(2)) <
              -map_manager_->getResolution()) {
            // Negative slope
            if (inclination > max_negative_inclination ||
                (std::atan2(std::abs(segment(2)), segment.head(2).norm())) >
                    max_negative_inclination) {
              path_gain = 0.0;
              negative_edge_leafs.push_back(leaf_vertices[i]->id);
              inadmissible_negative_edges.push_back(
                  path[ind - 1]->state.head(3));
              inadmissible_negative_edges.push_back(path[ind]->state.head(3));
              inadmissible_edge = true;
            }
          }
        }
        if (!inadmissible_edge) {
          path_gain += vol_gain * exp(-lambda * path_length);
          v_id->vol_gain.accumulative_gain = path_gain;
          num_unknown_voxels += v_id->vol_gain.num_unknown_voxels;
          if (v_id->vol_gain.is_frontier && !v_id->is_hanging)
            frontier_exists = true;
        }
      }
      path_len_time += tc.endTimer();
      ++path_len_count;
      if (inadmissible_edge) {
        continue;
      }

      // Compare with exploring direction to penalty not-forward paths.
      tc.reset();
      double lambda2 = planning_params_.path_direction_penalty;
      std::vector<Eigen::Vector3d> path_list;
      local_graph_->getShortestPath(id, local_graph_rep_, true, path_list);
      double fw_ratio =
          Trajectory::computeDistanceBetweenTrajectoryAndDirection(
              path_list, exploring_direction_, 0.2, true);
      path_gain *= exp(-lambda2 * fw_ratio);

      if (path_gain > best_gain) {
        best_gain = path_gain;
        best_path_id = id;
      }
      ++path_dir_count;
      path_dir_time += tc.endTimer();
    }
  }
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Mean path len timer: %f", path_len_time / path_len_count);
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Mean path dir timer: %f", path_dir_time / path_dir_count);

  Timer tc;
  double dt;

  if (planning_params_.auto_global_planner_enable) {
    if (!frontier_exists) {
      ++num_low_gain_iters_;
      ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "No frontier found in this round. Total rounds: %d",
               num_low_gain_iters_);
    } else {
      if (num_low_gain_iters_ > 0) --num_low_gain_iters_;
    }
    if (num_low_gain_iters_ >= planning_params_.max_low_gain_iters) {
      ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "%d consecutinve low gain paths, triggering global planner.",
               num_low_gain_iters_);
      num_low_gain_iters_ = 0;
      auto_global_planner_trig_ = true;
      return Rrg::GraphStatus::NOT_OK;
    }
  }

  dt = tc.endTimer();
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "No frontier check timer: %f", dt);

  // Visualization at the end.
  tc.reset();
  visualization_->visualizeShortestPaths(local_graph_, local_graph_rep_);
  if (best_gain > 0) {
    visualization_->visualizeBestPaths(local_graph_, local_graph_rep_, 10,
                                       best_path_id);
  }

  visualization_->visualizeNegativePaths(inadmissible_negative_edges,
                                         local_graph_, local_graph_rep_);
  dt = tc.endTimer();
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Visualization timer: %f", dt);

  tc.reset();
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
    ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Best path: with gain [%f] and ID [%d] ", best_gain, best_path_id);
    gstatus = Rrg::GraphStatus::OK;
    // visualization_->visualizeGlobalGraph(global_graph_);
  } else {
    gstatus = Rrg::GraphStatus::NO_GAIN;
  }
  dt = tc.endTimer();
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Best vertex selection timer: %f", dt);

  stat_->evaluate_graph_time = GET_ELAPSED_TIME(ttime);
  t2 = std::chrono::high_resolution_clock::now();
  stat_chrono_->evaluate_graph_time =
      std::chrono::duration<double, std::milli>(t2 - t1).count();
  stat_chrono_->printTime("Chrono");
  publishTimings(stat_chrono_);

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
  // Use the spherical with Cartesian (Forward, left, up) coordinate
  Eigen::Vector3d p_dir_norm = p_dir.normalized();
  double yaw_angle = std::atan2(p_dir_norm.y(), p_dir_norm.x());
  double pitch_angle =
      -std::atan2(p_dir_norm.z(), std::sqrt(p_dir_norm.x() * p_dir_norm.x() +
                                            p_dir_norm.y() * p_dir_norm.y()));
  quat_W2S = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(pitch_angle, Eigen::Vector3d::UnitY());

  pcl::PointCloud<pcl::PointXYZ>* pcl_tf(new pcl::PointCloud<pcl::PointXYZ>());
  Eigen::Translation<double, 3> trans_W2S(p_center);
  Eigen::Transform<double, 3, Eigen::Affine> tf_W2S(trans_W2S * quat_W2S);
  pcl::transformPointCloud(*obstacle_pcl, *pcl_tf, tf_W2S.inverse());

  // add a local bounding box
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
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[IMPRV] Path too close to obstacle");
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
  // all hyperplanes Not sure how to get the closed-form solution, also issue
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
  // test. --> move to the expandGlobalGraph
  // 2) Re-update all previous frontiers in graph if they are still
  // frontiers by checking if the are surrounded by normal vertices in local
  // graph, change the status to normal. 3) Sort all the path with frontiers
  // into desending list. 4) For each path, check if the frontier is surrounded
  // by normal vertices or any frontiers. If yes, don't add this path;
  // otherwise, add this path to the global graph.

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Global graph: %d vertices, %d edges.",
           global_graph_->getNumVertices(), global_graph_->getNumEdges());
  bool update_global_frontiers = true;
  if (update_global_frontiers) {
    std::vector<GbplannerVertex*> global_frontiers;
    int num_vertices = global_graph_->getNumVertices();
    for (int id = 0; id < num_vertices; ++id) {
      if (global_graph_->getVertex(id)->type == VertexType::kFrontier) {
        global_frontiers.push_back(global_graph_->getVertex(id));
      }
    }
    ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Have %d frontiers from global graph.",
             (int)global_frontiers.size());
    for (auto& v : global_frontiers) {
      computeVolumetricGainRayModelNoBound(v->state, v->vol_gain);
      if (!v->vol_gain.is_frontier) v->type = VertexType::kUnvisited;
    }
  }

  // Get all potential frontiers at leaf vertices of newly sampled local graph.
  std::vector<GbplannerVertex*> leaf_vertices;
  local_graph_->getLeafVertices(leaf_vertices);
  std::vector<GbplannerVertex*> frontier_vertices;
  for (auto& v : leaf_vertices) {
    if(planning_params_.add_only_frontiers_to_global_graph) {
      if (v->type == VertexType::kFrontier) {
        frontier_vertices.push_back(v);
      }
    }
    else {
      frontier_vertices.push_back(v);
    }
  }
  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Got %d leaf vertices from newly local graph.",
           (int)leaf_vertices.size());
  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Got %d frontiers from newly local graph.",
           (int)frontier_vertices.size());

  // Clustering the frontier and add principle path to the global.
  std::vector<int> cluster_ids = performShortestPathsClustering(
      local_graph_, local_graph_rep_, frontier_vertices);
  visualization_->visualizeClusteredPaths(local_graph_, local_graph_rep_,
                                          frontier_vertices, cluster_ids);
  // const double kRangeCheck = 1.0;
  // const double kUpdateRadius = 3.0;
  double kRangeCheck;
  double kUpdateRadius;
  if(planning_params_.add_only_frontiers_to_global_graph) {
    kRangeCheck = 1.0;
    kUpdateRadius = 2.0;
  }
  else {
    kRangeCheck = 0.7;
    kUpdateRadius = 0.7;
  }
  for (int i = 0; i < cluster_ids.size(); ++i) {
    GbplannerVertex* nearest_vertex = NULL;
    // To add principal path, verify if around that area already have vertices
    // before. Also if the robot already passed that area before.
    if (!global_graph_->getNearestVertexInRange(
            &(local_graph_->getVertex(cluster_ids[i])->state), kRangeCheck,
            &nearest_vertex)) {
      StateVec* nearest_state = NULL;
      if (!robot_state_hist_->getNearestStateInRange(
              &(local_graph_->getVertex(cluster_ids[i])->state), kUpdateRadius,
              &nearest_state)) {
        std::vector<GbplannerVertex*> path;
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

bool Rrg::resetTimerCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    resetMissionTimer();
    ROS_WARN_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "Mission time reset.");
    res.success = true;
    return true;
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

void Rrg::expandGlobalGraphFrontierAdditionTimerCallback(
    const ros::TimerEvent& event) {
  // map_manager_->voxelAllocationTest(Eigen::Vector3d(15.0, 3.0, 2.0));
  
  if (add_frontiers_to_global_graph_) {
    auto t1 = std::chrono::high_resolution_clock::now();
    auto t2 = t1;

    ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Timer: Adding frontiers to global graph");
    add_frontiers_to_global_graph_ = false;
    addFrontiers(0);  // id given as 0 because it is not used
    
    t2 = std::chrono::high_resolution_clock::now();
    double chrono_time = std::chrono::duration<double, std::milli>(t2 - t1).count();
    ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Add frontiers time: %f", chrono_time);
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
  //

  ros::Time time_lim;
  START_TIMER(time_lim);
  auto t1 = std::chrono::high_resolution_clock::now();
  auto t2 = t1;

  if (planner_trigger_count_ == 0) return;

  bool update_global_frontiers = false;
  if (update_global_frontiers) {
    std::vector<GbplannerVertex*> global_frontiers;
    int num_vertices = global_graph_->getNumVertices();
    for (int id = 0; id < num_vertices; ++id) {
      if (global_graph_->getVertex(id)->type == VertexType::kFrontier) {
        global_frontiers.push_back(global_graph_->getVertex(id));
      }
    }
    for (auto& v : global_frontiers) {
      computeVolumetricGainRayModel(v->state, v->vol_gain);
      if (!v->vol_gain.is_frontier) v->type = VertexType::kUnvisited;
    }
  }

  std::vector<GbplannerVertex*> unvisited_vertices;
  int global_graph_size = global_graph_->getNumVertices();
  for (int id = 0; id < global_graph_size; ++id) {
    if (global_graph_->getVertex(id)->type == VertexType::kUnvisited) {
      unvisited_vertices.push_back(global_graph_->getVertex(id));
    }
  }
  if (unvisited_vertices.empty()) return;

  const double kLocalBoxRadius = 10;
  const double kLocalBoxRadiusSq = kLocalBoxRadius * kLocalBoxRadius;
  std::vector<Eigen::Vector3d> cluster_centroids;
  std::vector<GbplannerVertex*> unvisited_vertices_remain;
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

  // Expand global graph.
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
      GbplannerVertex new_vertex(-1, StateVec::Zero());
      if (!sampleVertex(random_sampler_, centroid_state, new_vertex)) continue;
      if (new_vertex.is_hanging) continue;
      if (robot_params_.type == RobotType::kGroundRobot) {
        MapManager::VoxelStatus vs;
        Eigen::Vector3d new_vertex_pos = new_vertex.state.head(3);
        double ground_height = projectSample(new_vertex_pos, vs);
        if (vs == MapManager::VoxelStatus::kOccupied) {
          new_vertex.state(2) -=
              (ground_height - planning_params_.max_ground_height);
        }
      }
      // Only expand samples in sparse areas & not yet passed by the robot & not
      // closed to any frontiers
      const double kSparseRadius = 5.0;              // m
      const double kOverlappedFrontierRadius = 5.0;  // m
      std::vector<StateVec*> s_res;
      robot_state_hist_->getNearestStates(&new_vertex.state, kSparseRadius,
                                          &s_res);
      if (s_res.size()) continue;
      std::vector<GbplannerVertex*> v_res;
      global_graph_->getNearestVertices(&new_vertex.state, kSparseRadius,
                                        &v_res);
      if (v_res.size()) continue;
      std::vector<GbplannerVertex*> f_res;
      global_graph_->getNearestVertices(&new_vertex.state,
                                        kOverlappedFrontierRadius, &f_res);
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
      expandGraph(global_graph_, new_vertex, rep);
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
  t2 = std::chrono::high_resolution_clock::now();
  double chrono_time = std::chrono::duration<double, std::milli>(t2 - t1).count();
  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Expand global graph time: %f", chrono_time);
}

void Rrg::semanticsCallback(
    const planner_semantic_msgs::SemanticPoint& semantic) {
  // std::cout << "Inside semantic callback" << std::endl;
  StateVec* new_state =
      new StateVec(semantic.point.x, semantic.point.y, semantic.point.z, 0.0);
  Eigen::Vector3d sem((*new_state)[0], (*new_state)[1], (*new_state)[2]);

  if (MapManager::VoxelStatus::kFree !=
      map_manager_->getBoxStatus(sem + robot_params_.center_offset,
                                 robot_box_size_, true)) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[SEMANTICS]: Marker state not free.");
    return;
  }

  GbplannerVertex* temp_nearest_vertex;
  if (!global_graph_->getNearestVertex(new_state, &temp_nearest_vertex)) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[SEMANTICS]: No nearest vertex found.");
    return;
  }
  std::vector<GbplannerVertex*> nearest_vertices;
  GbplannerVertex* nearest_vertex;
  Eigen::Vector3d nv(temp_nearest_vertex->state[0],
                     temp_nearest_vertex->state[1],
                     temp_nearest_vertex->state[2]);

  // Range of search = 2*closest node
  if (!global_graph_->getNearestVertices(new_state, 2.0 * ((sem - nv).norm()),
                                         &nearest_vertices)) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[SEMANTICS]: No nearest vertex found.");
    return;
  }

  std::vector<geometry_msgs::Pose> path_ret;
  bool path_status = false;
  const int kMaxNumTrials = 5;
  for (int i = 0; (i < nearest_vertices.size()) && (i < kMaxNumTrials); ++i) {
    nearest_vertex = nearest_vertices[i];
    geometry_msgs::Pose start, end;
    end.position.x = semantic.point.x;
    end.position.y = semantic.point.y;
    end.position.z = semantic.point.z;
    end.orientation.x = 0.0;
    end.orientation.y = 0.0;
    end.orientation.z = 0.0;
    end.orientation.w = 1.0;

    start.position.x = nearest_vertex->state[0];
    start.position.y = nearest_vertex->state[1];
    start.position.z = nearest_vertex->state[2];
    start.orientation.x = 0.0;
    start.orientation.y = 0.0;
    start.orientation.z = 0.0;
    start.orientation.w = 1.0;

    path_status = search(start, end, false, path_ret);

    if (path_status) break;
  }

  if (!path_status) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[SEMANTICS]: Cannot connect to nearest node");
    return;
  } else {
    std::vector<GbplannerVertex*> semantic_path;
    ROS_INFO_COND(global_verbosity >= Verbosity::INFO, 
        "[SEMANTICS]: Found a path to the semantic point with %d vertices.",
        (int)path_ret.size());
    if (path_ret.size() > 2) {
      for (int i = 1; i < path_ret.size(); ++i) {
        StateVec next_state(path_ret[i].position.x, path_ret[i].position.y,
                            path_ret[i].position.z, 0.0);
        GbplannerVertex* vert = new GbplannerVertex(i, next_state);
        if (i == path_ret.size() - 1) {
          vert->semantic_class.value = semantic.type.value;
          vert->type = VertexType::kFrontier;
          vert->is_leaf_vertex = true;
        } else {
          vert->semantic_class.value =
              planner_semantic_msgs::SemanticClass::kNone;
        }
        semantic_path.push_back(vert);
      }
    } else {
      StateVec next_state(path_ret[1].position.x, path_ret[1].position.y,
                          path_ret[1].position.z, 0.0);
      GbplannerVertex* vert = new GbplannerVertex(global_graph_->generateVertexID(), next_state);
      vert->semantic_class.value = semantic.type.value;
      vert->type = VertexType::kFrontier;
      vert->is_leaf_vertex = true;
      global_graph_->addVertex(vert);
      Eigen::Vector3d tgt_pos(vert->state[0], vert->state[1], vert->state[2]);
      Eigen::Vector3d src_pos(nearest_vertex->state[0],
                              nearest_vertex->state[1],
                              nearest_vertex->state[2]);
      global_graph_->addEdge(nearest_vertex, vert, (tgt_pos - src_pos).norm());
    }
  }
  visualization_->visualizeGlobalGraph(global_graph_);
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
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Start searching ...");
  int final_target_id;
  // Eigen::Vector3d robot_box_og = robot_box_size_;
  // robot_box_size_ = robot_params_.size;
  sampling_params.reached_target_radius = 1.0;
  sampling_params.num_vertices_max = 800;
  // map_manager_->setRobotRadius((robot_params_.size + robot_params_.size_extension_min).maxCoeff() / 2.0);
  // map_manager_->setBoxCheckMethod(1);
  // map_manager_->setLineCheckMethod(2);
  ConnectStatus status = findPathToConnect(
      source, target, graph_search, sampling_params, final_target_id, path_ret);
  // robot_box_size_ = robot_box_og;
  // map_manager_->setRobotRadius(robot_box_size_.maxCoeff() / 2.0);
  // map_manager_->setBoxCheckMethod(planning_params_.box_check_method);
  // map_manager_->setLineCheckMethod(planning_params_.line_check_method);
  // visualization
  visualization_->visualizeGraph(graph_search);
  visualization_->visualizeSampler(random_sampler_to_search_);
  if (status == ConnectStatus::kSuccess)
    return true;
  else
    return false;
}

ConnectStatus Rrg::findPathToConnect(
    StateVec& source, StateVec& target,
    std::shared_ptr<GraphManager> graph_manager, RandomSamplingParams& params,
    int& final_target_id, std::vector<geometry_msgs::Pose>& path_ret) {
  ConnectStatus status;
  path_ret.clear();
  graph_manager->reset();

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Search a path from src [%f,%f,%f] to tgt [%f,%f,%f]", source[0],
           source[1], source[2], target[0], target[1], target[2]);

  // Check a corner case if exists a direct collision-free path to connect
  // source and target.
  MapManager::VoxelStatus voxel_state;
  bool try_straight_path = true;
  if (try_straight_path) {
    Eigen::Vector3d src_pos(source[0], source[1], source[2]);
    Eigen::Vector3d tgt_pos(target[0], target[1], target[2]);
    bool admissible = false;
    if(unknown_as_free_)
    {
      voxel_state = map_manager_->getPathStatus(
        src_pos + robot_params_.center_offset,
        tgt_pos + robot_params_.center_offset, robot_box_size_, true);
      if(voxel_state != MapManager::VoxelStatus::kOccupied)
        admissible = true;
    }
    else
    {
      voxel_state = map_manager_->getPathStatus(
          src_pos + robot_params_.center_offset,
          tgt_pos + robot_params_.center_offset, robot_box_size_, false);
      if(voxel_state == MapManager::VoxelStatus::kFree)
        admissible = true;
    }
    if (admissible) {
      ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Try straight path...");
      // Add source to the graph.
      GbplannerVertex* source_vertex =
          new GbplannerVertex(graph_manager->generateVertexID(), source);
      graph_manager->addVertex(source_vertex);
      // Add target to the graph.
      GbplannerVertex* target_vertex =
          new GbplannerVertex(graph_manager->generateVertexID(), target);
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
          ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Source position contains Occupied voxels --> Stop.");
          break;
        case MapManager::VoxelStatus::kUnknown:
          ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Source position contains Unknown voxels  --> Stop.");
          break;
        case MapManager::VoxelStatus::kFree:
          break;
      }
      status = ConnectStatus::kErrorCollisionAtSource;
      return status;
    }
  }
  // Add source to the graph.
  GbplannerVertex* source_vertex = new GbplannerVertex(graph_manager->generateVertexID(), source);
  graph_manager->addVertex(source_vertex);

  // Start sampling points and add to the graph.
  bool reached_target = false;
  int num_paths_to_target = 0;
  std::vector<GbplannerVertex*> target_neigbors;
  int loop_count = 0;
  int num_vertices = 0;
  int num_edges = 0;
  random_sampler_to_search_.reset();
  bool stop_sampling = false;
  while (!stop_sampling) {
    GbplannerVertex new_vertex(-1, StateVec::Zero());
    if (!sampleVertex(random_sampler_to_search_, source, new_vertex)) continue;
    // StateVec &new_state = new_vertex->state;
    ExpandGraphReport rep;
    expandGraph(graph_manager, new_vertex, rep);
    if (rep.status == ExpandGraphStatus::kSuccess) {
      num_vertices += rep.num_vertices_added;
      num_edges += rep.num_edges_added;
      // Check if this state reached the target.
      Eigen::Vector3d radius_vec(new_vertex.state[0] - target[0],
                                 new_vertex.state[1] - target[1],
                                 new_vertex.state[2] - target[2]);
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
  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Built a graph with %d vertices and %d edges.",
           graph_manager->getNumVertices(), graph_manager->getNumEdges());

  // Try to add target to graph as well.
  bool added_target = false;
  GbplannerVertex* target_vertex = NULL;
  if (reached_target) {
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Reached target.");
    // Check if the target voxel is free, then try to add to the graph.
    voxel_state = map_manager_->getBoxStatus(
        Eigen::Vector3d(target[0], target[1], target[2]) +
            robot_params_.center_offset,
        robot_box_size_, true);
    if (voxel_state == MapManager::VoxelStatus::kFree) {
      ExpandGraphReport rep;
      expandGraph(graph_manager, target, rep);
      if (rep.status == ExpandGraphStatus::kSuccess) {
        ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Added target to the graph successfully.");
        num_vertices += rep.num_vertices_added;
        num_edges += rep.num_edges_added;
        added_target = true;
        target_vertex = rep.vertex_added;
      } else {
        ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Cannot expand the graph to connect to the target.");
      }
    } else {
      ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Target is not free, failed to add to the graph.");
    }
  } else {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "ConnectStatus::kErrorNoFeasiblePath");
    status = ConnectStatus::kErrorNoFeasiblePath;
    return status;
  }

  // Get shortest path to the goal.
  ShortestPathsReport graph_rep;
  graph_manager->findShortestPaths(graph_rep);

  // Get id list of the shortest path.
  if (!added_target) {
    ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Sorting best path.");
    // Sort all the shortest path that go to target neigbors based on distance
    // in ascending order.
    std::sort(target_neigbors.begin(), target_neigbors.end(),
              [&graph_manager, &graph_rep](const GbplannerVertex* a, const GbplannerVertex* b) {
                return graph_manager->getShortestDistance(a->id, graph_rep) <
                       graph_manager->getShortestDistance(b->id, graph_rep);
              });
    // Pick the shortest one.
    target_vertex = target_neigbors[0];
  }
  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Get shortest path [%d] from %d path.", target_vertex->id,
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

  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Finish searching.");
  status = ConnectStatus::kSuccess;
  visualization_->visualizeBestPaths(graph_manager, graph_rep, 0,
                                     final_target_id);
  return status;
}

bool Rrg::loadParams(bool shared_params) {
  // shared_params == false -> global space params and robot params not set
  // through other code (ex. behaviour planner) Get the prefix name of the
  // parameters.
  std::string ns = ros::this_node::getName();

  // Load all relevant parameters.
  if (!sensor_params_.loadParams(ns + "/SensorParams")) return false;

  if (!planning_params_.loadParams(ns + "/PlanningParams")) return false;
  world_frame_ = planning_params_.global_frame_id;
  std::vector<double> empty_vec;
  for (int i = 0; i < planning_params_.num_vertices_max; ++i) {
    empty_vec.push_back(0.0);
  }
  for (int i = 0; i < planning_params_.num_vertices_max; ++i) {
    edge_inclinations_.push_back(empty_vec);
  }

  // if(planning_params_.annotate_map_with_camera) {
  // }
  camera_annotation_params_.loadParams(ns + "/CameraAnnotationParams");

  if(planning_params_.freespace_cloud_enable) {
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "FreeFrustumParams disabled.");
    if (!free_frustum_params_.loadParams(ns + "/FreeFrustumParams")) {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "No setting for FreeFrustumParams.");
    }
  }

  if(planning_params_.inspection_planning) {
    planning_params_.use_camera_gain = true;
    planning_params_.yaw_tangent_correction = false;
    // planning_params_.leafs_only_for_volumetric_gain = false;
    planning_params_.cluster_vertices_for_gain = false;
    planning_params_.path_safety_enhance_enable = false;
    planning_params_.annotate_map_with_camera = true;
  }
  else {
    planning_params_.use_camera_gain = false;
    planning_params_.yaw_tangent_correction = true;
    // planning_params_.annotate_map_with_camera = true;
  }

  if(!planning_params_.yaw_tangent_correction) {
    ROS_WARN_COND(param_verbosity >= Verbosity::ERROR, "[PlanningParams] yaw_tangent_correction is false, disabling path_safety_enhance_enable.");
    planning_params_.path_safety_enhance_enable = false;
  }

  map_manager_->setRaycastingParams(
      planning_params_.nonuniform_ray_cast,
      planning_params_.ray_cast_step_size_multiplier);
  // auto landing overrules auto homing:
  if (planning_params_.auto_landing_enable &&
      robot_params_.type == RobotType::kAerialRobot) {
    planning_params_.go_home_if_fully_explored = false;
    planning_params_.auto_homing_enable = false;
  }

  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "NoGainZones size: %d", planning_params_.no_gain_zones_list.size());
  if(global_verbosity >= Verbosity::DEBUG) {
    for (int i=0; i<planning_params_.no_gain_zones_list.size();++i)  std::cout << planning_params_.no_gain_zones_list[i] << std::endl;
  }
  if (planning_params_.no_gain_zones_list.size() <= 0) {
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "No NoGainZones.");
    use_no_gain_space_ = false;
  } else {
    for (auto& zone : planning_params_.no_gain_zones_list) {
      BoundedSpaceParams ngz;
      if (!ngz.loadParams(ns + "/NoGainZones/" + zone)) {
        continue;
      }
      no_gain_zones_.push_back(ngz);
    }
    if (no_gain_zones_.size() <= 0) {
      use_no_gain_space_ = false;
    }
  }

  if (!shared_params) {
    if (!robot_params_.loadParams(ns + "/RobotParams")) return false;
    if (!global_space_params_.loadParams(ns + "/BoundedSpaceParams/Global"))
      return false;
  }

  if (!local_space_params_.loadParams(ns + "/BoundedSpaceParams/Local"))
    return false;

  if (!local_search_params_.loadParams(ns + "/BoundedSpaceParams/LocalSearch"))
    return false;
  if (!local_adaptive_params_.loadParams(
          ns + "/BoundedSpaceParams/LocalAdaptiveExp")) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "No setting for adaptive exploration mode.");
  }
  if (!adaptive_obb_->loadParams(ns + "/AdaptiveObbParams")) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "No setting for adaptive exploration mode.");
  }
  adaptive_orig_min_val_ = local_adaptive_params_.min_val;
  adaptive_orig_max_val_ = local_adaptive_params_.max_val;

  // The sampler doesn't load params automatically.
  // Remember to initialize the sampler in initializeParams() function.
  if (!random_sampler_.loadParams(ns +
                                  "/RandomSamplerParams/SamplerForExploration"))
    return false;
  if (!random_sampler_to_search_.loadParams(
          ns + "/RandomSamplerParams/SamplerForSearching"))
    return false;
  if (!random_sampler_adaptive_.loadParams(
          ns + "/RandomSamplerParams/SamplerForAdaptiveExp")) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "No setting for adaptive exploration mode.");
  }

  if (!robot_dynamics_params_.loadParams(ns + "/RobotDynamics")) return false;

  if (!geofence_manager_->loadParams(ns + "/GeofenceParams")) return false;
  if (!darpa_gate_params_.loadParams(ns + "/DarpaGateParams")) return false;

  // @todo A temporary solution to load the velocity setting.
  // planning_params_.v_max = robot_dynamics_params_.v_max;
  // planning_params_.v_homing_max = robot_dynamics_params_.v_homing_max;

  // All other relevant const values should be initialized in this call
  // after loading parameters for all fields.
  initializeParams();
  return true;
}

void Rrg::setGeofenceManager(
    std::shared_ptr<GeofenceManager> geofence_manager) {
  geofence_manager_ = geofence_manager;
}

void Rrg::setSharedParams(const RobotParams& robot_params,
                          const BoundedSpaceParams& global_space_params) {
  robot_params_ = robot_params;
  robot_params_.getPlanningSize(robot_box_size_);

  global_space_params_ = global_space_params;
}

void Rrg::setSharedParams(const RobotParams& robot_params,
                          const BoundedSpaceParams& global_space_params,
                          const BoundedSpaceParams& local_space_params) {
  robot_params_ = robot_params;
  robot_params_.getPlanningSize(robot_box_size_);

  global_space_params_ = global_space_params;
  local_space_params_ = local_space_params;
}

void Rrg::initializeParams() {
  // Compute constant values after loading all parameters to speed up
  // computation later.
  // Set sampler params from BoundedSpaceParams if required.
  random_sampler_.setParams(global_space_params_, local_space_params_);
  random_sampler_to_search_.setParams(global_space_params_,
                                      local_search_params_);

  // Precompute the robot box for planning.
  robot_params_.getPlanningSize(robot_box_size_);
  planning_num_vertices_max_ = planning_params_.num_vertices_max;
  planning_num_edges_max_ = planning_params_.num_edges_max;

  // Get the global bounding box in the setting as default.
  // Visualize in the beginning for checking.
  global_bound_.setDefault(global_space_params_.min_val,
                           global_space_params_.max_val);
  // if (planning_params_.type == PlanningModeType::kAdaptiveExploration) {
  //   visualization_->visualizeWorkspace(
  //       root_vertex_->state, global_space_params_, local_adaptive_params_);
  // } else {
  //   visualization_->visualizeWorkspace(
  //       root_vertex_->state, global_space_params_, local_space_params_);
  // }
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
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, 
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
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, 
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
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, 
        "[GlobalBound] Reset to default: Min [%f, %f, %f], Max [%f, %f, %f]",
        v_min.x(), v_min.y(), v_min.z(), v_max.x(), v_max.y(), v_max.z());
  }
  if (planning_params_.type == PlanningModeType::kAdaptiveExploration) {
    visualization_->visualizeWorkspace(
        root_vertex_->state, global_space_params_, local_adaptive_params_);
  } else {
    visualization_->visualizeWorkspace(
        root_vertex_->state, global_space_params_, local_space_params_);
  }
  return true;
}

bool Rrg::setGlobalBound(
    planner_msgs::planner_dynamic_global_bound::Request bound) {
  if (bound.reset_to_default) {
    // reset to an original bounding box.
    global_bound_.reset();
    Eigen::Vector3d v_min, v_max;
    global_bound_.get(v_min, v_max);
    global_space_params_.min_val = v_min;
    global_space_params_.max_val = v_max;
    Eigen::Vector3d zero_vector = Eigen::Vector3d::Zero();
    Eigen::Vector3d center = 0.5 * (v_max - v_min);
    global_space_params_.setRotation(zero_vector);
    global_space_params_.setCenter(center, false);
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, 
        "[GlobalBound] Reset to default: Min [%f, %f, %f], Max [%f, %f, %f]",
        v_min.x(), v_min.y(), v_min.z(), v_max.x(), v_max.y(), v_max.z());
  } else {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "world frame: %s, %s", world_frame_.c_str(), planning_params_.global_frame_id.c_str());
    // Transform points to the world frame
    tf::StampedTransform darpa_to_world_transform;
    try {
      listener_->lookupTransform(world_frame_, bound.header.frame_id,
                                 ros::Time(0), darpa_to_world_transform);
    } catch (tf::TransformException ex) {
      ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "%s", ex.what());
    }
    // Center
    tf::Vector3 center_tf;
    tf::pointMsgToTF(bound.center, center_tf);
    tf::Vector3 center_trans_tf = darpa_to_world_transform * center_tf;
    geometry_msgs::Point center_trans_point;
    tf::pointTFToMsg(center_trans_tf, center_trans_point);
    Eigen::Vector3d center;
    convertPointToEigen(center_trans_point, center);
    // Left
    tf::Vector3 left_tf;
    tf::pointMsgToTF(bound.left, left_tf);
    tf::Vector3 left_trans_tf = darpa_to_world_transform * left_tf;
    geometry_msgs::Point left_trans_point;
    tf::pointTFToMsg(left_trans_tf, left_trans_point);
    Eigen::Vector3d left;
    convertPointToEigen(left_trans_point, left);
    // Up
    tf::Vector3 up_tf;
    tf::pointMsgToTF(bound.up, up_tf);
    tf::Vector3 up_trans_tf = darpa_to_world_transform * up_tf;
    geometry_msgs::Point up_trans_point;
    tf::pointTFToMsg(up_trans_tf, up_trans_point);
    Eigen::Vector3d up;
    convertPointToEigen(up_trans_point, up);
    // Front
    tf::Vector3 front_tf;
    tf::pointMsgToTF(bound.front, front_tf);
    tf::Vector3 front_trans_tf = darpa_to_world_transform * front_tf;
    geometry_msgs::Point front_trans_point;
    tf::pointTFToMsg(front_trans_tf, front_trans_point);
    Eigen::Vector3d front;
    convertPointToEigen(front_trans_point, front);

    Eigen::Vector3d dir1 = front - center;
    Eigen::Vector3d dir2 = left - center;
    Eigen::Vector3d dir3 = up - center;

    Eigen::Vector3d rotations;
    rotations(0) = atan2(dir1(1), dir1(0));       // Yaw
    rotations(1) = asin(-dir1(2) / dir1.norm());  // Pitch
    rotations(2) = asin(dir2(2) / dir2.norm());   // Roll

    Eigen::Vector3d min_val, max_val;
    min_val = -0.5 * Eigen::Vector3d(dir1.norm(), dir2.norm(), dir3.norm());
    max_val = 0.5 * Eigen::Vector3d(dir1.norm(), dir2.norm(), dir3.norm());

    global_space_params_.setRotation(rotations);
    // (max_val - min_val) / 2 gives the center of the cuboid w.r.t the vertex
    // 'center' cuboid_center is the center of the cuboid in gbplanner's fixed
    // frame
    Eigen::Vector3d cuboid_center =
        center + global_space_params_.getRotationMatrix().inverse() *
                     (0.5 * (max_val - min_val));
    global_space_params_.setCenter(cuboid_center, false);
    global_space_params_.setBound(min_val, max_val);
  }

  StateVec local_bb_root;
  if (root_vertex_ != NULL) {
    local_bb_root = root_vertex_->state;
  } else {
    local_bb_root = current_state_;
  }
  if (planning_params_.type == PlanningModeType::kAdaptiveExploration) {
    visualization_->visualizeWorkspace(local_bb_root, global_space_params_,
                                       local_adaptive_params_);
  } else {
    visualization_->visualizeWorkspace(local_bb_root, global_space_params_,
                                       local_space_params_);
  }
  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Visualization done");
  return true;
}

void Rrg::getGlobalBound(planner_msgs::PlanningBound& bound) {
  global_bound_.get(bound.min_val, bound.max_val);
}

void Rrg::computeSemAssistedExplorationGain() {
  const int id_viz = 20;  // random vertex to show volumetric gain.
  ros::Time tim;
  START_TIMER(tim);
  auto t1 = std::chrono::high_resolution_clock::now();
  auto t2 = t1;
  // Compute gain of all vertices in one function.
  for (const auto& v : local_graph_->vertices_map_) {
    bool viz_en = false;
    if (v.second->id == id_viz) viz_en = true;
    if (planning_params_.use_ray_model_for_volumetric_gain) {
      computeVolumetricGainRayModel(v.second->state, v.second->vol_gain,
                                      viz_en);
    } else {
      computeVolumetricGain(v.second->state, v.second->vol_gain, viz_en);
    }
    if (v.second->vol_gain.is_frontier) v.second->type = VertexType::kFrontier;

    // std::cout << "Vol gain computed" << std::endl;

    SemanticGain sem_gain;
    computeSemanticGainRayModel(v.second, sem_gain);

    // std::cout << "Sem gain computed" << std::endl;

    v.second->total_gain = (config_->prediction_params.min_exp_weight + 1 - config_->prediction_params.kSem) * v.second->vol_gain.normalized_gain
                           + (config_->prediction_params.kSemPriorWeight + config_->prediction_params.kSem) * sem_gain.normalized_gain;
  }
  stat_->compute_exp_gain_time = GET_ELAPSED_TIME(tim);
  t2 = std::chrono::high_resolution_clock::now();
  stat_chrono_->compute_exp_gain_time =
      std::chrono::duration<double, std::milli>(t2 - t1).count();
}

void Rrg::computeExplorationGain(bool only_leaf_vertices) {
  const int id_viz = 20;  // random vertex to show volumetric gain.
  ros::Time tim;
  START_TIMER(tim);
  auto t1 = std::chrono::high_resolution_clock::now();
  auto t2 = t1;
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
  t2 = std::chrono::high_resolution_clock::now();
  stat_chrono_->compute_exp_gain_time =
      std::chrono::duration<double, std::milli>(t2 - t1).count();
}

void Rrg::computeExplorationGain() {
  bool only_leaf_vertices = planning_params_.leafs_only_for_volumetric_gain;
  bool clustering = planning_params_.cluster_vertices_for_gain;
  const int id_viz = 20;  // random vertex to show volumetric gain.
  const double clustering_range = planning_params_.clustering_radius;
  int num_clusters = 0;
  ros::Time tim;
  START_TIMER(tim);
  auto t1 = std::chrono::high_resolution_clock::now();
  auto t2 = t1;
  std::unordered_map<int, GbplannerVertex*> vertex_map = local_graph_->vertices_map_;
  std::list<int> vertex_ids;
  for (int i = 0; i < local_graph_->getNumVertices(); ++i) {
    if (vertex_map[i]->is_leaf_vertex) {
      vertex_ids.push_front(i);
    } else {
      vertex_ids.push_back(i);
    }
  }

  while (!vertex_ids.empty()) {
    int v_id = (*vertex_ids.begin());
    vertex_ids.remove(v_id);
    ++num_clusters;
    bool viz_en = false;
    if (v_id == id_viz) viz_en = true;
    if (planning_params_.use_ray_model_for_volumetric_gain) {
      if (vertex_map[v_id]->is_leaf_vertex) {
        computeVolumetricGainRayModel(
            vertex_map[v_id]->state, vertex_map[v_id]->vol_gain, viz_en, false);
      } else {
        if (!only_leaf_vertices) {
          computeVolumetricGainRayModel(vertex_map[v_id]->state,
                                        vertex_map[v_id]->vol_gain, viz_en,
                                        true);
        }
      }
    } else {
      if ((!only_leaf_vertices) || (vertex_map[v_id]->is_leaf_vertex))
        computeVolumetricGain(vertex_map[v_id]->state,
                              vertex_map[v_id]->vol_gain, viz_en);
    }
    // Remove vertices in vicinity:
    if (clustering) {
      if ((!only_leaf_vertices) || (vertex_map[v_id]->is_leaf_vertex)) {
        std::vector<GbplannerVertex*> nearest_vertices;
        local_graph_->getNearestVertices(&vertex_map[v_id]->state,
                                         clustering_range, &nearest_vertices);
        for (auto v : nearest_vertices) {
          std::list<int>::iterator it;
          it = std::find(vertex_ids.begin(), vertex_ids.end(), v->id);
          if (it != vertex_ids.end()) {
            // This vertex was not assigned a gain yet
            v->vol_gain = vertex_map[v_id]->vol_gain;
            vertex_ids.remove(v->id);
          }
        }
      }
    }
    if (vertex_map[v_id]->vol_gain.is_frontier)
      vertex_map[v_id]->type = VertexType::kFrontier;
  }
  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Num clusters: %d", num_clusters);
  stat_->compute_exp_gain_time = GET_ELAPSED_TIME(tim);
  t2 = std::chrono::high_resolution_clock::now();
  stat_chrono_->compute_exp_gain_time =
      std::chrono::duration<double, std::milli>(t2 - t1).count();
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

void Rrg::computeInspectionGainRayModel(GbplannerVertex* vert) {
  vert->vol_gain.reset();

  // std::cout << "GbplannerVertex " << vert->id << ":" << std::endl;
  // std::cout

  for(std::string sensor_name : planning_params_.inspection_sensor_list) {
    std::vector<VoxelLog> voxel_logs;
    SensorParamsBase sensor = sensor_params_.sensor[sensor_name];

    // std::cout << "Sensor: " << sensor_name << std::endl;
    
    map_manager_->getCameraScanStatus(vert->state, sensor, voxel_logs);
    // std::cout << voxel_logs.size() << std::endl;

    for(VoxelLog log : voxel_logs) {
      if (global_space_params_.isInsideSpace(log.voxel_center)) {
        // valid voxel.
        bool no_gain_zone_cleared = true;
        if (use_no_gain_space_) {
          for (auto& zone : no_gain_zones_) {
            if (zone.isInsideSpace(log.voxel_center)) {
              no_gain_zone_cleared = false;
              break;
            }
          }
        }
        if(no_gain_zone_cleared) {
          vert->vol_gain.unseen_voxel_hash_keys.insert(log.voxel_hash);
        }
      }
    }
  }

  // std::cout << "Total num unseen keys:" << vert->vol_gain.unseen_voxel_hash_keys.size() << std::endl;
}

void Rrg::computeSemanticGainRayModel(GbplannerVertex* vert, SemanticGain &gain)
{
  double total_normalization_factor = 0.0;
  for(std::string sensor_name : planning_params_.inspection_sensor_list) {
    SensorParamsBase sensor = sensor_params_.sensor[sensor_name];
    std::vector<VoxelLog> voxel_logs;
    map_manager_->getSemanticScanStatus(vert->state, sensor, voxel_logs);

    for(VoxelLog log : voxel_logs) {
      if (global_space_params_.isInsideSpace(log.voxel_center)) {
        gain.voxel_hash_keys.insert(log.voxel_hash);
      }
    }
    total_normalization_factor += sensor.getMaxGain();
  }

  gain.gain = planning_params_.unknown_voxel_gain * gain.voxel_hash_keys.size();
  gain.normalized_gain = gain.voxel_hash_keys.size() / total_normalization_factor;
}

void Rrg::computeVolumetricGainRayModel(StateVec& state, VolumetricGain& vgain,
                                        bool vis_en, bool iterative) {
  vgain.reset();

  std::vector<std::tuple<int, int, int, int>> gain_log;
  std::vector<std::pair<Eigen::Vector3d, MapManager::VoxelStatus>> voxel_log;
  int raw_unk_voxels_count = 0;
  // @TODO tung.
  // Compute for each sensor in the exploration sensor list.
  // However, this would be a problem if those sensors have significant overlap.
  double total_normalization_factor = 0;
  for (int ind = 0; ind < planning_params_.exp_sensor_list.size(); ++ind) {
    std::string sensor_name = planning_params_.exp_sensor_list[ind];

    Eigen::Vector3d origin(state[0], state[1], state[2]);
    std::tuple<int, int, int> gain_log_tmp;
    std::vector<std::pair<Eigen::Vector3d, MapManager::VoxelStatus>>
        voxel_log_tmp;
    std::vector<Eigen::Vector3d> multiray_endpoints;
    sensor_params_.sensor[sensor_name].getFrustumEndpoints(state,
                                                           multiray_endpoints);

    if(planning_params_.use_camera_gain) {
      map_manager_->getCameraScanStatus(origin, multiray_endpoints, gain_log_tmp,
                                  voxel_log_tmp,
                                  sensor_params_.sensor[sensor_name]);  
    }
    else {
      map_manager_->getScanStatus(origin, multiray_endpoints, gain_log_tmp,
                                  voxel_log_tmp,
                                  sensor_params_.sensor[sensor_name]);
    }

    int num_unknown_voxels = 0, num_free_voxels = 0, num_occupied_voxels = 0,
        num_unknown_surf_voxels = 0;
    // num_unknown_voxels = std::get<0>(gain_log_tmp);
    // num_free_voxels = std::get<1>(gain_log_tmp);
    // num_occupied_voxels = std::get<2>(gain_log_tmp);
    // ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Total number of voxels queried: %d", (num_unknown_voxels +
    // num_free_voxels + num_occupied_voxels)); num_unknown_surf_voxels =
    // std::get<3>(gain_log_tmp); Have to remove those not belong to the local
    // bound. At the same time check if this is frontier.

    for (auto& vl : voxel_log_tmp) {
      Eigen::Vector3d voxel = vl.first;
      MapManager::VoxelStatus vs = vl.second;
      if (vs == MapManager::VoxelStatus::kUnknown) ++raw_unk_voxels_count;
      if (global_space_params_.isInsideSpace(voxel)) {
        // valid voxel.
        bool no_gain_zone_cleared = true;
        if (use_no_gain_space_) {
          for (auto& zone : no_gain_zones_) {
            if (zone.isInsideSpace(voxel)) {
              no_gain_zone_cleared = false;
              break;
            }
          }
        }
        if (no_gain_zone_cleared) {
          // valid voxel.
          if (vs == MapManager::VoxelStatus::kUnknown) {
            ++num_unknown_voxels;
          } else if (vs == MapManager::VoxelStatus::kFree) {
            ++num_free_voxels;
          } else if (vs == MapManager::VoxelStatus::kOccupied) {
            ++num_occupied_voxels;
          } else {
            ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "Unsupported voxel type.");
          }
          if (vis_en) voxel_log.push_back(std::make_pair(voxel, vs));
        }
      }
    }
    gain_log.push_back(std::make_tuple(num_unknown_voxels, num_free_voxels,
                                       num_occupied_voxels,
                                       num_unknown_surf_voxels));
    if (vis_en) {
      visualization_->visualizeRays(state, multiray_endpoints);
    }

    // Check if it is a potential frontier.
    if (sensor_params_.sensor[sensor_name].isFrontier(
            num_unknown_voxels * map_manager_->getResolution())) {
      vgain.is_frontier = true;  // Event E2
    }

    total_normalization_factor += sensor_params_.sensor[sensor_name].getMaxGain();
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
  vgain.normalized_gain = vgain.num_unknown_voxels * map_manager_->getResolution() / total_normalization_factor;

  // Visualize if required.
#if FULL_PLANNER_VIZ
  if (vis_en) {
    Eigen::Vector3d bound_min;
    Eigen::Vector3d bound_max;
    visualization_->visualizeVolumetricGain(bound_min, bound_max, voxel_log,
                                            map_manager_->getResolution());
  }
#endif
}

void Rrg::computeVolumetricGainRayModelNoBound(StateVec& state,
                                               VolumetricGain& vgain) {
  vgain.reset();

  std::vector<std::tuple<int, int, int>> gain_log;
  std::vector<std::pair<Eigen::Vector3d, MapManager::VoxelStatus>> voxel_log;
  // @TODO tung.
  // Compute for each sensor in the exploration sensor list.
  // However, this would be a problem if those sensors have significant overlap.
  for (int ind = 0; ind < planning_params_.exp_sensor_list.size(); ++ind) {
    std::string sensor_name = planning_params_.exp_sensor_list[ind];

    Eigen::Vector3d origin(state[0], state[1], state[2]);
    std::tuple<int, int, int> gain_log_tmp;
    std::vector<std::pair<Eigen::Vector3d, MapManager::VoxelStatus>>
        voxel_log_tmp;
    std::vector<Eigen::Vector3d> multiray_endpoints;
    sensor_params_.sensor[sensor_name].getFrustumEndpoints(state,
                                                           multiray_endpoints);
    map_manager_->getScanStatus(origin, multiray_endpoints, gain_log_tmp,
                                voxel_log_tmp,
                                sensor_params_.sensor[sensor_name]);
    int num_unknown_voxels = 0, num_free_voxels = 0, num_occupied_voxels = 0;
    // Have to remove those not belong to the local bound.
    // At the same time check if this is frontier.

    for (auto& vl : voxel_log_tmp) {
      Eigen::Vector3d voxel = vl.first;
      MapManager::VoxelStatus vs = vl.second;
      if (global_space_params_.isInsideSpace(voxel)) {
        // valid voxel.
        if (vs == MapManager::VoxelStatus::kUnknown) {
          ++num_unknown_voxels;
        } else if (vs == MapManager::VoxelStatus::kFree) {
          ++num_free_voxels;
        } else if (vs == MapManager::VoxelStatus::kOccupied) {
          ++num_occupied_voxels;
        } else {
          ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "Unsupported voxel type.");
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
      (state_for_planning_[2] == 0.0)) {
    planning_params_.use_current_state = true;
  } else {
    planning_params_.use_current_state = false;
  }
}

bool Rrg::setHomingPos() {
  if (global_graph_->getNumVertices() == 0) {
    ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Global graph is empty: add current state as homing position.");
    GbplannerVertex* g_root_vertex =
        new GbplannerVertex(global_graph_->generateVertexID(), current_state_);
    global_graph_->addVertex(g_root_vertex);
    return true;
  } else {
    ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Global graph is not empty, can not set current state as homing.");
    return false;
  }
}

std::vector<geometry_msgs::Pose> Rrg::searchHomingPath(
    std::string tgt_frame, const StateVec& current_state) {
  std::vector<geometry_msgs::Pose> ret_path;
  ret_path.clear();

  if (global_graph_->getNumVertices() <= 1) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[GlobalGraph] Graph is empty, nothing to search for homing.");
    return ret_path;
  }

  StateVec cur_state;
  cur_state << current_state[0], current_state[1], current_state[2],
      current_state[3];
  // offsetZAxis(cur_state);
  if (robot_params_.type == RobotType::kGroundRobot) {
    MapManager::VoxelStatus vs;
    Eigen::Vector3d new_vertex_pos = cur_state.head(3);
    double ground_height = projectSample(new_vertex_pos, vs);
    if (vs == MapManager::VoxelStatus::kOccupied) {
      cur_state(2) -= (ground_height - planning_params_.max_ground_height);
    }
  }
  GbplannerVertex* nearest_vertex = NULL;
  if (!global_graph_->getNearestVertex(&cur_state, &nearest_vertex))
    return ret_path;
  if (nearest_vertex == NULL) return ret_path;
  Eigen::Vector3d origin(nearest_vertex->state[0], nearest_vertex->state[1],
                         nearest_vertex->state[2]);
  Eigen::Vector3d direction(cur_state[0] - origin[0], cur_state[1] - origin[1],
                            cur_state[2] - origin[2]);
  double direction_norm = direction.norm();

  GbplannerVertex* link_vertex = NULL;
  const double kRadiusLimit = 1.0;
  bool connect_state_to_graph = true;
  if (direction_norm <= kRadiusLimit) {
    // Note: if kRadiusLimit <= edge_length_min it will fail with
    // kErrorShortEdge, dirty fix to check max
    // @TODO: find better way to do this.
    // Blindly add a link/vertex to the graph if small radius.
    GbplannerVertex* new_vertex =
        new GbplannerVertex(global_graph_->generateVertexID(), cur_state);
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
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[GlobalGraph] Try to add current state to the graph.");
    ExpandGraphReport rep;
    expandGraph(global_graph_, cur_state, rep);
    if (rep.status == ExpandGraphStatus::kSuccess) {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[GlobalGraph] Added successfully.");
      link_vertex = rep.vertex_added;
    } else {
      // Not implemented solution for this case yet.
      // Hopefully this one will not happen if the global planner always adds
      // vertices from odometry --> naive backtracking.
      connect_state_to_graph = false;
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[GlobalGraph] Can not add current state to graph since: ");
      switch (rep.status) {
        case ExpandGraphStatus::kErrorKdTree:
          ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "kErrorKdTree.");
          break;
        case ExpandGraphStatus::kErrorCollisionEdge:
          ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "kErrorCollisionEdge.");
          break;
        case ExpandGraphStatus::kErrorShortEdge:
          ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "kErrorShortEdge.");
          break;
        default:
          ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "kErrorUnknown.");
          break;
      }
      ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[GlobalGraph] Failed to find global path.");
    }
  }

  if (connect_state_to_graph) {
    if (!global_graph_->findShortestPaths(global_graph_rep_)) {
      ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "[GlobalGraph] Failed to find shortest path.");
      return ret_path;
    }
    std::vector<int> homing_path_id;
    global_graph_->getShortestPath(link_vertex->id, global_graph_rep_, false,
                                   homing_path_id);
    if (homing_path_id.empty() || homing_path_id.back() != 0) {
      ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "[GlobalGraph] Could not find a path to home.");
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
    visualization_->visualizeHomingPath(global_graph_, global_graph_rep_,
                                        link_vertex->id);
  }
  visualization_->visualizeGlobalGraph(global_graph_);

  return ret_path;
}

std::vector<geometry_msgs::Pose> Rrg::getGlobalPath(
    geometry_msgs::PoseStamped& waypoint) {
  std::vector<geometry_msgs::Pose> ret_path;
  if (global_graph_->getNumVertices() <= 1) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[GlobalGraph] Graph is empty, nothing to search for homing.");
    return ret_path;
  }

  StateVec cur_state;
  cur_state << current_state_[0], current_state_[1], current_state_[2],
      current_state_[3];

  StateVec wp;
  wp << waypoint.pose.position.x, waypoint.pose.position.y,
      waypoint.pose.position.z;

  GbplannerVertex* wp_nearest_vertex;
  if (!global_graph_->getNearestVertex(&wp, &wp_nearest_vertex)) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Cannot find any nearby vertex to reposition.");
    return ret_path;
  } else if (wp_nearest_vertex == NULL) {
    return ret_path;
  } else {
    Eigen::Vector3d diff(wp_nearest_vertex->state.x() - wp.x(),
                         wp_nearest_vertex->state.y() - wp.y(),
                         wp_nearest_vertex->state.z() - wp.z());
    if (diff.norm() > max_difference_waypoint_to_graph) {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, 
          "Waypoint is too far from the global graph (distance is '%.2f'; max "
          "allowed is '%.2f'). Choose a closer waypoint.",
          diff.norm(), max_difference_waypoint_to_graph);
      return ret_path;
    }
  }

  if (robot_params_.type == RobotType::kGroundRobot) {
    MapManager::VoxelStatus vs;
    Eigen::Vector3d new_vertex_pos = cur_state.head(3);
    double ground_height = projectSample(new_vertex_pos, vs);
    if (vs == MapManager::VoxelStatus::kOccupied) {
      cur_state(2) -= (ground_height - planning_params_.max_ground_height);
    }
  }
  GbplannerVertex* nearest_vertex = NULL;
  if (!global_graph_->getNearestVertex(&cur_state, &nearest_vertex))
    return ret_path;
  if (nearest_vertex == NULL) return ret_path;
  Eigen::Vector3d origin(nearest_vertex->state[0], nearest_vertex->state[1],
                         nearest_vertex->state[2]);
  Eigen::Vector3d direction(cur_state[0] - origin[0], cur_state[1] - origin[1],
                            cur_state[2] - origin[2]);
  double direction_norm = direction.norm();

  GbplannerVertex* link_vertex = NULL;
  const double kRadiusLimit = 1.0;
  bool connect_state_to_graph = true;
  if (direction_norm <= kRadiusLimit) {
    // Note: if kRadiusLimit <= edge_length_min it will fail with
    // kErrorShortEdge, dirty fix to check max
    // @TODO: find better way to do this.
    // Blindly add a link/vertex to the graph if small radius.
    GbplannerVertex* new_vertex =
        new GbplannerVertex(global_graph_->generateVertexID(), cur_state);
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
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[GlobalGraph] Try to add current state to the graph.");
    ExpandGraphReport rep;
    expandGraph(global_graph_, cur_state, rep);
    if (rep.status == ExpandGraphStatus::kSuccess) {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[GlobalGraph] Added successfully.");
      link_vertex = rep.vertex_added;
    } else {
      // Not implemented solution for this case yet.
      // Hopefully this one will not happen if the global planner always adds
      // vertices from odometry --> naive backtracking.
      connect_state_to_graph = false;
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[GlobalGraph] Can not add current state to graph since: ");
      switch (rep.status) {
        case ExpandGraphStatus::kErrorKdTree:
          ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "kErrorKdTree.");
          break;
        case ExpandGraphStatus::kErrorCollisionEdge:
          ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "kErrorCollisionEdge.");
          break;
        case ExpandGraphStatus::kErrorShortEdge:
          ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "kErrorShortEdge.");
          break;
        default:
          ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "kErrorUnknown.");
          break;
      }
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[GlobalGraph] Failed to find global path.");
    }
  }

  ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Finding a path from current[%d] to vertex[%d].", link_vertex->id,
           wp_nearest_vertex->id);

  if (connect_state_to_graph) {
    if (!global_graph_->findShortestPaths(link_vertex->id, global_graph_rep_)) {
      ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "[GlobalGraph] Failed to find shortest path.");
      return ret_path;
    }
    std::vector<int> global_path_id;
    global_graph_->getShortestPath(wp_nearest_vertex->id, global_graph_rep_,
                                   true, global_path_id);
    if (global_path_id.empty()) {
      ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "[GlobalGraph] Could not find a path to home.");
      return ret_path;
    }
    int global_path_id_size = global_path_id.size();
    for (int i = 0; i < global_path_id_size; ++i) {
      StateVec state = global_graph_->getVertex(global_path_id[i])->state;
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
  }
  visualization_->visualizeGlobalGraph(global_graph_);
  // Modify path if required
  if (planning_params_.path_safety_enhance_enable) {
    ros::Time mod_time;
    START_TIMER(mod_time);
    std::vector<geometry_msgs::Pose> mod_path;
    if (improveFreePath(ret_path, mod_path, true)) {
      ret_path = mod_path;
    }
    double dmod_time = GET_ELAPSED_TIME(mod_time);
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Compute an aternate path for homing in %f(s)", dmod_time);
    visualization_->visualizeModPath(mod_path);
  }

  // Interpolate path
  const double kInterpolationDistance =
      planning_params_.path_interpolation_distance;
  std::vector<geometry_msgs::Pose> interp_path;
  if (Trajectory::interpolatePath(ret_path, kInterpolationDistance,
                                  interp_path)) {
    ret_path = interp_path;
  }
  visualization_->visualizeRefPath(ret_path);
  return ret_path;
}

std::vector<geometry_msgs::Pose> Rrg::getHomingPath(std::string tgt_frame) {
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
  if (clean_short_edges && (robot_params_.type == RobotType::kGroundRobot)) {
    // check if the path is weird: sudden change in the orientation
    // find segments that need to be improved
    // build the graph to reconnect those segments only, keep the remaining the
    // same
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
    if (improveFreePath(ret_path, mod_path, true)) {
      ret_path = mod_path;
      // Re-assign yaw angle after modification.
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

    double dmod_time = GET_ELAPSED_TIME(mod_time);
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Compute an aternate path for homing in %f(s)", dmod_time);
    visualization_->visualizeModPath(mod_path);
  }

  // Interpolate path
  const double kInterpolationDistance =
      planning_params_.path_interpolation_distance;
  std::vector<geometry_msgs::Pose> interp_path;
  if (Trajectory::interpolatePath(ret_path, kInterpolationDistance,
                                  interp_path)) {
    ret_path = interp_path;
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
  GbplannerVertex* root_vertex = new GbplannerVertex(path_graph->generateVertexID(), root_state);
  path_graph->addVertex(root_vertex);

  // Add all remaining vertices of the path.
  std::vector<GbplannerVertex*> vertex_list;
  vertex_list.push_back(root_vertex);
  GbplannerVertex* parent_vertex = root_vertex;
  for (int i = 1; i < path_intp.size(); ++i) {
    StateVec new_state(path_intp[i][0], path_intp[i][1], path_intp[i][2], 0);
    GbplannerVertex* new_vertex = new GbplannerVertex(path_graph->generateVertexID(), new_state);
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
  // Check if needs to land
  if (planning_params_.auto_landing_enable &&
      robot_params_.type == RobotType::kAerialRobot) {
    double time_elapsed = 0.0;
    std::vector<geometry_msgs::Pose> empty_path;
    if ((ros::Time::now()).toSec() != 0.0) {
      if (rostime_start_.toSec() == 0.0) rostime_start_ = ros::Time::now();
      time_elapsed = (double)((ros::Time::now() - rostime_start_).toSec());
    }
    double time_budget_remaining =
        planning_params_.time_budget_before_landing - time_elapsed;
    if (time_budget_remaining <= 0.0) {
      ROS_WARN_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "RAN OUT OF TIME BUDGET --> LANDING.");
      landing_engaged_ = true;
      std_msgs::Bool stop_msg;
      stop_msg.data = true;

      std_srvs::Empty empty_srv;
      if (!landing_engaged_) {
        landing_srv_client_.call(empty_srv);
        pci_reset_pub_.publish(stop_msg);
      }
      /* TODO: What happens if the disarm service fails */
      status = planner_msgs::planner_srv::Response::kForward;
      return empty_path;
    }
  }
  // Decide if need to go home.
  if (planning_params_.auto_homing_enable) {
    status = planner_msgs::planner_srv::Response::kHoming;
    std::vector<geometry_msgs::Pose> homing_path;
    double time_elapsed = 0.0;
    if ((ros::Time::now()).toSec() != 0.0) {
      if (rostime_start_.toSec() == 0.0) rostime_start_ = ros::Time::now();
      time_elapsed = (double)((ros::Time::now() - rostime_start_).toSec());
    }
    double time_budget_remaining =
        planning_params_.time_budget_limit - time_elapsed;
    if (time_budget_remaining <= 0.0) {
      ROS_WARN_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "RAN OUT OF TIME BUDGET --> STOP HERE.");
      return homing_path;
    }
    if (current_battery_time_remaining_ <= 0.0) {
      ROS_WARN_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "RAN OUT OF BATTERY --> STOP HERE.");
      return homing_path;
    }
    // Check two conditions whatever which one comes first.
    double time_remaining =
        std::min(time_budget_remaining, current_battery_time_remaining_);
    // homing_path = getHomingPath(tgt_frame);
    // Start searching from current root vertex of spanning local graph.
    GbplannerVertex* root_vertex = local_graph_->getVertex(0);
    homing_path = searchHomingPath(tgt_frame, root_vertex->state);
    if (!homing_path.empty()) {
      double homing_len = Trajectory::getPathLength(homing_path);
      double time_to_home = homing_len / planning_params_.v_homing_max;
      ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Time to home: %f; Time remaining: %f", time_to_home,
               time_remaining);

      const double kTimeDelta = 20;
      if (time_to_home > time_remaining - kTimeDelta) {
        ROS_WARN_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "REACHED TIME LIMIT: HOMING ENGAGED.");
        if (planning_params_.path_safety_enhance_enable) {
          std::vector<geometry_msgs::Pose> mod_path;
          if (improveFreePath(homing_path, mod_path, true)) {
            homing_path = mod_path;
          }
        }

        const double kInterpolationDistance =
            planning_params_.path_interpolation_distance;
        std::vector<geometry_msgs::Pose> interp_path;
        if (Trajectory::interpolatePath(homing_path, kInterpolationDistance,
                                        interp_path)) {
          homing_path = interp_path;
        }

        visualization_->visualizeRefPath(homing_path);
        homing_engaged_ = true;
        return homing_path;
      }
    } else {
      // @TODO Issue with global graph, cannot find a homing path.
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Can not find a path to return home from here.");
    }
  }

  std::vector<geometry_msgs::Pose> ret, empty_path;
  int id = best_vertex_->id;
  if (id == 0) return ret;

  // Get potentially exploring direction.
  std::vector<Eigen::Vector3d> best_path_3d;
  local_graph_->getShortestPath(id, local_graph_rep_, true, best_path_3d);

  status = planner_msgs::planner_srv::Response::kForward;

  // Extract path.
  double traverse_length = 0;
  double traverse_time = 0;
  std::vector<StateVec> best_path;
  local_graph_->getShortestPath(id, local_graph_rep_, true, best_path);
  Eigen::Vector3d p0(best_path[0][0], best_path[0][1], best_path[0][2]);
  std::vector<GbplannerVertex*> best_path_vertices;
  local_graph_->getShortestPath(best_vertex_->id, local_graph_rep_, true,
                                best_path_vertices);

  const double kLenMin = 1.0;
  const double kLenMinMin = 0.3;
  std::vector<Eigen::Vector3d> path_vec;
  local_graph_->getShortestPath(best_vertex_->id, local_graph_rep_, true,
                                path_vec);
  double total_len = Trajectory::getPathLength(path_vec);
  double len_min_thres = kLenMin;

  len_min_thres = kLenMinMin;

  if (total_len <= len_min_thres) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Best path is too short.");
    return ret;
  }

  std::vector<GbplannerVertex*> ref_vertices;
  for (int i = 0; i < best_path.size(); ++i) {
    // Truncate path upto first hanging vertex. is_hanging will always be false
    // for robot type kAerialRobot
    if (best_path_vertices[i]->is_hanging) {
      break;
    }

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
      ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Segment [%d] is not clear.", i);
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
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Best path:  size = %d, length = %f, time = %f", (int)ret.size(),
           traverse_length, traverse_time);
  // Put this into global graph for homing later.
  bool path_added = false;
  if ((int)ret.size() <= 1) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "No ground attached path");
    return empty_path;
  }

  // Modify the best path.
  // ROS_WARN("Before mod:");
  // for(int i=0; i<ret.size(); ++i) {
  //   std::cout << ret[i].position.x << " " << ret[i].position.y << " " << ret[i].position.z << ", " << tf::getYaw(ret[i].orientation) << std::endl;
  // }
  if (planning_params_.path_safety_enhance_enable) {
    ros::Time mod_time;
    START_TIMER(mod_time);
    std::vector<geometry_msgs::Pose> mod_path;
    if (improveFreePath(ret, mod_path, false)) {
      ret = mod_path;
      addRefPathToGraph(global_graph_, mod_path);
      path_added = true;
    }
    double dmod_time = GET_ELAPSED_TIME(mod_time);
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Compute an aternate path in %f(s)", dmod_time);
    visualization_->visualizeModPath(mod_path);
  }
  // ROS_WARN("After mod:");
  // for(int i=0; i<ret.size(); ++i) {
  //   std::cout << ret[i].position.x << " " << ret[i].position.y << " " << ret[i].position.z << ", " << tf::getYaw(ret[i].orientation) << std::endl;
  // }

  if (!path_added) {
    addRefPathToGraph(global_graph_, ref_vertices);
  }

  // Interpolate path
  if(planning_params_.path_interpolation_distance > 0.0) {
    const double kInterpolationDistance =
        planning_params_.path_interpolation_distance;
    std::vector<geometry_msgs::Pose> interp_path;
    if (Trajectory::interpolatePath(ret, kInterpolationDistance, interp_path)) {
      ret = interp_path;
    }
  }

  visualization_->visualizeRefPath(ret);

  // ROS_WARN("Final Path:");
  // for(int i=0; i<ret.size(); ++i) {
  //   std::cout << ret[i].position.x << " " << ret[i].position.y << " " << ret[i].position.z << ", " << tf::getYaw(ret[i].orientation) << std::endl;
  // }

  return ret;
}

bool Rrg::improveFreePath(const std::vector<geometry_msgs::Pose>& path_orig,
                          std::vector<geometry_msgs::Pose>& path_mod,
                          bool relaxed) {
  // Few heuristics to improve the path.
  // a) Shorten path by reducing intermidiate nodes. (be careful with turning
  // cases) Shorten/reduce some very short paths to prevent small motion and
  // sudden change in angles. b) Adjust nodes to its neighbors to improve safety
  // c) Ignore leaf node of the path to prevent the robot to come too close the
  // obstacle

  if (path_orig.empty()) return false;

  // Interpolate path
  std::vector<geometry_msgs::Pose> path_orig_interp = path_orig;
  double kInterpolationDistance =
      planning_params_.path_interpolation_distance * 2.0;
  std::vector<geometry_msgs::Pose> interp_path_orig;
  if (Trajectory::interpolatePath(path_orig, kInterpolationDistance,
                                  interp_path_orig)) {
    path_orig_interp = interp_path_orig;
  }

  // Feature a) Remove short intermidiate vertices.
  std::vector<geometry_msgs::Pose> path_mod1 = path_orig_interp;

  const double kSegmentLenMin = 0.5;
  bool cont_refine = true;
  while (cont_refine) {
    cont_refine = false;
    if (path_mod1.size() > 2) {
      for (int i = 0; i < (path_mod1.size() - 2); ++i) {
        Eigen::Vector3d p_start(path_mod1[i].position.x,
                                path_mod1[i].position.y,
                                path_mod1[i].position.z);
        Eigen::Vector3d p_int(path_mod1[i + 1].position.x,
                              path_mod1[i + 1].position.y,
                              path_mod1[i + 1].position.z);
        Eigen::Vector3d p_end(path_mod1[i + 2].position.x,
                              path_mod1[i + 2].position.y,
                              path_mod1[i + 2].position.z);
        Eigen::Vector3d segment = p_int - p_start;
        double segment_len = segment.norm();
        // ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Segment length %f.", segment_len);
        if (segment_len < kSegmentLenMin) {
          if ((MapManager::VoxelStatus::kFree ==
               map_manager_->getPathStatus(p_start, p_end, robot_box_size_,
                                           false)) &&
              (!planning_params_.geofence_checking_enable ||
               (GeofenceManager::CoordinateStatus::kOK ==
                geofence_manager_->getPathStatus(
                    Eigen::Vector2d(p_start[0], p_start[1]),
                    Eigen::Vector2d(p_end[0], p_end[1]),
                    Eigen::Vector2d(robot_box_size_[0],
                                    robot_box_size_[1]))))) {
            // ignore the intermidiate nore, combine the first to the last node.
            ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Combine nodes to remove short segments.");
            path_mod1.erase(path_mod1.begin() + i + 1);
            cont_refine = true;
            break;
          }
        }
      }
    } else {
      return false;
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

    bool e1_admissible = false;
    bool e2_admissible = false;
    if (robot_params_.type == RobotType::kAerialRobot) {
      MapManager::VoxelStatus vs1 = map_manager_->getPathStatus(
          p0_mod, p1_parallel, robot_box_size_, true);
      if (vs1 == MapManager::VoxelStatus::kFree) e1_admissible = true;
      MapManager::VoxelStatus vs2 =
          map_manager_->getPathStatus(p0_mod, p1, robot_box_size_, true);
      if (vs2 == MapManager::VoxelStatus::kFree) e2_admissible = true;
    } else if (robot_params_.type == RobotType::kGroundRobot) {
      std::vector<Eigen::Vector3d> pr1, pr2;
      ProjectedEdgeStatus e_pr1, e_pr2;
      e_pr1 = getProjectedEdgeStatus(p0_mod, p1_parallel, robot_box_size_,
                                     false, pr1, false);
      if (ProjectedEdgeStatus::kAdmissible == e_pr1 ||
          ProjectedEdgeStatus::kSteep == e_pr1)
        e1_admissible = true;
      e_pr2 = getProjectedEdgeStatus(p0_mod, p1, robot_box_size_, false, pr2,
                                     false);
      if (ProjectedEdgeStatus::kAdmissible == e_pr2 ||
          ProjectedEdgeStatus::kSteep == e_pr2)
        e2_admissible = true;
    }
    if (e1_admissible) {
      p1_target = p1_parallel;
    } else if (e2_admissible) {
      p1_target = p1;
    } else {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Segment not free");
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
    Eigen::Vector3d safety_extension;
    safety_extension = robot_params_.safety_extension;
    if (relaxed)
      safety_extension(1) *= planning_params_.relaxed_corridor_multiplier;
    Eigen::Vector3d local_bbx(2 * (radius + safety_extension[0]),
                              2 * safety_extension[1], 2 * safety_extension[2]);
    std::vector<Eigen::Vector3d> occupied_voxels;
    std::vector<Eigen::Vector3d> free_voxels;
    map_manager_->extractLocalMapAlongAxis(p_center, p_dir, local_bbx,
                                           occupied_voxels, free_voxels);

    pcl::PointCloud<pcl::PointXYZ>* obstacle_pcl(
        new pcl::PointCloud<pcl::PointXYZ>());
    for (auto& v : occupied_voxels) {
      obstacle_pcl->push_back(pcl::PointXYZ(v.x(), v.y(), v.z()));
    }

    bool modification_successful =
        (modifyPath(obstacle_pcl, p0_mod, p1_target, p1_mod));
    if (seg_free && modification_successful) {
      bool e1_admissible = false;
      bool e2_admissible = false;
      if (robot_params_.type == RobotType::kAerialRobot) {
        MapManager::VoxelStatus vs1 =
            map_manager_->getPathStatus(p0_mod, p1_mod, robot_box_size_, true);
        if (vs1 == MapManager::VoxelStatus::kFree) e1_admissible = true;
        MapManager::VoxelStatus vs2 =
            map_manager_->getPathStatus(p1_mod, p2, robot_box_size_, true);
        if (vs2 == MapManager::VoxelStatus::kFree) e2_admissible = true;
      } else if (robot_params_.type == RobotType::kGroundRobot) {
        std::vector<Eigen::Vector3d> pr1, pr2;
        ProjectedEdgeStatus es1, es2;
        es1 = getProjectedEdgeStatus(p0_mod, p1_mod, robot_box_size_, false,
                                     pr1, false);
        if (!(ProjectedEdgeStatus::kAdmissible != es1 &&
              (!relaxed || ProjectedEdgeStatus::kSteep != es1)))
          e1_admissible = true;
        es2 = getProjectedEdgeStatus(p1_mod, p2, robot_box_size_, false, pr2,
                                     false);
        if (!(ProjectedEdgeStatus::kAdmissible != es2 &&
              (!relaxed || ProjectedEdgeStatus::kSteep != es2)))
          e2_admissible = true;
      }

      if (!(e1_admissible) ||
          (planning_params_.geofence_checking_enable &&
           (GeofenceManager::CoordinateStatus::kViolated ==
            geofence_manager_->getPathStatus(
                Eigen::Vector2d(p0_mod[0], p0_mod[1]),
                Eigen::Vector2d(p1_mod[0], p1_mod[1]),
                Eigen::Vector2d(robot_box_size_[0], robot_box_size_[1])))) ||
          (do_check_p2 && (!(e2_admissible) ||
                           (planning_params_.geofence_checking_enable &&
                            (GeofenceManager::CoordinateStatus::kViolated ==
                             geofence_manager_->getPathStatus(
                                 Eigen::Vector2d(p1_mod[0], p1_mod[1]),
                                 Eigen::Vector2d(p2[0], p2[1]),
                                 Eigen::Vector2d(robot_box_size_[0],
                                                 robot_box_size_[1]))))))) {
        p1_mod = p1;
        mod_success = false;
        ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Newly modified path is not collision-free.");
        // break; // break to save time @recheck
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
  path_mod[0].orientation.x = path_orig_interp[0].orientation.x;
  path_mod[0].orientation.y = path_orig_interp[0].orientation.y;
  path_mod[0].orientation.z = path_orig_interp[0].orientation.z;
  path_mod[0].orientation.w = path_orig_interp[0].orientation.w;
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

void Rrg::generateGridSamples(std::vector<int> &viewpoint_ids) {
  double xy_spacing = 1.0; // 2.0
  double z_spacing = 2.0; // 3.2;
  double thr_dist = 1.5;
  double target_viewing_dist = 2.5;
  std::vector<std::pair<Eigen::Vector3d, double>> selected_points;
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  Eigen::Vector3d max(-9999, -99999, -9999), min(9999, 9999, 9999);
  // for(double x = local_space_params_.min_val.x(); x <= local_space_params_.max_val.x(); x+=planning_params_.inspection_xy_spacing) {
  //   for(double y = local_space_params_.min_val.y(); y <= local_space_params_.max_val.y(); y+=planning_params_.inspection_xy_spacing) {
  //     for(double z = local_space_params_.min_val.z(); z <= local_space_params_.max_val.z(); z+=planning_params_.inspection_z_spacing) {
  //       Eigen::Vector3d sampled_point(x,y,z);
  //       sampled_point += current_state_.head(3);
  for(double x = inspection_bound_.min_val.x(); x <= inspection_bound_.max_val.x(); x+=planning_params_.inspection_xy_spacing) {
    for(double y = inspection_bound_.min_val.y(); y <= inspection_bound_.max_val.y(); y+=planning_params_.inspection_xy_spacing) {
      for(double z = inspection_bound_.min_val.z() + planning_params_.inspection_z_spacing; z <= inspection_bound_.max_val.z() - planning_params_.inspection_z_spacing; z+=planning_params_.inspection_z_spacing) {
        Eigen::Vector3d sampled_point(x,y,z);
        // sampled_point += current_state_.head(3);
        double dist = map_manager_->getPointDistance(sampled_point);
        ///
        // StateVec new_state(x,y,z,0.0);
        // GbplannerVertex* new_v = new GbplannerVertex(local_graph_->generateVertexID(), new_state);
        // local_graph_->addVertex(new_v);
        ///
        if(dist >= planning_params_.inspection_thr_esdf_dist - planning_params_.inspection_xy_spacing && dist <= planning_params_.inspection_thr_esdf_dist + planning_params_.inspection_xy_spacing) {
        // if(dist >= planning_params_.inspection_thr_esdf_dist && dist <= planning_params_.inspection_thr_esdf_dist + map_manager_->getResolution()) {
          // if(inspection_bound_.isInsideSpace(sampled_point)) {
          // }
          selected_points.push_back(std::make_pair(sampled_point, dist));
          // centroid += sampled_point;
          max = max.cwiseMax(sampled_point);
          min = min.cwiseMin(sampled_point);
          // StateVec new_state(x,y,z,0.0);
          // Eigen::Vector3d dir = sampled_point - current_state_.head(3);
          // dir.z() = 0.0;
          // sampled_point -= dir.normalized() * (target_viewing_dist - dist);
          // new_state.head(3) = sampled_point;
          // if(map_manager_->getBoxStatus(new_state.head(3), robot_box_size_, true) == MapManager::VoxelStatus::kFree) {
          //   GbplannerVertex* new_v = new GbplannerVertex(local_graph_->generateVertexID(), new_state);
          //   local_graph_->addVertex(new_v);
          // }
        }
      }
    }
  }

  // centroid /= selected_points.size();
  centroid = (max + min) / 2.0;
  // ROS_WARN("Centroid: %f, %f, %f", centroid.x(), centroid.y(), centroid.z());

  viewpoint_ids.clear();
  double edge_len_max_og = planning_params_.edge_length_max;
  double nearest_range_max_og = planning_params_.nearest_range_max;
  planning_params_.edge_length_max *= 3.0;
  planning_params_.nearest_range_max *= 3.0;
  planning_params_.nearest_range *= 3.0;
  for(auto sample : selected_points) {
    StateVec new_state;
    Eigen::Vector3d dir = sample.first - centroid;
    dir.z() = 0.0;
    sample.first -= dir.normalized() * (planning_params_.inspection_target_viewing_range - sample.second);
    new_state.head(3) = sample.first;
    new_state(3) = std::atan2(dir.y(), dir.x());
    bool success = false;
    if(map_manager_->getBoxStatus(new_state.head(3), robot_box_size_, true) == MapManager::VoxelStatus::kFree) {
      success = true;
    }
    else {
      Eigen::Vector3d dir_normed = dir.normalized();
      for(double dr=map_manager_->getResolution(); dr<(planning_params_.inspection_target_viewing_range - sample.second); dr+=map_manager_->getResolution()) {
        new_state.head(3) += dir_normed * dr;
        if(map_manager_->getBoxStatus(new_state.head(3), robot_box_size_, true) == MapManager::VoxelStatus::kFree) {
          success = true;
          sample.first = new_state.head(3);
          break;
        }
      }
    }
    if(success) {
      if(global_space_params_.isInsideSpace(sample.first)) {
        // GbplannerVertex* new_v = new GbplannerVertex(-1, new_state);
        double edge_len_min_og = planning_params_.edge_length_min;
        planning_params_.edge_length_min = -0.01;
        GbplannerVertex new_v(-1, new_state);
        ExpandGraphReport rep;
        expandGraph(local_graph_, new_v, rep);
        if(rep.status == ExpandGraphStatus::kSuccess) {
          viewpoint_ids.push_back(rep.vertex_added->id);
          if(planning_params_.use_flipped_yaw) {
            StateVec fliped_state;
            fliped_state = new_state;
            fliped_state[3] = M_PI + fliped_state[3];
            truncateYaw(fliped_state[3]);
            ExpandGraphReport fliped_rep;
            GbplannerVertex fliped_v(-1, fliped_state);
            expandGraph(local_graph_, fliped_v, fliped_rep);
            if(fliped_rep.status == ExpandGraphStatus::kSuccess) {
              viewpoint_ids.push_back(fliped_rep.vertex_added->id);
            }
          }
          
          // GbplannerVertex* new_v = new GbplannerVertex(local_graph_->generateVertexID(), new_state);
          // local_graph_->addVertex(new_v);
        }
        planning_params_.edge_length_min = edge_len_min_og;
      }
    }
  }
  planning_params_.edge_length_max = edge_len_max_og;
  planning_params_.nearest_range_max = nearest_range_max_og;
  planning_params_.nearest_range = nearest_range_max_og;

  // ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Number of ")
  // std::cout << "Number of vertices: " << local_graph_->getNumVertices();
}

std::vector<int> Rrg::getLongsInspectionViewpoints()
{
  // std::vector<geometry_msgs::Pose> inspection_path;

  // std::vector<geometry_msgs::Pose> long_viewpoints;
  // std::vector<StateVec> long_viewpoints_vec;

  // Eigen::Vector3d min_bound(999, 999, 999), max_bound(-999, -999, -999);

  // for(Longitudinal l : current_compartment_longs_)
  // {
  //   StateVec v_ep1, v_ep2;
  //   Eigen::Vector3d v_r = current_state_.head(3) - l.center;
  //   double yaw_dir = std::atan2(l.direction.y(), l.direction.x());
  //   Eigen::Vector3d insp_pt_1, insp_pt_2;
  //   if((v_r.cross(l.direction)).z() >= 0)
  //   {
  //     v_ep1(3) = yaw_dir + M_PI/2.0;
  //     v_ep2(3) = yaw_dir + M_PI/2.0;
  //   }
  //   else
  //   {
  //     v_ep1(3) = yaw_dir - M_PI/2.0;
  //     v_ep2(3) = yaw_dir - M_PI/2.0;
  //   }

  //   insp_pt_1 = l.end_point1 + l.direction * planning_params_.inspection_pt_dist;
  //   insp_pt_2 = l.end_point2 - l.direction * planning_params_.inspection_pt_dist;

  //   v_ep1.head(3) = l.end_point1 + (l.direction - Eigen::Vector3d(std::cos(v_ep1(3)), std::sin(v_ep1(3)), 0.0)) 
  //                   * planning_params_.inspection_pt_dist;
  //   v_ep2.head(3) = l.end_point2 - (l.direction + Eigen::Vector3d(std::cos(v_ep2(3)), std::sin(v_ep2(3)), 0.0)) 
  //                   * planning_params_.inspection_pt_dist;

  //   min_bound = min_bound.cwiseMin(v_ep1.head(3));
  //   min_bound = min_bound.cwiseMin(v_ep2.head(3));
  //   max_bound = max_bound.cwiseMax(v_ep1.head(3));
  //   max_bound = max_bound.cwiseMax(v_ep2.head(3));
    
  //   geometry_msgs::Pose ep1_pose, ep2_pose;

  //   bool ip1_seen = true;
  //   bool ip2_seen = true;
  //   for(double dx = -0.2; dx <= 0.2; dx+=0.2)
  //   {
  //     if(!map_manager_->isPointSeen(insp_pt_1 + dx*l.direction))
  //     {
  //       ip1_seen = false;
  //     }
  //     if(!map_manager_->isPointSeen(insp_pt_2 + dx*l.direction))
  //     {
  //       ip2_seen = false;
  //     }
  //   }
    
  //   if(!ip1_seen)
  //   {
  //     long_viewpoints_vec.push_back(v_ep1);
  //     convert(v_ep1, ep1_pose);
  //     long_viewpoints.push_back(ep1_pose);
  //   }
  //   if(!ip2_seen)
  //   {
  //     long_viewpoints_vec.push_back(v_ep2);
  //     convert(v_ep2, ep2_pose);
  //     long_viewpoints.push_back(ep2_pose);
  //   }
    
  // }

  // // visualization_->visualizeViewpoints(long_viewpoints);

  // min_bound = min_bound - 1.5*Eigen::Vector3d::Ones();
  // max_bound = max_bound + 1.5*Eigen::Vector3d::Ones();
  
  // // BoundedSpaceParams inspection_bound, og_bound;
  // // inspection_bound.setBound(min_bound, max_bound);
  // // og_bound = global_space_params_;
  // // setExplorationAndInspectionBounds(inspection_bound, inspection_bound);

  // bool og_param = planning_params_.use_current_state;
  // planning_params_.use_current_state = true;
  // int local_planner_graph_vertices_og = planning_params_.num_vertices_max;
  // planning_params_.num_vertices_max = planning_params_.inspection_graph_vertices;
  // planning_num_vertices_max_ = planning_params_.inspection_graph_vertices;
  // reset();
  // planning_params_.use_current_state = og_param;

  // Timer graph_timer;
  
  // GraphStatus status = buildGraph();
  // // if(status != GraphStatus::OK)
  // // {
  // //   std::vector<geometry_msgs::Pose> empty_path;
  // //   return empty_path;
  // // }
  // ROS_WARN_COND(global_verbosity >= Verbosity::INFO, "Num of vertices in conneting graph: %d", local_graph_->getNumVertices());

  // planning_params_.num_vertices_max = local_planner_graph_vertices_og;
  // planning_num_vertices_max_ = planning_params_.num_vertices_max;

  // // Add viewpoints
  // std::vector<int> viewpoint_ids;
  // for(int i=0; i<long_viewpoints.size(); ++i)
  // {
  //   MapManager::VoxelStatus vs = map_manager_->getBoxStatus(long_viewpoints_vec[i].head(3), robot_box_size_, true);
  //   // ROS_INFO("Viewpoint status: %d", vs);
  //   GbplannerVertex new_vertex(-1, long_viewpoints_vec[i]);
  //   ExpandGraphReport rep;
  //   expandGraph(local_graph_, new_vertex, rep, true);
  //   if(rep.status == ExpandGraphStatus::kSuccess)
  //   {
  //     viewpoint_ids.push_back(rep.vertex_added->id);
  //   }
  // }

  // ROS_WARN_COND(global_verbosity >= Verbosity::INFO, "Num of vertices in FULL graph: %d", local_graph_->getNumVertices());

  // visualization_->visualizeGraph(local_graph_);

  // return viewpoint_ids;

  return getLongsInspectionViewpoints(current_compartment_longs_);
}


std::vector<int> Rrg::getLongsInspectionViewpoints(std::vector<Longitudinal> longs, bool all)
{
  std::vector<geometry_msgs::Pose> inspection_path;

  std::vector<geometry_msgs::Pose> long_viewpoints;
  std::vector<StateVec> long_viewpoints_vec;

  // Eigen::Vector3d min_bound(999, 999, 999), max_bound(-999, -999, -999);

  // for(Longitudinal l : longs)
  // {
  //   StateVec v_ep1, v_ep2;
  //   Eigen::Vector3d v_r = current_state_.head(3) - l.center;
  //   double yaw_dir = std::atan2(l.direction.y(), l.direction.x());
  //   Eigen::Vector3d insp_pt_1, insp_pt_2;
  //   if((v_r.cross(l.direction)).z() >= 0)
  //   {
  //     v_ep1(3) = yaw_dir + M_PI/2.0;
  //     v_ep2(3) = yaw_dir + M_PI/2.0;
  //   }
  //   else
  //   {
  //     v_ep1(3) = yaw_dir - M_PI/2.0;
  //     v_ep2(3) = yaw_dir - M_PI/2.0;
  //   }

  //   insp_pt_1 = l.end_point1 + l.direction * planning_params_.inspection_pt_dist;
  //   insp_pt_2 = l.end_point2 - l.direction * planning_params_.inspection_pt_dist;

  //   // v_ep1.head(3) = l.end_point1 + (l.direction - Eigen::Vector3d(std::cos(v_ep1(3)), std::sin(v_ep1(3)), 0.0)) 
  //   //                 * planning_params_.inspection_pt_dist;
  //   // v_ep2.head(3) = l.end_point2 - (l.direction + Eigen::Vector3d(std::cos(v_ep2(3)), std::sin(v_ep2(3)), 0.0)) 
  //   //                 * planning_params_.inspection_pt_dist;

  //   v_ep1.head(3) = insp_pt_1 - Eigen::Vector3d(std::cos(v_ep1(3)), std::sin(v_ep1(3)), 0.0) * planning_params_.inspection_target_viewing_range;
  //   v_ep2.head(3) = insp_pt_2 - Eigen::Vector3d(std::cos(v_ep2(3)), std::sin(v_ep2(3)), 0.0) * planning_params_.inspection_target_viewing_range;
  //   // min_bound = min_bound.cwiseMin(v_ep1.head(3));
  //   // min_bound = min_bound.cwiseMin(v_ep2.head(3));
  //   // max_bound = max_bound.cwiseMax(v_ep1.head(3));
  //   // max_bound = max_bound.cwiseMax(v_ep2.head(3));
    
  //   geometry_msgs::Pose ep1_pose, ep2_pose;
    
  //   bool ip1_seen = true;
  //   bool ip2_seen = true;
  //   if(!all)
  //   {
  //     for(double dx = -0.4; dx <= 0.2; dx+=0.4)
  //     {
  //       if(!map_manager_->isPointSeen(insp_pt_1 + dx*l.direction))
  //       {
  //         ip1_seen = false;
  //       }
  //       if(!map_manager_->isPointSeen(insp_pt_2 + dx*l.direction))
  //       {
  //         ip2_seen = false;
  //       }
  //     }
  //   }
    
  //   if(!ip1_seen)
  //   {
  //     long_viewpoints_vec.push_back(v_ep1);
  //     convert(v_ep1, ep1_pose);
  //     long_viewpoints.push_back(ep1_pose);
  //   }
  //   if(!ip2_seen)
  //   {
  //     long_viewpoints_vec.push_back(v_ep2);
  //     convert(v_ep2, ep2_pose);
  //     long_viewpoints.push_back(ep2_pose);
  //   }
    
  // }

  std::cout << "Calculating viewpoints for " << longs.size() << " longs" << std::endl;
  long_viewpoints_vec = getLongsInspectionViewpointsOnly(longs, all);
  std::cout << "Num viewpoints: " << long_viewpoints_vec.size() << std::endl;
  for(auto vp : long_viewpoints_vec)
  {
    geometry_msgs::Pose vp_pose;
    convert(vp, vp_pose);
    long_viewpoints.push_back(vp_pose);
  }
  visualization_->visualizeViewpoints(long_viewpoints);

  // visualization_->visualizeViewpoints(long_viewpoints);

  // min_bound = min_bound - 1.5*Eigen::Vector3d::Ones();
  // max_bound = max_bound + 1.5*Eigen::Vector3d::Ones();
  
  // BoundedSpaceParams inspection_bound, og_bound;
  // inspection_bound.setBound(min_bound, max_bound);
  // og_bound = global_space_params_;
  // setExplorationAndInspectionBounds(inspection_bound, inspection_bound);

  bool og_param = planning_params_.use_current_state;
  // planning_params_.use_current_state = true;
  int local_planner_graph_vertices_og = planning_params_.num_vertices_max;
  planning_params_.num_vertices_max = planning_params_.inspection_graph_vertices;
  planning_num_vertices_max_ = planning_params_.inspection_graph_vertices;
  reset();
  planning_params_.use_current_state = og_param;

  Timer graph_timer;
  
  if(planning_params_.unknown_as_free)
    unknown_as_free_ = true;
  GraphStatus status = buildGraph();

  std::cout << "Graph timer " << graph_timer.endTimer() << std::endl;
  // if(status != GraphStatus::OK)
  // {
  //   std::vector<geometry_msgs::Pose> empty_path;
  //   return empty_path;
  // }
  ROS_WARN_COND(global_verbosity >= Verbosity::INFO, "Num of vertices in conneting graph: %d", local_graph_->getNumVertices());

  planning_params_.num_vertices_max = local_planner_graph_vertices_og;
  planning_num_vertices_max_ = planning_params_.num_vertices_max;

  // Add viewpoints
  std::vector<int> viewpoint_ids;
  for(int i=0; i<long_viewpoints.size(); ++i)
  {
    MapManager::VoxelStatus vs = map_manager_->getBoxStatus(long_viewpoints_vec[i].head(3), robot_box_size_, true);
    // ROS_INFO("Viewpoint status: %d", vs);
    GbplannerVertex new_vertex(-1, long_viewpoints_vec[i]);
    ExpandGraphReport rep;
    expandGraph(local_graph_, new_vertex, rep, true, true);
    if(rep.status == ExpandGraphStatus::kSuccess)
    {
      viewpoint_ids.push_back(rep.vertex_added->id);
    }
  }

  unknown_as_free_ = false;

  std::cout << "Num viewpoints added to graph: " << viewpoint_ids.size() << std::endl;

  ROS_WARN_COND(global_verbosity >= Verbosity::INFO, "Num of vertices in FULL graph: %d", local_graph_->getNumVertices());

  visualization_->visualizeGraph(local_graph_);

  return viewpoint_ids;
}

bool Rrg::isSeen(StateVec viewpoint, Longitudinal l)
{
  Eigen::Vector3d insp_pt;
  if((viewpoint.head(3) - l.end_point1).norm() < (viewpoint.head(3) - l.end_point2).norm())
  {
    insp_pt = l.end_point1 + l.direction * planning_params_.inspection_pt_dist;
  }
  else
  {
    insp_pt = l.end_point2 - l.direction * planning_params_.inspection_pt_dist;
  }

  bool ip_seen = true;
  for(double dx = -semantic_annotation_half_width_; dx <= semantic_annotation_half_width_; dx+=map_manager_->getResolution())
  {
    if(!map_manager_->isPointSeen(insp_pt + dx*l.direction))
    {
      ip_seen = false;
    }
  }

  return ip_seen;
}

std::vector<StateVec> Rrg::getLongsInspectionViewpointsOnly(std::vector<Longitudinal> longs, bool all)
{
  std::vector<geometry_msgs::Pose> inspection_path;

  std::vector<geometry_msgs::Pose> long_viewpoints;
  std::vector<StateVec> long_viewpoints_vec;

  Eigen::Vector3d min_bound(999, 999, 999), max_bound(-999, -999, -999);

  for(Longitudinal l : longs)
  {
    StateVec v_ep1, v_ep2, v_c;
    Eigen::Vector3d v_r = current_state_.head(3) - l.center;
    double yaw_dir = std::atan2(l.direction.y(), l.direction.x());
    Eigen::Vector3d insp_pt_1, insp_pt_2;
    // double ang_offset = 15.0 * M_PI / 180.0;
    double ang_offset = config_->prediction_params.viewpoint_ang_offset;
    if((v_r.cross(l.direction)).z() >= 0)
    {
      v_c(3) = yaw_dir + M_PI/2.0;
      v_ep1(3) = yaw_dir + (M_PI/2.0 - ang_offset);
      v_ep2(3) = yaw_dir + (M_PI/2.0 + ang_offset);
    }
    else
    {
      v_c(3) = yaw_dir - M_PI/2.0;
      v_ep1(3) = yaw_dir - (M_PI/2.0 - ang_offset);
      v_ep2(3) = yaw_dir - (M_PI/2.0 + ang_offset);
    }

    truncateAngle(v_c(3));
    truncateAngle(v_ep1(3));
    truncateAngle(v_ep2(3));

    insp_pt_1 = l.end_point1 + l.direction * planning_params_.inspection_pt_dist;
    insp_pt_2 = l.end_point2 - l.direction * planning_params_.inspection_pt_dist;

    v_ep1.head(3) = insp_pt_1 - Eigen::Vector3d(std::cos(v_ep1(3)), std::sin(v_ep1(3)), 0.0) * planning_params_.inspection_target_viewing_range;
    v_ep2.head(3) = insp_pt_2 - Eigen::Vector3d(std::cos(v_ep2(3)), std::sin(v_ep2(3)), 0.0) * planning_params_.inspection_target_viewing_range;
    v_c.head(3) = l.center - Eigen::Vector3d(std::cos(v_c(3)), std::sin(v_c(3)), 0.0) * planning_params_.inspection_target_viewing_range;
    
    geometry_msgs::Pose ep1_pose, ep2_pose, c_pose;
    if(!all)
    {
      bool ip1_seen = true;
      bool ip2_seen = true;
      bool c_seen = true;
      // for(double dx = -2.25; dx <= 2.25; dx+=map_manager_->getResolution())
      for(double dx = -semantic_annotation_half_width_; dx <= semantic_annotation_half_width_; dx+=map_manager_->getResolution())
      {
        if(!map_manager_->isPointSeen(l.center + dx*l.direction))
        {
          c_seen = false;
        }
        if(!map_manager_->isPointSeen(insp_pt_1 + dx*l.direction))
        {
          ip1_seen = false;
        }
        if(!map_manager_->isPointSeen(insp_pt_2 + dx*l.direction))
        {
          ip2_seen = false;
        }
      }
      
      if(!ip1_seen)
      {
        long_viewpoints_vec.push_back(v_ep1);
        convert(v_ep1, ep1_pose);
        long_viewpoints.push_back(ep1_pose);
      }
      if(!ip2_seen)
      {
        long_viewpoints_vec.push_back(v_ep2);
        convert(v_ep2, ep2_pose);
        long_viewpoints.push_back(ep2_pose);
      }
      if(!c_seen && config_->prediction_params.view_center)
      {
        long_viewpoints_vec.push_back(v_c);
        convert(v_c, c_pose);
        long_viewpoints.push_back(c_pose);
      }
    }
    else
    {
      // MapManager::VoxelStatus vs1 = map_manager_->getBoxStatus(v_ep1.head(3), robot_box_size_, true);
      // if(vs1 == MapManager::VoxelStatus::kFree)
      // {
      //   long_viewpoints_vec.push_back(v_ep1);
      //   convert(v_ep1, ep1_pose);
      //   long_viewpoints.push_back(ep1_pose);
      // }
      // MapManager::VoxelStatus vs2 = map_manager_->getBoxStatus(v_ep2.head(3), robot_box_size_, true);
      // if(vs2 == MapManager::VoxelStatus::kFree)
      // {
      //   long_viewpoints_vec.push_back(v_ep2);
      //   convert(v_ep2, ep2_pose);
      //   long_viewpoints.push_back(ep2_pose);
      // }

      long_viewpoints_vec.push_back(v_ep1);
      convert(v_ep1, ep1_pose);
      long_viewpoints.push_back(ep1_pose);

      long_viewpoints_vec.push_back(v_ep2);
      convert(v_ep2, ep2_pose);
      long_viewpoints.push_back(ep2_pose);

      if(config_->prediction_params.view_center)
      {
        long_viewpoints_vec.push_back(v_c);
        convert(v_c, c_pose);
        long_viewpoints.push_back(c_pose);
      }

    }
    
  }
  
  return long_viewpoints_vec;
}


void Rrg::insertLongsInspectionViewpoints(std::vector<Longitudinal> longs)
{
  std::vector<geometry_msgs::Pose> inspection_path;

  std::vector<geometry_msgs::Pose> long_viewpoints;
  std::vector<StateVec> long_viewpoints_vec;

  // Eigen::Vector3d min_bound(999, 999, 999), max_bound(-999, -999, -999);

  // for(Longitudinal l : longs)
  // {
  //   StateVec v_ep1, v_ep2;
  //   Eigen::Vector3d v_r = current_state_.head(3) - l.center;
  //   double yaw_dir = std::atan2(l.direction.y(), l.direction.x());
  //   Eigen::Vector3d insp_pt_1, insp_pt_2;
  //   if((v_r.cross(l.direction)).z() >= 0)
  //   {
  //     v_ep1(3) = yaw_dir + M_PI/2.0;
  //     v_ep2(3) = yaw_dir + M_PI/2.0;
  //   }
  //   else
  //   {
  //     v_ep1(3) = yaw_dir - M_PI/2.0;
  //     v_ep2(3) = yaw_dir - M_PI/2.0;
  //   }

  //   insp_pt_1 = l.end_point1 + l.direction * planning_params_.inspection_pt_dist;
  //   insp_pt_2 = l.end_point2 - l.direction * planning_params_.inspection_pt_dist;

  //   v_ep1.head(3) = l.end_point1 + (l.direction - Eigen::Vector3d(std::cos(v_ep1(3)), std::sin(v_ep1(3)), 0.0)) 
  //                   * planning_params_.inspection_pt_dist;
  //   v_ep2.head(3) = l.end_point2 - (l.direction + Eigen::Vector3d(std::cos(v_ep2(3)), std::sin(v_ep2(3)), 0.0)) 
  //                   * planning_params_.inspection_pt_dist;

  //   // min_bound = min_bound.cwiseMin(v_ep1.head(3));
  //   // min_bound = min_bound.cwiseMin(v_ep2.head(3));
  //   // max_bound = max_bound.cwiseMax(v_ep1.head(3));
  //   // max_bound = max_bound.cwiseMax(v_ep2.head(3));
    
  //   geometry_msgs::Pose ep1_pose, ep2_pose;
    
  //   bool ip1_seen = true;
  //   bool ip2_seen = true;
  //   for(double dx = -0.2; dx <= 0.2; dx+=0.2)
  //   {
  //     if(!map_manager_->isPointSeen(insp_pt_1 + dx*l.direction))
  //     {
  //       ip1_seen = false;
  //     }
  //     if(!map_manager_->isPointSeen(insp_pt_2 + dx*l.direction))
  //     {
  //       ip2_seen = false;
  //     }
  //   }
    
  //   if(!ip1_seen)
  //   {
  //     long_viewpoints_vec.push_back(v_ep1);
  //     convert(v_ep1, ep1_pose);
  //     long_viewpoints.push_back(ep1_pose);
  //   }
  //   if(!ip2_seen)
  //   {
  //     long_viewpoints_vec.push_back(v_ep2);
  //     convert(v_ep2, ep2_pose);
  //     long_viewpoints.push_back(ep2_pose);
  //   }
    
  // }

  long_viewpoints_vec = getLongsInspectionViewpointsOnly(longs, false);
  for(auto v : long_viewpoints_vec)
  {
    geometry_msgs::Pose vp_pose;
    convert(v, vp_pose);
    long_viewpoints.push_back(vp_pose);
  }

  // visualization_->visualizeViewpoints(long_viewpoints);

  // min_bound = min_bound - 1.5*Eigen::Vector3d::Ones();
  // max_bound = max_bound + 1.5*Eigen::Vector3d::Ones();
  
  // BoundedSpaceParams inspection_bound, og_bound;
  // inspection_bound.setBound(min_bound, max_bound);
  // og_bound = global_space_params_;
  // setExplorationAndInspectionBounds(inspection_bound, inspection_bound);

  // bool og_param = planning_params_.use_current_state;
  // planning_params_.use_current_state = true;
  // int local_planner_graph_vertices_og = planning_params_.num_vertices_max;
  // planning_params_.num_vertices_max = planning_params_.inspection_graph_vertices;
  // planning_num_vertices_max_ = planning_params_.inspection_graph_vertices;
  // reset();
  // planning_params_.use_current_state = og_param;

  Timer graph_timer;
  
  // GraphStatus status = buildGraph();
  // if(status != GraphStatus::OK)
  // {
  //   std::vector<geometry_msgs::Pose> empty_path;
  //   return empty_path;
  // }
  ROS_WARN_COND(global_verbosity >= Verbosity::INFO, "Num of vertices in conneting graph: %d", local_graph_->getNumVertices());

  // planning_params_.num_vertices_max = local_planner_graph_vertices_og;
  // planning_num_vertices_max_ = planning_params_.num_vertices_max;

  // Add viewpoints
  std::vector<int> viewpoint_ids;
  for(int i=0; i<long_viewpoints.size(); ++i)
  {
    MapManager::VoxelStatus vs = map_manager_->getBoxStatus(long_viewpoints_vec[i].head(3), robot_box_size_, true);
    // ROS_INFO("Viewpoint status: %d", vs);
    GbplannerVertex new_vertex(-1, long_viewpoints_vec[i]);
    ExpandGraphReport rep;
    expandGraph(local_graph_, new_vertex, rep, true);
    if(rep.status == ExpandGraphStatus::kSuccess)
    {
      viewpoint_ids.push_back(rep.vertex_added->id);
    }
  }

  ROS_WARN_COND(global_verbosity >= Verbosity::INFO, "Num of vertices in FULL graph: %d", local_graph_->getNumVertices());

  visualization_->visualizeGraph(local_graph_);
}

std::vector<StateVec> Rrg::getBlindTSPOrder(std::vector<StateVec> viewpoints)
{
  std::vector<int> temp_cost_mat_row(viewpoints.size(), 9999999);
	std::vector<std::vector<int>> cost_matrix(viewpoints.size(), temp_cost_mat_row);
	for (int i = 0; i < viewpoints.size(); ++i)
	{
		for (int j = 0; j < viewpoints.size(); ++j)
		{
			if (i == j || j == 0)  // Same vertex or connecting to current state
			{
				cost_matrix[i][j] = 0;
			}
			else
			{
        if(planning_params_.use_flipped_yaw) {
          double d_yaw = viewpoints[i][3] - viewpoints[j][3];
          truncateYaw(d_yaw);
          cost_matrix[i][j] = std::max((viewpoints[i].head(3) - viewpoints[j].head(3)).norm()/planning_params_.v_max, std::abs(d_yaw) / planning_params_.yaw_rate_max);
        }
        else {
          cost_matrix[i][j] = (viewpoints[i].head(3) - viewpoints[j].head(3)).norm();
        }
			}
		}
	}

  std::vector<int> vp_ids;
  for(int i=0; i<viewpoints.size(); ++i) vp_ids.push_back(i);

  std::vector<int> tsp_order = doTSP(vp_ids, cost_matrix);
  std::vector<int> reordered_tsp_sol;
  if(tsp_order[0] == 0) {
    reordered_tsp_sol = tsp_order;
  }
  else if(tsp_order.back() == 0) {
    reordered_tsp_sol.push_back(0);
    reordered_tsp_sol.insert(reordered_tsp_sol.begin()+1, tsp_order.begin(), tsp_order.end()-1);
  }
  else {
    auto it = std::find(tsp_order.begin(), tsp_order.end(), 0);
    if(it == tsp_order.end()) {
      ROS_WARN_COND(global_verbosity >= Verbosity::ERROR, "Root vertex not in TSP order");
    }
    int root_ind = it - tsp_order.begin();
    
    reordered_tsp_sol.push_back(0);
    reordered_tsp_sol.insert(reordered_tsp_sol.begin()+reordered_tsp_sol.size(), it+1, tsp_order.end());
    reordered_tsp_sol.insert(reordered_tsp_sol.begin()+reordered_tsp_sol.size(), tsp_order.begin(), it);
  }
  tsp_order = reordered_tsp_sol;
  if(tsp_order.size() >= 3)
  {
    double d_yaw_f = local_graph_->getVertex(0)->state[3] - local_graph_->getVertex(tsp_order[1])->state[3];
    truncateAngle(d_yaw_f);
    double d_yaw_b = local_graph_->getVertex(0)->state[3] - local_graph_->getVertex(tsp_order.back())->state[3];
    truncateAngle(d_yaw_b);
    double cf, cb;
    {
      double t_trans = (current_state_.head(3) - viewpoints[tsp_order[0]].head(3)).norm()/planning_params_.v_max;
      double t_rot = std::abs(d_yaw_f) / planning_params_.yaw_rate_max;
      if(t_trans > t_rot)
      {
        cf = t_trans;
      }
      else
      {
        cf = t_rot + t_trans;
      }
    }
    {
      double t_trans = (current_state_.head(3) - viewpoints[tsp_order.back()].head(3)).norm()/planning_params_.v_max;
      double t_rot = std::abs(d_yaw_b) / planning_params_.yaw_rate_max;
      if(t_trans > t_rot)
      {
        cb = t_trans;
      }
      else
      {
        cb = t_rot + t_trans;
      }
    }
    // std::cout << "f: " << local_graph_->getVertex(tsp_order[1])->state.transpose() << "cf: " << cf 
    //               << " | b: " << local_graph_->getVertex(tsp_order.back())->state.transpose() << " cb: " << cb << std::endl;
    // double cf = std::max((current_state_.head(3) - viewpoints[tsp_order[0]].head(3)).norm() / planning_params_.v_max,
    //                       std::abs(d_yaw_f) / planning_params_.yaw_rate_max);
    // double cb = std::max((current_state_.head(3) - viewpoints[tsp_order.back()].head(3)).norm() / planning_params_.v_max,
    //                       std::abs(d_yaw_b) / planning_params_.yaw_rate_max);
    if(cf > cb)
    {
      reordered_tsp_sol.clear();
      reordered_tsp_sol.insert(reordered_tsp_sol.begin(), tsp_order.begin()+1, tsp_order.begin()+tsp_order.size());
      std::reverse(reordered_tsp_sol.begin(), reordered_tsp_sol.end());

      reordered_tsp_sol.insert(reordered_tsp_sol.begin(), tsp_order[0]);
      tsp_order = reordered_tsp_sol;
    }
  }

  std::vector<StateVec> ordered_viewpoints;
  for(int i=0; i<tsp_order.size(); ++i)
  {
    ordered_viewpoints.push_back(viewpoints[tsp_order[i]]);
  }

  return ordered_viewpoints;
}

std::vector<StateVec> Rrg::getBlindInspectionOrder(std::vector<Longitudinal> longs)
{
  std::vector<StateVec> viewpoints = getLongsInspectionViewpointsOnly(longs, true);
  viewpoints.insert(viewpoints.begin(), current_state_);

  return getBlindTSPOrder(viewpoints);
}


std::vector<geometry_msgs::Pose> Rrg::getInspectionPath(InspectionStatus &status) {
  auto t1 = std::chrono::high_resolution_clock::now();
  auto t2 = t1;

  std::vector<int> viewpoint_ids;
  Timer gain_timer;
  std::set<size_t> cumulative_unseen_voxels;
  std::vector<std::pair<int, std::set<std::size_t>>> remaining_vertices;
  std::vector<int> all_vertex_ids;
  std::vector<int> selected_vertex_ids;
  std::map<int, ShortestPathsReport> path_rep_map;
  std::vector<geometry_msgs::Pose> ret_path;
  Timer T1;

  if(planning_params_.use_gvi)
  {
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Inspection bounds:");
    if(global_verbosity >= Verbosity::DEBUG) {
      std::cout << "Min val: " << inspection_bound_.min_val.transpose() << std::endl;
      std::cout << "Max val: " << inspection_bound_.max_val.transpose() << std::endl;
    }

    // inspection_bound_.max_val = Eigen::Vector3d(1.0, 1.5, 5.0);
    // inspection_bound_.min_val = Eigen::Vector3d(-3.0, -2.5, 0.0);

    bool og_param = planning_params_.use_current_state;
    planning_params_.use_current_state = true;
    int local_planner_graph_vertices_og = planning_params_.num_vertices_max;
    planning_params_.num_vertices_max = planning_params_.inspection_graph_vertices;
    planning_num_vertices_max_ = planning_params_.inspection_graph_vertices;
    reset();
    planning_params_.use_current_state = og_param;

    Timer graph_timer;
    
    GraphStatus status = buildGraph();
    if(status != GraphStatus::OK)
    {
      std::vector<geometry_msgs::Pose> empty_path;
      return empty_path;
    }
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Num of vertices in conneting graph: %d", local_graph_->getNumVertices());
    
    
    generateGridSamples(viewpoint_ids);  
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Total Num of vertices graph: %d", local_graph_->getNumVertices());

    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Number of Viewpoints: %d", viewpoint_ids.size());
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[Inpsection]: Graph building: %f", graph_timer.endTimer());

    planning_params_.num_vertices_max = local_planner_graph_vertices_og;
    planning_num_vertices_max_ = planning_params_.num_vertices_max;

    local_graph_->findShortestPaths(local_graph_rep_);
    add_frontiers_to_global_graph_ = true;
    
    // Compute gain
    

    // Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    // Eigen::Vector3d max(-9999, -99999, -9999), min(9999, 9999, 9999);
    // for(int i : viewpoint_ids) {
    //   GbplannerVertex* v = local_graph_->getVertex(i);
    //   max = max.cwiseMax(v->state.head(3));
    //   min = min.cwiseMin(v->state.head(3));
    // }
    // centroid = (max + min) / 2.0;
    Eigen::Vector3d centroid = (inspection_bound_.max_val + inspection_bound_.min_val) / 2.0;
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Centroid: %f, %f, %f", centroid.x(), centroid.y(), centroid.z());

    // for(int i=0; i<viewpoint_ids.size(); ++i) {
    for(int i : viewpoint_ids) {
      // all_vertex_ids.push_back(i);
      GbplannerVertex* v = local_graph_->getVertex(i);
      // if(map_manager_->getVoxelDistance(v->state.head(3)) <= 3.0) {
      if(true) {
        int best_gain = 0;
        double best_yaw = 0.0;
        VolumetricGain best_vol_gain;
        
        StateVec s_og = v->state;

        // Eigen::Vector3d dir = v->state.head(3) - centroid;

        computeInspectionGainRayModel(v);

        remaining_vertices.push_back(std::make_pair(i, v->vol_gain.unseen_voxel_hash_keys));
        // ROS_INFO("GbplannerVertex: %d, voxles seen: %d", v->id, v->vol_gain.unseen_voxel_hash_keys.size());
        cumulative_unseen_voxels.merge(v->vol_gain.unseen_voxel_hash_keys);
      }
    }
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[Inpsection]: Gain calculation: %f", gain_timer.endTimer());

    visualization_->visualizeGraph(local_graph_);
    // visualization_->visualizeGraphVertices(local_graph_, all_vertex_ids);

    // std::vector<geometry_msgs::Pose> empty_path;
    // return empty_path;

    Timer sorting_timer;
    int min_coverage = (int)(planning_params_.min_coverage_percentage * cumulative_unseen_voxels.size());
    int current_coverage = 0;  // Number of unseen voxels seen by all vertices selected so far
    std::vector<std::pair<int, std::set<std::size_t>>> selected_vertices;
    std::set<std::size_t> current_visibility;
    while(current_coverage <= min_coverage && !remaining_vertices.empty()) {
      std::sort(remaining_vertices.begin(), remaining_vertices.end(),
              [](const std::pair<int, std::set<std::size_t>> &v1, const std::pair<int, std::set<std::size_t>> &v2)
              {
                return v1.second.size() > v2.second.size();
              });

      // ROS_INFO("Best order: ");
      // for(auto p : remaining_vertices) {
      //   std::cout << p.first << " ";
      // }
      // std::cout << std::endl;
      // Selecting best vertex for now
      auto vert_selected = remaining_vertices[0];
      remaining_vertices.erase(remaining_vertices.begin());
      std::set<std::size_t> vert_visibility = vert_selected.second;
      current_visibility.merge(vert_visibility);
      selected_vertices.push_back(vert_selected);
      current_coverage = current_visibility.size();

      std::set<std::size_t> vert_sel_visibility = vert_selected.second;
      for (int i = 0; i < remaining_vertices.size(); ++i)
      {
        // std::set<std::size_t> vert_sel_visibility = full_vertices;
        std::vector<std::size_t> remaining_visibility;
        // logger_->debug("VP sel visibility: ");
        // for(auto fvr : vert_sel_visibility)
        // {
        // 	std::cout << fvr << std::endl;
        // }
        // ROS_INFO("GbplannerVertex: %d", i);
        int visibility_before = remaining_vertices[i].second.size();
        // ROS_INFO("Before: %d", remaining_vertices[i].second.size());
        std::set<std::size_t> vr_visibility = remaining_vertices[i].second;
        std::set_difference(remaining_vertices[i].second.begin(), remaining_vertices[i].second.end(),
                  vert_sel_visibility.begin(), vert_sel_visibility.end(),
                  std::back_inserter(remaining_visibility));
        
        // logger_->debug("VR visibility before: ");
        // for(auto fvr : remaining_vertices[i].second)
        // {
        // 	std::cout << fvr << std::endl;
        // }
        remaining_vertices[i].second.clear();
        remaining_vertices[i].second.insert(remaining_visibility.begin(), remaining_visibility.end());

        // if(remaining_vertices[i].second.size() < visibility_before) {
        //   ROS_INFO("Before: %d", visibility_before);
        //   ROS_INFO("After: %d", remaining_vertices[i].second.size());
        // }
        // logger_->debug("VR visibility after: ");
        // for(auto fvr : remaining_vertices[i].second)
        // {
        // 	std::cout << fvr << std::endl;
        // }
      }

      if(selected_vertices.size() > planning_params_.max_inspection_vertices) {
        ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Max inspection vertices limit reached");
        break;
      }
    }

    ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Selected vertices: %d, total vertices: %d", selected_vertices.size(), local_graph_->getNumVertices());
    ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Achieved coverage: %d, Max possible coverage: %d, percentage: %f", current_coverage, cumulative_unseen_voxels.size(), (100.0*current_coverage)/cumulative_unseen_voxels.size());

    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[Inpsection]: Sorting: %f", sorting_timer.endTimer());

    viewpoint_ids.push_back(0);  // Adding current node

    if(!selected_vertices.empty()) {      
      T1.reset();
      // for (int i = 0; i < viewpoint_ids.size(); ++i)
      // {
      for(auto p : selected_vertices) {
        ShortestPathsReport rep;
        local_graph_->findShortestPaths(p.first, rep);
        path_rep_map[p.first] = rep;
        selected_vertex_ids.push_back(p.first);
      }
      // for(auto p : selected_vertices) {
      //   selected_vertex_ids.push_back(p.first);
      // }
      {
        ShortestPathsReport rep;
        local_graph_->findShortestPaths(0, rep);
        path_rep_map[0] = rep;
      }
      selected_vertex_ids.push_back(0);
      ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[Inpsection]: TSP: Shortest path calculations: %f", T1.endTimer());
    }
  }
  else
  {
    viewpoint_ids = getLongsInspectionViewpoints();
    if(viewpoint_ids.empty())
    {
      status = InspectionStatus::kNothingToInspect;
      return ret_path;
    }
    selected_vertex_ids = viewpoint_ids;
    selected_vertex_ids.push_back(0);
    for(auto p : selected_vertex_ids) {
      ShortestPathsReport rep;
      local_graph_->findShortestPaths(p, rep);
      path_rep_map[p] = rep;
    }
    
  }


  /* Trial */
  // if(!selected_vertex_ids.empty()) {
  //   ROS_WARN("Viewpoints not empty");
  //   visualization_->visualizeGraphVertices(local_graph_, selected_vertex_ids);

  //   int closest_viewpoint = -1;
  //   std::vector<GbplannerVertex*> nearest_vertices;
  //   StateVec c = current_state_;
  //   // if(local_graph_->getNearestVertices(&current_state_, 5.0, &nearest_vertices))
  //   // {
  //   //   std::sort(nearest_vertices.begin(), nearest_vertices.end(), [&c](const GbplannerVertex* a, const GbplannerVertex* b)
  //   //   {
  //   //     double d_yaw_a = c[3] - a->state[3];
  //   //     double d_yaw_b = c[3] - b->state[3];
  //   //     truncateAngle(d_yaw_a);
  //   //     truncateAngle(d_yaw_b);
  //   //     return std::max((c.head(3) - a->state.head(3)).norm()/1.0, d_yaw_a/0.5) < std::max((c.head(3) - b->state.head(3)).norm()/1.0, d_yaw_b/0.5);
  //   //   });
  //   //   for(auto v : nearest_vertices)
  //   //   {
  //   //     if(v->id == 0) continue;
  //   //     auto it = std::find(selected_vertex_ids.begin(), selected_vertex_ids.end(), v->id);
  //   //     if(it != selected_vertex_ids.end())
  //   //     {
  //   //       closest_viewpoint = v->id;
  //   //       break;
  //   //     }
  //   //   }

  //   // }
  //   double min_cost = 9999.9;
  //   for(int id : selected_vertex_ids)
  //   {
  //     truncateAngle(local_graph_->getVertex(id)->state(3));
  //     double d_yaw = c[3] - local_graph_->getVertex(id)->state(3);
  //     truncateAngle(d_yaw);
  //     double cost = 
  //       std::max(
  //         (c.head(3) - local_graph_->getVertex(id)->state.head(3)).norm()/planning_params_.v_max, 
  //         std::abs(d_yaw)/planning_params_.yaw_rate_max);
  //     if(cost < min_cost && cost > 0.01)
  //     {
  //       min_cost = cost;
  //       closest_viewpoint = id;
  //     }
  //   }

  //   std::vector<geometry_msgs::Pose> current_path;
  //   std::cout << closest_viewpoint << std::endl;
  //   if(closest_viewpoint >= 0)
  //   {
  //     std::vector<int> order;
  //     order.push_back(0);
  //     order.push_back(closest_viewpoint);
	// 	  current_path = connectTSPOrder(order, path_rep_map, local_graph_);
  //   }

  //   status = InspectionStatus::kOk;

  //   return current_path;

  // }
  /*********/
  
  //// TSP /////
  // Connecting the selected viewpoints
  Timer tsp_timer; 
  if(!selected_vertex_ids.empty()) {

    std::vector<std::vector<int>> cost_matrix;
		generateCostMatrix(selected_vertex_ids, cost_matrix, path_rep_map);
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[Inpsection]: TSP: Cost matrix: %f", T1.endTimer());

    visualization_->visualizeGraphVertices(local_graph_, selected_vertex_ids);
    

    std::vector<int> tsp_order = doTSP(selected_vertex_ids, cost_matrix);
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[Inpsection]: TSP: doTSP: %f", T1.endTimer());
    
    // std::cout << "TSP order Before: "; for (int i = 0; i < tsp_order.size(); ++i) std::cout << tsp_order[i] << ", "; std::cout << std::endl;

    std::vector<int> reordered_tsp_sol;
    if(tsp_order[0] == 0) {
      reordered_tsp_sol = tsp_order;
    }
    else if(tsp_order.back() == 0) {
      reordered_tsp_sol.push_back(0);
      reordered_tsp_sol.insert(reordered_tsp_sol.begin()+1, tsp_order.begin(), tsp_order.end()-1);
    }
    else {
      auto it = std::find(tsp_order.begin(), tsp_order.end(), 0);
      if(it == tsp_order.end()) {
        ROS_WARN_COND(global_verbosity >= Verbosity::ERROR, "Root vertex not in TSP order");
      }
      int root_ind = it - tsp_order.begin();
      
      reordered_tsp_sol.push_back(0);
      reordered_tsp_sol.insert(reordered_tsp_sol.begin()+reordered_tsp_sol.size(), it+1, tsp_order.end());
      reordered_tsp_sol.insert(reordered_tsp_sol.begin()+reordered_tsp_sol.size(), tsp_order.begin(), it);
    }
    tsp_order = reordered_tsp_sol;
    // std::cout << "TSP order After: "; for (int i = 0; i < tsp_order.size(); ++i) std::cout << tsp_order[i] << ", "; std::cout << std::endl;
		

    std::vector<geometry_msgs::Pose> current_path;
		current_path = connectTSPOrder(tsp_order, path_rep_map, local_graph_);
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[Inpsection]: TSP: connectTSPOrder: %f", T1.endTimer());

    // geometry_msgs::Pose current_pose;
    // convert(current_state_, current_pose);
    // current_path.push_back(current_pose);

    visualization_->visualizeRefPath(current_path);

    // for(auto p : selected_vertices) {
    //   GbplannerVertex* v = local_graph_->getVertex(p.first);
    //   for(auto sensor : camera_annotation_params_.sensor_list) {
    //     std::vector<Eigen::Vector3d> multiray_endpoints;
    //     camera_annotation_params_.sensor[sensor].getFrustumEndpoints(v->state, multiray_endpoints);
    //     Eigen::Vector3d current_pos = v->state.head(3);
    //     map_manager_->annotateCameraVoxels(current_pos, multiray_endpoints);
    //   }
    // }
    ret_path = current_path;
  }
  else {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "No vertex selected");
  }
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[Inpsection]: TSP: %f", tsp_timer.endTimer());

  t2 = std::chrono::high_resolution_clock::now();

  ROS_WARN_COND(global_verbosity >= Verbosity::INFO, "Inspection path calculation time: %f", std::chrono::duration<double, std::milli>(t2 - t1).count());

  status = InspectionStatus::kOk;

  return ret_path;
}

std::vector<geometry_msgs::Pose> Rrg::getInspectionPath(std::vector<Longitudinal> longs, InspectionStatus &status) {
  auto t1 = std::chrono::high_resolution_clock::now();
  auto t2 = t1;

  std::vector<int> viewpoint_ids;
  Timer gain_timer;
  std::set<size_t> cumulative_unseen_voxels;
  std::vector<std::pair<int, std::set<std::size_t>>> remaining_vertices;
  std::vector<int> all_vertex_ids;
  std::vector<int> selected_vertex_ids;
  std::map<int, ShortestPathsReport> path_rep_map;
  std::vector<geometry_msgs::Pose> ret_path;
  Timer T1;

  viewpoint_ids = getLongsInspectionViewpoints(longs);
  if(viewpoint_ids.empty())
  {
    status = InspectionStatus::kNothingToInspect;
    return ret_path;
  }
  selected_vertex_ids = viewpoint_ids;
  // selected_vertex_ids.push_back(0);
  for(auto p : selected_vertex_ids) {
    ShortestPathsReport rep;
    local_graph_->findShortestPaths(p, rep);
    path_rep_map[p] = rep;
  }

  ShortestPathsReport rep;
  local_graph_->findShortestPaths(0, rep);
  path_rep_map[0] = rep;

  std::cout << "Added rep for 0 separately" << std::endl;


  /* Trial */
  // if(!selected_vertex_ids.empty()) {
  //   ROS_WARN("Viewpoints not empty");
  //   visualization_->visualizeGraphVertices(local_graph_, selected_vertex_ids);

  //   int closest_viewpoint = -1;
  //   std::vector<GbplannerVertex*> nearest_vertices;
  //   StateVec c = current_state_;
  //   // if(local_graph_->getNearestVertices(&current_state_, 5.0, &nearest_vertices))
  //   // {
  //   //   std::sort(nearest_vertices.begin(), nearest_vertices.end(), [&c](const GbplannerVertex* a, const GbplannerVertex* b)
  //   //   {
  //   //     return (c.head(3) - a->state.head(3)).norm() < (c.head(3) - b->state.head(3)).norm();
  //   //   });
  //   //   for(auto v : nearest_vertices)
  //   //   {
  //   //     if(v->id == 0 || (current_state_.head(3) - v->state.head(3)).norm() <= 0.1) continue;
  //   //     auto it = std::find(selected_vertex_ids.begin(), selected_vertex_ids.end(), v->id);
  //   //     if(it != selected_vertex_ids.end())
  //   //     {
  //   //       closest_viewpoint = v->id;
  //   //       break;
  //   //     }
  //   //   }
  //   // }

  //   double min_cost = 9999.9;
  //   for(int id : selected_vertex_ids)
  //   {
  //     truncateAngle(local_graph_->getVertex(id)->state(3));
  //     double d_yaw = c[3] - local_graph_->getVertex(id)->state(3);
  //     truncateAngle(d_yaw);
  //     double cost = 
  //       std::max(
  //         (c.head(3) - local_graph_->getVertex(id)->state.head(3)).norm()/planning_params_.v_max, 
  //         std::abs(d_yaw)/planning_params_.yaw_rate_max);
  //     if(cost < min_cost)
  //     {
  //       min_cost = cost;
  //       closest_viewpoint = id;
  //     }
  //   }

  //   std::vector<geometry_msgs::Pose> current_path;
  //   std::cout << closest_viewpoint << std::endl;
  //   if(closest_viewpoint >= 0)
  //   {
  //     std::vector<int> order;
  //     order.push_back(0);
  //     order.push_back(closest_viewpoint);
	// 	  current_path = connectTSPOrder(order, path_rep_map, local_graph_);
  //   }

  //   t2 = std::chrono::high_resolution_clock::now();

  //   ROS_WARN_COND(global_verbosity >= Verbosity::INFO, "Inspection path calculation time: %f", std::chrono::duration<double, std::milli>(t2 - t1).count());

  //   status = InspectionStatus::kOk;

  //   return current_path;

  // }
  /*********/
  
  //// TSP /////
  // Connecting the selected viewpoints
  Timer tsp_timer; 
  // if(!selected_vertex_ids.empty()) 
  // {
  //   if(selected_vertex_ids.size() == 1)
  //   {
  //     std::vector<int> tsp_order;
  //     tsp_order.push_back(0);
  //     tsp_order.push_back(selected_vertex_ids[0]);
  //     std::vector<geometry_msgs::Pose> current_path;
	// 	  current_path = connectTSPOrder(tsp_order, path_rep_map, local_graph_);
  //     visualization_->visualizeRefPath(current_path);
  //     ret_path = current_path;
  //   }
  //   else
  //   {
  //     Eigen::Vector3d mean_pos(0.0, 0.0, 0.0);
  //     for(int id : selected_vertex_ids)
  //     {
  //       mean_pos += local_graph_->getVertex(id)->state.head(3);
  //     }
  //     mean_pos /= selected_vertex_ids.size();
  //     std::vector<int> set1, set2;
  //     for(int id : selected_vertex_ids)
  //     {
  //       if(local_graph_->getVertex(id)->state.y() - mean_pos.y() > 0.2)
  //         set1.push_back(id);
  //       else
  //         set2.push_back(id);
  //     }
  //     if(local_graph_->getVertex(0)->state.y() - mean_pos.y() > 0.2)
  //       set1.push_back(0);
  //     else
  //       set2.push_back(0);
  //     if(!set1.empty())
  //     {
  //       std::vector<std::vector<int>> cost_matrix;
  //       generateCostMatrix(selected_vertex_ids, cost_matrix, path_rep_map);
  //       ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[Inpsection]: TSP: Cost matrix: %f", T1.endTimer());

  //       visualization_->visualizeGraphVertices(local_graph_, selected_vertex_ids);
        

  //       std::vector<int> tsp_order = doTSP(selected_vertex_ids, cost_matrix);
  //       ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[Inpsection]: TSP: doTSP: %f", T1.endTimer());
  //     }
  //   }
  // }
  if(!selected_vertex_ids.empty()) {

    if(selected_vertex_ids.size() == 1)
    {
      std::vector<int> tsp_order;
      tsp_order.push_back(0);
      tsp_order.push_back(selected_vertex_ids[0]);
      std::vector<geometry_msgs::Pose> current_path;
		  current_path = connectTSPOrder(tsp_order, path_rep_map, local_graph_);
      visualization_->visualizeRefPath(current_path);
      ret_path = current_path;
    }
    else
    {
      selected_vertex_ids.push_back(0);

      std::vector<std::vector<int>> cost_matrix;
      generateCostMatrix(selected_vertex_ids, cost_matrix, path_rep_map);
      ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[Inpsection]: TSP: Cost matrix: %f", T1.endTimer());

      visualization_->visualizeGraphVertices(local_graph_, selected_vertex_ids);
      

      std::vector<int> tsp_order = doTSP(selected_vertex_ids, cost_matrix);
      ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[Inpsection]: TSP: doTSP: %f", T1.endTimer());
      
      // std::cout << "TSP order Before: "; for (int i = 0; i < tsp_order.size(); ++i) std::cout << tsp_order[i] << ", "; std::cout << std::endl;

      std::vector<int> reordered_tsp_sol;
      if(tsp_order[0] == 0) {
        reordered_tsp_sol = tsp_order;
      }
      else if(tsp_order.back() == 0) {
        reordered_tsp_sol.push_back(0);
        reordered_tsp_sol.insert(reordered_tsp_sol.begin()+1, tsp_order.begin(), tsp_order.end()-1);
      }
      else {
        auto it = std::find(tsp_order.begin(), tsp_order.end(), 0);
        if(it == tsp_order.end()) {
          ROS_WARN_COND(global_verbosity >= Verbosity::ERROR, "Root vertex not in TSP order");
        }
        int root_ind = it - tsp_order.begin();
        
        reordered_tsp_sol.push_back(0);
        reordered_tsp_sol.insert(reordered_tsp_sol.begin()+reordered_tsp_sol.size(), it+1, tsp_order.end());
        reordered_tsp_sol.insert(reordered_tsp_sol.begin()+reordered_tsp_sol.size(), tsp_order.begin(), it);
      }
      tsp_order = reordered_tsp_sol;
      if(tsp_order.size() >= 3)
      {
        // std::cout << "Checking tsp order" << std::endl;
        StateVec f_state = local_graph_->getVertex(tsp_order[1])->state;
        int f_ind = tsp_order[1];
        if((local_graph_->getVertex(0)->state.head(3) - f_state.head(3)).norm() < config_->system_params.vp_reach_thr)
        {
          if(tsp_order.size() > 2)
          {
            f_state = local_graph_->getVertex(tsp_order[2])->state;
            f_ind = tsp_order[2];
          }
        }

        StateVec b_state = local_graph_->getVertex(tsp_order.back())->state;
        int b_ind = tsp_order.size()-1;
        if((local_graph_->getVertex(0)->state.head(3) - b_state.head(3)).norm() < config_->system_params.vp_reach_thr)
        {
          if(tsp_order.size() >= 2)
          {
            b_state = local_graph_->getVertex(tsp_order[tsp_order.size() - 2])->state;
            b_ind = tsp_order[tsp_order.size() - 2];
          }
        }

        double d_yaw_f = local_graph_->getVertex(0)->state[3] - f_state[3];
        truncateAngle(d_yaw_f);
        double d_yaw_b = local_graph_->getVertex(0)->state[3] - b_state[3];
        truncateAngle(d_yaw_b);
        // double cf = std::max(path_rep_map[0].distance_map[f_ind] / planning_params_.v_max,
        //                      std::abs(d_yaw_f) / planning_params_.yaw_rate_max);
        // double cb = std::max(path_rep_map[0].distance_map[b_ind] / planning_params_.v_max,
        //                      std::abs(d_yaw_b) / planning_params_.yaw_rate_max);
        // double cf, cb;
        // {
        //   double t_trans = path_rep_map[0].distance_map[f_ind]/planning_params_.v_max;
        //   double t_rot = std::abs(d_yaw_f) / planning_params_.yaw_rate_max;
        //   if(t_trans > t_rot)
        //   {
        //     cf = t_trans;
        //   }
        //   else
        //   {
        //     cf = t_rot + t_trans;
        //   }
        // }
        // {
        //   double t_trans = path_rep_map[0].distance_map[b_ind]/planning_params_.v_max;
        //   double t_rot = std::abs(d_yaw_b) / planning_params_.yaw_rate_max;
        //   if(t_trans > t_rot)
        //   {
        //     cb = t_trans;
        //   }
        //   else
        //   {
        //     cb = t_rot + t_trans;
        //   }
        // }
        // double cf = path_rep_map[0].distance_map[tsp_order[1]] + std::abs(d_yaw_f);
        // double cb = path_rep_map[0].distance_map[tsp_order.back()] + std::abs(d_yaw_b);
        double dxf = f_state.x() - local_graph_->getVertex(0)->state.x();
        double dyf = f_state.y() - local_graph_->getVertex(0)->state.y();
        double dzf = f_state.z() - local_graph_->getVertex(0)->state.z();

        double dxb = b_state.x() - local_graph_->getVertex(0)->state.x();
        double dyb = b_state.y() - local_graph_->getVertex(0)->state.y();
        double dzb = b_state.z() - local_graph_->getVertex(0)->state.z();

        double cf_check = std::sqrt(dxf*dxf + dyf*dyf + dzf*dzf);
        double cb_check = std::sqrt(dxb*dxb + dyb*dyb + dzb*dzb);
        double cf = (f_state.head(3) - local_graph_->getVertex(0)->state.head(3)).norm();
        double cb = (b_state.head(3) - local_graph_->getVertex(0)->state.head(3)).norm();
        if(std::abs(cf - cf_check) > 0.001 || std::abs(cb - cb_check) > 0.001)
        {
          std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! MATH ERROR !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
        }
        cf = cf_check;
        cb = cb_check;

        if(cf > cb)
        {
          reordered_tsp_sol.clear();
          // std::cout << "Needs to reverse" << std::endl;
          // std::cout << "OG:" << std::endl;
          // for(int e : tsp_order) std::cout << e << " ";
          // std::cout << std::endl;

          reordered_tsp_sol.insert(reordered_tsp_sol.begin(), tsp_order.begin()+1, tsp_order.begin()+tsp_order.size());
          // std::cout << "copied reordered:" << std::endl;
          // for(int e : reordered_tsp_sol) std::cout << e << " ";
          // std::cout << std::endl;

          std::reverse(reordered_tsp_sol.begin(), reordered_tsp_sol.end());
          // std::cout << "reversed reordered:" << std::endl;
          // for(int e : reordered_tsp_sol) std::cout << e << " ";
          // std::cout << std::endl;

          reordered_tsp_sol.insert(reordered_tsp_sol.begin(), tsp_order[0]);
          // std::cout << "final reordered:" << std::endl;
          // for(int e : reordered_tsp_sol) std::cout << e << " ";
          // std::cout << std::endl;
          
          tsp_order = reordered_tsp_sol;
        }
      }
      // std::cout << "TSP order After: "; for (int i = 0; i < tsp_order.size(); ++i) std::cout << tsp_order[i] << ", "; std::cout << std::endl;
      std::vector<int> pruned_tsp_path;
      //////////////////////////////////////////////////
      // pruned_tsp_path.push_back(tsp_order[0]);
      // if(tsp_order.size() > 1)
      // {
      //   pruned_tsp_path.push_back(tsp_order[1]);
      // }
      //////////////////////////////////////////////////
      for(auto it = tsp_order.begin(); it != tsp_order.end(); ++it)
      {
        pruned_tsp_path.push_back(*it);
        if((local_graph_->getVertex(*it)->state.head(3) - current_state_.head(3)).norm() >= config_->system_params.vp_reach_thr)
        {
          break;
        }
      }
      //////////////////////////////////////////////////

      std::vector<geometry_msgs::Pose> current_path;
      current_path = connectTSPOrder(pruned_tsp_path, path_rep_map, local_graph_);
      // std::cout << "Pruned path calculated" << std::endl;
      std::vector<geometry_msgs::Pose> full_path;
      full_path = connectTSPOrder(tsp_order, path_rep_map, local_graph_);
      ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[Inpsection]: TSP: connectTSPOrder: %f", T1.endTimer());

      // geometry_msgs::Pose current_pose;
      // convert(current_state_, current_pose);
      // current_path.push_back(current_pose);

      visualization_->visualizeRefPath(full_path);

      // for(auto p : selected_vertices) {
      //   GbplannerVertex* v = local_graph_->getVertex(p.first);
      //   for(auto sensor : camera_annotation_params_.sensor_list) {
      //     std::vector<Eigen::Vector3d> multiray_endpoints;
      //     camera_annotation_params_.sensor[sensor].getFrustumEndpoints(v->state, multiray_endpoints);
      //     Eigen::Vector3d current_pos = v->state.head(3);
      //     map_manager_->annotateCameraVoxels(current_pos, multiray_endpoints);
      //   }
      // }
      ret_path = current_path;
    }
  }
  else {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "No vertex selected");
  }
  ROS_WARN_COND(global_verbosity >= Verbosity::INFO, "[Inpsection]: TSP: %f", tsp_timer.endTimer());

  t2 = std::chrono::high_resolution_clock::now();

  ROS_WARN_COND(global_verbosity >= Verbosity::INFO, "Inspection path calculation time: %f", std::chrono::duration<double, std::milli>(t2 - t1).count());

  status = InspectionStatus::kOk;

  return ret_path;
}


void Rrg::doAnnotation(bool trig)
{
  planning_params_.annotate_map_with_camera = trig;
}

std::vector<StateVec> Rrg::getVerificationViewpoints(std::vector<Longitudinal> longs)
{
  std::vector<geometry_msgs::Pose> ret_path;

  Eigen::Vector3d long_centroid(0, 0, 0);
  for(auto l : longs)
  {
    long_centroid += l.center;
  }
  long_centroid /= longs.size();

  std::vector<StateVec> viewpoints;
  std::vector<geometry_msgs::Pose> viewpoints_msg;
  std::vector<Eigen::Vector3d> points_to_annotate;
  // Eigen::Matrix<double, 3, 8> neighbor_table;
  // neighbor_table << 0, 1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0,
  //                   0, 0, 1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0, 1, 1,
  //                   0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1;
  // neighbor_table *= 0.2;
  for(auto l : longs)
  {
    Eigen::Vector3d v_r = long_centroid - l.center;
    double yaw_dir = std::atan2(l.direction.y(), l.direction.x());
    StateVec vp;
    if((v_r.cross(l.direction)).z() >= 0)
    {
      vp(3) = yaw_dir + M_PI/2.0;
    }
    else
    {
      vp(3) = yaw_dir - M_PI/2.0;
    }

    vp.head(3) = l.center 
                    - planning_params_.verification_target_viewing_range 
                      * Eigen::Vector3d(std::cos(vp(3)), std::sin(vp(3)), 0.0);
    viewpoints.push_back(vp);
    geometry_msgs::Pose vp_pose;
    convert(vp, vp_pose);
    viewpoints_msg.push_back(vp_pose);

    // points_to_annotate.push_back(l.center - Eigen::Vector3d(4.2,0.0,0.0));
    // points_to_annotate.push_back(l.end_point1 - Eigen::Vector3d(4.2,0.0,0.0));
    // points_to_annotate.push_back(l.end_point2 - Eigen::Vector3d(4.2,0.0,0.0));
    
    // for(int i=0; i<8; ++i)
    // {
    //   points_to_annotate.push_back(l.center + neighbor_table.col(i) - Eigen::Vector3d(4.2,0.0,0.0));
    //   points_to_annotate.push_back(l.end_point1 + neighbor_table.col(i) - Eigen::Vector3d(4.2,0.0,0.0));
    //   points_to_annotate.push_back(l.end_point2 + neighbor_table.col(i) - Eigen::Vector3d(4.2,0.0,0.0));
    // }

    for(double dx=-0.2; dx<=0.2; dx+=0.2)
    {
      for(double dy=-0.2; dy<=0.2; dy+=0.2)
      {
        for(double dz=-0.2; dz<=0.2; dz+=0.2)
        {
          points_to_annotate.push_back(l.center + Eigen::Vector3d(dx, dy, dz) - Eigen::Vector3d(4.2,0.0,0.0));
          points_to_annotate.push_back(l.end_point1 + Eigen::Vector3d(dx, dy, dz) - Eigen::Vector3d(4.2,0.0,0.0));
          points_to_annotate.push_back(l.end_point2 + Eigen::Vector3d(dx, dy, dz) - Eigen::Vector3d(4.2,0.0,0.0));
        }
      }
    }
  }

  annotateSemanticPredictions(points_to_annotate);

  std::map<int, std::vector<int>> visibility_count;
  std::map<int, bool> visible;
  std::vector<StateVec> selected_viewpoints;

  for(int i=0; i<viewpoints.size(); ++i)
  {
    for(int j=0; j<viewpoints.size(); ++j)
    {
      if(j==i)
      {
        visibility_count[i].push_back(j);
        continue;
      }
      
      Eigen::Vector3d ray_c = longs[j].center - viewpoints[i].head(3);
      Eigen::Vector3d ray_ep1 = longs[j].end_point1 - viewpoints[i].head(3);
      Eigen::Vector3d ray_ep2 = longs[j].end_point2 - viewpoints[i].head(3);
      Eigen::Vector3d heading(std::cos(viewpoints[i][3]), std::sin(viewpoints[i][3]), 0.0);
      
      double theta_c = std::atan2(ray_c.y(), ray_c.x())-viewpoints[i][3];
      truncateAngle(theta_c);
      double phi_c = std::atan2(ray_c.z(), Eigen::Vector3d(ray_c.x(), ray_c.y(), 0.0).dot(heading));
      
      double theta_ep1 = std::atan2(ray_ep1.y(), ray_ep1.x())-viewpoints[i][3];
      truncateAngle(theta_ep1);
      double phi_ep1 = std::atan2(ray_ep1.z(), Eigen::Vector3d(ray_ep1.x(), ray_ep1.y(), 0.0).dot(heading));
      
      double theta_ep2 = std::atan2(ray_ep2.y(), ray_ep2.x())-viewpoints[i][3];
      truncateAngle(theta_ep2);
      double phi_ep2 = std::atan2(ray_ep2.z(), Eigen::Vector3d(ray_ep2.x(), ray_ep2.y(), 0.0).dot(heading));
      if(std::abs(theta_c) <= M_PI/2 && std::abs(phi_c) <= M_PI/4.0 &&
         std::abs(theta_ep1) <= M_PI/2 && std::abs(phi_ep1) <= M_PI/4.0 &&
         std::abs(theta_ep2) <= M_PI/2 && std::abs(phi_ep2) <= M_PI/4.0)
      {
        visibility_count[i].push_back(j);
      }
    }
  }

  std::vector<std::pair<int, std::vector<int>>> visibility_count_vec;
  for(auto it : visibility_count) visibility_count_vec.push_back(std::make_pair(it.first, it.second));

  std::sort(visibility_count_vec.begin(), visibility_count_vec.end(), [](std::pair<int, std::vector<int>>& a, std::pair<int, std::vector<int>>& b)
  {
    return a.second.size() > b.second.size();
  });

  int sems_seen = 0;
  for(auto it : visibility_count_vec)
  {
    bool new_sems_seen = false;
    for(int ind : it.second)
    {
      if(!visible[ind])
      {
        new_sems_seen = true;
        visible[ind] = true;
        ++sems_seen;
      }
    }
    if(new_sems_seen)
      selected_viewpoints.push_back(viewpoints[it.first]);
    
    if(sems_seen >= viewpoints.size())  // All sems seen
      break;
  }

  std::vector<geometry_msgs::Pose> selected_viewpoints_msg;
  for(auto &vp : selected_viewpoints)
  {
    vp.x() -= 4.2;  /// HACK!!!
    geometry_msgs::Pose vp_pose;
    convert(vp, vp_pose);
    selected_viewpoints_msg.push_back(vp_pose);
  }

  visualization_->visualizeViewpoints(selected_viewpoints_msg);

  return selected_viewpoints;
}

void Rrg::generateCostMatrix(std::vector<int> nodes, std::vector<std::vector<int>> &cost_matrix, std::map<int, ShortestPathsReport> &path_rep_map)
{
	std::vector<int> temp_cost_mat_row(nodes.size(), 9999999);
	std::vector<std::vector<int>> temp_cost_mat(nodes.size(), temp_cost_mat_row);
	cost_matrix = temp_cost_mat;

	for (int i = 0; i < nodes.size(); ++i)
	{
		for (int j = 0; j < nodes.size(); ++j)
		{
			if (i == j)
			{
				cost_matrix[i][j] = 0;
			}
			else
			{
        if(nodes[j] == 0)
        {
          cost_matrix[i][j] = 0;
        }
        else
        {
          if(planning_params_.use_flipped_yaw) {
            double d_yaw = local_graph_->getVertex(nodes[i])->state[3] - local_graph_->getVertex(nodes[j])->state[3];
            truncateYaw(d_yaw);
            // if(planning_params_.use_gvi)
            //   cost_matrix[i][j] = std::max(path_rep_map[nodes[i]].distance_map[nodes[j]]/planning_params_.v_max, std::abs(d_yaw) / planning_params_.yaw_rate_max);
            // else
            // {
            //   double t_trans = path_rep_map[nodes[i]].distance_map[nodes[j]]/planning_params_.v_max;
            //   double t_rot = std::abs(d_yaw) / planning_params_.yaw_rate_max;
            //   if(t_trans > t_rot)
            //   {
            //     cost_matrix[i][j] = t_trans;
            //   }
            //   else
            //   {
            //     cost_matrix[i][j] = t_rot + t_trans;
            //   }
            // }
            double t_trans = path_rep_map[nodes[i]].distance_map[nodes[j]]/planning_params_.v_max;
            double t_rot = std::abs(d_yaw) / planning_params_.yaw_rate_max;
            if(t_trans > t_rot)
            {
              cost_matrix[i][j] = t_trans;
            }
            else
            {
              cost_matrix[i][j] = t_rot + t_trans;
            }
            // cost_matrix[i][j] = path_rep_map[nodes[i]].distance_map[nodes[j]] + std::abs(d_yaw);
          }
          else {
            cost_matrix[i][j] = path_rep_map[nodes[i]].distance_map[nodes[j]];
          }
        }
			}
		}
	}

	// logger_->debug("Cost matrix:");
	// for(int i=0; i<nodes.size(); ++i)
	// {
	// 	logger_->debug("{}", fmt::join(cost_matrix[i], " "));
	// }
}

double Rrg::calculateTourCost(const std::vector<int>& tour, const std::vector<std::vector<double>>& costMatrix) {
    double total = 0.0;
    int n = tour.size();
    for (int i = 0; i < n; ++i) {
        total += costMatrix[tour[i]][tour[(i + 1) % n]];
    }
    return total;
}

// Perform a 2-opt swap (reverse tour segment between i and k)
void Rrg::twoOptSwap(std::vector<int>& tour, int i, int k) {
    std::reverse(tour.begin() + i, tour.begin() + k + 1);
}

// Solve TSP using 2-opt algorithm
std::vector<int> Rrg::solveTSP(const std::vector<std::vector<double>>& costMatrix) {
    int n = costMatrix.size();
    std::vector<int> tour(n);
    for (int i = 0; i < n; ++i) tour[i] = i;

    bool improved = true;
    double bestCost = calculateTourCost(tour, costMatrix);

    while (improved) {
        improved = false;
        for (int i = 1; i < n - 1; ++i) {
            for (int k = i + 1; k < n; ++k) {
                std::vector<int> newTour = tour;
                twoOptSwap(newTour, i, k);
                double newCost = calculateTourCost(newTour, costMatrix);
                if (newCost < bestCost) {
                    tour = newTour;
                    bestCost = newCost;
                    improved = true;
                }
            }
        }
    }

    return tour;
}

std::vector<int> Rrg::doTSP(std::vector<int> &nodes, std::vector<std::vector<int>> &cost_matrix)
{
  std::vector<int> tsp_order;
  std::vector<std::vector<double>> cost_matrix_double;
  cost_matrix_double.reserve(cost_matrix.size());

  for (const auto& row : cost_matrix) {
      std::vector<double> doubleRow;
      doubleRow.reserve(row.size());

      for (int val : row) {
          doubleRow.push_back(static_cast<double>(val));
      }

      cost_matrix_double.push_back(std::move(doubleRow));
  }
  std::vector<int> sol = solveTSP(cost_matrix_double);
	for (int i = 0; i < sol.size(); i++)
	{
		int id = nodes[sol[i]];
		tsp_order.push_back(id);
	}

	return tsp_order;
}

std::vector<geometry_msgs::Pose> Rrg::connectTSPOrder(std::vector<int> &tsp_nodes, std::map<int, ShortestPathsReport> &path_rep_map, std::shared_ptr<GraphManager> graph)
{
  // std::cout << std::endl;
	std::vector<geometry_msgs::Pose> tsp_path;
	for (int i = 0; i < tsp_nodes.size() - 1; ++i)
	{
		std::vector<StateVec> current_path_segment_vec;
		std::vector<geometry_msgs::Pose> current_path_segment;
		graph->getShortestPath(tsp_nodes[i + 1], path_rep_map[tsp_nodes[i]], true, current_path_segment_vec);
		convert(current_path_segment_vec, current_path_segment);
		linearlyInterpolateYaw(current_path_segment);
		for (int j = 0; j < current_path_segment.size() - 1; ++j)
		{
			tsp_path.push_back(current_path_segment[j]);
		}
	}

	geometry_msgs::Pose last_pose;
	convert(graph->getVertex(tsp_nodes.back())->state, last_pose);
	tsp_path.push_back(last_pose);

	return tsp_path;
}


bool Rrg::addRefPathToGraph(const std::shared_ptr<GraphManager> graph_manager,
                            const std::vector<GbplannerVertex*>& vertices) {
  if (vertices.size() <= 0) return false;

  // The whole path is collision free already and start from root vertex.
  // We only need to link the first vertex to the existing graph.
  // Then add the whole path to the graph.
  // Finally, add more edges along the path to the graph.

  StateVec first_state;
  first_state << vertices[0]->state[0], vertices[0]->state[1],
      vertices[0]->state[2], vertices[0]->state[3];
  GbplannerVertex* nearest_vertex = NULL;
  if (!graph_manager->getNearestVertex(&first_state, &nearest_vertex))
    return false;
  if (nearest_vertex == NULL) return false;
  Eigen::Vector3d origin(nearest_vertex->state[0], nearest_vertex->state[1],
                         nearest_vertex->state[2]);
  Eigen::Vector3d direction(first_state[0] - origin[0],
                            first_state[1] - origin[1],
                            first_state[2] - origin[2]);
  double direction_norm = direction.norm();
  GbplannerVertex* parent_vertex = NULL;
  const double kDeltaLimit = 0.1;
  const double kRadiusLimit = 0.5;

  // Add root vertex first.
  if (direction_norm <= kDeltaLimit) {
    parent_vertex = nearest_vertex;
  } else if (direction_norm <=
             std::max(kRadiusLimit, planning_params_.edge_length_min)) {
    // @TODO: find better way to do this.
    // Blindly add a link/vertex to the graph.
    GbplannerVertex* new_vertex =
        new GbplannerVertex(graph_manager->generateVertexID(), first_state);
    new_vertex->parent = nearest_vertex;
    new_vertex->distance = nearest_vertex->distance + direction_norm;
    nearest_vertex->children.push_back(new_vertex);
    graph_manager->addVertex(new_vertex);
    graph_manager->addEdge(new_vertex, nearest_vertex, direction_norm);
    parent_vertex = new_vertex;
  } else {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[GlobalGraph] Try to add current state to the graph.");
    ExpandGraphReport rep;
    GbplannerVertex new_vertex(-1, first_state);
    expandGraph(graph_manager, new_vertex, rep);
    if (rep.status == ExpandGraphStatus::kSuccess) {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[GlobalGraph] Added successfully.");
      parent_vertex = rep.vertex_added;
    } else {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[GlobalGraph] Can not add current state to the global graph.");
      return false;
    }
  }

  // Add all remaining vertices of the path.
  std::vector<GbplannerVertex*> vertex_list;
  vertex_list.push_back(parent_vertex);
  for (int i = 1; i < vertices.size(); ++i) {
    // Don't add the part of the path after the first hanging vertex
    if (vertices[i]->is_hanging) {
      break;
    }
    StateVec new_state;
    new_state << vertices[i]->state[0], vertices[i]->state[1],
        vertices[i]->state[2], vertices[i]->state[3];
    Eigen::Vector3d origin(parent_vertex->state[0], parent_vertex->state[1],
                           parent_vertex->state[2]);
    Eigen::Vector3d direction(new_state[0] - origin[0],
                              new_state[1] - origin[1],
                              new_state[2] - origin[2]);
    double direction_norm = direction.norm();

    GbplannerVertex* new_vertex =
        new GbplannerVertex(graph_manager->generateVertexID(), new_state);
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
          ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Can not add this vertex: kErrorKdTree.");
          break;
        case ExpandGraphStatus::kErrorCollisionEdge:
          ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Can not add this vertex: kErrorCollisionEdge.");
          break;
        case ExpandGraphStatus::kErrorShortEdge:
          ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Can not add this vertex: kErrorShortEdge.");
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
      GbplannerVertex* prev_vertex = vertex_list[i];
      double acc_len = 0;
      for (int j = 1; j < n_intp; ++j) {
        Eigen::Vector3d new_v;
        new_v = start_vertex + j * intp_len * edge_vec;
        StateVec new_state;
        new_state << new_v[0], new_v[1], new_v[2], vertex_list[i]->state[3];
        GbplannerVertex* new_vertex =
            new GbplannerVertex(graph_manager->generateVertexID(), new_state);
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
  GbplannerVertex* nearest_vertex = NULL;
  if (!graph_manager->getNearestVertex(&first_state, &nearest_vertex))
    return false;
  if (nearest_vertex == NULL) return false;
  Eigen::Vector3d origin(nearest_vertex->state[0], nearest_vertex->state[1],
                         nearest_vertex->state[2]);
  Eigen::Vector3d direction(first_state[0] - origin[0],
                            first_state[1] - origin[1],
                            first_state[2] - origin[2]);
  double direction_norm = direction.norm();
  GbplannerVertex* parent_vertex = NULL;
  const double kDeltaLimit = 0.1;
  const double kRadiusLimit = 0.5;

  // Add root vertex first.
  if (direction_norm <= kDeltaLimit) {
    parent_vertex = nearest_vertex;
  } else if (direction_norm <=
             std::max(kRadiusLimit, planning_params_.edge_length_min)) {
    // @TODO: find better way to do this.
    // Blindly add a link/vertex to the graph.
    GbplannerVertex* new_vertex =
        new GbplannerVertex(graph_manager->generateVertexID(), first_state);
    new_vertex->parent = nearest_vertex;
    new_vertex->distance = nearest_vertex->distance + direction_norm;
    nearest_vertex->children.push_back(new_vertex);
    graph_manager->addVertex(new_vertex);
    graph_manager->addEdge(new_vertex, nearest_vertex, direction_norm);
    parent_vertex = new_vertex;
  } else {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[GlobalGraph] Try to add current state to the graph.");
    ExpandGraphReport rep;
    GbplannerVertex new_vertex(-1, first_state);
    expandGraph(graph_manager, new_vertex, rep);
    if (rep.status == ExpandGraphStatus::kSuccess) {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[GlobalGraph] Added successfully.");
      parent_vertex = rep.vertex_added;
    } else {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[GlobalGraph] Can not add current state to the global graph.");
      return false;
    }
  }

  // Add all remaining vertices of the path.
  std::vector<GbplannerVertex*> vertex_list;
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

    GbplannerVertex* new_vertex =
        new GbplannerVertex(graph_manager->generateVertexID(), new_state);
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
          ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Can not add this vertex: kErrorKdTree.");
          break;
        case ExpandGraphStatus::kErrorCollisionEdge:
          ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Can not add this vertex: kErrorCollisionEdge.");
          break;
        case ExpandGraphStatus::kErrorShortEdge:
          ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Can not add this vertex: kErrorShortEdge.");
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
      GbplannerVertex* prev_vertex = vertex_list[i];
      double acc_len = 0;
      for (int j = 1; j < n_intp; ++j) {
        Eigen::Vector3d new_v;
        new_v = start_vertex + j * intp_len * edge_vec;
        StateVec new_state;
        new_state << new_v[0], new_v[1], new_v[2], vertex_list[i]->state[3];
        GbplannerVertex* new_vertex =
            new GbplannerVertex(graph_manager->generateVertexID(), new_state);
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
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Received the first odometry, reset the map");
    map_manager_->resetMap();
  }
  current_state_ = state;
  odometry_ready = true;
  // Clear free space based on current voxel size.
  if (planner_trigger_count_ < planning_params_.augment_free_voxels_time) {
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
  // ROS_WARN("tC: start: GG verts: %d, edges: %d", global_graph_->getNumVertices(), global_graph_->getNumEdges());
  if (rostime_start_.toSec() == 0) rostime_start_ = ros::Time::now();

  if (!odometry_ready) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Planner is waiting for odometry");
    return;
  }

  if (landing_engaged_ && robot_params_.type == RobotType::kAerialRobot) {
    return;
  }

  // Check if needs to land
  if (planning_params_.auto_landing_enable && !landing_engaged_ &&
      robot_params_.type == RobotType::kAerialRobot) {
    double time_elapsed = 0.0;
    if ((ros::Time::now()).toSec() != 0.0) {
      if (rostime_start_.toSec() == 0.0) rostime_start_ = ros::Time::now();
      time_elapsed = (double)((ros::Time::now() - rostime_start_).toSec());
    }
    double time_budget_remaining =
        planning_params_.time_budget_before_landing - time_elapsed;
    if (time_budget_remaining <= 0.0) {
      if (!local_exploration_ongoing_) {
        ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "RAN OUT OF TIME BUDGET --> LANDING.");
        landing_engaged_ = true;
        std_msgs::Bool stop_msg;
        stop_msg.data = true;
        std_srvs::Empty empty_srv;
        landing_srv_client_.call(empty_srv);
        pci_reset_pub_.publish(stop_msg);
        /* TODO: What happens if the disarm service fails */
      }
      return;
    }
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
  // map_manager_->setLineCheckMethod(2);
  bool enforce_vertex_from_odometry = true;
  double kOdoEnforceLength = planning_params_.edge_length_min;
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
          GbplannerVertex* new_vertex =
              new GbplannerVertex(global_graph_->generateVertexID(), bt_state);
          new_vertex->parent = robot_backtracking_prev_;
          new_vertex->distance = robot_backtracking_prev_->distance + dir_norm;
          // GbplannerVertex new_vertex(-1, bt_state);
          // new_vertex.parent = robot_backtracking_prev_;
          // new_vertex.distance = robot_backtracking_prev_->distance + dir_norm;
          // offsetZAxis(new_vertex->state);
          if (robot_params_.type == RobotType::kGroundRobot) {
            MapManager::VoxelStatus vs;
            Eigen::Vector3d new_vertex_pos = new_vertex->state.head(3);
            double ground_height = projectSample(new_vertex_pos, vs);
            if (vs == MapManager::VoxelStatus::kOccupied) {
              new_vertex->state(2) -=
                  (ground_height - planning_params_.max_ground_height);
            }
            // Eigen::Vector3d new_vertex_pos = new_vertex.state.head(3);
            // double ground_height = projectSample(new_vertex_pos, vs);
            // if (vs == MapManager::VoxelStatus::kOccupied) {
            //   new_vertex.state(2) -=
            //       (ground_height - planning_params_.max_ground_height);
            // }
          }
          global_graph_->addVertex(new_vertex);
          // could increase the distance to limit shortest paths along these
          // edges for safety purpose since we don't check the collision
          const double kEdgeWeightExtended = 1.0;
          global_graph_->addEdge(new_vertex, robot_backtracking_prev_,
                                 dir_norm * kEdgeWeightExtended);
          robot_backtracking_prev_ = new_vertex;
          ExpandGraphReport rep;
          // ROS_WARN("Expand edges on odometry");
          expandGraphEdges(global_graph_, new_vertex, rep);
          // ExpandGraphReport rep;
          // ROS_WARN("Expand edges on odometry");
          // expandGraph(global_graph_, new_vertex, rep, false);
        }
      }
    }
  }
  map_manager_->setLineCheckMethod(planning_params_.line_check_method);

  // Get position from odometry to add more vertices to the graph for homing.
  Eigen::Vector3d cur_dir(current_state_[0] - last_state_marker_[0],
                          current_state_[1] - last_state_marker_[1],
                          current_state_[2] - last_state_marker_[2]);

  double kOdoUpdateMinLength = kOdoEnforceLength;
  if (cur_dir.norm() >= kOdoUpdateMinLength) {
    // Add this to the global graph.
    const bool add_odometry_to_global_graph = true;
    if (add_odometry_to_global_graph) {
      StateVec new_state;
      new_state = current_state_;
      new_state[2] -=
          (planning_params_.robot_height - planning_params_.max_ground_height);
      ExpandGraphReport rep;
      GbplannerVertex new_vertex(-1, new_state);
      expandGraph(global_graph_, new_vertex, rep);
      // ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "From odometry, added %d vertices and %d edges",
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

  // ROS_WARN("tC: end: GG verts: %d, edges: %d", global_graph_->getNumVertices(), global_graph_->getNumEdges());
  visualization_->visualizeGlobalGraph(global_graph_);
}

void Rrg::setBoundMode(BoundModeType bmode) {
  constexpr double kNumVerticesRatio = 1.3;
  constexpr double kNumEdgesRatio = 1.3;
  robot_params_.setBoundMode(bmode);
  // Update the robot size for planning.
  robot_params_.getPlanningSize(robot_box_size_);

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

  if (std::abs(dyaw) <= kYawThres) {
    return true;
  } else {
    return false;
  }
}

std::vector<int> Rrg::performShortestPathsClustering(
    const std::shared_ptr<GraphManager> graph_manager,
    const ShortestPathsReport& graph_rep, std::vector<GbplannerVertex*>& vertices,
    double dist_threshold, double principle_path_min_length,
    bool refinement_enable) {
  // Asumme long paths are principle paths.
  // Go over one by one from the longest ones.
  // Group into clusters based on the normalized DTW distance metric.
  // Refine by choosing closest & valid principle path.

  // Sort into descending order.
  std::sort(vertices.begin(), vertices.end(),
            [&graph_manager, &graph_rep](const GbplannerVertex* a, const GbplannerVertex* b) {
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

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Cluster %d paths into %d clusters.", (int)vertices.size(),
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
    ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Clustering with refinement %d paths into %d clusters.",
             (int)vertices.size(), (int)cluster_paths_refine.size());
    return cluster_ids_refine;
  } else {
    return cluster_ids;
  }
}

void Rrg::cleanViolatedEdgesInGraph(
    std::shared_ptr<GraphManager> graph_manager) {
  std::shared_ptr<BaseGraph> g = graph_manager->graph_;
  std::pair<BaseGraph::GraphType::edge_iterator, BaseGraph::GraphType::edge_iterator>
      ei;
  g->getEdgeIterator(ei);
  for (BaseGraph::GraphType::edge_iterator it = ei.first; it != ei.second; ++it) {
    int src_id, tgt_id;
    double weight;
    std::tie(src_id, tgt_id, weight) = g->getEdgeProperty(it);
    GbplannerVertex* src_v = graph_manager->getVertex(src_id);
    GbplannerVertex* tgt_v = graph_manager->getVertex(tgt_id);
    if (GeofenceManager::CoordinateStatus::kViolated ==
        geofence_manager_->getPathStatus(
            Eigen::Vector2d(src_v->state[0], src_v->state[1]),
            Eigen::Vector2d(tgt_v->state[0], tgt_v->state[1]),
            Eigen::Vector2d(robot_box_size_[0], robot_box_size_[1]))) {
      graph_manager->removeEdge(src_v, tgt_v);
    }
  }
}

bool Rrg::connectStateToGraph(std::shared_ptr<GraphManager> graph,
                              StateVec& cur_state, GbplannerVertex*& v_added,
                              double dist_ignore_collision_check) {
  GbplannerVertex* nearest_vertex = NULL;
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
    GbplannerVertex* new_vertex = new GbplannerVertex(graph->generateVertexID(), cur_state);
    new_vertex->parent = nearest_vertex;
    new_vertex->distance = nearest_vertex->distance + direction_norm;
    graph->addVertex(new_vertex);
    graph->addEdge(new_vertex, nearest_vertex, direction_norm);
    // Add edges only from this vertex.
    ExpandGraphReport rep;
    expandGraphEdges(graph, new_vertex, rep);
    v_added = new_vertex;
  } else {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[GlobalGraph] Try to add current state to the graph.");
    ExpandGraphReport rep;
    GbplannerVertex new_vertex(-1, cur_state);
    expandGraph(graph, new_vertex, rep);
    if (rep.status == ExpandGraphStatus::kSuccess) {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[GlobalGraph] Added successfully.");
      v_added = rep.vertex_added;
    } else {
      // Not implemented solution for this case yet.
      connect_state_to_graph = false;
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[GlobalGraph] Can not add current state to graph since: ");
      switch (rep.status) {
        case ExpandGraphStatus::kErrorKdTree:
          ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "kErrorKdTree.");
          break;
        case ExpandGraphStatus::kErrorCollisionEdge:
          ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "kErrorCollisionEdge.");
          break;
        case ExpandGraphStatus::kErrorShortEdge:
          ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "kErrorShortEdge.");
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
  // Check two conditions (time budget vs battery) whatever which one comes
  // first.
  return std::min(time_budget_remaining, current_battery_time_remaining_);
}

bool Rrg::isRemainingTimeSufficient(const double& time_cost,
                                    double& time_spare) {
  const double kTimeDelta = 20;  // magic number, extra safety
  time_spare = getTimeRemained() - time_cost;
  if (time_spare < kTimeDelta) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "REACHED TIME LIMIT: BE CAREFUL.");
    return false;
  }
  return true;
}

bool Rrg::getManholePathCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
	std::vector<geometry_msgs::Pose> path = getManholeTraversalPath();
	
	if(!path.empty()) {
		res.success = true;
	}
	return true;
}

bool Rrg::approvePassingCallback(planner_msgs::planner_manhole_approval::Request &req, planner_msgs::planner_manhole_approval::Response &res) {
  if(req.approval == planner_msgs::planner_manhole_approval::Request::kApproved) {
    manhole_passing_approved_ = ManholeApproval::kApproved;
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Manhole Passing Approved");
  }
  else if(req.approval == planner_msgs::planner_manhole_approval::Request::kRejected) {
    manhole_passing_approved_ = ManholeApproval::kRejected;
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Manhole Passing Rejected");
  }
  else if(req.approval == planner_msgs::planner_manhole_approval::Request::kReEvaluate) {
    manhole_passing_approved_ = ManholeApproval::kReEvaluate;
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Manhole Passing to be Re-evaluated");
  }
  
  return true;
}

void Rrg::setNextCompartmentCenter(Eigen::Vector3d &center)
{
  next_compartment_ = center;
}

std::vector<geometry_msgs::Pose> Rrg::getManholeTraversalPath() {
	std::vector<geometry_msgs::Pose> through_path;

  std::vector<ManholeDetection> updated_detections;
  // if(!manhole_detector_->getStableManholes(updated_detections)) {
  //   return through_path;
  // }
  manhole_detector_->getStableManholes(updated_detections);

  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "MH Mode: %d", manhole_traversal_status_);
  
  if(manhole_traversal_status_ == ManholeTraversalMode::kNone) {
    manhole_traversal_status_ = ManholeTraversalMode::kGoingTo;
  }
  else if(manhole_traversal_status_ == ManholeTraversalMode::kGoingTo) {
    // manhole_detector_->reEvaluate(manhole_under_execution_);
    // double waiting_time = 0.75;
    // double dt = 0.01;
    // for(double t=0; t<waiting_time; t+=dt) {
    //   ros::Duration(dt).sleep();
    //   // ros::spinOnce();
    // }
    if(planning_params_.auto_manhole_path_approval) {
    	manhole_traversal_status_ = ManholeTraversalMode::kPassingThrough;
    }
    else{
    	manhole_traversal_status_ = ManholeTraversalMode::kPathCheck;
    }
  }
  else if(manhole_traversal_status_ == ManholeTraversalMode::kPathCheck) {
    if(planning_params_.auto_manhole_path_approval) {
      manhole_passing_approved_ = ManholeApproval::kApproved;
    }
    if(manhole_passing_approved_ == ManholeApproval::kApproved) {
      ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Manhole Traversal Approved");
      manhole_passing_approved_ = ManholeApproval::kWaiting;
      manhole_traversal_status_ = ManholeTraversalMode::kPassingThrough;
    }
    else if(manhole_passing_approved_ == ManholeApproval::kRejected) {
      ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Manhole Traversal Rejected");
      manhole_traversal_status_ = ManholeTraversalMode::kNone;
      detected_manholes_[manhole_under_execution_]->active = false;
      return through_path;
    }
    else if(manhole_passing_approved_ == ManholeApproval::kReEvaluate) {
      ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Manhole Traversal ReEvaluate");
      manhole_traversal_status_ = ManholeTraversalMode::kPathCheck;
    }
  }

  // if(!manhole_detector_->getStableManholes(updated_detections)) {
  //   return through_path;
  // }
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "MH Mode: %d", manhole_traversal_status_);
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "MH under exect: %d", manhole_under_execution_);

  if(manhole_traversal_status_ != ManholeTraversalMode::kPassingThrough) {
    manhole_detector_->getStableManholes(updated_detections);

    bool manhole_still_exists = false;
    // std::cout << "Dets IDs:" << std::endl;
    for(auto new_det : updated_detections) {
      auto it = detected_manholes_.find(new_det.id);
      // std::cout << new_det.id << std::endl;
      if((manhole_traversal_status_ == ManholeTraversalMode::kPathCheck) && 
        new_det.id == manhole_under_execution_) {
        manhole_still_exists = true;
      }
      if(it != detected_manholes_.end()) {
        std::shared_ptr<Manhole> mh = detected_manholes_[new_det.id];
        tf::Quaternion quat;
        quat.setEuler(0.0, 0.0, new_det.mean_yaw);
        tf::Vector3 origin(new_det.mean_pos[0], new_det.mean_pos[1], new_det.mean_pos[2]);
        tf::Pose poseTF(quat, origin);
        geometry_msgs::Pose pose;
        tf::poseTFToMsg(poseTF, pose);
        mh->pose = pose;
      }
      else {
        // New manhole
        if(new_det.mean_pos[2] > planning_params_.max_manhole_height) {
          ROS_WARN("Manhole %d too high (%f)", new_det.id, new_det.mean_pos[2]);
          continue;
        }
        std::shared_ptr<Manhole> mh;
        mh.reset(new Manhole());
        mh->id = new_det.id;
        tf::Quaternion quat;
        quat.setEuler(0.0, 0.0, new_det.mean_yaw);
        tf::Vector3 origin(new_det.mean_pos[0], new_det.mean_pos[1], new_det.mean_pos[2]);
        tf::Pose poseTF(quat, origin);
        geometry_msgs::Pose pose;
        tf::poseTFToMsg(poseTF, pose);
        mh->pose = pose;

        detected_manholes_[mh->id] = mh;
      }
    }

    if(manhole_traversal_status_ == ManholeTraversalMode::kPathCheck
      && !manhole_still_exists) {  // If after second detection, the manhole is found to be a false detection, reject it and go to the next closest
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "This MH does not exist");
      manhole_traversal_status_ = ManholeTraversalMode::kGoingTo;
    }
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Detections updated");
  }
  else {
    for(auto new_det : updated_detections) {
      auto it = detected_manholes_.find(new_det.id);
      if(it != detected_manholes_.end()) {
        std::shared_ptr<Manhole> mh = detected_manholes_[new_det.id];
        tf::Quaternion quat;
        quat.setEuler(0.0, 0.0, new_det.mean_yaw);
        tf::Vector3 origin(new_det.mean_pos[0], new_det.mean_pos[1], new_det.mean_pos[2]);
        tf::Pose poseTF(quat, origin);
        geometry_msgs::Pose pose;
        tf::poseTFToMsg(poseTF, pose);
        mh->pose = pose;
      }
      else {
        // New manhole
        if(new_det.mean_pos[2] > planning_params_.max_manhole_height) {
          ROS_WARN("Manhole %d too high (%f)", new_det.id, new_det.mean_pos[2]);
          continue;
        }
        std::shared_ptr<Manhole> mh;
        mh.reset(new Manhole());
        mh->id = new_det.id;
        tf::Quaternion quat;
        quat.setEuler(0.0, 0.0, new_det.mean_yaw);
        tf::Vector3 origin(new_det.mean_pos[0], new_det.mean_pos[1], new_det.mean_pos[2]);
        tf::Pose poseTF(quat, origin);
        geometry_msgs::Pose pose;
        tf::poseTFToMsg(poseTF, pose);
        mh->pose = pose;

        detected_manholes_[mh->id] = mh;
      }
    }
  }


	if(detected_manholes_.empty()) {
    ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "No Manholes at all");
		return through_path;
	}

	std::shared_ptr<Manhole> best_manhole;
	double closest_distance = std::numeric_limits<double>::max();
  bool found = false;
  if(manhole_traversal_status_ != ManholeTraversalMode::kPassingThrough && manhole_traversal_status_ != ManholeTraversalMode::kPathCheck) {
    for(auto it : detected_manholes_) {
      std::shared_ptr<Manhole> current_manhole = it.second;
      if(!current_manhole->active) {
        /* TODO: Update active status of all the semantics based on this */
        continue;
      }
      
      // double mh_robot_dist = (planning_params_.compartment_centers[next_compartment_index_-1] - Eigen::Vector3d(current_manhole->pose.position.x, current_manhole->pose.position.y, current_manhole->pose.position.z)).norm();
      // Eigen::Vector3d compartment_dim = planning_params_.compartment_dimensions.max_val - planning_params_.compartment_dimensions.min_val;
      // if(mh_robot_dist > compartment_dim.norm()/2.0)
      // {
      //   continue;
      // }
      // double dist;
      // if(next_compartment_.x() < std::numeric_limits<double>::max())
      //   dist = (next_compartment_ - Eigen::Vector3d(current_manhole->pose.position.x, current_manhole->pose.position.y, current_manhole->pose.position.z)).norm();
      // else
      //   dist = (current_state_.head(3) - Eigen::Vector3d(current_manhole->pose.position.x, current_manhole->pose.position.y, current_manhole->pose.position.z)).norm();
      // if(dist < closest_distance) {
      //   closest_distance = dist;
      //   best_manhole = current_manhole;
      //   found = true;
      // }
      if(planning_params_.exploration_only)  // If exploration only, go to the closest manhole
      {
        double mh_robot_dist = (current_state_.head(3) 
              - Eigen::Vector3d(current_manhole->pose.position.x, 
                                current_manhole->pose.position.y, 
                                current_manhole->pose.position.z)).norm();
        if(mh_robot_dist < closest_distance) {
          closest_distance = mh_robot_dist;
          best_manhole = current_manhole;
          found = true;
        }
      }
      else // If exploration + inspection, find the manhole that leads to the next compartment
      {
        double mh_robot_dist = (planning_params_.compartment_centers[next_compartment_index_-1] - Eigen::Vector3d(current_manhole->pose.position.x, current_manhole->pose.position.y, current_manhole->pose.position.z)).norm();
        Eigen::Vector3d compartment_dim = planning_params_.compartment_dimensions.max_val - planning_params_.compartment_dimensions.min_val;
        if(mh_robot_dist > compartment_dim.norm()/2.0)
        {
          continue;
        }
        double dist;
        if(next_compartment_.x() < std::numeric_limits<double>::max())
          dist = (next_compartment_ - Eigen::Vector3d(current_manhole->pose.position.x, current_manhole->pose.position.y, current_manhole->pose.position.z)).norm();
        else
          dist = (current_state_.head(3) - Eigen::Vector3d(current_manhole->pose.position.x, current_manhole->pose.position.y, current_manhole->pose.position.z)).norm();
        if(dist < closest_distance) {
          closest_distance = dist;
          best_manhole = current_manhole;
          found = true;
        }
      }
    }
    if(found)
      manhole_under_execution_ = best_manhole->id;
  }
  else {
    best_manhole = detected_manholes_[manhole_under_execution_];
    found = true;
  }

  if(!found) {
    ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "No Manhole Found");
    manhole_traversal_status_ = ManholeTraversalMode::kNone;
    return through_path;
  }

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Best mh found: %d", manhole_under_execution_);

  bool decision_made = false;
  int max_tries = 5;
  std::vector<geometry_msgs::Pose> empty_path;
  
  if(manhole_traversal_status_ != ManholeTraversalMode::kPassingThrough) {
    decision_made = true;
  }

  if(global_verbosity >= Verbosity::DEBUG) {
    std::cout << best_manhole->pose.position.x << " " << best_manhole->pose.position.y << " " << best_manhole->pose.position.z << " | "
              << best_manhole->pose.orientation.x << " " << best_manhole->pose.orientation.y << " " << best_manhole->pose.orientation.z << " " << best_manhole->pose.orientation.w << std::endl;
  }
  
  double direction = tf::getYaw(best_manhole->pose.orientation);
  // std::cout << "Direction: " << direction << std::endl;
  geometry_msgs::Pose p0;
  p0.position.x = best_manhole->pose.position.x - planning_params_.manhole_traversal_path_edge_length * std::cos(direction);
  p0.position.y = best_manhole->pose.position.y - planning_params_.manhole_traversal_path_edge_length * std::sin(direction);
  p0.position.z = best_manhole->pose.position.z;
  p0.orientation = best_manhole->pose.orientation;
  // through_path.push_back(p0);
  // through_path.push_back(best_manhole->pose);
  geometry_msgs::Pose p1;
  p1.position.x = best_manhole->pose.position.x + planning_params_.manhole_traversal_path_edge_length * std::cos(direction);
  p1.position.y = best_manhole->pose.position.y + planning_params_.manhole_traversal_path_edge_length * std::sin(direction);
  p1.position.z = best_manhole->pose.position.z;
  p1.orientation = best_manhole->pose.orientation;
  // through_path.push_back(p1);
  
  // best_manhole->active = false;
  // best_manhole->through_path = through_path;

  geometry_msgs::Pose first_pose;
  if(getDistance(current_state_, p0) < getDistance(current_state_, p1)) {
    first_pose = p0;
  }
  else {
    first_pose = p1;
  }

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Path direction set");
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "MH Mode (later): %d", manhole_traversal_status_);

  geometry_msgs::Quaternion corrected_quat = best_manhole->pose.orientation;

  if(getDistance(current_state_, p0) < getDistance(current_state_, p1)) {
      tf::Quaternion quat;
      double corrected_yaw = std::atan2(p1.position.y - p0.position.y, p1.position.x - p0.position.x);
      quat.setEuler(0.0, 0.0, corrected_yaw);
      tf::quaternionTFToMsg(quat, corrected_quat);
      tf::quaternionTFToMsg(quat, p0.orientation);
      tf::quaternionTFToMsg(quat, best_manhole->pose.orientation);
      tf::quaternionTFToMsg(quat, p1.orientation);
      // through_path.push_back(p0);
      // through_path.push_back(best_manhole->pose);
      // through_path.push_back(p1);

  }
  else {
    tf::Quaternion quat;
    double corrected_yaw = std::atan2(p0.position.y - p1.position.y, p0.position.x - p1.position.x);
    quat.setEuler(0.0, 0.0, corrected_yaw);
    tf::quaternionTFToMsg(quat, corrected_quat);
    tf::quaternionTFToMsg(quat, p0.orientation);
    tf::quaternionTFToMsg(quat, best_manhole->pose.orientation);
    tf::quaternionTFToMsg(quat, p1.orientation);
    // through_path.push_back(p1);
    // through_path.push_back(best_manhole->pose);
    // through_path.push_back(p0);
  }

  tf::Quaternion corrected_quat_tf;
  tf::quaternionMsgToTF(corrected_quat, corrected_quat_tf);
  double corrected_yaw = tf::getYaw(corrected_quat_tf);

  // std::cout << "p0" << p0.position.x << " " << p0.position.y << " " << p0.position.z << " " << corrected_yaw << std::endl;
  // std::cout << "mh" << best_manhole->pose.position.x << " " << best_manhole->pose.position.y << " " << best_manhole->pose.position.z << " " << corrected_yaw << std::endl;
  // std::cout << "p1" << p1.position.x << " " << p1.position.y << " " << p1.position.z << " " << corrected_yaw << std::endl;
  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Corrected quat: %f", corrected_yaw);

  if(manhole_traversal_status_ != ManholeTraversalMode::kGoingTo) {
    if(getDistance(current_state_, p0) < getDistance(current_state_, p1)) {
      // tf::Quaternion quat;
      // double corrected_yaw = std::atan2(p1.position.y - p0.position.y, p1.position.x - p0.position.x);
      // quat.setEuler(0.0, 0.0, corrected_yaw);
      // tf::quaternionTFToMsg(quat, p0.orientation);
      // tf::quaternionTFToMsg(quat, best_manhole->pose.orientation);
      // tf::quaternionTFToMsg(quat, p1.orientation);
      p0.orientation = corrected_quat;
      best_manhole->pose.orientation = corrected_quat;
      p1.orientation = corrected_quat;
      through_path.push_back(p0);
      through_path.push_back(best_manhole->pose);
      through_path.push_back(p1);

    }
    else {
      // tf::Quaternion quat;
      // double corrected_yaw = std::atan2(p0.position.y - p1.position.y, p0.position.x - p1.position.x);
      // quat.setEuler(0.0, 0.0, corrected_yaw);
      // tf::quaternionTFToMsg(quat, p0.orientation);
      // tf::quaternionTFToMsg(quat, best_manhole->pose.orientation);
      // tf::quaternionTFToMsg(quat, p1.orientation);
      p0.orientation = corrected_quat;
      best_manhole->pose.orientation = corrected_quat;
      p1.orientation = corrected_quat;
      through_path.push_back(p1);
      through_path.push_back(best_manhole->pose);
      through_path.push_back(p0);
    }

    // ROS_WARN_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "Manhole Traversal Path:");
    ROS_WARN_COND(true, "Manhole Traversal Path:");
    if(global_verbosity >= Verbosity::DEBUG) {
    // if(true) {
      std::cout << p0.position.x << " " << p0.position.y << " " << p0.position.z << " " << direction << std::endl;
      std::cout << best_manhole->pose.position.x << " " << best_manhole->pose.position.y << " " << best_manhole->pose.position.z << " " << direction << std::endl;
      std::cout << p1.position.x << " " << p1.position.y << " " << p1.position.z << " " << direction << std::endl;
    }
  }
  else {
    first_pose.orientation = corrected_quat;
    first_pose.position.z += planning_params_.manhole_alignment_z_offset;
    // std::cout << "First Pose z: " << first_pose.position.z << " offset: " << planning_params_.manhole_alignment_z_offset << std::endl;
  }

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Path set 1");

  geometry_msgs::Pose current_pose;
  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, current_state_[3]);
  tf::Vector3 origin(current_state_[0], current_state_[1], current_state_[2]);
  tf::Pose poseTF(quat, origin);
  tf::poseTFToMsg(poseTF, current_pose);

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Path set 2");

  std::vector<geometry_msgs::Pose> connecting_path;
  bool success = search(current_pose, first_pose, false, connecting_path);
  if(success) {
    // connecting_path.back().orientation = best_manhole->pose.orientation;
    connecting_path.push_back(first_pose);
    connecting_path.back().orientation = corrected_quat;
    through_path.insert(through_path.begin(), connecting_path.begin(), connecting_path.end());
  }
  else {
    ROS_ERROR_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "Connecting path not found");
    return empty_path;
  }

  linearlyInterpolateYaw(through_path);

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Path set 3");

  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "MH Mode (even later): %d", manhole_traversal_status_);

  visualization_->visualizeManholeTraversalPath(through_path);

  if(manhole_traversal_status_ == ManholeTraversalMode::kPathCheck) {
    return empty_path;
  } 
  else if(manhole_traversal_status_ == ManholeTraversalMode::kPassingThrough) {
    for(int i=0; i<through_path.size(); ++i) {
      // through_path[i].orientation = best_manhole->pose.orientation;
      through_path[i].orientation = corrected_quat;
    }
    best_manhole->active = false;

    manhole_traversal_status_ = ManholeTraversalMode::kNone;
    return through_path;

    // while(manhole_passing_approved_ == ManholeApproval::kWaiting) {
    //   ros::Duration(0.1).sleep();
    //   ros::spinOnce();
    // }
    // manhole_passing_approved_ = ManholeApproval::kApproved;
    // if(manhole_passing_approved_ == ManholeApproval::kReEvaluate) {
    //   ROS_WARN("Re evaluate the manhole location");
    //   manhole_detector_->reEvaluate(manhole_under_execution_);
    //   // double waiting_time = 0.75;
    //   // double dt = 0.01;
    //   // for(double t=0; t<waiting_time; t+=dt) {
    //   //   ros::Duration(dt).sleep();
    //   //   ros::spinOnce();
    //   // }
    //   ROS_WARN("Re evaluation done");
    //   std::vector<ManholeDetection> updated_detections;
    //   manhole_detector_->getStableManholes(updated_detections);
    //   for(auto new_det : updated_detections) {
    //     auto it = detected_manholes_.find(new_det.id);
    //     if(manhole_traversal_status_ == ManholeTraversalMode::kPassingThrough && 
    //       new_det.id == manhole_under_execution_) {
    //       manhole_still_exists = true;
    //     }
    //     if(it != detected_manholes_.end()) {
    //       std::shared_ptr<Manhole> mh = detected_manholes_[new_det.id];
    //       tf::Quaternion quat;
    //       quat.setEuler(0.0, 0.0, new_det.mean_yaw);
    //       tf::Vector3 origin(new_det.mean_pos[0], new_det.mean_pos[1], new_det.mean_pos[2]);
    //       tf::Pose poseTF(quat, origin);
    //       geometry_msgs::Pose pose;
    //       tf::poseTFToMsg(poseTF, pose);
    //       mh->pose = pose;
    //     }
    //     else {
    //       std::shared_ptr<Manhole> mh;
    //       mh.reset(new Manhole());
    //       mh->id = new_det.id;
    //       tf::Quaternion quat;
    //       quat.setEuler(0.0, 0.0, new_det.mean_yaw);
    //       tf::Vector3 origin(new_det.mean_pos[0], new_det.mean_pos[1], new_det.mean_pos[2]);
    //       tf::Pose poseTF(quat, origin);
    //       geometry_msgs::Pose pose;
    //       tf::poseTFToMsg(poseTF, pose);
    //       mh->pose = pose;

    //       detected_manholes_[mh->id] = mh;
    //     }
    //   }
    //   ROS_WARN("Resetting the manhole location");
    //   best_manhole = detected_manholes_[manhole_under_execution_];
    //   manhole_passing_approved_ = ManholeApproval::kWaiting;
    // }
    // else {
    //   decision_made = true;
    //   ROS_WARN("Decision Made: %d", (int)manhole_passing_approved_);
    // }
  }
  else {
    return through_path;
  }

  // if(manhole_traversal_status_ == ManholeTraversalMode::kPassingThrough) {
  //   manhole_traversal_status_ = ManholeTraversalMode::kNone;
  //   if(manhole_passing_approved_ == ManholeApproval::kApproved) {
  //     manhole_passing_approved_ = ManholeApproval::kWaiting;
  //     ROS_WARN("Manhole Traversal Approved");
  //     best_manhole->active = false;
  //     return through_path;
  //   }
  //   else {
  //     manhole_passing_approved_ = ManholeApproval::kWaiting;
  //     return empty_path;
  //   }
  // }
  // else {
  //   return through_path;
  // }
}

std::vector<geometry_msgs::Pose> Rrg::getManholeTraversalPath(ManholeTraversalMode mode, ManholeTraversalStatus &status, int mh_id) {
	std::vector<geometry_msgs::Pose> through_path;

  std::vector<ManholeDetection> updated_detections;
  // if(!manhole_detector_->getStableManholes(updated_detections)) {
  //   return through_path;
  // }
  manhole_detector_->getStableManholes(updated_detections);

  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "MH Mode: %d", mode);
  
  // if(manhole_traversal_status_ == ManholeTraversalMode::kNone) {
  //   manhole_traversal_status_ = ManholeTraversalMode::kGoingTo;
  // }
  // else if(manhole_traversal_status_ == ManholeTraversalMode::kGoingTo) {
  //   if(planning_params_.auto_manhole_path_approval) {
  //   	manhole_traversal_status_ = ManholeTraversalMode::kPassingThrough;
  //   }
  //   else{
  //   	manhole_traversal_status_ = ManholeTraversalMode::kPathCheck;
  //   }
  // }
  // else if(manhole_traversal_status_ == ManholeTraversalMode::kPathCheck) {
  //   if(planning_params_.auto_manhole_path_approval) {
  //     manhole_passing_approved_ = ManholeApproval::kApproved;
  //   }
  //   if(manhole_passing_approved_ == ManholeApproval::kApproved) {
  //     ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Manhole Traversal Approved");
  //     manhole_passing_approved_ = ManholeApproval::kWaiting;
  //     manhole_traversal_status_ = ManholeTraversalMode::kPassingThrough;
  //   }
  //   else if(manhole_passing_approved_ == ManholeApproval::kRejected) {
  //     ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Manhole Traversal Rejected");
  //     manhole_traversal_status_ = ManholeTraversalMode::kNone;
  //     detected_manholes_[manhole_under_execution_]->active = false;
  //     return through_path;
  //   }
  //   else if(manhole_passing_approved_ == ManholeApproval::kReEvaluate) {
  //     ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Manhole Traversal ReEvaluate");
  //     manhole_traversal_status_ = ManholeTraversalMode::kPathCheck;
  //   }
  // }

  // if(!manhole_detector_->getStableManholes(updated_detections)) {
  //   return through_path;
  // }
  // ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "MH Mode: %d", manhole_traversal_status_);
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "MH under exect: %d", manhole_under_execution_);

  manhole_detector_->getStableManholes(updated_detections);
  bool manhole_still_exists = false;
  // std::cout << "Dets IDs:" << std::endl;
  for(auto new_det : updated_detections) {
    auto it = detected_manholes_.find(new_det.id);
    // std::cout << new_det.id << std::endl;
    if((mode == ManholeTraversalMode::kPathCheck) && 
      new_det.id == manhole_under_execution_) {
      manhole_still_exists = true;
    }
    if(it != detected_manholes_.end()) {
      std::shared_ptr<Manhole> mh = detected_manholes_[new_det.id];
      tf::Quaternion quat;
      quat.setEuler(0.0, 0.0, new_det.mean_yaw);
      tf::Vector3 origin(new_det.mean_pos[0], new_det.mean_pos[1], new_det.mean_pos[2]);
      tf::Pose poseTF(quat, origin);
      geometry_msgs::Pose pose;
      tf::poseTFToMsg(poseTF, pose);
      mh->pose = pose;
    }
    else {
      // New manhole
      if(new_det.mean_pos[2] > config_->system_params.max_mh_height) {
        // ROS_WARN("Manhole %d too high (%f)", new_det.id, new_det.mean_pos[2]);
        continue;
      }
      std::shared_ptr<Manhole> mh;
      mh.reset(new Manhole());
      mh->id = new_det.id;
      tf::Quaternion quat;
      quat.setEuler(0.0, 0.0, new_det.mean_yaw);
      tf::Vector3 origin(new_det.mean_pos[0], new_det.mean_pos[1], new_det.mean_pos[2]);
      tf::Pose poseTF(quat, origin);
      geometry_msgs::Pose pose;
      tf::poseTFToMsg(poseTF, pose);
      mh->pose = pose;

      detected_manholes_[mh->id] = mh;
    }
  }

  if(mode == ManholeTraversalMode::kPathCheck)
  {
    if(manhole_still_exists) 
    {
      status = ManholeTraversalStatus::OK;
      return through_path;
    }
    else // If after second detection, the manhole is found to be a false detection, reject it and go to the next closest
    {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "This MH does not exist");
      status = ManholeTraversalStatus::MANHOLE_DOUBLE_CHECK_FAILED;
      return through_path;
    }
  }
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Detections updated");


	if(detected_manholes_.empty()) {
    ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "No Manholes at all");
    status = ManholeTraversalStatus::NO_MANHOLES;
		return through_path;
	}

	std::shared_ptr<Manhole> best_manhole;
	double closest_distance = std::numeric_limits<double>::max();
  bool found = false;
  if(mode == ManholeTraversalMode::kGoingTo) {
    for(auto it : detected_manholes_)
    {
      if(it.second->id == mh_id)
      {
        found = true;
        best_manhole = it.second;
        manhole_under_execution_ = mh_id;
      }
    }
  }
  else {
    best_manhole = detected_manholes_[manhole_under_execution_];
    found = true;
  }

  if(!found) {
    ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "No Appropriate Manhole Found");
    status = ManholeTraversalStatus::NO_MANHOLES;
    return through_path;
  }

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Best mh found: %d", manhole_under_execution_);

  int max_tries = 5;
  std::vector<geometry_msgs::Pose> empty_path;
  

  if(global_verbosity >= Verbosity::DEBUG) {
    std::cout << best_manhole->pose.position.x << " " << best_manhole->pose.position.y << " " << best_manhole->pose.position.z << " | "
              << best_manhole->pose.orientation.x << " " << best_manhole->pose.orientation.y << " " << best_manhole->pose.orientation.z << " " << best_manhole->pose.orientation.w << std::endl;
  }
  
  double direction = tf::getYaw(best_manhole->pose.orientation);
  // std::cout << "Direction: " << direction << std::endl;
  geometry_msgs::Pose p0;
  p0.position.x = best_manhole->pose.position.x - planning_params_.manhole_traversal_path_edge_length * std::cos(direction);
  p0.position.y = best_manhole->pose.position.y - planning_params_.manhole_traversal_path_edge_length * std::sin(direction);
  p0.position.z = best_manhole->pose.position.z;
  p0.orientation = best_manhole->pose.orientation;
  // through_path.push_back(p0);
  // through_path.push_back(best_manhole->pose);
  geometry_msgs::Pose p1;
  p1.position.x = best_manhole->pose.position.x + planning_params_.manhole_traversal_path_edge_length * std::cos(direction);
  p1.position.y = best_manhole->pose.position.y + planning_params_.manhole_traversal_path_edge_length * std::sin(direction);
  p1.position.z = best_manhole->pose.position.z;
  p1.orientation = best_manhole->pose.orientation;
  // through_path.push_back(p1);
  
  // best_manhole->active = false;
  // best_manhole->through_path = through_path;

  geometry_msgs::Pose first_pose;
  if(getDistance(current_state_, p0) < getDistance(current_state_, p1)) {
    first_pose = p0;
  }
  else {
    first_pose = p1;
  }

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Path direction set");
  // ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "MH Mode (later): %d", manhole_traversal_status_);

  geometry_msgs::Quaternion corrected_quat = best_manhole->pose.orientation;

  if(getDistance(current_state_, p0) < getDistance(current_state_, p1)) {
      tf::Quaternion quat;
      double corrected_yaw = std::atan2(p1.position.y - p0.position.y, p1.position.x - p0.position.x);
      quat.setEuler(0.0, 0.0, corrected_yaw);
      tf::quaternionTFToMsg(quat, corrected_quat);
      tf::quaternionTFToMsg(quat, p0.orientation);
      tf::quaternionTFToMsg(quat, best_manhole->pose.orientation);
      tf::quaternionTFToMsg(quat, p1.orientation);
  }
  else {
    tf::Quaternion quat;
    double corrected_yaw = std::atan2(p0.position.y - p1.position.y, p0.position.x - p1.position.x);
    quat.setEuler(0.0, 0.0, corrected_yaw);
    tf::quaternionTFToMsg(quat, corrected_quat);
    tf::quaternionTFToMsg(quat, p0.orientation);
    tf::quaternionTFToMsg(quat, best_manhole->pose.orientation);
    tf::quaternionTFToMsg(quat, p1.orientation);
  }

  tf::Quaternion corrected_quat_tf;
  tf::quaternionMsgToTF(corrected_quat, corrected_quat_tf);
  double corrected_yaw = tf::getYaw(corrected_quat_tf);

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Corrected quat: %f", corrected_yaw);

  if(mode == ManholeTraversalMode::kPassingThrough) {
    if(getDistance(current_state_, p0) < getDistance(current_state_, p1)) {
      p0.orientation = corrected_quat;
      best_manhole->pose.orientation = corrected_quat;
      p1.orientation = corrected_quat;
      through_path.push_back(p0);
      through_path.push_back(best_manhole->pose);
      through_path.push_back(p1);

    }
    else {
      p0.orientation = corrected_quat;
      best_manhole->pose.orientation = corrected_quat;
      p1.orientation = corrected_quat;
      through_path.push_back(p1);
      through_path.push_back(best_manhole->pose);
      through_path.push_back(p0);
    }

    // ROS_WARN_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "Manhole Traversal Path:");
    ROS_WARN_COND(true, "Manhole Traversal Path:");
    if(global_verbosity >= Verbosity::DEBUG) {
    // if(true) {
      std::cout << p0.position.x << " " << p0.position.y << " " << p0.position.z << " " << direction << std::endl;
      std::cout << best_manhole->pose.position.x << " " << best_manhole->pose.position.y << " " << best_manhole->pose.position.z << " " << direction << std::endl;
      std::cout << p1.position.x << " " << p1.position.y << " " << p1.position.z << " " << direction << std::endl;
    }
  }
  else {
    first_pose.orientation = corrected_quat;
    first_pose.position.z += planning_params_.manhole_alignment_z_offset;
    // std::cout << "First Pose z: " << first_pose.position.z << " offset: " << planning_params_.manhole_alignment_z_offset << std::endl;
  }

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Path set 1");

  geometry_msgs::Pose current_pose;
  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, current_state_[3]);
  tf::Vector3 origin(current_state_[0], current_state_[1], current_state_[2]);
  tf::Pose poseTF(quat, origin);
  tf::poseTFToMsg(poseTF, current_pose);

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Path set 2");

  std::vector<geometry_msgs::Pose> connecting_path;
  bool success = search(current_pose, first_pose, false, connecting_path);
  if(success) {
    // connecting_path.back().orientation = best_manhole->pose.orientation;
    connecting_path.push_back(first_pose);
    connecting_path.back().orientation = corrected_quat;
    through_path.insert(through_path.begin(), connecting_path.begin(), connecting_path.end());
  }
  else {
    ROS_ERROR_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "Connecting path not found");
    return empty_path;
  }

  linearlyInterpolateYaw(through_path);

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Path set 3");

  // ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "MH Mode (even later): %d", manhole_traversal_status_);

  visualization_->visualizeManholeTraversalPath(through_path);

  if(mode == ManholeTraversalMode::kPathCheck) {
    status = ManholeTraversalStatus::OK;
    return empty_path;
  } 
  else if(mode == ManholeTraversalMode::kPassingThrough) {
    for(int i=0; i<through_path.size(); ++i) {
      // through_path[i].orientation = best_manhole->pose.orientation;
      through_path[i].orientation = corrected_quat;
    }
    best_manhole->active = false;
    status = ManholeTraversalStatus::OK;
    return through_path;
  }
  else {
    status = ManholeTraversalStatus::OK;
    return through_path;
  }
}


std::vector<geometry_msgs::Pose> Rrg::getManholeTraversalPath(ManholeTraversalMode mode, ManholeTraversalStatus &status) {
	std::vector<geometry_msgs::Pose> through_path;

  std::vector<ManholeDetection> updated_detections;
  // if(!manhole_detector_->getStableManholes(updated_detections)) {
  //   return through_path;
  // }
  manhole_detector_->getStableManholes(updated_detections);

  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "MH Mode: %d", mode);
  
  // if(manhole_traversal_status_ == ManholeTraversalMode::kNone) {
  //   manhole_traversal_status_ = ManholeTraversalMode::kGoingTo;
  // }
  // else if(manhole_traversal_status_ == ManholeTraversalMode::kGoingTo) {
  //   if(planning_params_.auto_manhole_path_approval) {
  //   	manhole_traversal_status_ = ManholeTraversalMode::kPassingThrough;
  //   }
  //   else{
  //   	manhole_traversal_status_ = ManholeTraversalMode::kPathCheck;
  //   }
  // }
  // else if(manhole_traversal_status_ == ManholeTraversalMode::kPathCheck) {
  //   if(planning_params_.auto_manhole_path_approval) {
  //     manhole_passing_approved_ = ManholeApproval::kApproved;
  //   }
  //   if(manhole_passing_approved_ == ManholeApproval::kApproved) {
  //     ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Manhole Traversal Approved");
  //     manhole_passing_approved_ = ManholeApproval::kWaiting;
  //     manhole_traversal_status_ = ManholeTraversalMode::kPassingThrough;
  //   }
  //   else if(manhole_passing_approved_ == ManholeApproval::kRejected) {
  //     ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Manhole Traversal Rejected");
  //     manhole_traversal_status_ = ManholeTraversalMode::kNone;
  //     detected_manholes_[manhole_under_execution_]->active = false;
  //     return through_path;
  //   }
  //   else if(manhole_passing_approved_ == ManholeApproval::kReEvaluate) {
  //     ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Manhole Traversal ReEvaluate");
  //     manhole_traversal_status_ = ManholeTraversalMode::kPathCheck;
  //   }
  // }

  // if(!manhole_detector_->getStableManholes(updated_detections)) {
  //   return through_path;
  // }
  // ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "MH Mode: %d", manhole_traversal_status_);
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "MH under exect: %d", manhole_under_execution_);

  manhole_detector_->getStableManholes(updated_detections);
  bool manhole_still_exists = false;
  // std::cout << "Dets IDs:" << std::endl;
  for(auto new_det : updated_detections) {
    auto it = detected_manholes_.find(new_det.id);
    // std::cout << new_det.id << std::endl;
    if((mode == ManholeTraversalMode::kPathCheck) && 
      new_det.id == manhole_under_execution_) {
      manhole_still_exists = true;
    }
    if(it != detected_manholes_.end()) {
      std::shared_ptr<Manhole> mh = detected_manholes_[new_det.id];
      tf::Quaternion quat;
      quat.setEuler(0.0, 0.0, new_det.mean_yaw);
      tf::Vector3 origin(new_det.mean_pos[0], new_det.mean_pos[1], new_det.mean_pos[2]);
      tf::Pose poseTF(quat, origin);
      geometry_msgs::Pose pose;
      tf::poseTFToMsg(poseTF, pose);
      mh->pose = pose;
    }
    else {
      // New manhole
      if(new_det.mean_pos[2] > planning_params_.max_manhole_height) {
        ROS_WARN("Manhole %d too high (%f)", new_det.id, new_det.mean_pos[2]);
        continue;
      }
      std::shared_ptr<Manhole> mh;
      mh.reset(new Manhole());
      mh->id = new_det.id;
      tf::Quaternion quat;
      quat.setEuler(0.0, 0.0, new_det.mean_yaw);
      tf::Vector3 origin(new_det.mean_pos[0], new_det.mean_pos[1], new_det.mean_pos[2]);
      tf::Pose poseTF(quat, origin);
      geometry_msgs::Pose pose;
      tf::poseTFToMsg(poseTF, pose);
      mh->pose = pose;

      detected_manholes_[mh->id] = mh;
    }
  }

  if(mode == ManholeTraversalMode::kPathCheck)
  {
    if(manhole_still_exists) 
    {
      status = ManholeTraversalStatus::OK;
      return through_path;
    }
    else // If after second detection, the manhole is found to be a false detection, reject it and go to the next closest
    {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "This MH does not exist");
      status = ManholeTraversalStatus::MANHOLE_DOUBLE_CHECK_FAILED;
      return through_path;
    }
  }
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Detections updated");


	if(detected_manholes_.empty()) {
    ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "No Manholes at all");
    status = ManholeTraversalStatus::NO_MANHOLES;
		return through_path;
	}

	std::shared_ptr<Manhole> best_manhole;
	double closest_distance = std::numeric_limits<double>::max();
  bool found = false;
  if(mode == ManholeTraversalMode::kGoingTo) {
  // if(mode != ManholeTraversalMode::kPassingThrough && mode != ManholeTraversalMode::kPathCheck) {
    for(auto it : detected_manholes_) {
      std::shared_ptr<Manhole> current_manhole = it.second;
      if(!current_manhole->active) {
        /* TODO: Update active status of all the semantics based on this */
        continue;
      }
      
      if(planning_params_.exploration_only)  // If exploration only, go to the closest manhole
      {
        double mh_robot_dist = (current_state_.head(3) 
              - Eigen::Vector3d(current_manhole->pose.position.x, 
                                current_manhole->pose.position.y, 
                                current_manhole->pose.position.z)).norm();
        if(mh_robot_dist < closest_distance) {
          closest_distance = mh_robot_dist;
          best_manhole = current_manhole;
          found = true;
        }
      }
      else // If exploration + inspection, find the manhole that leads to the next compartment
      {
        double mh_robot_dist = (planning_params_.compartment_centers[next_compartment_index_-1] - Eigen::Vector3d(current_manhole->pose.position.x, current_manhole->pose.position.y, current_manhole->pose.position.z)).norm();
        Eigen::Vector3d compartment_dim = planning_params_.compartment_dimensions.max_val - planning_params_.compartment_dimensions.min_val;
        if(mh_robot_dist > compartment_dim.norm()/2.0)
        {
          continue;
        }
        double dist;
        if(next_compartment_.x() < std::numeric_limits<double>::max())
          dist = (next_compartment_ - Eigen::Vector3d(current_manhole->pose.position.x, current_manhole->pose.position.y, current_manhole->pose.position.z)).norm();
        else
          dist = (current_state_.head(3) - Eigen::Vector3d(current_manhole->pose.position.x, current_manhole->pose.position.y, current_manhole->pose.position.z)).norm();
        if(dist < closest_distance) {
          closest_distance = dist;
          best_manhole = current_manhole;
          found = true;
        }
      }
    }
    if(found)
      manhole_under_execution_ = best_manhole->id;
  }
  else {
    best_manhole = detected_manholes_[manhole_under_execution_];
    found = true;
  }

  if(!found) {
    ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "No Appropriate Manhole Found");
    status = ManholeTraversalStatus::NO_MANHOLES;
    return through_path;
  }

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Best mh found: %d", manhole_under_execution_);

  int max_tries = 5;
  std::vector<geometry_msgs::Pose> empty_path;
  

  if(global_verbosity >= Verbosity::DEBUG) {
    std::cout << best_manhole->pose.position.x << " " << best_manhole->pose.position.y << " " << best_manhole->pose.position.z << " | "
              << best_manhole->pose.orientation.x << " " << best_manhole->pose.orientation.y << " " << best_manhole->pose.orientation.z << " " << best_manhole->pose.orientation.w << std::endl;
  }
  
  double direction = tf::getYaw(best_manhole->pose.orientation);
  // std::cout << "Direction: " << direction << std::endl;
  geometry_msgs::Pose p0;
  p0.position.x = best_manhole->pose.position.x - planning_params_.manhole_traversal_path_edge_length * std::cos(direction);
  p0.position.y = best_manhole->pose.position.y - planning_params_.manhole_traversal_path_edge_length * std::sin(direction);
  p0.position.z = best_manhole->pose.position.z;
  p0.orientation = best_manhole->pose.orientation;
  // through_path.push_back(p0);
  // through_path.push_back(best_manhole->pose);
  geometry_msgs::Pose p1;
  p1.position.x = best_manhole->pose.position.x + planning_params_.manhole_traversal_path_edge_length * std::cos(direction);
  p1.position.y = best_manhole->pose.position.y + planning_params_.manhole_traversal_path_edge_length * std::sin(direction);
  p1.position.z = best_manhole->pose.position.z;
  p1.orientation = best_manhole->pose.orientation;
  // through_path.push_back(p1);
  
  // best_manhole->active = false;
  // best_manhole->through_path = through_path;

  geometry_msgs::Pose first_pose;
  if(getDistance(current_state_, p0) < getDistance(current_state_, p1)) {
    first_pose = p0;
  }
  else {
    first_pose = p1;
  }

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Path direction set");
  // ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "MH Mode (later): %d", manhole_traversal_status_);

  geometry_msgs::Quaternion corrected_quat = best_manhole->pose.orientation;

  if(getDistance(current_state_, p0) < getDistance(current_state_, p1)) {
      tf::Quaternion quat;
      double corrected_yaw = std::atan2(p1.position.y - p0.position.y, p1.position.x - p0.position.x);
      quat.setEuler(0.0, 0.0, corrected_yaw);
      tf::quaternionTFToMsg(quat, corrected_quat);
      tf::quaternionTFToMsg(quat, p0.orientation);
      tf::quaternionTFToMsg(quat, best_manhole->pose.orientation);
      tf::quaternionTFToMsg(quat, p1.orientation);
  }
  else {
    tf::Quaternion quat;
    double corrected_yaw = std::atan2(p0.position.y - p1.position.y, p0.position.x - p1.position.x);
    quat.setEuler(0.0, 0.0, corrected_yaw);
    tf::quaternionTFToMsg(quat, corrected_quat);
    tf::quaternionTFToMsg(quat, p0.orientation);
    tf::quaternionTFToMsg(quat, best_manhole->pose.orientation);
    tf::quaternionTFToMsg(quat, p1.orientation);
  }

  tf::Quaternion corrected_quat_tf;
  tf::quaternionMsgToTF(corrected_quat, corrected_quat_tf);
  double corrected_yaw = tf::getYaw(corrected_quat_tf);

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Corrected quat: %f", corrected_yaw);

  if(mode == ManholeTraversalMode::kPassingThrough) {
    if(getDistance(current_state_, p0) < getDistance(current_state_, p1)) {
      p0.orientation = corrected_quat;
      best_manhole->pose.orientation = corrected_quat;
      p1.orientation = corrected_quat;
      through_path.push_back(p0);
      through_path.push_back(best_manhole->pose);
      through_path.push_back(p1);

    }
    else {
      p0.orientation = corrected_quat;
      best_manhole->pose.orientation = corrected_quat;
      p1.orientation = corrected_quat;
      through_path.push_back(p1);
      through_path.push_back(best_manhole->pose);
      through_path.push_back(p0);
    }

    // ROS_WARN_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "Manhole Traversal Path:");
    ROS_WARN_COND(true, "Manhole Traversal Path:");
    if(global_verbosity >= Verbosity::DEBUG) {
    // if(true) {
      std::cout << p0.position.x << " " << p0.position.y << " " << p0.position.z << " " << direction << std::endl;
      std::cout << best_manhole->pose.position.x << " " << best_manhole->pose.position.y << " " << best_manhole->pose.position.z << " " << direction << std::endl;
      std::cout << p1.position.x << " " << p1.position.y << " " << p1.position.z << " " << direction << std::endl;
    }
  }
  else {
    first_pose.orientation = corrected_quat;
    first_pose.position.z += planning_params_.manhole_alignment_z_offset;
    // std::cout << "First Pose z: " << first_pose.position.z << " offset: " << planning_params_.manhole_alignment_z_offset << std::endl;
  }

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Path set 1");

  geometry_msgs::Pose current_pose;
  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, current_state_[3]);
  tf::Vector3 origin(current_state_[0], current_state_[1], current_state_[2]);
  tf::Pose poseTF(quat, origin);
  tf::poseTFToMsg(poseTF, current_pose);

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Path set 2");

  std::vector<geometry_msgs::Pose> connecting_path;
  bool success = search(current_pose, first_pose, false, connecting_path);
  if(success) {
    // connecting_path.back().orientation = best_manhole->pose.orientation;
    connecting_path.push_back(first_pose);
    connecting_path.back().orientation = corrected_quat;
    through_path.insert(through_path.begin(), connecting_path.begin(), connecting_path.end());
  }
  else {
    ROS_ERROR_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "Connecting path not found");
    return empty_path;
  }

  linearlyInterpolateYaw(through_path);

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Path set 3");

  // ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "MH Mode (even later): %d", manhole_traversal_status_);

  visualization_->visualizeManholeTraversalPath(through_path);

  if(mode == ManholeTraversalMode::kPathCheck) {
    status = ManholeTraversalStatus::OK;
    return empty_path;
  } 
  else if(mode == ManholeTraversalMode::kPassingThrough) {
    for(int i=0; i<through_path.size(); ++i) {
      // through_path[i].orientation = best_manhole->pose.orientation;
      through_path[i].orientation = corrected_quat;
    }
    best_manhole->active = false;
    manholes_traversed_.push_back(best_manhole->id);
    status = ManholeTraversalStatus::OK;
    return through_path;
  }
  else {
    status = ManholeTraversalStatus::OK;
    return through_path;
  }

  

}

std::vector<geometry_msgs::Pose> Rrg::reRunGlobalPlanner(int &status) {
  return runGlobalPlanner(current_global_vertex_id_, true, true, status);
}

std::vector<geometry_msgs::Pose> Rrg::runGlobalPlanner(int vertex_id,
                                                       bool not_check_frontier,
                                                       bool ignore_time,
                                                       int &status) {
  // @not_check_frontier: just check if it is feasible (collision-free + time)
  // @ignore_time: don't consider time budget.

  visualization_->visualizeGlobalGraph(global_graph_);

  // Check if exists any frontier in the global graph
  // Get the list of current frontiers.
  //
  ROS_INFO_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "Global planner triggered");
  if (vertex_id) {
    ros::Duration(3.0).sleep();  // sleep to unblock the thread to get and
                                 // update all latest pose update.
    ros::spinOnce();
  }
  
  
  status = planner_msgs::planner_srv::Response::kRepositioning;

  if(!vertex_id && planning_params_.enable_manhole_traversal && planning_params_.exploration_only)  // Don't do manhole traversal if asked to go to a specific vertex
  {
    status = -1;
		return getManholeTraversalPath();	
  }

  START_TIMER(ttime);
  std::vector<geometry_msgs::Pose> ret_path;
  ret_path.clear();

  ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "1");

  // Check if the global planner exists
  if (global_graph_->getNumVertices() <= 1) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[GlobalGraph] Graph is empty, nothing to search.");
    return ret_path;
  }

  ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "2");

  // Check if the vertex id exists
  if ((vertex_id < 0) || (vertex_id >= global_graph_->getNumVertices())) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, 
        "[GlobalGraph] GbplannerVertex ID doesn't exist, plz consider IDs in the range "
        "[0-%d].",
        global_graph_->getNumVertices() - 1);
    return ret_path;
  }

  ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "3");

  // Check if the time endurance is still available.
  if (!ignore_time) {
    if (getTimeRemained() <= 0.0) {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[Global] RAN OUT OF TIME --> STOP HERE.");
      return ret_path;
    }
  }

  ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "4");

  // Check if exists any frontiers in the graph.
  // Re-update all the frontiers based on the volumetric gain.
  std::vector<GbplannerVertex*> global_frontiers;
  int num_vertices = global_graph_->getNumVertices();
  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Re-check all frontiers.");
  global_frontiers.clear();
  for (int id = 0; id < num_vertices; ++id) {
    if (global_graph_->getVertex(id)->type == VertexType::kFrontier) {
      GbplannerVertex* v = global_graph_->getVertex(id);
      computeVolumetricGainRayModelNoBound(v->state, v->vol_gain);
      if (!v->vol_gain.is_frontier)
        v->type = VertexType::kUnvisited;
      else
        global_frontiers.push_back(global_graph_->getVertex(id));
    }
  }
  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Currently have %d frontiers in the global graph.",
           (int)global_frontiers.size());
  if ((!not_check_frontier) && (global_frontiers.size() <= 0)) {
    ROS_WARN_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "No frontier exists");
    // Keep exploring or go home if no more frontiers
    if (planning_params_.go_home_if_fully_explored) {
      ROS_WARN_COND(global_verbosity >= Verbosity::PLANNER_STATUS, " --> Calling HOMING instead.");
      ret_path = getHomingPath(world_frame_);
      homing_engaged_ = true;
    } else {
      num_low_gain_iters_ = 0;
    }
    status = planner_msgs::planner_srv::Response::kHoming;
    return ret_path;
  }

  ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "5");

  // Let's try to add current state to the global graph.
  ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Trying to add new vertex from current position.");
  StateVec cur_state;
  cur_state << current_state_[0], current_state_[1], current_state_[2],
      current_state_[3];
  cur_state[2] -=
      (planning_params_.robot_height - planning_params_.max_ground_height);
  GbplannerVertex* link_vertex = NULL;
  const double kRadiusLimit = 1.5;  // 0.5
  bool connected_to_graph =
      connectStateToGraph(global_graph_, cur_state, link_vertex, kRadiusLimit);

  if (!connected_to_graph) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Cannot add the state to the global graph.");
    return ret_path;
  }

  ROS_WARN_COND(global_verbosity >= Verbosity::WARN, 
      "Added current state to the graph. Start searching for the global path "
      "now.");
  // Get Dijsktra path from home to all.
  if (!global_graph_->findShortestPaths(global_graph_rep_)) {
    ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "[GlobalGraph] Failed to find shortest path.");
    return ret_path;
  }
  // Get Dijsktra path from current to all.
  ShortestPathsReport frontier_graph_rep;
  if (!global_graph_->findShortestPaths(link_vertex->id, frontier_graph_rep)) {
    ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "[GlobalGraph] Failed to find shortest path.");
    return ret_path;
  }
  // Check if the planner should find the best vertex automatically or manually
  double best_gain = -1.0;
  GbplannerVertex* best_frontier = NULL;

  if (vertex_id) {
    // Manual mode
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[Global Planner] Manual Mode");
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
      ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "[Global] Time remaining: %f (sec)", time_remaining);
      time_cost += time_to_target;
      ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "[Global] Time to [%3d]: %f (sec)", vertex_id, time_to_target);
      if (planning_params_.auto_homing_enable) {
        time_cost += time_to_home;
        ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "[Global] Time to home  : %f (sec)", time_to_home);
      }
    }
    double time_spare = 0;
    if (!isRemainingTimeSufficient(time_cost, time_spare)) {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[Global] Not enough time to go the vertex [%d]", vertex_id);
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, 
          "[Global] Consider change to another ID or set ignore_time to True");
      return ret_path;
    }
    best_frontier = global_graph_->getVertex(vertex_id);
    best_gain = 1.0;
  } else {
    // Auto mode
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[Global Planner] Auto mode");
    // Get list of feasible frontiers by checking remaining time.
    // Leave the check empty for now since it relate to time budget setting.
    std::vector<GbplannerVertex*> feasible_global_frontiers;
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
    ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Get %d feasible frontiers from global frontiers.",
             (int)feasible_global_frontiers.size());
    if (feasible_global_frontiers.size() <= 0) {
      ROS_INFO_COND(global_verbosity >= Verbosity::INFO, 
          "No feasible frontier exists --> Call HOMING instead if fully "
          "explored.");
      return ret_path;
    }

    // Compute exploration gain.
    std::unordered_map<int, double> frontier_exp_gain;
    for (int i = 0; i < feasible_global_frontiers.size(); ++i) {
      GbplannerVertex* f = feasible_global_frontiers[i];
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
              [&frontier_exp_gain](const GbplannerVertex* a, const GbplannerVertex* b) {
                return frontier_exp_gain[a->id] > frontier_exp_gain[b->id];
              });
    // Print out
    // ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "List of potential frontier in decreasing order of gain:");
    // for (int i = 0; i < feasible_global_frontiers.size(); ++i) {
    //   ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "ID [%d]: %d with gain %f", i,
    //   feasible_global_frontiers[i]->id,
    //   frontier_exp_gain[feasible_global_frontiers[i]->id]);
    // }
  }

  std::vector<int> current_to_frontier_path_id;
  std::vector<int> frontier_to_home_path_id;
  if (best_gain >= 0) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Found the best frontier to go is: %d", best_frontier->id);

    if (auto_global_planner_trig_) {
      current_global_vertex_id_ = best_frontier->id;
      global_exploration_ongoing_ = true;
    }

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
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, 
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
    if (improveFreePath(ret_path, mod_path, true)) {
      ret_path = mod_path;
    }
    double dmod_time = GET_ELAPSED_TIME(mod_time);
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Compute an aternate path for homing in %f(s)", dmod_time);
    visualization_->visualizeModPath(mod_path);
  }

  visualization_->visualizeGlobalPaths(
      global_graph_, current_to_frontier_path_id, frontier_to_home_path_id);

  double dtime = GET_ELAPSED_TIME(ttime);
  ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "runGlobalPlanner costs: %f (s)", dtime);

  // Path interpolation:
  const double kInterpolationDistance =
      planning_params_.path_interpolation_distance;
  std::vector<geometry_msgs::Pose> interp_path;
  if (Trajectory::interpolatePath(ret_path, kInterpolationDistance,
                                  interp_path)) {
    ret_path = interp_path;
  }

  
  visualization_->visualizeRefPath(ret_path);
  return ret_path;
}

void Rrg::addGeofenceAreas(const geometry_msgs::PolygonStamped& polygon_msgs) {
  if ((planning_params_.geofence_checking_enable) && (planner_trigger_count_)) {
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
        ROS_WARN_COND(global_verbosity >= Verbosity::WARN, 
            "Could not look up TF from polygon frame [%s] to the global frame "
            "[%s].",
            polygon_msgs.header.frame_id.c_str(),
            planning_params_.global_frame_id.c_str());
      }
    }
  }
}

void Rrg::clearUntraversableZones() {
  geofence_manager_.reset(new GeofenceManager());
  ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Clear all the traversable polygons.");
}

void Rrg::setGlobalFrame(std::string frame_id) {
  // Mainly use to set the frame_id for visualization.
  if (!frame_id.empty()) visualization_->setGlobalFrame(frame_id);
}

bool Rrg::isPathCollisionFree(const std::vector<geometry_msgs::Pose>& path,
                              const Eigen::Vector3d& robot_size) {
  if (path.empty()) return true;  // nothing to check

  Eigen::Vector3d voxel(path[0].position.x, path[0].position.y,
                        path[0].position.z);
  if (path.size() == 1) {
    if (MapManager::VoxelStatus::kFree ==
        map_manager_->getBoxStatus(voxel, robot_size, true)) {
      return true;
    } else {
      return false;
    }
  }

  for (int i = 0; i < (path.size() - 1); ++i) {
    Eigen::Vector3d start_point(path[i].position.x, path[i].position.y,
                                path[i].position.z);
    Eigen::Vector3d end_point(path[i + 1].position.x, path[i + 1].position.y,
                              path[i + 1].position.z);
    if (MapManager::VoxelStatus::kFree !=
        map_manager_->getPathStatus(start_point, end_point, robot_size, true)) {
      return false;
    }
  }
  return true;
}

bool Rrg::searchPathThroughCenterPoint(const StateVec& current_state,
                                       const Eigen::Vector3d& center,
                                       const double& heading,
                                       Eigen::Vector3d& robot_size,
                                       std::vector<geometry_msgs::Pose>& path) {
  path.clear();

  // Start searching a line to pass through
  double L_line = darpa_gate_params_.line_search_length;
  double rotation_step = darpa_gate_params_.line_search_step;
  int n_lines = (int)(darpa_gate_params_.line_search_range / rotation_step);
  geometry_msgs::Pose current_pose;
  convertStateToPoseMsg(current_state, current_pose);
  geometry_msgs::Pose start_pose;
  geometry_msgs::Pose end_pose;

  Eigen::Vector3d start_point;
  Eigen::Vector3d end_point;

  for (int i = 0; i < n_lines; ++i) {
    // Positive direction.
    double heading_sample = heading + i * rotation_step;
    Eigen::Vector3d u_vec(std::cos(heading_sample), std::sin(heading_sample),
                          0);
    start_point = center - u_vec * 0.5 * L_line;
    end_point = center + u_vec * 0.5 * L_line;
    StateVec start_state(start_point.x(), start_point.y(), start_point.z(),
                         0.0);
    StateVec end_state(end_point.x(), end_point.y(), end_point.z(), 0.0);
    convertStateToPoseMsg(start_state, start_pose);
    convertStateToPoseMsg(end_state, end_pose);
    path.clear();
    path.push_back(current_pose);
    path.push_back(start_pose);
    path.push_back(end_pose);
    if (isPathCollisionFree(path, robot_size)) {
      if(global_verbosity >= Verbosity::INFO) {
        std::cout << "Best voxel to go: " << center << std::endl;
        std::cout << "With robot size: " << robot_size << std::endl;
      }
      return true;
    }

    if (i > 0) {
      // Negative direction.
      heading_sample = heading - i * rotation_step;
      u_vec << std::cos(heading_sample), std::sin(heading_sample), 0;
      start_point = center - u_vec * 0.5 * L_line;
      end_point = center + u_vec * 0.5 * L_line;
      start_state << start_point.x(), start_point.y(), start_point.z(), 0.0;
      end_state << end_point.x(), end_point.y(), end_point.z(), 0.0;
      convertStateToPoseMsg(start_state, start_pose);
      convertStateToPoseMsg(end_state, end_pose);
      path.clear();
      path.push_back(current_pose);
      path.push_back(start_pose);
      path.push_back(end_pose);
      if (isPathCollisionFree(path, robot_size)) {
        if(global_verbosity >= Verbosity::INFO) {
          std::cout << "Best voxel to go: " << center << std::endl;
          std::cout << "With robot size: " << robot_size << std::endl;
        }
        return true;
      }
    }
  }

  return false;
}

std::vector<geometry_msgs::Pose> Rrg::searchPathToPassGate() {
  std::vector<geometry_msgs::Pose> ret_path;

  // Check if this is allowed.
  if (!darpa_gate_params_.enable) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Not allow to run search to pass the gate");
    return ret_path;
  }

  // Initial guess for the center.
  // Load default from yaml file first.
  Eigen::Vector3d gate_center = darpa_gate_params_.center;
  double gate_heading = darpa_gate_params_.heading;
  // Check if loading intial guess from tag detection or yaml file.
  if (darpa_gate_params_.load_from_darpa_frame) {
    // Look up the tf.
    tf::StampedTransform tfW2G;
    tf::TransformListener listener;
    try {
      listener.waitForTransform(darpa_gate_params_.world_frame_id,
                                darpa_gate_params_.gate_center_frame_id,
                                ros::Time(0), ros::Duration(1.0));
      listener.lookupTransform(darpa_gate_params_.world_frame_id,
                               darpa_gate_params_.gate_center_frame_id,
                               ros::Time(0), tfW2G);
      tf::Vector3 vec = tfW2G.getOrigin();
      gate_center << vec.x(), vec.y(), vec.z();
      if(global_verbosity >= Verbosity::INFO) {
        std::cout << "Darpa gate offset: "
                  << darpa_gate_params_.darpa_frame_offset << std::endl;
      }
      gate_center = gate_center + darpa_gate_params_.darpa_frame_offset;
      // don't use heading for now.
      // gate_heading = tf::getYaw(tfW2G.getRotation());
      if(global_verbosity >= Verbosity::INFO) {
        std::cout << "Darpa gate center: " << gate_center << std::endl;
      }
    } catch (tf::TransformException ex) {
      ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "%s", ex.what());
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Could not look up TF for gate center.");
      return ret_path;
    }
  }

  double search_radius = darpa_gate_params_.center_search_radius;
  double search_step = darpa_gate_params_.center_search_step;
  // Start searching from the biggest to smallest box.
  int loop_max = (int)(search_radius / search_step);
  Eigen::Vector3d best_voxel;
  Eigen::Vector3d bbx_size;
  bool stop = false;
  for (int bound_level = 0; (!stop) && (bound_level < 3); ++bound_level) {
    robot_params_.setBoundMode((BoundModeType)bound_level);
    // Update the robot size for planning.
    robot_params_.getPlanningSize(robot_box_size_);
    bbx_size = robot_box_size_;
    for (double loop = 0; (!stop) && (loop < loop_max); ++loop) {
      // Start checking each location, stop when detected collision free.
      if (loop == 0) {
        Eigen::Vector3d voxel;
        voxel = gate_center;
        if ((MapManager::VoxelStatus::kFree ==
             map_manager_->getBoxStatus(voxel, bbx_size, true)) &&
            searchPathThroughCenterPoint(current_state_, voxel, gate_heading,
                                         bbx_size, ret_path)) {
          stop = true;
          break;
        }
      } else {
        // Scan along z.
        for (int i = -loop + 1; i < loop; ++i) {
          Eigen::Vector3d voxel;
          voxel.x() = gate_center.x();
          voxel.y() = gate_center.y() + loop * search_step;
          voxel.z() = gate_center.z() + i * search_step;
          if ((MapManager::VoxelStatus::kFree ==
               map_manager_->getBoxStatus(voxel, bbx_size, true)) &&
              searchPathThroughCenterPoint(current_state_, voxel, gate_heading,
                                           bbx_size, ret_path)) {
            stop = true;
            break;
          }
          voxel.x() = gate_center.x();
          voxel.y() = gate_center.y() - loop * search_step;
          voxel.z() = gate_center.z() + i * search_step;
          if ((MapManager::VoxelStatus::kFree ==
               map_manager_->getBoxStatus(voxel, bbx_size, true)) &&
              searchPathThroughCenterPoint(current_state_, voxel, gate_heading,
                                           bbx_size, ret_path)) {
            stop = true;
            break;
          }
        }
        // Scan along y.
        for (int i = -loop; i <= loop; ++i) {
          Eigen::Vector3d voxel;
          voxel.x() = gate_center.x();
          voxel.y() = gate_center.y() + i * search_step;
          voxel.z() = gate_center.z() + loop * search_step;
          if ((MapManager::VoxelStatus::kFree ==
               map_manager_->getBoxStatus(voxel, bbx_size, true)) &&
              searchPathThroughCenterPoint(current_state_, voxel, gate_heading,
                                           bbx_size, ret_path)) {
            stop = true;
            break;
          }
          voxel.x() = gate_center.x();
          voxel.y() = gate_center.y() + i * search_step;
          voxel.z() = gate_center.z() - loop * search_step;
          if ((MapManager::VoxelStatus::kFree ==
               map_manager_->getBoxStatus(voxel, bbx_size, true)) &&
              searchPathThroughCenterPoint(current_state_, voxel, gate_heading,
                                           bbx_size, ret_path)) {
            stop = true;
            break;
          }
        }
      }
    }
  }

  if (stop) {
    // Consider this is homing position.
    setHomingPos();
    // Add this path to the global graph.
    if (global_graph_->getNumVertices()) {
      std::vector<GbplannerVertex*> vertices_to_add;
      for (int i = 0; i < ret_path.size(); ++i) {
        StateVec st(ret_path[i].position.x, ret_path[i].position.y,
                    ret_path[i].position.z, 0);
        GbplannerVertex* ver =
            new GbplannerVertex(i, st);  // temporary id to generate vertex list.
        ver->is_leaf_vertex = false;
        vertices_to_add.push_back(ver);
      }
      addRefPathToGraph(global_graph_, vertices_to_add);
      visualization_->visualizeGlobalGraph(global_graph_);
    }
    visualization_->visualizeRefPath(ret_path);
  } else {
    ret_path.clear();
  }

  if (ret_path.empty()) ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Could not find path to go through.");
  return ret_path;
}

void Rrg::cameraAnnotationTimerCallback(const ros::TimerEvent& event) {
  if(!planning_params_.annotate_map_with_camera) return;

  // ROS_WARN("Camera annotation enabled");

  for(auto sensor : camera_annotation_params_.sensor_list) {
    std::vector<Eigen::Vector3d> multiray_endpoints;
    camera_annotation_params_.sensor[sensor].getFrustumEndpoints(current_state_, multiray_endpoints);
    // std::cout << "Number of rays: " << multiray_endpoints.size() << std::endl;
    Eigen::Vector3d current_pos = current_state_.head(3);
    map_manager_->annotateCameraVoxels(current_pos, multiray_endpoints);
  }
}

void Rrg::annotateSemanticPredictions(std::vector<Eigen::Vector3d> points)
{
  for(auto pt : points)  map_manager_->annotateVoxel(pt, 1);
}

void Rrg::annotateSemanticPredictions(std::shared_ptr<SSGManager> predicted_graph)
{
  std::vector<Eigen::Vector3d> points_to_annotate;
  for(auto v_it : predicted_graph->vertices_map_)
  {
    if(v_it.second->label == L_LONG)
    {
      // LongInstance* l_inst = std::any_cast<LongInstance*>(v_it.second->metadata);
      Longitudinal l;

      // double min_dist = 999, max_dist = -999;
      // for(auto pt : l_inst->line.points->points)
      // {
      //   Eigen::Vector3d pt_vec(pt.x, pt.y, pt.z);
      //   pt_vec -= l_inst->line.center;
      //   double dist = pt_vec.dot(l_inst->line.direction.normalized());
      //   if(dist > max_dist)
      //     max_dist = dist;
      //   if(dist < min_dist)
      //     min_dist = dist;
      // }
      l.center = v_it.second->state.head(3);
      Eigen::Vector3d direction(std::cos(v_it.second->state[5]), std::sin(v_it.second->state[5]), 0.0);
      l.end_point1 = l.center - direction * (v_it.second->bbox(0)/2.0 - planning_params_.inspection_pt_dist); // min_dist
      l.end_point2 = l.center + direction * (v_it.second->bbox(0)/2.0 - planning_params_.inspection_pt_dist); // max_dist
      // std::cout << "Long " << v_it.first << " c: " << l.center.transpose()
      //           << " ep1: " << l.end_point1.transpose()
      //           << " ep2: " << l.end_point2.transpose() << std::endl;

      for(double dx=-semantic_annotation_half_width_; dx<=semantic_annotation_half_width_; dx+=0.2)
      {
        for(double dy=-0.2; dy<=0.2; dy+=0.2)
        {
          for(double dz=-0.2; dz<=0.2; dz+=0.2)
          {
            // points_to_annotate.push_back(l.center + Eigen::Vector3d(dx, dy, dz));
            points_to_annotate.push_back(l.end_point1 + Eigen::Vector3d(dx, dy, dz));
            points_to_annotate.push_back(l.end_point2 + Eigen::Vector3d(dx, dy, dz));
          }
        }
      }
    }
  }
  // std::cout << "All longs added processed" << std::endl;

  annotateSemanticPredictions(points_to_annotate);
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




// }  // namespace explorer
