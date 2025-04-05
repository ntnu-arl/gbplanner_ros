#include "gbplanner/gbplanner.h"

#include <nav_msgs/Path.h>

// namespace explorer {

Gbplanner::Gbplanner(const ros::NodeHandle& nh,
                     const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  planner_status_ = Gbplanner::PlannerStatus::NOT_READY;

  rrg_ = new Rrg(nh, nh_private);
  if (!(rrg_->loadParams(false))) {
    ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "Could not load all required parameters. Shutdown ROS node.");
    ros::shutdown();
  }

  initializeAttributes();
}

Gbplanner::Gbplanner(const ros::NodeHandle& nh,
                     const ros::NodeHandle& nh_private,
                     MapManager* map_manager)
    : nh_(nh), nh_private_(nh_private) {
  
  planner_status_ = Gbplanner::PlannerStatus::NOT_READY;
  rrg_ = new Rrg(nh, nh_private, map_manager);

  if (!(rrg_->loadParams(true))) {
    ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "Could not load all required parameters. Shutdown ROS node.");
    ros::shutdown();
  }

  initializeAttributes();
}

void Gbplanner::initializeAttributes() {
  planner_service_ = nh_.advertiseService(
      "gbplanner", &Gbplanner::plannerServiceCallback, this);
  global_planner_service_ = nh_.advertiseService(
      "gbplanner/global", &Gbplanner::globalPlannerServiceCallback, this);
  planner_homing_service_ = nh_.advertiseService(
      "gbplanner/homing", &Gbplanner::homingServiceCallback, this);
  planner_set_homing_pos_service_ =
      nh_.advertiseService("gbplanner/set_homing_pos",
                           &Gbplanner::setHomingPosServiceCallback, this);
  planner_search_service_ = nh_.advertiseService(
      "gbplanner/search", &Gbplanner::plannerSearchServiceCallback, this);
  planner_passing_gate_service_ = nh_.advertiseService(
      "gbplanner/passing_gate", &Gbplanner::passingGateCallback, this);
  planner_set_global_bound_service_ = nh_.advertiseService(
      "gbplanner/set_global_bound", &Gbplanner::setGlobalBound, this);
  planner_set_dynamic_global_bound_service_ =
      nh_.advertiseService("gbplanner/set_dynamic_global_bound",
                           &Gbplanner::setDynamicGlobalBound, this);
  planner_clear_untraversable_zones_service_ =
      nh_.advertiseService("gbplanner/clear_untraversable_zones",
                           &Gbplanner::clearUntraversableZones, this);
  planner_load_graph_service_ = nh_.advertiseService(
      "gbplanner/load_graph", &Gbplanner::plannerLoadGraphCallback, this);
  planner_save_graph_service_ = nh_.advertiseService(
      "gbplanner/save_graph", &Gbplanner::plannerSaveGraphCallback, this);
  planner_goto_wp_service_ =
      nh_.advertiseService("gbplanner/go_to_waypoint",
                           &Gbplanner::plannerGotoWaypointCallback, this);
  planner_enable_untraversable_polygon_subscriber_service_ =
      nh_.advertiseService(
          "gbplanner/enable_untraversable_polygon_subscriber",
          &Gbplanner::plannerEnableUntraversablePolygonSubscriberCallback,
          this);
  planner_set_planning_trigger_mode_service_ = nh_.advertiseService(
      "gbplanner/set_planning_trigger_mode",
      &Gbplanner::plannerSetPlanningTriggerModeCallback, this);
      
  inspection_path_service_ = nh_.advertiseService(
      "gbplanner/get_inspection_path",
      &Gbplanner::inspectionServiceCallback, this);

  force_compartment_transition_service_ = nh_.advertiseService(
      "gbplanner/force_compartment_transition",
      &Gbplanner::forceCompartmentChangeServiceCallback, this);

  pose_subscriber_ = nh_.subscribe("pose", 100, &Gbplanner::poseCallback, this);
  pose_stamped_subscriber_ =
      nh_.subscribe("pose_stamped", 100, &Gbplanner::poseStampedCallback, this);
  odometry_subscriber_ =
      nh_.subscribe("odometry", 100, &Gbplanner::odometryCallback, this);
  robot_status_subcriber_ =
      nh_.subscribe("/robot_status", 1, &Gbplanner::robotStatusCallback, this);
  untraversable_polygon_subscriber_ =
      nh_.subscribe("/traversability_estimation/untraversable_polygon", 100,
                    &Gbplanner::untraversablePolygonCallback, this);


  std::string ns = ros::this_node::getName();
  planning_params_.loadParams(ns + "/PlanningParams");

  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Compartment Centers:");
  if(global_verbosity >= Verbosity::DEBUG) {
    for(auto c : planning_params_.compartment_centers) {
      std::cout << c.transpose() << std::endl;
    }
  }
  
  planner_mode_ = PlannerMode::kExploration;
  
  BoundedSpaceParams rrg_global_bounds;
  rrg_->getGlobalBoundParams(rrg_global_bounds);
  rrg_->setExplorationAndInspectionBounds(rrg_global_bounds, rrg_global_bounds);
}

bool Gbplanner::inspectionServiceCallback(
    planner_msgs::planner_srv::Request& req,
    planner_msgs::planner_srv::Response& res) {
  //
  res.path = rrg_->getInspectionPath();
  return true;
}

bool Gbplanner::forceCompartmentChangeServiceCallback(
              std_srvs::Trigger::Request& req,
              std_srvs::Trigger::Response& res) {
  //
  planner_mode_ = PlannerMode::kCompartmentChange;
  return true;
}

bool Gbplanner::plannerGotoWaypointCallback(
    planner_msgs::planner_go_to_waypoint::Request& req,
    planner_msgs::planner_go_to_waypoint::Response& res) {
  res.path.clear();
  res.path = rrg_->getGlobalPath(req.waypoint);
  return true;
}

bool Gbplanner::plannerEnableUntraversablePolygonSubscriberCallback(
    std_srvs::SetBool::Request& request,
    std_srvs::SetBool::Response& response) {
  if (static_cast<bool>(request.data)) {
    ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Gbplanner checks traversability");
    untraversable_polygon_subscriber_ =
        nh_.subscribe("/traversability_estimation/untraversable_polygon", 100,
                      &Gbplanner::untraversablePolygonCallback, this);
  } else {
    ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Gbplanner stops checking traversability");
    untraversable_polygon_subscriber_.shutdown();
  }
  response.success = static_cast<unsigned char>(true);
  return true;
}

bool Gbplanner::plannerLoadGraphCallback(
    planner_msgs::planner_string_trigger::Request& req,
    planner_msgs::planner_string_trigger::Response& res) {
  res.success = rrg_->loadGraph(req.message);
  return true;
}

bool Gbplanner::plannerSaveGraphCallback(
    planner_msgs::planner_string_trigger::Request& req,
    planner_msgs::planner_string_trigger::Response& res) {
  res.success = rrg_->saveGraph(req.message);
  return true;
}

bool Gbplanner::setGlobalBound(
    planner_msgs::planner_set_global_bound::Request& req,
    planner_msgs::planner_set_global_bound::Response& res) {
  if (!req.get_current_bound)
    res.success = rrg_->setGlobalBound(req.bound, req.reset_to_default);
  else
    res.success = true;

  rrg_->getGlobalBound(res.bound_ret);
  return true;
}

bool Gbplanner::setDynamicGlobalBound(
    planner_msgs::planner_dynamic_global_bound::Request& req,
    planner_msgs::planner_dynamic_global_bound::Response& res) {
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Calling RRG set dynamic global bound");
  res.success = rrg_->setGlobalBound(req);
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "RRG set dynamic global bound returned");

  return true;
}

void Gbplanner::setGeofenceManager(
    std::shared_ptr<GeofenceManager> geofence_manager) {
  rrg_->setGeofenceManager(geofence_manager);
}

void Gbplanner::setSharedParams(const RobotParams& robot_params,
                                const BoundedSpaceParams& global_space_params) {
  rrg_->setSharedParams(robot_params, global_space_params);
}

void Gbplanner::setSharedParams(const RobotParams& robot_params,
                                const BoundedSpaceParams& global_space_params,
                                const BoundedSpaceParams& local_space_params) {
  rrg_->setSharedParams(robot_params, global_space_params, local_space_params);
}

bool Gbplanner::passingGateCallback(
    planner_msgs::planner_request_path::Request& req,
    planner_msgs::planner_request_path::Response& res) {
  res.path = rrg_->searchPathToPassGate();
  res.bound.mode = res.bound.kExtendedBound;
  return true;
}

bool Gbplanner::plannerServiceCallback(
    planner_msgs::planner_srv::Request& req,
    planner_msgs::planner_srv::Response& res) {

  rrg_->reset();
  
  if(planning_params_.exploration_only) {
    if(planning_params_.enable_opening_traversal) {
      if(exploration_counter_ >= planning_params_.max_exploration_iterations) {
        opening_traversal_requested_ = true;
        exploration_counter_ = 0;
        ROS_WARN("Exploration Iterations Complete, switching to: %d", (int)planner_mode_);
      }
      return getExplorationPath(req, res);
    }
    else {
      return getExplorationPath(req, res);
    }
  }

  ROS_WARN("Current Planner Mode: %d", (int)planner_mode_);

  bool success = true;
  std::vector<geometry_msgs::Pose> empty_path;
  switch (planner_mode_) {
    case PlannerMode::kExploration:
    {
      if(exploration_counter_ >= planning_params_.max_exploration_iterations) {
        res.path = empty_path;
        res.status = planner_msgs::planner_srv::Response::kForward;
        planner_mode_ = PlannerMode::kInspection;
        success = true;
        exploration_counter_ = 0;
        ROS_WARN("Exploration Iterations Complete, switching to: %d", (int)planner_mode_);
        break;
      }
      success = getExplorationPath(req, res);
      if(success)
      {
        ++exploration_counter_;
        ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Exploration Counter: %d", exploration_counter_);
        // If exploration is completed
        if(res.status != planner_msgs::planner_srv::Response::kForward) {
          res.path = empty_path;
          res.status = planner_msgs::planner_srv::Response::kForward;
          planner_mode_ = PlannerMode::kInspection;
          success = true;
          exploration_counter_ = 0;
          ROS_WARN("Exploration Complete, switching to: %d", (int)planner_mode_);
        }
      }
      else
      {
        ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Exploration Failed trying again:");
      }
      
      break;
    }

    case PlannerMode::kExplorationComplete:
    {
      res.path = empty_path;
      res.status = planner_msgs::planner_srv::Response::kManualCustomPath;
      planner_mode_ = PlannerMode::kInspection;
      ROS_WARN("Switching to: %d", (int)planner_mode_);
      break;
    }

    case PlannerMode::kInspection:
    {
      success = getInspectionPath(req, res);
      if(!success) {
        res.path = empty_path;
        res.status = planner_msgs::planner_srv::Response::kAutoCustomPath;
        ROS_WARN("Inspection Failed");
      }
      else {
        res.status = planner_msgs::planner_srv::Response::kAutoCustomPath;
        planner_mode_ = PlannerMode::kCompartmentChange;
        ROS_WARN("Inspection Successful. Switching to: %d", (int)planner_mode_);
      }
      break;
    }

    case PlannerMode::kCompartmentChange:
    {

      if(compartment_counter_ >= planning_params_.compartment_centers.size()-1) {
        res.status = planner_msgs::planner_srv::Response::kManualCustomPath;
        res.path = empty_path;
        success = true;
        compartment_change_tries_ = 0;
        ROS_WARN("All compartments explored");
        break;
      }

      bool search_success = getCompartmentTransitionPath(req, res);
      
      if(search_success) {
        compartment_change_tries_ = 0;
        success = true;
      }
      else {
        success = true;
        ++compartment_change_tries_;
        ROS_WARN_COND(global_verbosity>=Verbosity::WARN, "Path search failed. Try %d", compartment_change_tries_);
        if(compartment_change_tries_ <= max_compartment_change_tries_) {
          --compartment_counter_;  // To make sure that we try to replan
        }
      }
      rrg_->reset();
      
      break;
    }
    
    default:
      success = getExplorationPath(req, res);
      break;
  }

  return success;
}

Rrg::GlobalPlannerStatus Gbplanner::getGlobalExplorationPath()
{
  rrg_->setBoundMode(static_cast<BoundModeType>(in_srv_req_.bound_mode));

  int status;
  out_srv_res_.path = rrg_->runGlobalPlanner(0, false, false, status);
  out_srv_res_.status = status;
  if(status == planner_msgs::planner_srv::Response::kHoming)
  {
    bt_states_.global_exp_exhausted = true;
    return Rrg::GlobalPlannerStatus::G_HOMING;
  }

  if(out_srv_res_.path.empty())
  {
    return Rrg::GlobalPlannerStatus::G_ERR;
  }
  else
  {
    if(status == planner_msgs::planner_srv::Response::kHoming)
    {
      return Rrg::GlobalPlannerStatus::G_HOMING;
    }
  }
  return Rrg::GlobalPlannerStatus::G_OK;
}

bool Gbplanner::checkGlobalExplorationStatus()
{
  int status;
  std::vector<geometry_msgs::Pose> path = rrg_->runGlobalPlanner(0, false, false, status);
  if(status == planner_msgs::planner_srv::Response::kHoming)
  {
    bt_states_.global_exp_exhausted = true;
    return true;
  }

  return false;
}

Rrg::LocalPlannerStatus Gbplanner::getExplorationPath()
{
  // Extract setting from the request.
  rrg_->setGlobalFrame(in_srv_req_.header.frame_id);
  rrg_->setBoundMode(static_cast<BoundModeType>(in_srv_req_.bound_mode));
  rrg_->setRootStateForPlanning(in_srv_req_.root_pose);
  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Root state: %f %f %f, %f", in_srv_req_.root_pose.position.x, in_srv_req_.root_pose.position.y, in_srv_req_.root_pose.position.z, tf::getYaw(in_srv_req_.root_pose.orientation));

  Rrg::GraphStatus status;
  Rrg::LocalPlannerStatus ret_status;

  // Start the planner.
  out_srv_res_.path.clear();
  if (getPlannerStatus() == Gbplanner::PlannerStatus::NOT_READY) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "The planner is not ready.");
    status = Rrg::GraphStatus::NOT_OK;
    out_srv_res_.status = planner_msgs::planner_srv::Response::kForward;
    return Rrg::LocalPlannerStatus::L_ERR;
  }

  rrg_->reset();

  if (planning_params_.graph_building_mode == GraphBuildingModeType::kBasic) {
    status = rrg_->buildGraph();
  } else if (planning_params_.graph_building_mode == GraphBuildingModeType::kBatch) {
    status = rrg_->batchGraph();
  }

  switch (status) {
    case Rrg::GraphStatus::OK:
      ret_status = Rrg::LocalPlannerStatus::L_OK;
      break;
    case Rrg::GraphStatus::ERR_KDTREE:
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[PLANNER_ERROR] An issue occurred with kdtree data.");
      ret_status = Rrg::LocalPlannerStatus::L_ERR;
      break;
    case Rrg::GraphStatus::ERR_NO_FEASIBLE_PATH:
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[PLANNER_ERROR] No feasible path was found.");
      ret_status = Rrg::LocalPlannerStatus::L_ERR;
      break;
    case Rrg::GraphStatus::NOT_OK:
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[PLANNER_ERROR] Graph building: Not ok");
      ret_status = Rrg::LocalPlannerStatus::L_ERR;
      break;
    default:
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[PLANNER_ERROR] Error occurred in building graph.");
      ret_status = Rrg::LocalPlannerStatus::L_ERR;
      break;
  }

  if(status != Rrg::GraphStatus::OK) 
  {
    out_srv_res_.status = planner_msgs::planner_srv::Response::kForward;
    return ret_status;
  }
  
  status = rrg_->evaluateGraph();
  switch (status) {
    case Rrg::GraphStatus::OK:
      ret_status = Rrg::LocalPlannerStatus::L_OK;
      break;
    case Rrg::GraphStatus::NO_GAIN:
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[PLANNER_ERROR] No positive gain was found.");
      ret_status = Rrg::LocalPlannerStatus::L_OK;
      break;
    case Rrg::GraphStatus::CONSEC_LOW_GAIN:
      ROS_WARN_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "[GBPLANNER] Very low local gain. Triggering global planner");
      ret_status = Rrg::LocalPlannerStatus::L_EXHAUSTED;
      break;
    case Rrg::GraphStatus::NOT_OK:
      ROS_WARN_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "[PLANNER_ERROR] Error occurred in gain calculation.");
      ret_status = Rrg::LocalPlannerStatus::L_ERR;
      break;
    default:
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[PLANNER_ERROR] Error occurred in gain calculation.");
      ret_status = Rrg::LocalPlannerStatus::L_ERR;
      break;
  }

  if(status != Rrg::GraphStatus::OK) 
  {
    out_srv_res_.status = planner_msgs::planner_srv::Response::kForward;
    return ret_status;
  }

  if (status == Rrg::GraphStatus::OK) {
    out_srv_res_.path = rrg_->getBestPathSimplified();
    out_srv_res_.status = planner_msgs::planner_srv::Response::kForward;
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[GBPLANNER] Regular Planning");
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[GBPLANNER] Path status: %d", out_srv_res_.status);
  }
  return Rrg::LocalPlannerStatus::L_OK;
}

bool Gbplanner::transitionCompartment()
{
  rrg_->setNextCompartmentCenter(planning_params_.compartment_centers[compartment_counter_+1]);
  rrg_->setNextCompartmentIndex(compartment_counter_+1);
  BoundedSpaceParams translated_bound = planning_params_.compartment_dimensions;
  Eigen::Vector3d max_val = planning_params_.compartment_dimensions.max_val + planning_params_.compartment_centers[compartment_counter_];
  Eigen::Vector3d max_extension = planning_params_.compartment_dimensions.max_extension + planning_params_.compartment_centers[compartment_counter_];
  Eigen::Vector3d min_val = planning_params_.compartment_dimensions.min_val + planning_params_.compartment_centers[compartment_counter_];
  Eigen::Vector3d min_extension = planning_params_.compartment_dimensions.min_extension + planning_params_.compartment_centers[compartment_counter_];
  translated_bound.setBound(min_val, max_val);
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Compartment counter: %d", compartment_counter_);
  rrg_->setExplorationAndInspectionBounds(translated_bound, translated_bound);
  ++compartment_counter_;
  return true;
}

bool Gbplanner::allCompartmentsInspected()
{
  if(compartment_counter_ > planning_params_.compartment_centers.size()-1) 
  {
    return true;
  }
  else 
  {
    return false;
  }
}

bool Gbplanner::getExplorationPath(planner_msgs::planner_srv::Request& req,
      planner_msgs::planner_srv::Response& res) {
  //
  auto t1 = std::chrono::high_resolution_clock::now();
  auto t2 = t1;
  // Extract setting from the request.
  rrg_->setGlobalFrame(req.header.frame_id);
  t2 = std::chrono::high_resolution_clock::now();
  double reset_time = std::chrono::duration<double, std::milli>(t2 - t1).count();
  ROS_WARN("Reset time: %f", reset_time);
  rrg_->setBoundMode(static_cast<BoundModeType>(req.bound_mode));
  rrg_->setRootStateForPlanning(req.root_pose);
  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Root state: %f %f %f, %f", req.root_pose.position.x, req.root_pose.position.y, req.root_pose.position.z, tf::getYaw(req.root_pose.orientation));


  // Start the planner.
  res.path.clear();
  if (getPlannerStatus() == Gbplanner::PlannerStatus::NOT_READY) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "The planner is not ready.");
    return false;
  }

  t1 = std::chrono::high_resolution_clock::now();
  rrg_->reset();

  if(opening_traversal_requested_ || rrg_->openingTraversalOngoing()) {
    if(opening_traversal_requested_) {
      opening_traversal_requested_ = false;
    }
    res.path = rrg_->getOpeningTraversalPath();
    if(rrg_->autoOpeningPathApproval()) {
      res.status = planner_msgs::planner_srv::Response::kAutoCustomPath;
    }
    else {
      res.status = planner_msgs::planner_srv::Response::kManualCustomPath;
    }
    if(res.path.empty()) {
      if(rrg_->openingTraversalOngoing()) {
        return true;
      }
    }
    else {
      return true;
    }
  }

  Rrg::GraphStatus status;
  if (planning_params_.graph_building_mode == GraphBuildingModeType::kBasic) {
    status = rrg_->buildGraph();
  } else if (planning_params_.graph_building_mode == GraphBuildingModeType::kBatch) {
    status = rrg_->batchGraph();
  }

  switch (status) {
    case Rrg::GraphStatus::OK:
      break;
    case Rrg::GraphStatus::ERR_KDTREE:
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[PLANNER_ERROR] An issue occurred with kdtree data.");
      break;
    case Rrg::GraphStatus::ERR_NO_FEASIBLE_PATH:
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[PLANNER_ERROR] No feasible path was found.");
      break;
    case Rrg::GraphStatus::NOT_OK:
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[GBPLANNER] Resending global path");
      res.path = rrg_->reRunGlobalPlanner(res.status);
      break;
    default:
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[PLANNER_ERROR] Error occurred in building graph.");
      break;
  }

  bool global_planner_trig = false;
  if (status == Rrg::GraphStatus::OK) {
    status = rrg_->evaluateGraph();
    switch (status) {
      case Rrg::GraphStatus::OK:
        break;
      case Rrg::GraphStatus::NO_GAIN:
        ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[PLANNER_ERROR] No positive gain was found.");
        break;
      case Rrg::GraphStatus::NOT_OK:
        ROS_WARN_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "[GBPLANNER] Very low local gain. Triggering global planner");
        int status;
        res.path = rrg_->runGlobalPlanner(0, false, false, status);
        if(status < 0) {
          if(rrg_->autoOpeningPathApproval()) {
            res.status = planner_msgs::planner_srv::Response::kAutoCustomPath;
          }
          else {
            res.status = planner_msgs::planner_srv::Response::kManualCustomPath;
          }
          opening_traversal_ongoing_ = true;
        }
        else {
          res.status = status;
        }
        global_planner_trig = true;
        break;
      default:
        ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[PLANNER_ERROR] Error occurred in gain calculation.");
        break;
    }
  }
  else 
  {
    res.status = status;
    return false;
  }
  if (global_planner_trig) return true;

  if (status == Rrg::GraphStatus::OK) {
    if(planning_params_.exploration_only) {
      ++exploration_counter_;
    }
    res.path = rrg_->getBestPath(req.header.frame_id, res.status);
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[GBPLANNER] Regular Planning");
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[GBPLANNER] Path status: %d", res.status);
  }
  return true;
}

bool Gbplanner::getInspectionPath()
{
  rrg_->setBoundMode(static_cast<BoundModeType>(in_srv_req_.bound_mode));
  out_srv_res_.path = rrg_->getInspectionPath();
  out_srv_res_.status = planner_msgs::planner_srv::Response::kAutoCustomPath;

  if(out_srv_res_.path.size() > 0)
    return true;
  else 
    return false;
}

bool Gbplanner::getInspectionPath(planner_msgs::planner_srv::Request& req,
      planner_msgs::planner_srv::Response& res) {
  //
  res.path = rrg_->getInspectionPath();
  if(res.path.size() > 0)
    return true;
  else 
    return false;
}

void Gbplanner::getOpeningTraversalPath(OpeningTraversalMode mode, OpeningTraversalStatus &status)
{
  rrg_->setBoundMode(static_cast<BoundModeType>(in_srv_req_.bound_mode));
  out_srv_res_.path = rrg_->getOpeningTraversalPath(mode, status);
  out_srv_res_.status = planner_msgs::planner_srv::Response::kAutoCustomPath;
  if(status != OpeningTraversalStatus::OK)
  {
    out_srv_res_.path.clear();
  }
}

bool Gbplanner::getCompartmentTransitionPath(planner_msgs::planner_srv::Request& req,
      planner_msgs::planner_srv::Response& res) {
  //
  std::vector<geometry_msgs::Pose> empty_path;

  if(planning_params_.enable_opening_traversal) {
    rrg_->setNextCompartmentCenter(planning_params_.compartment_centers[compartment_counter_+1]);
    rrg_->setNextCompartmentIndex(compartment_counter_+1);
    res.path = rrg_->getOpeningTraversalPath();
    if(rrg_->autoOpeningPathApproval()) {
      res.status = planner_msgs::planner_srv::Response::kAutoCustomPath;
    }
    else {
      res.status = planner_msgs::planner_srv::Response::kManualCustomPath;
    }

    if(!rrg_->openingTraversalOngoing()) {  // Opening traversal finished
      planner_mode_ = PlannerMode::kExploration;
      ++compartment_counter_;
      BoundedSpaceParams translated_bound = planning_params_.compartment_dimensions;
      Eigen::Vector3d max_val = planning_params_.compartment_dimensions.max_val + planning_params_.compartment_centers[compartment_counter_];
      Eigen::Vector3d max_extension = planning_params_.compartment_dimensions.max_extension + planning_params_.compartment_centers[compartment_counter_];
      Eigen::Vector3d min_val = planning_params_.compartment_dimensions.min_val + planning_params_.compartment_centers[compartment_counter_];
      Eigen::Vector3d min_extension = planning_params_.compartment_dimensions.min_extension + planning_params_.compartment_centers[compartment_counter_];
      translated_bound.setBound(min_val, max_val);
      ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Compartment counter: %d", compartment_counter_);
      rrg_->setExplorationAndInspectionBounds(translated_bound, translated_bound);
    }
    else {
      planner_mode_ = PlannerMode::kCompartmentChange;
    }

    bool success = false;
    if(res.path.empty()) {
      if(rrg_->openingTraversalOngoing()) {
        success = true;
      }
    }
    else {
      success = true;
    }

    return success;
  }
  else {
    ++compartment_counter_;
    BoundedSpaceParams translated_bound = planning_params_.compartment_dimensions;
    Eigen::Vector3d max_val = planning_params_.compartment_dimensions.max_val + planning_params_.compartment_centers[compartment_counter_];
    Eigen::Vector3d max_extension = planning_params_.compartment_dimensions.max_extension + planning_params_.compartment_centers[compartment_counter_];
    Eigen::Vector3d min_val = planning_params_.compartment_dimensions.min_val + planning_params_.compartment_centers[compartment_counter_];
    Eigen::Vector3d min_extension = planning_params_.compartment_dimensions.min_extension + planning_params_.compartment_centers[compartment_counter_];
    translated_bound.setBound(min_val, max_val);
    BoundedSpaceParams extended_bound = planning_params_.compartment_dimensions;
    max_val = planning_params_.compartment_dimensions.max_val * 2.0;
    max_extension = planning_params_.compartment_dimensions.max_extension * 2.0;
    min_val = planning_params_.compartment_dimensions.min_val * 2.0;
    min_extension = planning_params_.compartment_dimensions.min_extension * 2.0;
    max_val += (planning_params_.compartment_centers[compartment_counter_] + planning_params_.compartment_centers[compartment_counter_-1]) / 2.0;
    max_extension += (planning_params_.compartment_centers[compartment_counter_] + planning_params_.compartment_centers[compartment_counter_-1]) / 2.0;
    min_val += (planning_params_.compartment_centers[compartment_counter_] + planning_params_.compartment_centers[compartment_counter_-1]) / 2.0;
    min_extension += (planning_params_.compartment_centers[compartment_counter_] + planning_params_.compartment_centers[compartment_counter_-1]) / 2.0;
    extended_bound.setBound(min_val, max_val);
    rrg_->setExplorationAndInspectionBounds(extended_bound, translated_bound);
    rrg_->reset();

    // ros::Duration(0.5).sleep();

    geometry_msgs::Pose current_pose;
    tf::Quaternion quat;
    quat.setEuler(0.0, 0.0, current_state_[3]);
    tf::Vector3 origin(current_state_[0], current_state_[1], current_state_[2]);
    tf::Pose poseTF(quat, origin);
    tf::poseTFToMsg(poseTF, current_pose);

    geometry_msgs::Pose target_pose;
    quat.setEuler(0.0, 0.0, 0.0);
    origin = tf::Vector3(planning_params_.compartment_centers[compartment_counter_][0], planning_params_.compartment_centers[compartment_counter_][1], planning_params_.compartment_centers[compartment_counter_][2]);
    tf::Pose poseTF_target(quat, origin);
    tf::poseTFToMsg(poseTF_target, target_pose);

    std::vector<geometry_msgs::Pose> connecting_path;
    bool search_success = rrg_->search(current_pose, target_pose, true, connecting_path);
    if(search_success) {
      rrg_->setExplorationAndInspectionBounds(translated_bound, translated_bound);
      res.path = connecting_path;
      res.status = planner_msgs::planner_srv::Response::kForward;
      planner_mode_ = PlannerMode::kExploration;
      ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Compartment counter: %d", compartment_counter_);
    }
    else {
      res.status = planner_msgs::planner_srv::Response::kForward;
      res.path = empty_path;
    }
    return search_success;
  }
}

bool Gbplanner::homingRequired()
{
  rrg_->setBoundMode(static_cast<BoundModeType>(in_srv_req_.bound_mode));
  out_srv_res_.status = planner_msgs::planner_srv::Response::kHoming;
  return rrg_->homingRequired(out_srv_res_.path);
}

bool Gbplanner::getHomingPath()
{
  if (getPlannerStatus() == Gbplanner::PlannerStatus::NOT_READY) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "The planner is not ready.");
    out_srv_res_.status = planner_msgs::planner_srv::Response::kForward;
    return false;
  }

  rrg_->setBoundMode(static_cast<BoundModeType>(in_srv_req_.bound_mode));
  out_srv_res_.path = rrg_->getHomingPath(in_srv_req_.header.frame_id);
  if(out_srv_res_.path.empty())
  {
    out_srv_res_.status = planner_msgs::planner_srv::Response::kForward;
    return false;  
  }

  out_srv_res_.status = planner_msgs::planner_srv::Response::kHoming;
  return true;
}

bool Gbplanner::homingServiceCallback(
    planner_msgs::planner_homing::Request& req,
    planner_msgs::planner_homing::Response& res) {
  res.path.clear();
  if (getPlannerStatus() == Gbplanner::PlannerStatus::NOT_READY) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "The planner is not ready.");
    return false;
  }
  res.path = rrg_->getHomingPath(req.header.frame_id);
  return true;
}

bool Gbplanner::globalPlannerServiceCallback(
    planner_msgs::planner_global::Request& req,
    planner_msgs::planner_global::Response& res) {
  res.path.clear();
  if (getPlannerStatus() == Gbplanner::PlannerStatus::NOT_READY) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "The planner is not ready.");
    return false;
  }
  int status;
  res.path =
      rrg_->runGlobalPlanner(req.id, req.not_check_frontier, req.ignore_time, status);
  return true;
}

bool Gbplanner::setHomingPosServiceCallback(
    planner_msgs::planner_set_homing_pos::Request& req,
    planner_msgs::planner_set_homing_pos::Response& res) {
  if (getPlannerStatus() == Gbplanner::PlannerStatus::NOT_READY) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "The planner is not ready.");
    return false;
  }
  res.success = rrg_->setHomingPos();
  return true;
}

bool Gbplanner::plannerSearchServiceCallback(
    planner_msgs::planner_search::Request& req,
    planner_msgs::planner_search::Response& res) {
  rrg_->setBoundMode(static_cast<BoundModeType>(req.bound_mode));
  res.success =
      rrg_->search(req.source, req.target, req.use_current_state, res.path);
  return true;
}

bool Gbplanner::plannerSetPlanningTriggerModeCallback(
    planner_msgs::planner_set_planning_mode::Request& request,
    planner_msgs::planner_set_planning_mode::Response& response) {
  PlannerTriggerModeType in_trig_mode;
  if (request.planning_mode == request.kAuto)
    in_trig_mode = PlannerTriggerModeType::kAuto;
  else if (request.planning_mode == request.kManual)
    in_trig_mode = PlannerTriggerModeType::kManual;
  rrg_->setPlannerTriggerMode(in_trig_mode);
  response.success = true;
  return true;
}

bool Gbplanner::clearUntraversableZones(std_srvs::Trigger::Request& req,
                                        std_srvs::Trigger::Response& res) {
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Clearing untraversable zones");
  rrg_->clearUntraversableZones();
  res.success = true;
  return true;
}

void Gbplanner::untraversablePolygonCallback(
    const geometry_msgs::PolygonStamped& polygon_msgs) {
  // Add the new polygon into geofence list
  if (!polygon_msgs.polygon.points.empty()) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Detected untraversable area");
    // Add this to the list
    rrg_->addGeofenceAreas(polygon_msgs);
  }
}

void Gbplanner::setUntraversablePolygon(
    const geometry_msgs::PolygonStamped& polygon_msgs) {
  std::cout << "Untraversable polygon size: "
            << polygon_msgs.polygon.points.size() << std::endl;
  if (!polygon_msgs.polygon.points.empty()) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Detected untraversable area");
    // Add this to the list
    rrg_->addGeofenceAreas(polygon_msgs);
  }
}

void Gbplanner::poseCallback(
    const geometry_msgs::PoseWithCovarianceStamped& pose) {
  processPose(pose.pose.pose);
}

void Gbplanner::poseStampedCallback(const geometry_msgs::PoseStamped& pose) {
  processPose(pose.pose);
}

void Gbplanner::processPose(const geometry_msgs::Pose& pose) {
  StateVec state;
  state[0] = pose.position.x;
  state[1] = pose.position.y;
  state[2] = pose.position.z;
  state[3] = tf::getYaw(pose.orientation);
  rrg_->setState(state);
  current_state_ = state;
}

void Gbplanner::odometryCallback(const nav_msgs::Odometry& odo) {
  StateVec state;
  state[0] = odo.pose.pose.position.x;
  state[1] = odo.pose.pose.position.y;
  state[2] = odo.pose.pose.position.z;
  state[3] = tf::getYaw(odo.pose.pose.orientation);
  rrg_->setState(state);
  current_state_ = state;
}

void Gbplanner::robotStatusCallback(const planner_msgs::RobotStatus& status) {
  rrg_->setTimeRemaining(status.time_remaining);
}

Gbplanner::PlannerStatus Gbplanner::getPlannerStatus() {


  // Should have a list of checking conditions to set the planner as ready.
  // For examples:
  // + ROS ok
  // + All params loaded properly
  // + Map is ready to use
  if (planner_status_ == Gbplanner::PlannerStatus::READY)
    return Gbplanner::PlannerStatus::READY;

  return Gbplanner::PlannerStatus::READY;
}

// }  // namespace explorer
