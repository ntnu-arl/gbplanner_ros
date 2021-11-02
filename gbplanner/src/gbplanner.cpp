#include "gbplanner/gbplanner.h"

#include <nav_msgs/Path.h>

namespace explorer {

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
                     MapManagerVoxblox<MapManagerVoxbloxServer,
                                       MapManagerVoxbloxVoxel>* map_manager)
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
  // Extract setting from the request.
  rrg_->setGlobalFrame(req.header.frame_id);
  rrg_->setBoundMode(static_cast<BoundModeType>(req.bound_mode));
  rrg_->setRootStateForPlanning(req.root_pose);

  // Start the planner.
  res.path.clear();
  if (getPlannerStatus() == Gbplanner::PlannerStatus::NOT_READY) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "The planner is not ready.");
    return false;
  }

  rrg_->reset();
  Rrg::GraphStatus status = rrg_->buildGraph();
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
      res.path = rrg_->reRunGlobalPlanner();
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
        res.path = rrg_->runGlobalPlanner(0, false, false);
        res.status = planner_msgs::planner_srv::Response::kRepositioning;
        break;
      default:
        ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[PLANNER_ERROR] Error occurred in gain calculation.");
        break;
    }
  }
  if (global_planner_trig) return true;

  if (status == Rrg::GraphStatus::OK) {
    res.path = rrg_->getBestPath(req.header.frame_id, res.status);
  }
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
  res.path =
      rrg_->runGlobalPlanner(req.id, req.not_check_frontier, req.ignore_time);
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
}

void Gbplanner::odometryCallback(const nav_msgs::Odometry& odo) {
  StateVec state;
  state[0] = odo.pose.pose.position.x;
  state[1] = odo.pose.pose.position.y;
  state[2] = odo.pose.pose.position.z;
  state[3] = tf::getYaw(odo.pose.pose.orientation);
  rrg_->setState(state);
}

void Gbplanner::robotStatusCallback(const planner_msgs::RobotStatus& status) {
  rrg_->setTimeRemaining(status.time_remaining);
}

Gbplanner::PlannerStatus Gbplanner::getPlannerStatus() {
  // if (!ros::ok()) {
  //   ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "ROS node failed.");
  //   return false;
  // }

  // Should have a list of checking conditions to set the planner as ready.
  // For examples:
  // + ROS ok
  // + All params loaded properly
  // + Map is ready to use
  if (planner_status_ == Gbplanner::PlannerStatus::READY)
    return Gbplanner::PlannerStatus::READY;

  return Gbplanner::PlannerStatus::READY;
}

}  // namespace explorer
