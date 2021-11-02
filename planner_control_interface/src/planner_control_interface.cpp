
#include "planner_control_interface/planner_control_interface.h"

#include <chrono>
#include <thread>

#include <std_msgs/Bool.h>

namespace explorer {

PlannerControlInterface::PlannerControlInterface(
    ros::NodeHandle& nh, ros::NodeHandle& nh_private,
    std::shared_ptr<PCIManager> pci_manager)
    : nh_(nh), nh_private_(nh_private) {
  reference_pub_ = nh_.advertise<visualization_msgs::Marker>("ref_pose", 5);
  planner_status_pub_ = nh_.advertise<std_msgs::Bool>("gbplanner_status", 5);
  stop_request_pub_ = nh_.advertise<std_msgs::Bool>(
      "planner_control_interface/stop_request", 5);

  odometry_sub_ = nh_.subscribe(
      "odometry", 1, &PlannerControlInterface::odometryCallback, this);
  pose_sub_ =
      nh_.subscribe("pose", 1, &PlannerControlInterface::poseCallback, this);
  pose_stamped_sub_ = nh_.subscribe(
      "pose_stamped", 1, &PlannerControlInterface::poseStampedCallback, this);
  pci_server_ =
      nh_.advertiseService("planner_control_interface_trigger",
                           &PlannerControlInterface::triggerCallback, this);
  pci_std_automatic_planning_server_ = nh_.advertiseService(
      "planner_control_interface/std_srvs/automatic_planning",
      &PlannerControlInterface::stdSrvsAutomaticPlanningCallback, this);
  pci_std_single_planning_server_ = nh_.advertiseService(
      "planner_control_interface/std_srvs/single_planning",
      &PlannerControlInterface::stdSrvsSinglePlanningCallback, this);
  pci_homing_server_ = nh_.advertiseService(
      "pci_homing_trigger", &PlannerControlInterface::homingCallback, this);
  pci_std_homing_server_ = nh_.advertiseService(
      "planner_control_interface/std_srvs/homing_trigger",
      &PlannerControlInterface::stdSrvHomingCallback, this);
  pci_std_go_to_waypoint_server_ = nh_.advertiseService(
      "planner_control_interface/std_srvs/go_to_waypoint",
      &PlannerControlInterface::stdSrvGoToWaypointCallback, this);
  pci_initialization_server_ = nh_.advertiseService(
      "pci_initialization_trigger",
      &PlannerControlInterface::initializationCallback, this);
  while (!(planner_client_ = nh.serviceClient<planner_msgs::planner_srv>(
               "planner_server", true))) {  // true for persistent
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                  "PCI: service planner_server is not available: waiting...");
    sleep(1);
  }
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO,
                "PCI: connected to service planner_server.");
  while (
      !(planner_homing_client_ = nh.serviceClient<planner_msgs::planner_homing>(
            "planner_homing_server", true))) {  // true for persistent
    ROS_WARN_COND(
        global_verbosity >= Verbosity::WARN,
        "PCI: service planner_homing_server is not available: waiting...");
    sleep(1);
  }
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO,
                "PCI: connected to service planner_homing_server.");

  pci_set_homing_pos_server_ = nh_.advertiseService(
      "pci_set_homing_pos", &PlannerControlInterface::setHomingPosCallback,
      this);
  pci_std_set_homing_pos_server_ = nh_.advertiseService(
      "planner_control_interface/std_srvs/set_homing_position_here",
      &PlannerControlInterface::stdSrvSetHomingPositionHereCallback, this);
  planner_set_homing_pos_client_ =
      nh.serviceClient<planner_msgs::planner_set_homing_pos>(
          "gbplanner/set_homing_pos");

  planner_set_trigger_mode_client_ =
      nh.serviceClient<planner_msgs::planner_set_planning_mode>(
          "/gbplanner/set_planning_trigger_mode");

  pci_search_server_ = nh_.advertiseService(
      "pci_search", &PlannerControlInterface::searchCallback, this);
  planner_search_client_ =
      nh.serviceClient<planner_msgs::planner_search>("gbplanner/search");

  pci_global_server_ = nh_.advertiseService(
      "pci_global", &PlannerControlInterface::globalPlannerCallback, this);
  planner_global_client_ =
      nh.serviceClient<planner_msgs::planner_global>("gbplanner/global");
  pci_stop_server_ = nh_.advertiseService(
      "pci_stop", &PlannerControlInterface::stopPlannerCallback, this);
  pci_std_stop_server_ = nh_.advertiseService(
      "planner_control_interface/std_srvs/stop",
      &PlannerControlInterface::stdSrvStopPlannerCallback, this);

  planner_geofence_client_ =
      nh.serviceClient<planner_msgs::planner_geofence>("gbplanner/geofence");
  pci_geofence_server_ = nh_.advertiseService(
      "pci_geofence", &PlannerControlInterface::addGeofenceCallback, this);
  pci_to_waypoint_server_ = nh_.advertiseService(
      "pci_to_waypoint", &PlannerControlInterface::goToWaypointCallback, this);

  planner_passing_gate_client_ =
      nh_.serviceClient<planner_msgs::planner_request_path>(
          "gbplanner/passing_gate");
  pci_passing_gate_server_ =
      nh_.advertiseService("planner_control_interface/std_srvs/pass_gate",
                           &PlannerControlInterface::passingGateCallback, this);

  rotate_180_deg_server_ = nh_.advertiseService(
      "pci_rotate_180_trigger", &PlannerControlInterface::rotate180DegCallback,
      this);

  pci_std_global_last_specified_frontier_server_ = nh_.advertiseService(
      "planner_control_interface/std_srvs/replan_last_specified_frontier",
      &PlannerControlInterface::stdSrvReplanLastSpecifiedFrontierCallback,
      this);

  planner_set_exp_mode_client_ =
      nh_.serviceClient<planner_msgs::planner_set_exp_mode>(
          "gbplanner/set_exp_mode");

  imarker_server_.reset(
      new interactive_markers::InteractiveMarkerServer("waypoints", "", false));

  semantic_server.reset(
      new interactive_markers::InteractiveMarkerServer("semantics", "", false));
  semantic_pub = nh_.advertise<planner_semantic_msgs::SemanticPoint>(
      "semantic_location", 10);

  nav_goal_sub_ =
      nh_.subscribe("/move_base_simple/goal", 1,
                    &PlannerControlInterface::navGoalCallback, this);
  pose_goal_sub_ =
      nh_.subscribe("/global_planner/waypoint_request", 1,
                    &PlannerControlInterface::poseGoalCallback, this);
  nav_goal_client_ = nh_.serviceClient<planner_msgs::planner_go_to_waypoint>(
      "gbplanner/go_to_waypoint");
  go_to_waypoint_visualization_pub_ = nh_.advertise<visualization_msgs::Marker>(
      "gbplanner/go_to_waypoint_pose_visualization", 0);

  ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                "[PCI]: Setting pci_manager_");
  pci_manager_ = pci_manager;

  ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[PCI]: Loading params");
  if (!loadParams()) {
    ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR,
                   "Can not load params. Shut down ros node.");
    ros::shutdown();
  }

  ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[PCI]: Initializing pci");
  if (!init()) {
    ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR,
                   "Can not initialize the node. Shut down ros node.");
    ros::shutdown();
  }
  // pci_manager_->initialize();
  ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                "[PCI]: Starting run() loop");
  run();
}

void PlannerControlInterface::poseGoalCallback(
    const geometry_msgs::PoseStamped& pose_msgs) {
  setGoal(pose_msgs);
}

void PlannerControlInterface::navGoalCallback(
    const geometry_msgs::PoseStamped& nav_msgs) {
  geometry_msgs::PoseStamped posest;
  posest.header = nav_msgs.header;
  posest.pose = nav_msgs.pose;
  setGoal(posest);
}

void PlannerControlInterface::setGoal(const geometry_msgs::PoseStamped& pose) {
  geometry_msgs::PoseStamped pose_in_world_frame;

  tf::Stamped<tf::Pose> pin, pout;
  poseStampedMsgToTF(pose, pin);
  tf::StampedTransform darpa_to_world_transform;
  try {
    tf_listener_.lookupTransform(world_frame_id_, pose.header.frame_id,
                                 ros::Time(0), darpa_to_world_transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR,
                   "[gbplanner_pci::setGoal] %s", ex.what());
    return;
  }
  pout.setData(darpa_to_world_transform * pin);
  pout.stamp_ = darpa_to_world_transform.stamp_;
  pout.frame_id_ = world_frame_id_;
  poseStampedTFToMsg(pout, pose_in_world_frame);

  received_first_waypoint_to_go_ = true;
  go_to_waypoint_with_checking_ = true;
  set_waypoint_stamped_ = pose_in_world_frame;
  ROS_INFO_COND(
      global_verbosity >= Verbosity::INFO,
      "[gbplanner_pci::setGoal] set waypoint to (%.2f, %.2f, %.2f) in frame "
      "'%s'",
      pose_in_world_frame.pose.position.x, pose_in_world_frame.pose.position.y,
      pose_in_world_frame.pose.position.z,
      pose_in_world_frame.header.frame_id.c_str());
  ros::Rate rr(10);  // 10Hz
  for (int i = 0; i < 5; ++i) {
    publishGoToWaypointVisualization(set_waypoint_stamped_);
    ros::spinOnce();
    rr.sleep();
  }
}

bool PlannerControlInterface::passingGateCallback(
    std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  if (!passing_gate_success_) {
    passing_gate_request_ = true;
    return true;
  } else {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                  "Passing gate already activated.");
    return false;
  }
}

void PlannerControlInterface::resetPlanner() {
  // Set back to manual mode, and stop all current requests.
  trigger_mode_ = PlannerTriggerModeType::kManual;
  run_en_ = false;
  search_request_ = false;
  homing_request_ = false;
  init_request_ = false;
  global_request_ = false;
  go_to_waypoint_request_ = false;
  go_to_waypoint_with_checking_ = false;

  // Remove the last waypoint to prevent the planner starts from that last wp.
  current_path_.clear();
  pci_manager_->setStatus(PCIManager::PCIStatus::kReady);
}

bool PlannerControlInterface::rotate180DegCallback(
    std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  planner_msgs::planner_set_planning_mode planning_mode_srv;
  planning_mode_srv.request.planning_mode =
      planner_msgs::planner_set_planning_mode::Request::kManual;
  planner_set_trigger_mode_client_.call(planning_mode_srv);

  go_to_waypoint_request_ = true;
  go_to_waypoint_with_checking_ = false;
  set_waypoint_.position.x = current_pose_.position.x;
  set_waypoint_.position.y = current_pose_.position.y;
  set_waypoint_.position.z = current_pose_.position.z;
  set_waypoint_.orientation.x = 0.0;
  set_waypoint_.orientation.y = 0.0;
  set_waypoint_.orientation.z = 1.0;
  set_waypoint_.orientation.w = 0.0;
  return true;
}

bool PlannerControlInterface::goToWaypointCallback(
    planner_msgs::pci_to_waypoint::Request& req,
    planner_msgs::pci_to_waypoint::Response& res) {
  planner_msgs::planner_set_planning_mode planning_mode_srv;
  planning_mode_srv.request.planning_mode =
      planner_msgs::planner_set_planning_mode::Request::kManual;
  planner_set_trigger_mode_client_.call(planning_mode_srv);

  go_to_waypoint_request_ = true;
  go_to_waypoint_with_checking_ = false;
  set_waypoint_.position.x = req.waypoint.position.x;
  set_waypoint_.position.y = req.waypoint.position.y;
  set_waypoint_.position.z = req.waypoint.position.z;
  set_waypoint_.orientation.x = req.waypoint.orientation.x;
  set_waypoint_.orientation.y = req.waypoint.orientation.y;
  set_waypoint_.orientation.z = req.waypoint.orientation.z;
  set_waypoint_.orientation.w = req.waypoint.orientation.w;
  return true;
}

bool PlannerControlInterface::searchCallback(
    planner_msgs::pci_search::Request& req,
    planner_msgs::pci_search::Response& res) {
  search_request_ = true;
  exe_path_en_ = !req.not_exe_path;
  use_current_state_ = req.use_current_state;
  bound_mode_ = req.bound_mode;
  return true;
}

bool PlannerControlInterface::globalPlannerCallback(
    planner_msgs::pci_global::Request& req,
    planner_msgs::pci_global::Response& res) {
  global_request_ = true;
  exe_path_en_ = !req.not_exe_path;
  bound_mode_ = req.bound_mode;
  frontier_id_ = req.id;
  pci_global_request_params_ = req;
  res.success = true;
  return true;
}

bool PlannerControlInterface::stopPlannerCallback(
    planner_msgs::pci_stop::Request& req,
    planner_msgs::pci_stop::Response& res) {
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO,
                "[PlannerControlInterface::stopPlannerCallback]");
  std_msgs::Bool stop_msg;
  stop_request_pub_.publish(stop_msg);
  pci_manager_->stopPCI();
  stop_planner_request_ = true;
  resetPlanner();

  planner_msgs::planner_set_planning_mode planning_mode_srv;
  planning_mode_srv.request.planning_mode =
      planner_msgs::planner_set_planning_mode::Request::kManual;
  planner_set_trigger_mode_client_.call(planning_mode_srv);

  res.success = true;
  ROS_WARN_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "[PCI] STOP PLANNER.");
  return true;
}

bool PlannerControlInterface::stdSrvStopPlannerCallback(
    std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  planner_msgs::pci_stop::Request stop_request;
  planner_msgs::pci_stop::Response stop_response;

  res.success = stopPlannerCallback(stop_request, stop_response);
  res.success &= stop_response.success;

  return true;
}

bool PlannerControlInterface::addGeofenceCallback(
    planner_msgs::pci_geofence::Request& req,
    planner_msgs::pci_geofence::Response& res) {
  planner_msgs::planner_geofence plan_srv;
  plan_srv.request.rectangles = req.rectangles;
  if (planner_geofence_client_.call(plan_srv))
    res.success = plan_srv.response.success;

  return true;
}

bool PlannerControlInterface::setHomingPosCallback(
    planner_msgs::pci_set_homing_pos::Request& req,
    planner_msgs::pci_set_homing_pos::Response& res) {
  // Bypass this request to the planner.
  planner_msgs::planner_set_homing_pos plan_srv;
  if (planner_set_homing_pos_client_.call(plan_srv)) {
    res.success = plan_srv.response.success;
  }
  return true;
}

bool PlannerControlInterface::stdSrvSetHomingPositionHereCallback(
    std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  planner_msgs::pci_set_homing_pos::Request set_homing_pos_request;
  planner_msgs::pci_set_homing_pos::Response set_homing_pos_response;

  res.success =
      setHomingPosCallback(set_homing_pos_request, set_homing_pos_response);
  res.success &= set_homing_pos_response.success;

  return true;
}

bool PlannerControlInterface::initializationCallback(
    planner_msgs::pci_initialization::Request& req,
    planner_msgs::pci_initialization::Response& res) {
  init_request_ = true;
  res.success = true;
  return true;
}

bool PlannerControlInterface::homingCallback(
    planner_msgs::pci_homing_trigger::Request& req,
    planner_msgs::pci_homing_trigger::Response& res) {
  exe_path_en_ = !req.not_exe_path;
  homing_request_ = true;
  res.success = true;
  return true;
}

bool PlannerControlInterface::stdSrvHomingCallback(
    std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  planner_msgs::pci_homing_trigger::Request homing_trigger_request;
  planner_msgs::pci_homing_trigger::Response homing_trigger_response;

  homing_trigger_request.not_exe_path = false;

  res.success = homingCallback(homing_trigger_request, homing_trigger_response);
  res.success &= homing_trigger_response.success;

  return true;
}

bool PlannerControlInterface::triggerCallback(
    planner_msgs::pci_trigger::Request& req,
    planner_msgs::pci_trigger::Response& res) {
  if (pci_manager_->getStatus() == PCIManager::PCIStatus::kError) {
    ROS_WARN_COND(
        global_verbosity >= Verbosity::WARN,
        "PCIManager is curretely in error state and cannot accept planning "
        "requests.");
    res.success = false;
  } else {
    if ((!req.set_auto) && (trigger_mode_ == PlannerTriggerModeType::kAuto)) {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                    "Switch to manual mode.");
      trigger_mode_ = PlannerTriggerModeType::kManual;
    } else if ((req.set_auto) &&
               (trigger_mode_ == PlannerTriggerModeType::kManual)) {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                    "Switch to auto mode.");
      trigger_mode_ = PlannerTriggerModeType::kAuto;
    }
    pci_manager_->setVelocity(req.vel_max);
    bound_mode_ = req.bound_mode;
    run_en_ = true;
    exe_path_en_ = !req.not_exe_path;
    res.success = true;
  }
  return true;
}

bool PlannerControlInterface::stdSrvsAutomaticPlanningCallback(
    std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  planner_msgs::pci_trigger::Request pci_trigger_request;
  planner_msgs::pci_trigger::Response pci_trigger_response;
  pci_trigger_request.not_exe_path = false;
  pci_trigger_request.set_auto = true;
  pci_trigger_request.bound_mode = 0;
  pci_trigger_request.vel_max = 0.0;

  res.success = triggerCallback(pci_trigger_request, pci_trigger_response);
  res.success &= pci_trigger_response.success;

  return true;
}

bool PlannerControlInterface::stdSrvGoToWaypointCallback(
    std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  planner_msgs::planner_set_planning_mode planning_mode_srv;
  planning_mode_srv.request.planning_mode =
      planner_msgs::planner_set_planning_mode::Request::kManual;
  planner_set_trigger_mode_client_.call(planning_mode_srv);

  if (received_first_waypoint_to_go_) {
    go_to_waypoint_request_ = true;
    go_to_waypoint_with_checking_ = true;
    res.success = true;
  } else {
    ROS_ERROR_COND(
        global_verbosity >= Verbosity::ERROR,
        "No waypoint was set, 'go_to_waypoint' feature will not be triggered.");
    res.success = false;
  }
  return true;
}

bool PlannerControlInterface::stdSrvsSinglePlanningCallback(
    std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  planner_msgs::pci_trigger::Request pci_trigger_request;
  planner_msgs::pci_trigger::Response pci_trigger_response;
  pci_trigger_request.not_exe_path = false;
  pci_trigger_request.set_auto = false;
  pci_trigger_request.bound_mode = 0;
  pci_trigger_request.vel_max = 0.0;

  res.success = triggerCallback(pci_trigger_request, pci_trigger_response);
  res.success &= pci_trigger_response.success;
  return true;
}

bool PlannerControlInterface::stdSrvReplanLastSpecifiedFrontierCallback(
    std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  planner_msgs::pci_global::Request pci_global_request;
  planner_msgs::pci_global::Response pci_global_response;
  pci_global_request.not_exe_path = false;
  pci_global_request.set_auto = false;
  pci_global_request.bound_mode = 0;
  pci_global_request.vel_max = 0.0;
  // Use last frontier specified via service call.
  pci_global_request.id = frontier_id_;

  res.success = globalPlannerCallback(pci_global_request, pci_global_response);
  res.success &= pci_global_response.success;

  return true;
}

bool PlannerControlInterface::init() {
  planner_iteration_ = 0;
  reference_pub_id_ = 0;
  homing_request_ = false;
  run_en_ = false;
  exe_path_en_ = true;
  pose_is_ready_ = false;
  planner_msgs::planner_srv plan_srv_temp;
  bound_mode_ = plan_srv_temp.request.kExtendedBound;
  force_forward_ = true;
  init_request_ = false;
  search_request_ = false;
  global_request_ = false;
  stop_planner_request_ = false;
  passing_gate_request_ = false;
  passing_gate_success_ = false;
  go_to_waypoint_request_ = false;
  // Wait for the system is ready.
  // For example: checking odometry is ready.
  ros::Rate rr(1);
  while (!pose_is_ready_) {
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Waiting for odometry.");
    ros::spinOnce();
    rr.sleep();
  }
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG,
                "[PCI]:[init()]: recieved odom");
  initIMarker();
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG,
                "[PCI]:[init()]: initIMarker");
  initSemanticIMarker();
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG,
                "[PCI]:[init()]: initSemanticIMarker");
  if (!pci_manager_->initialize()) return false;
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG,
                "[PCI]:[init()]: pci manager init done");
  return true;
}

void PlannerControlInterface::run() {
  ros::Rate rr(10);  // 10Hz
  bool cont = true;
  while (cont) {
    PCIManager::PCIStatus pci_status = pci_manager_->getStatus();
    // TODO: Fix by prioritizing and sequencing exclusive cases (with bad
    // if/else and flags approach)
    if (pci_status == PCIManager::PCIStatus::kReady) {
      // Priority 1: Check if require homing.
      if (homing_request_) {
        homing_request_ = false;
        trigger_mode_ = PlannerTriggerModeType::kManual;  // also unset auto
                                                          // mode
        ROS_INFO_COND(global_verbosity >= Verbosity::INFO,
                      "PlannerControlInterface: Running Homing");
        runHoming(exe_path_en_);
      }
      // Priority 2: Check if require initialization step.
      else if (init_request_) {
        init_request_ = false;
        ROS_INFO_COND(global_verbosity >= Verbosity::INFO,
                      "PlannerControlInterface: Running Initialization");
        runInitialization();
        // Priority 3: Stop
      } else if (stop_planner_request_) {
        // Stop at current pose.
        ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                      "PCI: run: stop requested");
        stop_planner_request_ = false;
        pci_manager_->goToWaypoint(current_pose_);
      } else if ((trigger_mode_ == PlannerTriggerModeType::kAuto) ||
                 (run_en_)) {
        run_en_ = false;
        ROS_INFO_COND(global_verbosity >= Verbosity::INFO,
                      "PlannerControlInterface: Running Planner (%s)",
                      std::string(trigger_mode_ == PlannerTriggerModeType::kAuto
                                      ? "kAuto"
                                      : "kManual")
                          .c_str());
        runPlanner(exe_path_en_);
      } else if (search_request_) {
        search_request_ = false;
        ROS_INFO_COND(global_verbosity >= Verbosity::INFO,
                      "Request the planner to search for connection path.");
        runSearch(exe_path_en_);
      } else if (global_request_) {
        global_request_ = false;
        ROS_INFO_COND(global_verbosity >= Verbosity::INFO,
                      "Request the global planner.");
        runGlobalPlanner(exe_path_en_);
      } else if (passing_gate_request_) {
        passing_gate_request_ = false;
        runPassingGate();
      } else if (go_to_waypoint_request_) {
        go_to_waypoint_request_ = false;
        if (!go_to_waypoint_with_checking_)
          pci_manager_->goToWaypoint(set_waypoint_);
        else
          runGlobalRepositioning();
      }
    } else if (pci_status == PCIManager::PCIStatus::kError) {
      // For ANYmal, reset everything to manual then wait for operator.
      resetPlanner();
    }
    cont = ros::ok();
    ros::spinOnce();
    rr.sleep();
  }
}

void PlannerControlInterface::runGlobalRepositioning() {
  ROS_INFO_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "Global Repositioning %i",
                planner_iteration_);

  planner_msgs::planner_set_planning_mode planning_mode_srv;
  planning_mode_srv.request.planning_mode =
      planner_msgs::planner_set_planning_mode::Request::kManual;
  planner_set_trigger_mode_client_.call(planning_mode_srv);

  planner_msgs::planner_go_to_waypoint planner_srv;
  planner_srv.request.check_collision = true;
  planner_srv.request.waypoint.header = set_waypoint_stamped_.header;
  planner_srv.request.waypoint.pose = set_waypoint_stamped_.pose;

  if (nav_goal_client_.call(planner_srv)) {
    if (!planner_srv.response.path.empty()) {
      // Execute path.
      current_path_.clear();
      // resetPlanner();
      std::vector<geometry_msgs::Pose> path_to_be_exe;
      pci_manager_->executePath(planner_srv.response.path, path_to_be_exe,
                                PCIManager::ExecutionPathType::kGlobalPath);
      current_path_ = path_to_be_exe;
    } else {
      ROS_WARN_THROTTLE(1, "Will not execute the path.");
      ros::Duration(0.5).sleep();
    }
    planner_iteration_++;
  } else {
    ROS_WARN_THROTTLE(1, "Planner service failed");
    ros::Duration(0.5).sleep();
  }
}

void PlannerControlInterface::runPassingGate() {
  bool success = true;

  planner_msgs::planner_set_planning_mode planning_mode_srv;
  planning_mode_srv.request.planning_mode =
      planner_msgs::planner_set_planning_mode::Request::kManual;
  planner_set_trigger_mode_client_.call(planning_mode_srv);

  planner_msgs::planner_request_path plan_srv;
  if ((!planner_passing_gate_client_.call(plan_srv)) ||
      (plan_srv.response.path.empty()))
    success = false;

  std_msgs::Bool planner_success_msg;
  planner_success_msg.data = success;
  planner_status_pub_.publish(planner_success_msg);

  if (success) {
    passing_gate_success_ = true;
    std::vector<geometry_msgs::Pose> path_to_be_exe;
    pci_manager_->executePath(plan_srv.response.path, path_to_be_exe);
  }
}

void PlannerControlInterface::runGlobalPlanner(bool exe_path = false) {
  ROS_INFO_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "Planning iteration %i",
                planner_iteration_);

  planner_msgs::planner_set_planning_mode planning_mode_srv;
  planning_mode_srv.request.planning_mode =
      planner_msgs::planner_set_planning_mode::Request::kManual;
  planner_set_trigger_mode_client_.call(planning_mode_srv);

  planner_msgs::planner_global plan_srv;
  plan_srv.request.id = pci_global_request_params_.id;
  plan_srv.request.not_check_frontier =
      pci_global_request_params_.not_check_frontier;
  plan_srv.request.ignore_time = pci_global_request_params_.ignore_time;
  if (planner_global_client_.call(plan_srv)) {
    if ((exe_path) && (!plan_srv.response.path.empty())) {
      // Execute path.
      std::vector<geometry_msgs::Pose> path_to_be_exe;
      pci_manager_->executePath(plan_srv.response.path, path_to_be_exe,
                                PCIManager::ExecutionPathType::kGlobalPath);
      current_path_ = path_to_be_exe;
    } else {
      ROS_WARN_THROTTLE(1, "Will not execute the path.");
      ros::Duration(0.5).sleep();
    }
    planner_iteration_++;
  } else {
    ROS_WARN_THROTTLE(1, "Planner service failed");
    ros::Duration(0.5).sleep();
  }
}

void PlannerControlInterface::runPlanner(bool exe_path = false) {
  const int kBBoxLevel = 3;
  bool success = false;

  planner_msgs::planner_set_planning_mode planning_mode_srv;
  if (trigger_mode_ == PlannerTriggerModeType::kAuto) {
    planning_mode_srv.request.planning_mode =
        planner_msgs::planner_set_planning_mode::Request::kAuto;
  } else {
    planning_mode_srv.request.planning_mode =
        planner_msgs::planner_set_planning_mode::Request::kManual;
  }
  planner_set_trigger_mode_client_.call(planning_mode_srv);

  for (int ind = 0; ind < kBBoxLevel; ++ind) {
    ros::Duration(0.01)
        .sleep();  // sleep to unblock the thread to get latest cmd.
    ros::spinOnce();
    if (stop_planner_request_) return;

    bound_mode_ = ind;
    ROS_INFO_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "Planning iteration %i",
                  planner_iteration_);
    planner_msgs::planner_srv plan_srv;
    plan_srv.request.header.stamp = ros::Time::now();
    plan_srv.request.header.seq = planner_iteration_;
    plan_srv.request.header.frame_id = world_frame_id_;
    plan_srv.request.bound_mode = bound_mode_;
    plan_srv.request.root_pose = getPoseToStart();
    if (planner_client_.call(plan_srv)) {
      if (!plan_srv.response.path.empty()) {
        // Execute path.
        if (exe_path) {
          if ((!force_forward_) ||
              (plan_srv.response.status != plan_srv.response.kBackward) ||
              (ind == (kBBoxLevel - 1))) {
            if (ind == (kBBoxLevel - 1))
              ROS_WARN_COND(
                  global_verbosity >= Verbosity::WARN,
                  "Using minimum bound, pick the current best one regardless "
                  "the direction.");
            current_path_.clear();
            std::vector<geometry_msgs::Pose> path_to_be_exe;
            PCIManager::ExecutionPathType path_type =
                PCIManager::ExecutionPathType::kLocalPath;
            if (plan_srv.response.status == plan_srv.response.kHoming) {
              // Perform homing step, set back to manual mode, and stop all
              // current requests.
              resetPlanner();
              path_type = PCIManager::ExecutionPathType::kHomingPath;
            }
            v_current_ = pci_manager_->getVelocity(path_type);
            // Publish the status
            publishPlannerStatus(plan_srv.response, true);
            pci_manager_->executePath(plan_srv.response.path, path_to_be_exe,
                                      path_type);
            success = true;
            current_path_ = path_to_be_exe;
          } else if (ind < (kBBoxLevel - 1)) {
            publishPlannerStatus(plan_srv.response, false);
            ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                          "Attemp to re-plan with smaller bound.");
          }
        }
      } else {
        publishPlannerStatus(plan_srv.response, false);
        ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Planner returned an empty path");
        ros::Duration(0.5).sleep();
      }
      planner_iteration_++;
      if (success) break;
    } else {
      ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "Planner service failed");
      ros::Duration(0.5).sleep();
    }
  }

  // Reset default mode again.
  bound_mode_ = 0;
}

void PlannerControlInterface::publishPlannerStatus(
    const planner_msgs::planner_srv::Response& res, bool success) {
  std_msgs::Bool planner_success_msg;
  planner_success_msg.data = success;
  planner_status_pub_.publish(planner_success_msg);
}

void PlannerControlInterface::publishGoToWaypointVisualization(
    const geometry_msgs::PoseStamped& poseStamped) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = poseStamped.header.frame_id;
  marker.header.stamp = poseStamped.header.stamp;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = poseStamped.pose;
  marker.pose.position.z += 0.5;
  marker.scale.x = 4.0;
  marker.scale.y = 0.45;
  marker.scale.z = 0.45;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  go_to_waypoint_visualization_pub_.publish(marker);
}

void PlannerControlInterface::runHoming(bool exe_path) {
  ROS_WARN_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "Start homing ...");
  planner_msgs::planner_set_planning_mode planning_mode_srv;
  planning_mode_srv.request.planning_mode =
      planner_msgs::planner_set_planning_mode::Request::kManual;
  planner_set_trigger_mode_client_.call(planning_mode_srv);

  planner_msgs::planner_homing plan_srv;
  plan_srv.request.header.stamp = ros::Time::now();
  plan_srv.request.header.seq = planner_iteration_;
  plan_srv.request.header.frame_id = world_frame_id_;
  if (planner_homing_client_.call(plan_srv)) {
    if (!plan_srv.response.path.empty()) {
      if (exe_path) {
        std::vector<geometry_msgs::Pose> path_to_be_exe;
        pci_manager_->executePath(plan_srv.response.path, path_to_be_exe,
                                  PCIManager::ExecutionPathType::kHomingPath);
        current_path_ = path_to_be_exe;
      }
    } else {
      ROS_WARN_THROTTLE(1, "Planner Homing returned an empty path");
    }
  } else {
    ROS_WARN_THROTTLE(1, "Planner Homing service failed");
    ros::Duration(0.5).sleep();
  }
  planner_iteration_++;
}

void PlannerControlInterface::runInitialization() {
  ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                "Start initialization ...");
  planner_msgs::planner_set_planning_mode planning_mode_srv;
  planning_mode_srv.request.planning_mode =
      planner_msgs::planner_set_planning_mode::Request::kManual;
  planner_set_trigger_mode_client_.call(planning_mode_srv);

  pci_manager_->initMotion();
}

void PlannerControlInterface::runSearch(bool exe_path) {
  ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Start searching ...");

  planner_msgs::planner_set_planning_mode planning_mode_srv;
  planning_mode_srv.request.planning_mode =
      planner_msgs::planner_set_planning_mode::Request::kManual;
  planner_set_trigger_mode_client_.call(planning_mode_srv);

  planner_msgs::planner_search plan_srv;
  plan_srv.request.header.stamp = ros::Time::now();
  plan_srv.request.header.seq = planner_iteration_;
  plan_srv.request.header.frame_id = world_frame_id_;
  plan_srv.request.use_current_state = use_current_state_;
  plan_srv.request.bound_mode = bound_mode_;

  if (!use_current_state_) {
    plan_srv.request.source.position.x = source_setpoint_.position.x;
    plan_srv.request.source.position.y = source_setpoint_.position.y;
    plan_srv.request.source.position.z = source_setpoint_.position.z;
    plan_srv.request.source.orientation.x = source_setpoint_.orientation.x;
    plan_srv.request.source.orientation.y = source_setpoint_.orientation.y;
    plan_srv.request.source.orientation.z = source_setpoint_.orientation.z;
    plan_srv.request.source.orientation.w = source_setpoint_.orientation.w;
  } else {
    plan_srv.request.source.position.x = source_setpoint_.position.x;
    plan_srv.request.source.position.y = source_setpoint_.position.y;
    plan_srv.request.source.position.z = source_setpoint_.position.z;
    plan_srv.request.source.orientation.x = source_setpoint_.orientation.x;
    plan_srv.request.source.orientation.y = source_setpoint_.orientation.y;
    plan_srv.request.source.orientation.z = source_setpoint_.orientation.z;
    plan_srv.request.source.orientation.w = source_setpoint_.orientation.w;
  }
  plan_srv.request.target.position.x = target_setpoint_.position.x;
  plan_srv.request.target.position.y = target_setpoint_.position.y;
  plan_srv.request.target.position.z = target_setpoint_.position.z;
  plan_srv.request.target.orientation.x = target_setpoint_.orientation.x;
  plan_srv.request.target.orientation.y = target_setpoint_.orientation.y;
  plan_srv.request.target.orientation.z = target_setpoint_.orientation.z;
  plan_srv.request.target.orientation.w = target_setpoint_.orientation.w;

  if (planner_search_client_.call(plan_srv)) {
    if (!plan_srv.response.path.empty()) {
      if (exe_path) {
        std::vector<geometry_msgs::Pose> path_to_be_exe;
        pci_manager_->executePath(plan_srv.response.path, path_to_be_exe);
        current_path_ = path_to_be_exe;
      }
    } else {
      ROS_WARN_THROTTLE(1, "Planner Search returned an empty path");
    }
  } else {
    ROS_WARN_THROTTLE(1, "Planner Search service failed");
    ros::Duration(0.5).sleep();
  }
  planner_iteration_++;
}

geometry_msgs::Pose PlannerControlInterface::getPoseToStart() {
  geometry_msgs::Pose ret;
  // use current state as default
  ret.position.x = 0.0;
  ret.position.y = 0.0;
  ret.position.z = 0.0;
  ret.orientation.x = 0.0;
  ret.orientation.y = 0.0;
  ret.orientation.z = 0.0;
  ret.orientation.w = 1.0;

  // Use the last waypoint as a starting pose if required to plan ahead
  if (pci_manager_->planAhead() && (current_path_.size()))
    ret = current_path_.back();
  return ret;
}

bool PlannerControlInterface::loadParams() {
  std::string ns = ros::this_node::getName();
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Loading: %s", ns.c_str());

  // Required params for robot interface.
  if (!pci_manager_->loadParams(ns)) return false;

  // Other params.
  std::string param_name;
  std::string parse_str;
  param_name = ns + "/trigger_mode";
  ros::param::get(param_name, parse_str);
  if (!parse_str.compare("kManual"))
    trigger_mode_ = PlannerTriggerModeType::kManual;
  else if (!parse_str.compare("kAuto"))
    trigger_mode_ = PlannerTriggerModeType::kAuto;
  else {
    trigger_mode_ = PlannerTriggerModeType::kManual;
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                  "No trigger mode setting, set it to kManual.");
  }

  param_name = ns + "/world_frame_id";
  if (!ros::param::get(param_name, world_frame_id_)) {
    world_frame_id_ = "world";
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                  "No world_frame_id setting, set it to: %s.",
                  world_frame_id_.c_str());
  }

  ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Done.");
  return true;
}

void PlannerControlInterface::odometryCallback(const nav_msgs::Odometry& odo) {
  current_pose_.position.x = odo.pose.pose.position.x;
  current_pose_.position.y = odo.pose.pose.position.y;
  current_pose_.position.z = odo.pose.pose.position.z;
  current_pose_.orientation.x = odo.pose.pose.orientation.x;
  current_pose_.orientation.y = odo.pose.pose.orientation.y;
  current_pose_.orientation.z = odo.pose.pose.orientation.z;
  current_pose_.orientation.w = odo.pose.pose.orientation.w;
  pci_manager_->setState(current_pose_);
  if (!pose_is_ready_) {
    previous_pose_ = current_pose_;
  } else {
    Eigen::Vector3d prev_state(previous_pose_.position.x,
                               previous_pose_.position.y,
                               previous_pose_.position.z);
    Eigen::Vector3d curr_state(current_pose_.position.x,
                               current_pose_.position.y,
                               current_pose_.position.z);
    const double kMinDist = 5.0;
    if ((prev_state - curr_state).norm() > kMinDist) {
      if (menu_initialized) {
        geometry_msgs::Pose new_pose;
        new_pose.position = current_pose_.position;
        new_pose.orientation.x = 0.0;
        new_pose.orientation.y = 0.0;
        new_pose.orientation.z = 0.0;
        new_pose.orientation.w = 1.0;
        semantic_server->setPose("semantic", new_pose);
        semantic_server->applyChanges();
        previous_pose_ = current_pose_;
      }
    }
  }
  pose_is_ready_ = true;
}

void PlannerControlInterface::poseCallback(
    const geometry_msgs::PoseWithCovarianceStamped& pose) {
  processPose(pose.pose.pose);
}

void PlannerControlInterface::poseStampedCallback(
    const geometry_msgs::PoseStamped& pose) {
  processPose(pose.pose);
}

void PlannerControlInterface::processPose(const geometry_msgs::Pose& pose) {
  current_pose_.position.x = pose.position.x;
  current_pose_.position.y = pose.position.y;
  current_pose_.position.z = pose.position.z;
  current_pose_.orientation.x = pose.orientation.x;
  current_pose_.orientation.y = pose.orientation.y;
  current_pose_.orientation.z = pose.orientation.z;
  current_pose_.orientation.w = pose.orientation.w;
  pci_manager_->setState(current_pose_);
  if (!pose_is_ready_) {
    previous_pose_ = current_pose_;
  } else {
    Eigen::Vector3d prev_state(previous_pose_.position.x,
                               previous_pose_.position.y,
                               previous_pose_.position.z);
    Eigen::Vector3d curr_state(current_pose_.position.x,
                               current_pose_.position.y,
                               current_pose_.position.z);
    const double kMinDist = 5.0;
    if ((prev_state - curr_state).norm() > kMinDist) {
      if (menu_initialized) {
        geometry_msgs::Pose new_pose;
        new_pose.position = current_pose_.position;
        new_pose.orientation.x = 0.0;
        new_pose.orientation.y = 0.0;
        new_pose.orientation.z = 0.0;
        new_pose.orientation.w = 1.0;
        semantic_server->setPose("semantic", new_pose);
        semantic_server->applyChanges();
        previous_pose_ = current_pose_;
      }
    }
  }
  pose_is_ready_ = true;
}

void PlannerControlInterface::initIMarker() {
  {
    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = world_frame_id_;
    int_marker.header.stamp = ros::Time::now();
    int_marker.name = source_marker_name;
    int_marker.description = "Source Pose";

    tf::Vector3 position;
    position =
        tf::Vector3(current_pose_.position.x, current_pose_.position.y, 1.5);
    tf::pointTFToMsg(position, int_marker.pose.position);

    // create a grey box marker
    visualization_msgs::Marker box_marker;
    box_marker.type = visualization_msgs::Marker::CUBE;
    box_marker.scale.x = 0.45;
    box_marker.scale.y = 0.45;
    box_marker.scale.z = 0.45;
    box_marker.color.r = 0.8;
    box_marker.color.g = 0.1;
    box_marker.color.b = 0.1;
    box_marker.color.a = 1.0;

    // create a non-interactive control which contains the box
    visualization_msgs::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.markers.push_back(box_marker);

    // add the control to the interactive marker
    int_marker.controls.push_back(box_control);

    // create a control which will move the box
    // this control does not contain any markers,
    // which will cause RViz to insert two arrows
    visualization_msgs::InteractiveMarkerControl control;

    tf::Quaternion orien(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, control.orientation);
    control.name = "move_xy";
    control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    int_marker.controls.push_back(control);

    orien = tf::Quaternion(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, control.orientation);
    control.name = "move_z";
    control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    imarker_server_->insert(int_marker);
    imarker_server_->setCallback(
        int_marker.name,
        boost::bind(&PlannerControlInterface::processFeedback, this, _1));
  }
  {
    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = world_frame_id_;
    int_marker.header.stamp = ros::Time::now();
    int_marker.name = target_marker_name;
    int_marker.description = "Target Pose";

    tf::Vector3 position;
    position = tf::Vector3(current_pose_.position.x + 1.0,
                           current_pose_.position.y, 1.5);
    tf::pointTFToMsg(position, int_marker.pose.position);

    // create a grey box marker
    visualization_msgs::Marker box_marker;
    box_marker.type = visualization_msgs::Marker::CUBE;
    box_marker.scale.x = 0.45;
    box_marker.scale.y = 0.45;
    box_marker.scale.z = 0.45;
    box_marker.color.r = 0.1;
    box_marker.color.g = 0.8;
    box_marker.color.b = 0.1;
    box_marker.color.a = 1.0;

    // create a non-interactive control which contains the box
    visualization_msgs::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.markers.push_back(box_marker);

    // add the control to the interactive marker
    int_marker.controls.push_back(box_control);

    // create a control which will move the box
    // this control does not contain any markers,
    // which will cause RViz to insert two arrows
    visualization_msgs::InteractiveMarkerControl control;

    tf::Quaternion orien(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, control.orientation);
    control.name = "move_xy";
    control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    int_marker.controls.push_back(control);

    orien = tf::Quaternion(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, control.orientation);
    control.name = "move_z";
    control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    imarker_server_->insert(int_marker);
    imarker_server_->setCallback(
        int_marker.name,
        boost::bind(&PlannerControlInterface::processFeedback, this, _1));
  }
  // 'commit' changes and send to all clients
  imarker_server_->applyChanges();
}

void PlannerControlInterface::processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
  if (!feedback->marker_name.compare(source_marker_name)) {
    // update source wp.
    source_setpoint_.position.x = feedback->pose.position.x;
    source_setpoint_.position.y = feedback->pose.position.y;
    source_setpoint_.position.z = feedback->pose.position.z;
    source_setpoint_.orientation.x = feedback->pose.orientation.x;
    source_setpoint_.orientation.y = feedback->pose.orientation.y;
    source_setpoint_.orientation.z = feedback->pose.orientation.z;
    source_setpoint_.orientation.w = feedback->pose.orientation.w;
  } else if (!feedback->marker_name.compare(target_marker_name)) {
    // update target wp.
    target_setpoint_.position.x = feedback->pose.position.x;
    target_setpoint_.position.y = feedback->pose.position.y;
    target_setpoint_.position.z = feedback->pose.position.z;
    target_setpoint_.orientation.x = feedback->pose.orientation.x;
    target_setpoint_.orientation.y = feedback->pose.orientation.y;
    target_setpoint_.orientation.z = feedback->pose.orientation.z;
    target_setpoint_.orientation.w = feedback->pose.orientation.w;
  }
}

// Semantics
void PlannerControlInterface::initSemanticIMarker() {
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = world_frame_id_;
  int_marker.header.stamp = ros::Time::now();
  int_marker.name = "semantic";
  int_marker.description = "";
  int_marker.pose.position.x = current_pose_.position.x + 1.5;
  int_marker.pose.position.y = current_pose_.position.y;
  int_marker.pose.position.z = current_pose_.position.z;
  int_marker.scale = control_size;

  visualization_msgs::InteractiveMarkerControl x_control;
  x_control.name = "x_control";
  x_control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(x_control);

  visualization_msgs::InteractiveMarkerControl y_control;
  y_control.name = "y_control";
  y_control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  y_control.orientation.x = 0;
  y_control.orientation.y = 0;
  y_control.orientation.z = 0.707;
  y_control.orientation.w = 0.707;
  int_marker.controls.push_back(y_control);

  visualization_msgs::InteractiveMarkerControl z_control;
  z_control.name = "z_control";
  z_control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  z_control.orientation.x = 0;
  z_control.orientation.y = 0.707;
  z_control.orientation.z = 0;
  z_control.orientation.w = 0.707;
  int_marker.controls.push_back(z_control);

  visualization_msgs::Marker button_box_marker;
  button_box_marker.type = visualization_msgs::Marker::CUBE;
  button_box_marker.scale.x = 1.0;
  button_box_marker.scale.y = 1.0;
  button_box_marker.scale.z = 1.0;
  button_box_marker.color.r = 0.5;
  button_box_marker.color.g = 0.5;
  button_box_marker.color.b = 0.5;
  button_box_marker.color.a = 0.65;

  visualization_msgs::InteractiveMarkerControl button_control;
  button_control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::BUTTON;
  button_control.name = "button_control";
  button_control.description = "menu_button";
  button_control.markers.push_back(button_box_marker);
  button_control.always_visible = true;

  int_marker.controls.push_back(button_control);

  semantic_server->insert(int_marker);
  semantic_server->setCallback(
      int_marker.name,
      boost::bind(&PlannerControlInterface::semanticMarkerFeedback, this, _1));

  if (!menu_initialized) {
    class_entry_handle = menu_handler.insert("Class");
    accept_entry_handle = menu_handler.insert(
        "Accept",
        boost::bind(&PlannerControlInterface::acceptButtonFeedback, this, _1));

    sub_class_entry_handle = menu_handler.insert(
        class_entry_handle, kStaircaseStr,
        boost::bind(&PlannerControlInterface::selectSemanticsFeedback, this,
                    _1));
    menu_handler.setCheckState(sub_class_entry_handle,
                               interactive_markers::MenuHandler::UNCHECKED);
    sub_class_entry_handle = menu_handler.insert(
        class_entry_handle, kDoorStr,
        boost::bind(&PlannerControlInterface::selectSemanticsFeedback, this,
                    _1));
    menu_handler.setCheckState(sub_class_entry_handle,
                               interactive_markers::MenuHandler::UNCHECKED);

    menu_initialized = true;
  }
  menu_handler.apply(*semantic_server, "semantic");

  semantic_server->applyChanges();
}

void PlannerControlInterface::semanticMarkerFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
  visualization_msgs::InteractiveMarker marker;
  semantic_server->get("semantic", marker);

  semantic_position.x = marker.pose.position.x;
  semantic_position.y = marker.pose.position.y;
  semantic_position.z = marker.pose.position.z;
}

void PlannerControlInterface::acceptButtonFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
  visualization_msgs::InteractiveMarker marker;
  semantic_server->get("semantic", marker);

  semantic_position.x = marker.pose.position.x;
  semantic_position.y = marker.pose.position.y;
  semantic_position.z = marker.pose.position.z;

  semantic_location.point = semantic_position;
  semantic_location.type.value = current_semantic_class_.value;

  semantic_pub.publish(semantic_location);
}

void PlannerControlInterface::selectSemanticsFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
  visualization_msgs::InteractiveMarker marker;
  semantic_server->get("semantic", marker);

  menu_handler.setCheckState(sub_class_entry_handle,
                             interactive_markers::MenuHandler::UNCHECKED);
  sub_class_entry_handle = feedback->menu_entry_id;
  menu_handler.setCheckState(sub_class_entry_handle,
                             interactive_markers::MenuHandler::CHECKED);
  menu_handler.reApply(*semantic_server);
  semantic_server->applyChanges();
  std::string semantic_class;
  menu_handler.getTitle(sub_class_entry_handle, semantic_class);
  if (!semantic_class.compare(kStaircaseStr))
    current_semantic_class_.value =
        planner_semantic_msgs::SemanticClass::kStaircase;
  else if (!semantic_class.compare(kDoorStr))
    current_semantic_class_.value = planner_semantic_msgs::SemanticClass::kDoor;
  else
    current_semantic_class_.value = planner_semantic_msgs::SemanticClass::kNone;
}

}  // namespace explorer
