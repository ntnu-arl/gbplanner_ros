
#include "planner_control_interface/planner_control_interface.h"

#include <chrono>
#include <thread>

namespace explorer {

PlannerControlInterface::PlannerControlInterface(
    ros::NodeHandle &nh, ros::NodeHandle &nh_private,
    std::shared_ptr<PCIManager> pci_manager)
    : nh_(nh), nh_private_(nh_private) {
  planner_status_pub_ =
      nh_.advertise<planner_msgs::PlannerStatus>("gbplanner_status", 5);

  // Pose information.
  odometry_sub_ = nh_.subscribe(
      "odometry", 1, &PlannerControlInterface::odometryCallback, this);
  pose_sub_ =
      nh_.subscribe("pose", 1, &PlannerControlInterface::poseCallback, this);
  pose_stamped_sub_ = nh_.subscribe(
      "pose_stamped", 1, &PlannerControlInterface::poseStampedCallback, this);

  // Services and several standard services accompanied for easier trigger.
  pci_server_ =
      nh_.advertiseService("planner_control_interface_trigger",
                           &PlannerControlInterface::triggerCallback, this);
  pci_std_automatic_planning_server_ = nh_.advertiseService(
      "planner_control_interface/std_srvs/automatic_planning",
      &PlannerControlInterface::stdSrvsAutomaticPlanningCallback, this);
  pci_homing_server_ = nh_.advertiseService(
      "pci_homing_trigger", &PlannerControlInterface::homingCallback, this);
  pci_std_homing_server_ = nh_.advertiseService(
      "planner_control_interface/std_srvs/homing_trigger",
      &PlannerControlInterface::stdSrvHomingCallback, this);
  pci_initialization_server_ = nh_.advertiseService(
      "pci_initialization_trigger",
      &PlannerControlInterface::initializationCallback, this);
  //
  while (!(planner_client_ = nh.serviceClient<planner_msgs::planner_srv>(
               "planner_server", true))) {  // true for persistent
    ROS_WARN("PCI: service planner_server is not available: waiting...");
    sleep(1);
  }
  ROS_INFO("PCI: connected to service planner_server.");
  while (
      !(planner_homing_client_ = nh.serviceClient<planner_msgs::planner_homing>(
            "planner_homing_server", true))) {  // true for persistent
    ROS_WARN("PCI: service planner_homing_server is not available: waiting...");
    sleep(1);
  }
  ROS_INFO("PCI: connected to service planner_homing_server.");

  pci_set_homing_pos_server_ = nh_.advertiseService(
      "pci_set_homing_pos", &PlannerControlInterface::setHomingPosCallback,
      this);
  pci_std_set_homing_pos_server_ = nh_.advertiseService(
      "planner_control_interface/std_srvs/set_homing_position_here",
      &PlannerControlInterface::stdSrvSetHomingPositionHereCallback, this);
  planner_set_homing_pos_client_ =
      nh.serviceClient<planner_msgs::planner_set_homing_pos>(
          "gbplanner/set_homing_pos");

  pci_global_server_ = nh_.advertiseService(
      "pci_global", &PlannerControlInterface::globalPlannerCallback, this);
  planner_global_client_ =
      nh.serviceClient<planner_msgs::planner_global>("gbplanner/global");
  pci_std_global_server_ = nh_.advertiseService(
      "planner_control_interface/std_srvs/global_planning",
      &PlannerControlInterface::stdSrvGlobalPlannerCallback, this);

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

  planner_set_exp_mode_client_ =
      nh_.serviceClient<planner_msgs::planner_set_exp_mode>(
          "gbplanner/set_exp_mode");
  pci_std_set_vertical_exp_server_ = nh_.advertiseService(
      "planner_control_interface/std_srvs/set_vertical_mode",
      &PlannerControlInterface::stdSrvsSetVerticalModeCallback, this);
  pci_std_set_horizontal_exp_server_ = nh_.advertiseService(
      "planner_control_interface/std_srvs/set_horizontal_mode",
      &PlannerControlInterface::stdSrvsSetHorizontalModeCallback, this);

  pci_manager_ = pci_manager;
  if (!loadParams()) {
    ROS_ERROR("Can not load params. Shut down ROS node.");
    ros::shutdown();
  }

  if (!init()) {
    ROS_ERROR("Can not initialize the node. Shut down ROS node.");
    ros::shutdown();
  }
  run();
}

bool PlannerControlInterface::stdSrvsSetVerticalModeCallback(
    std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  planner_msgs::planner_set_exp_mode set_mode_srv;
  set_mode_srv.request.exp_mode.mode =
      planner_msgs::PlanningMode::kVerticalExploration;

  if (planner_set_exp_mode_client_.call(set_mode_srv)) {
    return true;
  }
  return false;
}

bool PlannerControlInterface::stdSrvsSetHorizontalModeCallback(
    std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  planner_msgs::planner_set_exp_mode set_mode_srv;
  set_mode_srv.request.exp_mode.mode =
      planner_msgs::PlanningMode::kBasicExploration;

  if (planner_set_exp_mode_client_.call(set_mode_srv)) {
    return true;
  }
  return false;
}

void PlannerControlInterface::resetPlanner() {
  // Set back to manual mode, and stop all current requests.
  trigger_mode_ = PlannerTriggerModeType::kManual;
  run_en_ = false;
  homing_request_ = false;
  init_request_ = false;
  global_request_ = false;
  go_to_waypoint_request_ = false;

  // Remove the last waypoint to prevent the planner starts from that last wp.
  current_path_.clear();
  pci_manager_->setStatus(PCIManager::PCIStatus::kReady);
}

bool PlannerControlInterface::goToWaypointCallback(
    planner_msgs::pci_to_waypoint::Request &req,
    planner_msgs::pci_to_waypoint::Response &res) {
  go_to_waypoint_request_ = true;
  set_waypoint_.position.x = req.waypoint.position.x;
  set_waypoint_.position.y = req.waypoint.position.y;
  set_waypoint_.position.z = req.waypoint.position.z;
  set_waypoint_.orientation.x = req.waypoint.orientation.x;
  set_waypoint_.orientation.y = req.waypoint.orientation.y;
  set_waypoint_.orientation.z = req.waypoint.orientation.z;
  set_waypoint_.orientation.w = req.waypoint.orientation.w;
  return true;
}

bool PlannerControlInterface::globalPlannerCallback(
    planner_msgs::pci_global::Request &req,
    planner_msgs::pci_global::Response &res) {
  global_request_ = true;
  exe_path_en_ = !req.not_exe_path;
  bound_mode_ = req.bound_mode;
  frontier_id_ = req.id;
  pci_global_request_params_ = req;
  res.success = true;
  return true;
}

bool PlannerControlInterface::stopPlannerCallback(
    planner_msgs::pci_stop::Request &req,
    planner_msgs::pci_stop::Response &res) {
  pci_manager_->stopPCI();
  stop_planner_request_ = true;
  resetPlanner();

  res.success = true;
  ROS_WARN("[PCI] STOP PLANNER.");
  return true;
}

bool PlannerControlInterface::stdSrvStopPlannerCallback(
    std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  planner_msgs::pci_stop::Request stop_request;
  planner_msgs::pci_stop::Response stop_response;

  res.success = stopPlannerCallback(stop_request, stop_response);
  res.success &= stop_response.success;

  return true;
}

bool PlannerControlInterface::addGeofenceCallback(
    planner_msgs::pci_geofence::Request &req,
    planner_msgs::pci_geofence::Response &res) {
  planner_msgs::planner_geofence plan_srv;
  plan_srv.request.rectangles = req.rectangles;
  if (planner_geofence_client_.call(plan_srv))
    res.success = plan_srv.response.success;
  return true;
}

bool PlannerControlInterface::setHomingPosCallback(
    planner_msgs::pci_set_homing_pos::Request &req,
    planner_msgs::pci_set_homing_pos::Response &res) {
  // Bypass this request to the planner.
  planner_msgs::planner_set_homing_pos plan_srv;
  if (planner_set_homing_pos_client_.call(plan_srv)) {
    res.success = plan_srv.response.success;
  }
  return true;
}

bool PlannerControlInterface::stdSrvSetHomingPositionHereCallback(
    std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  planner_msgs::pci_set_homing_pos::Request set_homing_pos_request;
  planner_msgs::pci_set_homing_pos::Response set_homing_pos_response;

  res.success =
      setHomingPosCallback(set_homing_pos_request, set_homing_pos_response);
  res.success &= set_homing_pos_response.success;

  return true;
}

bool PlannerControlInterface::initializationCallback(
    planner_msgs::pci_initialization::Request &req,
    planner_msgs::pci_initialization::Response &res) {
  init_request_ = true;
  res.success = true;
  return true;
}

bool PlannerControlInterface::homingCallback(
    planner_msgs::pci_homing_trigger::Request &req,
    planner_msgs::pci_homing_trigger::Response &res) {
  exe_path_en_ = !req.not_exe_path;
  homing_request_ = true;
  res.success = true;
  return true;
}

bool PlannerControlInterface::stdSrvGlobalPlannerCallback(
    std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  planner_msgs::pci_global::Request global_trigger_request;
  planner_msgs::pci_global::Response global_trigger_response;

  global_trigger_request.not_exe_path = false;
  global_trigger_request.id = 0;
  global_trigger_request.not_check_frontier = false;
  global_trigger_request.ignore_time = false;

  res.success =
      globalPlannerCallback(global_trigger_request, global_trigger_response);
  res.success &= global_trigger_response.success;
  return true;
}

bool PlannerControlInterface::stdSrvHomingCallback(
    std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  planner_msgs::pci_homing_trigger::Request homing_trigger_request;
  planner_msgs::pci_homing_trigger::Response homing_trigger_response;

  homing_trigger_request.not_exe_path = false;

  res.success = homingCallback(homing_trigger_request, homing_trigger_response);
  res.success &= homing_trigger_response.success;

  return true;
}

bool PlannerControlInterface::triggerCallback(
    planner_msgs::pci_trigger::Request &req,
    planner_msgs::pci_trigger::Response &res) {
  if (pci_manager_->getStatus() == PCIManager::PCIStatus::kError) {
    ROS_WARN(
        "PCIManager is curretely in error state and cannot accept planning "
        "requests.");
    res.success = false;
  } else {
    if ((!req.set_auto) && (trigger_mode_ == PlannerTriggerModeType::kAuto)) {
      ROS_WARN("Switch to manual mode.");
      trigger_mode_ = PlannerTriggerModeType::kManual;
    } else if ((req.set_auto) &&
               (trigger_mode_ == PlannerTriggerModeType::kManual)) {
      ROS_WARN("Switch to auto mode.");
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
    std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
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

bool PlannerControlInterface::init() {
  planner_iteration_ = 0;
  homing_request_ = false;
  run_en_ = false;
  exe_path_en_ = true;
  pose_is_ready_ = false;
  planner_msgs::planner_srv plan_srv_temp;
  bound_mode_ = plan_srv_temp.request.kExtendedBound;
  force_forward_ = true;
  init_request_ = false;
  global_request_ = false;
  stop_planner_request_ = false;
  go_to_waypoint_request_ = false;
  // Wait for the system is ready.
  // For example: checking odometry is ready.
  ros::Rate rr(1);
  bool cont = true;
  while (!pose_is_ready_) {
    ROS_WARN("Waiting for odometry.");
    ros::spinOnce();
    rr.sleep();
  }
  if (!pci_manager_->initialize()) return false;
  return true;
}

void PlannerControlInterface::run() {
  ros::Rate rr(10);  // 10Hz
  bool cont = true;
  while (cont) {
    PCIManager::PCIStatus pci_status = pci_manager_->getStatus();
    if (pci_status == PCIManager::PCIStatus::kReady) {
      // Priority 1: Check if require homing.
      if (homing_request_) {
        homing_request_ = false;
        trigger_mode_ = PlannerTriggerModeType::kManual; // also unset automode
        ROS_INFO("PlannerControlInterface: Running Homing");
        runHoming(exe_path_en_);
      } // Priority 2: Check if require initialization step.
      else if (init_request_) {
        init_request_ = false;
        ROS_INFO("PlannerControlInterface: Running Initialization");
        runInitialization();
      }  // Priority 3: Stop.
      else if (stop_planner_request_) {
        stop_planner_request_ = false;
        pci_manager_->goToWaypoint(current_pose_);
      } // Priority 4: Local Planning.
      else if ((trigger_mode_ == PlannerTriggerModeType::kAuto) || (run_en_)) {
        run_en_ = false;
        ROS_INFO_STREAM(
            "PlannerControlInterface: Running Planner ("
            << std::string(trigger_mode_ == PlannerTriggerModeType::kAuto
                               ? "kAuto"
                               : "kManual")
            << ")");
        runPlanner(exe_path_en_);
      } // Priority 5: Global Planning.
      else if (global_request_) {
        global_request_ = false;
        ROS_INFO("Request the global planner.");
        runGlobalPlanner(exe_path_en_);
      }  // Priority 6: Go to waypoint.
      else if (go_to_waypoint_request_) {
        go_to_waypoint_request_ = false;
        pci_manager_->goToWaypoint(set_waypoint_);
      }
    } else if (pci_status == PCIManager::PCIStatus::kError) {
      // Reset everything to manual then wait for operator.
      resetPlanner();
    }
    cont = ros::ok();
    ros::spinOnce();
    rr.sleep();
  }
}

void PlannerControlInterface::runGlobalPlanner(bool exe_path = false) {
  ROS_INFO("Planning iteration %i", planner_iteration_);
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
  bool stop = false;

  for (int ind = 0; ind < kBBoxLevel; ++ind) {
    ros::Duration(0.01)
        .sleep();  // sleep to unblock the thread to get latest cmd.
    ros::spinOnce();
    if (stop_planner_request_) return;

    bound_mode_ = ind;
    ROS_INFO("Planning iteration %i", planner_iteration_);
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
              ROS_WARN(
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
            ROS_WARN("Attemp to re-plan with smaller bound.");
          }
        }
      } else {
        publishPlannerStatus(plan_srv.response, false);
        ROS_WARN_THROTTLE(1, "Planner returned an empty path");
        ros::Duration(0.5).sleep();
      }
      planner_iteration_++;
      if (success) break;
    } else {
      ROS_WARN_THROTTLE(1, "Planner service failed");
      ros::Duration(0.5).sleep();
    }
  }

  // Reset default mode again.
  bound_mode_ = 0;
}

void PlannerControlInterface::publishPlannerStatus(
    const planner_msgs::planner_srv::Response &res, bool success) {
  planner_msgs::PlannerStatus planner_status_msg;
  planner_status_msg.header.stamp = ros::Time::now();
  planner_status_msg.header.frame_id = world_frame_id_;

  planner_status_msg.success = success;
  planner_status_msg.trigger_mode.mode =
      static_cast<decltype(planner_status_msg.trigger_mode.mode)>(
          trigger_mode_);
  planner_status_msg.bound_mode.mode == bound_mode_;
  planner_status_msg.max_vel = v_current_;

  // feeback from planner msg: N.A
  // planner_status_msg.planning_mode = res.planning_mode;
  // planner_status_msg.exe_path_mode = res.exe_path_mode;
  planner_status_pub_.publish(planner_status_msg);
}

void PlannerControlInterface::runHoming(bool exe_path) {
  ROS_WARN("Start homing ...");
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
  ROS_WARN("Start initialization ...");
  pci_manager_->initMotion();
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
  ROS_INFO("Loading: %s", ns.c_str());

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
    ROS_WARN("No trigger mode setting, set it to kManual.");
  }

  param_name = ns + "/world_frame_id";
  if (!ros::param::get(param_name, world_frame_id_)) {
    world_frame_id_ = "world";
    ROS_WARN("No world_frame_id setting, set it to: %s.",
             world_frame_id_.c_str());
  }

  ROS_INFO("Done.");
  return true;
}

void PlannerControlInterface::odometryCallback(const nav_msgs::Odometry &odo) {
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
  }
  pose_is_ready_ = true;
}

void PlannerControlInterface::poseCallback(
    const geometry_msgs::PoseWithCovarianceStamped &pose) {
  processPose(pose.pose.pose);
}

void PlannerControlInterface::poseStampedCallback(
    const geometry_msgs::PoseStamped &pose) {
  processPose(pose.pose);
}

void PlannerControlInterface::processPose(const geometry_msgs::Pose &pose) {
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
  }
  pose_is_ready_ = true;
}
}  // namespace explorer
