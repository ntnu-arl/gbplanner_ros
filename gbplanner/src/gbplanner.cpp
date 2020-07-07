#include "gbplanner/gbplanner.h"

#include <nav_msgs/Path.h>

namespace explorer {

Gbplanner::Gbplanner(const ros::NodeHandle &nh,
                     const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private) {
  planner_status_ = Gbplanner::PlannerStatus::NOT_READY;

  rrg_ = new gbplanner::Rrg(nh, nh_private);
  if (!(rrg_->loadParams())) {
    ROS_ERROR("Could not load all required parameters. Shutdown ROS node.");
    ros::shutdown();
  }

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
  planner_set_global_bound_service_ = nh_.advertiseService(
      "gbplanner/set_global_bound", &Gbplanner::setGlobalBound, this);
  planner_set_exp_mode_service_ = nh_.advertiseService(
      "gbplanner/set_exp_mode", &Gbplanner::setExpMode, this);
  planner_load_graph_service_ = nh_.advertiseService(
      "gbplanner/load_graph", &Gbplanner::plannerLoadGraphCallback, this);
  planner_save_graph_service_ = nh_.advertiseService(
      "gbplanner/save_graph", &Gbplanner::plannerSaveGraphCallback, this);

  pose_subscriber_ = nh_.subscribe("pose", 100, &Gbplanner::poseCallback, this);
  pose_stamped_subscriber_ =
      nh_.subscribe("pose_stamped", 100, &Gbplanner::poseStampedCallback, this);
  odometry_subscriber_ =
      nh_.subscribe("odometry", 100, &Gbplanner::odometryCallback, this);
  robot_status_subcriber_ =
      nh_.subscribe("/robot_status", 1, &Gbplanner::robotStatusCallback, this);
}

bool Gbplanner::plannerLoadGraphCallback(
    planner_msgs::planner_string_trigger::Request &req,
    planner_msgs::planner_string_trigger::Response &res) {
  res.success = rrg_->loadGraph(req.message);
  return true;
}

bool Gbplanner::plannerSaveGraphCallback(
    planner_msgs::planner_string_trigger::Request &req,
    planner_msgs::planner_string_trigger::Response &res) {
  res.success = rrg_->saveGraph(req.message);
  return true;
}

bool Gbplanner::setExpMode(planner_msgs::planner_set_exp_mode::Request &req,
                           planner_msgs::planner_set_exp_mode::Response &res) {
  rrg_->setExpMode(req.exp_mode);
  res.success = true;
  return true;
}

bool Gbplanner::setGlobalBound(
    planner_msgs::planner_set_global_bound::Request &req,
    planner_msgs::planner_set_global_bound::Response &res) {
  if (!req.get_current_bound)
    res.success = rrg_->setGlobalBound(req.bound, req.reset_to_default);
  else
    res.success = true;

  rrg_->getGlobalBound(res.bound_ret);
  return true;
}

bool Gbplanner::plannerServiceCallback(
    planner_msgs::planner_srv::Request &req,
    planner_msgs::planner_srv::Response &res) {
  // Extract setting from the request.
  rrg_->setGlobalFrame(req.header.frame_id);
  rrg_->setBoundMode(static_cast<explorer::BoundModeType>(req.bound_mode));
  rrg_->setRootStateForPlanning(req.root_pose);

  // Start the planner.
  res.path.clear();
  if (getPlannerStatus() == Gbplanner::PlannerStatus::NOT_READY) {
    ROS_WARN("The planner is not ready.");
    return false;
  }

  rrg_->reset();
  gbplanner::Rrg::GraphStatus status = rrg_->buildGraph();
  switch (status) {
    case gbplanner::Rrg::GraphStatus::OK:
      break;
    case gbplanner::Rrg::GraphStatus::ERR_KDTREE:
      ROS_WARN("[PLANNER_ERROR] An issue occurred with kdtree data.");
      break;
    case gbplanner::Rrg::GraphStatus::ERR_NO_FEASIBLE_PATH:
      ROS_WARN("[PLANNER_ERROR] No feasible path was found.");
  }

  if (status == gbplanner::Rrg::GraphStatus::OK) {
    status = rrg_->evaluateGraph();
    switch (status) {
      case gbplanner::Rrg::GraphStatus::OK:
        break;
      case gbplanner::Rrg::GraphStatus::NO_GAIN:
        ROS_WARN("[PLANNER_ERROR] No positive gain was found.");
        break;
    }
  }

  if (status == gbplanner::Rrg::GraphStatus::OK) {
    res.path = rrg_->getBestPath(req.header.frame_id, res.status);
  } else {
    // Could not find the best path, what should I do here?
    // Possible causes:
    // a) In collision spot.
    // b) Block by surrouding obstacle in narrow space.
    // Possible options:
    // a) homing
    // b)
  }
  return true;
}

bool Gbplanner::homingServiceCallback(
    planner_msgs::planner_homing::Request &req,
    planner_msgs::planner_homing::Response &res) {
  res.path.clear();
  if (getPlannerStatus() == Gbplanner::PlannerStatus::NOT_READY) {
    ROS_WARN("The planner is not ready.");
    return false;
  }
  res.path = rrg_->getHomingPath(req.header.frame_id);
  return true;
}

bool Gbplanner::globalPlannerServiceCallback(
    planner_msgs::planner_global::Request &req,
    planner_msgs::planner_global::Response &res) {
  res.path.clear();
  if (getPlannerStatus() == Gbplanner::PlannerStatus::NOT_READY) {
    ROS_WARN("The planner is not ready.");
    return false;
  }
  res.path =
      rrg_->runGlobalPlanner(req.id, req.not_check_frontier, req.ignore_time);
  return true;
}

bool Gbplanner::setHomingPosServiceCallback(
    planner_msgs::planner_set_homing_pos::Request &req,
    planner_msgs::planner_set_homing_pos::Response &res) {
  if (getPlannerStatus() == Gbplanner::PlannerStatus::NOT_READY) {
    ROS_WARN("The planner is not ready.");
    return false;
  }
  res.success = rrg_->setHomingPos();
  return true;
}

bool Gbplanner::plannerSearchServiceCallback(
    planner_msgs::planner_search::Request &req,
    planner_msgs::planner_search::Response &res) {
  rrg_->setBoundMode(static_cast<explorer::BoundModeType>(req.bound_mode));
  res.success =
      rrg_->search(req.source, req.target, req.use_current_state, res.path);
  return true;
}

void Gbplanner::poseCallback(
    const geometry_msgs::PoseWithCovarianceStamped &pose) {
  processPose(pose.pose.pose);
}

void Gbplanner::poseStampedCallback(const geometry_msgs::PoseStamped &pose) {
  processPose(pose.pose);
}

void Gbplanner::processPose(const geometry_msgs::Pose &pose) {
  StateVec state;
  state[0] = pose.position.x;
  state[1] = pose.position.y;
  state[2] = pose.position.z;
  state[3] = tf::getYaw(pose.orientation);
  rrg_->setState(state);
}

void Gbplanner::odometryCallback(const nav_msgs::Odometry &odo) {
  StateVec state;
  state[0] = odo.pose.pose.position.x;
  state[1] = odo.pose.pose.position.y;
  state[2] = odo.pose.pose.position.z;
  state[3] = tf::getYaw(odo.pose.pose.orientation);
  rrg_->setState(state);
}

void Gbplanner::robotStatusCallback(const planner_msgs::RobotStatus &status) {
  rrg_->setTimeRemaining(status.time_remaining);
}

Gbplanner::PlannerStatus Gbplanner::getPlannerStatus() {
  if (planner_status_ == Gbplanner::PlannerStatus::READY)
    return Gbplanner::PlannerStatus::READY;

  return Gbplanner::PlannerStatus::READY;
}

}  // namespace explorer
