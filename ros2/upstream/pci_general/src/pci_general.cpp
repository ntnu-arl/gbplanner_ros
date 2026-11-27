#include "pci_general/pci_general.h"

namespace explorer {

PCIGeneral::PCIGeneral(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private)
    : PCIManager(nh, nh_private), ac_("pci_output_path", true) {
  trajectory_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
  path_pub_ = nh_.advertise<geometry_msgs::PoseArray>("pci_command_path", 10);
}

bool PCIGeneral::initialize() {
  // Init some parameters if required.
  n_seq_ = 0;
  finish_goal_ = true;
  pci_status_ = PCIStatus::kReady;
  if (run_mode_ == RunModeType::kSim) {
    // Start the initialization motion first.
    if (init_motion_enable_) {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "PCIGeneral::initialize --> manually from UI.");
    }
  } else if (run_mode_ == RunModeType::kReal) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "PCIGeneral::initialize --> manually from UI.");
  } else {
    ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "PCIGeneral::initialize --> Not support.");
  }
  if (output_type_ == OutputType::kTopic) {
    execution_timer_ = nh_.createTimer(
        ros::Duration(0.2), &PCIGeneral::executionTimerCallback, this);
  }
  return true;
}

bool PCIGeneral::initMotion() {
  // Set initialization function here.
  ros::Duration(1.0).sleep();
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Performing initialization motion");
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Current pose: %f, %f, %f", current_pose_.position.x,
           current_pose_.position.y, current_pose_.position.z);

  std::vector<geometry_msgs::Pose> init_path, exec_path;
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Robot type: %d", (int)robot_type_);
  if (robot_type_ == RobotType::kAerial) {
    {
      geometry_msgs::Pose pose;
      pose.position.x = current_pose_.position.x;
      pose.position.y = current_pose_.position.y;
      pose.position.z = current_pose_.position.z;
      pose.orientation.x = current_pose_.orientation.x;
      pose.orientation.y = current_pose_.orientation.y;
      pose.orientation.z = current_pose_.orientation.z;
      pose.orientation.w = current_pose_.orientation.w;
      init_path.push_back(pose);
    }
    {
      geometry_msgs::Pose pose;
      pose.position.x = current_pose_.position.x;
      pose.position.y = current_pose_.position.y;
      pose.position.z = current_pose_.position.z + init_z_takeoff_;
      pose.orientation.x = 0;
      pose.orientation.y = 0;
      pose.orientation.z = 0;
      pose.orientation.w = 1;
      init_path.push_back(pose);
    }
    {
      geometry_msgs::Pose pose;
      pose.position.x = current_pose_.position.x;
      pose.position.y = current_pose_.position.y;
      pose.position.z =
          current_pose_.position.z + init_z_takeoff_ - init_z_drop_;
      pose.orientation.x = 0;
      pose.orientation.y = 0;
      pose.orientation.z = 0;
      pose.orientation.w = 1;
      init_path.push_back(pose);
    }
    {
      geometry_msgs::Pose pose;
      pose.position.x = current_pose_.position.x + init_x_forward_;
      pose.position.y = current_pose_.position.y;
      pose.position.z =
          current_pose_.position.z + init_z_takeoff_ - init_z_drop_;
      pose.orientation.x = 0;
      pose.orientation.y = 0;
      pose.orientation.z = 0;
      pose.orientation.w = 1;
      init_path.push_back(pose);
    }
  } else {
    {
      geometry_msgs::Pose pose;
      pose.position.x = current_pose_.position.x;
      pose.position.y = current_pose_.position.y;
      pose.position.z = current_pose_.position.z;
      pose.orientation.x = current_pose_.orientation.x;
      pose.orientation.y = current_pose_.orientation.y;
      pose.orientation.z = current_pose_.orientation.z;
      pose.orientation.w = current_pose_.orientation.w;
      init_path.push_back(pose);
    }
    {
      geometry_msgs::Pose pose;
      pose.position.x = current_pose_.position.x + init_x_forward_;
      pose.position.y = current_pose_.position.y;
      pose.position.z = current_pose_.position.z;
      pose.orientation.x = current_pose_.orientation.x;
      pose.orientation.y = current_pose_.orientation.y;
      pose.orientation.z = current_pose_.orientation.z;
      pose.orientation.w = current_pose_.orientation.w;
      init_path.push_back(pose);
    }
  }
  executePath(init_path, exec_path, ExecutionPathType::kManualPath);
  return true;
}

visualization_msgs::MarkerArray::Ptr PCIGeneral::generateTrajectoryMarkerArray(
    const trajectory_msgs::MultiDOFJointTrajectory& traj) const {
  auto m_arr = boost::make_shared<visualization_msgs::MarkerArray>();

  visualization_msgs::Marker me;
  me.points.resize(2);
  me.header.stamp = traj.header.stamp;
  me.header.seq = traj.header.seq;
  me.header.frame_id = traj.header.frame_id;
  me.id = 0;
  me.ns = "command_trajectory_edges";
  me.type = visualization_msgs::Marker::ARROW;
  me.action = visualization_msgs::Marker::ADD;
  me.pose.position.x = 0;
  me.pose.position.y = 0;
  me.pose.position.z = 0;
  me.pose.orientation.x = 0;
  me.pose.orientation.y = 0;
  me.pose.orientation.z = 0;
  me.pose.orientation.w = 1;
  me.scale.x = 0.125;
  me.scale.y = 0.25;
  me.scale.z = 0.5;
  // Green
  me.color.r = 0.0;
  me.color.g = 0.75;
  me.color.b = 0.0;
  me.color.a = 1.0;
  me.lifetime = ros::Duration(0.0);
  me.frame_locked = false;

  visualization_msgs::Marker mo;
  mo.header.stamp = traj.header.stamp;
  mo.header.seq = traj.header.seq;
  mo.header.frame_id = traj.header.frame_id;
  mo.id = 0;
  mo.ns = "command_trajectory_orientations";
  mo.type = visualization_msgs::Marker::ARROW;
  mo.action = visualization_msgs::Marker::ADD;
  mo.scale.x = 0.75;
  mo.scale.y = 0.175;
  mo.scale.z = 0.175;
  // Yellow
  mo.color.r = 0.75;
  mo.color.g = 0.75;
  mo.color.b = 0.0;
  mo.color.a = 1.0;
  mo.lifetime = ros::Duration(0.0);
  mo.frame_locked = false;

  for (size_t i = 0; i < traj.points.size() - 1; ++i) {
    if (!m_arr->markers.empty()) {
      ++me.id;
      ++mo.id;
    }
    auto ti_v3 = traj.points.at(i).transforms.at(0).translation;
    auto tip1_v3 = traj.points.at(i + 1).transforms.at(0).translation;
    auto& me_p0 = me.points.at(0);
    me_p0.x = ti_v3.x;
    me_p0.y = ti_v3.y;
    me_p0.z = ti_v3.z;
    auto& me_p1 = me.points.at(1);
    me_p1.x = tip1_v3.x;
    me_p1.y = tip1_v3.y;
    me_p1.z = tip1_v3.z;
    m_arr->markers.push_back(me);

    mo.pose.position.x = ti_v3.x;
    mo.pose.position.y = ti_v3.y;
    mo.pose.position.z = ti_v3.z;
    auto ti_q = traj.points.at(i).transforms.at(0).rotation;
    mo.pose.orientation.x = ti_q.x;
    mo.pose.orientation.y = ti_q.y;
    mo.pose.orientation.z = ti_q.z;
    mo.pose.orientation.w = ti_q.w;
    m_arr->markers.push_back(mo);
  }
  if (!m_arr->markers.empty()) {
    ++mo.id;
  }
  auto tb_v3 = traj.points.back().transforms.at(0).translation;
  mo.pose.position.x = tb_v3.x;
  mo.pose.position.y = tb_v3.y;
  mo.pose.position.z = tb_v3.z;
  auto tb_q = traj.points.back().transforms.at(0).rotation;
  mo.pose.orientation.x = tb_q.x;
  mo.pose.orientation.y = tb_q.y;
  mo.pose.orientation.z = tb_q.z;
  mo.pose.orientation.w = tb_q.w;
  m_arr->markers.push_back(mo);

  return m_arr;
}

visualization_msgs::MarkerArray::Ptr PCIGeneral::generateTrajectoryMarkerArray(
    const std::vector<geometry_msgs::Pose>& traj) const {
  auto m_arr = boost::make_shared<visualization_msgs::MarkerArray>();

  visualization_msgs::Marker me;
  me.points.resize(2);
  me.header.stamp = ros::Time::now();
  me.header.frame_id = world_frame_id_;
  me.id = 0;
  me.ns = "command_trajectory_edges";
  me.type = visualization_msgs::Marker::ARROW;
  me.action = visualization_msgs::Marker::ADD;
  me.pose.position.x = 0;
  me.pose.position.y = 0;
  me.pose.position.z = 0;
  me.pose.orientation.x = 0;
  me.pose.orientation.y = 0;
  me.pose.orientation.z = 0;
  me.pose.orientation.w = 1;
  me.scale.x = 0.125;
  me.scale.y = 0.25;
  me.scale.z = 0.5;
  // Green
  me.color.r = 0.0;
  me.color.g = 0.75;
  me.color.b = 0.0;
  me.color.a = 1.0;
  me.lifetime = ros::Duration(0.0);
  me.frame_locked = false;

  visualization_msgs::Marker mo;
  mo.header.stamp = ros::Time::now();
  mo.header.frame_id = world_frame_id_;
  mo.id = 0;
  mo.ns = "command_trajectory_orientations";
  mo.type = visualization_msgs::Marker::ARROW;
  mo.action = visualization_msgs::Marker::ADD;
  mo.scale.x = 0.75;
  mo.scale.y = 0.175;
  mo.scale.z = 0.175;
  // Yellow
  mo.color.r = 0.75;
  mo.color.g = 0.75;
  mo.color.b = 0.0;
  mo.color.a = 1.0;
  mo.lifetime = ros::Duration(0.0);
  mo.frame_locked = false;

  for (size_t i = 0; i < traj.size() - 1; ++i) {
    m_arr->markers.empty() ?: ++me.id, ++mo.id;
    auto ti_v3 = traj.at(i).position;
    auto tip1_v3 = traj.at(i + 1).position;
    auto& me_p0 = me.points.at(0);
    me_p0.x = ti_v3.x;
    me_p0.y = ti_v3.y;
    me_p0.z = ti_v3.z;
    auto& me_p1 = me.points.at(1);
    me_p1.x = tip1_v3.x;
    me_p1.y = tip1_v3.y;
    me_p1.z = tip1_v3.z;
    m_arr->markers.push_back(me);

    mo.pose.position.x = ti_v3.x;
    mo.pose.position.y = ti_v3.y;
    mo.pose.position.z = ti_v3.z;
    auto ti_q = traj.at(i).orientation;
    mo.pose.orientation.x = ti_q.x;
    mo.pose.orientation.y = ti_q.y;
    mo.pose.orientation.z = ti_q.z;
    mo.pose.orientation.w = ti_q.w;
    m_arr->markers.push_back(mo);
  }
  m_arr->markers.empty() ?: ++mo.id;
  auto tb_v3 = traj.back().position;
  mo.pose.position.x = tb_v3.x;
  mo.pose.position.y = tb_v3.y;
  mo.pose.position.z = tb_v3.z;
  auto tb_q = traj.back().orientation;
  mo.pose.orientation.x = tb_q.x;
  mo.pose.orientation.y = tb_q.y;
  mo.pose.orientation.z = tb_q.z;
  mo.pose.orientation.w = tb_q.w;
  m_arr->markers.push_back(mo);

  return m_arr;
}

bool PCIGeneral::goToWaypoint(geometry_msgs::Pose& pose) {
  ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Going to waypoint.");
  // From current pose to the target pose.
  std::vector<geometry_msgs::Pose> path{current_pose_, pose};
  std::vector<geometry_msgs::Pose> path_to_exe;
  executePath(path, path_to_exe, ExecutionPathType::kManualPath);
  return true;
}

bool PCIGeneral::executePath(const std::vector<geometry_msgs::Pose>& path,
                             std::vector<geometry_msgs::Pose>& modified_path,
                             ExecutionPathType path_type) {
  if (path.size() == 0) return false;
  std::vector<geometry_msgs::Pose> path_new = path;

  if (path_type != ExecutionPathType::kManualPath) {
    // Only modify for path derived from auto mode.
    // Extend the path to current position if necessary to achieve better
    // transition.
    if (reconnectPath(path, path_new)) {
      ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Reconnect done");
    } else {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, 
          "Unsafe to execute this path since it is too far from the current "
          "position");
      modified_path.push_back(current_pose_);
      return false;
    }
    if (smooth_heading_enable_ && robot_type_ == RobotType::kAerial) {
      allocateYawAlongPath(path_new);
    }
  }

  // const bool turn_yaw_first = homing_yaw_allocation_enable_ &&
  //                           ((path_type == ExecutionPathType::kHomingPath) ||
  //                           (path_type == ExecutionPathType::kGlobalPath));
  // if (turn_yaw_first) {
  //   allocateYawAlongFistSegment(path_new);
  // }

  if (path_new.size() > 1) {
    std::vector<geometry_msgs::Pose> path_interp;
    interpolatePath(path_new, path_interp);
    path_new = path_interp;
  }

  // Store the current path
  executing_path_ = path_new;
  // Return the final path
  modified_path = path_new;

  // Execute the path: same interface for both simulation and real system.
  if ((run_mode_ == RunModeType::kSim) || (run_mode_ == RunModeType::kReal)) {
    if (output_type_ == OutputType::kTopic) {
      n_seq_++;
      // std::vector<geometry_msgs::Pose> path_intp;
      // interpolatePath(path_new, path_new);
      samples_array_.header.seq = n_seq_;
      samples_array_.header.stamp = ros::Time::now();
      samples_array_.header.frame_id = world_frame_id_;
      samples_array_.points.clear();
      double time_sum = 0;
      geometry_msgs::PoseArray command_path;
      command_path.header.frame_id = world_frame_id_;
      for (int i = 0; i < path_new.size(); i++) {
        double yaw = tf::getYaw(path_new[i].orientation);
        Eigen::Vector3d p(path_new[i].position.x, path_new[i].position.y,
                          path_new[i].position.z);
        trajectory_point_.position_W.x() = p.x();
        trajectory_point_.position_W.y() = p.y();
        trajectory_point_.position_W.z() = p.z();
        trajectory_point_.setFromYaw(yaw);
        mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(
            trajectory_point_, &trajectory_point_msg_);
        time_sum += dt_;
        trajectory_point_msg_.time_from_start = ros::Duration(time_sum);
        samples_array_.points.push_back(trajectory_point_msg_);
        command_path.poses.push_back(path_new[i]);
      }
      trajectory_vis_pub_.publish(
          generateTrajectoryMarkerArray(samples_array_));
      trajectory_pub_.publish(samples_array_);
      path_pub_.publish(command_path);
      pci_status_ = PCIStatus::kRunning;
    } else if (output_type_ == OutputType::kAction) {
      // Prepare to send goals to the action.
      planner_msgs::pathFollowerActionGoal goal;
      std::vector<geometry_msgs::PoseStamped> path_stamped;
      for (auto& p : path_new) {
        geometry_msgs::PoseStamped ps;
        ps.pose = p;
        path_stamped.push_back(ps);
      }
      goal.path = path_stamped;

      ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Going to send the goal");
      if (ac_.waitForServer(ros::Duration(kServerWatingTimeout))) {
        trajectory_vis_pub_.publish(generateTrajectoryMarkerArray(path_new));
        ac_.sendGoal(
            goal, boost::bind(&PCIGeneral::actionDoneCallback, this, _1, _2),
            boost::bind(&PCIGeneral::actionActiveCallback, this),
            boost::bind(&PCIGeneral::actionFeedbackCallback, this, _1));
        pci_status_ = PCIStatus::kRunning;
        ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Goal sent.");
      } else {
        ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "It was not possible to send path to action server.");
      }
    }
  } else {
    ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "PCIGeneral::executePath --> Not support.");
  }
  return true;
}

bool PCIGeneral::reconnectPath(const std::vector<geometry_msgs::Pose>& path,
                               std::vector<geometry_msgs::Pose>& path_new) {
  path_new = path;

  // Extend the path to current position if necessary to achieve better
  // transition. Check if the path starts from current pose.
  const double kLimLow = 0.1;  // all magic numbers
  const double kLimHigh = 1.5;
  Eigen::Vector3d root_pos(path[0].position.x, path[0].position.y,
                           path[0].position.z);
  Eigen::Vector3d second_pos(path[1].position.x, path[1].position.y,
                             path[1].position.z);
  Eigen::Vector3d cur_pos(current_pose_.position.x, current_pose_.position.y,
                          current_pose_.position.z);
  Eigen::Vector3d ext_seg;
  ext_seg = root_pos - cur_pos;
  double d_ext_seg = ext_seg.norm();
  if (d_ext_seg <= kLimLow) {
    // no change needed.
    path_new.insert(path_new.begin(), current_pose_);
    return true;
  } else if (d_ext_seg <= kLimHigh) {
    // Connect current pose to the path: assume that in the short distance it is
    // safe to do so. Check if the current pose and the second node in the path
    // are in the same side of the hyperplane. Compute the hyperplane
    ext_seg = ext_seg / d_ext_seg;
    double b = ext_seg.dot(root_pos);
    if ((ext_seg.dot(cur_pos) - b) * (ext_seg.dot(second_pos) - b) <= 0) {
      // different side: add current pose to the root node
    } else {
      // same side: ignore the root node, add current pose to the second one.
      path_new.erase(path_new.begin());
    }
    path_new.insert(path_new.begin(), current_pose_);
    return true;
  } else if (planAhead() && concatenate_path_enable_ &&
             (executing_path_.size() >= 1)) {
    // Set path is too far away from current position,
    // but if we want and assume that planner is in auto mode.
    // Extend previous path to this path for safety purpose.
    // First check if we are able to connect them.
    bool reconnect_ok = false;
    const double kDiffEps = 0.001;
    // Deviation from the path due to tracking error in the control.
    const double kPointToSegmentDistThreshold = 0.20;
    if (calculateDistance(executing_path_.back(), path.front()) <= kDiffEps) {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Trying to connect prev path and current path.");
      // Check if current pos belongs to any segment.
      int exe_path_size = executing_path_.size();
      for (int ind = exe_path_size - 1; ind > 0; --ind) {
        if (calculateDistance(executing_path_[ind - 1], current_pose_) <
            kLimLow) {
          path_new.insert(path_new.begin(), current_pose_);
          reconnect_ok = true;
          break;
        } else {
          // check if same direction.
          Eigen::Vector3d v1(executing_path_[ind - 1].position.x -
                                 executing_path_[ind].position.x,
                             executing_path_[ind - 1].position.y -
                                 executing_path_[ind].position.y,
                             executing_path_[ind - 1].position.z -
                                 executing_path_[ind].position.z);
          Eigen::Vector3d v2(
              current_pose_.position.x - executing_path_[ind].position.x,
              current_pose_.position.y - executing_path_[ind].position.y,
              current_pose_.position.z - executing_path_[ind].position.z);
          double v_dot_prod = v1.dot(v2);
          Eigen::Vector3d v_cross_prod = v1.cross(v2);
          if ((v_dot_prod >= 0) && (v_dot_prod < v1.squaredNorm()) &&
              (v_cross_prod.squaredNorm() / v1.norm() <
               kPointToSegmentDistThreshold)) {
            ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Belong to this segment");
            path_new.insert(path_new.begin(), current_pose_);
            reconnect_ok = true;
            break;
          } else {
            // keep adding old vertices until find the solution.
            path_new.insert(path_new.begin(), executing_path_[ind - 1]);
          }
        }
      }
    }
    return reconnect_ok;
  } else {
    return false;
  }
}

void PCIGeneral::allocateYawAlongFistSegment(
    std::vector<geometry_msgs::Pose>& path) const {
  // Return if only one vertex or less.
  if (path.size() <= 1) return;
  // Assign heading on first segment only
  // BUT: Make sure we don't consider a micro-segment for orientation
  // calculation
  for (int i = 1; i < path.size(); ++i) {
    Eigen::Vector3d dir_vec;
    dir_vec << path[i].position.x - path[i - 1].position.x,
        path[i].position.y - path[i - 1].position.y,
        path[i].position.z - path[i - 1].position.z;
    if (dir_vec.norm() > 0.5) {  // TODO: Make parameter for this value (control
                                 // how big a micro-segment is)
      double yaw_first = std::atan2(path[i].position.y - path[0].position.y,
                                    path[i].position.x - path[0].position.x);
      tf::Quaternion quat;
      quat.setEuler(0.0, 0.0, yaw_first);
      for (--i; i >= 0; --i) {
        path[i].orientation.x = quat.x();
        path[i].orientation.y = quat.y();
        path[i].orientation.z = quat.z();
        path[i].orientation.w = quat.w();
      }
      break;
    }
  }
}

void PCIGeneral::allocateYawAlongPath(
    std::vector<geometry_msgs::Pose>& path) const {
  // Return if only one vertex or less.
  if (path.size() <= 1) return;
  ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "V max: %f, yr max: %f", v_max_, yaw_rate_max_);
  // Assign new heading along each segment except the first one.
  double yaw_prev = tf::getYaw(path[0].orientation);
  for (int i = 1; i < path.size(); ++i) {
    Eigen::Vector3d dir_vec;
    dir_vec << path[i].position.x - path[i - 1].position.x,
        path[i].position.y - path[i - 1].position.y,
        path[i].position.z - path[i - 1].position.z;
    double yaw_now = tf::getYaw(path[i].orientation);

    double dyaw = yaw_now - yaw_prev;
    if (dyaw < -M_PI)
      dyaw += 2 * M_PI;
    else if (dyaw > M_PI)
      dyaw -= 2 * M_PI;
    double trans_time = dir_vec.norm() / v_max_;
    double rot_time = std::abs(dyaw / yaw_rate_max_);
    if (trans_time < rot_time) {
      // Re-assign another heading.
      yaw_now =
          yaw_prev + trans_time * yaw_rate_max_ * ((dyaw >= 0) ? 1.0 : -1.0);
      // Truncate again if neccessary.
      if (yaw_now < -M_PI)
        yaw_now += 2 * M_PI;
      else if (yaw_now > M_PI)
        yaw_now -= 2 * M_PI;
      tf::Quaternion quat;
      quat.setEuler(0.0, 0.0, yaw_now);
      path[i].orientation.x = quat.x();
      path[i].orientation.y = quat.y();
      path[i].orientation.z = quat.z();
      path[i].orientation.w = quat.w();
    }
    yaw_prev = yaw_now;
  }
}

void PCIGeneral::interpolatePath(const std::vector<geometry_msgs::Pose>& path,
                                 std::vector<geometry_msgs::Pose>& path_res) {
  path_res.clear();
  if (path.size() == 0) return;
  if (path.size() == 1) path_res.push_back(path[0]);

  for (int i = 0; i < (path.size() - 1); ++i) {
    // Interpolate each segment.
    Eigen::Vector3d start(path[i].position.x, path[i].position.y,
                          path[i].position.z);
    Eigen::Vector3d end(path[i + 1].position.x, path[i + 1].position.y,
                        path[i + 1].position.z);
    Eigen::Vector3d distance = end - start;
    double yaw_start = tf::getYaw(path[i].orientation);
    double yaw_end = tf::getYaw(path[i + 1].orientation);
    double yaw_direction = yaw_end - yaw_start;
    if (yaw_direction > M_PI) {
      yaw_direction -= 2.0 * M_PI;
    }
    if (yaw_direction < -M_PI) {
      yaw_direction += 2.0 * M_PI;
    }

    double dist_norm = distance.norm();
    double disc = std::min(dt_ * v_max_ / dist_norm,
                           dt_ * yaw_rate_max_ / abs(yaw_direction));
    // const double kEpsilon = 0.0001;

    bool int_flag = true;

    if (int_flag) {
      for (double it = 0.0; it <= 1.0; it += disc) {
        tf::Vector3 origin((1.0 - it) * start[0] + it * end[0],
                           (1.0 - it) * start[1] + it * end[1],
                           (1.0 - it) * start[2] + it * end[2]);
        double yaw = yaw_start + yaw_direction * it;
        if (yaw > M_PI) yaw -= 2.0 * M_PI;
        if (yaw < -M_PI) yaw += 2.0 * M_PI;
        tf::Quaternion quat;
        quat.setEuler(0.0, 0.0, yaw);
        tf::Pose poseTF(quat, origin);
        geometry_msgs::Pose pose;
        tf::poseTFToMsg(poseTF, pose);
        path_res.push_back(pose);
      }
    }
  }
}

void PCIGeneral::executionTimerCallback(const ros::TimerEvent& event) {
  if (pci_status_ == PCIStatus::kRunning) {
    double kGoalThres = 0.5;
    double remaining_dist = getEndPointDistanceAlongPath(executing_path_);
    if (planner_trigger_lead_time_ > 0.0) {
      if (remaining_dist / v_max_ <= planner_trigger_lead_time_) {
        triggerPlanner();
      }
    } else {
      if (remaining_dist <= kGoalThres) {
        triggerPlanner();
      }
    }
  }
}

void PCIGeneral::actionActiveCallback() { ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "PCI: Goal went active"); }

void PCIGeneral::actionFeedbackCallback(
    const planner_msgs::pathFollowerActionFeedbackConstPtr& feedback) {
  if (planner_trigger_lead_time_ > 0.0 &&
      feedback->estimated_time_remaining < planner_trigger_lead_time_) {
    triggerPlanner();
  }
}

bool PCIGeneral::actionDoneCallback(
    const actionlib::SimpleClientGoalState& state,
    const planner_msgs::pathFollowerActionResultConstPtr& result) {
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Finished Goal.");
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    triggerPlanner();
  } else {
    pci_status_ = PCIStatus::kError;
  }
  return true;
}

void PCIGeneral::triggerPlanner() {
  ROS_INFO_COND(param_verbosity >= Verbosity::INFO, "PCI: Ready to trigger the planner.");
  // legacy flag, clean later.
  finish_goal_ = true;
  // should check this status instead.
  pci_status_ = PCIStatus::kReady;
}

bool PCIGeneral::loadParams(const std::string ns) {
  std::string param_name;
  std::string parse_str;

  param_name = ns + "/run_mode";
  ros::param::get(param_name, parse_str);
  if (!parse_str.compare("kReal"))
    run_mode_ = RunModeType::kReal;
  else if (!parse_str.compare("kSim"))
    run_mode_ = RunModeType::kSim;
  else {
    run_mode_ = RunModeType::kSim;
    ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No run mode setting, set it to kSim.");
  }

  param_name = ns + "/init_motion_enable";
  if (!ros::param::get(param_name, init_motion_enable_)) {
    init_motion_enable_ = true;
    ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No motion initialization setting, set it to True.");
  }

  param_name = ns + "/world_frame_id";
  if (!ros::param::get(param_name, world_frame_id_)) {
    world_frame_id_ = "world";
    ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No world_frame_id setting, set it to: %s ",
             world_frame_id_.c_str());
  }

  param_name = ns + "/init_motion/z_takeoff";
  if (!ros::param::get(param_name, init_z_takeoff_)) {
    init_z_takeoff_ = 0.0;
    ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No init_motion/z_takeoff setting, set it to 0.0 (s).");
  }
  param_name = ns + "/init_motion/z_drop";
  if (!ros::param::get(param_name, init_z_drop_)) {
    init_z_drop_ = 0.0;
    ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No init_motion/z_drop setting, set it to 0.0 (s).");
  }
  param_name = ns + "/init_motion/x_forward";
  if (!ros::param::get(param_name, init_x_forward_)) {
    init_x_forward_ = 0.0;
    ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No init_motion/x_forward setting, set it to 0.0 (s).");
  }

  param_name = ns + "/RobotDynamics/v_max";
  if (!ros::param::get(param_name, v_max_)) {
    v_max_ = 0.2;
    ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No v_max setting, set it to 0.2 (m/s).");
  }

  param_name = ns + "/RobotDynamics/v_init_max";
  if (!ros::param::get(param_name, v_init_max_)) {
    v_init_max_ = 0.2;
    ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No v_max setting, set it to 0.2 (m/s).");
  }

  param_name = ns + "/RobotDynamics/v_homing_max";
  if (!ros::param::get(param_name, v_homing_max_)) {
    v_homing_max_ = v_max_;
    ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No v_max_homing setting, set it to %f (m/s).", v_homing_max_);
  }

  param_name = ns + "/RobotDynamics/v_narrow_env_max";
  if (!ros::param::get(param_name, v_narrow_env_max_)) {
    v_narrow_env_max_ = v_max_;
    ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No v_narrow_env_max setting, set it to %f (m/s).",
             v_narrow_env_max_);
  }

  param_name = ns + "/RobotDynamics/yaw_rate_max";
  if (!ros::param::get(param_name, yaw_rate_max_)) {
    yaw_rate_max_ = M_PI_4 / 2.0;
    ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No yaw_rate_max setting, set it to PI/8 (rad/s)");
  }

  param_name = ns + "/RobotDynamics/dt";
  if (!ros::param::get(param_name, dt_)) {
    dt_ = 0.1;
    ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No dt setting, set it to 0.1 (s).");
  }

  param_name = ns + "/planner_trigger_lead_time";
  if (!ros::param::get(param_name, planner_trigger_lead_time_) ||
      (planner_trigger_lead_time_ <= 0.0)) {
    planner_trigger_lead_time_ = 0.0;
    ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No planner_trigger_lead_time setting, set it to 0.0 (s).");
  } else {
    ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "planner_trigger_lead_time_: %f", planner_trigger_lead_time_);
  }

  param_name = ns + "/smooth_heading_enable";
  if (!ros::param::get(param_name, smooth_heading_enable_)) {
    smooth_heading_enable_ = true;
    ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No smooth_heading_enable_ setting, set it to: True.");
  }

  param_name = ns + "/homing_yaw_allocation_enable";
  if (!ros::param::get(param_name, homing_yaw_allocation_enable_)) {
    homing_yaw_allocation_enable_ = true;
    ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No homing_yaw_allocation_enable setting, set it to: True.");
  }

  param_name = ns + "/use_action_server_path_manager";
  if (!ros::param::get(param_name, use_action_client_path_manager_)) {
    use_action_client_path_manager_ = "false";
    ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No use_action_server_path_manager setting, set it to: false.");
  }

  param_name = ns + "/robot_type";
  std::string robot_type_in;
  if (!ros::param::get(param_name, robot_type_in)) {
    robot_type_ = RobotType::kAerial;
    ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No robot_type setting, set it to: kAerial.");
  } else {
    if (robot_type_in == "kAerial") {
      robot_type_ = RobotType::kAerial;
    } else if (robot_type_in == "kGround") {
      robot_type_ = RobotType::kGround;
    } else {
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "Unknown Robot Type %s, setting it to: kAerial.",
               robot_type_in.c_str());
      robot_type_ = RobotType::kAerial;
    }
  }

  param_name = ns + "/output_type";
  std::string output_type_in;
  if (!ros::param::get(param_name, output_type_in)) {
    output_type_ = OutputType::kTopic;
    ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "No output_type setting, set it to: kAerial.");
  } else {
    if (output_type_in == "kTopic") {
      output_type_ = OutputType::kTopic;
    } else if (output_type_in == "kAction") {
      output_type_ = OutputType::kAction;
    } else {
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN, "Unknown Robot Type %s, setting it to: kAerial.",
               output_type_in.c_str());
      output_type_ = OutputType::kTopic;
    }
  }

  return true;
}

void PCIGeneral::setState(const geometry_msgs::Pose& pose) {
  current_pose_.position.x = pose.position.x;
  current_pose_.position.y = pose.position.y;
  current_pose_.position.z = pose.position.z;
  current_pose_.orientation.x = pose.orientation.x;
  current_pose_.orientation.y = pose.orientation.y;
  current_pose_.orientation.z = pose.orientation.z;
  current_pose_.orientation.w = pose.orientation.w;
}

void PCIGeneral::setVelocity(double v) {
  if ((v >= kVelMin) && (v <= kVelMax)) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Changed velocity from %f to %f (m/s)", v_max_, v);
    v_max_ = v;
  }
}

double PCIGeneral::getEndPointDistanceAlongPath(
    const std::vector<geometry_msgs::Pose>& path) {
  std::vector<double> dists;
  for (int i = 0; i < path.size(); ++i) {
    dists.push_back(calculateDistance(current_pose_, path[i]));
  }
  int closest_waypoint_ind =
      std::min_element(dists.begin(), dists.end()) - dists.begin();

  double total_dist = 0.0;
  total_dist += calculateDistance(current_pose_, path[closest_waypoint_ind]);
  for (int i = closest_waypoint_ind + 1; i < path.size(); ++i) {
    total_dist += calculateDistance(path[i], path[i - 1]);
  }
  return total_dist;
}

double PCIGeneral::calculateDistance(const geometry_msgs::Pose& p1,
                                     const geometry_msgs::Pose& p2) {
  Eigen::Vector3d v1(p1.position.x, p1.position.y, p1.position.z);
  Eigen::Vector3d v2(p2.position.x, p2.position.y, p2.position.z);

  return (v1 - v2).head(2).norm();
}

}  // namespace explorer
