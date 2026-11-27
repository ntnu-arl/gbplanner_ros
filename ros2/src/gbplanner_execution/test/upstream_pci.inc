void Reference::allocateYawAlongPath(
    std::vector<geometry_msgs::msg::Pose>& path) const {
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
void Reference::interpolatePath(const std::vector<geometry_msgs::msg::Pose>& path,
                                 std::vector<geometry_msgs::msg::Pose>& path_res) {
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
        geometry_msgs::msg::Pose pose;
        tf::poseTFToMsg(poseTF, pose);
        path_res.push_back(pose);
      }
    }
  }
}
bool Reference::reconnectPath(const std::vector<geometry_msgs::msg::Pose>& path,
                               std::vector<geometry_msgs::msg::Pose>& path_new) {
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
