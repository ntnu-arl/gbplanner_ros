#include "gbplanner/params.h"

namespace explorer {

bool SensorParamsBase::loadParams(std::string ns) {
  ROSPARAM_INFO("Loading: " + ns);
  std::string param_name;
  std::vector<double> param_val;

  std::string parse_str;
  param_name = ns + "/type";
  ros::param::get(param_name, parse_str);
  if (!parse_str.compare("kCamera"))
    type = SensorType::kCamera;
  else if (!parse_str.compare("kLidar"))
    type = SensorType::kLidar;
  else {
    ROSPARAM_ERROR(param_name);
    return false;
  }

  param_name = ns + "/max_range";
  if (!ros::param::get(param_name, max_range)) {
    max_range = 5.0;
    ROSPARAM_WARN(param_name, max_range);
  }

  param_val.clear();
  param_name = ns + "/center_offset";
  if ((!ros::param::get(param_name, param_val)) || (param_val.size() != 3)) {
    param_val.resize(3);
    param_val[0] = 0.0;
    param_val[1] = 0.0;
    param_val[2] = 0.0;
    ROSPARAM_WARN(param_name, "{0,0,0}");
  }
  center_offset << param_val[0], param_val[1], param_val[2];

  param_val.clear();
  param_name = ns + "/rotations";
  if ((!ros::param::get(param_name, param_val)) || (param_val.size() != 3)) {
    param_val.resize(3);
    param_val[0] = 0.0;
    param_val[1] = 0.0;
    param_val[2] = 0.0;
    ROSPARAM_WARN(param_name, "{0,0,0}");
  }
  rotations << param_val[0], param_val[1], param_val[2];
  std::cout << ns << " " << rotations[0] << ", " << rotations[1] << ", "
            << rotations[2] << std::endl;

  param_val.clear();
  param_name = ns + "/fov";
  if ((!ros::param::get(param_name, param_val)) || (param_val.size() != 2)) {
    param_val.resize(2);
    param_val[0] = 1.57;     // 90
    param_val[1] = 1.04719;  // 60
    ROSPARAM_WARN(param_name, "{1.57, 1.04719}");
  }
  fov << param_val[0], param_val[1];

  param_val.clear();
  param_name = ns + "/resolution";
  if ((!ros::param::get(param_name, param_val)) || (param_val.size() != 2)) {
    param_val.resize(2);
    param_val[0] = M_PI / 180.0;
    param_val[1] = M_PI / 180.0;
    ROSPARAM_WARN(param_name, param_val[0] << "," << param_val[1]);
  }
  resolution << param_val[0], param_val[1];

  param_name = ns + "/frontier_percentage_threshold";
  if (!ros::param::get(param_name, frontier_percentage_threshold)) {
    frontier_percentage_threshold = 0.1;
    ROSPARAM_WARN(param_name, frontier_percentage_threshold);
  }

  param_name = ns + "/frame_id";
  if (!ros::param::get(param_name, frame_id)) {
    frame_id = "";
    ROSPARAM_WARN(param_name, frame_id);
  }

  // Precompute some const parameters to be used later.
  // Rotation from B to S.
  rot_B2S = Eigen::AngleAxisd(rotations[0], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rotations[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rotations[2], Eigen::Vector3d::UnitX());
  rot_S2B = rot_B2S.inverse();

  // Compute the normal vectors enclosed the FOV.
  if (type == SensorType::kCamera) {
    // Compute 4 normal vectors for left, right, top, down planes.
    // First, compute 4 coner points.
    // Assume the range is along the hypotenuse of the right angle.
    double h_2 = fov[0] / 2;
    double v_2 = fov[1] / 2;
    Eigen::Vector3d pTL(cos(h_2), sin(h_2), sin(v_2));
    Eigen::Vector3d pTR(cos(h_2), -sin(h_2), sin(v_2));
    Eigen::Vector3d pBR(cos(h_2), -sin(h_2), -sin(v_2));
    Eigen::Vector3d pBL(cos(h_2), sin(h_2), -sin(v_2));
    edge_points.col(0) = pTL;
    edge_points.col(1) = pTR;
    edge_points.col(2) = pBR;
    edge_points.col(3) = pBL;
    // Compute normal vectors for 4 planes. (normalized)
    normal_vectors.col(0) = edge_points.col(0).cross(edge_points.col(1));
    normal_vectors.col(1) = edge_points.col(1).cross(edge_points.col(2));
    normal_vectors.col(2) = edge_points.col(2).cross(edge_points.col(3));
    normal_vectors.col(3) = edge_points.col(3).cross(edge_points.col(0));
    // Compute correct points based on the sensor range.
    edge_points = max_range * edge_points;
    edge_points_B = rot_B2S * edge_points;
    // Frustum endpoints in (S) for gain calculation.
    frustum_endpoints.clear();
    frustum_endpoints_B.clear();
    double h_lim_2 = fov[0] / 2;
    double v_lim_2 = fov[1] / 2;
    for (double dv = -v_lim_2; dv < v_lim_2; dv += resolution[1]) {
      for (double dh = -h_lim_2; dh < h_lim_2; dh += resolution[0]) {
        double x = max_range * cos(dh);
        double y = max_range * sin(dh);
        double z = max_range * sin(dv);
        Eigen::Vector3d ep = Eigen::Vector3d(x, y, z);
        frustum_endpoints.push_back(ep);
        Eigen::Vector3d ep_B = rot_B2S * ep + center_offset;
        frustum_endpoints_B.push_back(ep_B);
      }
    }
    ROS_INFO(
        "Computed multiray_endpoints for volumetric gain [kCamera]: [%d] "
        "points.",
        frustum_endpoints_B.size());
  } else if (type == SensorType::kLidar) {
    // Frustum endpoints in (S) for gain calculation.
    frustum_endpoints.clear();
    frustum_endpoints_B.clear();
    double h_lim_2 = fov[0] / 2;
    double v_lim_2 = fov[1] / 2;
    for (double dv = -v_lim_2; dv < v_lim_2; dv += resolution[1]) {
      for (double dh = -h_lim_2; dh < h_lim_2; dh += resolution[0]) {
        double x = max_range * cos(dh);
        double y = max_range * sin(dh);
        double z = max_range * sin(dv);
        Eigen::Vector3d ep = Eigen::Vector3d(x, y, z);
        frustum_endpoints.push_back(ep);
        Eigen::Vector3d ep_B = rot_B2S * ep + center_offset;
        frustum_endpoints_B.push_back(ep_B);
      }
    }
    ROS_INFO(
        "Computed multiray_endpoints for volumetric gain [kLidar]: [%d] "
        "points.",
        frustum_endpoints_B.size());
  }

  // Compute a number to compare to check for frontier.
  num_voxels_full_fov =
      (fov[0] / resolution[0]) * (fov[1] / resolution[1]) * max_range;

  ROSPARAM_INFO("Done.");
  return true;
}

void SensorParamsBase::getFrustumEndpoints(StateVec &state,
                                           std::vector<Eigen::Vector3d> &ep) {
  // Convert rays from B to W.
  Eigen::Vector3d origin(state[0], state[1], state[2]);
  Eigen::Matrix3d rot_W2B;
  rot_W2B = Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
  ep.clear();
  for (auto &p : frustum_endpoints_B) {
    Eigen::Vector3d p_tf = origin + rot_W2B * p;
    ep.push_back(p_tf);
  }
}

void SensorParamsBase::convertBodyToSensor(
    pcl::PointCloud<pcl::PointXYZ>::Ptr ep,
    pcl::PointCloud<pcl::PointXYZ>::Ptr ep_s) {
  ep_s->points.clear();

  // Sensor in ROS coordinate, if want to transform to real camera coordinate,
  // apply this TF
  Eigen::Matrix3d rot_S2Cam;
  rot_S2Cam = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()) *
              Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());

  for (auto p : ep->points) {
    Eigen::Vector3d ip_p(p.x, p.y, p.z);
    Eigen::Vector3d op_p = rot_S2B * (ip_p - center_offset);
    if (type == SensorType::kCamera) op_p = rot_S2Cam * op_p;
    pcl::PointXYZ data;
    data.x = op_p(0);
    data.y = op_p(1);
    data.z = op_p(2);
    ep_s->points.push_back(data);
  }
}

void SensorParamsBase::getFrustumEdges(StateVec &state,
                                       std::vector<Eigen::Vector3d> &edges) {
  Eigen::Vector3d origin(state[0], state[1], state[2]);
  Eigen::Matrix3d rot_W2B;
  rot_W2B = Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
  edges.clear();
  for (int i = 0; i < 4; ++i) {
    edges.push_back(origin + rot_W2B * edge_points_B.col(i));
  }
}

bool SensorParamsBase::isInsideFOV(StateVec &state, Eigen::Vector3d &pos) {
  // Method:
  // a) Convert a point into Sensor coordinate.
  //    Usually from World (W) -> Body (B) -> Sensor (S).
  // b) Check if it is inside FOV of sensor.
  //    Check distance to sensor first, then angle.
  //    For Camera: use normal vectors in (S) to identify.
  //    For LiDAR: compute horizontal and vertical angle in (S) to identify.

  // Transform to sensor coordinate.
  Eigen::Vector3d origin(state[0], state[1], state[2]);
  Eigen::Vector3d pos_S =
      rot_S2B * Eigen::AngleAxisd(-state[3], Eigen::Vector3d::UnitZ()) *
          (pos - origin) -
      center_offset;
  float pos_S_norm = pos_S.norm();

  // Check range.
  if (pos_S_norm > max_range) return false;

  // Check FOV angles.
  if (type == SensorType::kCamera) {
    for (int i = 0; i < 4; ++i) {
      double res = pos_S.dot(normal_vectors.col(i));
      if (res <= 0) return false;
    }
  } else if (type == SensorType::kLidar) {
    // @TODO: this might be very costly.
    float h_angle = std::atan2((float)pos_S.y(), (float)pos_S.x());
    float v_angle = std::asin((float)pos_S.z() / pos_S_norm);
    if ((std::abs(h_angle) > (fov[0] / 2)) ||
        (std::abs(v_angle) > (fov[1] / 2)))
      return false;
  } else {
    // Unsupported sensor.
    return false;
  }
  return true;
}

bool SensorParamsBase::isFrontier(double num_unknown_voxels_normalized) {
  // Given the sensor FOV
  // Number of unknow voxels perceived, scaled to map resolution (per meter).
  double unknown_percentage =
      num_unknown_voxels_normalized / num_voxels_full_fov;
  if (unknown_percentage >= frontier_percentage_threshold)
    return true;
  else
    return false;
}

bool SensorParams::loadParams(std::string ns) {
  ROSPARAM_INFO("Loading: " + ns);
  std::string param_name;
  std::vector<double> param_val;

  std::vector<std::string> parse_str_list;
  param_name = ns + "/sensor_list";
  ros::param::get(param_name, parse_str_list);
  if (parse_str_list.size() <= 0) {
    ROSPARAM_ERROR(param_name);
    return false;
  }
  sensor_list = parse_str_list;
  for (auto it = sensor_list.begin(); it != sensor_list.end(); ++it) {
    SensorParamsBase spb;
    std::string sensor_ns = ns + "/" + *it;
    if (spb.loadParams(sensor_ns)) {
      sensor.emplace(std::make_pair(*it, spb));
    } else {
      ROSPARAM_ERROR(sensor_ns);
      return false;
    }
  }

  ROSPARAM_INFO("Done.");
  return true;
}

bool RobotParams::loadParams(std::string ns) {
  ROSPARAM_INFO("Loading: " + ns);
  std::string param_name;
  std::vector<double> param_val;

  std::string parse_str;
  param_name = ns + "/type";
  ros::param::get(param_name, parse_str);
  if (!parse_str.compare("kAerialRobot"))
    type = RobotType::kAerialRobot;
  else if (!parse_str.compare("kLeggedRobot"))
    type = RobotType::kLeggedRobot;
  else {
    type = RobotType::kAerialRobot;
    ROSPARAM_WARN(ns + "/type", "kAerialRobot");
  }

  param_val.clear();
  param_name = ns + "/size";
  if ((!ros::param::get(param_name, param_val)) || (param_val.size() != 3)) {
    ROSPARAM_ERROR(param_name);
    return false;
  }
  size << param_val[0], param_val[1], param_val[2];

  param_val.clear();
  param_name = ns + "/size_extension_min";
  if ((!ros::param::get(param_name, param_val)) || (param_val.size() != 3)) {
    ROSPARAM_ERROR(param_name);
    return false;
  }
  size_extension_min << param_val[0], param_val[1], param_val[2];

  param_val.clear();
  param_name = ns + "/size_extension";
  if ((!ros::param::get(param_name, param_val)) || (param_val.size() != 3) ||
      (param_val[0] < size_extension_min[0]) ||
      (param_val[1] < size_extension_min[1]) ||
      (param_val[2] < size_extension_min[2])) {
    param_val.resize(3);
    param_val[0] = size_extension_min[0];
    param_val[1] = size_extension_min[1];
    param_val[2] = size_extension_min[2];
    ROSPARAM_WARN(param_name, "{" << param_val[0] << "," << param_val[1] << ","
                                  << param_val[2] << "}");
  }
  size_extension << param_val[0], param_val[1], param_val[2];

  param_val.clear();
  param_name = ns + "/center_offset";
  if ((!ros::param::get(param_name, param_val)) || (param_val.size() != 3)) {
    param_val.resize(3);
    param_val[0] = 0.0;
    param_val[1] = 0.0;
    param_val[2] = 0.0;
    ROSPARAM_WARN(param_name, "{0,0,0}");
  }
  center_offset << param_val[0], param_val[1], param_val[2];

  param_val.clear();
  param_name = ns + "/safety_extension";
  if ((!ros::param::get(param_name, param_val)) || (param_val.size() != 3)) {
    ROSPARAM_ERROR(param_name);
    return false;
  }
  safety_extension << param_val[0], param_val[1], param_val[2];

  param_name = ns + "/relax_ratio";
  if (!ros::param::get(param_name, relax_ratio)) {
    relax_ratio = 0.5;
    ROSPARAM_WARN(param_name, relax_ratio);
  }

  parse_str = "";
  param_name = ns + "/bound_mode";
  ros::param::get(param_name, parse_str);
  if (!parse_str.compare("kExtendedBound"))
    bound_mode = BoundModeType::kExtendedBound;
  else if (!parse_str.compare("kRelaxedBound"))
    bound_mode = BoundModeType::kRelaxedBound;
  else if (!parse_str.compare("kMinBound"))
    bound_mode = BoundModeType::kMinBound;
  else if (!parse_str.compare("kExactBound"))
    bound_mode = BoundModeType::kExactBound;
  else if (!parse_str.compare("kNoBound"))
    bound_mode = BoundModeType::kNoBound;
  else {
    bound_mode = BoundModeType::kExtendedBound;
    ROSPARAM_WARN(ns + "/bound_mode", "kExtendedBound");
  }

  ROSPARAM_INFO("Done.");
  return true;
}

void RobotParams::setBoundMode(BoundModeType bmode) {
  bound_mode = bmode;
  switch (bound_mode) {
    case BoundModeType::kExtendedBound:
      ROS_INFO("Set bound mode: kExtendedBound");
      break;
    case BoundModeType::kRelaxedBound:
      ROS_INFO("Set bound mode: kRelaxedBound");
      break;
    case BoundModeType::kMinBound:
      ROS_INFO("Set bound mode: kMinBound");
      break;
    case BoundModeType::kExactBound:
      ROS_INFO("Set bound mode: kExactBound");
      break;
    case BoundModeType::kNoBound:
      ROS_INFO("Set bound mode: kNoBound");
      break;
  }
}

void RobotParams::getPlanningSize(Eigen::Vector3d &psize) {
  psize << 0.0, 0.0, 0.0;
  switch (bound_mode) {
    case BoundModeType::kExtendedBound:
      psize = size + size_extension;
      break;
    case BoundModeType::kRelaxedBound:
      psize = size + relax_ratio * size_extension_min +
              (1 - relax_ratio) * size_extension;
      break;
    case BoundModeType::kMinBound:
      psize = size + size_extension_min;
      break;
    case BoundModeType::kExactBound:
      psize = size;
      break;
    case BoundModeType::kNoBound:
      psize << 0.0, 0.0, 0.0;
      break;
  }
}

bool BoundedSpaceParams::loadParams(std::string ns) {
  ROSPARAM_INFO("Loading: " + ns);
  std::string param_name;
  std::vector<double> param_val;

  std::string parse_str;
  param_name = ns + "/type";
  ros::param::get(param_name, parse_str);
  if (!parse_str.compare("kCuboid"))
    type = BoundedSpaceType::kCuboid;
  else if (!parse_str.compare("kSphere"))
    type = BoundedSpaceType::kSphere;
  else {
    ROSPARAM_ERROR(param_name);
    return false;
  }

  if (type == BoundedSpaceType::kCuboid) {
    param_val.clear();
    param_name = ns + "/min_val";
    if ((!ros::param::get(param_name, param_val)) || (param_val.size() != 3)) {
      ROSPARAM_ERROR(param_name);
      return false;
    }
    min_val << param_val[0], param_val[1], param_val[2];

    param_val.clear();
    param_name = ns + "/max_val";
    if ((!ros::param::get(param_name, param_val)) || (param_val.size() != 3)) {
      ROSPARAM_ERROR(param_name);
      return false;
    }
    max_val << param_val[0], param_val[1], param_val[2];

    param_val.clear();
    param_name = ns + "/min_extension";
    if ((!ros::param::get(param_name, param_val)) || (param_val.size() != 3)) {
      param_val.resize(3);
      param_val[0] = 0.0;
      param_val[1] = 0.0;
      param_val[2] = 0.0;
      ROSPARAM_WARN(param_name, "{0,0,0}");
    }
    min_extension << param_val[0], param_val[1], param_val[2];

    param_val.clear();
    param_name = ns + "/max_extension";
    if ((!ros::param::get(param_name, param_val)) || (param_val.size() != 3)) {
      param_val.resize(3);
      param_val[0] = 0.0;
      param_val[1] = 0.0;
      param_val[2] = 0.0;
      ROSPARAM_WARN(param_name, "{0,0,0}");
    }
    max_extension << param_val[0], param_val[1], param_val[2];

    param_val.clear();
    param_name = ns + "/rotations";
    if ((!ros::param::get(param_name, param_val)) || (param_val.size() != 3)) {
      param_val.resize(3);
      param_val[0] = 0.0;
      param_val[1] = 0.0;
      param_val[2] = 0.0;
      ROSPARAM_WARN(param_name, "{0,0,0}");
    }
    rotations << param_val[0], param_val[1], param_val[2];

  } else if (type == BoundedSpaceType::kSphere) {
    param_name = ns + "/radius";
    if (!ros::param::get(param_name, radius)) {
      // One could use the diagonal length of the bounding box to compute this
      // radius.
      ROSPARAM_ERROR(param_name);
      return false;
    }

    param_name = ns + "/radius_extension";
    if (!ros::param::get(param_name, radius_extension)) {
      radius_extension = 0.0;
      ROSPARAM_WARN(param_name, "0.0m");
    }
  }
  ROSPARAM_INFO("Done.");
  return true;
}

void BoundedSpaceParams::setCenter(StateVec &state, bool use_extension) {
  root_pos << state[0], state[1], state[2];
  if (use_extension) {
    min_val_total = root_pos + min_val + min_extension;
    max_val_total = root_pos + max_val + max_extension;
    radius_total = radius + radius_extension;
  } else {
    min_val_total = root_pos + min_val;
    max_val_total = root_pos + max_val;
    radius_total = radius;
  }
  Eigen::Matrix3d rot_W2B;
  rot_W2B = Eigen::AngleAxisd(rotations[0], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rotations[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rotations[2], Eigen::Vector3d::UnitX());
  rot_B2W = rot_W2B.inverse();
}

bool BoundedSpaceParams::isInsideSpace(Eigen::Vector3d &pos) {
  bool res = true;
  if (type == BoundedSpaceType::kSphere) {
    // No need to check orientation.
    Eigen::Vector3d dist = pos - root_pos;
    double dist_norm = dist.norm();
    if (dist_norm > radius_total) res = false;
  } else if (type == BoundedSpaceType::kCuboid) {
    // Have to check orientation.
    Eigen::Vector3d pos_B = root_pos + rot_B2W * (pos - root_pos);
    for (int i = 0; i < 3; ++i) {
      if ((pos_B[i] < min_val_total[i]) || (pos_B[i] > max_val_total[i])) {
        res = false;
        break;
      }
    }
  }
  return res;
}

bool PlanningParams::loadParams(std::string ns) {
  ROSPARAM_INFO("Loading: " + ns);
  std::string param_name;

  std::string parse_str;
  param_name = ns + "/type";
  ros::param::get(param_name, parse_str);
  if (!parse_str.compare("kBasicExploration"))
    type = PlanningModeType::kBasicExploration;
  else if (!parse_str.compare("kNarrowEnvExploration"))
    type = PlanningModeType::kNarrowEnvExploration;
  else {
    type = PlanningModeType::kBasicExploration;
    ROSPARAM_WARN(ns + "/type", "kBasicExploration");
  }

  parse_str = "";
  param_name = ns + "/rr_mode";
  ros::param::get(param_name, parse_str);
  if (!parse_str.compare("kTree"))
    rr_mode = RRModeType::kTree;
  else if (!parse_str.compare("kGraph"))
    rr_mode = RRModeType::kGraph;
  else {
    rr_mode = RRModeType::kGraph;
    ROSPARAM_WARN(ns + "/rr_mode", "kGraph");
  }

  std::vector<std::string> parse_str_list;
  param_name = ns + "/exp_sensor_list";
  ros::param::get(param_name, parse_str_list);
  if (parse_str_list.size() <= 0) {
    ROSPARAM_ERROR(param_name);
  } else {
    exp_sensor_list = parse_str_list;
    std::string str_tmp = "Sensors for exploration: ";
    for (int i = 0; i < exp_sensor_list.size(); ++i) {
      str_tmp += exp_sensor_list[i] + ", ";
    }
    ROSPARAM_INFO(str_tmp);
  }

  param_name = ns + "/v_max";
  if (!ros::param::get(param_name, v_max)) {
    v_max = 0.2;
    ROSPARAM_WARN(param_name, "0.2 m/s");
  }

  param_name = ns + "/v_homing_max";
  if (!ros::param::get(param_name, v_homing_max)) {
    v_homing_max = v_max;
    ROSPARAM_WARN(param_name, "0.2 m/s");
  }

  param_name = ns + "/yaw_rate_max";
  if (!ros::param::get(param_name, yaw_rate_max)) {
    yaw_rate_max = 0.4;
    ROSPARAM_WARN(param_name, "0.4 rad/s");
  }

  param_name = ns + "/yaw_tangent_correction";
  if (!ros::param::get(param_name, yaw_tangent_correction)) {
    yaw_tangent_correction = false;
    ROSPARAM_WARN(param_name, "false");
  }

  param_name = ns + "/exp_gain_voxel_size";
  if (!ros::param::get(param_name, exp_gain_voxel_size)) {
    exp_gain_voxel_size = 0.4;
    ROSPARAM_WARN(param_name, exp_gain_voxel_size);
  }

  param_name = ns + "/use_ray_model_for_volumetric_gain";
  if (!ros::param::get(param_name, use_ray_model_for_volumetric_gain)) {
    use_ray_model_for_volumetric_gain = false;
    ROSPARAM_WARN(param_name, use_ray_model_for_volumetric_gain);
  }

  param_name = ns + "/free_voxel_gain";
  if (!ros::param::get(param_name, free_voxel_gain)) {
    free_voxel_gain = 1;
    ROSPARAM_WARN(param_name, free_voxel_gain);
  }

  param_name = ns + "/occupied_voxel_gain";
  if (!ros::param::get(param_name, occupied_voxel_gain)) {
    occupied_voxel_gain = 1;
    ROSPARAM_WARN(param_name, occupied_voxel_gain);
  }

  param_name = ns + "/unknown_voxel_gain";
  if (!ros::param::get(param_name, unknown_voxel_gain)) {
    unknown_voxel_gain = 10;
    ROSPARAM_WARN(param_name, unknown_voxel_gain);
  }

  param_name = ns + "/edge_length_min";
  if (!ros::param::get(param_name, edge_length_min)) {
    edge_length_min = 0.2;
    ROSPARAM_WARN(param_name, edge_length_min);
  }

  param_name = ns + "/edge_length_max";
  if (!ros::param::get(param_name, edge_length_max)) {
    edge_length_max = 0.2;
    ROSPARAM_WARN(param_name, edge_length_max);
  }

  param_name = ns + "/num_vertices_max";
  if (!ros::param::get(param_name, num_vertices_max)) {
    num_vertices_max = 500;
    ROSPARAM_WARN(param_name, num_vertices_max);
  }

  param_name = ns + "/num_edges_max";
  if (!ros::param::get(param_name, num_edges_max)) {
    num_edges_max = 5000;
    ROSPARAM_WARN(param_name, num_edges_max);
  }

  param_name = ns + "/edge_overshoot";
  if (!ros::param::get(param_name, edge_overshoot)) {
    edge_overshoot = 0.2;
    ROSPARAM_WARN(param_name, edge_overshoot);
  }

  param_name = ns + "/num_loops_cutoff";
  if (!ros::param::get(param_name, num_loops_cutoff)) {
    num_loops_cutoff = 1000;
    ROSPARAM_WARN(param_name, num_loops_cutoff);
  }

  param_name = ns + "/num_loops_max";
  if (!ros::param::get(param_name, num_loops_max)) {
    num_loops_max = 10000;
    ROSPARAM_WARN(param_name, num_loops_max);
  }

  param_name = ns + "/nearest_range";
  if (!ros::param::get(param_name, nearest_range)) {
    nearest_range = 1.0;
    ROSPARAM_WARN(param_name, nearest_range);
  }

  param_name = ns + "/nearest_range_min";
  if (!ros::param::get(param_name, nearest_range_min)) {
    nearest_range_min = 0.5;
    ROSPARAM_WARN(param_name, nearest_range_min);
  }

  param_name = ns + "/nearest_range_max";
  if (!ros::param::get(param_name, nearest_range_max)) {
    nearest_range_max = 2.0;
    ROSPARAM_WARN(param_name, nearest_range_max);
  }

  param_name = ns + "/use_current_state";
  if (!ros::param::get(param_name, use_current_state)) {
    use_current_state = true;
    ROSPARAM_WARN(param_name, "false");
  }

  param_name = ns + "/path_length_penalty";
  if (!ros::param::get(param_name, path_length_penalty)) {
    path_length_penalty = 0.0;  // no penalty
    ROSPARAM_WARN(param_name, path_length_penalty);
  }

  param_name = ns + "/path_direction_penalty";
  if (!ros::param::get(param_name, path_direction_penalty)) {
    path_direction_penalty = 0.0;  // no penalty
    ROSPARAM_WARN(param_name, path_direction_penalty);
  }

  param_name = ns + "/traverse_length_max";
  if (!ros::param::get(param_name, traverse_length_max)) {
    traverse_length_max = edge_length_max;
    ROSPARAM_WARN(param_name, edge_length_max);
  }

  param_name = ns + "/traverse_time_max";
  if (!ros::param::get(param_name, traverse_time_max)) {
    traverse_time_max = edge_length_max / v_max;
    ROSPARAM_WARN(param_name, traverse_time_max);
  }

  param_name = ns + "/augment_free_voxels_time";
  if (!ros::param::get(param_name, augment_free_voxels_time)) {
    augment_free_voxels_time = 5;
    ROSPARAM_WARN(param_name, augment_free_voxels_time);
  }

  param_name = ns + "/z_sample_from_ground";
  if (!ros::param::get(param_name, z_sample_from_ground)) {
    z_sample_from_ground = false;
    ROSPARAM_WARN(param_name, z_sample_from_ground);
  }

  param_name = ns + "/adjust_local_sampling_direction";
  if (!ros::param::get(param_name, adjust_local_sampling_direction)) {
    adjust_local_sampling_direction = false;
    ROSPARAM_WARN(param_name, adjust_local_sampling_direction);
  }

  param_name = ns + "/free_frustum_before_planning";
  if (!ros::param::get(param_name, free_frustum_before_planning)) {
    free_frustum_before_planning = false;
    ROSPARAM_WARN(param_name, free_frustum_before_planning);
  }

  param_name = ns + "/auto_homing_enable";
  if (!ros::param::get(param_name, auto_homing_enable)) {
    auto_homing_enable = false;
    ROSPARAM_WARN(param_name, "False");
  }

  param_name = ns + "/geofence_checking_enable";
  if (!ros::param::get(param_name, geofence_checking_enable)) {
    geofence_checking_enable = false;
    ROSPARAM_WARN(param_name, "False");
  }

  param_name = ns + "/time_budget_limit";
  if (!ros::param::get(param_name, time_budget_limit)) {
    time_budget_limit = std::numeric_limits<double>::max();
    ROSPARAM_WARN(param_name, "std::numeric_limits<double>::max()");
  }

  param_name = ns + "/homing_backward";
  if (!ros::param::get(param_name, homing_backward)) {
    homing_backward = false;
    ROSPARAM_WARN(param_name, "False");
  }

  param_name = ns + "/planning_backward";
  if (!ros::param::get(param_name, planning_backward)) {
    planning_backward = false;
    ROSPARAM_WARN(param_name, "False");
  }

  param_name = ns + "/safety_aware_enable";
  safety_aware_enable = false;  // disable this feature.
  if (!ros::param::get(param_name, safety_aware_enable)) {
    safety_aware_enable = false;
    ROSPARAM_WARN(param_name, "False");
  }

  param_name = ns + "/path_safety_enhance_enable";
  if (!ros::param::get(param_name, path_safety_enhance_enable)) {
    path_safety_enhance_enable = false;
    ROSPARAM_WARN(param_name, "False");
  }

  param_name = ns + "/global_frame_id";
  if (!ros::param::get(param_name, global_frame_id)) {
    global_frame_id = "world";
    ROSPARAM_WARN(param_name, global_frame_id);
  }

  param_name = ns + "/freespace_cloud_enable";
  if (!ros::param::get(param_name, freespace_cloud_enable)) {
    freespace_cloud_enable = false;
    ROSPARAM_WARN(param_name, freespace_cloud_enable);
  }

  ROSPARAM_INFO("Done.");
  return true;
}

void PlanningParams::setPlanningMode(PlanningModeType pmode) {
  switch (pmode) {
    case PlanningModeType::kBasicExploration:
      type = PlanningModeType::kBasicExploration;
      ROS_WARN("Exploration mode is set to kBasicExploration");
      break;
    case PlanningModeType::kVerticalExploration:
      type = PlanningModeType::kVerticalExploration;
      ROS_WARN("Exploration mode is set to kVerticalExploration");
      break;
    default:
      break;
  }
}

bool RobotDynamicsParams::loadParams(std::string ns) {
  ROSPARAM_INFO("Loading: " + ns);
  std::string param_name;

  param_name = ns + "/v_max";
  if (!ros::param::get(param_name, v_max)) {
    v_max = 0.2;
    ROSPARAM_WARN(param_name, v_max);
  }

  param_name = ns + "/v_homing_max";
  if (!ros::param::get(param_name, v_homing_max)) {
    v_homing_max = v_max;
    ROSPARAM_WARN(param_name, v_homing_max);
  }

  param_name = ns + "/yaw_rate_max";
  if (!ros::param::get(param_name, yaw_rate_max)) {
    yaw_rate_max = M_PI_4 / 2.0;
    ROSPARAM_WARN(param_name, yaw_rate_max);
  }

  ROSPARAM_INFO("Done.");
  return true;
}

}  // namespace explorer
