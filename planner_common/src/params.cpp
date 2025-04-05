#include "planner_common/params.h"

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

  parse_str.clear();
  param_name = ns + "/CameraType";
  if (type == SensorType::kCamera) {
    ros::param::get(param_name, parse_str);
    if (!parse_str.compare("kFixed"))
      camera_type = CameraType::kFixed;
    else if (!parse_str.compare("kRotating"))
      camera_type = CameraType::kRotating;
    else if (!parse_str.compare("kZoom"))
      camera_type = CameraType::kZoom;
    else if (!parse_str.compare("kRotatingZoom"))
      camera_type = CameraType::kRotatingZoom;
    else {
      ROSPARAM_ERROR(param_name);
      return false;
    }
  } else {
    camera_type = CameraType::kFixed;
  }

  param_name = ns + "/callback_topic";
  ros::param::get(param_name, parse_str);
  callback_topic = parse_str;

  param_name = ns + "/focal_length_topic";
  ros::param::get(param_name, parse_str);
  focal_lenght_topic = parse_str;

  param_name = ns + "/min_range";
  if (!ros::param::get(param_name, min_range)) {
    min_range = 0.0;
    ROSPARAM_WARN(param_name, min_range);
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
  // std::cout << ns << " " << rotations[0] << ", " << rotations[1] << ", "
  //           << rotations[2] << std::endl;

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

  param_val.clear();
  param_name = ns + "/rot_lims";
  if ((!ros::param::get(param_name, param_val)) || (param_val.size() != 2)) {
    param_val.resize(2);
    param_val[0] = 0.0;
    param_val[1] = 0.0;
    ROSPARAM_WARN(param_name, param_val[0] << "," << param_val[1]);
  }
  rot_lims << param_val[0], param_val[1];

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

  param_name = ns + "/width";
  if (!ros::param::get(param_name, width)) {
    width = 0;
    ROSPARAM_WARN(param_name, frame_id);
  }

  param_name = ns + "/height";
  if (!ros::param::get(param_name, height)) {
    height = 0;
    ROSPARAM_WARN(param_name, frame_id);
  }

  param_name = ns + "/width_removal";
  if (!ros::param::get(param_name, widthRemoval)) {
    widthRemoval = 0;
    ROSPARAM_WARN(param_name, frame_id);
  }

  param_name = ns + "/height_removal";
  if (!ros::param::get(param_name, heightRemoval)) {
    heightRemoval = 0;
    ROSPARAM_WARN(param_name, frame_id);
  }

  // Precompute some const parameters to be used later.
  // Rotation from B to S.
  rot_B2S = Eigen::AngleAxisd(rotations[0], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rotations[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rotations[2], Eigen::Vector3d::UnitX());
  rot_S2B = rot_B2S.inverse();

  double v_res, h_res;
  v_res = (resolution[1] > 0) ? resolution[1] : (1.0 * M_PI / 180.0);
  h_res = (resolution[0] > 0) ? resolution[0] : (1.0 * M_PI / 180.0);
  // Compute the normal vectors enclosed the FOV.
  if (type == SensorType::kCamera) {
    if (width == 0 && height == 0) {
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
      int w = 0, h = 0;
      height = 0;
      width = 0;
      double h_lim_2 = fov[0] / 2;
      double v_lim_2 = fov[1] / 2;
      for (double dv = -v_lim_2; dv < v_lim_2; dv += v_res) {
        ++h;
        for (double dh = -h_lim_2; dh < h_lim_2; dh += h_res) {
          if (width == 0) {
            ++w;
          }
          double x = max_range * cos(dh);
          double y = max_range * sin(dh);
          double z = max_range * sin(dv);
          Eigen::Vector3d ep = Eigen::Vector3d(x, y, z);
          frustum_endpoints.push_back(ep);
          Eigen::Vector3d ep_B = rot_B2S * ep + center_offset;
          frustum_endpoints_B.push_back(ep_B);
        }
        if (width == 0) {
          width = w;
        }
      }
      height = h;
      ROS_INFO_COND(global_verbosity >= Verbosity::INFO, 
          "Computed multiray_endpoints for volumetric gain [kCamera]: [%d] "
          "points.",
          frustum_endpoints_B.size());
    }
  } else if (type == SensorType::kLidar) {
    // Frustum endpoints in (S) for gain calculation.
    frustum_endpoints.clear();
    frustum_endpoints_B.clear();
    int w = 0, h = 0;
    height = 0;
    width = 0;
    double h_lim_2 = fov[0] / 2;
    double v_lim_2 = fov[1] / 2;
    for (double dv = -v_lim_2; dv < v_lim_2; dv += v_res) {
      ++h;
      for (double dh = -h_lim_2; dh < h_lim_2; dh += h_res) {
        if (width == 0) {
          ++w;
        }
        double x = max_range * cos(dh);
        double y = max_range * sin(dh);
        double z = max_range * sin(dv);
        Eigen::Vector3d ep = Eigen::Vector3d(x, y, z);
        frustum_endpoints.push_back(ep);
        Eigen::Vector3d ep_B = rot_B2S * ep + center_offset;
        frustum_endpoints_B.push_back(ep_B);
      }
      if (width == 0) {
        width = w;
      }
    }
    height = h;
    ROS_INFO_COND(global_verbosity >= Verbosity::INFO, 
        "Computed multiray_endpoints for volumetric gain [kLidar]: [%d] "
        "points.",
        frustum_endpoints_B.size());
  }

  // Compute a number to compare to check for frontier.
  num_voxels_full_fov = (fov[0] / h_res) * (fov[1] / v_res) * max_range;

  ROSPARAM_INFO("Done.");
  return true;
}

void SensorParamsBase::getFrustumEndpoints(StateVec& state,
                                           std::vector<Eigen::Vector3d>& ep) {
  // Convert rays from B to W.
  Eigen::Vector3d origin(state[0], state[1], state[2]);
  Eigen::Matrix3d rot_W2B;
  // rot_W2B = Eigen::AngleAxisd(state[4], Eigen::Vector3d::UnitY()) *
  //           Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) *
  //           Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
  rot_W2B = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
  rot_W2B = rot_W2B * Eigen::AngleAxisd(state[4], Eigen::Vector3d::UnitY());
  ep.clear();
  for (auto& p : frustum_endpoints_B) {
    Eigen::Vector3d p_tf = origin + rot_W2B * p;
    ep.push_back(p_tf);
  }
}

void SensorParamsBase::getFrustumEndpoints(StateVec& state,
                                           std::vector<Eigen::Vector3d>& ep,
                                           float darkness_range) {
  // Convert rays from B to W.
  Eigen::Vector3d origin(state[0], state[1], state[2]);
  Eigen::Matrix3d rot_W2B;
  // rot_W2B = Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) *
  //           Eigen::AngleAxisd(state[4], Eigen::Vector3d::UnitY()) *
  //           Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
  rot_W2B = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
  rot_W2B = rot_W2B * Eigen::AngleAxisd(state[4], Eigen::Vector3d::UnitY());
  ep.clear();
  for (auto& p : frustum_endpoints_B) {
    Eigen::Vector3d p_tf = origin + rot_W2B * p * darkness_range;
    ep.push_back(p_tf);
  }
}

void SensorParamsBase::updateFrustumEndpoints() {
  rot_B2S = Eigen::AngleAxisd(rotations[0], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rotations[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rotations[2], Eigen::Vector3d::UnitX());
  rot_S2B = rot_B2S.inverse();
  // ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "[PARAMS]: fov: h: %f v: %f", fov[0], fov[1]);
  double v_res, h_res;
  v_res = (resolution[1] > 0) ? resolution[1] : (1.0 * M_PI / 180.0);
  h_res = (resolution[0] > 0) ? resolution[0] : (1.0 * M_PI / 180.0);
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
    for (double dv = -v_lim_2; dv < v_lim_2; dv += v_res) {
      for (double dh = -h_lim_2; dh < h_lim_2; dh += h_res) {
        double x = max_range * cos(dh);
        double y = max_range * sin(dh);
        double z = max_range * sin(dv);
        Eigen::Vector3d ep = Eigen::Vector3d(x, y, z);
        frustum_endpoints.push_back(ep);
        Eigen::Vector3d ep_B = rot_B2S * ep + center_offset;
        frustum_endpoints_B.push_back(ep_B);
      }
    }
    // ROS_INFO_COND(global_verbosity >= Verbosity::INFO, 
    //     "Computed multiray_endpoints for volumetric gain [kCamera]: [%d] "
    //     "points.",
    //     frustum_endpoints_B.size());
  } else if (type == SensorType::kLidar) {
    // Frustum endpoints in (S) for gain calculation.
    frustum_endpoints.clear();
    frustum_endpoints_B.clear();
    double h_lim_2 = fov[0] / 2;
    double v_lim_2 = fov[1] / 2;
    for (double dv = -v_lim_2; dv < v_lim_2; dv += v_res) {
      for (double dh = -h_lim_2; dh < h_lim_2; dh += h_res) {
        double x = max_range * cos(dh);
        double y = max_range * sin(dh);
        double z = max_range * sin(dv);
        Eigen::Vector3d ep = Eigen::Vector3d(x, y, z);
        frustum_endpoints.push_back(ep);
        Eigen::Vector3d ep_B = rot_B2S * ep + center_offset;
        frustum_endpoints_B.push_back(ep_B);
      }
    }
    // ROS_INFO_COND(global_verbosity >= Verbosity::INFO, 
    //     "Computed multiray_endpoints for volumetric gain [kLidar]: [%d] "
    //     "points.",
    //     (int)frustum_endpoints_B.size());
  }
  else if(type == SensorType::kSpherical) {
    frustum_endpoints.clear();
    frustum_endpoints_B.clear();
    double h_lim_2 = fov[0] / 2;
    double v_lim_2 = fov[1] / 2;
    for (double dv = -v_lim_2; dv < v_lim_2; dv += v_res) {
      for (double dh = -h_lim_2; dh < h_lim_2; dh += h_res) {
        double z = max_range * sin(dv);
        double x = max_range * cos(dv) * cos(dh);
        double y = max_range * cos(dv) * sin(dh);
        Eigen::Vector3d ep = Eigen::Vector3d(x, y, z);
        frustum_endpoints.push_back(ep);
        Eigen::Vector3d ep_B = rot_B2S * ep + center_offset;
        frustum_endpoints_B.push_back(ep_B);
      }
    }
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

void SensorParamsBase::getFrustumEdges(StateVec& state,
                                       std::vector<Eigen::Vector3d>& edges) {
  Eigen::Vector3d origin(state[0], state[1], state[2]);
  Eigen::Matrix3d rot_W2B;
  rot_W2B = Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(state[4], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
  edges.clear();
  for (int i = 0; i < 4; ++i) {
    edges.push_back(origin + rot_W2B * edge_points_B.col(i));
  }
}

bool SensorParamsBase::isInsideFOV(StateVec& state, Eigen::Vector3d& pos) {
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
      rot_S2B * Eigen::AngleAxisd(-state[3], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(-state[4], Eigen::Vector3d::UnitY()) *
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
  else if (!parse_str.compare("kGroundRobot"))
    type = RobotType::kGroundRobot;
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
      ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Set bound mode: kExtendedBound");
      break;
    case BoundModeType::kRelaxedBound:
      ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Set bound mode: kRelaxedBound");
      break;
    case BoundModeType::kMinBound:
      ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Set bound mode: kMinBound");
      break;
    case BoundModeType::kExactBound:
      ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Set bound mode: kExactBound");
      break;
    case BoundModeType::kNoBound:
      ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Set bound mode: kNoBound");
      break;
  }
}

void RobotParams::getPlanningSize(Eigen::Vector3d& psize) {
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

    root_pos = Eigen::Vector3d::Zero();

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
    min_val_total = min_val + min_extension;
    max_val_total = max_val + max_extension;

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

    this->setRotation(rotations);

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

void BoundedSpaceParams::setCenter(StateVec& state, bool use_extension) {
  root_pos << state[0], state[1], state[2];
  if (use_extension) {
    min_val_total = min_val + min_extension;
    max_val_total = max_val + max_extension;
    radius_total = radius + radius_extension;
  } else {
    min_val_total = min_val;
    max_val_total = max_val;
    radius_total = radius;
  }
  Eigen::Matrix3d rot_W2B;
  rot_W2B = Eigen::AngleAxisd(rotations[0], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rotations[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rotations[2], Eigen::Vector3d::UnitX());
  rot_B2W = rot_W2B.transpose();
}

void BoundedSpaceParams::setCenter(Eigen::Vector3d& root, bool use_extension) {
  root_pos = root;
  if (use_extension) {
    min_val_total = min_val + min_extension;
    max_val_total = max_val + max_extension;
    radius_total = radius + radius_extension;
  } else {
    min_val_total = min_val;
    max_val_total = max_val;
    radius_total = radius;
  }
  Eigen::Matrix3d rot_W2B;
  rot_W2B = Eigen::AngleAxisd(rotations[0], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rotations[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rotations[2], Eigen::Vector3d::UnitX());
  rot_B2W = rot_W2B.transpose();
}

void BoundedSpaceParams::setBound(Eigen::Vector3d& min_val_in,
                                  Eigen::Vector3d& max_val_in) {
  min_val = min_val_in;
  max_val = max_val_in;
}

void BoundedSpaceParams::setRotation(Eigen::Vector3d& rotations_in) {
  rotations = rotations_in;

  Eigen::Matrix3d rot_W2B;
  rot_W2B = Eigen::AngleAxisd(rotations[0], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rotations[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rotations[2], Eigen::Vector3d::UnitX());
  rot_B2W = rot_W2B.transpose();
}

bool BoundedSpaceParams::isInsideSpace(Eigen::Vector3d& pos) {
  bool res = true;
  if (type == BoundedSpaceType::kSphere) {
    // No need to check orientation.
    Eigen::Vector3d dist = pos - root_pos;
    double dist_norm = dist.norm();
    if (dist_norm > radius_total) res = false;
  } else if (type == BoundedSpaceType::kCuboid) {
    // Have to check orientation.
    Eigen::Vector3d pos_B = rot_B2W * (pos - root_pos);
    for (int i = 0; i < 3; ++i) {
      if ((pos_B[i] < min_val[i]) || (pos_B[i] > max_val[i])) {
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
  else if (!parse_str.compare("kAdaptiveExploration"))
    type = PlanningModeType::kAdaptiveExploration;
  else {
    type = PlanningModeType::kBasicExploration;
    ROSPARAM_WARN(ns + "/type", "kBasicExploration");
  }

  parse_str = "";
  param_name = ns + "/graph_building_mode";
  ros::param::get(param_name, parse_str);
  if (!parse_str.compare("kBatch"))
    graph_building_mode = GraphBuildingModeType::kBatch;
  else if (!parse_str.compare("kBasic"))
    graph_building_mode = GraphBuildingModeType::kBasic;
  else {
    graph_building_mode = GraphBuildingModeType::kBasic;
    ROSPARAM_WARN(ns + "/graph_building_mode", "kBasic");
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
    ROSPARAM_WARN(param_name, "");
  } else {
    exp_sensor_list = parse_str_list;
    std::string str_tmp = "Sensors for exploration: ";
    for (int i = 0; i < exp_sensor_list.size(); ++i) {
      str_tmp += exp_sensor_list[i] + ", ";
    }
    ROSPARAM_INFO(str_tmp);
  }

  parse_str_list.clear();
  param_name = ns + "/inspection_sensor_list";
  ros::param::get(param_name, parse_str_list);
  if (parse_str_list.size() <= 0) {
    ROSPARAM_WARN(param_name, "");
  } else {
    inspection_sensor_list = parse_str_list;
    std::string str_tmp = "Sensors for inspection: ";
    for (int i = 0; i < inspection_sensor_list.size(); ++i) {
      str_tmp += inspection_sensor_list[i] + ", ";
    }
    ROSPARAM_INFO(str_tmp);
  }

  param_name = ns + "/no_gain_zones_list";
  parse_str_list.clear();
  ros::param::get(param_name, parse_str_list);
  if (parse_str_list.size() <= 0) {
    ROSPARAM_WARN(param_name, "");
  } else {
    no_gain_zones_list = parse_str_list;
    std::string str_tmp = "No gain zones: ";
    for (int i = 0; i < no_gain_zones_list.size(); ++i) {
      str_tmp += no_gain_zones_list[i] + ", ";
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

  param_name = ns + "/hanging_vertex_penalty";
  if (!ros::param::get(param_name, hanging_vertex_penalty)) {
    hanging_vertex_penalty = 0.0;  // no penalty
    ROSPARAM_WARN(param_name, hanging_vertex_penalty);
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

  param_name = ns + "/leafs_only_for_volumetric_gain";
  if (!ros::param::get(param_name, leafs_only_for_volumetric_gain)) {
    leafs_only_for_volumetric_gain = false;
    ROSPARAM_WARN(param_name, leafs_only_for_volumetric_gain);
  }

  param_name = ns + "/max_ground_height";
  if (!ros::param::get(param_name, max_ground_height)) {
    max_ground_height = 1.2;
    ROSPARAM_WARN(param_name, max_ground_height);
  }

  param_name = ns + "/robot_height";
  if (!ros::param::get(param_name, robot_height)) {
    robot_height = 1.0;
    ROSPARAM_WARN(param_name, robot_height);
  }

  param_name = ns + "/max_inclination";
  if (!ros::param::get(param_name, max_inclination)) {
    max_inclination = 0.52;
    ROSPARAM_WARN(param_name, max_inclination);
  }

  param_name = ns + "/cluster_vertices_for_gain";
  if (!ros::param::get(param_name, cluster_vertices_for_gain)) {
    cluster_vertices_for_gain = false;
    ROSPARAM_WARN(param_name, cluster_vertices_for_gain);
  }

  param_name = ns + "/clustering_radius";
  if (!ros::param::get(param_name, clustering_radius)) {
    clustering_radius = 2.0;
    ROSPARAM_WARN(param_name, clustering_radius);
  }

  param_name = ns + "/path_interpolation_distance";
  if (!ros::param::get(param_name, path_interpolation_distance)) {
    path_interpolation_distance = 0.5;
    ROSPARAM_WARN(param_name, path_interpolation_distance);
  }

  param_name = ns + "/auto_global_planner_enable";
  if (!ros::param::get(param_name, auto_global_planner_enable)) {
    auto_global_planner_enable = true;
    ROSPARAM_WARN(param_name, auto_global_planner_enable);
  }

  param_name = ns + "/relaxed_corridor_multiplier";
  if (!ros::param::get(param_name, relaxed_corridor_multiplier)) {
    relaxed_corridor_multiplier = 1.0;
    ROSPARAM_WARN(param_name, relaxed_corridor_multiplier);
  }

  param_name = ns + "/interpolate_projection_distance";
  if (!ros::param::get(param_name, interpolate_projection_distance)) {
    interpolate_projection_distance = false;
    ROSPARAM_WARN(param_name, interpolate_projection_distance);
  }

  param_name = ns + "/go_home_if_fully_explored";
  if (!ros::param::get(param_name, go_home_if_fully_explored)) {
    go_home_if_fully_explored = false;
    ROSPARAM_WARN(param_name, go_home_if_fully_explored);
  }

  param_name = ns + "/ray_cast_step_size_multiplier";
  if (!ros::param::get(param_name, ray_cast_step_size_multiplier)) {
    ray_cast_step_size_multiplier = 1.0;
    ROSPARAM_WARN(param_name, ray_cast_step_size_multiplier);
  }

  param_name = ns + "/nonuniform_ray_cast";
  if (!ros::param::get(param_name, nonuniform_ray_cast)) {
    nonuniform_ray_cast = true;
    ROSPARAM_WARN(param_name, nonuniform_ray_cast);
  }

  param_name = ns + "/time_budget_before_landing";
  if (!ros::param::get(param_name, time_budget_before_landing)) {
    time_budget_before_landing = time_budget_limit;
    ROSPARAM_WARN(param_name, time_budget_before_landing);
  }

  param_name = ns + "/auto_landing_enable";
  if (!ros::param::get(param_name, auto_landing_enable)) {
    auto_landing_enable = false;
    ROSPARAM_WARN(param_name, auto_landing_enable);
  }

  param_name = ns + "/use_camera_gain";
  if (!ros::param::get(param_name, use_camera_gain)) {
    use_camera_gain = false;
    ROSPARAM_WARN(param_name, use_camera_gain);
  }

  param_name = ns + "/annotate_map_with_camera";
  if (!ros::param::get(param_name, annotate_map_with_camera)) {
    annotate_map_with_camera = false;
    ROSPARAM_WARN(param_name, annotate_map_with_camera);
  }

  param_name = ns + "/inspection_planning";
  if (!ros::param::get(param_name, inspection_planning)) {
    inspection_planning = false;
    ROSPARAM_WARN(param_name, inspection_planning);
  }

  param_name = ns + "/keep_leaf_yaw_only";
  if (!ros::param::get(param_name, keep_leaf_yaw_only)) {
    keep_leaf_yaw_only = false;
    ROSPARAM_WARN(param_name, keep_leaf_yaw_only);
  }

  param_name = ns + "/enable_opening_traversal";
  if (!ros::param::get(param_name, enable_opening_traversal)) {
    enable_opening_traversal = false;
    ROSPARAM_WARN(param_name, enable_opening_traversal);
  }

  param_name = ns + "/opening_traversal_path_edge_length";
  if (!ros::param::get(param_name, opening_traversal_path_edge_length)) {
    opening_traversal_path_edge_length = 1.0;
    ROSPARAM_WARN(param_name, opening_traversal_path_edge_length);
  }

  param_name = ns + "/opening_alignment_z_offset";
  if (!ros::param::get(param_name, opening_alignment_z_offset)) {
    opening_alignment_z_offset = 0.0;
    ROSPARAM_WARN(param_name, opening_alignment_z_offset);
  }

  param_name = ns + "/only_opening_traversal";
  if (!ros::param::get(param_name, only_opening_traversal)) {
    only_opening_traversal = false;
    ROSPARAM_WARN(param_name, only_opening_traversal);
  }

  param_name = ns + "/auto_opening_path_approval";
  if (!ros::param::get(param_name, auto_opening_path_approval)) {
    auto_opening_path_approval = false;
    ROSPARAM_WARN(param_name, auto_opening_path_approval);
  }

  param_name = ns + "/min_coverage_percentage";
  if (!ros::param::get(param_name, min_coverage_percentage)) {
    min_coverage_percentage = 0.9;
    ROSPARAM_WARN(param_name, min_coverage_percentage);
  }

  param_name = ns + "/inspection_graph_vertices";
  if (!ros::param::get(param_name, inspection_graph_vertices)) {
    inspection_graph_vertices = num_vertices_max;
    ROSPARAM_WARN(param_name, inspection_graph_vertices);
  }

  param_name = ns + "/max_inspection_vertices";
  if (!ros::param::get(param_name, max_inspection_vertices)) {
    max_inspection_vertices = inspection_graph_vertices;
    ROSPARAM_WARN(param_name, max_inspection_vertices);
  }

  param_name = ns + "/inspection_xy_spacing";
  if (!ros::param::get(param_name, inspection_xy_spacing)) {
    inspection_xy_spacing = 1.0;
    ROSPARAM_WARN(param_name, inspection_xy_spacing);
  }

  param_name = ns + "/inspection_z_spacing";
  if (!ros::param::get(param_name, inspection_z_spacing)) {
    inspection_z_spacing = 2.0;
    ROSPARAM_WARN(param_name, inspection_z_spacing);
  }

  param_name = ns + "/inspection_thr_esdf_dist";
  if (!ros::param::get(param_name, inspection_thr_esdf_dist)) {
    inspection_thr_esdf_dist = 1.5;
    ROSPARAM_WARN(param_name, inspection_thr_esdf_dist);
  }

  param_name = ns + "/inspection_target_viewing_range";
  if (!ros::param::get(param_name, inspection_target_viewing_range)) {
    inspection_target_viewing_range = 2.5;
    ROSPARAM_WARN(param_name, inspection_target_viewing_range);
  }

  param_name = ns + "/max_exploration_iterations";
  if (!ros::param::get(param_name, max_exploration_iterations)) {
    max_exploration_iterations = 4;
    ROSPARAM_WARN(param_name, max_exploration_iterations);
  }

  param_name = ns + "/exploration_only";
  if (!ros::param::get(param_name, exploration_only)) {
    exploration_only = true;
    ROSPARAM_WARN(param_name, exploration_only);
  }

  param_name = ns + "/box_check_method";
  if (!ros::param::get(param_name, box_check_method)) {
    box_check_method = 0;
    ROSPARAM_WARN(param_name, box_check_method);
  }

  param_name = ns + "/line_check_method";
  if (!ros::param::get(param_name, line_check_method)) {
    line_check_method = 1;
    ROSPARAM_WARN(param_name, line_check_method);
  }

  param_name = ns + "/add_only_frontiers_to_global_graph";
  if (!ros::param::get(param_name, add_only_frontiers_to_global_graph)) {
    add_only_frontiers_to_global_graph = true;
    ROSPARAM_WARN(param_name, add_only_frontiers_to_global_graph);
  }

  param_name = ns + "/use_flipped_yaw";
  if (!ros::param::get(param_name, use_flipped_yaw)) {
    use_flipped_yaw = false;
    ROSPARAM_WARN(param_name, use_flipped_yaw);
  }

  param_name = ns + "/max_opening_height";
  if (!ros::param::get(param_name, max_opening_height)) {
    max_opening_height = 100.0;  // Too high so won't be used
    ROSPARAM_WARN(param_name, max_opening_height);
  }

  param_name = ns + "/max_surface_distance";
  if (!ros::param::get(param_name, max_surface_distance)) {
    max_surface_distance = 1.5;  // Too high so won't be used
    ROSPARAM_WARN(param_name, max_surface_distance);
  }

  param_name = ns + "/limit_vertices_to_surface";
  if (!ros::param::get(param_name, limit_vertices_to_surface)) {
    limit_vertices_to_surface = false;  // Too high so won't be used
    ROSPARAM_WARN(param_name, limit_vertices_to_surface);
  }

  param_name = ns + "/min_occ_surface";
  if (!ros::param::get(param_name, min_occ_surface)) {
    min_occ_surface = 0.0;
    ROSPARAM_WARN(param_name, min_occ_surface);
  }

  param_name = ns + "/max_opening_attempts";
  if (!ros::param::get(param_name, max_opening_attempts)) {
    max_opening_attempts = 3;
    ROSPARAM_WARN(param_name, max_opening_attempts);
  }

  param_name = ns + "/global_graph_odom_dist";
  if (!ros::param::get(param_name, global_graph_odom_dist)) {
    global_graph_odom_dist = 0.5;
    ROSPARAM_WARN(param_name, global_graph_odom_dist);
  }
  
  param_name = ns + "/global_graph_odom_connect_radius";
  if (!ros::param::get(param_name, global_graph_odom_connect_radius)) {
    global_graph_odom_connect_radius = edge_length_max;
    ROSPARAM_WARN(param_name, global_graph_odom_connect_radius);
  }

  param_name = ns + "/allow_sudden_dir_change";
  if (!ros::param::get(param_name, allow_sudden_dir_change)) {
    allow_sudden_dir_change = true;
    ROSPARAM_WARN(param_name, allow_sudden_dir_change);
  }

  param_name = ns + "/max_num_low_gain_iters";
  if (!ros::param::get(param_name, max_num_low_gain_iters)) {
    max_num_low_gain_iters = 3;
    ROSPARAM_WARN(param_name, max_num_low_gain_iters);
  }

  
  std::vector<double> param_val;

  param_val.clear();
  param_name = ns + "/compartment_centers";
  if ((!ros::param::get(param_name, param_val)) || (param_val.size() % 3 != 0)) {
    if(!exploration_only) {
      param_val.resize(3);
      param_val[0] = 0.0;
      param_val[1] = 0.0;
      param_val[2] = 0.0;
      ROSPARAM_ERROR(param_name);
    }
    else {
      ROSPARAM_WARN(param_name, 0.0);
    }
  }
  for(int i=0; i<param_val.size(); i+=3) {
    Eigen::Vector3d center;
    center << param_val[i], param_val[i+1], param_val[i+2];
    compartment_centers.push_back(center);
  }

  param_val.clear();
  param_name = ns + "/compartment_dimensions";
  compartment_dimensions.loadParams(param_name);

  

  ROSPARAM_INFO("Done.");
  return true;
}

void PlanningParams::setPlanningMode(PlanningModeType pmode) {
  switch (pmode) {
    case PlanningModeType::kBasicExploration:
      type = PlanningModeType::kBasicExploration;
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Exploration mode is set to kBasicExploration");
      break;
    default:
      type = PlanningModeType::kBasicExploration;
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Exploration mode is set to kBasicExploration");
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

bool DarpaGateParams::loadParams(std::string ns) {
  ROSPARAM_INFO("Loading: " + ns);
  std::string param_name;
  std::vector<double> param_val;

  param_name = ns + "/enable";
  if (!ros::param::get(param_name, enable)) {
    ROSPARAM_WARN(param_name, enable);
  }

  param_name = ns + "/load_from_darpa_frame";
  if (!ros::param::get(param_name, load_from_darpa_frame)) {
    ROSPARAM_WARN(param_name, load_from_darpa_frame);
  }

  param_name = ns + "/world_frame_id";
  if (!ros::param::get(param_name, world_frame_id)) {
    ROSPARAM_WARN(param_name, world_frame_id);
  }

  param_name = ns + "/gate_center_frame_id";
  if (!ros::param::get(param_name, gate_center_frame_id)) {
    ROSPARAM_WARN(param_name, gate_center_frame_id);
  }

  param_val.clear();
  param_name = ns + "/darpa_frame_offset";
  if ((!ros::param::get(param_name, param_val)) || (param_val.size() != 3)) {
    ROSPARAM_WARN(param_name, darpa_frame_offset[0]
                                  << "," << darpa_frame_offset[1] << ","
                                  << darpa_frame_offset[2]);
  } else {
    darpa_frame_offset << param_val[0], param_val[1], param_val[2];
  }

  param_val.clear();
  param_name = ns + "/center";
  if ((!ros::param::get(param_name, param_val)) || (param_val.size() != 3)) {
    ROSPARAM_WARN(param_name,
                  center[0] << "," << center[1] << "," << center[2]);
  } else {
    center << param_val[0], param_val[1], param_val[2];
  }

  param_name = ns + "/heading";
  if (!ros::param::get(param_name, heading)) {
    ROSPARAM_WARN(param_name, heading);
  }

  param_name = ns + "/center_search_radius";
  if (!ros::param::get(param_name, center_search_radius)) {
    ROSPARAM_WARN(param_name, center_search_radius);
  }

  param_name = ns + "/center_search_step";
  if (!ros::param::get(param_name, center_search_step)) {
    ROSPARAM_WARN(param_name, center_search_step);
  }

  param_name = ns + "/line_search_length";
  if (!ros::param::get(param_name, line_search_length)) {
    ROSPARAM_WARN(param_name, line_search_length);
  }

  param_name = ns + "/line_search_range";
  if (!ros::param::get(param_name, line_search_range)) {
    ROSPARAM_WARN(param_name, line_search_range);
  }

  param_name = ns + "/line_search_step";
  if (!ros::param::get(param_name, line_search_step)) {
    ROSPARAM_WARN(param_name, line_search_step);
  }

  ROSPARAM_INFO("Done.");
  return true;
}

// }  // namespace explorer
