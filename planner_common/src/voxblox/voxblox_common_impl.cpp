#include "planner_common/map_manager_voxblox_impl.h"

// TSDF
template <typename SDFServerType, typename SDFVoxelType>
voxblox::Layer<SDFVoxelType>*
MapManagerVoxblox<SDFServerType, SDFVoxelType>::getSDFLayer() {
  return sdf_server_.getTsdfMapPtr()->getTsdfLayerPtr();
}

// ESDF
template <>
voxblox::Layer<voxblox::EsdfVoxel>*
MapManagerVoxblox<voxblox::EsdfServer, voxblox::EsdfVoxel>::getSDFLayer() {
  return sdf_server_.getEsdfMapPtr()->getEsdfLayerPtr();
}

template <typename SDFServerType, typename SDFVoxelType>
MapManagerVoxblox<SDFServerType, SDFVoxelType>::MapManagerVoxblox(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : MapManager(nh, nh_private),
      sdf_server_(nh, nh_private),
      occupancy_distance_voxelsize_factor_(1.0F) {
  sdf_layer_ = getSDFLayer();
  CHECK_NOTNULL(sdf_layer_);
  interpolator_ = new voxblox::Interpolator<SDFVoxelType>(sdf_layer_);

  // Get local parameters from passed nodehandle
  if (!nh_private.getParam("occupancy_distance_voxelsize_factor",
                           occupancy_distance_voxelsize_factor_)) {
    ROS_INFO_COND(global_verbosity >= Verbosity::INFO,
                  "MapManagerVoxblox: failed to find parameter for "
                  "occupancy_distance_voxelsize_factor, using default of: %f",
                  occupancy_distance_voxelsize_factor_);
  }

  // Setup E/TsdfIntegratorBase::Config and object separately (also called in
  // e/tsdf_server_ ctor)
  tsdf_integrator_config_ =
      voxblox::getTsdfIntegratorConfigFromRosParam(nh_private);
  esdf_integrator_config_ =
      voxblox::getEsdfIntegratorConfigFromRosParam(nh_private);

#if (COL_CHECK_METHOD == 0)
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO,
                "[MapManager]: Point collision checking method: Box check");
#elif (COL_CHECK_METHOD == 1)
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO,
                "[MapManager]: Point collision checking method: Direct T/ESDF");
#elif (COL_CHECK_METHOD == 2)
  ROS_INFO_COND(
      global_verbosity >= Verbosity::INFO,
      "[MapManager]: Point collision checking method: Interpolated T/ESDF");
#endif

#if (EDGE_CHECK_METHOD == 0)
  ROS_INFO_COND(
      global_verbosity >= Verbosity::INFO,
      "[MapManager]: Line collision checking method: Multiple box checks");
#elif (EDGE_CHECK_METHOD == 1)
  ROS_INFO_COND(
      global_verbosity >= Verbosity::INFO,
      "[MapManager]: Line collision checking method: Cuboid around the");
#elif (EDGE_CHECK_METHOD == 2)
  ROS_INFO_COND(
      global_verbosity >= Verbosity::INFO,
      "[MapManager]: Line collision checking method: Direct T/ESDF check");
#elif (EDGE_CHECK_METHOD == 3)
  ROS_INFO_COND(
      global_verbosity >= Verbosity::INFO,
      "[MapManager]: Line collision checking method: Interpolated T/ESDF "
      "check");
#endif

#if (RAY_CAST_METHOD == 0)
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO,
                "[MapManager]: Ray casting method: Original");
#elif (RAY_CAST_METHOD == 1)
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO,
                "[MapManager]: Ray casting method: Iterative");
#endif
}

template <typename SDFServerType, typename SDFVoxelType>
double MapManagerVoxblox<SDFServerType, SDFVoxelType>::getResolution() const {
  return sdf_layer_->voxel_size();
}

template <typename SDFServerType, typename SDFVoxelType>
bool MapManagerVoxblox<SDFServerType, SDFVoxelType>::getStatus() const {
  return true;
}

// TSDF
template <typename SDFServerType, typename SDFVoxelType>
bool MapManagerVoxblox<SDFServerType, SDFVoxelType>::checkUnknownStatus(
    const SDFVoxelType* voxel) const {
  if (voxel == nullptr || voxel->weight < 1e-6) {
    return true;
  }
  return false;
}

// ESDF
template <>
bool MapManagerVoxblox<voxblox::EsdfServer, voxblox::EsdfVoxel>::
    checkUnknownStatus(const voxblox::EsdfVoxel* voxel) const {
  if (voxel == nullptr || !voxel->observed) {
    return true;
  }
  return false;
}

template <typename SDFServerType, typename SDFVoxelType>
typename MapManagerVoxblox<SDFServerType, SDFVoxelType>::VoxelStatus
MapManagerVoxblox<SDFServerType, SDFVoxelType>::getVoxelStatus(
    const Eigen::Vector3d& position) const {
  SDFVoxelType* voxel = sdf_layer_->getVoxelPtrByCoordinates(
      position.cast<voxblox::FloatingPoint>());

  if (checkUnknownStatus(voxel)) {
    return VoxelStatus::kUnknown;
  }
  const float distance_thres =
      occupancy_distance_voxelsize_factor_ * sdf_layer_->voxel_size() + 1e-6;
  if (voxel->distance <= distance_thres) {
    return VoxelStatus::kOccupied;
  }
  return VoxelStatus::kFree;
}

template <typename SDFServerType, typename SDFVoxelType>
typename MapManagerVoxblox<SDFServerType, SDFVoxelType>::VoxelStatus
MapManagerVoxblox<SDFServerType, SDFVoxelType>::getRayStatus(
    const Eigen::Vector3d& view_point, const Eigen::Vector3d& voxel_to_test,
    bool stop_at_unknown_voxel) const {
  // This involves doing a raycast from view point to voxel to test.
  // Let's get the global voxel coordinates of both.
  float voxel_size = sdf_layer_->voxel_size();
  float voxel_size_inv = 1.0 / voxel_size;

  const voxblox::Point start_scaled =
      view_point.cast<voxblox::FloatingPoint>() * voxel_size_inv;
  const voxblox::Point end_scaled =
      voxel_to_test.cast<voxblox::FloatingPoint>() * voxel_size_inv;

  voxblox::LongIndexVector global_voxel_indices;
  voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);

  const float distance_thres =
      occupancy_distance_voxelsize_factor_ * sdf_layer_->voxel_size() + 1e-6;
  // Iterate over the ray.
  for (const voxblox::GlobalIndex& global_index : global_voxel_indices) {
    SDFVoxelType* voxel = sdf_layer_->getVoxelPtrByGlobalIndex(global_index);
    if (checkUnknownStatus(voxel)) {
      if (stop_at_unknown_voxel) {
        return VoxelStatus::kUnknown;
      }
    } else if (voxel->distance <= distance_thres) {
      return VoxelStatus::kOccupied;
    }
  }
  return VoxelStatus::kFree;
}

template <typename SDFServerType, typename SDFVoxelType>
typename MapManagerVoxblox<SDFServerType, SDFVoxelType>::VoxelStatus
MapManagerVoxblox<SDFServerType, SDFVoxelType>::getRayStatus(
    const Eigen::Vector3d& view_point, const Eigen::Vector3d& voxel_to_test,
    bool stop_at_unknown_voxel, Eigen::Vector3d& end_voxel,
    double& tsdf_dist) const {
  // This involves doing a raycast from view point to voxel to test.
  // Let's get the global voxel coordinates of both.
  const float voxel_size = sdf_layer_->voxel_size();
  const float voxel_size_inv = 1.0 / voxel_size;

  const voxblox::Point start_scaled =
      view_point.cast<voxblox::FloatingPoint>() * voxel_size_inv;
  const voxblox::Point end_scaled =
      voxel_to_test.cast<voxblox::FloatingPoint>() * voxel_size_inv;

  voxblox::LongIndexVector global_voxel_indices;

  const float distance_thres =
      occupancy_distance_voxelsize_factor_ * sdf_layer_->voxel_size() + 1e-6;

  // Iterate over the ray.
  Eigen::Vector3d ray = (voxel_to_test - view_point);
  double ray_len = ray.norm();
  Eigen::Vector3d ray_normed = ray / ray_len;
  for (double d = 0; d < ray_len; d += voxel_size * 0.9) {
    Eigen::Vector3d voxel_coordi = view_point + d * ray_normed;
    voxblox::LongIndex center_voxel_index =
        voxblox::getGridIndexFromPoint<voxblox::LongIndex>(
            voxel_coordi.cast<voxblox::FloatingPoint>(), voxel_size_inv);
    SDFVoxelType* voxel =
        sdf_layer_->getVoxelPtrByGlobalIndex(center_voxel_index);
    voxblox::Point voxel_center = voxblox::getCenterPointFromGridIndex(
        center_voxel_index, voxel_size_inv);

    if (checkUnknownStatus(voxel)) {
      if (stop_at_unknown_voxel) {
        end_voxel = voxel_coordi;
        tsdf_dist = -tsdf_integrator_config_.default_truncation_distance;
        return VoxelStatus::kUnknown;
      }
    } else if (voxel->distance <= distance_thres) {
      end_voxel = voxel_coordi;
      tsdf_dist = voxel->distance;
      return VoxelStatus::kOccupied;
    }
  }

  voxblox::LongIndex center_voxel_index =
      voxblox::getGridIndexFromPoint<voxblox::LongIndex>(
          voxel_to_test.cast<voxblox::FloatingPoint>(), voxel_size_inv);
  SDFVoxelType* voxel =
      sdf_layer_->getVoxelPtrByGlobalIndex(center_voxel_index);
  voxblox::Point voxel_center =
      voxblox::getCenterPointFromGridIndex(center_voxel_index, voxel_size_inv);

  end_voxel = voxel_to_test;
  if (checkUnknownStatus(voxel)) {
    tsdf_dist = -tsdf_integrator_config_.default_truncation_distance;
  } else {
    tsdf_dist = voxel->distance;
  }
  return VoxelStatus::kFree;
}

template <typename SDFServerType, typename SDFVoxelType>
typename MapManagerVoxblox<SDFServerType, SDFVoxelType>::VoxelStatus
MapManagerVoxblox<SDFServerType, SDFVoxelType>::getBoxStatusInVoxels(
    const voxblox::LongIndex& box_center, const voxblox::AnyIndex& box_voxels,
    bool stop_at_unknown_voxel) const {
  VoxelStatus current_status = VoxelStatus::kFree;

  const float distance_thres =
      occupancy_distance_voxelsize_factor_ * sdf_layer_->voxel_size() + 1e-6;
  voxblox::LongIndex voxel_index = box_center;
  for (voxel_index.x() = box_center.x() - box_voxels.x() / 2;
       voxel_index.x() < box_center.x() + std::ceil(box_voxels.x() / 2.0);
       voxel_index.x()++) {
    for (voxel_index.y() = box_center.y() - box_voxels.y() / 2;
         voxel_index.y() < box_center.y() + std::ceil(box_voxels.y() / 2.0);
         voxel_index.y()++) {
      for (voxel_index.z() = box_center.z() - box_voxels.z() / 2;
           voxel_index.z() < box_center.z() + std::ceil(box_voxels.z() / 2.0);
           voxel_index.z()++) {
        SDFVoxelType* voxel = sdf_layer_->getVoxelPtrByGlobalIndex(voxel_index);
        if (checkUnknownStatus(voxel)) {
          if (stop_at_unknown_voxel) {
            return VoxelStatus::kUnknown;
          }
          current_status = VoxelStatus::kUnknown;
        } else if (voxel->distance <= distance_thres) {
          return VoxelStatus::kOccupied;
        }
      }
    }
  }
  return current_status;
}

// [NEED TO REVIEW]
template <typename SDFServerType, typename SDFVoxelType>
float MapManagerVoxblox<SDFServerType, SDFVoxelType>::getVoxelDistance(
    const Eigen::Vector3d& center) const {
  float voxel_size = sdf_layer_->voxel_size();
  float voxel_size_inv =
      1.0 /
      voxel_size;  // Get the center of the bounding box as a global index.
  voxblox::LongIndex center_voxel_index =
      voxblox::getGridIndexFromPoint<voxblox::LongIndex>(
          center.cast<voxblox::FloatingPoint>(), voxel_size_inv);
  SDFVoxelType* voxel =
      sdf_layer_->getVoxelPtrByGlobalIndex(center_voxel_index);
  if (checkUnknownStatus(voxel)) return -1.0;
  return voxel->distance;
}

// TSDF
template <typename SDFServerType, typename SDFVoxelType>
void MapManagerVoxblox<SDFServerType, SDFVoxelType>::clearIfUnknown(
    SDFVoxelType& voxel) {
  static constexpr float visualizeDistanceIntensityTsdfVoxels_kMinWeight =
      1e-3 + 1e-6;  // value for points to appear with
                    // visualizeDistanceIntensityTsdfVoxels
  if (voxel.weight < 1e-6) {
    voxel.weight = visualizeDistanceIntensityTsdfVoxels_kMinWeight;
    voxel.distance = tsdf_integrator_config_.default_truncation_distance;
  }
}

// ESDF
template <>
void MapManagerVoxblox<voxblox::EsdfServer, voxblox::EsdfVoxel>::clearIfUnknown(
    voxblox::EsdfVoxel& voxel) {
  if (!voxel.observed) {
    voxel.observed = true;
    voxel.hallucinated = true;
    voxel.distance = esdf_integrator_config_.default_distance_m;
  }
}

template <typename SDFServerType, typename SDFVoxelType>
bool MapManagerVoxblox<SDFServerType, SDFVoxelType>::augmentFreeBox(
    const Eigen::Vector3d& position, const Eigen::Vector3d& box_size) {
  voxblox::HierarchicalIndexMap block_voxel_list;
  voxblox::utils::getAndAllocateBoxAroundPoint(
      position.cast<voxblox::FloatingPoint>(), box_size, sdf_layer_,
      &block_voxel_list);
  for (const std::pair<voxblox::BlockIndex, voxblox::VoxelIndexList>& kv :
       block_voxel_list) {
    // Get block.
    typename voxblox::Block<SDFVoxelType>::Ptr block_ptr =
        sdf_layer_->getBlockPtrByIndex(kv.first);

    for (const voxblox::VoxelIndex& voxel_index : kv.second) {
      if (!block_ptr->isValidVoxelIndex(voxel_index)) {
        continue;
      }
      SDFVoxelType& voxel = block_ptr->getVoxelByVoxelIndex(voxel_index);
      // Clear voxels that haven't been cleared yet
      clearIfUnknown(voxel);
    }
  }
  return true;
}

template <typename SDFServerType, typename SDFVoxelType>
double MapManagerVoxblox<SDFServerType, SDFVoxelType>::getPointDistance(
    const Eigen::Vector3d& point) const {
  voxblox::FloatingPoint out_dist;
  voxblox::Point out_grad;
  bool success = interpolator_->getDistance(
      point.cast<voxblox::FloatingPoint>(), &out_dist, true);
  if (!success) {
    success = interpolator_->getDistance(point.cast<voxblox::FloatingPoint>(),
                                         &out_dist, false);
    if (!success)
      out_dist = -1.0;  // Unknown
    else {
      if (out_dist < 0.0) out_dist = 0.0;  // Occupied
    }
  } else {
    if (out_dist < 0.0) out_dist = 0.0;  // Occupied
  }

  return out_dist;  // Free
}

// [NEED TO REVIEW]
template <typename SDFServerType, typename SDFVoxelType>
void MapManagerVoxblox<SDFServerType, SDFVoxelType>::getFreeSpacePointCloud(
    std::vector<Eigen::Vector3d> multiray_endpoints, StateVec state,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  pcl::PointCloud<pcl::PointXYZ> free_cloud;

  Eigen::Vector3d state_vec(state[0], state[1], state[2]);

  for (auto ep : multiray_endpoints) {
    Eigen::Vector3d ray = (ep - state_vec);
    double ray_length = ray.norm();
    double voxel_size = 0.1;
    bool hit = false;

    for (int i = 0; i < (int)(ray_length / voxel_size); i++) {
      Eigen::Vector3d p = i * voxel_size * ray + state_vec;
      VoxelStatus voxel_state = getVoxelStatus(p);

      if (voxel_state == VoxelStatus::kOccupied) {
        hit = true;
        break;
      }
    }
    if (!hit) {
      pcl::PointXYZ data;
      Eigen::Matrix3d rot_W2B;
      rot_W2B = Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
      Eigen::Matrix3d rot_B2W;
      rot_B2W = rot_W2B.inverse();
      Eigen::Vector3d origin(state[0], state[1], state[2]);
      Eigen::Vector3d epb = rot_B2W * (ep - origin);
      data.x = epb(0);
      data.y = epb(1);
      data.z = epb(2);
      cloud->points.push_back(data);
    }
  }
}

template <typename SDFServerType, typename SDFVoxelType>
void MapManagerVoxblox<SDFServerType, SDFVoxelType>::augmentFreeFrustum() {
  ROS_WARN_THROTTLE(5.0, "MapManagerVoxblox::augmentFreeFrustum: N/A");
}

template <typename SDFServerType, typename SDFVoxelType>
void MapManagerVoxblox<SDFServerType, SDFVoxelType>::extractLocalMap(
    const Eigen::Vector3d& center, const Eigen::Vector3d& bounding_box_size,
    std::vector<Eigen::Vector3d>& occupied_voxels,
    std::vector<Eigen::Vector3d>& free_voxels) {
  // ROS_WARN_THROTTLE(5.0,
  //                   "MapManagerVoxblox::extractLocalMap --> Temporary
  //                   solution " "to be consistent with Octomap interface.");
  occupied_voxels.clear();
  free_voxels.clear();
  double resolution = getResolution();

  int Nx = std::ceil(bounding_box_size.x() / (2 * resolution)) *
           2;  // always even number
  int Ny = std::ceil(bounding_box_size.y() / (2 * resolution)) * 2;
  int Nz = std::ceil(bounding_box_size.z() / (2 * resolution)) * 2;

  // Correct center voxel depeding on the resolution of the map.
  const Eigen::Vector3d center_corrected(
      resolution * std::floor(center.x() / resolution) + resolution / 2.0,
      resolution * std::floor(center.y() / resolution) + resolution / 2.0,
      resolution * std::floor(center.z() / resolution) + resolution / 2.0);

  Eigen::Vector3d origin_offset;
  origin_offset = center_corrected - Eigen::Vector3d(Nx * resolution / 2,
                                                     Ny * resolution / 2,
                                                     Nz * resolution / 2);
  for (double x_ind = 0; x_ind <= Nx; ++x_ind) {
    for (double y_ind = 0; y_ind <= Ny; ++y_ind) {
      for (double z_ind = 0; z_ind <= Nz; ++z_ind) {
        Eigen::Vector3d pos(x_ind * resolution, y_ind * resolution,
                            z_ind * resolution);
        pos = pos + origin_offset;
        VoxelStatus vs = getVoxelStatus(pos);
        if (vs == VoxelStatus::kFree) {
          free_voxels.push_back(pos);
        } else if (vs == VoxelStatus::kOccupied) {
          occupied_voxels.push_back(pos);
        }
      }
    }
  }
}

template <typename SDFServerType, typename SDFVoxelType>
void MapManagerVoxblox<SDFServerType, SDFVoxelType>::extractLocalMapAlongAxis(
    const Eigen::Vector3d& center, const Eigen::Vector3d& axis,
    const Eigen::Vector3d& bounding_box_size,
    std::vector<Eigen::Vector3d>& occupied_voxels,
    std::vector<Eigen::Vector3d>& free_voxels) {
  occupied_voxels.clear();
  free_voxels.clear();
  double resolution = getResolution();

  int Nx = std::ceil(bounding_box_size.x() / (2 * resolution)) *
           2;  // always even number
  int Ny = std::ceil(bounding_box_size.y() / (2 * resolution)) * 2;
  int Nz = std::ceil(bounding_box_size.z() / (2 * resolution)) * 2;

  // Correct center voxel depeding on the resolution of the map.
  const Eigen::Vector3d center_corrected(
      resolution * std::floor(center.x() / resolution) + resolution / 2.0,
      resolution * std::floor(center.y() / resolution) + resolution / 2.0,
      resolution * std::floor(center.z() / resolution) + resolution / 2.0);

  Eigen::Vector3d origin_offset;
  Eigen::Vector3d half_box(Nx * resolution / 2, Ny * resolution / 2,
                           Nz * resolution / 2);
  origin_offset = center_corrected - half_box;

  Eigen::Vector3d x_axis(1.0, 0.0, 0.0);
  Eigen::Quaternion<double> quat_W2S;
  quat_W2S.setFromTwoVectors(x_axis, axis.normalized());

  for (double x_ind = 0; x_ind <= Nx; ++x_ind) {
    for (double y_ind = 0; y_ind <= Ny; ++y_ind) {
      for (double z_ind = 0; z_ind <= Nz; ++z_ind) {
        Eigen::Vector3d pos(x_ind * resolution, y_ind * resolution,
                            z_ind * resolution);
        pos = pos - half_box;
        pos = quat_W2S * pos + center_corrected;
        VoxelStatus vs = getVoxelStatus(pos);
        if (vs == VoxelStatus::kFree) {
          free_voxels.push_back(pos);
        } else if (vs == VoxelStatus::kOccupied) {
          occupied_voxels.push_back(pos);
        }
      }
    }
  }
}

template <typename SDFServerType, typename SDFVoxelType>
void MapManagerVoxblox<SDFServerType, SDFVoxelType>::getLocalPointcloud(
    const Eigen::Vector3d& center, const double& range, const double& yaw,
    pcl::PointCloud<pcl::PointXYZI>& pcl, bool include_unknown_voxels) {
  // use ray cast function
  // x = range * cos (theta) * sin(phi), y = range * sin(theta) * sin(phi), z =
  // range * cos(phi) theta - 0 to 2pi, phi - 0 to pi

  pcl::PointXYZI point;
  point.intensity = 250;

  const float voxel_size = sdf_layer_->voxel_size();
  const float voxel_size_inv = 1.0 / voxel_size;
  const double angular_res = 5.0 * M_PI / 180.0;
  // ((2 * range * range - voxel_size * voxel_size) / (2 * range * range)) *
  // M_PI / 180;

  // std::cout << "Angular range is: " << angular_res << std::endl;

  const voxblox::Point start_scaled =
      center.cast<voxblox::FloatingPoint>() * voxel_size_inv;
  const float distance_thres =
      occupancy_distance_voxelsize_factor_ * sdf_layer_->voxel_size() + 1e-6;

  for (double theta = 0; theta < 2 * M_PI; theta += angular_res) {
    for (double phi = 0; phi < M_PI; phi += angular_res) {
      Eigen::Vector3d end_point;
      end_point << center.x() + range * cos(theta) * sin(phi),
          center.y() + range * sin(theta) * sin(phi),
          center.z() + range * cos(phi);
      const voxblox::Point end_scaled =
          end_point.cast<voxblox::FloatingPoint>() * voxel_size_inv;

      voxblox::LongIndexVector global_voxel_indices;
      voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);
      // Iterate over the ray
      for (size_t k = 0; k < global_voxel_indices.size(); ++k) {
        const voxblox::GlobalIndex& global_index = global_voxel_indices[k];

        SDFVoxelType* voxel =
            sdf_layer_->getVoxelPtrByGlobalIndex(global_index);
        // Unknown
        if (checkUnknownStatus(voxel)) {
          if (!include_unknown_voxels) continue;
        }
        // Free
        else if (voxel->distance > distance_thres) {
          continue;
        }
        // Occupied
        Eigen::Vector3d voxel_coordi;
        voxel_coordi =
            voxblox::getCenterPointFromGridIndex(global_index, voxel_size)
                .cast<double>();
        point.x = voxel_coordi.x();
        point.y = voxel_coordi.y();
        point.z = voxel_coordi.z();
        pcl.points.push_back(point);
        break;
      }
    }
  }
}

template <typename SDFServerType, typename SDFVoxelType>
void MapManagerVoxblox<SDFServerType, SDFVoxelType>::getLocalPointcloud(
    const Eigen::Vector3d& center, const double& range, const double& yaw,
    const Eigen::Vector2d& z_limits, pcl::PointCloud<pcl::PointXYZI>& pcl,
    bool include_unknown_voxels) {
  // use ray cast function
  // x = range * cos (theta) * sin(phi), y = range * sin(theta) * sin(phi), z =
  // range * cos(phi) theta - 0 to 2pi, phi - 0 to pi

  pcl::PointXYZI point;
  point.intensity = 250;

  const float voxel_size = sdf_layer_->voxel_size();
  const float voxel_size_inv = 1.0 / voxel_size;
  const double angular_res = 2.0 * M_PI / 180.0;
  // ((2 * range * range - voxel_size * voxel_size) / (2 * range * range)) *
  // M_PI / 180;

  // std::cout << "Angular range is: " << angular_res << std::endl;

  const voxblox::Point start_scaled =
      center.cast<voxblox::FloatingPoint>() * voxel_size_inv;
  const float distance_thres =
      occupancy_distance_voxelsize_factor_ * sdf_layer_->voxel_size() + 1e-6;

  for (double theta = 0; theta < 2 * M_PI; theta += angular_res) {
    for (double phi = 0; phi < M_PI; phi += angular_res) {
      Eigen::Vector3d end_point;
      end_point << center.x() + range * cos(theta) * sin(phi),
          center.y() + range * sin(theta) * sin(phi),
          center.z() + range * cos(phi);
      const voxblox::Point end_scaled =
          end_point.cast<voxblox::FloatingPoint>() * voxel_size_inv;

      voxblox::LongIndexVector global_voxel_indices;
      voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);
      // Iterate over the ray
      size_t k = 0;
      for (k = 0; k < global_voxel_indices.size(); ++k) {
        const voxblox::GlobalIndex& global_index = global_voxel_indices[k];

        SDFVoxelType* voxel =
            sdf_layer_->getVoxelPtrByGlobalIndex(global_index);
        Eigen::Vector3d voxel_coordi;
        voxel_coordi =
            voxblox::getCenterPointFromGridIndex(global_index, voxel_size)
                .cast<double>();
        if (voxel_coordi(2) > center(2) + z_limits(1) ||
            voxel_coordi(2) < center(2) + z_limits(0)) {
          break;
        }
        // Unknown
        if (checkUnknownStatus(voxel)) {
          if (!include_unknown_voxels) continue;
        }
        // Free
        else if (voxel->distance > distance_thres) {
          continue;
        }
        // Occupied
        point.x = voxel_coordi.x();
        point.y = voxel_coordi.y();
        point.z = voxel_coordi.z();
        pcl.points.push_back(point);
        break;
      }
    }
  }
}

template <typename SDFServerType, typename SDFVoxelType>
pcl::PointXYZI
MapManagerVoxblox<SDFServerType, SDFVoxelType>::eigenVec3dToPCLPoint(
    Eigen::Vector3d& vec) const {
  pcl::PointXYZI point;
  point.x = vec.x();
  point.y = vec.y();
  point.z = vec.z();
  return point;
}
