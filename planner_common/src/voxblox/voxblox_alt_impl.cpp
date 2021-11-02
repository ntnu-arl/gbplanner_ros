#include "planner_common/map_manager_voxblox_impl.h"

#if (COL_CHECK_METHOD == 0)
template <typename SDFServerType, typename SDFVoxelType>
typename MapManagerVoxblox<SDFServerType, SDFVoxelType>::VoxelStatus
MapManagerVoxblox<SDFServerType, SDFVoxelType>::getBoxStatus(
    const Eigen::Vector3d& center, const Eigen::Vector3d& size,
    bool stop_at_unknown_voxel) const {
  float voxel_size = sdf_layer_->voxel_size();
  float voxel_size_inv = 1.0 / voxel_size;

  // Get the center of the bounding box as a global index.
  voxblox::LongIndex center_voxel_index =
      voxblox::getGridIndexFromPoint<voxblox::LongIndex>(
          center.cast<voxblox::FloatingPoint>(), voxel_size_inv);

  voxblox::AnyIndex box_voxels(std::floor(size.x() * voxel_size_inv + 0.5),
                               std::floor(size.y() * voxel_size_inv + 0.5),
                               std::floor(size.z() * voxel_size_inv + 0.5));

  // Iterate over all voxels in the bounding box.
  return getBoxStatusInVoxels(center_voxel_index, box_voxels,
                              stop_at_unknown_voxel);
}
#elif (COL_CHECK_METHOD == 1)
template <typename SDFServerType, typename SDFVoxelType>
typename MapManagerVoxblox<SDFServerType, SDFVoxelType>::VoxelStatus
MapManagerVoxblox<SDFServerType, SDFVoxelType>::getBoxStatus(
    const Eigen::Vector3d& center, const Eigen::Vector3d& size,
    bool stop_at_unknown_voxel) const {
  float voxel_size = sdf_layer_->voxel_size();
  float voxel_size_inv = 1.0 / voxel_size;

  // Get the center of the bounding box as a global index.
  double dist = getPointDistance(center);
  if (dist < robot_radius_) {
    if (dist < 0.0) {
      if (stop_at_unknown_voxel) {
        return VoxelStatus::kUnknown;
      }
    } else {
      return VoxelStatus::kOccupied;
    }
  }
  return VoxelStatus::kFree;
}
#elif (COL_CHECK_METHOD == 2)
template <typename SDFServerType, typename SDFVoxelType>
typename MapManagerVoxblox<SDFServerType, SDFVoxelType>::VoxelStatus
MapManagerVoxblox<SDFServerType, SDFVoxelType>::getBoxStatus(
    const Eigen::Vector3d& center, const Eigen::Vector3d& size,
    bool stop_at_unknown_voxel) const {
  float voxel_size = sdf_layer_->voxel_size();
  float voxel_size_inv = 1.0 / voxel_size;
  float check_size = voxel_size * 2.0;
  const float distance_thres = check_size / 2.0 + 1e-6;

  Eigen::Vector3d reduced_size =
      size - Eigen::Vector3d(check_size, check_size, check_size);
  for (int i = 0; i < 3; ++i) {
    if (reduced_size(i) < 0.0) reduced_size(i) = 0.0;
  }

  for (double dx = 0.0; dx < (reduced_size(0) + check_size / 2.0 + 1e-2);
       dx += check_size) {
    for (double dy = 0.0; dy < (reduced_size(1) + check_size / 2.0 + 1e-2);
         dy += check_size) {
      for (double dz = 0.0; dz < (reduced_size(2) + check_size / 2.0 + 1e-2);
           dz += check_size) {
        Eigen::Vector3d point =
            center + Eigen::Vector3d(dx, dy, dz) - reduced_size / 2.0;
        double dist = getPointDistance(point);
        if (dist < distance_thres) {
          if (dist < 0.0) {
            if (stop_at_unknown_voxel) {
              return VoxelStatus::kUnknown;
            }
          } else {
            return VoxelStatus::kOccupied;
          }
        }
      }
    }
  }

  return VoxelStatus::kFree;
}
#endif

/* Line checking: */

#if (EDGE_CHECK_METHOD == 0)
template <typename SDFServerType, typename SDFVoxelType>
typename MapManagerVoxblox<SDFServerType, SDFVoxelType>::VoxelStatus
MapManagerVoxblox<SDFServerType, SDFVoxelType>::getPathStatus(
    const Eigen::Vector3d& start, const Eigen::Vector3d& end,
    const Eigen::Vector3d& box_size, bool stop_at_unknown_voxel) const {
  // Cast ray along the center to make sure we don't miss anything.
  float voxel_size = sdf_layer_->voxel_size();
  float voxel_size_inv = 1.0 / voxel_size;

  Eigen::Vector3d edge_vec;
  edge_vec = end - start;
  Eigen::Vector3d start_e;
  start_e = start;
  int check_count = std::ceil(edge_vec.norm() * voxel_size_inv);
  for (int i = 0; i < check_count; i++) {
    Eigen::Vector3d vox_center = start_e + i * edge_vec / edge_vec.norm();
    VoxelStatus vs = getBoxStatus(vox_center, box_size, stop_at_unknown_voxel);
    if (vs != VoxelStatus::kFree) {
      return vs;
    }
  }
  return VoxelStatus::kFree;
}
#elif (EDGE_CHECK_METHOD == 1)
template <typename SDFServerType, typename SDFVoxelType>
typename MapManagerVoxblox<SDFServerType, SDFVoxelType>::VoxelStatus
MapManagerVoxblox<SDFServerType, SDFVoxelType>::getPathStatus(
    const Eigen::Vector3d& start, const Eigen::Vector3d& end,
    const Eigen::Vector3d& box_size, bool stop_at_unknown_voxel) const {
  float voxel_size = sdf_layer_->voxel_size();
  float voxel_size_inv = 1.0 / voxel_size;
  const float distance_thres =
      occupancy_distance_voxelsize_factor_ * sdf_layer_->voxel_size() + 1e-6;

  Eigen::Vector3d edge_vec;
  edge_vec = end - start;
  edge_vec(2) = 0.0;
  double edge_len = edge_vec.norm();
  edge_vec /= edge_len;

  Eigen::Vector3d rot_axis;
  rot_axis = edge_vec.cross(Eigen::Vector3d::UnitX());
  rot_axis = rot_axis / rot_axis.norm();

  double cos_angle = edge_vec.dot(Eigen::Vector3d::UnitX());
  if (cos_angle >= 1.0)
    cos_angle = 1.0;
  else if (cos_angle <= -1.0)
    cos_angle = -1.0;

  double angle = acos(cos_angle);

  Eigen::Matrix3d unitx2Edge;
  unitx2Edge = Eigen::AngleAxisd(angle, -rot_axis);

  // Top bottom
  double z_up = box_size[2] / 2.0;
  double z_down = -box_size[2] / 2.0;
  int horiz_voz_count = std::ceil(box_size[1] * voxel_size_inv);

  for (double i = -box_size[1] / 2; i < box_size[1] / 2; i += voxel_size) {
    Eigen::Vector3d top_vox_center_start(0.0, i, z_up);
    Eigen::Vector3d bottom_vox_center_start(0.0, i, z_down);

    Eigen::Vector3d top_vox_center_end(0.0, 0.0 + i, 0.0 + z_up);
    Eigen::Vector3d bottom_vox_center_end(0.0, 0.0 + i, 0.0 + z_down);

    Eigen::Vector3d top_vox_center_start_tf, top_vox_center_end_tf;
    Eigen::Vector3d bottom_vox_center_start_tf, bottom_vox_center_end_tf;

    top_vox_center_start_tf = start + unitx2Edge * top_vox_center_start;
    top_vox_center_end_tf = end + unitx2Edge * top_vox_center_end;
    bottom_vox_center_start_tf = start + unitx2Edge * bottom_vox_center_start;
    bottom_vox_center_end_tf = end + unitx2Edge * bottom_vox_center_end;

    // top ray
    const voxblox::Point top_start_scaled =
        top_vox_center_start_tf.cast<voxblox::FloatingPoint>() * voxel_size_inv;
    const voxblox::Point top_end_scaled =
        top_vox_center_end_tf.cast<voxblox::FloatingPoint>() * voxel_size_inv;

    voxblox::LongIndexVector global_voxel_indices;
    voxblox::castRay(top_start_scaled, top_end_scaled, &global_voxel_indices);
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

    // bottom ray
    const voxblox::Point bottom_start_scaled =
        bottom_vox_center_start_tf.cast<voxblox::FloatingPoint>() *
        voxel_size_inv;
    const voxblox::Point bottom_end_scaled =
        bottom_vox_center_end_tf.cast<voxblox::FloatingPoint>() *
        voxel_size_inv;

    voxblox::LongIndexVector bottom_global_voxel_indices;
    voxblox::castRay(bottom_start_scaled, bottom_end_scaled,
                     &bottom_global_voxel_indices);
    for (const voxblox::GlobalIndex& global_index :
         bottom_global_voxel_indices) {
      SDFVoxelType* voxel = sdf_layer_->getVoxelPtrByGlobalIndex(global_index);
      if (checkUnknownStatus(voxel)) {
        if (stop_at_unknown_voxel) {
          return VoxelStatus::kUnknown;
        }
      } else if (voxel->distance <= distance_thres) {
        return VoxelStatus::kOccupied;
      }
    }
  }

  // Left Right
  double w_left = box_size[1] / 2.0;
  double w_right = -box_size[1] / 2.0;

  for (double i = -box_size[2] / 2; i < box_size[2] / 2; i += voxel_size) {
    Eigen::Vector3d left_vox_center_start(0.0, 0.0 + w_left, 0.0 + i);
    Eigen::Vector3d right_vox_center_start(0.0, 0.0 + w_right, 0.0 + i);

    Eigen::Vector3d left_vox_center_end(0.0, 0.0 + w_left, 0.0 + i);
    Eigen::Vector3d right_vox_center_end(0.0, 0.0 + w_right, 0.0 + i);

    Eigen::Vector3d left_vox_center_start_tf, left_vox_center_end_tf;
    Eigen::Vector3d right_vox_center_start_tf, right_vox_center_end_tf;

    left_vox_center_start_tf = start + unitx2Edge * left_vox_center_start;
    left_vox_center_end_tf = end + unitx2Edge * left_vox_center_end;
    right_vox_center_start_tf = start + unitx2Edge * right_vox_center_start;
    right_vox_center_end_tf = end + unitx2Edge * right_vox_center_end;

    // left ray
    const voxblox::Point left_start_scaled =
        left_vox_center_start_tf.cast<voxblox::FloatingPoint>() *
        voxel_size_inv;
    const voxblox::Point left_end_scaled =
        left_vox_center_end_tf.cast<voxblox::FloatingPoint>() * voxel_size_inv;

    voxblox::LongIndexVector left_global_voxel_indices;
    voxblox::castRay(left_start_scaled, left_end_scaled,
                     &left_global_voxel_indices);
    for (const voxblox::GlobalIndex& global_index : left_global_voxel_indices) {
      SDFVoxelType* voxel = sdf_layer_->getVoxelPtrByGlobalIndex(global_index);
      if (checkUnknownStatus(voxel)) {
        if (stop_at_unknown_voxel) {
          return VoxelStatus::kUnknown;
        }
      } else if (voxel->distance <= distance_thres) {
        return VoxelStatus::kOccupied;
      }
    }

    // bottom ray
    const voxblox::Point right_start_scaled =
        right_vox_center_start_tf.cast<voxblox::FloatingPoint>() *
        voxel_size_inv;
    const voxblox::Point right_end_scaled =
        right_vox_center_end_tf.cast<voxblox::FloatingPoint>() * voxel_size_inv;

    voxblox::LongIndexVector right_global_voxel_indices;
    voxblox::castRay(right_start_scaled, right_end_scaled,
                     &right_global_voxel_indices);
    for (const voxblox::GlobalIndex& global_index :
         right_global_voxel_indices) {
      SDFVoxelType* voxel = sdf_layer_->getVoxelPtrByGlobalIndex(global_index);
      if (checkUnknownStatus(voxel)) {
        if (stop_at_unknown_voxel) {
          return VoxelStatus::kUnknown;
        }
      } else if (voxel->distance <= distance_thres) {
        return VoxelStatus::kOccupied;
      }
    }
  }
  return VoxelStatus::kFree;
}
#elif (EDGE_CHECK_METHOD == 2)
template <typename SDFServerType, typename SDFVoxelType>
typename MapManagerVoxblox<SDFServerType, SDFVoxelType>::VoxelStatus
MapManagerVoxblox<SDFServerType, SDFVoxelType>::getPathStatus(
    const Eigen::Vector3d& start, const Eigen::Vector3d& end,
    const Eigen::Vector3d& box_size, bool stop_at_unknown_voxel) const {
  float voxel_size = sdf_layer_->voxel_size();
  float voxel_size_inv = 1.0 / voxel_size;

  Eigen::Vector3d edge_vec;
  edge_vec = end - start;
  Eigen::Vector3d start_e;
  start_e = start;
  int check_count = std::ceil(edge_vec.norm() * voxel_size_inv);
  for (int i = 0; i < check_count; i++) {
    Eigen::Vector3d vox_center = start_e + i * edge_vec / edge_vec.norm();
    double dist = getPointDistance(vox_center);
    if (dist < robot_radius_) {
      if (dist < 0.0) {
        if (stop_at_unknown_voxel) {
          return VoxelStatus::kUnknown;
        }
      } else {
        return VoxelStatus::kOccupied;
      }
    }
  }
  return VoxelStatus::kFree;
}
#elif (EDGE_CHECK_METHOD == 3)
template <typename SDFServerType, typename SDFVoxelType>
typename MapManagerVoxblox<SDFServerType, SDFVoxelType>::VoxelStatus
MapManagerVoxblox<SDFServerType, SDFVoxelType>::getPathStatus(
    const Eigen::Vector3d& start, const Eigen::Vector3d& end,
    const Eigen::Vector3d& box_size, bool stop_at_unknown_voxel) const {
  float voxel_size = sdf_layer_->voxel_size();
  float voxel_size_inv = 1.0 / voxel_size;
  float check_size = voxel_size * 1.0;
  const float distance_thres = voxel_size / 2.0 + 1e-6;

  Eigen::Vector3d edge_vec;
  edge_vec = end - start;
  edge_vec(2) = 0.0;
  double edge_len = edge_vec.norm();
  edge_vec /= edge_len;

  Eigen::Vector3d rot_axis;
  rot_axis = edge_vec.cross(Eigen::Vector3d::UnitX());
  rot_axis = rot_axis / rot_axis.norm();

  double cos_angle = edge_vec.dot(Eigen::Vector3d::UnitX());
  if (cos_angle >= 1.0)
    cos_angle = 1.0;
  else if (cos_angle <= -1.0)
    cos_angle = -1.0;

  double angle = acos(cos_angle);

  Eigen::Matrix3d unitx2Edge;
  unitx2Edge = Eigen::AngleAxisd(angle, -rot_axis);

  // Top bottom
  double z_up = box_size[2] / 2.0;
  double z_down = -box_size[2] / 2.0;
  int horiz_voz_count = std::ceil(box_size[1] * voxel_size_inv);

  for (double i = -box_size[1] / 2; i < box_size[1] / 2; i += voxel_size) {
    Eigen::Vector3d top_vox_center_start(0.0, i, z_up);
    Eigen::Vector3d bottom_vox_center_start(0.0, i, z_down);

    Eigen::Vector3d top_vox_center_end(0.0, 0.0 + i, 0.0 + z_up);
    Eigen::Vector3d bottom_vox_center_end(0.0, 0.0 + i, 0.0 + z_down);

    Eigen::Vector3d top_vox_center_start_tf, top_vox_center_end_tf;
    Eigen::Vector3d bottom_vox_center_start_tf, bottom_vox_center_end_tf;

    top_vox_center_start_tf = start + unitx2Edge * top_vox_center_start;
    top_vox_center_end_tf = end + unitx2Edge * top_vox_center_end;
    bottom_vox_center_start_tf = start + unitx2Edge * bottom_vox_center_start;
    bottom_vox_center_end_tf = end + unitx2Edge * bottom_vox_center_end;

    Eigen::Vector3d ray = top_vox_center_end_tf - top_vox_center_start_tf;
    Eigen::Vector3d ray_normed = ray / ray.norm();
    for (double d = 0.0; d < ray.norm(); d += check_size) {
      Eigen::Vector3d point = top_vox_center_start_tf + d * ray_normed;
      double dist = getPointDistance(point);
      if (dist < distance_thres) {
        if (dist < 0.0) {
          if (stop_at_unknown_voxel) {
            return VoxelStatus::kUnknown;
          }
        } else {
          return VoxelStatus::kOccupied;
        }
      }
    }

    ray = bottom_vox_center_end_tf - bottom_vox_center_start_tf;
    ray_normed = ray / ray.norm();
    for (double d = 0.0; d < ray.norm(); d += check_size) {
      Eigen::Vector3d point = bottom_vox_center_start_tf + d * ray_normed;
      double dist = getPointDistance(point);
      if (dist < distance_thres) {
        if (dist < 0.0) {
          if (stop_at_unknown_voxel) {
            return VoxelStatus::kUnknown;
          }
        } else {
          return VoxelStatus::kOccupied;
        }
      }
    }
  }

  // Left Right
  double w_left = box_size[1] / 2.0;
  double w_right = -box_size[1] / 2.0;
  // int horiz_voz_count = std::ceil(box_size[1]*voxel_size_inv);
  for (double i = -box_size[2] / 2; i < box_size[2] / 2; i += voxel_size) {
    Eigen::Vector3d left_vox_center_start(0.0, 0.0 + w_left, 0.0 + i);
    Eigen::Vector3d right_vox_center_start(0.0, 0.0 + w_right, 0.0 + i);

    Eigen::Vector3d left_vox_center_end(0.0, 0.0 + w_left, 0.0 + i);
    Eigen::Vector3d right_vox_center_end(0.0, 0.0 + w_right, 0.0 + i);

    Eigen::Vector3d left_vox_center_start_tf, left_vox_center_end_tf;
    Eigen::Vector3d right_vox_center_start_tf, right_vox_center_end_tf;

    left_vox_center_start_tf = start + unitx2Edge * left_vox_center_start;
    left_vox_center_end_tf = end + unitx2Edge * left_vox_center_end;
    right_vox_center_start_tf = start + unitx2Edge * right_vox_center_start;
    right_vox_center_end_tf = end + unitx2Edge * right_vox_center_end;

    Eigen::Vector3d ray = left_vox_center_end_tf - left_vox_center_start_tf;
    Eigen::Vector3d ray_normed = ray / ray.norm();
    for (double d = 0.0; d < ray.norm(); d += check_size) {
      Eigen::Vector3d point = left_vox_center_start_tf + d * ray_normed;
      double dist = getPointDistance(point);
      if (dist < distance_thres) {
        if (dist < 0.0) {
          if (stop_at_unknown_voxel) {
            return VoxelStatus::kUnknown;
          }
        } else {
          return VoxelStatus::kOccupied;
        }
      }
    }

    ray = right_vox_center_end_tf - right_vox_center_start_tf;
    ray_normed = ray / ray.norm();
    for (double d = 0.0; d < ray.norm(); d += check_size) {
      Eigen::Vector3d point = right_vox_center_start_tf + d * ray_normed;
      double dist = getPointDistance(point);
      if (dist < distance_thres) {
        if (dist < 0.0) {
          if (stop_at_unknown_voxel) {
            return VoxelStatus::kUnknown;
          }
        } else {
          return VoxelStatus::kOccupied;
        }
      }
    }
  }
  return VoxelStatus::kFree;
}
#endif

/* Ray casting (getScanStatus) */
#if (RAY_CAST_METHOD == 0)
template <typename SDFServerType, typename SDFVoxelType>
void MapManagerVoxblox<SDFServerType, SDFVoxelType>::getScanStatus(
    Eigen::Vector3d& pos, std::vector<Eigen::Vector3d>& multiray_endpoints,
    std::tuple<int, int, int>& gain_log,
    std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& voxel_log,
    SensorParamsBase& sensor_params) {
  unsigned int num_unknown_voxels = 0, num_free_voxels = 0,
               num_occupied_voxels = 0;

  const float voxel_size = sdf_layer_->voxel_size();
  const float voxel_size_inv = 1.0 / voxel_size;
  const float step_size_change_dist =
      4.0;  // After every these many meters increase the ray interpolation
            // distance
  const float step_size_change_dist_end =
      12.0;  // After this much distance stick to the last interpolation
             // distance

  const voxblox::Point start_scaled =
      pos.cast<voxblox::FloatingPoint>() * voxel_size_inv;

  const float distance_thres =
      occupancy_distance_voxelsize_factor_ * sdf_layer_->voxel_size() + 1e-6;

  // NOTES: no optimization / no twice-counting considerations possible without
  // refactoring planning strategy here

  // Iterate for every endpoint, insert unknown voxels found over every ray into
  // a set to avoid double-counting Important: do not use <VoxelIndex> type
  // directly, will break voxblox's implementations
  // Move away from std::unordered_set and work with std::vector + std::unique
  // count at the end (works best for now)
  /*std::vector<std::size_t> raycast_unknown_vec_, raycast_occupied_vec_,
  raycast_free_vec_; raycast_unknown_vec_.reserve(multiray_endpoints.size() *
  tsdf_integrator_config_.max_ray_length_m * voxel_size_inv); //optimize for
  number of rays raycast_free_vec_.reserve(multiray_endpoints.size() *
  tsdf_integrator_config_.max_ray_length_m * voxel_size_inv); //optimize for
  number of rays raycast_occupied_vec_.reserve(multiray_endpoints.size());
  //optimize for number of rays*/
  // voxel_log.reserve(multiray_endpoints.size() *
  // tsdf_integrator_config_.max_ray_length_m * voxel_size_inv); //optimize for
  // number of rays
  for (size_t i = 0; i < multiray_endpoints.size(); ++i) {
    float step_size = voxel_size * ray_cast_step_size_multiplier_;
    float step_size_inv = 1.0 / step_size;
    float og_step_size = step_size;

    Eigen::Vector3d ray_normalized =
        (multiray_endpoints[i] - pos);  // Not yet noramlized
    double ray_norm = ray_normalized.norm();
    ray_normalized = ray_normalized / ray_norm;  // Normalized here

    // Iterate over the ray.
    double prev_step_dist = 0.0;
    for (double step = 0.0; step <= ray_norm; step += step_size) {
      if (nonuniform_ray_cast_) {
        if (std::abs(prev_step_dist - step) > step_size_change_dist &&
            step <= step_size_change_dist_end) {
          prev_step_dist = step;
          step_size += og_step_size;
        }
      }
      Eigen::Vector3d voxel_coordi = (pos + ray_normalized * step);

      voxblox::LongIndex global_index =
          voxblox::getGridIndexFromPoint<voxblox::LongIndex>(
              voxel_coordi.cast<voxblox::FloatingPoint>(), voxel_size_inv);

      SDFVoxelType* voxel = sdf_layer_->getVoxelPtrByGlobalIndex(global_index);
      // Unknown
      if (checkUnknownStatus(voxel)) {
        /*raycast_unknown_vec_.push_back(std::hash<voxblox::GlobalIndex>()(global_index));*/
        ++num_unknown_voxels;
        voxel_log.push_back(std::make_pair(
            voxblox::getCenterPointFromGridIndex(global_index, voxel_size)
                .cast<double>(),
            VoxelStatus::kUnknown));
        continue;
      }
      // Free
      if (voxel->distance > distance_thres) {
        /*raycast_free_vec_.push_back(std::hash<voxblox::GlobalIndex>()(global_index));*/
        ++num_free_voxels;
        voxel_log.push_back(std::make_pair(
            voxblox::getCenterPointFromGridIndex(global_index, voxel_size)
                .cast<double>(),
            VoxelStatus::kFree));
        continue;
      }
      // Occupied
      /*raycast_occupied_vec_.push_back(std::hash<voxblox::GlobalIndex>()(global_index));*/
      ++num_occupied_voxels;
      voxel_log.push_back(std::make_pair(
          voxblox::getCenterPointFromGridIndex(global_index, voxel_size)
              .cast<double>(),
          VoxelStatus::kOccupied));
      break;
    }
  }
  /*std::sort(raycast_unknown_vec_.begin(), raycast_unknown_vec_.end());
  std::sort(raycast_occupied_vec_.begin(), raycast_occupied_vec_.end());
  std::sort(raycast_free_vec_.begin(), raycast_free_vec_.end());
  num_unknown_voxels = std::unique(raycast_unknown_vec_.begin(),
  raycast_unknown_vec_.end()) - raycast_unknown_vec_.begin();
  num_occupied_voxels = std::unique(raycast_occupied_vec_.begin(),
  raycast_occupied_vec_.end()) - raycast_occupied_vec_.begin();
  num_free_voxels = std::unique(raycast_free_vec_.begin(),
  raycast_free_vec_.end()) - raycast_free_vec_.begin();*/
  gain_log =
      std::make_tuple(num_unknown_voxels, num_free_voxels, num_occupied_voxels);
}

#elif (RAY_CAST_METHOD == 1)
template <typename SDFServerType, typename SDFVoxelType>
void MapManagerVoxblox<SDFServerType, SDFVoxelType>::getScanStatus(
    Eigen::Vector3d& pos, std::vector<Eigen::Vector3d>& multiray_endpoints,
    std::tuple<int, int, int>& gain_log,
    std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& voxel_log,
    SensorParamsBase& sensor_params) {
  unsigned int num_unknown_voxels = 0, num_free_voxels = 0,
               num_occupied_voxels = 0;
  int height = sensor_params.height;
  int width = sensor_params.width;
  Eigen::MatrixXd starting_points(
      height, width);  // This will store the distance along the ray from where
                       // the raycasting should begin
  starting_points.setZero();

  const float voxel_size = sdf_layer_->voxel_size();
  const float voxel_size_inv = 1.0 / voxel_size;
  const float step_size_change_dist =
      4.0;  // After every these many meters increase the ray interpolation
            // distance
  const float step_size_change_dist_end =
      12.0;  // After this much distance stick to the last interpolation
             // distance

  const voxblox::Point start_scaled =
      pos.cast<voxblox::FloatingPoint>() * voxel_size_inv;

  const float distance_thres =
      occupancy_distance_voxelsize_factor_ * sdf_layer_->voxel_size() + 1e-6;

  // NOTES: no optimization / no twice-counting considerations possible without
  // refactoring planning strategy here

  // Iterate for every endpoint, insert unknown voxels found over every ray into
  // a set to avoid double-counting Important: do not use <VoxelIndex> type
  // directly, will break voxblox's implementations
  // Move away from std::unordered_set and work with std::vector + std::unique
  // count at the end (works best for now)
  /*std::vector<std::size_t> raycast_unknown_vec_, raycast_occupied_vec_,
  raycast_free_vec_; raycast_unknown_vec_.reserve(multiray_endpoints.size() *
  tsdf_integrator_config_.max_ray_length_m * voxel_size_inv); //optimize for
  number of rays raycast_free_vec_.reserve(multiray_endpoints.size() *
  tsdf_integrator_config_.max_ray_length_m * voxel_size_inv); //optimize for
  number of rays raycast_occupied_vec_.reserve(multiray_endpoints.size());
  //optimize for number of rays*/

  for (int y = height - 1; y >= 0; --y) {
    for (int x = 0; x < width; ++x) {
      float step_size = voxel_size * ray_cast_step_size_multiplier_;
      float step_size_inv = 1.0 / step_size;
      float og_step_size = step_size;

      int ep_ind = (height - y - 1) * width +
                   x;  // Index of the endpoint corresponding to this cell
      const double ray_start_point = starting_points(x, y);

      Eigen::Vector3d ray_normalized =
          (multiray_endpoints[ep_ind] - pos);  // Not yet noramlized
      double ray_norm = ray_normalized.norm();
      ray_normalized = ray_normalized / ray_norm;  // Normalized here
      Eigen::Vector3d start_point =
          pos + ray_normalized * std::abs(ray_start_point);
      Eigen::Vector3d truncated_ray = multiray_endpoints[ep_ind] - start_point;
      double truncated_ray_len = truncated_ray.norm();
      int num_steps =
          ray_norm * step_size_inv;  // This is the number of steps to traverse
                                     // the entire ray NOT the truncated ray

      if (ray_start_point < 0.0) {
        // Ray hits an occupied voxel
        ++num_occupied_voxels;
        voxel_log.push_back(
            std::make_pair(start_point, VoxelStatus::kOccupied));
        continue;
      }

      int count = 0;
      double prev_step_dist = ray_start_point;
      for (double step = ray_start_point; step <= ray_norm; step += step_size) {
        count++;

        // Update step size
        if (nonuniform_ray_cast_) {
          if (std::abs(prev_step_dist - step) > step_size_change_dist &&
              step <= step_size_change_dist_end) {
            prev_step_dist = step;
            step_size += og_step_size;
          }
        }

        Eigen::Vector3d voxel_coordi = (pos + ray_normalized * step);
        Eigen::Vector3d voxel_coordi_scaled = voxel_coordi * voxel_size_inv;

        voxblox::LongIndex center_voxel_index =
            voxblox::getGridIndexFromPoint<voxblox::LongIndex>(
                voxel_coordi.cast<voxblox::FloatingPoint>(), voxel_size_inv);

        SDFVoxelType* voxel =
            sdf_layer_->getVoxelPtrByGlobalIndex(center_voxel_index);
        // Unknown
        if (checkUnknownStatus(voxel)) {
          /*raycast_unknown_vec_.push_back(std::hash<voxblox::GlobalIndex>()(global_index));*/
          int new_voxels_added = std::ceil(step_size / og_step_size);
          for (int vi = 0; vi < new_voxels_added; ++vi)
            voxel_log.push_back(std::make_pair(voxel_coordi.cast<double>(),
                                               VoxelStatus::kUnknown));
          num_unknown_voxels += new_voxels_added;
          // double x_increment_dist = k * step_size *
          // sensor_params.resolution[0]; double y_increment_dist = k *
          // step_size * sensor_params.resolution[1]; double diag_increment_dist
          // = std::sqrt(std::pow(x_increment_dist, 2) +
          // std::pow(y_increment_dist, 2)); if(x_increment_dist <= voxel_size
          // && x > 0)
          //   starting_points(x-1, y) = std::min(num_steps-1, k);
          // if(y_increment_dist <= voxel_size && y > 0)
          //   starting_points(x, y-1) = std::min(num_steps-1, k);
          // if(diag_increment_dist <= voxel_size && x < width && y > 0)
          //   starting_points(x-1, y-1) = std::min(num_steps-1, k);

          continue;
        }
        // Free
        if (voxel->distance > distance_thres) {
          /*raycast_free_vec_.push_back(std::hash<voxblox::GlobalIndex>()(global_index));*/
          int new_voxels_added = std::ceil(step_size / og_step_size);
          num_free_voxels += new_voxels_added;
          for (int vi = 0; vi < new_voxels_added; ++vi)
            voxel_log.push_back(std::make_pair(voxel_coordi.cast<double>(),
                                               VoxelStatus::kFree));
          double x_increment_dist = step * sensor_params.resolution[0];
          double y_increment_dist = step * sensor_params.resolution[1];
          double diag_increment_dist = std::sqrt(std::pow(x_increment_dist, 2) +
                                                 std::pow(y_increment_dist, 2));
          if (x_increment_dist <= voxel_size && x < width)
            starting_points(x + 1, y) = std::min(ray_norm, step);
          if (y_increment_dist <= voxel_size && y > 0)
            starting_points(x, y - 1) = std::min(ray_norm, step);
          if (diag_increment_dist <= voxel_size && x < width && y > 0)
            starting_points(x + 1, y - 1) = std::min(ray_norm, step);
          continue;
        }
        // Occupied
        /*raycast_occupied_vec_.push_back(std::hash<voxblox::GlobalIndex>()(global_index));*/
        int new_voxels_added = std::ceil(step_size / og_step_size);
        for (int vi = 0; vi < new_voxels_added; ++vi)
          voxel_log.push_back(std::make_pair(voxel_coordi.cast<double>(),
                                             VoxelStatus::kOccupied));
        num_occupied_voxels += new_voxels_added;
        starting_points(x, y) = -1;
        double x_increment_dist = step * sensor_params.resolution[0];
        double y_increment_dist = step * sensor_params.resolution[1];
        double diag_increment_dist = std::sqrt(std::pow(x_increment_dist, 2) +
                                               std::pow(y_increment_dist, 2));
        if (x_increment_dist <= voxel_size && x < width)
          starting_points(x + 1, y) = -step;
        if (y_increment_dist <= voxel_size && y > 0)
          starting_points(x, y - 1) = -step;
        if (diag_increment_dist <= voxel_size && x < width && y > 0)
          starting_points(x + 1, y - 1) = -step;
        break;
      }
    }
  }
  gain_log =
      std::make_tuple(num_unknown_voxels, num_free_voxels, num_occupied_voxels);
}
#endif

template <typename SDFServerType, typename SDFVoxelType>
void MapManagerVoxblox<SDFServerType, SDFVoxelType>::getScanStatusIterative(
    Eigen::Vector3d& pos, std::vector<Eigen::Vector3d>& multiray_endpoints,
    std::tuple<int, int, int>& gain_log,
    std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& voxel_log,
    SensorParamsBase& sensor_params) {
  unsigned int num_unknown_voxels = 0, num_free_voxels = 0,
               num_occupied_voxels = 0;
  int height = sensor_params.height;
  int width = sensor_params.width;
  Eigen::MatrixXd starting_points(
      height, width);  // This will store the distance along the ray from where
                       // the raycasting should begin
  starting_points.setZero();

  const float voxel_size = sdf_layer_->voxel_size();
  const float voxel_size_inv = 1.0 / voxel_size;
  const float step_size_change_dist =
      4.0;  // After every these many meters increase the ray interpolation
            // distance
  const float step_size_change_dist_end =
      12.0;  // After this much distance stick to the last interpolation
             // distance

  const voxblox::Point start_scaled =
      pos.cast<voxblox::FloatingPoint>() * voxel_size_inv;

  const float distance_thres =
      occupancy_distance_voxelsize_factor_ * sdf_layer_->voxel_size() + 1e-6;

  // NOTES: no optimization / no twice-counting considerations possible without
  // refactoring planning strategy here

  // Iterate for every endpoint, insert unknown voxels found over every ray into
  // a set to avoid double-counting Important: do not use <VoxelIndex> type
  // directly, will break voxblox's implementations
  // Move away from std::unordered_set and work with std::vector + std::unique
  // count at the end (works best for now)
  /*std::vector<std::size_t> raycast_unknown_vec_, raycast_occupied_vec_,
  raycast_free_vec_; raycast_unknown_vec_.reserve(multiray_endpoints.size() *
  tsdf_integrator_config_.max_ray_length_m * voxel_size_inv); //optimize for
  number of rays raycast_free_vec_.reserve(multiray_endpoints.size() *
  tsdf_integrator_config_.max_ray_length_m * voxel_size_inv); //optimize for
  number of rays raycast_occupied_vec_.reserve(multiray_endpoints.size());
  //optimize for number of rays*/

  for (int y = height - 1; y >= 0; --y) {
    for (int x = 0; x < width; ++x) {
      float step_size = voxel_size * ray_cast_step_size_multiplier_;
      float step_size_inv = 1.0 / step_size;
      float og_step_size = step_size;

      int ep_ind = (height - y - 1) * width +
                   x;  // Index of the endpoint corresponding to this cell
      const double ray_start_point = starting_points(x, y);

      Eigen::Vector3d ray_normalized =
          (multiray_endpoints[ep_ind] - pos);  // Not yet noramlized
      double ray_norm = ray_normalized.norm();
      ray_normalized = ray_normalized / ray_norm;  // Normalized here
      Eigen::Vector3d start_point =
          pos + ray_normalized * std::abs(ray_start_point);
      Eigen::Vector3d truncated_ray = multiray_endpoints[ep_ind] - start_point;
      double truncated_ray_len = truncated_ray.norm();
      int num_steps =
          ray_norm * step_size_inv;  // This is the number of steps to traverse
                                     // the entire ray NOT the truncated ray

      if (ray_start_point < 0.0) {
        // Ray hits an occupied voxel
        ++num_occupied_voxels;
        voxel_log.push_back(
            std::make_pair(start_point, VoxelStatus::kOccupied));
        continue;
      }

      int count = 0;
      double prev_step_dist = ray_start_point;
      for (double step = ray_start_point; step <= ray_norm; step += step_size) {
        count++;

        // Update step size
        if (nonuniform_ray_cast_) {
          if (std::abs(prev_step_dist - step) > step_size_change_dist &&
              step <= step_size_change_dist_end) {
            prev_step_dist = step;
            step_size += og_step_size;
          }
        }

        Eigen::Vector3d voxel_coordi = (pos + ray_normalized * step);
        Eigen::Vector3d voxel_coordi_scaled = voxel_coordi * voxel_size_inv;

        voxblox::LongIndex center_voxel_index =
            voxblox::getGridIndexFromPoint<voxblox::LongIndex>(
                voxel_coordi.cast<voxblox::FloatingPoint>(), voxel_size_inv);

        SDFVoxelType* voxel =
            sdf_layer_->getVoxelPtrByGlobalIndex(center_voxel_index);
        // Unknown
        if (checkUnknownStatus(voxel)) {
          /*raycast_unknown_vec_.push_back(std::hash<voxblox::GlobalIndex>()(global_index));*/
          int new_voxels_added = std::ceil(step_size / og_step_size);
          for (int vi = 0; vi < new_voxels_added; ++vi)
            voxel_log.push_back(std::make_pair(voxel_coordi.cast<double>(),
                                               VoxelStatus::kUnknown));
          num_unknown_voxels += new_voxels_added;
          // double x_increment_dist = k * step_size *
          // sensor_params.resolution[0]; double y_increment_dist = k *
          // step_size * sensor_params.resolution[1]; double diag_increment_dist
          // = std::sqrt(std::pow(x_increment_dist, 2) +
          // std::pow(y_increment_dist, 2)); if(x_increment_dist <= voxel_size
          // && x > 0)
          //   starting_points(x-1, y) = std::min(num_steps-1, k);
          // if(y_increment_dist <= voxel_size && y > 0)
          //   starting_points(x, y-1) = std::min(num_steps-1, k);
          // if(diag_increment_dist <= voxel_size && x < width && y > 0)
          //   starting_points(x-1, y-1) = std::min(num_steps-1, k);

          continue;
        }
        // Free
        if (voxel->distance > distance_thres) {
          /*raycast_free_vec_.push_back(std::hash<voxblox::GlobalIndex>()(global_index));*/
          int new_voxels_added = std::ceil(step_size / og_step_size);
          num_free_voxels += new_voxels_added;
          for (int vi = 0; vi < new_voxels_added; ++vi)
            voxel_log.push_back(std::make_pair(voxel_coordi.cast<double>(),
                                               VoxelStatus::kFree));
          double x_increment_dist = step * sensor_params.resolution[0];
          double y_increment_dist = step * sensor_params.resolution[1];
          double diag_increment_dist = std::sqrt(std::pow(x_increment_dist, 2) +
                                                 std::pow(y_increment_dist, 2));
          if (x_increment_dist <= voxel_size && x < width)
            starting_points(x + 1, y) = std::min(ray_norm, step);
          if (y_increment_dist <= voxel_size && y > 0)
            starting_points(x, y - 1) = std::min(ray_norm, step);
          if (diag_increment_dist <= voxel_size && x < width && y > 0)
            starting_points(x + 1, y - 1) = std::min(ray_norm, step);
          continue;
        }
        // Occupied
        /*raycast_occupied_vec_.push_back(std::hash<voxblox::GlobalIndex>()(global_index));*/
        int new_voxels_added = std::ceil(step_size / og_step_size);
        for (int vi = 0; vi < new_voxels_added; ++vi)
          voxel_log.push_back(std::make_pair(voxel_coordi.cast<double>(),
                                             VoxelStatus::kOccupied));
        num_occupied_voxels += new_voxels_added;
        starting_points(x, y) = -1;
        double x_increment_dist = step * sensor_params.resolution[0];
        double y_increment_dist = step * sensor_params.resolution[1];
        double diag_increment_dist = std::sqrt(std::pow(x_increment_dist, 2) +
                                               std::pow(y_increment_dist, 2));
        if (x_increment_dist <= voxel_size && x < width)
          starting_points(x + 1, y) = -step;
        if (y_increment_dist <= voxel_size && y > 0)
          starting_points(x, y - 1) = -step;
        if (diag_increment_dist <= voxel_size && x < width && y > 0)
          starting_points(x + 1, y - 1) = -step;
        break;
      }
    }
  }
  gain_log =
      std::make_tuple(num_unknown_voxels, num_free_voxels, num_occupied_voxels);
}