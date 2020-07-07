#include "gbplanner/map_manager_voxblox_impl.h"

namespace explorer {
namespace gbplanner {

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

  // Get local parameters from passed nodehandle
  if (!nh_private.getParam("occupancy_distance_voxelsize_factor",
                           occupancy_distance_voxelsize_factor_)) {
    ROS_INFO_STREAM(
        "MapManagerVoxblox: failed to find parameter for "
        "occupancy_distance_voxelsize_factor, using default of: "
        << occupancy_distance_voxelsize_factor_);
  }

  // Setup E/TsdfIntegratorBase::Config amd object separately (also called in
  // e/tsdf_server_ ctor)
  tsdf_integrator_config_ =
      voxblox::getTsdfIntegratorConfigFromRosParam(nh_private);
  esdf_integrator_config_ =
      voxblox::getEsdfIntegratorConfigFromRosParam(nh_private);
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
MapManagerVoxblox<SDFServerType, SDFVoxelType>::getBoxStatus(
    const Eigen::Vector3d& center, const Eigen::Vector3d& size,
    bool stop_at_unknown_voxel) const {
  float voxel_size = sdf_layer_->voxel_size();
  float voxel_size_inv = 1.0 / voxel_size;

  // Get the center of the bounding box as a global index.
  voxblox::LongIndex center_voxel_index =
      voxblox::getGridIndexFromPoint<voxblox::LongIndex>(
          center.cast<voxblox::FloatingPoint>(), voxel_size_inv);

  // Get the bounding box size in terms of voxels.
  voxblox::AnyIndex box_voxels(std::ceil(size.x() * voxel_size_inv),
                               std::ceil(size.y() * voxel_size_inv),
                               std::ceil(size.z() * voxel_size_inv));

  // Iterate over all voxels in the bounding box.
  return getBoxStatusInVoxels(center_voxel_index, box_voxels,
                              stop_at_unknown_voxel);
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
       voxel_index.x() <= box_center.x() + box_voxels.x() / 2;
       voxel_index.x()++) {
    for (voxel_index.y() = box_center.y() - box_voxels.y() / 2;
         voxel_index.y() <= box_center.y() + box_voxels.y() / 2;
         voxel_index.y()++) {
      for (voxel_index.z() = box_center.z() - box_voxels.z() / 2;
           voxel_index.z() <= box_center.z() + box_voxels.z() / 2;
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

template <typename SDFServerType, typename SDFVoxelType>
typename MapManagerVoxblox<SDFServerType, SDFVoxelType>::VoxelStatus
MapManagerVoxblox<SDFServerType, SDFVoxelType>::getPathStatus(
    const Eigen::Vector3d& start, const Eigen::Vector3d& end,
    const Eigen::Vector3d& box_size, bool stop_at_unknown_voxel) const {
  // Cast ray along the center to make sure we don't miss anything.
  float voxel_size = sdf_layer_->voxel_size();
  float voxel_size_inv = 1.0 / voxel_size;

  const voxblox::Point start_scaled =
      start.cast<voxblox::FloatingPoint>() * voxel_size_inv;
  const voxblox::Point end_scaled =
      end.cast<voxblox::FloatingPoint>() * voxel_size_inv;

  voxblox::LongIndexVector global_voxel_indices;
  voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);

  // Get the bounding box size in terms of voxels.
  voxblox::AnyIndex box_voxels(std::ceil(box_size.x() * voxel_size_inv),
                               std::ceil(box_size.y() * voxel_size_inv),
                               std::ceil(box_size.z() * voxel_size_inv));

  // Iterate over the ray.
  VoxelStatus current_status = VoxelStatus::kFree;
  for (const voxblox::GlobalIndex& global_index : global_voxel_indices) {
    VoxelStatus box_status =
        getBoxStatusInVoxels(global_index, box_voxels, stop_at_unknown_voxel);
    if (box_status == VoxelStatus::kOccupied) {
      return box_status;
    }
    if (stop_at_unknown_voxel && box_status == VoxelStatus::kUnknown) {
      return box_status;
    }
    current_status = box_status;
  }
  return current_status;
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
void MapManagerVoxblox<SDFServerType, SDFVoxelType>::getFreeSpacePointCloud(
    std::vector<Eigen::Vector3d> multiray_endpoints, StateVec state,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  pcl::PointCloud<pcl::PointXYZ> free_cloud;

  Eigen::Vector3d state_vec(state[0], state[1], state[2]);
  for (auto ep : multiray_endpoints) {
    VoxelStatus vs = getRayStatus(state_vec, ep, false);
    if (vs != VoxelStatus::kOccupied) {
      pcl::PointXYZ data;
      Eigen::Matrix3d rot_W2B;
      rot_W2B = Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
      Eigen::Matrix3d rot_B2W;
      rot_B2W = rot_W2B.inverse();
      Eigen::Vector3d origin(state[0], state[1], state[2]);
      Eigen::Vector3d epb = rot_B2W * (ep - origin);
      // Eigen::Vector3d epb = ep;
      data.x = epb(0);
      data.y = epb(1);
      data.z = epb(2);

      cloud->points.push_back(data);
    }
  }
}

template <typename SDFServerType, typename SDFVoxelType>
void MapManagerVoxblox<SDFServerType, SDFVoxelType>::getScanStatus(
    Eigen::Vector3d& pos, std::vector<Eigen::Vector3d>& multiray_endpoints,
    std::tuple<int, int, int>& gain_log,
    std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& voxel_log) {
  unsigned int num_unknown_voxels = 0, num_free_voxels = 0,
               num_occupied_voxels = 0;

  const float voxel_size = sdf_layer_->voxel_size();
  const float voxel_size_inv = 1.0 / voxel_size;

  const voxblox::Point start_scaled =
      pos.cast<voxblox::FloatingPoint>() * voxel_size_inv;

  const float distance_thres =
      occupancy_distance_voxelsize_factor_ * sdf_layer_->voxel_size() + 1e-6;

  // GBPLANNER_NOTES: no optimization / no twice-counting considerations
  // possible without refactoring planning strategy here

  // Iterate for every endpoint, insert unknown voxels found over every ray into
  // a set to avoid double-counting Important: do not use <VoxelIndex> type
  // directly, will break voxblox's implementations
  // Move away from std::unordered_set and work with std::vector + std::unique
  // count at the end (works best for now)
  voxel_log.reserve(multiray_endpoints.size() *
                    tsdf_integrator_config_.max_ray_length_m *
                    voxel_size_inv);  // optimize for number of rays
  for (size_t i = 0; i < multiray_endpoints.size(); ++i) {
    const voxblox::Point end_scaled =
        multiray_endpoints[i].cast<voxblox::FloatingPoint>() * voxel_size_inv;

    voxblox::LongIndexVector global_voxel_indices;
    voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);
    // Iterate over the ray.
    for (size_t k = 0; k < global_voxel_indices.size(); ++k) {
      const voxblox::GlobalIndex& global_index = global_voxel_indices[k];
      SDFVoxelType* voxel = sdf_layer_->getVoxelPtrByGlobalIndex(global_index);
      // Unknown
      if (checkUnknownStatus(voxel)) {
        ++num_unknown_voxels;
        voxel_log.push_back(std::make_pair(
            voxblox::getCenterPointFromGridIndex(global_index, voxel_size)
                .cast<double>(),
            VoxelStatus::kUnknown));
        continue;
      }
      // Free
      if (voxel->distance > distance_thres) {
        ++num_free_voxels;
        voxel_log.push_back(std::make_pair(
            voxblox::getCenterPointFromGridIndex(global_index, voxel_size)
                .cast<double>(),
            VoxelStatus::kFree));
        continue;
      }
      // Occupied
      ++num_occupied_voxels;
      voxel_log.push_back(std::make_pair(
          voxblox::getCenterPointFromGridIndex(global_index, voxel_size)
              .cast<double>(),
          VoxelStatus::kOccupied));
      break;
    }
  }
  gain_log =
      std::make_tuple(num_unknown_voxels, num_free_voxels, num_occupied_voxels);
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
  ROS_WARN_THROTTLE(5.0,
                    "MapManagerVoxblox::extractLocalMap --> Temporary solution "
                    "to be consistent with Octomap interface.");
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
}  // namespace gbplanner
}  // namespace explorer
