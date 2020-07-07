#include "gbplanner/map_manager_octomap_impl.h"

namespace explorer {
namespace gbplanner {
MapManagerOctomap::MapManagerOctomap(const ros::NodeHandle& nh,
                                     const ros::NodeHandle& nh_private)
    : MapManager(nh, nh_private) {
  octomap_manager_ = new volumetric_mapping::OctomapManager(nh, nh_private);
}

double MapManagerOctomap::getResolution() const {
  return octomap_manager_->getResolution();
}

bool MapManagerOctomap::getStatus() const { return true; }

MapManagerOctomap::VoxelStatus MapManagerOctomap::convertStatus(
    const volumetric_mapping::OctomapManager::CellStatus& status) const {
  switch (status) {
    case volumetric_mapping::OctomapManager::CellStatus::kFree:
      return MapManager::VoxelStatus::kFree;
      break;
    case volumetric_mapping::OctomapManager::CellStatus::kOccupied:
      return MapManager::VoxelStatus::kOccupied;
      break;
    case volumetric_mapping::OctomapManager::CellStatus::kUnknown:
      return MapManager::VoxelStatus::kUnknown;
      break;
  }
}

MapManagerOctomap::VoxelStatus MapManagerOctomap::getVoxelStatus(
    const Eigen::Vector3d& position) const {
  volumetric_mapping::OctomapManager::CellStatus status =
      octomap_manager_->getCellStatusPoint(position);
  return convertStatus(status);
}

MapManagerOctomap::VoxelStatus MapManagerOctomap::getRayStatus(
    const Eigen::Vector3d& view_point, const Eigen::Vector3d& voxel_to_test,
    bool stop_at_unknown_voxel) const {
  volumetric_mapping::OctomapManager::CellStatus status =
      octomap_manager_->getVisibility(view_point, voxel_to_test,
                                      stop_at_unknown_voxel);
  return convertStatus(status);
}

MapManagerOctomap::VoxelStatus MapManagerOctomap::getBoxStatus(
    const Eigen::Vector3d& center, const Eigen::Vector3d& size,
    bool stop_at_unknown_voxel) const {
  volumetric_mapping::OctomapManager::CellStatus status =
      octomap_manager_->getCellStatusBoundingBox(center, size);
  return convertStatus(status);
}

MapManagerOctomap::VoxelStatus MapManagerOctomap::getPathStatus(
    const Eigen::Vector3d& start, const Eigen::Vector3d& end,
    const Eigen::Vector3d& box_size, bool stop_at_unknown_voxel) const {
  volumetric_mapping::OctomapManager::CellStatus status =
      octomap_manager_->getDirectionalLineStatusBoundingBox(start, end,
                                                            box_size, true);
  return convertStatus(status);
}

bool MapManagerOctomap::augmentFreeBox(const Eigen::Vector3d& position,
                                       const Eigen::Vector3d& box_size) {
  octomap_manager_->setFree(position, box_size,
                            volumetric_mapping::BoundHandling::kDefault);
  return true;
}

void MapManagerOctomap::getFreeSpacePointCloud(
    std::vector<Eigen::Vector3d> multiray_endpoints, StateVec state,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  pcl::PointCloud<pcl::PointXYZ> free_cloud;

  for (auto ep : multiray_endpoints) {
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

void MapManagerOctomap::getScanStatus(
    Eigen::Vector3d& pos, std::vector<Eigen::Vector3d>& multiray_endpoints,
    std::tuple<int, int, int>& gain_log,
    std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& voxel_log) {
  std::vector<std::pair<Eigen::Vector3d,
                        volumetric_mapping::OctomapManager::CellStatus>>
      voxel_log_temp;
  voxel_log_temp.clear();
  std::vector<std::tuple<int, int, int>> gain_log_vec;
  octomap_manager_->getScanStatus(pos, multiray_endpoints, gain_log_vec,
                                  voxel_log_temp);

  int num_unknown_voxels = 0, num_free_voxels = 0, num_occupied_voxels = 0;
  for (auto& gl : gain_log_vec) {
    num_unknown_voxels += std::get<0>(gl);
    num_free_voxels += std::get<1>(gl);
    num_occupied_voxels += std::get<2>(gl);
  }
  gain_log =
      std::make_tuple(num_unknown_voxels, num_free_voxels, num_occupied_voxels);
  for (auto& vl : voxel_log_temp) {
    voxel_log.push_back(std::make_pair(vl.first, convertStatus(vl.second)));
  }
}

void MapManagerOctomap::augmentFreeFrustum() {
  octomap_manager_->augmentFreeFrustum();
}

void MapManagerOctomap::extractLocalMap(
    const Eigen::Vector3d& center, const Eigen::Vector3d& bounding_box_size,
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

void MapManagerOctomap::extractLocalMapAlongAxis(
    const Eigen::Vector3d& center, const Eigen::Vector3d& axis,
    const Eigen::Vector3d& bounding_box_size,
    std::vector<Eigen::Vector3d>& occupied_voxels,
    std::vector<Eigen::Vector3d>& free_voxels) {
  occupied_voxels.clear();
  free_voxels.clear();
  double resolution = getResolution();

  int Nx = std::ceil(bounding_box_size.x() / (2 * resolution)) * 2;
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
