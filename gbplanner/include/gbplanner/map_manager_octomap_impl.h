#ifndef MAP_MANAGER_OCTOMAP_IMPL_H_
#define MAP_MANAGER_OCTOMAP_IMPL_H_

#include "gbplanner/map_manager.h"
#include "gbplanner/params.h"
#include "octomap_world/octomap_manager.h"

namespace explorer {
namespace gbplanner {
class MapManagerOctomap : MapManager {
 public:
  MapManagerOctomap(const ros::NodeHandle& nh,
                    const ros::NodeHandle& nh_private);

  double getResolution() const;
  bool getStatus() const;
  VoxelStatus getVoxelStatus(const Eigen::Vector3d& position) const;
  VoxelStatus getRayStatus(const Eigen::Vector3d& view_point,
                           const Eigen::Vector3d& voxel_to_test,
                           bool stop_at_unknown_voxel) const;
  VoxelStatus getBoxStatus(const Eigen::Vector3d& center,
                           const Eigen::Vector3d& size,
                           bool stop_at_unknown_voxel) const;
  VoxelStatus getPathStatus(const Eigen::Vector3d& start,
                            const Eigen::Vector3d& end,
                            const Eigen::Vector3d& box_size,
                            bool stop_at_unknown_voxel) const;
  bool augmentFreeBox(const Eigen::Vector3d& position,
                      const Eigen::Vector3d& box_size);

  void getFreeSpacePointCloud(std::vector<Eigen::Vector3d>, StateVec,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr);

  void getScanStatus(
      Eigen::Vector3d& pos, std::vector<Eigen::Vector3d>& multiray_endpoints,
      std::tuple<int, int, int>& gain_log,
      std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& voxel_log);

  void augmentFreeFrustum();

  void extractLocalMap(const Eigen::Vector3d& center,
                       const Eigen::Vector3d& bounding_box_size,
                       std::vector<Eigen::Vector3d>& occupied_voxels,
                       std::vector<Eigen::Vector3d>& free_voxels);

  void extractLocalMapAlongAxis(const Eigen::Vector3d& center,
                                const Eigen::Vector3d& axis,
                                const Eigen::Vector3d& bounding_box_size,
                                std::vector<Eigen::Vector3d>& occupied_voxels,
                                std::vector<Eigen::Vector3d>& free_voxels);

  void resetMap() { octomap_manager_->resetMap(); }

 private:
  volumetric_mapping::OctomapManager* octomap_manager_;

  VoxelStatus convertStatus(
      const volumetric_mapping::OctomapManager::CellStatus& status) const;
};
}  // namespace gbplanner
}  // namespace explorer

#endif