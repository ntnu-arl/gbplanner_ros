#ifndef MAP_MANAGER_H_
#define MAP_MANAGER_H_

#include <eigen3/Eigen/Dense>

#include "planner_common/params.h"

// namespace explorer {

class MapManager {
 public:
  enum VoxelStatus { kUnknown = 0, kOccupied, kFree };

  MapManager(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
      : nh_(nh), nh_private_(nh_private) {}
  virtual double getResolution() const = 0;
  virtual bool getStatus() const = 0;
  virtual VoxelStatus getVoxelStatus(const Eigen::Vector3d& position) const = 0;
  virtual VoxelStatus getRayStatus(const Eigen::Vector3d& view_point,
                                   const Eigen::Vector3d& voxel_to_test,
                                   bool stop_at_unknown_voxel) const = 0;
  virtual VoxelStatus getRayStatus(const Eigen::Vector3d& view_point,
                                   const Eigen::Vector3d& voxel_to_test,
                                   bool stop_at_unknown_voxel,
                                   Eigen::Vector3d& end_voxel,
                                   double& tsdf_dist) const = 0;
  virtual VoxelStatus getBoxStatus(const Eigen::Vector3d& center,
                                   const Eigen::Vector3d& size,
                                   bool stop_at_unknown_voxel) const = 0;
  virtual VoxelStatus getPathStatus(const Eigen::Vector3d& start,
                                    const Eigen::Vector3d& end,
                                    const Eigen::Vector3d& box_size,
                                    bool stop_at_unknown_voxel) const = 0;
  virtual bool augmentFreeBox(const Eigen::Vector3d& position,
                              const Eigen::Vector3d& box_size) = 0;

  virtual void getScanStatus(
      Eigen::Vector3d& pos, std::vector<Eigen::Vector3d>& multiray_endpoints,
      std::tuple<int, int, int>& gain_log,
      std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& voxel_log,
      SensorParamsBase& sensor_params) = 0;

  virtual void augmentFreeFrustum() = 0;

  virtual void extractLocalMap(const Eigen::Vector3d& center,
                               const Eigen::Vector3d& bounding_box_size,
                               std::vector<Eigen::Vector3d>& occupied_voxels,
                               std::vector<Eigen::Vector3d>& free_voxels) = 0;

  virtual void extractLocalMapAlongAxis(
      const Eigen::Vector3d& center, const Eigen::Vector3d& axis,
      const Eigen::Vector3d& bounding_box_size,
      std::vector<Eigen::Vector3d>& occupied_voxels,
      std::vector<Eigen::Vector3d>& free_voxels) = 0;

  virtual void resetMap() = 0;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
};

// }  // namespace explorer

#endif
