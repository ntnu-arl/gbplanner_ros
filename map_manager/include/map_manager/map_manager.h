#ifndef MAP_MANAGER_H_
#define MAP_MANAGER_H_

#include "map_manager/map_manager_utils.h"

#include "map_manager/map_manager_voxblox_impl.h"

// namespace explorer {

class MapManager {
 public:
  

  MapManager(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

  double getResolution();
  bool getStatus();
  VoxelStatus getVoxelStatus(const Eigen::Vector3d& position);
  float getVoxelDistance(const Eigen::Vector3d& center);
  double getPointDistance(const Eigen::Vector3d& point);
  Eigen::Vector3d getPointGradient(const Eigen::Vector3d& point);

  VoxelStatus getRayStatus(const Eigen::Vector3d& view_point,
                                   const Eigen::Vector3d& voxel_to_test,
                                   bool stop_at_unknown_voxel);
  VoxelStatus getRayStatus(const Eigen::Vector3d& view_point,
                                   const Eigen::Vector3d& voxel_to_test,
                                   bool stop_at_unknown_voxel,
                                   Eigen::Vector3d& end_voxel,
                                   double& tsdf_dist);
  VoxelStatus getBoxStatus(const Eigen::Vector3d& center,
                                   const Eigen::Vector3d& size,
                                   bool stop_at_unknown_voxel);
  VoxelStatus getPathStatus(const Eigen::Vector3d& start,
                                    const Eigen::Vector3d& end,
                                    const Eigen::Vector3d& box_size,
                                    bool stop_at_unknown_voxel);
  bool augmentFreeBox(const Eigen::Vector3d& position,
                              const Eigen::Vector3d& box_size);

  void getScanStatus(
      Eigen::Vector3d& pos, std::vector<Eigen::Vector3d>& multiray_endpoints,
      std::tuple<int, int, int>& gain_log,
      std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& voxel_log,
      SensorParamsBase& sensor_params);
  void annotateCameraVoxels(Eigen::Vector3d& pos, std::vector<Eigen::Vector3d>& multiray_endpoints);
  void getCameraScanStatus(Eigen::Vector3d& pos, std::vector<Eigen::Vector3d>& multiray_endpoints,
    std::tuple<int, int, int>& gain_log,
    std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& voxel_log,
    SensorParamsBase& sensor_params);
  void getCameraScanStatus(StateVec& state, SensorParamsBase& sensor_params, std::vector<VoxelLog> &out_logs);

  void augmentFreeFrustum();

  void getFreeSpacePointCloud(std::vector<Eigen::Vector3d>, StateVec,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr);

  void extractLocalMap(const Eigen::Vector3d& center,
                               const Eigen::Vector3d& bounding_box_size,
                               std::vector<Eigen::Vector3d>& occupied_voxels,
                               std::vector<Eigen::Vector3d>& free_voxels);

  void extractLocalMapAlongAxis(
      const Eigen::Vector3d& center, const Eigen::Vector3d& axis,
      const Eigen::Vector3d& bounding_box_size,
      std::vector<Eigen::Vector3d>& occupied_voxels,
      std::vector<Eigen::Vector3d>& free_voxels);
  
  void getLocalPointcloud(const Eigen::Vector3d& center, const double& range,
                          const double& yaw,
                          pcl::PointCloud<pcl::PointXYZI>& pcl,
                          bool include_unknown_voxels = false);
  void getLocalPointcloud(const Eigen::Vector3d& center, const double& range,
                          const double& yaw, const Eigen::Vector2d& z_limits,
                          pcl::PointCloud<pcl::PointXYZI>& pcl,
                          bool include_unknown_voxels = false);

  void resetMap();

  void setRaycastingParams(bool nonuniform_ray_cast,
                           double ray_cast_step_size_multiplier);

  void setRobotRadius(double robot_radius);
  void setBoxCheckMethod(int m);
  void setLineCheckMethod(int m);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  std::shared_ptr<MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>> map_manager_impl_;
};

// }  // namespace explorer

#endif
