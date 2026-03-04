#include "map_manager/map_manager.h"

MapManager::MapManager(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
  : nh_(nh), nh_private_(nh_private) 
{
  map_manager_impl_ = std::make_shared<MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>>(nh, nh_private);
}

double MapManager::getResolution()
{
  return map_manager_impl_->getResolution();
}

bool MapManager::getStatus()
{
  return map_manager_impl_->getStatus();
}

VoxelStatus MapManager::getVoxelStatus(const Eigen::Vector3d& position)
{
  return map_manager_impl_->getVoxelStatus(position);
}

float MapManager::getVoxelDistance(const Eigen::Vector3d& center)
{
  return map_manager_impl_->getVoxelDistance(center);
}

double MapManager::getPointDistance(const Eigen::Vector3d& point)
{
  return map_manager_impl_->getPointDistance(point);
}

Eigen::Vector3d MapManager::getPointGradient(const Eigen::Vector3d& point)
{
  return map_manager_impl_->getPointGradient(point);
}

VoxelStatus MapManager::getRayStatus(const Eigen::Vector3d& view_point,
                                  const Eigen::Vector3d& voxel_to_test,
                                  bool stop_at_unknown_voxel)
{
  return map_manager_impl_->getRayStatus(view_point,
                                  voxel_to_test,
                                  stop_at_unknown_voxel);
}

VoxelStatus MapManager::getRayStatus(const Eigen::Vector3d& view_point,
                                  const Eigen::Vector3d& voxel_to_test,
                                  bool stop_at_unknown_voxel,
                                  Eigen::Vector3d& end_voxel,
                                  double& tsdf_dist)
{
  return map_manager_impl_->getRayStatus(view_point,
                                  voxel_to_test,
                                  stop_at_unknown_voxel,
                                  end_voxel,
                                  tsdf_dist);
}

VoxelStatus MapManager::getBoxStatus(const Eigen::Vector3d& center,
                                  const Eigen::Vector3d& size,
                                  bool stop_at_unknown_voxel)
{
  return map_manager_impl_->getBoxStatus(center, size, stop_at_unknown_voxel);
}

VoxelStatus MapManager::getPathStatus(const Eigen::Vector3d& start,
                                  const Eigen::Vector3d& end,
                                  const Eigen::Vector3d& box_size,
                                  bool stop_at_unknown_voxel)
{
  return map_manager_impl_->getPathStatus(start, end, box_size, stop_at_unknown_voxel);
}

bool MapManager::augmentFreeBox(const Eigen::Vector3d& position,
                            const Eigen::Vector3d& box_size)
{
  return map_manager_impl_->augmentFreeBox(position, box_size);
}

void MapManager::getFreeSpacePointCloud(std::vector<Eigen::Vector3d> multiray_endpoints, StateVec state,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  map_manager_impl_->getFreeSpacePointCloud(multiray_endpoints, state,
    cloud);
}

void MapManager::getScanStatus(
    Eigen::Vector3d& pos, std::vector<Eigen::Vector3d>& multiray_endpoints,
    std::tuple<int, int, int>& gain_log,
    std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& voxel_log,
    SensorParamsBase& sensor_params)
{
  map_manager_impl_->getScanStatus( pos, multiray_endpoints,
    gain_log, voxel_log, sensor_params);
}

void MapManager::annotateCameraVoxels(Eigen::Vector3d& pos, std::vector<Eigen::Vector3d>& multiray_endpoints)
{
  map_manager_impl_->annotateCameraVoxels(pos, multiray_endpoints);
}

void MapManager::getCameraScanStatus(Eigen::Vector3d& pos, std::vector<Eigen::Vector3d>& multiray_endpoints,
  std::tuple<int, int, int>& gain_log,
  std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& voxel_log,
  SensorParamsBase& sensor_params)
{
  map_manager_impl_->getCameraScanStatus(pos, multiray_endpoints,
          gain_log, voxel_log, sensor_params);
}

void MapManager::getCameraScanStatus(StateVec& state, SensorParamsBase& sensor_params, std::vector<VoxelLog> &out_logs)
{
  map_manager_impl_->getCameraScanStatus(state, sensor_params, out_logs);
}

void MapManager::augmentFreeFrustum()
{
  map_manager_impl_->augmentFreeFrustum();
}

void MapManager::extractLocalMap(const Eigen::Vector3d& center,
                              const Eigen::Vector3d& bounding_box_size,
                              std::vector<Eigen::Vector3d>& occupied_voxels,
                              std::vector<Eigen::Vector3d>& free_voxels)
{
  map_manager_impl_->extractLocalMap(center,
                              bounding_box_size, occupied_voxels, free_voxels);
}

void MapManager::extractLocalMapAlongAxis(
    const Eigen::Vector3d& center, const Eigen::Vector3d& axis,
    const Eigen::Vector3d& bounding_box_size,
    std::vector<Eigen::Vector3d>& occupied_voxels,
    std::vector<Eigen::Vector3d>& free_voxels)
{
  map_manager_impl_->extractLocalMapAlongAxis(
    center, axis,
    bounding_box_size,
    occupied_voxels,
    free_voxels);
}

void MapManager::getLocalPointcloud(const Eigen::Vector3d& center, const double& range,
                        const double& yaw,
                        pcl::PointCloud<pcl::PointXYZI>& pcl,
                        bool include_unknown_voxels)
{
  map_manager_impl_->getLocalPointcloud(center, range,
                        yaw, pcl, include_unknown_voxels);
}
void MapManager::getLocalPointcloud(const Eigen::Vector3d& center, const double& range,
                        const double& yaw, const Eigen::Vector2d& z_limits,
                        pcl::PointCloud<pcl::PointXYZI>& pcl,
                        bool include_unknown_voxels)
{
  map_manager_impl_->getLocalPointcloud(center, range,
                        yaw, z_limits, pcl,
                        include_unknown_voxels);
}                        

void MapManager::resetMap()
{
  map_manager_impl_->resetMap();
}

void MapManager::setRaycastingParams(bool nonuniform_ray_cast,
                          double ray_cast_step_size_multiplier)
{
  map_manager_impl_->setRaycastingParams(nonuniform_ray_cast,
                          ray_cast_step_size_multiplier);
}

void MapManager::setRobotRadius(double robot_radius)
{
  map_manager_impl_->setRobotRadius(robot_radius);
}

void MapManager::setBoxCheckMethod(int m)
{
  map_manager_impl_->setBoxCheckMethod(m);
}

void MapManager::setLineCheckMethod(int m)
{
  map_manager_impl_->setLineCheckMethod(m);
}
