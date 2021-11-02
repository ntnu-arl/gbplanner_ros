#ifndef MAP_MANAGER_VOXBLOX_IMPL_H_
#define MAP_MANAGER_VOXBLOX_IMPL_H_

#include <algorithm>
#include <cmath>  // for PI
#include <cstdint>
#include <iomanip>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <voxblox/interpolator/interpolator.h>
#include <voxblox/utils/layer_utils.h>
#include <voxblox/utils/planning_utils.h>
#include <voxblox/utils/planning_utils_inl.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox_ros/ros_params.h>
#include <voxblox_ros/transformer.h>

#include "planner_common/map_manager.h"
#include "planner_common/params.h"

#define use_tsdf

#ifdef use_tsdf
typedef voxblox::TsdfServer MapManagerVoxbloxServer;
typedef voxblox::TsdfVoxel MapManagerVoxbloxVoxel;
#else
typedef voxblox::EsdfServer MapManagerVoxbloxServer;
typedef voxblox::EsdfVoxel MapManagerVoxbloxVoxel;
#endif

// typedef voxblox::TsdfServer MapManagerVoxbloxServer;
// typedef voxblox::TsdfVoxel MapManagerVoxbloxVoxel;

// Experimental: enable __gnu_parallel:: variants
#ifdef _GLIBCXX_PARALLEL
#include <parallel/algorithm>
#endif

namespace voxblox {
namespace utils {

// An extension of some Voxblox functionalities.
template <typename VoxelType>
void getBoxAroundPoint(const Layer<VoxelType>& layer, const Point& center,
                       const Eigen::Vector3d& bounding_box_size,
                       HierarchicalIndexMap* block_voxel_list) {
  CHECK_NOTNULL(block_voxel_list);
  float voxel_size = layer.voxel_size();
  float voxel_size_inv = 1.0 / voxel_size;
  int voxels_per_side = layer.voxels_per_side();

  const GlobalIndex center_index =
      getGridIndexFromPoint<GlobalIndex>(center, voxel_size_inv);
  const Eigen::Vector3f halfrange_in_voxels =
      (bounding_box_size / 2 / voxel_size).cast<FloatingPoint>();

  for (FloatingPoint x = -halfrange_in_voxels[0]; x <= halfrange_in_voxels[0];
       x++) {
    for (FloatingPoint y = -halfrange_in_voxels[1]; y <= halfrange_in_voxels[1];
         y++) {
      for (FloatingPoint z = -halfrange_in_voxels[2];
           z <= halfrange_in_voxels[2]; z++) {
        Point point_voxel_space(x, y, z);

        GlobalIndex voxel_offset_index(std::floor(point_voxel_space.x()),
                                       std::floor(point_voxel_space.y()),
                                       std::floor(point_voxel_space.z()));
        // Get the block and voxel indices from this.
        BlockIndex block_index;
        VoxelIndex voxel_index;

        getBlockAndVoxelIndexFromGlobalVoxelIndex(
            voxel_offset_index + center_index, voxels_per_side, &block_index,
            &voxel_index);
        (*block_voxel_list)[block_index].push_back(voxel_index);
      }
    }
  }
}

template <typename VoxelType>
void getAndAllocateBoxAroundPoint(const Point& center,
                                  const Eigen::Vector3d& bounding_box_size,
                                  Layer<VoxelType>* layer,
                                  HierarchicalIndexMap* block_voxel_list) {
  CHECK_NOTNULL(layer);
  CHECK_NOTNULL(block_voxel_list);
  getBoxAroundPoint(*layer, center, bounding_box_size, block_voxel_list);
  for (auto it = block_voxel_list->begin(); it != block_voxel_list->end();
       ++it) {
    layer->allocateBlockPtrByIndex(it->first);
  }
}

}  // namespace utils
}  // namespace voxblox

// An extension of std, requirement to support
// std::unordered_set<Eigen::Matrix<int64_t, 3, 1>>
namespace std {
// hash an Eigen::Matrix from
// https://github.com/ethz-asl/map_api/blob/master/map-api-common/include/map-api-common/eigen-hash.h
// based on boost::hash_combine
// http://www.boost.org/doc/libs/1_55_0/doc/html/hash/reference.html#boost.hash_combine
template <typename Scalar, int Rows, int Cols>
struct hash<Eigen::Matrix<Scalar, Rows, Cols>> {
  size_t operator()(const Eigen::Matrix<Scalar, Rows, Cols>& matrix) const {
    size_t seed = 0;
    for (Eigen::Index i = 0; i < matrix.size(); ++i) {
      Scalar elem = *(matrix.data() + i);
      seed ^=
          std::hash<Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

}  // namespace std

// namespace explorer {
template <typename SDFServerType, typename SDFVoxelType>
class MapManagerVoxblox : MapManager {
 public:
  MapManagerVoxblox(const ros::NodeHandle& nh,
                    const ros::NodeHandle& nh_private);

  double getResolution() const;
  bool getStatus() const;
  VoxelStatus getVoxelStatus(const Eigen::Vector3d& position) const;

  VoxelStatus getRayStatus(const Eigen::Vector3d& view_point,
                           const Eigen::Vector3d& voxel_to_test,
                           bool stop_at_unknown_voxel) const;
  VoxelStatus getRayStatus(const Eigen::Vector3d& view_point,
                           const Eigen::Vector3d& voxel_to_test,
                           bool stop_at_unknown_voxel,
                           Eigen::Vector3d& end_voxel, double& tsdf_dist) const;
  VoxelStatus getBoxStatus(const Eigen::Vector3d& center,
                           const Eigen::Vector3d& size,
                           bool stop_at_unknown_voxel) const;

  float getVoxelDistance(const Eigen::Vector3d& center) const;
  double getPointDistance(const Eigen::Vector3d& point) const;

  VoxelStatus getPathStatus(const Eigen::Vector3d& start,
                            const Eigen::Vector3d& end,
                            const Eigen::Vector3d& box_size,
                            bool stop_at_unknown_voxel) const;

  bool augmentFreeBox(const Eigen::Vector3d& position,
                      const Eigen::Vector3d& box_size);
  void getScanStatus(
      Eigen::Vector3d& pos, std::vector<Eigen::Vector3d>& multiray_endpoints,
      std::tuple<int, int, int>& gain_log,
      std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& voxel_log,
      SensorParamsBase& sensor_params);

  void getScanStatusIterative(
      Eigen::Vector3d& pos, std::vector<Eigen::Vector3d>& multiray_endpoints,
      std::tuple<int, int, int>& gain_log,
      std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& voxel_log,
      SensorParamsBase& sensor_params);

  void augmentFreeFrustum();

  void getFreeSpacePointCloud(std::vector<Eigen::Vector3d>, StateVec,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr);

  void extractLocalMap(const Eigen::Vector3d& center,
                       const Eigen::Vector3d& bounding_box_size,
                       std::vector<Eigen::Vector3d>& occupied_voxels,
                       std::vector<Eigen::Vector3d>& free_voxels);

  void extractLocalMapAlongAxis(const Eigen::Vector3d& center,
                                const Eigen::Vector3d& axis,
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

  double getTruncationDistance() {
    return tsdf_integrator_config_.default_truncation_distance;
  }

  void resetMap() { sdf_server_.clear(); }

  pcl::PointXYZI eigenVec3dToPCLPoint(Eigen::Vector3d& vec) const;

  void setRaycastingParams(bool nonuniform_ray_cast,
                           double ray_cast_step_size_multiplier) {
    nonuniform_ray_cast_ = nonuniform_ray_cast;
    ray_cast_step_size_multiplier_ = ray_cast_step_size_multiplier;
  }

  void setRobotRadius(double robot_radius) { robot_radius_ = robot_radius; }

 private:
  VoxelStatus getBoxStatusInVoxels(
      const voxblox::LongIndex& bounding_box_center,
      const voxblox::AnyIndex& bounding_box_voxels,
      bool stop_at_unknown_voxel) const;

  voxblox::Layer<SDFVoxelType>* getSDFLayer();
  bool checkUnknownStatus(const SDFVoxelType*) const;
  void clearIfUnknown(SDFVoxelType& voxel);

  SDFServerType sdf_server_;
  voxblox::Layer<SDFVoxelType>* sdf_layer_;

  voxblox::Interpolator<SDFVoxelType>* interpolator_;

  // multiplier of a single voxel size to consider as a distance metric for
  // occupancy threshold
  float occupancy_distance_voxelsize_factor_;

  // Separate object to setup because tsdf_server_ has
  // protected tsdf_integrator_, otherwise could use getConfig()
  voxblox::TsdfIntegratorBase::Config tsdf_integrator_config_;
  voxblox::EsdfIntegrator::Config esdf_integrator_config_;

  bool nonuniform_ray_cast_ = true;
  double ray_cast_step_size_multiplier_ = 1.0;

  // Used when single point collision checking is used (compilation flag
  // COL_CHECK_METHOD = 1) Note: The TSDF truncation distance has to be larger
  // than the robot radius to use this method No such restrictions for using
  // ESDF
  double robot_radius_;
};

// Helper
template class MapManagerVoxblox<MapManagerVoxbloxServer,
                                 MapManagerVoxbloxVoxel>;

#endif
