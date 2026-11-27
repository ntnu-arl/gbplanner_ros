#pragma once
#include "gbplanner_local/parameters.hpp"
#include "gbplanner_local/voxel_map.hpp"
#include <cstddef>
#include <string>
#include <vector>

namespace gbplanner2 {
struct FreeSpaceSensor {
  SensorParameters sensor;
  // Camera outputs use the optical frame; lidar uses its configured frame.
  std::string frame;
};
struct FreeSpaceCloud {
  std::string name,frame;
  std::vector<Eigen::Vector3d> points;
  std::size_t tested_rays{0},blocked_rays{0};
};

// Port of freePointCloudtimerCallback / getFreeSpacePointCloud. base_pose is
// the measured robot body pose, not the ground planner's elevated root state.
// Each returned point is a max-range endpoint of a ray with no occupied cell;
// unknown cells are intentionally allowed for the free-space clearing input.
std::vector<FreeSpaceCloud> buildFreeSpaceClouds(
    const MapQuery& map,const StateVec& base_pose,const std::vector<FreeSpaceSensor>& sensors);
}  // namespace gbplanner2
