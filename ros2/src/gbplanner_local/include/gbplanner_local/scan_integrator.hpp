#pragma once
#include "gbplanner_local/voxel_map.hpp"

namespace gbplanner2 {
struct IntegrationResult {
  std::size_t accepted_points{0},skipped_points{0},free_updates{0},occupied_updates{0};
};
// Minimal deterministic occupancy adapter for the migration testbed. Free rays
// and occupied endpoints are integrated per scan; this is not upstream Voxblox.
class ScanIntegrator {
 public:
  ScanIntegrator(double min_range,double max_range);
  IntegrationResult integrate(VoxelMap& map,const Eigen::Vector3d& origin,
                               const std::vector<Eigen::Vector3d>& endpoints) const;
 private:
  double min_range_,max_range_;
};
}
