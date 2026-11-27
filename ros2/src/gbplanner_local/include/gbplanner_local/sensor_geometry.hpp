#pragma once

#include <Eigen/Core>
#include <planner_common/graph_base.h>
#include <utility>
#include <vector>

namespace gbplanner2 {
struct SensorParameters;

// Sensor parameters are validated when configured. This preserves the original
// isInsideFOV convention: R^-1*yaw^-1*(point-origin)-center_offset, which differs
// from the offset order used when generating gain-ray endpoints.
bool sensorContains(const SensorParameters& sensor,const StateVec& viewpoint,
                    const Eigen::Vector3d& point);

// Static camera pyramid: four origin-to-corner edges, then four rim edges.
// Upstream getFrustumEdges omits center_offset and uses the robot origin as the
// apex. Preserve that drawing convention. Lidar has no pyramid and returns {}.
std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d>> sensorFrustum(
    const SensorParameters& sensor,const StateVec& viewpoint);

}  // namespace gbplanner2
