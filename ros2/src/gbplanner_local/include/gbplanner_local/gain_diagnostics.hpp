#pragma once
#include "gbplanner_local/gain_raycast.hpp"
#include <planner_common/graph_manager.h>
#include <string>
#include <vector>

namespace gbplanner2 {
struct SensorGain {
  std::string name;
  std::string mode{"ray"};
  VolumetricGain gain;
  std::size_t ray_count{0};
  std::string ray_method{"none"};
  std::size_t ray_width{0},ray_height{0};
  GainRaycastDiagnostics raycast;
  std::size_t voxel_count{0},fov_voxel_count{0},visibility_queries{0};
};
struct GainContribution {
  Eigen::Vector3d point{Eigen::Vector3d::Zero()};
  VoxelStatus status{VoxelStatus::Unknown};
  std::size_t weight{0};
};
struct SensorGainGeometry {
  SensorGainGeometry(){result.gain.reset();}
  SensorGain result;
  // Ray mode: complete sensor endpoints, NOT obstacle hit points. Voxel mode:
  // endpoints of the visibility rays actually submitted to MapQuery::ray.
  std::vector<Eigen::Vector3d> endpoints;
  // Accepted reward events in original double precision and visitation order.
  // Iterative-ray events may be inherited hits; they are not all map queries.
  std::vector<GainContribution> contributions;
  Eigen::Vector3d minimum{Eigen::Vector3d::Zero()},maximum{Eigen::Vector3d::Zero()};
  bool roi_valid{false};
  double step{0.},map_resolution{0.};
  bool map_resolution_valid{false};
  // complete covers this sensor's evaluation, independently of storage caps.
  // Iterative raycast totals exist only after the raycaster returns normally.
  bool complete{false},raycast_complete{false};
  std::size_t total_endpoints{0},dropped_endpoints{0},total_samples{0},dropped_samples{0};
};
struct GainGeometry {
  GainGeometry(){gain.reset();}
  static constexpr std::size_t kMaxEndpoints=4096,kMaxSamples=8192;
  // entered means computeExplorationGain ran; evaluated means the selected
  // viewpoint's exact evaluation began (it may subsequently throw).
  bool enabled{false},entered{false},evaluated{false};
  // complete requires a completed exact evaluation AND uncapped geometry.
  bool complete{false},partial{false},truncated{false};
  int requested_id{20},vertex_id{-1},source_id{-1};
  StateVec state{StateVec::Zero()};
  bool state_valid{false};
  std::string mode,policy,reason{"disabled"};
  VolumetricGain gain;
  bool gain_valid{false};
  std::vector<SensorGainGeometry> sensors;
  std::size_t total_endpoints{0},dropped_endpoints{0},total_samples{0},dropped_samples{0};
  void endpoint(SensorGainGeometry& sensor,const Eigen::Vector3d& point){
    ++total_endpoints;++sensor.total_endpoints;
    if(total_endpoints-dropped_endpoints>kMaxEndpoints){
      ++dropped_endpoints;++sensor.dropped_endpoints;truncated=true;partial=true;return;
    }
    sensor.endpoints.push_back(point);
  }
  void contribution(SensorGainGeometry& sensor,const Eigen::Vector3d& point,
                    VoxelStatus status,std::size_t weight){
    ++total_samples;++sensor.total_samples;
    if(total_samples-dropped_samples>kMaxSamples){
      ++dropped_samples;++sensor.dropped_samples;truncated=true;partial=true;return;
    }
    sensor.contributions.push_back({point,status,weight});
  }
};
}  // namespace gbplanner2
