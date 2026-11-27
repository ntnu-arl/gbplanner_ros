// Port of GBPlanner 2 MapManagerVoxblox::getScanStatus, RAY_CAST_METHOD=1.
// Copyright (c) 2020 UNR Autonomous Robots Lab. BSD-3-Clause.
#pragma once
#include "gbplanner_local/voxel_map.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace gbplanner2 {
struct GainRaycastDiagnostics {
  std::size_t queries{0},weighted_samples{0},reused_free_prefixes{0},reused_occupied_rays{0},step_increases{0};
};
struct GainRaycastOptions {
  bool nonuniform{true};
  double step_multiplier{1.};
  std::size_t max_samples{1000000};
  // Opt-in experiment; a measured paired speedup has not been established.
  bool reuse_invariants{false};
};

// Endpoints are ordered by increasing vertical angle, then horizontal angle,
// as SensorParamsBase::getFrustumEndpoints. This is an approximate gain query,
// not a replacement for MapQuery::ray, swept collision, or ground support.
// Each callback weight represents the repeated voxel_log entries in method 1.
template<class Visitor>
GainRaycastDiagnostics iterativeGainRaycast(const MapQuery& map,const Eigen::Vector3d& origin,
    const std::vector<Eigen::Vector3d>& endpoints,std::size_t width,std::size_t height,
    double horizontal_resolution,double vertical_resolution,GainRaycastOptions options,
    Visitor&& visit){
  if(!origin.allFinite()||!std::isfinite(map.resolution())||map.resolution()<=0.||
     !std::isfinite(options.step_multiplier)||options.step_multiplier<=0.||
     !std::isfinite(horizontal_resolution)||horizontal_resolution<=0.||
     !std::isfinite(vertical_resolution)||vertical_resolution<=0.||
     width>10000||height>10000||width*height!=endpoints.size()||endpoints.size()>10000)
    throw std::invalid_argument("Invalid iterative gain ray grid");
  if(options.max_samples==0)throw std::length_error("Gain ray sample budget exceeded");
  // Keep upstream float step arithmetic, including ceil(step/base) weights.
  const float voxel_size=static_cast<float>(map.resolution());
  const float base_step=voxel_size*options.step_multiplier;
  if(!std::isfinite(base_step)||base_step<=0.)
    throw std::invalid_argument("Invalid gain ray step");
  for(const auto& endpoint:endpoints)if(!endpoint.allFinite())
    throw std::invalid_argument("Nonfinite gain ray endpoint");
  struct Start {double distance{0.};bool occupied{false};};
  // A row-major cache replaces upstream's transposed Eigen indexing. Occupied
  // is separate from distance: an obstacle at step zero cannot be encoded -0.
  std::vector<Start> starts(endpoints.size());
  GainRaycastDiagnostics result;
  const auto emit=[&](const Eigen::Vector3d& point,VoxelStatus status,std::size_t weight){
    if(weight>options.max_samples-result.weighted_samples)
      throw std::length_error("Gain ray sample budget exceeded");
    result.weighted_samples+=weight;visit(point,status,weight);
  };
  for(std::size_t row=0;row<height;++row)for(std::size_t col=0;col<width;++col){
    const std::size_t id=row*width+col;
    const Eigen::Vector3d ray=endpoints[id]-origin;const double length=ray.norm();
    if(!std::isfinite(length))throw std::invalid_argument("Invalid gain ray length");
    const Eigen::Vector3d direction=length>0.?Eigen::Vector3d(ray/length):Eigen::Vector3d::Zero();
    const auto cached=starts[id];
    // Sensor offsets can give adjacent endpoints different ranges. A cached
    // hit beyond this ray cannot invent an occupied point outside its range.
    const double start=std::min(cached.distance,length);
    if(cached.occupied&&cached.distance<=length){
      ++result.reused_occupied_rays;
      emit(origin+start*direction,VoxelStatus::Occupied,1);
      continue; // Upstream does not propagate an inherited occupied shortcut.
    }
    if(start>0.)++result.reused_free_prefixes;
    float step_size=base_step;double previous_change=start;
    std::size_t cached_weight=1; // ceil(base_step/base_step), exactly one.
    for(double distance=start;distance<=length;){
      if(options.nonuniform&&std::abs(previous_change-distance)>4.&&distance<=12.){
        previous_change=distance;step_size+=base_step;++result.step_increases;
        // Keep the same float addition/division/ceil. The value changes only
        // here, not at every occupancy query with the same step_size.
        if(options.reuse_invariants)
          cached_weight=static_cast<std::size_t>(std::ceil(step_size/base_step));
      }
      const Eigen::Vector3d point=origin+distance*direction;
      if(result.weighted_samples>=options.max_samples)
        throw std::length_error("Gain ray sample budget exceeded");
      // The source indexes Voxblox with a float point, but logs the original
      // double coordinate for global/no-gain filtering. Preserve that order.
      const auto status=map.voxel(point.cast<float>().cast<double>());
      ++result.queries;
      const auto weight=options.reuse_invariants?cached_weight:
          static_cast<std::size_t>(std::ceil(step_size/base_step));
      emit(point,status,weight);
      if(status!=VoxelStatus::Unknown){
        const Start next{distance,status==VoxelStatus::Occupied};
        const double dx=distance*horizontal_resolution,dy=distance*vertical_resolution;
        // These are assignments, not maxima: preserve the source's ordered
        // reuse from the preceding horizontal/vertical/diagonal neighbor.
        if(dx<=voxel_size&&col+1<width)starts[id+1]=next;
        if(dy<=voxel_size&&row+1<height)starts[id+width]=next;
        if(std::sqrt(dx*dx+dy*dy)<=voxel_size&&col+1<width&&row+1<height)
          starts[id+width+1]=next;
      }
      if(status==VoxelStatus::Occupied)break;
      const double next_distance=distance+step_size;
      if(!std::isfinite(next_distance)||next_distance<=distance)
        throw std::length_error("Gain ray step cannot advance");
      distance=next_distance;
    }
  }
  return result;
}
}  // namespace gbplanner2
