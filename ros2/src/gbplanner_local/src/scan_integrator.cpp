#include "gbplanner_local/scan_integrator.hpp"
#include <set>
#include <tuple>
#include <stdexcept>

namespace gbplanner2 {
ScanIntegrator::ScanIntegrator(double min_range,double max_range):min_range_(min_range),max_range_(max_range) {
  if(!std::isfinite(min_range)||!std::isfinite(max_range)||min_range<0||max_range<=min_range||max_range>100)
    throw std::invalid_argument("Invalid point-cloud integration range");
}
IntegrationResult ScanIntegrator::integrate(VoxelMap& map,const Eigen::Vector3d& origin,
                                           const std::vector<Eigen::Vector3d>& endpoints) const {
  if(!origin.allFinite()||!map.inside(map.index(origin)))
    throw std::invalid_argument("Sensor origin outside map");
  if(endpoints.size()>1000000||max_range_/map.resolution()>30000)
    throw std::invalid_argument("Scan integration budget exceeded");
  using Key=std::tuple<int,int,int>;
  std::set<Key> free,occupied;
  IntegrationResult result;
  for(const auto& endpoint:endpoints){
    if(!endpoint.allFinite()){++result.skipped_points;continue;}
    Eigen::Vector3d delta=endpoint-origin;double distance=delta.norm();
    if(!std::isfinite(distance)||distance<=min_range_){++result.skipped_points;continue;}
    ++result.accepted_points;
    const bool within_range=distance<=max_range_;
    Eigen::Vector3d end=within_range?endpoint:Eigen::Vector3d(origin+delta*(max_range_/distance));
    bool hit=false;Eigen::Vector3i hit_index=Eigen::Vector3i::Zero();
    if(within_range && (end.array()>=map.minimum().array()).all() && (end.array()<map.maximum().array()).all()){
      hit_index=map.index(end);hit=true;
    }
    map.ray(origin,end,[&](const Eigen::Vector3d& p,VoxelStatus){
      auto index=map.index(p);if(!map.inside(index))return false;
      Key key(index.x(),index.y(),index.z());
      if(hit && index==hit_index)occupied.insert(key);else free.insert(key);
      return true;
    });
  }
  // A hit wins if another ray traverses the same cell in this scan. The full
  // scan is classified before mutation, so input order does not change results.
  for(const auto& key:free)if(!occupied.count(key)){
    map.set({std::get<0>(key),std::get<1>(key),std::get<2>(key)},VoxelStatus::Free);++result.free_updates;
  }
  for(const auto& key:occupied){
    map.set({std::get<0>(key),std::get<1>(key),std::get<2>(key)},VoxelStatus::Occupied);++result.occupied_updates;
  }
  return result;
}
}
