#pragma once
#include "gbplanner_local/ground_terrain.hpp"
#include <algorithm>
#include <cstddef>
#include <utility>
#include <vector>

namespace gbplanner2 {
enum class GroundEdgeRole { Parent,Neighbor };
enum class GroundEdgeDirection { Forward,Reverse };
inline const char* groundEdgeRoleName(GroundEdgeRole role){
  return role==GroundEdgeRole::Parent?"parent":"neighbor";
}
inline const char* groundEdgeDirectionName(GroundEdgeDirection direction){
  return direction==GroundEdgeDirection::Forward?"forward":"reverse";
}
struct GroundEdgeObservation {
  std::size_t check_id{0};
  GroundEdgeRole role{GroundEdgeRole::Parent};
  GroundEdgeDirection direction{GroundEdgeDirection::Forward};
  bool overshoot{false};
  // Logical endpoints retain their directed order, including on an extension.
  StateVec from{StateVec::Zero()},to{StateVec::Zero()};
  StateVec checked_from{StateVec::Zero()},checked_to{StateVec::Zero()};
  GroundStatus status{GroundStatus::Invalid};
  // admitted describes this candidateEdge result under the current policy;
  // a later overshoot rejection can still prevent insertion into the graph.
  bool speculative{false},admitted{false};
  double actual_length{0.},max_uphill{0.},max_downhill{0.},mean_inclination{0.};
  // Exactly the existing result, possibly an early-return prefix. Never fill
  // missing geometry by drawing the requested endpoint or querying the map.
  std::vector<StateVec> polyline;
  std::size_t total_points{0},dropped_points{0};
};
struct GroundEdgeDiagnostics {
  static constexpr std::size_t kMaxRecords=512,kMaxPoints=8192;
  bool enabled{false},partial{false},truncated{false};
  std::size_t total_checks{0},completed_checks{0};
  std::size_t total_records{0},dropped_records{0},total_points{0},dropped_points{0};
  std::vector<GroundEdgeObservation> records;
  std::size_t beginCheck(){
    if(!enabled)return 0;
    partial=true;return ++total_checks;
  }
  void finishCheck(){
    if(!enabled)return;
    ++completed_checks;partial=truncated||completed_checks!=total_checks;
  }
  void record(std::size_t check_id,GroundEdgeRole role,GroundEdgeDirection direction,bool overshoot,
      const StateVec& from,const StateVec& to,const StateVec& checked_from,const StateVec& checked_to,
      const GroundEdge& edge,bool admitted){
    if(!enabled)return;
    ++total_records;total_points+=edge.polyline.size();
    if(records.size()>=kMaxRecords){
      ++dropped_records;dropped_points+=edge.polyline.size();partial=true;truncated=true;return;
    }
    GroundEdgeObservation item;
    item.check_id=check_id;item.role=role;item.direction=direction;item.overshoot=overshoot;
    item.from=from;item.to=to;item.checked_from=checked_from;item.checked_to=checked_to;
    item.status=edge.status;item.speculative=edge.speculative;item.admitted=admitted;
    item.actual_length=edge.actual_length;item.max_uphill=edge.max_uphill;
    item.max_downhill=edge.max_downhill;item.mean_inclination=edge.mean_inclination;
    item.total_points=edge.polyline.size();
    const auto stored_before=total_points-edge.polyline.size()-dropped_points;
    const auto count=std::min(edge.polyline.size(),kMaxPoints-stored_before);
    item.polyline.assign(edge.polyline.begin(),edge.polyline.begin()+count);
    item.dropped_points=edge.polyline.size()-count;dropped_points+=item.dropped_points;
    if(item.dropped_points){partial=true;truncated=true;}
    records.push_back(std::move(item));
  }
};
}  // namespace gbplanner2
