#pragma once
#include "gbplanner_local/global_memory.hpp"
#include "gbplanner_local/planning_markers.hpp"
#include <array>
#include <algorithm>
#include <cmath>

namespace gbplanner2 {
inline std::array<double,3> frontierClusterColor(std::size_t index,std::size_t count){
  // Same six-stop palette and index/count sampling as visualizeClusteredPaths.
  constexpr std::array<std::array<double,3>,6> colors{{
    {254,178,76},{253,141,60},{252,78,42},{227,26,28},{189,0,38},{128,0,38}}};
  const double value=count?std::clamp(double(index)/double(count),0.,1.)*5.:0.;
  const auto lower=static_cast<std::size_t>(std::floor(value));
  const auto upper=std::min(lower+1,colors.size()-1);
  std::array<double,3> result{};
  for(std::size_t axis=0;axis<3;++axis)
    result[axis]=(colors[lower][axis]+(value-lower)*(colors[upper][axis]-colors[lower][axis]))/255.;
  return result;
}
inline visualization_msgs::msg::MarkerArray frontierClusterMarkers(
    const FrontierClusteringDiagnostics& diagnostic,const std_msgs::msg::Header& header){
  using Marker=visualization_msgs::msg::Marker;
  auto snapshot=markerSnapshot(header);
  for(std::size_t index=0;index<diagnostic.representative_ids.size();++index){
    const int representative_id=diagnostic.representative_ids[index];
    const auto color=frontierClusterColor(index,diagnostic.representative_ids.size());
    auto members=planningMarker(header,"cluster"+std::to_string(index),Marker::LINE_LIST,
      .06,color[0],color[1],color[2]);
    const FrontierClusterPath* representative=nullptr;
    for(const auto& candidate:diagnostic.candidates){
      if(candidate.vertex_id==representative_id)representative=&candidate;
      // Refinement may leave old assignments when all representatives were
      // removed. Only the actual returned IDs define visible retained clusters.
      if(candidate.cluster_id!=representative_id)continue;
      for(std::size_t i=1;i<candidate.path.size();++i){
        members.points.push_back(markerPoint(candidate.path[i-1].head<3>()));
        members.points.push_back(markerPoint(candidate.path[i].head<3>()));
      }
    }
    if(!members.points.empty())snapshot.markers.push_back(std::move(members));
    if(representative&&representative->path.size()>1){
      auto path=planningMarker(header,"representative",Marker::LINE_STRIP,.12,color[0],color[1],color[2]);
      path.id=representative_id;
      for(const auto& state:representative->path)path.points.push_back(markerPoint(state.head<3>()));
      snapshot.markers.push_back(std::move(path));
      auto label=planningMarker(header,"representative_id",Marker::TEXT_VIEW_FACING,.2,color[0],color[1],color[2]);
      label.id=representative_id;label.pose.position=markerPoint(representative->path.back().head<3>());
      label.pose.position.z+=.25;label.text="cluster "+std::to_string(index)+" id="+std::to_string(representative_id);
      snapshot.markers.push_back(std::move(label));
    }
  }
  return snapshot;
}
}  // namespace gbplanner2
