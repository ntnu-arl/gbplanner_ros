#pragma once
#include "gbplanner_local/global_memory.hpp"
#include "gbplanner_local/planning_markers.hpp"

namespace gbplanner2 {
inline visualization_msgs::msg::MarkerArray homingPathMarkers(
    const HomingPathDiagnostics& diagnostic,const std_msgs::msg::Header& header){
  using Marker=visualization_msgs::msg::Marker;
  auto snapshot=markerSnapshot(header);
  if(!diagnostic.captured||diagnostic.vertices.size()<2)return snapshot;
  // Upstream visualizeHomingPath: green, 0.4 m, best_path namespace.
  auto marker=planningMarker(header,"best_path",Marker::LINE_LIST,.4,0.,1.,0.);
  for(std::size_t i=1;i<diagnostic.vertices.size();++i){
    marker.points.push_back(markerPoint(diagnostic.vertices[i-1].head<3>()));
    marker.points.push_back(markerPoint(diagnostic.vertices[i].head<3>()));
  }
  snapshot.markers.push_back(std::move(marker));return snapshot;
}
inline std::string homingPathDiagnosticsJson(const HomingPathDiagnostics& diagnostic){
  std::ostringstream out;out<<std::setprecision(17)<<std::boolalpha
    <<"{\"captured\":"<<diagnostic.captured
    <<",\"geometry\":\"raw_graph_vertices\",\"height_convention\":\"planning\",\"vertex_ids\":[";
  for(std::size_t i=0;i<diagnostic.vertex_ids.size();++i){if(i)out<<',';out<<diagnostic.vertex_ids[i];}
  out<<"],\"vertices\":[";
  for(std::size_t i=0;i<diagnostic.vertices.size();++i){
    if(i)out<<',';const auto& state=diagnostic.vertices[i];
    out<<'['<<state.x()<<','<<state.y()<<','<<state.z()<<','<<state[3]<<']';
  }
  out<<"]}";return out.str();
}
inline visualization_msgs::msg::MarkerArray globalPathMarkers(
    const GlobalPathDiagnostics& diagnostic,const std_msgs::msg::Header& header){
  using Marker=visualization_msgs::msg::Marker;
  auto snapshot=markerSnapshot(header);
  if(!diagnostic.captured)return snapshot;
  const auto append=[&](const std::vector<StateVec>& path,const char* name,double red,double green){
    if(path.size()<2)return;
    auto marker=planningMarker(header,name,Marker::LINE_LIST,.4,red,green,0.);
    for(std::size_t i=1;i<path.size();++i){
      marker.points.push_back(markerPoint(path[i-1].head<3>()));
      marker.points.push_back(markerPoint(path[i].head<3>()));
    }
    snapshot.markers.push_back(std::move(marker));
  };
  append(diagnostic.current_to_frontier,"current2frontier",0.,1.);
  if(diagnostic.home_reason=="ok")append(diagnostic.frontier_to_home,"frontier2home",.5,0.);
  return snapshot;
}
inline std::string globalPathDiagnosticsJson(const GlobalPathDiagnostics& diagnostic){
  std::ostringstream out;out<<std::setprecision(17)<<std::boolalpha
    <<"{\"captured\":"<<diagnostic.captured
    <<",\"geometry\":\"raw_graph_vertices\",\"height_convention\":\"planning\""
    <<",\"home_reason\":"<<std::quoted(diagnostic.home_reason)
    <<",\"home_reachable\":"<<(diagnostic.captured&&diagnostic.home_reason=="ok"&&
      !diagnostic.frontier_to_home.empty());
  const auto append=[&](const char* name,const std::vector<int>& ids,const std::vector<StateVec>& path){
    out<<",\""<<name<<"\":{\"vertex_ids\":[";
    for(std::size_t i=0;i<ids.size();++i){if(i)out<<',';out<<ids[i];}
    out<<"],\"vertices\":[";
    for(std::size_t i=0;i<path.size();++i){
      if(i)out<<',';const auto& state=path[i];
      out<<'['<<state.x()<<','<<state.y()<<','<<state.z()<<','<<state[3]<<']';
    }
    out<<"]}";
  };
  append("current_to_frontier",diagnostic.current_to_frontier_ids,diagnostic.current_to_frontier);
  append("frontier_to_home",diagnostic.frontier_to_home_ids,diagnostic.frontier_to_home);
  out<<'}';return out.str();
}
}  // namespace gbplanner2
