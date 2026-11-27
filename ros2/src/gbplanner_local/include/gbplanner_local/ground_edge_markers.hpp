#pragma once
#include "gbplanner_local/ground_edge_diagnostics.hpp"
#include "gbplanner_local/planning_markers.hpp"
#include <cmath>
#include <iomanip>
#include <sstream>

namespace gbplanner2 {
inline std::string groundEdgeDiagnosticsJson(const GroundEdgeDiagnostics& diagnostics){
  std::ostringstream out;out<<std::setprecision(17)<<std::boolalpha;
  const auto number=[&](double value){if(std::isfinite(value))out<<value;else out<<"null";};
  const auto state=[&](const StateVec& value){
    out<<'[';for(int i=0;i<4;++i){if(i)out<<',';number(value[i]);}out<<']';
  };
  out<<"{\"scope\":\"ros2_directed_edge_admission\",\"enabled\":"<<diagnostics.enabled
    <<",\"partial\":"<<diagnostics.partial<<",\"truncated\":"<<diagnostics.truncated
    <<",\"max_records\":"<<GroundEdgeDiagnostics::kMaxRecords
    <<",\"max_points\":"<<GroundEdgeDiagnostics::kMaxPoints
    <<",\"total_checks\":"<<diagnostics.total_checks<<",\"completed_checks\":"<<diagnostics.completed_checks
    <<",\"total_records\":"<<diagnostics.total_records<<",\"dropped_records\":"<<diagnostics.dropped_records
    <<",\"total_points\":"<<diagnostics.total_points<<",\"dropped_points\":"<<diagnostics.dropped_points
    <<",\"records\":[";
  for(std::size_t index=0;index<diagnostics.records.size();++index){
    if(index)out<<',';
    const auto& item=diagnostics.records[index];
    out<<"{\"check_id\":"<<item.check_id<<",\"role\":"<<std::quoted(groundEdgeRoleName(item.role))
      <<",\"direction\":"<<std::quoted(groundEdgeDirectionName(item.direction))
      <<",\"overshoot\":"<<item.overshoot<<",\"from\":";state(item.from);
    out<<",\"to\":";state(item.to);out<<",\"checked_from\":";state(item.checked_from);
    out<<",\"checked_to\":";state(item.checked_to);
    out<<",\"status\":"<<std::quoted(groundStatusName(item.status))
      <<",\"speculative\":"<<item.speculative<<",\"admitted\":"<<item.admitted
      <<",\"actual_length\":";number(item.actual_length);
    out<<",\"max_uphill\":";number(item.max_uphill);out<<",\"max_downhill\":";number(item.max_downhill);
    out<<",\"mean_inclination\":";number(item.mean_inclination);
    out<<",\"total_points\":"<<item.total_points<<",\"dropped_points\":"<<item.dropped_points<<",\"polyline\":[";
    for(std::size_t point=0;point<item.polyline.size();++point){if(point)out<<',';state(item.polyline[point]);}
    out<<"]}";
  }
  out<<"]}";return out.str();
}
inline visualization_msgs::msg::MarkerArray groundEdgeMarkers(
    const GroundEdgeDiagnostics& diagnostics,const std_msgs::msg::Header& header){
  using Marker=visualization_msgs::msg::Marker;
  auto snapshot=markerSnapshot(header);
  if(!diagnostics.enabled)return snapshot;
  for(std::size_t index=0;index<diagnostics.records.size();++index){
    const auto& item=diagnostics.records[index];
    const double red=!item.admitted?1.:item.speculative?.95:.1;
    const double green=!item.admitted?.15:item.speculative?.65:.8;
    const double blue=!item.admitted?.1:.2;
    auto line=planningMarker(header,"ground_checked_polyline",Marker::LINE_STRIP,.035,red,green,blue);
    line.id=static_cast<int>(index);
    // Preserve recorded order and gaps. A prefix is never extended to the
    // requested endpoint, including when its last point was storage-capped.
    bool finite=true;
    for(const auto& point:item.polyline){
      if(!point.allFinite()){finite=false;break;}
      line.points.push_back(markerPoint(point.head<3>()));
    }
    if(finite&&line.points.size()>1)snapshot.markers.push_back(std::move(line));
    Eigen::Vector3d arrow_from=item.checked_from.head<3>(),arrow_to=item.checked_to.head<3>();
    bool projected_direction=false;
    if(finite)for(std::size_t i=item.polyline.size();i>1;--i){
      const Eigen::Vector3d a=item.polyline[i-2].head<3>(),b=item.polyline[i-1].head<3>();
      if((b-a).norm()>1.e-10){arrow_from=a;arrow_to=b;projected_direction=true;break;}
    }
    if(arrow_from.allFinite()&&arrow_to.allFinite()&&(arrow_to-arrow_from).norm()>1.e-10){
      auto arrow=planningMarker(header,projected_direction?"ground_polyline_direction":"ground_request_direction",
          Marker::ARROW,.025,red,green,blue);
      arrow.id=static_cast<int>(index);arrow.scale.y=.07;arrow.scale.z=.1;
      arrow.points={markerPoint(arrow_from),markerPoint(arrow_to)};snapshot.markers.push_back(std::move(arrow));
    }
    if(!arrow_to.allFinite())continue;
    auto label=planningMarker(header,"ground_check_status",Marker::TEXT_VIEW_FACING,.1,red,green,blue);
    label.id=static_cast<int>(index);label.pose.position=markerPoint(arrow_to);label.pose.position.z+=.13;
    std::ostringstream text;text<<item.check_id<<' '<<groundEdgeRoleName(item.role)<<' '
      <<groundEdgeDirectionName(item.direction)<<' '<<(item.overshoot?"overshoot":"base")<<' '
      <<(item.admitted?"admitted ":"rejected ")<<groundStatusName(item.status);
    if(!projected_direction)text<<" (request)";
    if(item.dropped_points)text<<" (capped)";
    label.text=text.str();snapshot.markers.push_back(std::move(label));
  }
  return snapshot;
}
}  // namespace gbplanner2
