#pragma once
#include "gbplanner_local/gain_diagnostics.hpp"
#include "gbplanner_local/planning_markers.hpp"
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace gbplanner2 {
inline visualization_msgs::msg::MarkerArray gainRayMarkers(
    const GainGeometry& geometry,const std_msgs::msg::Header& header){
  using Marker=visualization_msgs::msg::Marker;
  auto snapshot=markerSnapshot(header);
  if(!geometry.evaluated||!geometry.state_valid)return snapshot;
  for(std::size_t index=0;index<geometry.sensors.size();++index){
    const auto& sensor=geometry.sensors[index];
    auto lines=planningMarker(header,"gain_rays",Marker::LINE_LIST,.015,.2,.55,1.);
    lines.id=static_cast<int>(index);
    // Ray mode shows full sensor endpoints, not obstacle-truncated hit rays.
    // Voxel mode shows the actual endpoints of visibility queries instead.
    for(const auto& endpoint:sensor.endpoints)if(endpoint.allFinite()){
      lines.points.push_back(markerPoint(geometry.state.head<3>()));
      lines.points.push_back(markerPoint(endpoint));
    }
    if(!lines.points.empty())snapshot.markers.push_back(std::move(lines));
  }
  return snapshot;
}
inline visualization_msgs::msg::MarkerArray gainSampleMarkers(
    const GainGeometry& geometry,const std_msgs::msg::Header& header){
  using Marker=visualization_msgs::msg::Marker;
  auto snapshot=markerSnapshot(header);
  if(!geometry.evaluated||!geometry.state_valid)return snapshot;
  constexpr std::array<VoxelStatus,3> statuses{VoxelStatus::Unknown,VoxelStatus::Free,VoxelStatus::Occupied};
  constexpr std::array<const char*,3> names{"gain_unknown","gain_free","gain_occupied"};
  constexpr std::array<std::array<double,3>,3> colors{{{.2,.5,1.},{.1,.8,.35},{1.,.15,.1}}};
  for(std::size_t index=0;index<geometry.sensors.size();++index){
    const auto& sensor=geometry.sensors[index];
    for(std::size_t kind=0;kind<statuses.size();++kind){
      auto samples=planningMarker(header,names[kind],Marker::SPHERE_LIST,.06,
          colors[kind][0],colors[kind][1],colors[kind][2]);
      samples.id=static_cast<int>(index*3+kind);
      // One glyph per recorded contribution, preserving its double position.
      // A weight counts gain multiplicity; it is not expanded or deduplicated.
      for(const auto& item:sensor.contributions)
        if(item.status==statuses[kind]&&item.point.allFinite())samples.points.push_back(markerPoint(item.point));
      if(!samples.points.empty())snapshot.markers.push_back(std::move(samples));
    }
    if(sensor.roi_valid&&sensor.minimum.allFinite()&&sensor.maximum.allFinite()&&
       (sensor.minimum.array()<sensor.maximum.array()).all()){
      const Eigen::Vector3d size=sensor.maximum-sensor.minimum;
      auto roi=planningMarker(header,"gain_roi",Marker::CUBE,1.,.5,.5,.9);
      roi.id=static_cast<int>(index);roi.color.a=.1;
      roi.pose.position=markerPoint(.5*(sensor.minimum+sensor.maximum));
      roi.scale.x=size.x();roi.scale.y=size.y();roi.scale.z=size.z();snapshot.markers.push_back(std::move(roi));
    }
  }
  return snapshot;
}
inline std::string gainGeometryJson(const GainGeometry& geometry){
  std::ostringstream out;out<<std::setprecision(17)<<std::boolalpha;
  const auto number=[&](double value){if(std::isfinite(value))out<<value;else out<<"null";};
  const auto vector=[&](const auto& value){
    out<<'[';for(int i=0;i<value.size();++i){if(i)out<<',';number(value[i]);}out<<']';
  };
  const auto counts=[&](const VolumetricGain& gain){
    out<<"{\"unknown\":"<<gain.num_unknown_voxels<<",\"free\":"<<gain.num_free_voxels
      <<",\"occupied\":"<<gain.num_occupied_voxels<<'}';
  };
  const auto gain_value=[&](const VolumetricGain& gain){
    out<<"{\"gain\":";number(gain.gain);out<<",\"counts\":";counts(gain);
    out<<",\"is_frontier\":"<<gain.is_frontier<<'}';
  };
  out<<"{\"enabled\":"<<geometry.enabled<<",\"entered\":"<<geometry.entered
    <<",\"evaluated\":"<<geometry.evaluated<<",\"complete\":"<<geometry.complete
    <<",\"partial\":"<<geometry.partial<<",\"truncated\":"<<geometry.truncated
    <<",\"requested_id\":"<<geometry.requested_id<<",\"vertex_id\":"<<geometry.vertex_id
    <<",\"source_id\":"<<geometry.source_id<<",\"state\":";vector(geometry.state);
  out<<",\"state_valid\":"<<geometry.state_valid<<",\"mode\":"<<std::quoted(geometry.mode)
    <<",\"policy\":"<<std::quoted(geometry.policy)<<",\"reason\":"<<std::quoted(geometry.reason)
    <<",\"gain_valid\":"<<geometry.gain_valid<<",\"gain\":";gain_value(geometry.gain);
  out<<",\"total_endpoints\":"<<geometry.total_endpoints<<",\"dropped_endpoints\":"<<geometry.dropped_endpoints
    <<",\"total_samples\":"<<geometry.total_samples<<",\"dropped_samples\":"<<geometry.dropped_samples
    <<",\"sensors\":[";
  for(std::size_t index=0;index<geometry.sensors.size();++index){
    if(index)out<<',';
    const auto& sensor=geometry.sensors[index];const auto& result=sensor.result;
    out<<"{\"result\":{\"name\":"<<std::quoted(result.name)<<",\"mode\":"<<std::quoted(result.mode)
      <<",\"gain\":";number(result.gain.gain);out<<",\"counts\":";counts(result.gain);
    out<<",\"is_frontier\":"<<result.gain.is_frontier<<",\"ray_count\":"<<result.ray_count
      <<",\"ray_method\":"<<std::quoted(result.ray_method)<<",\"ray_width\":"<<result.ray_width
      <<",\"ray_height\":"<<result.ray_height<<",\"voxel_count\":"<<result.voxel_count
      <<",\"fov_voxel_count\":"<<result.fov_voxel_count<<",\"visibility_queries\":"<<result.visibility_queries
      <<",\"raycast\":{\"queries\":"<<result.raycast.queries<<",\"weighted_samples\":"<<result.raycast.weighted_samples
      <<",\"reused_free_prefixes\":"<<result.raycast.reused_free_prefixes
      <<",\"reused_occupied_rays\":"<<result.raycast.reused_occupied_rays
      <<",\"step_increases\":"<<result.raycast.step_increases<<"}}"
      <<",\"minimum\":";vector(sensor.minimum);out<<",\"maximum\":";vector(sensor.maximum);
    out<<",\"roi_valid\":"<<sensor.roi_valid<<",\"step\":";number(sensor.step);
    out<<",\"map_resolution\":";number(sensor.map_resolution);
    out<<",\"map_resolution_valid\":"<<sensor.map_resolution_valid<<",\"complete\":"<<sensor.complete
      <<",\"raycast_complete\":"<<sensor.raycast_complete<<",\"total_endpoints\":"<<sensor.total_endpoints
      <<",\"dropped_endpoints\":"<<sensor.dropped_endpoints<<",\"total_samples\":"<<sensor.total_samples
      <<",\"dropped_samples\":"<<sensor.dropped_samples<<",\"endpoints\":[";
    for(std::size_t i=0;i<sensor.endpoints.size();++i){if(i)out<<',';vector(sensor.endpoints[i]);}
    out<<"],\"contributions\":[";
    for(std::size_t i=0;i<sensor.contributions.size();++i){
      if(i)out<<',';const auto& item=sensor.contributions[i];
      out<<"{\"point\":";vector(item.point);out<<",\"status\":"<<int(item.status)<<",\"weight\":"<<item.weight<<'}';
    }
    out<<"]}";
  }
  out<<"]}";return out.str();
}
}  // namespace gbplanner2
