#pragma once
#include "gbplanner_local/local_planner.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <iomanip>
#include <sstream>

namespace gbplanner2 {
// Display geometry comes from the planner's actual report and cached ground
// polylines. Never reconstruct a shortest path from the undirected wire graph.
inline geometry_msgs::msg::Point markerPoint(const Eigen::Vector3d& value){
  geometry_msgs::msg::Point point;point.x=value.x();point.y=value.y();point.z=value.z();return point;
}
inline visualization_msgs::msg::MarkerArray markerSnapshot(const std_msgs::msg::Header& header){
  visualization_msgs::msg::Marker clear;clear.header=header;
  clear.action=visualization_msgs::msg::Marker::DELETEALL;
  visualization_msgs::msg::MarkerArray snapshot;snapshot.markers.push_back(clear);return snapshot;
}
inline visualization_msgs::msg::Marker planningMarker(const std_msgs::msg::Header& header,
    const std::string& name,int type,double scale,double red,double green,double blue){
  visualization_msgs::msg::Marker marker;marker.header=header;marker.ns=name;marker.id=0;
  marker.action=visualization_msgs::msg::Marker::ADD;marker.type=type;marker.pose.orientation.w=1.;
  marker.scale.x=scale;marker.scale.y=scale;marker.scale.z=scale;
  marker.color.r=red;marker.color.g=green;marker.color.b=blue;marker.color.a=1.;return marker;
}
inline visualization_msgs::msg::MarkerArray shortestPathMarkers(
    const PlanningDiagnostics& diagnostics,const std_msgs::msg::Header& header){
  using Marker=visualization_msgs::msg::Marker;
  auto snapshot=markerSnapshot(header);
  if(diagnostics.shortest_vertices.empty())return snapshot;
  auto edges=planningMarker(header,"shortest_edges",Marker::LINE_LIST,.06,0.,0.,.9);
  auto vertices=planningMarker(header,"vertices",Marker::SPHERE_LIST,.12,.78,.39,0.);
  auto leaves=planningMarker(header,"leaves",Marker::SPHERE_LIST,.18,.95,.75,.05);
  std::map<int,const PlanningPathVertex*> states;
  for(const auto& vertex:diagnostics.shortest_vertices)states.emplace(vertex.id,&vertex);
  for(const auto& [id,vertex]:states){
    const auto point=markerPoint(vertex->state.head<3>());vertices.points.push_back(point);
    if(vertex->leaf)leaves.points.push_back(point);
    const auto parent=states.find(vertex->parent);
    if(id!=vertex->parent&&parent!=states.end()){
      edges.points.push_back(point);edges.points.push_back(markerPoint(parent->second->state.head<3>()));
    }
  }
  if(!edges.points.empty())snapshot.markers.push_back(std::move(edges));
  snapshot.markers.push_back(std::move(vertices));
  if(!leaves.points.empty())snapshot.markers.push_back(std::move(leaves));
  return snapshot;
}
inline visualization_msgs::msg::MarkerArray bestPathMarkers(
    const PlanningDiagnostics& diagnostics,const std_msgs::msg::Header& header){
  using Marker=visualization_msgs::msg::Marker;
  auto snapshot=markerSnapshot(header);
  if(diagnostics.best_id<0||diagnostics.best_path.empty())return snapshot;
  auto path=planningMarker(header,"best_path",Marker::LINE_STRIP,.12,.043,.447,.106);
  for(const auto& state:diagnostics.best_path)path.points.push_back(markerPoint(state.head<3>()));
  if(path.points.size()>1)snapshot.markers.push_back(std::move(path));
  auto target=planningMarker(header,"best_target",Marker::TEXT_VIEW_FACING,.22,.043,.447,.106);
  target.pose.position=markerPoint(diagnostics.best_path.back().head<3>());target.pose.position.z+=.2;
  std::ostringstream label;label<<"id="<<diagnostics.best_id<<" gain="<<std::setprecision(8)<<diagnostics.best_gain;
  target.text=label.str();snapshot.markers.push_back(std::move(target));return snapshot;
}
inline visualization_msgs::msg::MarkerArray noGainZoneMarkers(
    const std::vector<GainZone>& zones,const std_msgs::msg::Header& header){
  using Marker=visualization_msgs::msg::Marker;
  auto snapshot=markerSnapshot(header);
  for(const auto& zone:zones){
    const bool sphere=zone.type=="kSphere";
    auto marker=planningMarker(header,"no_gain_zone_"+zone.name,sphere?Marker::SPHERE:Marker::CUBE,1.,1.,0.,0.);
    const Eigen::Matrix3d rotation=(Eigen::AngleAxisd(zone.rotations[0],Eigen::Vector3d::UnitZ())*
      Eigen::AngleAxisd(zone.rotations[1],Eigen::Vector3d::UnitY())*
      Eigen::AngleAxisd(zone.rotations[2],Eigen::Vector3d::UnitX())).toRotationMatrix();
    // GainZone::contains transforms about zone.center. The midpoint of an
    // asymmetric box must rotate too; a sphere uses its explicit center.
    Eigen::Vector3d center=zone.center,size=Eigen::Vector3d::Constant(2.*zone.radius);
    if(!sphere){center+=rotation*(.5*(zone.minimum+zone.maximum));size=zone.maximum-zone.minimum;}
    marker.pose.position=markerPoint(center);const Eigen::Quaterniond q(sphere?Eigen::Matrix3d::Identity():rotation);
    marker.pose.orientation.x=q.x();marker.pose.orientation.y=q.y();marker.pose.orientation.z=q.z();marker.pose.orientation.w=q.w();
    marker.scale.x=size.x();marker.scale.y=size.y();marker.scale.z=size.z();marker.color.a=.2;
    snapshot.markers.push_back(std::move(marker));
  }
  return snapshot;
}
}  // namespace gbplanner2
