#pragma once
#include "gbplanner_local/path_refinement.hpp"
#include "gbplanner_local/planning_markers.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <iomanip>
#include <sstream>

namespace gbplanner2 {
// Upstream visualizeModPath: the complete accepted refinement, not the final
// interpolated service path or a concatenation of partially accepted attempts.
inline visualization_msgs::msg::MarkerArray refinedPathMarkers(
    const PathRefinementDiagnostics& diagnostic,const std_msgs::msg::Header& header,
    bool response_success){
  using Marker=visualization_msgs::msg::Marker;
  auto snapshot=markerSnapshot(header);
  if(!response_success||!diagnostic.geometry.path_accepted||diagnostic.accepted_path.size()<2)
    return snapshot;
  auto edges=planningMarker(header,"mod_path",Marker::LINE_LIST,.25,244./255.,217./255.,66./255.);
  edges.scale.y=0.;edges.scale.z=0.;
  for(std::size_t i=1;i<diagnostic.accepted_path.size();++i){
    edges.points.push_back(markerPoint(diagnostic.accepted_path[i-1].head<3>()));
    edges.points.push_back(markerPoint(diagnostic.accepted_path[i].head<3>()));
  }
  snapshot.markers.push_back(std::move(edges));return snapshot;
}
inline std::string refinedPathJson(const PathRefinementDiagnostics& diagnostic){
  std::ostringstream out;out<<std::setprecision(17)<<'[';
  for(std::size_t i=0;i<diagnostic.accepted_path.size();++i){
    if(i)out<<',';out<<'[';
    for(int axis=0;axis<4;++axis){
      if(axis)out<<',';const double value=diagnostic.accepted_path[i][axis];
      if(std::isfinite(value))out<<value;else out<<"null";
    }
    out<<']';
  }
  out<<']';return out.str();
}
// A sampled endpoint section is evidence from a modification attempt, not a
// certified free volume. Plane patches are deliberately finite display glyphs.
inline visualization_msgs::msg::MarkerArray refinementMarkers(
    const RefinementGeometry& geometry,const std_msgs::msg::Header& header,
    bool response_success){
  using Marker=visualization_msgs::msg::Marker;
  auto snapshot=markerSnapshot(header);
  auto tangents=planningMarker(header,"refinement_tangents",Marker::CUBE_LIST,.06,.2,.4,1.);
  int plane_id=0;
  for(std::size_t index=0;index<geometry.attempts.size();++index){
    const auto& attempt=geometry.attempts[index];
    const bool accepted=response_success&&geometry.path_accepted&&attempt.locally_accepted;
    const double red=accepted?.1:1.,green=accepted?.8:.55,blue=accepted?.65:.1;
    for(const auto& plane:attempt.planes){
      const double norm=plane.normal.stableNorm();
      if(!plane.normal.allFinite()||!plane.tangent.allFinite()||!std::isfinite(norm)||norm<=0.)continue;
      const Eigen::Vector3d normal=plane.normal/norm;
      const Eigen::Quaterniond rotation=Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(),normal);
      auto patch=planningMarker(header,"refinement_planes",Marker::CUBE,.02,red,green,blue);
      patch.id=plane_id;patch.scale.y=.75;patch.scale.z=.75;patch.color.a=.18;
      patch.pose.position=markerPoint(plane.tangent);
      patch.pose.orientation.x=rotation.x();patch.pose.orientation.y=rotation.y();
      patch.pose.orientation.z=rotation.z();patch.pose.orientation.w=rotation.w();
      snapshot.markers.push_back(std::move(patch));
      auto arrow=planningMarker(header,"refinement_normals",Marker::ARROW,.025,red,green,blue);
      arrow.id=plane_id++;arrow.scale.y=.055;arrow.scale.z=.08;
      arrow.points={markerPoint(plane.tangent),markerPoint(plane.tangent+.35*normal)};
      snapshot.markers.push_back(std::move(arrow));
      tangents.points.push_back(markerPoint(plane.tangent));
    }
    if(attempt.centroid_valid){
      auto centroid=planningMarker(header,"refinement_centroids",Marker::SPHERE,.10,red,green,blue);
      centroid.id=static_cast<int>(index);centroid.pose.position=markerPoint(attempt.centroid);
      snapshot.markers.push_back(std::move(centroid));
    }
  }
  if(!tangents.points.empty())snapshot.markers.push_back(std::move(tangents));
  return snapshot;
}
inline sensor_msgs::msg::PointCloud2 refinementCloud(
    const RefinementGeometry& geometry,const std_msgs::msg::Header& header){
  sensor_msgs::msg::PointCloud2 cloud;cloud.header=header;cloud.height=1;cloud.is_dense=true;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(3,"x",1,sensor_msgs::msg::PointField::FLOAT32,
      "y",1,sensor_msgs::msg::PointField::FLOAT32,"z",1,sensor_msgs::msg::PointField::FLOAT32);
  std::size_t count=0;for(const auto& attempt:geometry.attempts)count+=attempt.free_points.size();
  modifier.resize(count);
  if(count){
    sensor_msgs::PointCloud2Iterator<float> x(cloud,"x"),y(cloud,"y"),z(cloud,"z");
    for(const auto& attempt:geometry.attempts)for(const auto& point:attempt.free_points){
      *x=point.x();*y=point.y();*z=point.z();
      cloud.is_dense=cloud.is_dense&&std::isfinite(*x)&&std::isfinite(*y)&&std::isfinite(*z);
      ++x;++y;++z;
    }
  }
  return cloud;
}
inline std::string refinementGeometryJson(const RefinementGeometry& geometry){
  std::ostringstream out;out<<std::setprecision(17)<<std::boolalpha;
  const auto number=[&](double value){if(std::isfinite(value))out<<value;else out<<"null";};
  const auto vector=[&](const Eigen::Vector3d& value){
    out<<'[';number(value.x());out<<',';number(value.y());out<<',';number(value.z());out<<']';
  };
  out<<"{\"captured\":"<<geometry.captured<<",\"path_accepted\":"<<geometry.path_accepted
    <<",\"truncated\":"<<geometry.truncated<<",\"attempt_count\":"<<geometry.attempt_count
    <<",\"dropped_attempts\":"<<geometry.dropped_attempts<<",\"plane_count\":"<<geometry.plane_count
    <<",\"dropped_planes\":"<<geometry.dropped_planes<<",\"free_point_count\":"<<geometry.free_point_count
    <<",\"dropped_free_points\":"<<geometry.dropped_free_points<<",\"attempts\":[";
  for(std::size_t i=0;i<geometry.attempts.size();++i){
    if(i)out<<',';
    const auto& a=geometry.attempts[i];
    out<<"{\"segment_index\":"<<a.segment_index<<",\"start\":";vector(a.start);
    out<<",\"target\":";vector(a.target);out<<",\"center\":";vector(a.center);
    out<<",\"rotation\":[";
    for(int r=0;r<3;++r)for(int c=0;c<3;++c){if(r||c)out<<',';number(a.rotation(r,c));}
    out<<"],\"adjusted\":";vector(a.adjusted);out<<",\"centroid\":";vector(a.centroid);
    out<<",\"centroid_valid\":"<<a.centroid_valid<<",\"modify_attempted\":"<<a.modify_attempted
      <<",\"modify_succeeded\":"<<a.modify_succeeded<<",\"locally_accepted\":"<<a.locally_accepted
      <<",\"reason\":"<<std::quoted(a.reason)<<",\"planes\":[";
    for(std::size_t j=0;j<a.planes.size();++j){
      if(j)out<<',';const auto& plane=a.planes[j];
      out<<"{\"normal\":";vector(plane.normal);out<<",\"offset\":";number(plane.offset);out<<",\"tangent\":";
      vector(plane.tangent);out<<",\"obstacle\":"<<plane.obstacle<<'}';
    }
    out<<"],\"free_points\":[";
    for(std::size_t j=0;j<a.free_points.size();++j){if(j)out<<',';vector(a.free_points[j]);}
    out<<"]}";
  }
  out<<"]}";return out.str();
}
}  // namespace gbplanner2
