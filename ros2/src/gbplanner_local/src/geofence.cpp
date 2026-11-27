// Geometry adapted from planner_common/src/geofence_manager.cpp, GBPlanner 2.
// Copyright (c) 2020 UNR Autonomous Robots Lab. BSD-3-Clause.
#include "gbplanner_local/geofence.hpp"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/multi_point.hpp>
#include <cmath>
#include <stdexcept>

namespace gbplanner2 {
namespace bg=boost::geometry;
namespace {
void checkMotion(const Eigen::Vector3d& start,const Eigen::Vector3d& end,
                 const Eigen::Vector3d& size) {
  if(!start.allFinite()||!end.allFinite()||!size.allFinite()||
     (!(size.array()==0.).all()&&(size.array()<=0.).any()))
    throw std::invalid_argument("Invalid swept-box query");
}
}

bool GeofenceSet::add(const std::vector<Eigen::Vector2d>& points,std::string* error) {
  auto reject=[error](const std::string& reason){if(error)*error=reason;return false;};
  std::vector<Eigen::Vector2d> ring;
  ring.reserve(points.size());
  for(const auto& p:points){
    if(!p.allFinite())return reject("Geofence contains a nonfinite vertex");
    if(ring.empty()||(ring.back().array()!=p.array()).any())ring.push_back(p);
  }
  // Boost expects one closing vertex, while ROS polygon messages are usually
  // open. Normalize either form before correction/validation, without sorting.
  while(ring.size()>1&&(ring.front().array()==ring.back().array()).all())ring.pop_back();
  if(ring.size()<3)return reject("Geofence needs at least three distinct vertices");
  Polygon candidate;
  for(const auto& p:ring)bg::append(candidate.outer(),Point(p.x(),p.y()));
  bg::correct(candidate);
  std::string validity;
  if(!bg::is_valid(candidate,validity))return reject("Invalid geofence polygon: "+validity);
  const double area=bg::area(candidate);
  if(!std::isfinite(area)||area<=0)return reject("Geofence area must be finite and positive");
  polygons_.push_back(std::move(candidate));
  if(error)error->clear();
  return true;
}

bool GeofenceSet::violatesBox(const Eigen::Vector2d& center,const Eigen::Vector2d& size) const {
  return violatesPath(center,center,size);
}

bool GeofenceSet::violatesPath(const Eigen::Vector2d& start,const Eigen::Vector2d& end,
                              const Eigen::Vector2d& size) const {
  const bool point=(size.array()==0.).all();
  if(!start.allFinite()||!end.allFinite()||!size.allFinite()||
     (!point&&(size.array()<=0.).any()))
    throw std::invalid_argument("Invalid geofence swept-box query");
  if(point){
    // A degenerate polygon hull does not represent a point body. Query the
    // point/line directly; touching a forbidden polygon boundary is a hit.
    const Point from(start.x(),start.y()),to(end.x(),end.y());
    if((start.array()==end.array()).all()){
      for(const auto& polygon:polygons_)if(bg::covered_by(from,polygon))return true;
    }else{
      const bg::model::linestring<Point> segment{from,to};
      for(const auto& polygon:polygons_)if(bg::intersects(segment,polygon))return true;
    }
    return false;
  }
  // The convex hull of the two endpoint rectangles is exactly the Minkowski
  // sum of an axis-aligned robot footprint and this segment. This preserves
  // upstream's quadrant-based swept polygon, including boundary contact,
  // without normalizing a zero-length horizontal/vertical-only displacement.
  bg::model::multi_point<Point> corners;
  for(const auto& center:{start,end})for(double x:{-0.5,0.5})for(double y:{-0.5,0.5}){
    const Eigen::Vector2d corner=center+Eigen::Vector2d(x*size.x(),y*size.y());
    if(!corner.allFinite())throw std::invalid_argument("Geofence query extent overflow");
    corners.emplace_back(corner.x(),corner.y());
  }
  if(polygons_.empty())return false;
  Polygon swept;
  bg::convex_hull(corners,swept);
  bg::correct(swept);
  for(const auto& polygon:polygons_)if(bg::intersects(polygon,swept))return true;
  return false;
}

GeofenceMap::GeofenceMap(std::shared_ptr<const MapQuery> map,
                         std::shared_ptr<const GeofenceSet> fences)
 :map_(std::move(map)),fences_(std::move(fences)) {
  if(!map_||!fences_)throw std::invalid_argument("GeofenceMap requires map and fence snapshots");
}
VoxelStatus GeofenceMap::box(const Eigen::Vector3d& p,const Eigen::Vector3d& size) const {
  checkMotion(p,p,size);
  if(fences_->violatesBox(p.head<2>(),size.head<2>()))return VoxelStatus::Occupied;
  return map_->box(p,size);
}
VoxelStatus GeofenceMap::path(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
                             const Eigen::Vector3d& size) const {
  checkMotion(a,b,size);
  if(fences_->violatesPath(a.head<2>(),b.head<2>(),size.head<2>()))return VoxelStatus::Occupied;
  return map_->path(a,b,size);
}

}  // namespace gbplanner2
