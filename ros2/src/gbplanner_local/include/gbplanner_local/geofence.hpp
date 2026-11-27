#pragma once

#include "gbplanner_local/voxel_map.hpp"
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <memory>
#include <string>
#include <vector>

namespace gbplanner2 {

// GBPlanner's forbidden areas are XY polygons extending through every height.
// Callers publish an immutable snapshot by copying this set before editing it.
class GeofenceSet {
 public:
  // Accept clockwise/counterclockwise vertices, with optional repeated closing
  // points. A failed addition leaves this set unchanged and explains why.
  bool add(const std::vector<Eigen::Vector2d>& points, std::string* error=nullptr);
  void clear() { polygons_.clear(); }
  std::size_t size() const { return polygons_.size(); }
  bool violatesBox(const Eigen::Vector2d& center,const Eigen::Vector2d& size) const;
  bool violatesPath(const Eigen::Vector2d& start,const Eigen::Vector2d& end,
                    const Eigen::Vector2d& size) const;

 private:
  using Point=boost::geometry::model::d2::point_xy<double>;
  using Polygon=boost::geometry::model::polygon<Point>;
  std::vector<Polygon> polygons_;
};

// A policy constraint is not a physical surface: voxel/ray queries and gain
// retain the original observations; only robot motion checks include fences.
class GeofenceMap final:public MapQuery {
 public:
  GeofenceMap(std::shared_ptr<const MapQuery> map,std::shared_ptr<const GeofenceSet> fences);
  double resolution() const override {return map_->resolution();}
  Eigen::Vector3d minimum() const override {return map_->minimum();}
  Eigen::Vector3d maximum() const override {return map_->maximum();}
  VoxelStatus voxel(const Eigen::Vector3d& p) const override {return map_->voxel(p);}
  std::optional<TsdfDistance> tsdfDistance(const Eigen::Vector3d& p) const override {
    return map_->tsdfDistance(p);
  }
  std::optional<TsdfDistance> tsdfDistanceSimplex(const Eigen::Vector3d& p) const override {
    return map_->tsdfDistanceSimplex(p);
  }
  std::optional<TsdfDistance> tsdfDistanceSimplexCached(
      const Eigen::Vector3d& p,TsdfQueryContext& context) const override {
    return map_->tsdfDistanceSimplexCached(p,context);
  }
  VoxelStatus box(const Eigen::Vector3d& p,const Eigen::Vector3d& size) const override;
  VoxelStatus path(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
                   const Eigen::Vector3d& size) const override;
  void ray(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
           const std::function<bool(const Eigen::Vector3d&,VoxelStatus)>& visit) const override {
    map_->ray(a,b,visit);
  }
 private:
  std::shared_ptr<const MapQuery> map_;
  std::shared_ptr<const GeofenceSet> fences_;
};

}  // namespace gbplanner2
