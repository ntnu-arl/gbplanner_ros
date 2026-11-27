#pragma once

#include "gbplanner_local/voxel_map.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

namespace gbplanner2 {

// A convex workspace cuboid or sphere. Cuboid rotation columns are its local
// axes in world coordinates; robot sizes remain world-aligned, like MapQuery.
class GlobalBound {
 public:
  enum class Type {Cuboid,Sphere};

  static GlobalBound sphere(const Eigen::Vector3d& center,double radius) {
    if(!center.allFinite()||!std::isfinite(radius)||radius<=0.)
      throw std::invalid_argument("Invalid global sphere center or radius");
    // BoundedSpaceParams::loadParams(kSphere) left root_pos/radius_total
    // uninitialized upstream. This factory initializes the complete geometry
    // immediately, without depending on a later dynamic-bound callback.
    return GlobalBound(center,Eigen::Vector3d::Constant(radius),
                       Eigen::Matrix3d::Identity(),Type::Sphere);
  }

  static GlobalBound axisAligned(const Eigen::Vector3d& minimum,
                                 const Eigen::Vector3d& maximum) {
    if(!minimum.allFinite()||!maximum.allFinite()||
       (minimum.array()>=maximum.array()).any())
      throw std::invalid_argument("Invalid axis-aligned global bound");
    // Divide first so finite bounds spanning zero do not overflow in max-min.
    const Eigen::Vector3d half=maximum*.5-minimum*.5;
    const Eigen::Vector3d center=minimum*.5+maximum*.5;
    return GlobalBound(center,half,Eigen::Matrix3d::Identity());
  }

  // The service's 'center' field is the corner shared by three adjacent edges.
  static GlobalBound fromCorners(const Eigen::Vector3d& corner,
                                 const Eigen::Vector3d& front,
                                 const Eigen::Vector3d& left,
                                 const Eigen::Vector3d& up) {
    if(!corner.allFinite()||!front.allFinite()||!left.allFinite()||!up.allFinite())
      throw std::invalid_argument("Nonfinite global bound corners");
    const Eigen::Vector3d edge_x=front-corner,edge_y=left-corner,edge_z=up-corner;
    const Eigen::Vector3d lengths(edge_x.stableNorm(),edge_y.stableNorm(),edge_z.stableNorm());
    if(!lengths.allFinite()||(lengths.array()<=0.).any())
      throw std::invalid_argument("Degenerate global bound edges");
    const Eigen::Vector3d x=edge_x/lengths.x(),y=edge_y/lengths.y(),z=edge_z/lengths.z();
    constexpr double orthogonal_tolerance=1e-9;
    if(std::abs(x.dot(y))>orthogonal_tolerance||
       std::abs(x.dot(z))>orthogonal_tolerance||
       std::abs(y.dot(z))>orthogonal_tolerance||x.cross(y).dot(z)<=0.)
      throw std::invalid_argument("Global bound edges must form a right-handed orthogonal frame");
    // Remove only accepted numerical orthogonality error; avoid Euler-angle
    // reconstruction, whose roll formula loses information under tilt.
    Eigen::Matrix3d rotation;
    rotation.col(0)=x;
    rotation.col(2)=x.cross(y).normalized();
    rotation.col(1)=rotation.col(2).cross(x).normalized();
    const Eigen::Vector3d half=lengths*.5;
    return GlobalBound(corner+rotation*half,half,rotation);
  }

  const Eigen::Vector3d& center() const {return center_;}
  const Eigen::Vector3d& halfSize() const {return half_size_;}
  const Eigen::Matrix3d& rotation() const {return rotation_;}
  Type type() const {return type_;}
  const char* typeName() const {return type_==Type::Sphere?"kSphere":"kCuboid";}
  double radius() const {return type_==Type::Sphere?half_size_.x():0.;}
  Eigen::Vector3d minimum() const {return center_-rotation_.cwiseAbs()*half_size_;}
  Eigen::Vector3d maximum() const {return center_+rotation_.cwiseAbs()*half_size_;}

  bool contains(const Eigen::Vector3d& point) const {
    return containsBox(point,Eigen::Vector3d::Zero());
  }

  bool containsBox(const Eigen::Vector3d& center,const Eigen::Vector3d& size) const {
    if(!center.allFinite()||!size.allFinite()||(size.array()<0.).any())return false;
    const double scale=std::max({1.,center.cwiseAbs().maxCoeff(),
                                center_.cwiseAbs().maxCoeff(),half_size_.maxCoeff()});
    const double tolerance=32.*std::numeric_limits<double>::epsilon()*scale;
    if(type_==Type::Sphere){
      // The most distant of all eight world-aligned body corners has these
      // absolute coordinates relative to the sphere center. Checking only the
      // robot center or the sphere's AABB would admit a body across the shell.
      const Eigen::Vector3d farthest=(center-center_).cwiseAbs()+size*.5;
      if(!farthest.allFinite())return false;
      const double distance=farthest.stableNorm();
      return std::isfinite(distance)&&distance<=radius()+tolerance;
    }
    const Eigen::Vector3d offset=rotation_.transpose()*(center-center_);
    // Support of the complete world-aligned robot box in each bound axis.
    const Eigen::Vector3d projected_half=rotation_.transpose().cwiseAbs()*(size*.5);
    const Eigen::Vector3d extent=offset.cwiseAbs()+projected_half;
    if(!extent.allFinite())return false;
    // Inclusive faces with a tolerance limited to floating-point roundoff at
    // the coordinates being compared, rather than a physical clearance margin.
    return (extent.array()<=half_size_.array()+tolerance).all();
  }

 private:
  GlobalBound(Eigen::Vector3d center,Eigen::Vector3d half_size,Eigen::Matrix3d rotation,
              Type type=Type::Cuboid)
      :center_(std::move(center)),half_size_(std::move(half_size)),rotation_(std::move(rotation)),type_(type) {
    if(!center_.allFinite()||!half_size_.allFinite()||
       (half_size_.array()<=0.).any()||!minimum().allFinite()||!maximum().allFinite())
      throw std::invalid_argument("Global bound extent overflow or zero size");
  }
  Eigen::Vector3d center_,half_size_;
  Eigen::Matrix3d rotation_;
  Type type_{Type::Cuboid};
};

// Workspace policy restricts motion and reward, while retaining the observed
// map extent and every physical surface used for sensing and occlusion.
class BoundedMap final:public MapQuery {
 public:
  BoundedMap(std::shared_ptr<const MapQuery> base,GlobalBound bound)
      :base_(std::move(base)),bound_(std::move(bound)) {
    if(!base_)throw std::invalid_argument("Missing map for global bound");
  }
  double resolution() const override {return base_->resolution();}
  Eigen::Vector3d minimum() const override {return base_->minimum();}
  Eigen::Vector3d maximum() const override {return base_->maximum();}
  VoxelStatus voxel(const Eigen::Vector3d& point) const override {return base_->voxel(point);}
  std::optional<TsdfDistance> tsdfDistance(const Eigen::Vector3d& point) const override {
    return base_->tsdfDistance(point);
  }
  std::optional<TsdfDistance> tsdfDistanceSimplex(const Eigen::Vector3d& point) const override {
    return base_->tsdfDistanceSimplex(point);
  }
  std::optional<TsdfDistance> tsdfDistanceSimplexCached(
      const Eigen::Vector3d& point,TsdfQueryContext& context) const override {
    return base_->tsdfDistanceSimplexCached(point,context);
  }
  bool gainAllowed(const Eigen::Vector3d& point) const override {
    return base_->gainAllowed(point)&&bound_.contains(point);
  }
  VoxelStatus box(const Eigen::Vector3d& center,const Eigen::Vector3d& size) const override {
    if(!bound_.containsBox(center,size))return VoxelStatus::Occupied;
    return base_->box(center,size);
  }
  VoxelStatus path(const Eigen::Vector3d& start,const Eigen::Vector3d& end,
                   const Eigen::Vector3d& size) const override {
    // A fixed-size box translated along a segment lies inside either convex
    // workspace exactly when both endpoint boxes lie inside it.
    if(!bound_.containsBox(start,size)||!bound_.containsBox(end,size))
      return VoxelStatus::Occupied;
    return base_->path(start,end,size);
  }
  void ray(const Eigen::Vector3d& start,const Eigen::Vector3d& end,
           const std::function<bool(const Eigen::Vector3d&,VoxelStatus)>& visit) const override {
    base_->ray(start,end,visit);
  }
 private:
  std::shared_ptr<const MapQuery> base_;
  GlobalBound bound_;
};

}  // namespace gbplanner2
