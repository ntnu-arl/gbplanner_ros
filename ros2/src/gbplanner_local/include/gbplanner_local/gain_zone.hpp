// Geometry derived from GBPlanner 2 BoundedSpaceParams, BSD-3-Clause.
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>
#include <string>

namespace gbplanner2 {

// Excludes observation rewards only. Collision checks and ray occlusion use
// the original map independently of this region.
struct GainZone {
  std::string name;
  std::string type{"kCuboid"};
  Eigen::Vector3d minimum{-1.,-1.,-1.},maximum{1.,1.,1.};
  Eigen::Vector3d center{0.,0.,0.},rotations{0.,0.,0.};
  // The upstream no-gain sphere loader leaves its center/effective radius
  // uninitialized. Use an explicit center and positive radius here.
  double radius{1.};

  void validate() const {
    if(name.empty()||!center.allFinite()||!rotations.allFinite())
      throw std::invalid_argument("Invalid no-gain zone name, center or rotation");
    if(type=="kCuboid"){
      if(!minimum.allFinite()||!maximum.allFinite()||
         (minimum.array()>=maximum.array()).any())
        throw std::invalid_argument("Invalid no-gain cuboid bounds");
    }else if(type=="kSphere"){
      if(!std::isfinite(radius)||radius<=0.)
        throw std::invalid_argument("Invalid no-gain sphere radius");
    }else{
      throw std::invalid_argument("Unsupported no-gain zone type: "+type);
    }
  }

  // Validate once after configuring the region. Boundaries belong to the zone.
  bool contains(const Eigen::Vector3d& world) const {
    if(!world.allFinite())return false;
    if(type=="kSphere")return (world-center).norm()<=radius;
    if(type!="kCuboid")return false;
    // Original isInsideSpace uses R.transpose() * (world-center), with
    // rotations ordered yaw/pitch/roll. This differs from sampling clipping.
    const Eigen::Matrix3d rotation=(
        Eigen::AngleAxisd(rotations[0],Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(rotations[1],Eigen::Vector3d::UnitY())*
        Eigen::AngleAxisd(rotations[2],Eigen::Vector3d::UnitX())).toRotationMatrix();
    const Eigen::Vector3d local=rotation.transpose()*(world-center);
    return (local.array()>=minimum.array()).all()&&
           (local.array()<=maximum.array()).all();
  }
};

}  // namespace gbplanner2
