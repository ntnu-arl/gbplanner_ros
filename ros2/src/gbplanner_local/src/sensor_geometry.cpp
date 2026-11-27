// Adapted from SensorParamsBase in GBPlanner 2 planner_common/src/params.cpp.
// Copyright (c) 2020 UNR Autonomous Robots Lab, BSD-3-Clause.
#include "gbplanner_local/sensor_geometry.hpp"
#include "gbplanner_local/parameters.hpp"
#include <Eigen/Geometry>
#include <array>
#include <cmath>

namespace gbplanner2 {
namespace {
Eigen::Matrix3d configuredRotation(const SensorParameters& sensor){
  return (Eigen::AngleAxisd(sensor.rotations[0],Eigen::Vector3d::UnitZ())*
          Eigen::AngleAxisd(sensor.rotations[1],Eigen::Vector3d::UnitY())*
          Eigen::AngleAxisd(sensor.rotations[2],Eigen::Vector3d::UnitX())).toRotationMatrix();
}
std::array<Eigen::Vector3d,4> cameraCorners(const SensorParameters& sensor){
  const double x=std::cos(sensor.horizontal_fov/2.),y=std::sin(sensor.horizontal_fov/2.),
               z=std::sin(sensor.vertical_fov/2.);
  // TL, TR, BR, BL. Preserve the source's unnormalized corners: z is sin(v/2),
  // not tan(v/2), and x does not contain a vertical-angle cosine factor.
  return {{{x,y,z},{x,-y,z},{x,-y,-z},{x,y,-z}}};
}
}

bool sensorContains(const SensorParameters& sensor,const StateVec& viewpoint,
                    const Eigen::Vector3d& point){
  if(!viewpoint.allFinite()||!point.allFinite()||
     (sensor.type!="kCamera"&&sensor.type!="kLidar"))return false;
  // Preserve source operation order, including inverse() rather than a changed
  // extrinsics convention shared with ray generation.
  const Eigen::Vector3d local=configuredRotation(sensor).inverse()*
      Eigen::AngleAxisd(-viewpoint[3],Eigen::Vector3d::UnitZ())*
      (point-viewpoint.head<3>())-sensor.center_offset;
  if(!local.allFinite())return false;
  const float norm=local.norm();
  if(norm>sensor.range)return false;
  // Original volume-FOV checks do not apply min_range.
  if(sensor.type=="kCamera"){
    const auto corners=cameraCorners(sensor);
    for(std::size_t i=0;i<corners.size();++i){
      const Eigen::Vector3d normal=corners[i].cross(corners[(i+1)%corners.size()]);
      if(local.dot(normal)<=0.)return false;
    }
    return true;
  }
  // At the lidar origin the source accepted the point through NaN angle
  // comparisons. Keep the accepted value without evaluating 0/0.
  if(norm==0.f)return true;
  const float horizontal=std::atan2(static_cast<float>(local.y()),static_cast<float>(local.x()));
  const float vertical=std::asin(static_cast<float>(local.z())/norm);
  return !(std::abs(horizontal)>sensor.horizontal_fov/2.||
           std::abs(vertical)>sensor.vertical_fov/2.);
}

std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d>> sensorFrustum(
    const SensorParameters& sensor,const StateVec& viewpoint){
  std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d>> lines;
  if(sensor.type!="kCamera"||!viewpoint.allFinite())return lines;
  const auto corners=cameraCorners(sensor);
  const Eigen::Matrix3d rotation=configuredRotation(sensor);
  const Eigen::AngleAxisd body(viewpoint[3],Eigen::Vector3d::UnitZ());
  const Eigen::Vector3d origin=viewpoint.head<3>();
  std::array<Eigen::Vector3d,4> world;
  lines.reserve(8);
  for(std::size_t i=0;i<world.size();++i){
    // Match edge_points_B=R*(range*corner) and getFrustumEdges exactly;
    // the original visual frustum does not add center_offset.
    world[i]=origin+body*(rotation*(sensor.range*corners[i]));
    lines.emplace_back(origin,world[i]);
  }
  for(std::size_t i=0;i<world.size();++i)lines.emplace_back(world[i],world[(i+1)%world.size()]);
  return lines;
}
}  // namespace gbplanner2
