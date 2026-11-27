// Adapted from GBPlanner 2 free-space pointcloud and sensor transforms.
// Copyright (c) 2020 UNR Autonomous Robots Lab, BSD-3-Clause.
#include "gbplanner_local/free_space.hpp"
#include <Eigen/Geometry>
#include <cmath>
#include <set>
#include <stdexcept>

namespace gbplanner2 {
std::vector<FreeSpaceCloud> buildFreeSpaceClouds(
    const MapQuery& map,const StateVec& base_pose,const std::vector<FreeSpaceSensor>& sensors){
  if(!base_pose.allFinite())throw std::invalid_argument("Invalid free-space body pose");
  if(sensors.size()>32)throw std::invalid_argument("Too many free-space sensors");
  std::set<std::string> names;double ray_budget=0.;
  for(const auto& configured:sensors){
    const auto& sensor=configured.sensor;sensor.validate();
    if(configured.frame.empty())throw std::invalid_argument("Empty free-space sensor frame");
    if(!names.insert(sensor.name).second)throw std::invalid_argument("Duplicate free-space sensor name");
    ray_budget+=std::ceil(sensor.horizontal_fov/sensor.horizontal_resolution)*
        std::ceil(sensor.vertical_fov/sensor.vertical_resolution);
  }
  if(ray_budget>10000)throw std::invalid_argument("Aggregate free-space ray budget exceeded");

  const Eigen::Vector3d origin=base_pose.head<3>();
  const Eigen::Matrix3d body_to_world=Eigen::AngleAxisd(base_pose[3],Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Matrix3d world_to_body=body_to_world.inverse();
  const Eigen::Matrix3d sensor_to_optical=(Eigen::AngleAxisd(M_PI/2.,Eigen::Vector3d::UnitZ())*
      Eigen::AngleAxisd(-M_PI/2.,Eigen::Vector3d::UnitY())).toRotationMatrix();
  std::vector<FreeSpaceCloud> clouds;clouds.reserve(sensors.size());
  for(const auto& configured:sensors){
    const auto& sensor=configured.sensor;
    const Eigen::Matrix3d sensor_to_body=(Eigen::AngleAxisd(sensor.rotations[0],Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(sensor.rotations[1],Eigen::Vector3d::UnitY())*
        Eigen::AngleAxisd(sensor.rotations[2],Eigen::Vector3d::UnitX())).toRotationMatrix();
    const Eigen::Matrix3d body_to_sensor=sensor_to_body.inverse();
    // Allocate per sensor. The original shared body cloud accidentally fed
    // all earlier sensors' rays through each later sensor's extrinsics.
    FreeSpaceCloud cloud;cloud.name=sensor.name;cloud.frame=configured.frame;
    cloud.points.reserve(static_cast<std::size_t>(std::ceil(sensor.horizontal_fov/sensor.horizontal_resolution)*
        std::ceil(sensor.vertical_fov/sensor.vertical_resolution)));
    for(double vertical=-sensor.vertical_fov/2.;vertical<sensor.vertical_fov/2.-1e-10;
        vertical+=sensor.vertical_resolution){
      for(double horizontal=-sensor.horizontal_fov/2.;horizontal<sensor.horizontal_fov/2.-1e-10;
          horizontal+=sensor.horizontal_resolution){
        // Keep SensorParamsBase's original angular grid: z=range*sin(v),
        // without a vertical cosine in x/y. min_range is unused in this branch.
        const Eigen::Vector3d angular(sensor.range*std::cos(horizontal),
            sensor.range*std::sin(horizontal),sensor.range*std::sin(vertical));
        const Eigen::Vector3d endpoint=origin+body_to_world*(sensor_to_body*angular+sensor.center_offset);
        if(!endpoint.allFinite())throw std::invalid_argument("Nonfinite free-space ray endpoint");
        ++cloud.tested_rays;bool blocked=false;
        // Traverse only the segment to the endpoint. Upstream multiplied an
        // unnormalized direction by metric steps and could scan beyond it.
        // Unknown remains transparent, while any occupied cell rejects the
        // entire ray; planning policies must not invent physical surfaces.
        map.ray(origin,endpoint,[&](const auto&,VoxelStatus status){
          if(status==VoxelStatus::Occupied){blocked=true;return false;}
          return true;
        });
        if(blocked){++cloud.blocked_rays;continue;}
        const Eigen::Vector3d body_endpoint=world_to_body*(endpoint-origin);
        // convertBodyToSensor subtracts the body-frame translation before
        // inverse rotation. This intentionally differs from isInsideFOV.
        Eigen::Vector3d sensor_endpoint=body_to_sensor*(body_endpoint-sensor.center_offset);
        if(sensor.type=="kCamera")sensor_endpoint=sensor_to_optical*sensor_endpoint;
        cloud.points.push_back(sensor_endpoint);
      }
    }
    clouds.push_back(std::move(cloud));
  }
  return clouds;
}
}  // namespace gbplanner2
