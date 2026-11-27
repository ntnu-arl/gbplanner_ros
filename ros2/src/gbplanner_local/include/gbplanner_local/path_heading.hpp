// Adapted from GBPlanner 2 Rrg::correctYaw, improveFreePath, and
// comparePathWithDirectionApprioximately, BSD-3-Clause.
// Copyright (c) 2020 UNR Autonomous Robots Lab.
#pragma once
#include <Eigen/Core>
#include <cmath>
#include <vector>

namespace gbplanner2 {
// StateVec is Eigen::Vector4d (x, y, z, yaw). Keep this helper independent of
// graph and ROS message headers so every path producer uses the same rule.
inline void applySegmentHeading(const Eigen::Vector4d& start,
                                 Eigen::Vector4d& end,bool backward){
  double dx=end[0]-start[0],dy=end[1]-start[1];
  // A coincident or purely vertical segment has no horizontal tangent.
  // Preserve its heading rather than introducing an arbitrary turn to zero.
  if(std::hypot(dx,dy)<=1e-10)return;
  if(backward){dx=-dx;dy=-dy;}
  end[3]=std::atan2(dy,dx);
}

inline void applyPathHeading(std::vector<Eigen::Vector4d>& path,
                             bool tangent_yaw,bool backward){
  if(!tangent_yaw)return;
  // The robot's current yaw at the first pose is retained.
  for(std::size_t i=1;i<path.size();++i)
    applySegmentHeading(path[i-1],path[i],backward);
}

inline bool comparePathWithDirectionApproximately(
    const std::vector<Eigen::Vector4d>& path,double yaw){
  if(path.size()<=1)return true;
  constexpr double kPi=3.14159265358979323846;
  constexpr double kMinSegmentLength=2.;
  // Upstream uses the first point more than 2 m from the root, ignoring z.
  // Its zero-direction fallback for shorter paths is deliberately retained.
  double path_yaw=0.;
  for(std::size_t i=1;i<path.size();++i){
    const double dx=path[i][0]-path.front()[0];
    const double dy=path[i][1]-path.front()[1];
    if(std::hypot(dx,dy)>kMinSegmentLength){
      path_yaw=std::atan2(dy,dx);break;
    }
  }
  const double difference=std::remainder(path_yaw-yaw,2.*kPi);
  return std::abs(difference)<=.5*kPi;
}

inline void applyHomingPathHeading(std::vector<Eigen::Vector4d>& path,
                                   bool tangent_yaw,bool homing_backward){
  if(!tangent_yaw||path.empty())return;
  // Unlike planning_backward, homing_backward permits reverse travel only
  // when the route disagrees with the robot's initial heading by over 90 deg.
  const bool backward=homing_backward&&
      !comparePathWithDirectionApproximately(path,path.front()[3]);
  applyPathHeading(path,true,backward);
}
}  // namespace gbplanner2
