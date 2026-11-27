#pragma once

#include "gbplanner_local/ground_terrain.hpp"
#include "gbplanner_local/robot_bounds.hpp"
#include <planner_msgs/srv/validate_path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <algorithm>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>

namespace gbplanner2 {

// Captured once after parameter configuration. Runtime planning changes never
// alter the geometry used by the independent validation executor.
struct PathValidationPolicy {
  std::string frame;
  bool ground_robot{false};
  double resolution{.25};
  RobotBounds robot_bounds;
  GroundParameters ground;
  Eigen::Vector3d center_offset{Eigen::Vector3d::Zero()};
  StateVec baseState(const StateVec& planning)const{
    StateVec result=planning;
    if(ground_robot)result.z()+=ground.robot_height-ground.max_ground_height;
    return result;
  }
  // All external poses describe base_link. Graphs and algorithm paths retain
  // the upstream planning height; convert once at the ROS boundary.
  bool poseState(const geometry_msgs::msg::Pose& pose,StateVec& state)const {
    Eigen::Quaterniond q(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z);
    state<<pose.position.x,pose.position.y,pose.position.z,0;
    if(!state.allFinite()||!q.coeffs().allFinite()||q.norm()<1e-8)return false;
    q.normalize();state[3]=std::atan2(2*(q.w()*q.z()+q.x()*q.y()),1-2*(q.y()*q.y()+q.z()*q.z()));
    if(ground_robot)state.z()+=ground.max_ground_height-ground.robot_height;
    return true;
  }
};
struct PathValidationMap {
  std::shared_ptr<const MapQuery> map;
  int revision{0};
  sensor_msgs::msg::PointCloud2::ConstSharedPtr source_cloud;
  std::shared_ptr<const std::string> diagnostic_metadata;
};
struct PathValidationObservation {
  StateVec state{StateVec::Zero()};
  bool valid{false};
};

// Writers build new maps and immutable fence/bound wrappers before publication.
// Neither a map query nor a terrain check holds this observation mutex. Map and
// revision are one atomic object, so readers cannot combine different updates.
class PathValidationInputs {
 public:
  using Snapshot=std::shared_ptr<const PathValidationMap>;
  PathValidationInputs():map_(std::make_shared<const PathValidationMap>()){}
  void publishMap(std::shared_ptr<const MapQuery> map,int revision,
                  sensor_msgs::msg::PointCloud2::ConstSharedPtr source_cloud={},
                  std::shared_ptr<const std::string> diagnostic_metadata={}){
    Snapshot next=std::make_shared<const PathValidationMap>(
      PathValidationMap{std::move(map),revision,std::move(source_cloud),std::move(diagnostic_metadata)});
    std::atomic_store(&map_,std::move(next));
  }
  Snapshot mapSnapshot()const{return std::atomic_load(&map_);}
  void publishObservation(const PathValidationObservation& observation){
    std::lock_guard<std::mutex> lock(observation_mutex_);observation_=observation;
  }
  PathValidationObservation observation()const{
    std::lock_guard<std::mutex> lock(observation_mutex_);return observation_;
  }
 private:
  Snapshot map_;
  mutable std::mutex observation_mutex_;
  PathValidationObservation observation_;
};

// The complete geometry context is immutable for one request. It has no node,
// graph, mission-budget, random-generator or live planning-state references.
class PathValidationContext {
 public:
  PathValidationContext(const PathValidationPolicy& policy,PathValidationInputs::Snapshot snapshot)
      :policy_(policy),snapshot_(std::move(snapshot)),map_(snapshot_->map){}
  void validate(const planner_msgs::srv::ValidatePath::Request& request,
                const PathValidationObservation& observation,
                planner_msgs::srv::ValidatePath::Response& response)const{
    response.valid=false;response.map_revision=snapshot_->revision;response.failure_segment=-1;
    if(request.header.frame_id!=policy_.frame){response.reason="wrong_frame";return;}
    if(!map_){response.reason="map_missing";return;}
    if(request.path.empty()||request.path.size()>20000){response.reason="invalid_path_size";return;}
    if(!std::isfinite(request.clearance_margin)||request.clearance_margin<0||request.clearance_margin>1.){
      response.reason="invalid_clearance_margin";return;
    }
    if(!RobotBounds::validMode(request.bound_mode)){response.reason="unsupported_bound_mode";return;}
    const Eigen::Vector3d robot_size=policy_.robot_bounds.size(request.bound_mode);
    const Eigen::Vector3d checked_size=robot_size+Eigen::Vector3d::Constant(2.*request.clearance_margin);
    std::vector<StateVec> states;
    for(const auto& pose:request.path){StateVec state;if(!poseState(pose,state)){response.reason="invalid_pose";return;}states.push_back(state);}
    try{
      // Validate the complete commanded polyline conservatively. This may
      // cancel on an obstacle behind the robot too; it never shortens or
      // substitutes the route when responding to a validation request.
      if(!observation.valid||map_->box(baseState(observation.state).head<3>()+policy_.center_offset,robot_size)!=VoxelStatus::Free){response.reason="current_pose_not_free";return;}
      for(std::size_t i=0;i<states.size();++i){
        const auto& a=states[i];const auto& b=states[std::min(i+1,states.size()-1)];
        const auto& size=(request.clearance_segments==0||i<request.clearance_segments)?checked_size:robot_size;
        const auto reason=(i==0&&request.first_pose_is_observation&&policy_.ground_robot)?
            validateObservedGroundConnection(a,b,size):validateSegment(a,b,size);
        if(!reason.empty()){response.reason=reason;response.failure_segment=static_cast<int32_t>(i);return;}
      }
      response.valid=true;response.reason="ok";
    }catch(const std::exception&){response.reason="invalid_geometry";}
  }
  StateVec baseState(const StateVec& planning)const{return policy_.baseState(planning);}
  bool poseState(const geometry_msgs::msg::Pose& pose,StateVec& state)const{return policy_.poseState(pose,state);}
  std::string validateObservedGroundConnection(const StateVec& observed,const StateVec& route,
                                              const Eigen::Vector3d& size)const{
    // A retained guard may itself be the original measured start pose. If
    // this complete connector already passes ordinary terrain validation,
    // projecting just one end would invent a slope that was never requested.
    const auto original_reason=validateSegment(observed,route,size,true);
    if(original_reason!="terrain_slope_exceeded")return original_reason;
    // A live odometry pose and a TSDF waypoint use different height estimates.
    // On a short retained guard segment that estimation error is not terrain
    // inclination. Normalize this explicitly identified observation; the
    // continuous simplex mode below also checks the route attachment's current
    // support. Commanded waypoints and ordinary requests stay unchanged.
    const auto observed_reason=validateSegment(observed,observed,size);
    if(!observed_reason.empty())return observed_reason;
    if(map_->path(baseState(observed).head<3>()+policy_.center_offset,
                  baseState(route).head<3>()+policy_.center_offset,size)!=VoxelStatus::Free)
      return "path_not_free";
    auto ground=policy_.ground;ground.robot_size=size;ground.point_robot=size.isZero(0.);
    const auto support=TerrainProjector(map_,ground).project(observed);
    if(support.status!=GroundStatus::Supported||
       (support.projected_state-observed).head<2>().norm()>1e-8)return "terrain_support_missing";
    if(std::abs(support.projected_state.z()-observed.z())>policy_.resolution+1e-8)return "path_off_terrain";
    if(ground.interpolate_projection_distance&&
       ground.tsdf_projection_mode==GroundTsdfProjectionMode::ZeroCrossingSimplex){
      // This explicit observation connector is validation geometry. Its route
      // attachment may be an interpolation of older fixed waypoint heights;
      // over a millimetre of XY that estimation difference is not a slope.
      // In the continuous simplex field, check the current support at BOTH
      // endpoints, without changing the commanded route or its later edges.
      // Original sweep/support/height checks still apply, and the normalized
      // connector must pass the ordinary directional slope and body checks.
      const auto route_reason=validateSegment(route,route,size);
      if(!route_reason.empty())return route_reason;
      const auto route_support=TerrainProjector(map_,ground).project(route);
      if(route_support.status!=GroundStatus::Supported||
         (route_support.projected_state-route).head<2>().norm()>1e-8)return "terrain_support_missing";
      if(std::abs(route_support.projected_state.z()-route.z())>policy_.resolution+1e-8)return "path_off_terrain";
      auto target=route_support.projected_state;
      // Numerically identical support sites do not define a vertical drive.
      if((target-support.projected_state).head<3>().norm()<=1e-10)
        target=support.projected_state;
      return validateSegment(support.projected_state,target,size);
    }
    // Reprojecting the retained waypoint would compound the TSDF step's
    // starting-height dependence. Keep it fixed, as required by the command.
    if((observed-route).head<3>().norm()<1e-10)return {};
    return validateSegment(support.projected_state,route,size,true);
  }
  std::string validateSegment(const StateVec& a,const StateVec& b,const Eigen::Vector3d& size,
                              bool observed_first=false)const{
    if(!policy_.ground_robot)
      return map_->path(a.head<3>()+policy_.center_offset,b.head<3>()+policy_.center_offset,size)==VoxelStatus::Free?
          "":"path_not_free";
    auto ground=policy_.ground;ground.robot_size=size;ground.point_robot=size.isZero(0.);
    // Validate the requested geometry. Projecting its samples again can move
    // an already checked TSDF route into the floor or a different obstacle.
    // The shared check retains body sweep, center support, slope and height
    // constraints, including only the existing narrow observation seam.
    const bool legacy_attachment=ground.interpolate_projection_distance&&
        ground.tsdf_projection_mode==GroundTsdfProjectionMode::UpstreamStep;
    const bool height_attachment=legacy_attachment||observed_first;
    const auto checked=TerrainProjector(map_,ground).validateFixedEdge(a,b,height_attachment);
    switch(checked.status){
      case GroundStatus::Admissible:return {};
      case GroundStatus::Occupied:
      case GroundStatus::Unknown:return "path_not_free";
      case GroundStatus::Steep:return "terrain_slope_exceeded";
      case GroundStatus::OffTerrain:return "path_off_terrain";
      default:return "terrain_support_missing";
    }
  }

 private:
  const PathValidationPolicy policy_;
  const PathValidationInputs::Snapshot snapshot_;
  const std::shared_ptr<const MapQuery> map_;
};

// A normal map/policy commit during a query is not a collision. Recheck the
// entire requested path against the new snapshot, including the latest pose.
// Continuous updates are bounded; never return a successful superseded result.
inline planner_msgs::srv::ValidatePath::Response validateCurrentPath(
    const PathValidationPolicy& policy,const PathValidationInputs& inputs,
    const planner_msgs::srv::ValidatePath::Request& request,
    PathValidationInputs::Snapshot* validated_snapshot=nullptr,
    PathValidationObservation* validated_observation=nullptr){
  planner_msgs::srv::ValidatePath::Response response;
  for(int attempt=0;attempt<3;++attempt){
    const auto snapshot=inputs.mapSnapshot();
    const auto observation=inputs.observation();
    PathValidationContext(policy,snapshot).validate(request,observation,response);
    if(inputs.mapSnapshot()==snapshot){
      if(validated_snapshot)*validated_snapshot=snapshot;
      if(validated_observation)*validated_observation=observation;
      return response;
    }
  }
  response.valid=false;response.reason="map_changed_during_validation";
  response.failure_segment=-1;
  if(validated_snapshot)validated_snapshot->reset();
  return response;
}

}  // namespace gbplanner2
