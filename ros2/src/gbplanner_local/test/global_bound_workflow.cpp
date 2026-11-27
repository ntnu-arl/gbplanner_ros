// Standalone geometric workflow; prints one compact result for the caller.
#include "gbplanner_local/global_bound.hpp"
#include <Eigen/Geometry>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <utility>
#include <vector>

using namespace gbplanner2;
using V3=Eigen::Vector3d;

namespace {
void require(bool condition,const char* reason){
  if(!condition)throw std::runtime_error(reason);
}
using RayLog=std::vector<std::pair<V3,VoxelStatus>>;
RayLog scan(const MapQuery& map,const V3& start,const V3& end){
  RayLog log;
  map.ray(start,end,[&](const V3& point,VoxelStatus status){
    log.emplace_back(point,status);
    return status!=VoxelStatus::Occupied;
  });
  return log;
}
}

int main(){
  try{
    auto map=std::make_shared<VoxelMap>(.25,V3(-5.,-5.,-2.),Eigen::Vector3i(40,40,16));
    map->fill(VoxelStatus::Free);
    const Eigen::Matrix3d rotation=Eigen::AngleAxisd(std::acos(-1.)/4.,V3::UnitZ()).toRotationMatrix();
    const V3 half(2.,.5,1.);
    const V3 corner=rotation*(-half);
    const GlobalBound bound=GlobalBound::fromCorners(corner,
        corner+rotation*V3(4.,0.,0.),corner+rotation*V3(0.,1.,0.),corner+rotation*V3(0.,0.,2.));
    BoundedMap bounded(map,bound);
    require(bound.center().norm()<1e-14,"Wrong cuboid center");
    require((bound.halfSize()-half).norm()<1e-14,"Wrong cuboid half-size");
    require((bound.rotation()-rotation).norm()<1e-14,"Wrong cuboid orientation");
    require(bound.contains(corner),"Closed cuboid boundary was excluded");

    // Its center is inside both the OBB and enclosing AABB, but a finite-size
    // robot crosses the oblique face. Center-only or AABB-only checks miss it.
    const V3 edge_center=rotation*V3(1.8,.35,0.);
    const V3 large_robot=V3::Constant(.4);
    const GlobalBound enclosing=GlobalBound::axisAligned(bound.minimum(),bound.maximum());
    require(enclosing.contains(edge_center)&&bound.contains(edge_center),"Fixture center must be inside");
    require(map->box(edge_center,large_robot)==VoxelStatus::Free,"Fixture must have no obstacle");
    require(!bound.containsBox(edge_center,large_robot),"Robot crossing the OBB face was accepted");
    require(bounded.box(edge_center,large_robot)==VoxelStatus::Occupied,"Out-of-bound robot box was accepted");
    require(bounded.path(V3::Zero(),edge_center,large_robot)==VoxelStatus::Occupied,"Out-of-bound swept box was accepted");

    const V3 start=rotation*V3(-1.,0.,0.),end=rotation*V3(1.,0.,0.);
    require(bounded.path(start,end,V3::Constant(.2))==VoxelStatus::Free,"Internal route was rejected");
    const V3 aabb_only(0.,1.5,0.);
    require(enclosing.contains(aabb_only)&&!bound.contains(aabb_only),"Fixture must distinguish AABB and OBB");
    require(bounded.gainAllowed(V3::Zero())&&!bounded.gainAllowed(aabb_only),"Gain did not respect the OBB");
    auto narrow=std::make_shared<BoundedMap>(map,GlobalBound::axisAligned(V3(-.8,-3.,-1.5),V3(.8,3.,1.5)));
    const BoundedMap combined(narrow,bound);
    require(bound.contains(V3(1.,1.,0.))&&!combined.gainAllowed(V3(1.,1.,0.)),"Base gain restriction was lost");

    // An observed wall outside the workspace still terminates a sensor ray.
    const V3 wall(2.1,1.1,.1);
    map->set(map->index(wall),VoxelStatus::Occupied);
    require(!bounded.gainAllowed(wall),"Outside wall was included in gain");
    require(bounded.voxel(wall)==VoxelStatus::Occupied,"Observed wall was altered");
    const auto raw=scan(*map,V3(-3.,1.1,.1),V3(3.,1.1,.1));
    const auto constrained=scan(bounded,V3(-3.,1.1,.1),V3(3.,1.1,.1));
    require(!raw.empty()&&raw.back().second==VoxelStatus::Occupied,"Ray must terminate at the wall");
    require(raw.size()==constrained.size(),"Workspace policy truncated the physical ray");
    for(std::size_t i=0;i<raw.size();++i)
      require(raw[i].second==constrained[i].second&&raw[i].first==constrained[i].first,"Ray observation changed");
    require(bounded.minimum()==map->minimum()&&bounded.maximum()==map->maximum(),"Observed map extent changed");

    std::cout<<std::setprecision(12)
      <<"{\"status\":\"passed\",\"yaw_rad\":"<<std::acos(-1.)/4.
      <<",\"obb_crossing_box_rejected\":true,\"internal_path_free\":true"
      <<",\"outside_gain_excluded\":true,\"base_gain_preserved\":true"
      <<",\"ray_samples\":"<<raw.size()<<",\"outside_wall_occlusion_preserved\":true}\n";
    return 0;
  }catch(const std::exception& error){
    std::cerr<<"Global bound workflow failed: "<<error.what()<<'\n';
    return 1;
  }
}
