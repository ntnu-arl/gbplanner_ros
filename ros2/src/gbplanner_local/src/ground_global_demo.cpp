// Exercise direction-dependent ground routing through GlobalMemory itself.
#include "gbplanner_local/global_memory.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

using namespace gbplanner2;
namespace {
void require(bool condition,const char* message){
  if(!condition)throw std::runtime_error(message);
}
std::shared_ptr<VoxelMap> ramp(){
  auto map=std::make_shared<VoxelMap>(.05,Eigen::Vector3d(-2.,-1.,-1.),Eigen::Vector3i(80,40,80));
  for(int z=0;z<80;++z)for(int y=0;y<40;++y)for(int x=0;x<80;++x){
    const Eigen::Vector3i index(x,y,z);const auto point=map->center(index);
    const double height=.5*std::clamp(point.x()+1.,0.,2.);
    map->set(index,point.z()<height?VoxelStatus::Occupied:VoxelStatus::Free);
  }
  return map;
}
void printPath(const std::vector<StateVec>& path){
  std::cout<<'[';
  for(std::size_t i=0;i<path.size();++i){
    if(i)std::cout<<',';
    std::cout<<'['<<path[i].x()<<','<<path[i].y()<<','<<path[i].z()<<']';
  }
  std::cout<<']';
}
}

int main(){
  try{
    const auto map=ramp();Parameters p;p.ground_robot=true;p.overshoot=0.;
    p.robot_size={.3,.3,.3};p.ground.max_ground_height=.8;p.ground.robot_height=.6;
    p.ground.max_inclination=.9;p.ground.max_negative_inclination=.35;
    TerrainProjector terrain(map,p.groundParameters());
    const auto low=terrain.project(StateVec(-1.5,0.,2.,0.));
    const auto high=terrain.project(StateVec(1.5,0.,2.,0.));
    require(low.status==GroundStatus::Supported&&high.status==GroundStatus::Supported,
            "Ramp endpoints have no observed support");
    const std::vector<StateVec> reference{low.projected_state,high.projected_state};
    GlobalMemory restricted(p);restricted.updateMap(map);
    require(restricted.setHomingPosition(reference.front()),"Failed to set ground home");
    require(restricted.addReferencePath(reference),"Uphill reference route was rejected");
    const int top_id=static_cast<int>(restricted.graph().vertices_map_.size())-1;
    const auto gain=[](const StateVec&){return VolumetricGain{};};
    const auto outward=restricted.selectFrontier(reference.front(),gain,top_id);
    require(outward.success&&outward.path.size()>10,"Directed uphill global route was not found");
    const int edges_before=restricted.edgeCount();
    restricted.updateMap(map);
    require(restricted.edgeCount()==edges_before&&restricted.lastMapEdgeUpdate().invalidated==0,
            "Map refresh removed a link whose uphill direction remains valid");
    const auto rejected=restricted.homing(reference.back());
    require(!rejected.success&&rejected.reason=="global_target_unreachable",
            "Global Dijkstra traversed a forbidden downhill route");

    // A generous time budget cannot make an impossible return direction legal.
    GlobalTimeConstraint budget;budget.remaining=10000.;budget.speed=.3;budget.include_home=true;
    const auto no_return=restricted.selectFrontier(reference.front(),gain,top_id,&budget);
    require(!no_return.success&&no_return.reason=="global_time_insufficient",
            "Frontier feasibility reused outbound distance as a downhill return");

    p.ground.max_negative_inclination=.9;
    GlobalMemory permissive(p);permissive.updateMap(map);
    require(permissive.setHomingPosition(reference.front())&&permissive.addReferencePath(reference),
            "Permissive ground graph construction failed");
    const auto home=permissive.homing(reference.back());
    require(home.success&&home.path.size()>10,"Permissive homing did not return a terrain polyline");
    require((home.path.front().head<3>()-reference.back().head<3>()).norm()<1e-9&&
            (home.path.back().head<3>()-reference.front().head<3>()).norm()<1e-9,
            "Homing changed its requested endpoints");
    double length=0.,min_z=home.path.front().z(),max_z=min_z;
    for(std::size_t i=0;i<home.path.size();++i){
      min_z=std::min(min_z,home.path[i].z());max_z=std::max(max_z,home.path[i].z());
      if(i)length+=(home.path[i]-home.path[i-1]).head<3>().norm();
    }
    const double chord=(reference.back()-reference.front()).head<3>().norm();
    require(max_z-min_z>.9&&std::abs(length-home.distance)<1e-9&&length>chord+.01,
            "Global route length or elevation was flattened to its endpoint chord");
    std::cout<<std::setprecision(15)
      <<"{\"status\":\"passed\",\"height_convention\":\"planning\""
      <<",\"uphill_allowed\":true,\"downhill_rejected\":true,\"one_way_edges_preserved\":true"
      <<",\"impossible_return_budget_rejected\":true,\"vertices\":"<<permissive.graph().vertices_map_.size()
      <<",\"route_points\":"<<home.path.size()<<",\"actual_length_m\":"<<home.distance
      <<",\"endpoint_chord_m\":"<<chord<<",\"height_span_m\":"<<max_z-min_z
      <<",\"planning_path\":";
    printPath(home.path);std::cout<<"}\n";return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
