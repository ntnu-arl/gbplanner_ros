// Strict full-start attachment beyond the RRG neighbor radius, without odometry.
#include "gbplanner_local/global_memory.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

using namespace gbplanner2;
namespace {
void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
Parameters parameters(bool ground){
  Parameters p;p.ground_robot=ground;p.robot_size={.2,.2,.2};
  p.edge_min=.1;p.edge_max=3.;p.neighbor_radius=2.5;p.neighbor_max=3.;p.overshoot=.2;
  p.ground.max_ground_height=.8;p.ground.robot_height=.6;
  p.ground.max_inclination=.9;p.ground.max_negative_inclination=.9;
  p.refinement.enabled=false;p.tangent_yaw=false;return p;
}
std::shared_ptr<VoxelMap> flat(bool ground){
  auto map=std::make_shared<VoxelMap>(.1,Eigen::Vector3d(-1.,-1.,-1.),Eigen::Vector3i(60,20,40));
  map->fill(VoxelStatus::Free);
  if(ground)for(int z=0;z<10;++z)for(int y=0;y<20;++y)for(int x=0;x<60;++x)
    map->set({x,y,z},VoxelStatus::Occupied);
  return map;
}
std::shared_ptr<VoxelMap> ramp(){
  auto map=std::make_shared<VoxelMap>(.05,Eigen::Vector3d(-1.,-.5,-.5),Eigen::Vector3i(100,20,80));
  for(int z=0;z<80;++z)for(int y=0;y<20;++y)for(int x=0;x<100;++x){
    const Eigen::Vector3i index(x,y,z);const auto point=map->center(index);
    const double height=.5*std::clamp(point.x(),0.,2.5);
    map->set(index,point.z()<height?VoxelStatus::Occupied:VoxelStatus::Free);
  }
  return map;
}
struct Outcome {GlobalResult route;int vertices{0},edges{0};};
Outcome connect(const std::shared_ptr<VoxelMap>& map,const Parameters& p,const StateVec& home,const StateVec& current){
  GlobalMemory memory(p);memory.updateMap(map);
  require(memory.setHomingPosition(home),"Could not seed the graph's home anchor");
  const auto visit_revision=memory.visitRevision();
  // No recordOdometry or reference insertion: only the route's connect()
  // can create the new parent edge. A query is not an observed robot visit.
  Outcome out;out.route=memory.homing(current);
  out.vertices=static_cast<int>(memory.graph().vertices_map_.size());out.edges=memory.edgeCount();
  require(memory.visitedStates().empty()&&memory.visitRevision()==visit_revision&&
          memory.lastVisitedMarker().isZero(),"Connection query invented a visit event");
  return out;
}
void rejected(const Outcome& out,const char* reason){
  require(!out.route.success&&out.route.path.empty()&&out.route.reason=="current_not_connected"&&
          out.vertices==1&&out.edges==0,reason);
}
void exactRoute(const Outcome& out,const StateVec& home,const StateVec& current){
  require(out.route.success&&out.route.path.size()>=2&&out.vertices==2&&out.edges==1,
          "Fallback did not add exactly one real parent edge");
  require((out.route.path.front()-current).norm()<1e-9&&
          (out.route.path.back()-home).head<3>().norm()<1e-9,"Fallback changed the requested start or home");
  double length=0.;
  for(std::size_t i=1;i<out.route.path.size();++i)
    length+=(out.route.path[i]-out.route.path[i-1]).head<3>().norm();
  require(std::abs(length-out.route.distance)<1e-9,"Returned distance differs from complete path length");
}
}

int main(){
  try{
    const StateVec aerial_home(0.,0.,0.,0.),aerial_current(2.75,0.,0.,.3);
    auto aerial_map=flat(false);const auto aerial=parameters(false);
    // A root anchor does not extend backwards. The actual route and its
    // candidate-side overshoot are free, but a symmetric extension is not.
    aerial_map->set(aerial_map->index({-.25,0.,0.}),VoxelStatus::Occupied);
    const auto aerial_result=connect(aerial_map,aerial,aerial_home,aerial_current);
    exactRoute(aerial_result,aerial_home,aerial_current);
    rejected(connect(aerial_map,aerial,aerial_home,StateVec(3.01,0.,0.,0.)),
             "An over-length actual start was clipped or partially inserted");
    auto overshoot_map=flat(false);
    overshoot_map->set(overshoot_map->index({2.95,0.,0.}),VoxelStatus::Occupied);
    require(overshoot_map->path(aerial_home.head<3>(),aerial_current.head<3>(),aerial.robot_size)==VoxelStatus::Free,
            "Overshoot obstacle fixture also blocks the actual parent edge");
    rejected(connect(overshoot_map,aerial,aerial_home,aerial_current),"Fallback skipped the candidate-side overshoot");
    // An existing nearby path keeps its previous admission rules, even when
    // edge_max is shorter than neighbor_radius. The fallback must not tighten it.
    auto near_parameters=aerial;near_parameters.edge_max=1.5;
    const StateVec nearby(2.,0.,0.,.3);
    exactRoute(connect(flat(false),near_parameters,aerial_home,nearby),aerial_home,nearby);

    const auto ground=parameters(true);auto ground_map=flat(true);
    const StateVec ground_home(0.,0.,.8,0.),ground_current(2.75,0.,.8,.3);
    const auto ground_result=connect(ground_map,ground,ground_home,ground_current);
    exactRoute(ground_result,ground_home,ground_current);
    require(ground_result.route.path.size()>2,"Ground connection lost its checked terrain polyline");
    rejected(connect(ground_map,ground,ground_home,StateVec(3.01,0.,.8,0.)),
             "Over-length ground start was shortened");
    auto blocked=flat(true),unknown=flat(true);
    for(int z=0;z<40;++z)for(int y=0;y<20;++y)for(int x=0;x<60;++x){
      const Eigen::Vector3i index(x,y,z);const auto point=blocked->center(index);
      if(point.x()>1.2&&point.x()<1.6){
        if(point.z()<1.8)blocked->set(index,VoxelStatus::Occupied);
        if(point.z()<0.)unknown->set(index,VoxelStatus::Unknown);
      }
    }
    rejected(connect(blocked,ground,ground_home,ground_current),"Ground fallback crossed a body obstacle");
    rejected(connect(unknown,ground,ground_home,ground_current),"Ground fallback bridged unknown support");

    auto slope_map=ramp();auto slope_parameters=ground;slope_parameters.ground.max_negative_inclination=.35;
    TerrainProjector terrain(slope_map,slope_parameters.groundParameters());
    const auto low=terrain.project(StateVec(0.,0.,2.5,0.));
    const auto high=terrain.project(StateVec(2.5,0.,2.5,.3));
    require(low.status==GroundStatus::Supported&&high.status==GroundStatus::Supported,
            "Slope fixture endpoints must have observed support");
    const double slope_chord=(high.projected_state-low.projected_state).head<3>().norm();
    require(slope_chord>slope_parameters.neighbor_radius&&slope_chord<=slope_parameters.edge_max&&
            terrain.edge(low.projected_state,high.projected_state).admissible()&&
            !terrain.edge(high.projected_state,low.projected_state).admissible(),
            "Slope fixture must permit the parent direction but forbid current-to-home");
    rejected(connect(slope_map,slope_parameters,low.projected_state,high.projected_state),
             "Minimum bidirectional cost admitted a forbidden return direction");

    std::cout<<std::setprecision(15)<<"{\"status\":\"passed\",\"neighbor_radius_m\":2.5,\"edge_max_m\":3"
      <<",\"connector_m\":2.75,\"aerial_points\":"<<aerial_result.route.path.size()
      <<",\"ground_points\":"<<ground_result.route.path.size()<<",\"ground_length_m\":"<<ground_result.route.distance
      <<",\"slope_chord_m\":"<<slope_chord
      <<",\"exact_start_preserved\":true,\"single_parent_edge\":true,\"root_overshoot_exception\":true"
      <<",\"candidate_overshoot_checked\":true,\"existing_nearby_behavior_preserved\":true"
      <<",\"over_max_rejected\":true,\"body_obstacle_rejected\":true,\"unknown_support_rejected\":true"
      <<",\"forbidden_return_direction_rejected\":true,\"no_odometry_or_visit_fabrication\":true}\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
