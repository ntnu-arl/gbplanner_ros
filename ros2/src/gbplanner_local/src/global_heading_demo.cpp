// Compare the three upstream global-route heading policies on one graph.
#include "gbplanner_local/global_memory.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace gbplanner2;
namespace {
constexpr double kPi=3.14159265358979323846;
void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
StateVec pose(double x,double yaw=.7){return StateVec(x,0.,0.,yaw);}
VolumetricGain gain(const StateVec&){VolumetricGain result;result.is_frontier=true;result.gain=10.;return result;}
void check(const GlobalResult& result,const VoxelMap& map,const Parameters& parameters,
           int target,double endpoint,double expected_yaw){
  require(result.success&&result.target==target&&result.path.size()>=2,"Global route failed or changed target");
  require((result.path.front()-pose(0.,.25)).norm()<1e-9,"Route changed the actual start pose");
  require((result.path.back().head<3>()-pose(endpoint).head<3>()).norm()<1e-9&&
          std::abs(result.distance-std::abs(endpoint))<1e-9,"Route changed endpoint or length");
  for(std::size_t i=1;i<result.path.size();++i){
    require(std::abs(std::remainder(result.path[i][3]-expected_yaw,2.*kPi))<1e-9,
            "Global route used another producer's heading policy");
    require(map.path(result.path[i-1].head<3>(),result.path[i].head<3>(),parameters.robot_size)==VoxelStatus::Free,
            "Heading comparison route contains a blocked segment");
  }
}
void refined(const GlobalResult& result){
  require(result.refinement.attempted&&result.refinement.modified&&result.refinement.removed_vertices>0&&
          result.refinement.reason=="refined","Comparison did not exercise successful path enhancement");
}
}

int main(){
  try{
    Parameters p;p.refinement.homing_backward=true;p.refinement.planning_backward=false;
    p.refinement.safety_extension={.5,.5,.5};
    auto map=std::make_shared<VoxelMap>(.25,Eigen::Vector3d(-6.,-2.,-2.),Eigen::Vector3i(32,16,16));
    map->fill(VoxelStatus::Free);
    GlobalMemory memory(p);memory.updateMap(map);
    // Quarter-meter graph segments make the enhancement pass actually prune
    // vertices. Home and frontier both lie >2 m behind the robot's heading.
    std::vector<StateVec> reference;
    for(int i=0;i<=16;++i)reference.push_back(pose(-4.+.25*i));
    require(memory.setHomingPosition(reference.front())&&memory.addReferencePath(reference),"Could not seed global graph");
    const int frontier=memory.addSemanticPath({pose(-3.)},planner_semantic_msgs::msg::SemanticClass::K_DOOR);
    require(frontier>0,"Could not label the retained frontier");
    const auto current=pose(0.,.25);
    auto global=memory.selectFrontier(current,gain);
    auto home=memory.homing(current);
    auto waypoint=memory.routeToWaypoint(current,pose(-3.));
    check(global,*map,p,frontier,-3.,kPi);
    check(home,*map,p,0,-4.,0.);
    check(waypoint,*map,p,frontier,-3.,0.);
    require(!global.refinement.attempted&&!home.refinement.attempted&&!waypoint.refinement.attempted,
            "Disabled enhancement was unexpectedly attempted");

    p.refinement.enabled=true;memory.setPlanningParameters(p);
    global=memory.selectFrontier(current,gain);
    home=memory.homing(current);
    waypoint=memory.routeToWaypoint(current,pose(-3.));
    refined(global);refined(home);refined(waypoint);
    check(global,*map,p,frontier,-3.,kPi);
    check(home,*map,p,0,-4.,0.);
    // getGlobalPath keeps improveFreePath's planning_backward=false tangents;
    // getHomingPath alone restores the homing-relative heading afterward.
    check(waypoint,*map,p,frontier,-3.,kPi);

    // runGlobalPlanner also keeps planning_backward from a successful
    // enhancement, rather than forcing its initial forward headings again.
    p.refinement.planning_backward=true;memory.setPlanningParameters(p);
    global=memory.selectFrontier(current,gain,frontier);
    refined(global);check(global,*map,p,frontier,-3.,0.);
    std::cout<<"{\"status\":\"passed\",\"shared_graph\":true,\"unrefined_global_forward\":true"
      <<",\"homing_backward_preserved\":true,\"unrefined_waypoint_homing_policy\":true"
      <<",\"successful_refinement\":true,\"refined_waypoint_planning_policy\":true"
      <<",\"refined_global_planning_backward\":true,\"actual_start_yaw_preserved\":true}\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
