// Focused numerical check of the checked ground-return reconnection port.
#include "gbplanner_local/global_memory.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

using namespace gbplanner2;
namespace {
void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
double length(const std::vector<StateVec>& path){
  double result=0.;for(std::size_t i=1;i<path.size();++i)result+=(path[i]-path[i-1]).head<3>().norm();
  return result;
}
std::shared_ptr<VoxelMap> scene(bool wall=false,bool gap=false,bool rolling=false){
  auto map=std::make_shared<VoxelMap>(.02,Eigen::Vector3d(-.5,-.5,-.2),Eigen::Vector3i(200,100,80));
  for(int z=0;z<80;++z)for(int y=0;y<100;++y)for(int x=0;x<200;++x){
    const Eigen::Vector3i index(x,y,z);const auto point=map->center(index);
    const double height=rolling?.12+.1*std::sin(2.*point.x()):0.;
    VoxelStatus status=point.z()<height?VoxelStatus::Occupied:VoxelStatus::Free;
    const bool strip=point.x()<1.8&&point.y()>-.04&&point.y()<0.;
    if(wall&&strip&&point.z()<.8)status=VoxelStatus::Occupied;
    if(gap&&strip&&point.z()<.02)status=VoxelStatus::Unknown;
    map->set(index,status);
  }
  return map;
}
std::vector<StateVec> loop(const TerrainProjector& terrain){
  std::vector<StateVec> path;
  for(const auto& xy:std::vector<Eigen::Vector2d>{{0.,-.06},{2.,-.06},{2.,1.},{1.06,1.},{1.06,.02},{0.,.02}}){
    const auto projected=terrain.project(StateVec(xy.x(),xy.y(),1.,.4));
    require(projected.status==GroundStatus::Supported,"Loop has no supported endpoint");
    path.push_back(projected.projected_state);
  }
  path.front()[3]=.7;path.back()[3]=-.3;
  return path;
}
void checked(const TerrainProjector& terrain,const std::vector<StateVec>& path){
  for(std::size_t i=1;i<path.size();++i)
    require(terrain.edge(path[i-1],path[i]).admissible(),"Output contains a rejected terrain segment");
}
}
int main(){
  try{
    GroundParameters ground;ground.max_ground_height=.6;ground.robot_height=.4;
    ground.robot_size={.02,.02,.02};ground.max_inclination=.9;ground.max_negative_inclination=.9;
    auto flat_map=scene();TerrainProjector flat(flat_map,ground);
    auto free_path=loop(flat);const auto original=free_path;const double original_length=length(original);
    checked(flat,free_path);
    double free_length=original_length;const auto free_result=reconnectGroundPath(flat,free_path,free_length);
    require(free_result.modified&&free_result.used_shortcuts>0&&free_length<.11,
            "Nearby return branches were not reconnected");
    require((free_path.front()-original.front()).norm()<1e-8&&
            (free_path.back()-original.back()).norm()<1e-8,
            "Reconnection changed a requested endpoint pose");
    checked(flat,free_path);

    double wall_length=0.,gap_length=0.;
    for(int kind=0;kind<2;++kind){
      TerrainProjector obstructed(scene(kind==0,kind==1),ground);
      auto path=loop(obstructed);checked(obstructed,path);double distance=length(path);
      const auto result=reconnectGroundPath(obstructed,path,distance);
      require(result.attempted&&distance>5.,"Reconnection crossed a wall or unknown support gap");
      require(std::any_of(path.begin(),path.end(),[](const StateVec& state){return state.x()>1.8;}),
              "Safe detour around the blocked near-neighbor connection was removed");
      require(std::abs(distance-length(path))<1e-8,"Reconnected length disagrees with its output geometry");
      checked(obstructed,path);
      if(kind==0)wall_length=distance;else gap_length=distance;
    }

    TerrainProjector rolling(scene(false,false,true),ground);
    auto wave_edge=rolling.edge(StateVec(0.,.3,1.,.2),StateVec(3.,.3,1.,.2));
    require(wave_edge.admissible()&&wave_edge.polyline.size()>3,"Rolling-terrain reference is invalid");
    auto wave_path=wave_edge.polyline;const auto wave_start=wave_path.front(),wave_end=wave_path.back();
    double wave_length=length(wave_path);const auto wave_result=reconnectGroundPath(rolling,wave_path,wave_length);
    require(wave_result.attempted,"Rolling-terrain reconnection was skipped");
    double min_height=wave_path.front().z(),max_height=min_height;
    for(const auto& point:wave_path){min_height=std::min(min_height,point.z());max_height=std::max(max_height,point.z());}
    require(max_height-min_height>.15&&wave_path.size()>10&&
            wave_length>(wave_end-wave_start).head<3>().norm()+.01,
            "Straight-run compression flattened the terrain profile");
    checked(rolling,wave_path);

    Parameters parameters;parameters.ground_robot=true;parameters.ground=ground;
    parameters.robot_size=ground.robot_size;parameters.overshoot=0.;
    parameters.refinement.homing_backward=true;parameters.refinement.planning_backward=false;
    GlobalMemory global(parameters);global.updateMap(flat_map);
    const StateVec home(0.,.3,.6,3.14159265358979323846);
    const StateVec stored(3.,.3,.6,3.14159265358979323846);
    require(global.setHomingPosition(home)&&global.addReferencePath({home,stored}),
            "Global return reference construction failed");
    const auto vertices=global.graph().vertices_map_.size();const int graph_edges=global.edgeCount();
    StateVec current=stored;current[3]=.25;
    const auto returning=global.homing(current);
    require(returning.success&&returning.reconnection.attempted&&returning.path.size()>2,
            "GlobalMemory.homing did not run ground reconnection");
    require(std::abs(returning.path.front()[3]-.25)<1e-8,"Homing lost the actual current yaw");
    for(std::size_t i=1;i<returning.path.size();++i)
      require(std::abs(std::remainder(returning.path[i][3],2.*3.14159265358979323846))<1e-8,
              "Homing backward orientation was overwritten by planning_backward");
    // connect() may add validated neighbors at the existing start, so compare
    // repeated homing calls after that normal graph attachment has completed.
    const auto settled_vertices=global.graph().vertices_map_.size();const int settled_edges=global.edgeCount();
    const auto repeated=global.homing(current);
    require(repeated.success&&global.graph().vertices_map_.size()==settled_vertices&&
            global.edgeCount()==settled_edges&&settled_vertices==vertices&&settled_edges>=graph_edges,
            "The temporary return graph leaked into persistent global topology");

    std::cout<<std::setprecision(15)<<"{\"status\":\"passed\",\"original_length_m\":"<<original_length
      <<",\"reconnected_length_m\":"<<free_length<<",\"used_shortcuts\":"<<free_result.used_shortcuts
      <<",\"wall_detour_length_m\":"<<wall_length<<",\"unknown_detour_length_m\":"<<gap_length
      <<",\"terrain_height_span_m\":"<<max_height-min_height<<",\"terrain_length_m\":"<<wave_length
      <<",\"global_homing_points\":"<<returning.path.size()<<",\"current_yaw_preserved\":true"
      <<",\"homing_backward_applied\":true,\"temporary_graph_only\":true}\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
