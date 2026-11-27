// Paired synthetic trials: same map/seed, local-only versus retained global graph.
#include "gbplanner_local/global_memory.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

using namespace gbplanner2;
namespace {
std::shared_ptr<VoxelMap> scene(){
  auto grid=std::make_shared<VoxelMap>(.25,Eigen::Vector3d(-5,-5,-1.5),Eigen::Vector3i(40,40,12));
  for(int z=0;z<12;++z)for(int y=0;y<40;++y)for(int x=0;x<40;++x){
    auto center=grid->center({x,y,z});
    auto status=center.x()>=1.5?VoxelStatus::Unknown:VoxelStatus::Free;
    if(std::abs(center.x())<.25&&std::abs(center.y())>1)status=VoxelStatus::Occupied;
    grid->set({x,y,z},status);
  }
  return grid;
}
bool independentlySafe(const std::vector<StateVec>& path,const VoxelMap& map){
  if(path.empty())return true;
  for(std::size_t n=0;n<path.size();++n){
    auto start=path[n].head<3>().eval();
    Eigen::Vector3d delta=n+1<path.size()?Eigen::Vector3d(path[n+1].head<3>()-start):Eigen::Vector3d::Zero();
    int count=std::max(1,int(std::ceil(delta.norm()/.025)));
    for(int k=0;k<=count;++k)for(int x=-2;x<=2;++x)for(int y=-2;y<=2;++y)for(int z=-2;z<=2;++z)
      if(map.voxel(start+delta*(double(k)/count)+Eigen::Vector3d(x,y,z)*.1)!=VoxelStatus::Free)return false;
  }
  return true;
}
void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
}
int main(int argc,char** argv){
  try{
    const std::filesystem::path output=argc>1?argv[1]:"results/global_experiments";
    std::filesystem::create_directories(output);
    std::ofstream csv(output/"benchmark.csv");csv<<std::setprecision(12);
    csv<<"seed,initial_local_success,frontier_candidates,frontier_clusters,rejected_near_graph,rejected_near_history,rejected_path,retained_frontiers,local_dead_end_reason,global_success,global_reason,global_path_m,global_ms,homing_success,homing_path_m,blocked_homing_reason,independent_collision_check\n";
    std::ofstream paths(output/"paths.csv");paths<<std::setprecision(12)<<"seed,mode,index,x,y,z\n";
    int recovered=0,local_stalls=0,homes=0,blocked=0,initial_successes=0;
    for(uint32_t seed=0;seed<20;++seed){
      Parameters p;p.seed=seed;auto grid=scene();GlobalMemory memory(p);memory.updateMap(grid);
      StateVec home(-2,0,0,0),corner(-4.5,-4.5,0,0);
      require(memory.recordOdometry(home),"Cannot initialize home");
      LocalPlanner initial(grid,p);auto first=initial.plan(home);
      if(first.success)++initial_successes;
      planner_msgs::msg::Graph message;initial.graphMessage(message);
      auto gain=[&](const StateVec& state){return initial.gain(state);};
      memory.absorbLocal(message,gain);int frontiers=memory.frontierCount();
      const auto update=memory.lastFrontierUpdate();
      std::vector<StateVec> traveled{home,{-3,0,0,0},{-4,0,0,0},{-4.5,-1,0,0},
        {-4.5,-2,0,0},{-4.5,-3,0,0},corner};
      require(independentlySafe(traveled,*grid),"Outward route intersects obstacle");
      for(const auto& state:traveled)require(memory.recordOdometry(state),"Outward odometry rejected");
      LocalPlanner local(grid,p);auto stalled=local.plan(corner);
      if(stalled.reason=="no_gain")++local_stalls;
      local.graphMessage(message);memory.absorbLocal(message,gain);
      auto before=std::chrono::steady_clock::now();auto global=memory.selectFrontier(corner,gain);
      double milliseconds=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-before).count();
      if(global.success){++recovered;require(independentlySafe(global.path,*grid),"Global route collision");}
      auto returning=memory.homing(corner);
      require(returning.success&&independentlySafe(returning.path,*grid),"Homing failed or collided");++homes;
      require((returning.path.front()-corner).norm()<1e-9&&(returning.path.back()-home).norm()<1e-9,"Homing endpoints differ");
      for(const auto& pair:{std::make_pair(std::string("global"),global.path),std::make_pair(std::string("homing"),returning.path)})
        for(std::size_t i=0;i<pair.second.size();++i)paths<<seed<<","<<pair.first<<","<<i<<","<<pair.second[i].x()<<","<<pair.second[i].y()<<","<<pair.second[i].z()<<"\n";
      // Introduce a complete horizontal barrier between current pose and home.
      for(int x=0;x<40;++x)for(int z=0;z<12;++z)grid->set({x,grid->index({0,-2.5,0}).y(),z},VoxelStatus::Occupied);
      memory.updateMap(grid);auto rejected=memory.homing(corner);
      require(!rejected.success&&rejected.reason=="global_target_unreachable","Blocked homing was not rejected");++blocked;
      csv<<seed<<","<<first.success<<","<<update.candidates<<","<<update.clusters<<","<<update.near_existing<<","<<update.near_visited<<","<<update.invalid_paths<<","<<frontiers<<","<<stalled.reason<<","<<global.success<<","<<global.reason<<","<<global.distance<<","<<milliseconds<<","<<returning.success<<","<<returning.distance<<","<<rejected.reason<<",1\n";
    }
    std::ofstream summary(output/"summary.json");
    summary<<"{\n  \"status\": \"passed\",\n  \"trials\": 20,\n  \"initial_local_successes\": "<<initial_successes
      <<",\n  \"local_no_gain_cases\": "<<local_stalls<<",\n  \"global_recoveries\": "<<recovered
      <<",\n  \"successful_homing_cases\": "<<homes<<",\n  \"blocked_homing_rejections\": "<<blocked
      <<",\n  \"independent_collision_checks_passed\": true,\n  \"motion_model\": \"prescribed safe odometry to a dead-end; no robot dynamics\"\n}\n";
    std::cout<<"20 seeds: "<<local_stalls<<" local no_gain, "<<recovered<<" global recoveries, "<<homes<<" homing, "<<blocked<<" blocked-route rejections\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<std::endl;return 1;}
}
