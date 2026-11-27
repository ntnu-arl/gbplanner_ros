// Offline repeatability and synthetic closed-loop experiments, not a dynamics simulator.
#include "gbplanner_local/local_planner.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

using namespace gbplanner2;
namespace {
std::shared_ptr<VoxelMap> scene(const std::string& name,bool ground_truth=false) {
  auto map=std::make_shared<VoxelMap>(0.25,Eigen::Vector3d(-5,-5,-1.5),Eigen::Vector3i(40,40,12));
  map->fill(VoxelStatus::Free);
  for(int z=0;z<12;++z)for(int y=0;y<40;++y)for(int x=0;x<40;++x){
    Eigen::Vector3i i(x,y,z);auto p=map->center(i);
    if(!ground_truth && p.x()>=1.5)map->set(i,VoxelStatus::Unknown);
    if(name!="open" && std::abs(p.x())<0.25 &&
       (name=="sealed"||std::abs(p.y())>1.0))map->set(i,VoxelStatus::Occupied);
  }
  return map;
}
// Independent footprint probe; the planner uses analytic segment/AABB intersections.
bool verify(const std::vector<StateVec>& path,const VoxelMap& map,const Eigen::Vector3d& size) {
  for(std::size_t n=1;n<path.size();++n){
    Eigen::Vector3d a=path[n-1].head<3>(),delta=path[n].head<3>()-a;
    int steps=std::max(1,int(std::ceil(delta.norm()/0.025)));
    for(int k=0;k<=steps;++k)for(int ix=-2;ix<=2;++ix)for(int iy=-2;iy<=2;++iy)for(int iz=-2;iz<=2;++iz){
      Eigen::Vector3d p=a+delta*(double(k)/steps)+size.cwiseProduct(Eigen::Vector3d(ix,iy,iz))*0.25;
      if(map.voxel(p)!=VoxelStatus::Free)return false;
    }
  }
  return true;
}
void saveSlice(const VoxelMap& map,const std::filesystem::path& file){
  std::ofstream out(file);out<<"x,y,status\n";
  int z=map.index({0,0,0}).z();
  for(int y=0;y<map.dimensions().y();++y)for(int x=0;x<map.dimensions().x();++x){
    auto p=map.center({x,y,z});out<<p.x()<<","<<p.y()<<","<<int(map.voxel(p))<<"\n";
  }
}
// Ideal lidar oracle: only observed voxels enter the planner's belief map.
// Occlusion comes from a separate known world. Motion is kinematic.
int observe(VoxelMap& belief,const VoxelMap& truth,const StateVec& state){
  for(double v=-M_PI/3;v<=M_PI/3;v+=M_PI/60)for(double h=-M_PI;h<M_PI;h+=M_PI/60){
    Eigen::Vector3d direction(std::cos(v)*std::cos(h),std::cos(v)*std::sin(h),std::sin(v));
    truth.ray(state.head<3>(),state.head<3>()+3.0*direction,[&](auto p,auto status){
      auto index=truth.index(p);if(!truth.inside(index))return false;
      belief.set(index,status);return status!=VoxelStatus::Occupied;
    });
  }
  int count=0;
  for(int z=0;z<truth.dimensions().z();++z)for(int y=0;y<truth.dimensions().y();++y)for(int x=0;x<truth.dimensions().x();++x)
    if(belief.voxel(belief.center({x,y,z}))!=VoxelStatus::Unknown)++count;
  return count;
}
}
int main(int argc,char** argv){
 try {
  const std::filesystem::path output=argc>1?argv[1]:"results/local_experiments";
  std::filesystem::create_directories(output);
  std::ofstream csv(output/"benchmark.csv");
  csv<<std::setprecision(12)<<"scene,mode,seed,success,reason,vertices,edges,attempts,gain,path_length_m,build_ms,evaluate_ms,collision_free\n";
  std::ofstream paths(output/"paths.csv");paths<<std::setprecision(12)<<"scene,mode,seed,index,x,y,z\n";
  int successes=0,runs=0;
  for(const auto& name:{"open","doorway","sealed"}) {
    auto map=scene(name);saveSlice(*map,output/(std::string(name)+"_map.csv"));
    for(bool graph:{false,true})for(uint32_t seed=0;seed<20;++seed){
      Parameters p;p.seed=seed;p.graph_mode=graph;
      LocalPlanner planner(map,p);auto result=planner.plan(StateVec(-2,0,0,0));
      bool safe=verify(result.path,*map,p.robot_size);
      if(!safe)throw std::runtime_error("Independent collision check failed");
      if(result.success)++successes;
      ++runs;
      csv<<name<<","<<(graph?"graph":"tree")<<","<<seed<<","<<result.success<<","<<result.reason<<","<<result.vertices<<","<<result.edges<<","<<result.attempts<<","<<result.gain<<","<<result.path_length<<","<<result.build_ms<<","<<result.evaluate_ms<<","<<safe<<"\n";
      for(std::size_t i=0;i<result.path.size();++i)paths<<name<<","<<(graph?"graph":"tree")<<","<<seed<<","<<i<<","<<result.path[i].x()<<","<<result.path[i].y()<<","<<result.path[i].z()<<"\n";
    }
  }
  auto truth=scene("doorway",true);
  auto belief=std::make_shared<VoxelMap>(0.25,truth->minimum(),truth->dimensions());
  StateVec state(-2,0,0,0);int initial_known=observe(*belief,*truth,state);
  saveSlice(*belief,output/"exploration_initial_map.csv");
  std::ofstream history(output/"exploration.csv");
  history<<std::setprecision(12)<<"iteration,x,y,z,known_before,known_after,known_fraction,planned_length_m,executed_length_m,gain,planning_ms,reason\n";
  std::ofstream trajectory(output/"trajectory.csv");trajectory<<std::setprecision(12)<<"index,x,y,z\n0,"<<state.x()<<","<<state.y()<<","<<state.z()<<"\n";
  int known=initial_known,executed=0;double traveled=0;
  std::string stop="iteration_limit";
  for(int iteration=0;iteration<30;++iteration){
    Parameters p;p.seed=42+iteration;p.max_vertices=150;p.max_attempts=6000;
    LocalPlanner planner(belief,p);auto result=planner.plan(state);int before=known;
    double step_distance=0;
    if(result.success && result.path.size()>1){
      // Receding horizon: execute only the first edge, observe, then rebuild.
      std::vector<StateVec> first{result.path[0],result.path[1]};
      if(!verify(first,*belief,p.robot_size)||!verify(first,*truth,p.robot_size))
        throw std::runtime_error("Closed-loop motion failed independent collision check");
      step_distance=(first[1]-first[0]).head<3>().norm();state=first[1];traveled+=step_distance;
      ++executed;trajectory<<executed<<","<<state.x()<<","<<state.y()<<","<<state.z()<<"\n";
      known=observe(*belief,*truth,state);
      if(known<before)throw std::runtime_error("Known-map cells decreased");
    }else stop=result.reason;
    history<<iteration<<","<<state.x()<<","<<state.y()<<","<<state.z()<<","<<before<<","<<known<<","<<double(known)/belief->size()<<","<<result.path_length<<","<<step_distance<<","<<result.gain<<","<<result.build_ms+result.evaluate_ms<<","<<result.reason<<"\n";
    if(!result.success)break;
  }
  saveSlice(*belief,output/"exploration_final_map.csv");
  saveSlice(*truth,output/"exploration_truth_map.csv");
  std::ofstream summary(output/"experiment_summary.json");
  summary<<"{\n  \"benchmark_runs\": "<<runs<<",\n  \"successful_plans\": "<<successes
    <<",\n  \"all_output_paths_collision_free\": true,\n  \"closed_loop\": {\n    \"type\": \"synthetic ideal lidar and kinematic first-edge execution\",\n    \"initial_known_cells\": "<<initial_known
    <<",\n    \"final_known_cells\": "<<known<<",\n    \"total_cells\": "<<belief->size()
    <<",\n    \"executed_steps\": "<<executed<<",\n    \"distance_m\": "<<traveled
    <<",\n    \"stop_reason\": \""<<stop<<"\"\n  }\n}\n";
  std::cout<<"Completed "<<runs<<" planning runs, "<<successes<<" successful plans; closed loop "<<executed<<" steps, known cells "<<initial_known<<" -> "<<known<<std::endl;
  return 0;
 }catch(const std::exception& error){std::cerr<<error.what()<<std::endl;return 1;}
}
