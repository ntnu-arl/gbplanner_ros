// findPathToConnect's accepted-sample loop count and finite raw-draw budget.
#include "gbplanner_local/search_planner.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

using namespace gbplanner2;
namespace {
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}

// The real planner samples and builds its graph. A bounded acceptance gate
// isolates a complete failed sampleVertex batch without changing that sampler.
// A static wall blocks source -> target; source -> (4,0) -> target is free.
class SamplingMap final:public MapQuery {
 public:
  SamplingMap(int rejected_draws,bool blocked_edges):
      map_(.1,Eigen::Vector3d(-1.,-1.,-.5),Eigen::Vector3i(70,50,25)),
      rejected_draws_(rejected_draws),blocked_edges_(blocked_edges){
    map_.fill(VoxelStatus::Free);
    for(int z=0;z<25;++z)for(int y=0;y<50;++y)for(int x=0;x<70;++x){
      const Eigen::Vector3i index(x,y,z);const auto point=map_.center(index);
      if(std::abs(point.x()-2.)<.16&&std::abs(point.y()-1.)<.16)
        map_.set(index,VoxelStatus::Occupied);
    }
  }
  double resolution()const override{return map_.resolution();}
  Eigen::Vector3d minimum()const override{return map_.minimum();}
  Eigen::Vector3d maximum()const override{return map_.maximum();}
  VoxelStatus voxel(const Eigen::Vector3d& point)const override{return map_.voxel(point);}
  VoxelStatus box(const Eigen::Vector3d& point,const Eigen::Vector3d& size)const override{
    if(point.x()==4.&&point.y()==0.&&++sample_queries<=rejected_draws_)
      return VoxelStatus::Occupied;
    return map_.box(point,size);
  }
  VoxelStatus path(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
                   const Eigen::Vector3d& size)const override{
    return blocked_edges_?VoxelStatus::Occupied:map_.path(a,b,size);
  }
  void ray(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
           const std::function<bool(const Eigen::Vector3d&,VoxelStatus)>& visit)const override{
    map_.ray(a,b,visit);
  }
  mutable int sample_queries{0};
 private:
  VoxelMap map_;int rejected_draws_;bool blocked_edges_;
};

Parameters parameters(){
  Parameters p;p.local_min={-.5,-.5,-.1};p.local_max={4.5,2.5,.1};
  p.robot_size={.1,.1,.1};p.edge_min=.05;p.edge_max=5.;p.overshoot=0.;
  p.graph_mode=false;p.refinement.enabled=false;
  for(auto& axis:p.axis_sampling){axis.distribution="constant";axis.constant=0.;}
  p.axis_sampling[0].constant=4.;return p;
}
void print(const char* name,const SearchResult& result){
  std::cout<<'"'<<name<<"\":{\"success\":"<<(result.success?"true":"false")
    <<",\"attempts\":"<<result.attempts<<",\"sampling_rounds\":"<<result.sampling_rounds
    <<",\"empty_graph_cutoff_reached\":"<<(result.empty_graph_cutoff_reached?"true":"false")
    <<",\"vertices\":"<<result.vertices<<",\"edges\":"<<result.edges
    <<",\"path_length_m\":"<<result.path_length<<'}';
}
}

int main(){
  try{
    const StateVec source(0.,0.,.6,0.),target(4.,2.,.6,0.);const auto p=parameters();
    SearchParameters options;options.max_attempts=21;options.empty_graph_cutoff=7;
    options.max_vertices=10;options.max_edges=100;options.target_radius=.1;
    const auto isolated=SearchPlanner(std::make_shared<SamplingMap>(0,true),p,options).search(source,target);
    require(!isolated.success&&isolated.vertices==1&&isolated.attempts==8&&
            isolated.sampling_rounds==8&&isolated.empty_graph_cutoff_reached,
            "Search cutoff=7 did not stop after the eighth failed expansion");

    const auto growing=SearchPlanner(std::make_shared<SamplingMap>(0,false),p,options).search(source,target);
    require(!growing.success&&growing.vertices==2&&growing.edges==1&&growing.attempts==21&&
            growing.sampling_rounds==21&&!growing.empty_graph_cutoff_reached,
            "An established graph stopped on later duplicate samples");

    options.empty_graph_cutoff=1;options.max_attempts=1004;options.target_radius=2.1;
    const auto late=SearchPlanner(std::make_shared<SamplingMap>(1000,false),p,options).search(source,target);
    require(late.success&&late.exact_target&&late.attempts==1004&&late.sampling_rounds==4&&
            !late.empty_graph_cutoff_reached&&late.vertices==3&&std::abs(late.path_length-6.)<1e-8,
            "A rejected batch consumed the search cutoff before a valid detour could grow");

    const auto late_blocked=SearchPlanner(std::make_shared<SamplingMap>(1000,true),p,options).search(source,target);
    require(!late_blocked.success&&late_blocked.attempts==1002&&late_blocked.sampling_rounds==2&&
            late_blocked.empty_graph_cutoff_reached&&late_blocked.vertices==1,
            "A rejected batch incorrectly advanced the post-increment search cutoff");

    options.max_attempts=2001;
    const auto rejected=SearchPlanner(std::make_shared<SamplingMap>(2001,true),p,options).search(source,target);
    require(!rejected.success&&rejected.attempts==2001&&rejected.sampling_rounds==0&&
            !rejected.empty_graph_cutoff_reached&&rejected.vertices==1,
            "Completely rejected batches did not retain the raw-draw hard limit");

    auto free_map=std::make_shared<VoxelMap>(.1,Eigen::Vector3d(-1.,-1.,-.5),Eigen::Vector3i(70,50,25));
    free_map->fill(VoxelStatus::Free);
    const auto direct=SearchPlanner(free_map,p,options).search(source,target);
    require(direct.success&&direct.exact_target&&direct.attempts==0&&direct.sampling_rounds==0&&
            !direct.empty_graph_cutoff_reached,"Direct search unexpectedly entered random sampling");

    std::cout<<std::setprecision(15)<<"{\"status\":\"passed\",\"sample_vertex_draw_limit\":1000,";
    print("isolated",isolated);std::cout<<',';print("growing",growing);std::cout<<',';
    print("late_accepted",late);std::cout<<',';print("late_failed_expansion",late_blocked);std::cout<<',';
    print("all_rejected",rejected);std::cout<<',';print("direct",direct);
    std::cout<<",\"raw_attempt_budget_preserved\":true}\n";return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
