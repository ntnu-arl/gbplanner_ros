#include "gbplanner_local/local_planner.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

using namespace gbplanner2;
namespace {
class CountedMap final:public MapQuery {
 public:
  explicit CountedMap(std::shared_ptr<const VoxelMap> map):map_(std::move(map)){}
  double resolution()const override{return map_->resolution();}
  Eigen::Vector3d minimum()const override{return map_->minimum();}
  Eigen::Vector3d maximum()const override{return map_->maximum();}
  VoxelStatus voxel(const Eigen::Vector3d& point)const override{return map_->voxel(point);}
  VoxelStatus box(const Eigen::Vector3d& point,const Eigen::Vector3d& size)const override{return map_->box(point,size);}
  VoxelStatus path(const Eigen::Vector3d& from,const Eigen::Vector3d& to,const Eigen::Vector3d& size)const override{
    return map_->path(from,to,size);
  }
  void ray(const Eigen::Vector3d& from,const Eigen::Vector3d& to,
      const std::function<bool(const Eigen::Vector3d&,VoxelStatus)>& visit)const override{
    ++ray_calls_;map_->ray(from,to,visit);
  }
  std::size_t rayCalls()const{return ray_calls_;}
 private:
  std::shared_ptr<const VoxelMap> map_;
  mutable std::size_t ray_calls_{0};
};

std::shared_ptr<VoxelMap> doorway(){
  auto map=std::make_shared<VoxelMap>(.25,Eigen::Vector3d(-5,-5,-1.5),Eigen::Vector3i(40,40,12));
  map->fill(VoxelStatus::Free);
  for(int z=0;z<12;++z)for(int y=0;y<40;++y)for(int x=0;x<40;++x){
    const Eigen::Vector3i index(x,y,z);const auto point=map->center(index);
    if(point.x()>=1.5)map->set(index,VoxelStatus::Unknown);
    if(std::abs(point.x())<.25&&std::abs(point.y())>1.)map->set(index,VoxelStatus::Occupied);
  }
  return map;
}

bool feasible(const PlanResult& result,const VoxelMap& map,const Parameters& p){
  if(!result.success||result.path.size()<2)return false;
  for(const auto& state:result.path)
    if(map.box(state.head<3>()+p.center_offset,p.robot_size)!=VoxelStatus::Free)return false;
  for(std::size_t i=1;i<result.path.size();++i)
    if(map.path(result.path[i-1].head<3>()+p.center_offset,
                result.path[i].head<3>()+p.center_offset,p.robot_size)!=VoxelStatus::Free)return false;
  return true;
}

bool sameGeometry(const GraphManager& a,const GraphManager& b){
  if(a.vertices_map_.size()!=b.vertices_map_.size()||a.edge_map_!=b.edge_map_)return false;
  for(const auto& item:a.vertices_map_){
    const auto found=b.vertices_map_.find(item.first);
    if(found==b.vertices_map_.end()||item.second->state!=found->second->state)return false;
  }
  return true;
}

void writeRun(std::ostream& out,const PlanResult& result,const CountedMap& map){
  out<<"{\"success\":true,\"path_feasible\":true,\"vertices\":"<<result.vertices
      <<",\"edges\":"<<result.edges<<",\"gain_evaluations\":"<<result.gain_evaluations
      <<",\"reused_gains\":"<<result.reused_gains<<",\"ray_calls\":"<<map.rayCalls()
      <<",\"build_ms\":"<<result.build_ms<<",\"evaluate_ms\":"<<result.evaluate_ms
      <<",\"gain\":"<<result.gain<<",\"path_length_m\":"<<result.path_length
      <<",\"best_vertex\":"<<result.best_id<<",\"path\":[";
  for(std::size_t i=0;i<result.path.size();++i){
    if(i)out<<',';const auto& state=result.path[i];
    out<<'['<<state[0]<<','<<state[1]<<','<<state[2]<<','<<state[3]<<']';
  }
  out<<"]}";
}
}

int main(){
  try{
    const auto map=doorway();Parameters p;p.seed=42;p.max_vertices=300;p.max_edges=12000;p.max_attempts=12000;
    p.clustering_radius=.5;const StateVec root(-2,0,0,0);
    const auto baseline_map=std::make_shared<CountedMap>(map);
    LocalPlanner baseline(baseline_map,p);const auto off=baseline.plan(root);
    p.cluster_vertices_for_gain=true;
    const auto clustered_map=std::make_shared<CountedMap>(map);
    LocalPlanner clustered(clustered_map,p);const auto on=clustered.plan(root);
    if(!feasible(off,*map,p)||!feasible(on,*map,p))throw std::runtime_error("A selected path is not feasible");
    if(!sameGeometry(baseline.graph(),clustered.graph()))throw std::runtime_error("The graph geometry changed between runs");
    if(clustered_map->rayCalls()>=baseline_map->rayCalls()||on.gain_evaluations>=off.gain_evaluations||on.reused_gains<=0)
      throw std::runtime_error("Clustering did not reduce actual map ray calls");
    const auto saved=baseline_map->rayCalls()-clustered_map->rayCalls();
    std::cout<<std::setprecision(15)<<"{\"scene\":\"doorway\",\"seed\":42,\"clustering_radius_m\":0.5,\"same_graph\":true"
        <<",\"ray_calls_saved\":"<<saved<<",\"ray_reduction_fraction\":"<<double(saved)/baseline_map->rayCalls()
        <<",\"off\":";writeRun(std::cout,off,*baseline_map);
    std::cout<<",\"on\":";writeRun(std::cout,on,*clustered_map);std::cout<<"}\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
