// Differential core checks for explicit immutable global terrain-query reuse.
#include "gbplanner_local/global_memory.hpp"
#include "gbplanner_local/sparse_voxel_map.hpp"
#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <vector>

using namespace gbplanner2;
namespace {
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
struct Calls {std::uint64_t simplex{0},cached_simplex{0},stencil_hits{0},stencil_lookups{0};};
class CountingMap final:public MapQuery {
 public:
  explicit CountingMap(std::shared_ptr<SparseVoxelMap> source):source(std::move(source)){}
  double resolution()const override{return source->resolution();}
  Eigen::Vector3d minimum()const override{return source->minimum();}
  Eigen::Vector3d maximum()const override{return source->maximum();}
  VoxelStatus voxel(const Eigen::Vector3d& p)const override{return source->voxel(p);}
  VoxelStatus box(const Eigen::Vector3d& p,const Eigen::Vector3d& size)const override{return source->box(p,size);}
  VoxelStatus path(const Eigen::Vector3d& a,const Eigen::Vector3d& b,const Eigen::Vector3d& size)const override{
    return source->path(a,b,size);
  }
  void ray(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
      const std::function<bool(const Eigen::Vector3d&,VoxelStatus)>& visit)const override{source->ray(a,b,visit);}
  std::optional<TsdfDistance> tsdfDistance(const Eigen::Vector3d& p)const override{return source->tsdfDistance(p);}
  std::optional<TsdfDistance> tsdfDistanceSimplex(const Eigen::Vector3d& p)const override{
    ++calls.simplex;return source->tsdfDistanceSimplex(p);
  }
  std::optional<TsdfDistance> tsdfDistanceSimplexCached(
      const Eigen::Vector3d& p,TsdfQueryContext& context)const override{
    ++calls.cached_simplex;const auto hits=context.hits,lookups=context.lookups;
    const auto value=source->tsdfDistanceSimplexCached(p,context);
    calls.stencil_hits+=context.hits-hits;calls.stencil_lookups+=context.lookups-lookups;return value;
  }
  std::shared_ptr<SparseVoxelMap> source;
  mutable Calls calls;
};
Parameters parameters(){
  Parameters p;p.ground_robot=true;p.robot_size={.1,.1,.1};p.overshoot=0.;
  p.edge_min=.15;p.edge_max=.8;p.neighbor_radius=.8;p.neighbor_min=.15;p.neighbor_max=.8;
  p.ground.max_ground_height=.8;p.ground.robot_height=.6;
  p.ground.max_inclination=.9;p.ground.max_negative_inclination=.9;
  p.ground.interpolate_projection_distance=true;
  p.ground.tsdf_projection_mode=GroundTsdfProjectionMode::ZeroCrossingSimplex;
  p.refinement.enabled=false;p.tangent_yaw=false;p.validate();return p;
}
std::shared_ptr<SparseVoxelMap> flat(){
  auto map=std::make_shared<SparseVoxelMap>(.1);map->reserve(24000);
  for(int z=-6;z<16;++z)for(int y=-26;y<18;++y)for(int x=-6;x<18;++x){
    const Eigen::Vector3i index(x,y,z);const double height=map->center(index).z();
    map->setObservation(index,height<0.?VoxelStatus::Occupied:VoxelStatus::Free,height,2.);
  }
  return map;
}
planner_msgs::msg::Graph snapshot(GlobalMemory& memory){
  planner_msgs::msg::Graph message;memory.graphMessage(message);return message;
}
void compare(GlobalMemory& baseline,GlobalMemory& reused){
  require(snapshot(baseline)==snapshot(reused),"Global graph changed under immutable query reuse");
  const auto& a=baseline.lastMapEdgeUpdate();const auto& b=reused.lastMapEdgeUpdate();
  require(a.invalidated==b.invalidated&&a.restored==b.restored&&a.pending==b.pending,
          "Map-edge invalidation/restoration differs under query reuse");
}
int exactId(const GlobalMemory& memory,const StateVec& state){
  for(const auto& item:memory.graph().vertices_map_)
    if((item.second->state.array()==state.array()).all())return item.first;
  throw std::runtime_error("Fixture waypoint was changed");
}
bool linked(const GlobalMemory& memory,int from,int to){
  const auto found=memory.graph().edge_map_.find(from);
  if(found!=memory.graph().edge_map_.end())for(const auto& item:found->second)if(item.first==to)return true;
  return false;
}
void sameRoute(const GlobalResult& a,const GlobalResult& b){
  require(a.success==b.success&&a.reason==b.reason&&a.target==b.target&&
          a.distance==b.distance&&a.path.size()==b.path.size(),"Global route metadata/cost changed");
  for(std::size_t i=0;i<a.path.size();++i)
    require((a.path[i].array()==b.path[i].array()).all(),"Global route XYZ/yaw changed");
}
double routes(GlobalMemory& baseline,GlobalMemory& reused,const std::vector<StateVec>& path){
  const auto gain=[](const StateVec&){return VolumetricGain{};};
  const auto forward=baseline.selectFrontier(path.front(),gain,exactId(baseline,path.back()));
  const auto cached_forward=reused.selectFrontier(path.front(),gain,exactId(reused,path.back()));
  require(forward.success,"Reference destination is not reachable");sameRoute(forward,cached_forward);
  const auto backward=baseline.homing(path.back()),cached_backward=reused.homing(path.back());
  require(backward.success,"Reference home is not reachable");sameRoute(backward,cached_backward);
  compare(baseline,reused);return backward.distance;
}
}

int main(){try{
  const auto p=parameters();auto backing=flat();
  auto plain_map=std::make_shared<CountingMap>(backing),cached_map=std::make_shared<CountingMap>(backing);
  GlobalMemory plain(p),cached(p);
  plain.updateMap(plain_map);cached.updateMap(cached_map,TerrainQueryReuse::ImmutableSnapshot);
  TerrainProjector terrain(plain_map,p.groundParameters());std::vector<StateVec> reference;
  for(const auto& xy:std::vector<Eigen::Vector2d>{{0.,0.},{0.,1.},{1.,1.},{1.,0.},{.5,0.},{.5,-1.},{.5,-2.}}){
    const auto projected=terrain.project(StateVec(xy.x(),xy.y(),.8,.1+reference.size()));
    require(projected.status==GroundStatus::Supported,"Fixture lacks observed terrain");
    reference.push_back(projected.projected_state);
  }
  for(auto* memory:{&plain,&cached}){
    require(memory->setHomingPosition(reference.front()),"Could not establish fixture home");
    require(memory->addReferencePath(reference),"Could not add fixed reference path");
  }
  compare(plain,cached);
  plain_map->calls={};cached_map->calls={};
  plain.updateMap(plain_map);cached.updateMap(cached_map,TerrainQueryReuse::ImmutableSnapshot);
  compare(plain,cached);const Calls old_calls=plain_map->calls,new_calls=cached_map->calls;
  require(old_calls.simplex>0&&old_calls.cached_simplex==0&&new_calls.cached_simplex>0&&
          new_calls.stencil_hits>0,"Explicit reuse/default-disabled paths were not exercised");
  const double initial=routes(plain,cached,reference);
  const int home=exactId(plain,reference.front()),junction=exactId(plain,reference[4]);
  require(linked(plain,home,junction),"Fixture lacks the short ground connection");

  // End the immutable interval, mutate this SAME backing object, and notify
  // each GlobalMemory before making any query. Pointer identity is unchanged.
  const auto obstacle=backing->index({.25,0.,.6});backing->set(obstacle,VoxelStatus::Occupied);
  plain.updateMap(plain_map);cached.updateMap(cached_map,TerrainQueryReuse::ImmutableSnapshot);
  compare(plain,cached);const auto blocked=plain.lastMapEdgeUpdate();
  require(blocked.invalidated>0&&blocked.pending>0&&!linked(cached,home,junction),
          "A previous free verdict survived same-pointer map replacement");
  const double detour=routes(plain,cached,reference);
  require(detour>initial+.5,"Blocking the shortcut did not change the route");

  backing->set(obstacle,VoxelStatus::Free);
  plain.updateMap(plain_map);cached.updateMap(cached_map,TerrainQueryReuse::ImmutableSnapshot);
  compare(plain,cached);const auto restored=plain.lastMapEdgeUpdate();
  require(restored.restored>0&&linked(cached,home,junction),
          "A previous rejecting verdict survived same-pointer map recovery");
  const double recovered=routes(plain,cached,reference);
  require(std::abs(recovered-initial)<1e-12,"Restored map did not restore the original route cost");

  // A caller can return to the original entry point. Its new projector must
  // not retain the prior opt-in, independently of the unchanged map pointer.
  plain.updateMap(plain_map);cached_map->calls={};cached.updateMap(cached_map);
  require(cached_map->calls.cached_simplex==0&&cached_map->calls.simplex>0,
          "Legacy updateMap inherited reuse from a previous explicit call");
  compare(plain,cached);
  std::cout<<"{\"status\":\"passed\",\"graph_exact\":true,\"map_edge_update_equal\":true,"
      "\"bidirectional_routes_exact\":true,\"same_pointer_refresh\":true,\"legacy_default_disabled\":true,"
      "\"baseline_simplex_calls\":"<<old_calls.simplex<<",\"cached_simplex_calls\":"<<new_calls.cached_simplex
      <<",\"stencil_hits\":"<<new_calls.stencil_hits<<",\"stencil_lookups\":"<<new_calls.stencil_lookups
      <<",\"invalidated\":"<<blocked.invalidated<<",\"restored\":"<<restored.restored<<"}\n";
  return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
