// Link against the production planner library; no ROS executor or simulation.
// Checks ordinary/fixed append, strict producer prechecks, directional slopes,
// and same-pointer map replacement without rounding graph costs to Graph.msg.
#include "gbplanner_local/global_memory.hpp"
#include "gbplanner_local/sparse_voxel_map.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <tuple>

using namespace gbplanner2;
namespace {
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
bool bits(double a,double b){return std::memcmp(&a,&b,sizeof(double))==0;}
void sameState(const StateVec& a,const StateVec& b){
  for(int i=0;i<4;++i)require(bits(a[i],b[i]),"Graph/route state bits changed");
}
void sameGain(const VolumetricGain& a,const VolumetricGain& b){
  require(bits(a.gain,b.gain)&&bits(a.accumulative_gain,b.accumulative_gain)&&
      a.num_unknown_voxels==b.num_unknown_voxels&&a.num_free_voxels==b.num_free_voxels&&
      a.num_occupied_voxels==b.num_occupied_voxels&&a.num_unknown_surf_voxels==b.num_unknown_surf_voxels&&
      a.is_frontier==b.is_frontier&&a.unseen_voxel_hash_keys==b.unseen_voxel_hash_keys,
      "Vertex gain changed");
}
void sameGraph(const GlobalMemory& a,const GlobalMemory& b){
  const auto& x=a.graph();const auto& y=b.graph();
  require(x.vertices_map_.size()==y.vertices_map_.size()&&x.edge_map_.size()==y.edge_map_.size()&&
      a.edgeCount()==b.edgeCount(),"Graph topology size changed");
  for(const auto& item:x.vertices_map_){
    const auto found=y.vertices_map_.find(item.first);require(found!=y.vertices_map_.end(),"Vertex ID changed");
    const auto& u=*item.second;const auto& v=*found->second;sameState(u.state,v.state);sameGain(u.vol_gain,v.vol_gain);
    require(u.id==v.id&&bits(u.distance,v.distance)&&bits(u.dm,v.dm)&&
        u.type==v.type&&u.is_leaf_vertex==v.is_leaf_vertex&&u.is_hanging==v.is_hanging&&
        u.cluster_id==v.cluster_id&&u.semantic_class==v.semantic_class&&
        (u.parent?u.parent->id:-1)==(v.parent?v.parent->id:-1)&&u.children.size()==v.children.size(),
        "Vertex metadata or parent changed");
    for(std::size_t i=0;i<u.children.size();++i)
      require(u.children[i]->id==v.children[i]->id,"Child insertion order changed");
  }
  for(const auto& item:x.edge_map_){
    const auto found=y.edge_map_.find(item.first);require(found!=y.edge_map_.end(),"Adjacency source changed");
    auto u=item.second,v=found->second;std::sort(u.begin(),u.end());std::sort(v.begin(),v.end());
    require(u.size()==v.size(),"Adjacency count changed");
    for(std::size_t i=0;i<u.size();++i)
      require(u[i].first==v[i].first&&bits(u[i].second,v[i].second),"Internal double edge weight changed");
  }
  require(a.snapshotRevision()==b.snapshotRevision()&&a.visitRevision()==b.visitRevision()&&
      a.visitedStates().size()==b.visitedStates().size(),"Graph/visit revision changed");
  for(std::size_t i=0;i<a.visitedStates().size();++i)sameState(a.visitedStates()[i],b.visitedStates()[i]);
  sameState(a.lastVisitedMarker(),b.lastVisitedMarker());
  const auto& m=a.lastMapEdgeUpdate();const auto& n=b.lastMapEdgeUpdate();
  require(std::tie(m.invalidated,m.restored,m.pending)==std::tie(n.invalidated,n.restored,n.pending),
          "Map invalidation/restoration changed");
  const auto& f=a.lastFrontierUpdate();const auto& g=b.lastFrontierUpdate();
  require(std::tie(f.candidates,f.clusters,f.near_existing,f.near_visited,f.invalid_paths,f.added)==
      std::tie(g.candidates,g.clusters,g.near_existing,g.near_visited,g.invalid_paths,g.added),
      "Frontier admission changed");
}
void sameRoute(const GlobalResult& a,const GlobalResult& b){
  require(a.success==b.success&&a.reason==b.reason&&a.target==b.target&&
      bits(a.distance,b.distance)&&bits(a.score,b.score)&&a.path.size()==b.path.size(),
      "Route status or double cost changed");
  for(std::size_t i=0;i<a.path.size();++i)sameState(a.path[i],b.path[i]);
}
VolumetricGain frontier(const StateVec&){
  VolumetricGain gain;gain.is_frontier=true;gain.gain=10.;gain.num_unknown_voxels=10;return gain;
}
Parameters parameters(){
  Parameters p;p.ground_robot=true;p.robot_size={.1,.1,.1};p.overshoot=0.;
  p.edge_min=.05;p.edge_max=1.;p.neighbor_radius=1.;p.neighbor_min=.05;p.neighbor_max=1.;
  p.ground.max_ground_height=.8;p.ground.robot_height=.6;
  p.ground.max_inclination=.9;p.ground.max_negative_inclination=.9;
  p.refinement.enabled=false;p.tangent_yaw=false;return p;
}
class CountMap final:public MapQuery {
 public:
  explicit CountMap(std::shared_ptr<const MapQuery> map):source(std::move(map)){}
  double resolution()const override{return source->resolution();}
  Eigen::Vector3d minimum()const override{return source->minimum();}
  Eigen::Vector3d maximum()const override{return source->maximum();}
  VoxelStatus voxel(const Eigen::Vector3d& p)const override{return source->voxel(p);}
  VoxelStatus box(const Eigen::Vector3d& p,const Eigen::Vector3d& size)const override{return source->box(p,size);}
  VoxelStatus path(const Eigen::Vector3d& a,const Eigen::Vector3d& b,const Eigen::Vector3d& size)const override{
    ++paths;return source->path(a,b,size);
  }
  void ray(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
      const std::function<bool(const Eigen::Vector3d&,VoxelStatus)>& visit)const override{source->ray(a,b,visit);}
  std::optional<TsdfDistance> tsdfDistance(const Eigen::Vector3d& p)const override{return source->tsdfDistance(p);}
  std::optional<TsdfDistance> tsdfDistanceSimplex(const Eigen::Vector3d& p)const override{return source->tsdfDistanceSimplex(p);}
  std::optional<TsdfDistance> tsdfDistanceSimplexCached(const Eigen::Vector3d& p,TsdfQueryContext& context)const override{
    return source->tsdfDistanceSimplexCached(p,context);
  }
  std::shared_ptr<const MapQuery> source;mutable std::size_t paths{0};
};
std::shared_ptr<SparseVoxelMap> flat(){
  auto map=std::make_shared<SparseVoxelMap>(.1);map->reserve(120000);
  for(int z=-6;z<16;++z)for(int y=-36;y<36;++y)for(int x=-36;x<36;++x){
    const Eigen::Vector3i index(x,y,z);const double height=map->center(index).z();
    map->setObservation(index,height<0.?VoxelStatus::Occupied:VoxelStatus::Free,height,2.);
  }
  return map;
}
void setup(GlobalMemory& memory,const std::shared_ptr<const MapQuery>& map,
           TerrainQueryReuse reuse,const StateVec& home){
  memory.updateMap(map,reuse);require(memory.setHomingPosition(home),"Cannot establish fixture home");
}
void update(GlobalMemory& a,GlobalMemory& b,const std::shared_ptr<const MapQuery>& map){
  a.updateMap(map);b.updateMap(map,TerrainQueryReuse::ImmutableSnapshot);sameGraph(a,b);
}
std::pair<std::size_t,std::size_t> fixedEdges(){
  auto p=parameters();p.ground.interpolate_projection_distance=true;
  p.ground.tsdf_projection_mode=GroundTsdfProjectionMode::ZeroCrossingSimplex;
  auto map=std::make_shared<CountMap>(flat());TerrainProjector terrain(map,p.groundParameters());
  std::vector<StateVec> path;
  for(const auto& xy:std::vector<Eigen::Vector2d>{{0.,0.},{.6,0.},{1.2,.3},{1.2,1.}}){
    const auto projected=terrain.project(StateVec(xy.x(),xy.y(),.8,.2+path.size()));
    require(projected.status==GroundStatus::Supported,"Fixed-path terrain is unsupported");
    path.push_back(projected.projected_state);
  }
  GlobalMemory old(p),retained(p);
  setup(old,map,TerrainQueryReuse::Disabled,path.front());
  setup(retained,map,TerrainQueryReuse::ImmutableSnapshot,path.front());
  int old_target=-1,new_target=-1;
  require(old.addReferencePath(path,&old_target)&&retained.addReferencePath(path,&new_target)&&
          old_target==new_target,"Fixed reference insertion changed");sameGraph(old,retained);
  map->paths=0;
  const auto forward=old.selectFrontier(path.front(),frontier,old_target);
  const auto backward=old.homing(path.back());const auto old_paths=map->paths;
  map->paths=0;
  const auto cached_forward=retained.selectFrontier(path.front(),frontier,new_target);
  const auto cached_backward=retained.homing(path.back());const auto new_paths=map->paths;
  require(forward.success&&backward.success,"Fixed bidirectional route is unavailable");
  sameRoute(forward,cached_forward);sameRoute(backward,cached_backward);sameGraph(old,retained);
  require(new_paths<old_paths,"Retained append edges did not reduce repeated swept-body checks");
  return {old_paths,new_paths};
}
struct SuspensionCalls {
  std::size_t default_blocked{0},disabled_blocked{0},retained_blocked{0};
  std::size_t retained_pending{0},retained_restored{0};
};
SuspensionCalls newlySuspendedFixedEdge(){
  auto p=parameters();p.ground.interpolate_projection_distance=true;
  p.ground.tsdf_projection_mode=GroundTsdfProjectionMode::ZeroCrossingSimplex;
  auto backing=flat();
  auto default_map=std::make_shared<CountMap>(backing),disabled_map=std::make_shared<CountMap>(backing),
       retained_map=std::make_shared<CountMap>(backing);
  TerrainProjector terrain(default_map,p.groundParameters());
  std::vector<StateVec> path;
  for(double x:{0.,.6}){
    const auto projected=terrain.project(StateVec(x,0.,.8,.2));
    require(projected.status==GroundStatus::Supported,"Suspension fixture is unsupported");
    path.push_back(projected.projected_state);
  }
  GlobalMemory legacy(p),disabled(p),retained(p);
  legacy.updateMap(default_map);
  disabled.updateMap(disabled_map,TerrainQueryReuse::Disabled);
  retained.updateMap(retained_map,TerrainQueryReuse::ImmutableSnapshot);
  for(auto* memory:{&legacy,&disabled,&retained}){
    require(memory->setHomingPosition(path.front())&&memory->addReferencePath(path),
            "Cannot establish suspension fixture");
    require(memory->graph().vertices_map_.size()==2&&memory->edgeCount()==1,
            "Suspension fixture must have exactly one fixed bidirectional edge");
  }
  sameGraph(legacy,disabled);sameGraph(legacy,retained);
  const auto refresh=[&]{
    default_map->paths=disabled_map->paths=retained_map->paths=0;
    legacy.updateMap(default_map);
    disabled.updateMap(disabled_map,TerrainQueryReuse::Disabled);
    retained.updateMap(retained_map,TerrainQueryReuse::ImmutableSnapshot);
    sameGraph(legacy,disabled);sameGraph(legacy,retained);
  };
  const int column=backing->index({.3,0.,0.}).x();
  const auto wall=[&](bool blocked){
    for(int y=-36;y<36;++y)for(int z=1;z<16;++z)
      backing->set({column,y,z},blocked?VoxelStatus::Occupied:VoxelStatus::Free);
  };
  // End the immutable interval, then replace contents of the SAME map object.
  // The fixed-edge occupancy sweep always runs before terrain-query caches.
  // Four versus two sweeps therefore isolates this update's skipped second
  // bidirectional recheck, rather than ordinary projection-cache hits.
  wall(true);refresh();
  require(legacy.edgeCount()==0&&legacy.lastMapEdgeUpdate().invalidated==1&&
          legacy.lastMapEdgeUpdate().pending==1&&legacy.lastMapEdgeUpdate().restored==0,
          "New obstruction did not suspend the fixed link");
  SuspensionCalls calls{default_map->paths,disabled_map->paths,retained_map->paths,0,0};
  require(calls.default_blocked==4&&calls.disabled_blocked==4&&calls.retained_blocked==2,
          "Newly suspended edge was not reused only in explicit immutable mode");

  // A previously suspended link has no newly checked entry after cache reset;
  // it must run both directions even when the same obstruction persists.
  refresh();calls.retained_pending=retained_map->paths;
  require(default_map->paths==2&&disabled_map->paths==2&&calls.retained_pending==2&&
          retained.lastMapEdgeUpdate().invalidated==0&&retained.lastMapEdgeUpdate().pending==1,
          "Old suspended link skipped its new-map check");

  wall(false);refresh();calls.retained_restored=retained_map->paths;
  require(default_map->paths==2&&disabled_map->paths==2&&calls.retained_restored==2&&
          retained.edgeCount()==1&&retained.lastMapEdgeUpdate().restored==1&&
          retained.lastMapEdgeUpdate().pending==0,"Cleared wall did not restore the fixed link");
  const auto forward=legacy.selectFrontier(path.front(),frontier,1),backward=legacy.homing(path.back());
  require(forward.success&&backward.success,"Restored fixed link is not bidirectionally usable");
  sameRoute(forward,retained.selectFrontier(path.front(),frontier,1));
  sameRoute(backward,retained.homing(path.back()));sameGraph(legacy,retained);
  return calls;
}
std::shared_ptr<VoxelMap> ramp(){
  auto map=std::make_shared<VoxelMap>(.05,Eigen::Vector3d(-2.,-1.,-1.),Eigen::Vector3i(80,40,80));
  for(int z=0;z<80;++z)for(int y=0;y<40;++y)for(int x=0;x<80;++x){
    const Eigen::Vector3i index(x,y,z);const auto point=map->center(index);
    map->set(index,point.z()<.5*std::clamp(point.x()+1.,0.,2.)?VoxelStatus::Occupied:VoxelStatus::Free);
  }
  return map;
}
void asymmetricAndMapChange(){
  auto p=parameters();p.robot_size={.3,.3,.3};p.ground.max_negative_inclination=.35;
  auto backing=ramp();auto map=std::make_shared<CountMap>(backing);TerrainProjector terrain(map,p.groundParameters());
  std::vector<StateVec> states;
  for(double x:{-.75,0.,.75}){
    const auto projected=terrain.project(StateVec(x,0.,2.,.3));
    require(projected.status==GroundStatus::Supported,"Ramp fixture is unsupported");
    states.push_back(projected.projected_state);
  }
  require(terrain.edge(states.front(),states.back()).admissible()&&
          !terrain.edge(states.back(),states.front()).admissible(),"Ramp is not directionally asymmetric");
  GraphManager local;
  for(std::size_t i=0;i<states.size();++i){
    local.addVertex(new Vertex(int(i),states[i]));
    if(i)local.addEdge(local.getVertex(int(i)-1),local.getVertex(int(i)),1.);
  }
  local.getVertex(2)->vol_gain=frontier(states.back());
  planner_msgs::msg::Graph message;local.convertGraphToMsg(message);
  GlobalMemory old(p),retained(p);
  setup(old,map,TerrainQueryReuse::Disabled,states.front());
  setup(retained,map,TerrainQueryReuse::ImmutableSnapshot,states.front());
  old.absorbLocal(message,frontier);retained.absorbLocal(message,frontier);sameGraph(old,retained);
  require(old.lastFrontierUpdate().added==1,"Ordinary directed path was not absorbed");
  int target=-1;for(const auto& item:old.graph().vertices_map_)
    if(item.second->type==VertexType::kFrontier)target=item.first;
  require(target>0,"Absorbed endpoint is not a frontier");
  const StateVec top=old.graph().vertices_map_.at(target)->state;
  const auto forward=old.selectFrontier(states.front(),frontier,target);
  const auto cached_forward=retained.selectFrontier(states.front(),frontier,target);
  const auto backward=old.homing(top),cached_backward=retained.homing(top);
  require(forward.success&&!backward.success&&backward.reason=="global_target_unreachable",
          "An uphill edge was rejected or a forbidden downhill edge became usable");
  sameRoute(forward,cached_forward);sameRoute(backward,cached_backward);sameGraph(old,retained);

  // End the immutable interval before changing this SAME map object. A wall
  // blocks the previous parent chain; all saved directional results must clear.
  const int column=backing->index({0.,0.,0.}).x();
  for(int y=0;y<40;++y)for(int z=0;z<80;++z)backing->set({column,y,z},VoxelStatus::Occupied);
  update(old,retained,map);
  require(old.lastMapEdgeUpdate().invalidated>0&&old.lastMapEdgeUpdate().pending>0,
          "Accepted parent-edge verdict survived map replacement");
  const auto blocked=old.selectFrontier(states.front(),frontier,target);
  const auto cached_blocked=retained.selectFrontier(states.front(),frontier,target);
  require(!blocked.success,"Map wall did not block the former route");sameRoute(blocked,cached_blocked);sameGraph(old,retained);
  for(int y=0;y<40;++y)for(int z=0;z<80;++z){
    const Eigen::Vector3i index(column,y,z);const auto point=backing->center(index);
    backing->set(index,point.z()<.5*std::clamp(point.x()+1.,0.,2.)?VoxelStatus::Occupied:VoxelStatus::Free);
  }
  update(old,retained,map);require(old.lastMapEdgeUpdate().restored>0,"Recovered parent links stayed suspended");
  const auto recovered=old.selectFrontier(states.front(),frontier,target);
  const auto cached_recovered=retained.selectFrontier(states.front(),frontier,target);
  require(recovered.success,"Recovered map did not recover the uphill route");
  sameRoute(recovered,cached_recovered);sameGraph(old,retained);
  const auto recovered_backward=old.homing(top),cached_recovered_backward=retained.homing(top);
  require(!recovered_backward.success&&recovered_backward.reason=="global_target_unreachable",
          "Restoring a one-way link enabled its forbidden downhill direction");
  sameRoute(recovered_backward,cached_recovered_backward);sameGraph(old,retained);
}
std::size_t producerPrecheck(){
  auto p=parameters();p.ground.interpolate_projection_distance=true;
  p.ground.tsdf_projection_mode=GroundTsdfProjectionMode::ZeroCrossingSimplex;
  p.local_min={-2.5,-2.5,-.05};p.local_max={2.5,2.5,.05};
  p.max_vertices=28;p.max_edges=200;p.max_attempts=1000;p.seed=17;
  p.sensor_range=.5;p.horizontal_fov=M_PI/2;p.vertical_fov=M_PI/4;
  p.horizontal_resolution=M_PI/2;p.vertical_resolution=M_PI/4;
  auto map=std::make_shared<CountMap>(flat());const StateVec home(0.,0.,.8,.3);
  LocalPlanner producer(map,p,{},ExactRayGainReuse::ImmutableSnapshot);
  const auto result=producer.plan(home);require(result.graph_build_completed&&result.vertices>2,"Producer fixture did not build edges");
  planner_msgs::msg::Graph message;producer.graphMessage(message);
  // A constant frontier oracle isolates path absorption from sensor reward.
  // States, IDs and topology remain the real producer's generated graph.
  for(auto& vertex:message.vertices)if(vertex.id)vertex.is_frontier=true;
  std::size_t reusable=0;
  for(const auto& adjacency:producer.graph().edge_map_)for(const auto& edge:adjacency.second){
    const auto& from=producer.graph().vertices_map_.at(adjacency.first)->state;
    const auto& to=producer.graph().vertices_map_.at(edge.first)->state;
    if(producer.validatedGroundLength(map,p.groundParameters(),adjacency.first,edge.first,from,to))++reusable;
  }
  require(reusable>0,"Producer fixture has no reusable strict edge");
  GlobalMemory old(p),retained(p),with_producer(p);
  setup(old,map,TerrainQueryReuse::Disabled,home);
  setup(retained,map,TerrainQueryReuse::ImmutableSnapshot,home);
  setup(with_producer,map,TerrainQueryReuse::ImmutableSnapshot,home);
  old.absorbLocal(message,frontier);retained.absorbLocal(message,frontier);
  with_producer.absorbLocal(message,frontier,nullptr,{},&producer);
  require(old.lastFrontierUpdate().added>0,"Producer fixture did not exercise path admission");
  sameGraph(old,retained);sameGraph(old,with_producer);
  for(const auto& item:old.graph().vertices_map_)if(item.second->type==VertexType::kFrontier){
    const int id=item.first;
    const auto a=old.selectFrontier(home,frontier,id),b=retained.selectFrontier(home,frontier,id),
        c=with_producer.selectFrontier(home,frontier,id);
    require(a.success,"Absorbed producer frontier is unreachable");sameRoute(a,b);sameRoute(a,c);
    break;
  }
  sameGraph(old,retained);sameGraph(old,with_producer);return reusable;
}
} // namespace
int main(int argc,char** argv){try{
  const auto suspension=newlySuspendedFixedEdge();
  if(argc==2&&std::string(argv[1])=="--suspension-only"){
    asymmetricAndMapChange();
    std::cout<<"{\"status\":\"passed\",\"newly_suspended_fixed_edge\":true,"
        "\"same_pointer_persistent_obstacle_rechecked\":true,\"cleared_obstacle_restored\":true,"
        "\"directional_slope_preserved_after_restore\":true,\"full_graph_and_double_weights_exact\":true,"
        "\"default_blocked_path_queries\":"<<suspension.default_blocked
        <<",\"disabled_blocked_path_queries\":"<<suspension.disabled_blocked
        <<",\"retained_blocked_path_queries\":"<<suspension.retained_blocked
        <<",\"retained_pending_path_queries\":"<<suspension.retained_pending
        <<",\"retained_restored_path_queries\":"<<suspension.retained_restored<<"}\n";
    return 0;
  }
  require(argc==1,"Usage: check_global_edge_retention [--suspension-only]");
  const auto calls=fixedEdges();asymmetricAndMapChange();const auto reusable=producerPrecheck();
  std::cout<<"{\"status\":\"passed\",\"full_graph_and_double_weights_exact\":true,"
      "\"routes_exact\":true,\"directional_slope_preserved\":true,\"same_pointer_map_refresh\":true,"
      "\"strict_producer_precheck_equal\":true,\"fixed_route_baseline_path_queries\":"<<calls.first
      <<",\"fixed_route_retained_path_queries\":"<<calls.second
      <<",\"producer_reusable_directed_edges\":"<<reusable
      <<",\"newly_suspended_default_path_queries\":"<<suspension.default_blocked
      <<",\"newly_suspended_retained_path_queries\":"<<suspension.retained_blocked<<"}\n";
  return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
