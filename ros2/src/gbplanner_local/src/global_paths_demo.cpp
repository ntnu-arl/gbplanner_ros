// Selected frontier graph chains: both directions, without replacing execution geometry.
#include "gbplanner_local/global_path_markers.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace gbplanner2;
namespace {
void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
bool samePath(const std::vector<StateVec>& a,const std::vector<StateVec>& b){
  if(a.size()!=b.size())return false;
  for(std::size_t i=0;i<a.size();++i)if((a[i]-b[i]).norm()>1e-9)return false;
  return true;
}
Parameters aerialParameters(){
  Parameters p;p.robot_size={.1,.1,.1};p.overshoot=0.;
  p.edge_min=.15;p.edge_max=.8;p.neighbor_radius=.8;p.neighbor_min=.15;p.neighbor_max=.8;
  p.refinement.enabled=false;p.tangent_yaw=false;p.validate();return p;
}
std::shared_ptr<VoxelMap> freeMap(){
  auto map=std::make_shared<VoxelMap>(.05,Eigen::Vector3d(-1.,-1.,-1.),Eigen::Vector3i(80,40,40));
  map->fill(VoxelStatus::Free);return map;
}
struct Outcome {GlobalResult selected;int gain_calls{0};};
class CountingMap final:public MapQuery {
 public:
  explicit CountingMap(std::shared_ptr<const MapQuery> map):map_(std::move(map)){}
  double resolution()const override{++queries;return map_->resolution();}
  Eigen::Vector3d minimum()const override{++queries;return map_->minimum();}
  Eigen::Vector3d maximum()const override{++queries;return map_->maximum();}
  VoxelStatus voxel(const Eigen::Vector3d& p)const override{++queries;return map_->voxel(p);}
  VoxelStatus box(const Eigen::Vector3d& p,const Eigen::Vector3d& size)const override{
    ++queries;return map_->box(p,size);
  }
  VoxelStatus path(const Eigen::Vector3d& a,const Eigen::Vector3d& b,const Eigen::Vector3d& size)const override{
    ++queries;return map_->path(a,b,size);
  }
  void ray(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
      const std::function<bool(const Eigen::Vector3d&,VoxelStatus)>& visit)const override{
    ++queries;map_->ray(a,b,visit);
  }
  mutable std::size_t queries{0};
 private:std::shared_ptr<const MapQuery> map_;
};
HomingPathDiagnostics aerialHoming(){
  auto map=std::make_shared<CountingMap>(freeMap());
  GlobalMemory memory(aerialParameters());memory.updateMap(map);
  const std::vector<StateVec> reference{{0.,0.,0.,0.},{1.,0.,0.,.2},{2.,0.,0.,.4}};
  require(memory.setHomingPosition(reference.front())&&memory.addReferencePath(reference),"Could not seed homing chain");
  const StateVec current(2.,0.,0.,.7);
  map->queries=0;const auto plain=memory.homing(current);const auto plain_queries=map->queries;
  const auto revision=memory.snapshotRevision(),visits=memory.visitRevision();
  HomingPathDiagnostics diagnostic;
  map->queries=0;const auto captured=memory.homing(current,false,&diagnostic);
  require(captured.success&&plain.success&&samePath(captured.path,plain.path)&&captured.distance==plain.distance,
          "Homing capture changed execution geometry or distance");
  require(map->queries==plain_queries&&memory.snapshotRevision()==revision&&memory.visitRevision()==visits,
          "Homing capture changed map query count, topology or visits");
  auto reverse=reference;std::reverse(reverse.begin(),reverse.end());
  require(diagnostic.captured&&diagnostic.vertex_ids==std::vector<int>({2,1,0})&&samePath(diagnostic.vertices,reverse),
          "Homing display lost raw graph chain or yaw");
  require(captured.path.front()[3]==.7&&diagnostic.vertices.front()[3]==.4,"Display replaced raw yaw by execution yaw");
  const auto retained=diagnostic;
  const auto already_home=memory.homing(reference.front(),false,&diagnostic);
  require(already_home.success&&diagnostic.captured&&diagnostic.vertex_ids==std::vector<int>({0})&&
          homingPathMarkers(diagnostic,{}).markers.size()==1,"Single-point home created a stale line");
  memory.reset();const auto failed=memory.homing(current,false,&diagnostic);
  require(!failed.success&&!diagnostic.captured&&diagnostic.vertex_ids.empty()&&diagnostic.vertices.empty(),
          "Failed homing retained prior call geometry");
  return retained;
}
Outcome aerial(bool automatic,bool capture){
  GlobalMemory memory(aerialParameters());memory.updateMap(freeMap());
  const std::vector<StateVec> reference{{0.,0.,0.,0.},{1.,0.,0.,0.},{2.,0.,0.,0.}};
  require(memory.setHomingPosition(reference.front()),"Could not establish home");
  const int target=memory.addSemanticPath(reference,planner_semantic_msgs::msg::SemanticClass::K_DOOR);
  require(target==2,"Expected three-vertex reference chain");
  Outcome outcome;
  const auto gain=[&](const StateVec&){
    ++outcome.gain_calls;VolumetricGain value;value.reset();
    value.gain=10.;value.num_unknown_voxels=1;value.is_frontier=true;return value;
  };
  const auto visit_revision=memory.visitRevision();
  const StateVec current(0.,0.,0.,.7);
  outcome.selected=memory.selectFrontier(current,gain,automatic?-1:target,nullptr,false,false,capture);
  const int selected_gain_calls=outcome.gain_calls;
  const auto& selected=outcome.selected;
  require(selected.success&&selected.target==target,"Frontier selection failed");
  require(std::abs(selected.path.front()[3]-.7)<1e-9,"Execution path lost requested heading");
  require(memory.visitRevision()==visit_revision&&memory.visitedStates().empty(),"Display invented a visit");
  if(capture){
    const auto& diagnostic=selected.graph_paths;
    require(diagnostic.captured&&diagnostic.home_reason=="ok","Expected both graph chains");
    require(diagnostic.current_to_frontier_ids==std::vector<int>({0,1,2})&&
            diagnostic.frontier_to_home_ids==std::vector<int>({2,1,0}),"Graph chain direction changed");
    auto reverse=reference;std::reverse(reverse.begin(),reverse.end());
    require(samePath(diagnostic.current_to_frontier,reference)&&samePath(diagnostic.frontier_to_home,reverse),
            "Display used execution yaw or changed raw vertices");
    const auto rejected=memory.selectFrontier(current,gain,999,nullptr,false,false,true);
    require(!rejected.success&&!rejected.graph_paths.captured&&rejected.graph_paths.current_to_frontier.empty()&&
            rejected.graph_paths.frontier_to_home.empty(),"Failed selection retained graph geometry");
    const auto home=memory.homing(reference.back());
    require(home.success&&!home.graph_paths.captured,"Homing invented a selected frontier display");
    memory.reset();
    const auto empty=memory.selectFrontier(current,gain,-1,nullptr,false,false,true);
    require(!empty.success&&!empty.graph_paths.captured,"Graph reset retained selected paths");
  }else require(!selected.graph_paths.captured&&selected.graph_paths.current_to_frontier.empty()&&
               selected.graph_paths.frontier_to_home.empty(),"Disabled capture retained geometry");
  outcome.gain_calls=selected_gain_calls;
  return outcome;
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
GlobalResult oneWay(){
  auto map=ramp();Parameters p;p.ground_robot=true;p.overshoot=0.;p.refinement.enabled=false;
  p.robot_size={.3,.3,.3};p.ground.max_ground_height=.8;p.ground.robot_height=.6;
  p.ground.max_inclination=.9;p.ground.max_negative_inclination=.35;
  TerrainProjector terrain(map,p.groundParameters());
  const auto low=terrain.project(StateVec(-1.5,0.,2.,0.));
  const auto high=terrain.project(StateVec(1.5,0.,2.,0.));
  require(low.status==GroundStatus::Supported&&high.status==GroundStatus::Supported,"Ramp has no support");
  const auto uphill=terrain.edge(low.projected_state,high.projected_state);
  require(uphill.admissible()&&!terrain.edge(high.projected_state,low.projected_state).admissible(),
          "Ramp does not distinguish uphill and downhill reachability");
  GlobalMemory memory(p);memory.updateMap(map);
  require(memory.setHomingPosition(low.projected_state),"Could not seed ground home");
  // Reference insertion preserves the supplied geometry. A straight chord
  // between the two flat ends does not follow this piecewise ramp, so supply
  // the actual checked terrain polyline rather than its endpoint chord.
  int target=-1;ReferencePathDiagnostics admission;
  if(!memory.addReferencePath(uphill.polyline,&target,&admission))
    throw std::runtime_error("Could not seed uphill terrain polyline: "+admission.reason);
  const auto gain=[](const StateVec&){return VolumetricGain{};};
  const auto plain=memory.selectFrontier(low.projected_state,gain,target);
  const auto captured=memory.selectFrontier(low.projected_state,gain,target,nullptr,false,false,true);
  require(plain.success&&captured.success&&samePath(plain.path,captured.path)&&plain.distance==captured.distance,
          "Missing graph return changed successful outbound execution");
  const auto& diagnostic=captured.graph_paths;
  require(diagnostic.captured&&diagnostic.home_reason=="home_unreachable"&&
          diagnostic.frontier_to_home_ids.empty()&&diagnostic.frontier_to_home.empty(),
          "Undirected topology was shown as a traversable downhill return");
  require(diagnostic.current_to_frontier.size()>=2&&
          diagnostic.current_to_frontier_ids.size()==diagnostic.current_to_frontier.size(),
          "Ground display omitted graph vertices");
  for(std::size_t i=0;i<diagnostic.current_to_frontier.size();++i){
    const auto found=memory.graph().vertices_map_.find(diagnostic.current_to_frontier_ids[i]);
    require(found!=memory.graph().vertices_map_.end()&&
      (found->second->state-diagnostic.current_to_frontier[i]).norm()<1e-9,"Ground display changed planning height");
  }
  HomingPathDiagnostics homing;
  const auto rejected_home=memory.homing(high.projected_state,false,&homing);
  require(!rejected_home.success&&!homing.captured&&homing.vertices.empty(),"Downhill failure published a home chain");
  p.ground.max_negative_inclination=.9;
  GlobalMemory reversible(p);reversible.updateMap(map);
  require(reversible.setHomingPosition(low.projected_state)&&reversible.addReferencePath(uphill.polyline),
          "Could not seed reversible ground route");
  const auto ground_home=reversible.homing(high.projected_state,false,&homing);
  require(ground_home.success&&homing.captured&&homing.vertices.size()==homing.vertex_ids.size(),
          "Ground homing display absent");
  for(std::size_t i=0;i<homing.vertices.size();++i)
    require((reversible.graph().vertices_map_.at(homing.vertex_ids[i])->state-homing.vertices[i]).norm()<1e-9,
            "Ground homing display changed planning height");
  return captured;
}
}
int main(){
  try{
    const auto explicit_plain=aerial(false,false),explicit_capture=aerial(false,true);
    const auto automatic_plain=aerial(true,false),automatic_capture=aerial(true,true);
    for(const auto* pair:{&explicit_capture,&automatic_capture}){
      const auto& plain=pair==&explicit_capture?explicit_plain:automatic_plain;
      require(pair->selected.success==plain.selected.success&&pair->selected.target==plain.selected.target&&
        pair->selected.distance==plain.selected.distance&&pair->selected.score==plain.selected.score&&
        samePath(pair->selected.path,plain.selected.path)&&pair->gain_calls==plain.gain_calls,
        "Capture changed frontier selection, execution geometry or gain calls");
    }
    const auto homing=aerialHoming();
    const auto ground=oneWay();std_msgs::msg::Header header;header.frame_id="map";
    const auto home_markers=homingPathMarkers(homing,header);
    require(home_markers.markers.size()==2&&home_markers.markers[0].action==visualization_msgs::msg::Marker::DELETEALL&&
      home_markers.markers[1].ns=="best_path"&&home_markers.markers[1].color.g==1.&&
      home_markers.markers[1].color.r==0.&&home_markers.markers[1].scale.x==.4&&
      home_markers.markers[1].points.size()==4,"Homing marker snapshot differs from original style");
    const auto both=globalPathMarkers(explicit_capture.selected.graph_paths,header);
    const auto outward=globalPathMarkers(ground.graph_paths,header);
    const auto clear=globalPathMarkers({},header);
    using Marker=visualization_msgs::msg::Marker;
    require(both.markers.size()==3&&both.markers[0].action==Marker::DELETEALL&&
      both.markers[1].ns=="current2frontier"&&both.markers[2].ns=="frontier2home"&&
      both.markers[1].color.g==1.&&both.markers[2].color.r==.5&&
      both.markers[1].points.size()==4&&both.markers[2].points.size()==4,
      "Two-chain marker snapshot differs from upstream namespaces/colors");
    require(outward.markers.size()==2&&outward.markers[0].action==Marker::DELETEALL&&
      outward.markers[1].ns=="current2frontier"&&clear.markers.size()==1&&
      clear.markers[0].action==Marker::DELETEALL,"Missing/cleared return leaves a stale marker");
    std::cout<<"{\"status\":\"passed\",\"explicit_and_automatic\":true,\"capture_preserves_selection\":true"
      <<",\"raw_graph_heading_preserved\":true,\"ground_planning_height_preserved\":true"
      <<",\"one_way_return_not_fabricated\":true,\"failure_reset_homing_empty\":true"
      <<",\"homing_capture_preserves_result_queries_graph_visits\":true,\"homing_raw_yaw_and_ground_height\":true"
      <<",\"homing_one_way_failure_and_single_point_clear\":true,\"homing\":"<<homingPathDiagnosticsJson(homing)
      <<",\"marker_snapshots_clear_stale_paths\":true,\"aerial\":"
      <<globalPathDiagnosticsJson(explicit_capture.selected.graph_paths)<<",\"ground\":"
      <<globalPathDiagnosticsJson(ground.graph_paths)<<"}\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
