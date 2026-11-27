// Observe existing ROS 2 directed ground admission; no negative-slope rescoring.
#include "gbplanner_local/local_planner.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace gbplanner2;
namespace {
void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
bool samePath(const std::vector<StateVec>& a,const std::vector<StateVec>& b){
  if(a.size()!=b.size())return false;
  for(std::size_t i=0;i<a.size();++i)if((a[i]-b[i]).norm()!=0.)return false;
  return true;
}
bool sameEdge(const GroundEdge& a,const GroundEdge& b){
  return a.status==b.status&&a.speculative==b.speculative&&a.actual_length==b.actual_length&&
    a.max_uphill==b.max_uphill&&a.max_downhill==b.max_downhill&&a.mean_inclination==b.mean_inclination&&
    samePath(a.polyline,b.polyline);
}
class CountingMap final:public MapQuery {
 public:
  explicit CountingMap(std::shared_ptr<const MapQuery> map):map_(std::move(map)){}
  double resolution()const override{++queries[0];return map_->resolution();}
  Eigen::Vector3d minimum()const override{++queries[1];return map_->minimum();}
  Eigen::Vector3d maximum()const override{++queries[2];return map_->maximum();}
  VoxelStatus voxel(const Eigen::Vector3d& point)const override{++queries[3];return map_->voxel(point);}
  std::optional<TsdfDistance> tsdfDistance(const Eigen::Vector3d& point)const override{
    ++queries[4];return map_->tsdfDistance(point);
  }
  std::optional<TsdfDistance> tsdfDistanceSimplex(const Eigen::Vector3d& point)const override{
    ++queries[5];return map_->tsdfDistanceSimplex(point);
  }
  VoxelStatus box(const Eigen::Vector3d& point,const Eigen::Vector3d& size)const override{
    ++queries[6];return map_->box(point,size);
  }
  VoxelStatus path(const Eigen::Vector3d& a,const Eigen::Vector3d& b,const Eigen::Vector3d& size)const override{
    ++queries[7];return map_->path(a,b,size);
  }
  bool gainAllowed(const Eigen::Vector3d& point)const override{++queries[8];return map_->gainAllowed(point);}
  void ray(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
      const std::function<bool(const Eigen::Vector3d&,VoxelStatus)>& visit)const override{
    ++queries[9];map_->ray(a,b,visit);
  }
  mutable std::array<std::size_t,10> queries{};
 private:
  std::shared_ptr<const MapQuery> map_;
};
std::shared_ptr<VoxelMap> scene(bool ramp){
  auto map=std::make_shared<VoxelMap>(.05,Eigen::Vector3d(-4.,-1.,-1.),Eigen::Vector3i(160,40,80));
  for(int z=0;z<80;++z)for(int y=0;y<40;++y)for(int x=0;x<160;++x){
    const Eigen::Vector3i index(x,y,z);const auto point=map->center(index);
    const double height=ramp?.5*std::clamp(point.x()+1.,0.,2.):0.;
    auto status=point.z()<height?VoxelStatus::Occupied:VoxelStatus::Free;
    if(!ramp&&std::abs(point.x())<.75&&point.z()<.1)status=VoxelStatus::Unknown;
    map->set(index,status);
  }
  return map;
}
Parameters parameters(){
  Parameters p;p.ground_robot=true;p.ground.max_ground_height=.8;p.ground.robot_height=.6;
  p.ground.max_inclination=.9;p.ground.max_negative_inclination=.35;
  p.robot_size={.2,.2,.2};p.edge_min=.05;p.edge_max=4.;p.overshoot=0.;p.graph_mode=false;
  p.neighbor_min=.05;p.neighbor_radius=4.;p.neighbor_max=4.;return p;
}
GroundEdgeDiagnostics paired(const std::shared_ptr<VoxelMap>& map,const Parameters& p,
    const std::vector<StateVec>& vertices,const StateVec& target,bool expected_expansion){
  auto off_map=std::make_shared<CountingMap>(map),on_map=std::make_shared<CountingMap>(map);
  GraphManager off,on;
  for(const auto& state:vertices){
    off.addVertex(new Vertex(off.generateVertexID(),state));on.addVertex(new Vertex(on.generateVertexID(),state));
  }
  PlanningDiagnostics off_diagnostics,on_diagnostics;
  off_diagnostics.reset();on_diagnostics.options.ground_edges=true;on_diagnostics.reset();
  GroundEdgeCache off_edges,on_edges;PlanResult a,b;
  const auto off_added=expandGround(off,off_map,p,target,a,p.max_edges,&off_edges,true,&off_diagnostics);
  const auto on_added=expandGround(on,on_map,p,target,b,p.max_edges,&on_edges,true,&on_diagnostics);
  require(off_added==on_added&&on_added==expected_expansion,"Capture changed expansion or fixture expectation");
  require(off_map->queries==on_map->queries,"Capture issued additional map queries");
  require(a.collision_rejections==b.collision_rejections&&a.hanging_vertices==b.hanging_vertices&&
    off.getNumVertices()==on.getNumVertices()&&off.edge_map_==on.edge_map_,"Capture changed graph or counters");
  for(const auto& [id,v]:off.vertices_map_){
    const auto* w=on.vertices_map_.at(id);
    require((v->state-w->state).norm()==0.&&v->distance==w->distance&&v->is_hanging==w->is_hanging,
      "Capture changed graph vertex state");
  }
  require(off_edges.size()==on_edges.size(),"Capture changed directional cache size");
  for(const auto& [key,edge]:off_edges)require(sameEdge(edge,on_edges.at(key)),"Capture changed a directional edge");
  require(!off_diagnostics.ground_edges.enabled&&off_diagnostics.ground_edges.records.empty()&&
    off_diagnostics.ground_edges.total_records==0&&off_diagnostics.ground_edges.total_points==0,
    "Disabled capture retained geometry");
  auto& captured=on_diagnostics.ground_edges;
  require(captured.enabled&&!captured.partial&&!captured.truncated&&
    captured.completed_checks==captured.total_checks&&captured.total_records==captured.records.size(),
    "Completed fixture has incomplete capture");
  std::size_t points=0;for(const auto& item:captured.records){
    points+=item.polyline.size();require(item.total_points==item.polyline.size()&&item.dropped_points==0,
      "Uncapped record lost projected points");
  }
  require(points==captured.total_points,"Capture point total is inconsistent");
  return captured;
}
const GroundEdgeObservation& find(const GroundEdgeDiagnostics& diagnostics,GroundEdgeRole role,
    GroundEdgeDirection direction,bool overshoot){
  for(const auto& item:diagnostics.records)
    if(item.role==role&&item.direction==direction&&item.overshoot==overshoot)return item;
  throw std::runtime_error("Expected directed observation is missing");
}
void compare(const GroundEdgeObservation& item,const GroundEdge& expected){
  require(item.status==expected.status&&item.speculative==expected.speculative&&
    item.actual_length==expected.actual_length&&item.max_uphill==expected.max_uphill&&
    item.max_downhill==expected.max_downhill&&item.mean_inclination==expected.mean_inclination&&
    samePath(item.polyline,expected.polyline),"Observation differs from the original GroundEdge");
}
void checkCaps(){
  GroundEdge edge;edge.status=GroundStatus::Admissible;edge.polyline={StateVec::Zero(),StateVec(1.,0.,0.,0.)};
  GroundEdgeDiagnostics records;records.enabled=true;
  for(std::size_t i=0;i<GroundEdgeDiagnostics::kMaxRecords+1;++i){
    const auto id=records.beginCheck();records.record(id,GroundEdgeRole::Parent,GroundEdgeDirection::Forward,
      false,edge.polyline.front(),edge.polyline.back(),edge.polyline.front(),edge.polyline.back(),edge,true);
    records.finishCheck();
  }
  require(records.truncated&&records.partial&&records.records.size()==GroundEdgeDiagnostics::kMaxRecords&&
    records.dropped_records==1&&records.dropped_points==2,"Record cap is not request-wide");
  GroundEdgeDiagnostics points;points.enabled=true;
  edge.polyline.resize(GroundEdgeDiagnostics::kMaxPoints+3,StateVec(1.,0.,0.,0.));
  const auto id=points.beginCheck();points.record(id,GroundEdgeRole::Parent,GroundEdgeDirection::Forward,
    false,edge.polyline.front(),edge.polyline.back(),edge.polyline.front(),edge.polyline.back(),edge,true);
  points.record(id,GroundEdgeRole::Parent,GroundEdgeDirection::Forward,true,
    edge.polyline.front(),edge.polyline.back(),edge.polyline.front(),edge.polyline.back(),edge,true);
  points.finishCheck();
  require(points.truncated&&points.partial&&points.records[0].polyline.size()==GroundEdgeDiagnostics::kMaxPoints&&
    points.records[0].dropped_points==3&&points.records[1].polyline.empty()&&
    points.total_points-points.dropped_points==GroundEdgeDiagnostics::kMaxPoints,
    "Point cap is not request-wide or hides a truncated prefix");
  GroundEdgeDiagnostics interrupted;interrupted.enabled=true;interrupted.beginCheck();
  require(interrupted.partial&&!interrupted.truncated&&interrupted.completed_checks==0,
    "An interrupted terrain evaluation appears complete");
  PlanningDiagnostics reset;reset.options.ground_edges=true;reset.ground_edges=std::move(points);reset.reset();
  require(reset.ground_edges.enabled&&reset.ground_edges.records.empty()&&!reset.ground_edges.partial&&
    reset.ground_edges.total_points==0,"Request reset retained previous geometry");
}
}
int main(){
  try{
    const auto ramp_map=scene(true),gap_map=scene(false);auto p=parameters();
    TerrainProjector ramp(ramp_map,p.groundParameters()),gap(gap_map,p.groundParameters());
    const auto low=ramp.project(StateVec(-1.5,0.,2.,0.)).projected_state;
    const auto high=ramp.project(StateVec(1.5,0.,2.,0.)).projected_state;
    const auto directional=paired(ramp_map,p,{low},high,true);
    const auto& forward=find(directional,GroundEdgeRole::Parent,GroundEdgeDirection::Forward,false);
    const auto& reverse=find(directional,GroundEdgeRole::Parent,GroundEdgeDirection::Reverse,false);
    require(forward.admitted&&!reverse.admitted&&reverse.status==GroundStatus::Steep&&
      reverse.max_downhill>p.ground.max_negative_inclination&&reverse.polyline.size()>2,
      "Directed downhill rejection or its actual polyline was lost");
    compare(forward,ramp.candidateEdge(low,high,GroundHangingPermission::Forbid));
    compare(reverse,ramp.candidateEdge(high,low,GroundHangingPermission::Forbid));
    require((reverse.from-high).norm()==0.&&(reverse.to-low).norm()==0.,"Reverse logical endpoints were swapped");

    const auto left=gap.project(StateVec(-1.5,0.,2.,0.)).projected_state;
    const auto right=gap.project(StateVec(1.5,0.,2.,0.)).projected_state;
    const auto unsupported=paired(gap_map,p,{left},right,false);
    const auto& crossing=find(unsupported,GroundEdgeRole::Parent,GroundEdgeDirection::Forward,false);
    require(!crossing.admitted&&crossing.status==GroundStatus::Hanging&&!crossing.speculative,
      "Unknown support rejection was hidden or relabelled");
    compare(crossing,gap.candidateEdge(left,right,GroundHangingPermission::Forbid));

    p.overshoot=1.4;
    const auto source=gap.project(StateVec(-2.4,0.,2.,0.)).projected_state;
    const auto target=gap.project(StateVec(-1.4,0.,2.,0.)).projected_state;
    const auto extended=paired(gap_map,p,{source},target,false);
    const auto& base=find(extended,GroundEdgeRole::Parent,GroundEdgeDirection::Forward,false);
    const auto& extra=find(extended,GroundEdgeRole::Parent,GroundEdgeDirection::Forward,true);
    require(base.admitted&&!extra.admitted&&base.check_id==extra.check_id&&
      (extra.from-source).norm()==0.&&(extra.to-target).norm()==0.&&
      (extra.checked_from-source).norm()==0.&&std::abs(extra.checked_to.x())<1.e-9,
      "Overshoot lost the logical pair, the actual extension or the original base result");
    compare(extra,gap.candidateEdge(extra.checked_from,extra.checked_to,GroundHangingPermission::Forbid));

    p.overshoot=0.;p.graph_mode=true;
    const auto near=ramp.project(StateVec(-1.5,.4,2.,0.)).projected_state;
    const auto neighbors=paired(ramp_map,p,{low,high},near,true);
    const auto& outward=find(neighbors,GroundEdgeRole::Neighbor,GroundEdgeDirection::Forward,false);
    const auto& inward=find(neighbors,GroundEdgeRole::Neighbor,GroundEdgeDirection::Reverse,false);
    require(outward.admitted&&!inward.admitted&&inward.status==GroundStatus::Steep,
      "Neighbor direction or downhill rejection was lost");
    checkCaps();
    std::cout<<"{\"status\":\"passed\",\"scope\":\"ros2_directed_edge_admission\","
      <<"\"directional_downhill_rejection\":true,\"unknown_support_rejection\":true,"
      <<"\"actual_overshoot_geometry\":true,\"neighbor_directions\":true,"
      <<"\"unchanged_map_queries_and_graph\":true,\"request_wide_caps\":true}\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
