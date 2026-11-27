// A supported endpoint on each bank must not authorize an unknown-ground bridge.
#include "gbplanner_local/local_planner.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace gbplanner2;
namespace {
void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
std::shared_ptr<VoxelMap> scene(){
  auto map=std::make_shared<VoxelMap>(.1,Eigen::Vector3d(-4.,-2.,-1.),Eigen::Vector3i(80,40,30));
  map->fill(VoxelStatus::Free);
  for(int z=0;z<10;++z)for(int y=0;y<40;++y)for(int x=0;x<80;++x){
    const Eigen::Vector3i index(x,y,z);
    map->set(index,std::abs(map->center(index).x())<.75?VoxelStatus::Unknown:VoxelStatus::Occupied);
  }
  return map;
}
Parameters parameters(){
  Parameters p;p.ground_robot=true;p.ground.max_ground_height=.8;p.ground.robot_height=.6;
  p.ground.max_inclination=.9;p.ground.max_negative_inclination=.9;
  p.robot_size={.2,.2,.2};p.edge_min=.05;p.edge_max=4.;p.overshoot=0.;p.graph_mode=false;
  return p;
}
void add(GraphManager& graph,const StateVec& state){graph.addVertex(new Vertex(graph.generateVertexID(),state));}
bool linked(const GraphManager& graph,int from,int to){
  const auto neighbors=graph.edge_map_.find(from);
  if(neighbors==graph.edge_map_.end())return false;
  for(const auto& edge:neighbors->second)if(edge.first==to)return true;
  return false;
}
void checkFixedBranches(const std::shared_ptr<VoxelMap>& map,const Parameters& p,
                        const TerrainProjector& terrain,const StateVec& left,const StateVec& right){
  GraphManager reverse_parent;add(reverse_parent,right);GroundEdgeCache reverse_edges;PlanResult reverse_counters;
  require(!expandGround(reverse_parent,map,p,left,reverse_counters,p.max_edges,&reverse_edges,true)&&
    reverse_parent.getNumVertices()==1&&reverse_edges.empty(),"Reverse parent edge crossed the support gap");

  // The newly added vertex has a legal parent on its own bank. A graph-mode
  // neighbor on the other bank must be rejected in BOTH checked directions.
  auto neighbors=p;neighbors.graph_mode=true;neighbors.neighbor_radius=4.;neighbors.neighbor_max=3.5;
  GraphManager graph;add(graph,StateVec(-1.8,0.,.8,0.));add(graph,right);
  GroundEdgeCache cache;PlanResult counters;
  require(expandGround(graph,map,neighbors,left,counters,neighbors.max_edges,&cache,true)&&
    linked(graph,0,2)&&!linked(graph,1,2)&&!linked(graph,2,1)&&
    cache.count({0,2})&&cache.count({2,0})&&!cache.count({1,2})&&!cache.count({2,1}),
    "Supported neighbor endpoints authorized a speculative edge in one direction");

  // Only the extension endpoint is hanging. The logical parent endpoints are
  // supported, so that virtual endpoint must not grant candidate permission.
  const StateVec overshoot_source(-2.4,0.,.8,0.),overshoot_target(-1.4,0.,.8,0.);
  const StateVec unknown(0.,0.,.8,0.);
  require(terrain.edge(overshoot_source,overshoot_target).admissible()&&
    terrain.project(unknown).hanging&&terrain.candidateEdge(overshoot_source,unknown).candidateAdmissible(),
    "Parent overshoot fixture did not isolate a newly hanging virtual endpoint");
  auto extended=p;extended.overshoot=1.4;
  GraphManager extension;add(extension,overshoot_source);GroundEdgeCache extension_cache;PlanResult extension_count;
  require(!expandGround(extension,map,extended,overshoot_target,extension_count,extended.max_edges,
    &extension_cache,true)&&extension.getNumVertices()==1,
    "Parent overshoot acquired permission from its newly hanging virtual endpoint");

  // A vertical-on-the-map parent remains fully supported after extension.
  // The lateral neighbor's extension reaches unknown ground at either its
  // start or its end, depending on which directed check is being performed.
  auto neighbor_extension=neighbors;neighbor_extension.overshoot=1.;
  const StateVec added(-1.6,0.,.8,0.),near(-1.,0.,.8,0.);
  const StateVec extended_left(-2.6,0.,.8,0.);
  require(terrain.edge(added,near).admissible()&&
    terrain.candidateEdge(extended_left,unknown).candidateAdmissible()&&
    terrain.candidateEdge(unknown,extended_left).candidateAdmissible(),
    "Neighbor overshoot fixture did not distinguish logical and virtual endpoints");
  GraphManager extension_neighbors;add(extension_neighbors,StateVec(-1.6,-.4,.8,0.));add(extension_neighbors,near);
  GroundEdgeCache extension_neighbors_cache;PlanResult extension_neighbors_count;
  require(expandGround(extension_neighbors,map,neighbor_extension,added,extension_neighbors_count,
    neighbor_extension.max_edges,&extension_neighbors_cache,true)&&linked(extension_neighbors,0,2)&&
    !linked(extension_neighbors,1,2)&&!linked(extension_neighbors,2,1)&&
    !extension_neighbors_cache.count({1,2})&&!extension_neighbors_cache.count({2,1}),
    "Neighbor overshoot relaxed the logical endpoints' permission in one direction");

  // Actual hanging endpoints retain lookahead. Carry that original permission
  // even when the overshoot endpoint happens to land on the opposite bank.
  require(terrain.candidateEdge(left,unknown).candidateAdmissible()&&
    terrain.candidateEdge(unknown,left).candidateAdmissible()&&!terrain.edge(left,unknown).admissible(),
    "Actual hanging-endpoint lookahead was removed or became executable");
  auto hanging_extension=p;hanging_extension.overshoot=1.5;
  GraphManager hanging;add(hanging,left);GroundEdgeCache hanging_cache;PlanResult hanging_count;
  require(expandGround(hanging,map,hanging_extension,unknown,hanging_count,hanging_extension.max_edges,
    &hanging_cache,true)&&hanging.getVertex(1)->is_hanging&&
    hanging_cache.at({0,1}).candidateAdmissible()&&hanging_cache.at({1,0}).candidateAdmissible()&&
    !hanging_cache.at({0,1}).admissible()&&!hanging_cache.at({1,0}).admissible(),
    "Original hanging permission was lost when an extension landed on known support");
  GraphManager hanging_neighbors;add(hanging_neighbors,left);add(hanging_neighbors,StateVec(1.8,0.,.8,0.));
  GroundEdgeCache hanging_neighbors_cache;PlanResult hanging_neighbors_count;
  require(expandGround(hanging_neighbors,map,neighbors,unknown,hanging_neighbors_count,neighbors.max_edges,
    &hanging_neighbors_cache,true)&&hanging_neighbors.getVertex(2)->is_hanging&&
    hanging_neighbors_cache.at({1,2}).candidateAdmissible()&&
    hanging_neighbors_cache.at({2,1}).candidateAdmissible()&&
    !hanging_neighbors_cache.at({1,2}).admissible()&&!hanging_neighbors_cache.at({2,1}).admissible(),
    "Actual hanging neighbor lost its permission in one direction");

  // Keep body and observed lack-of-ground rejection independent of permission.
  // Offset the body patch away from all five support rays so it cannot change
  // the support projection and thereby accidentally mask a clearance failure.
  for(const auto status:{VoxelStatus::Unknown,VoxelStatus::Occupied}){
    auto blocked=scene();auto blocked_parameters=p;blocked_parameters.center_offset={0.,.25,0.};
    blocked->set(blocked->index({0.,.25,.6}),status);
    TerrainProjector blocked_terrain(blocked,blocked_parameters.groundParameters());
    require(blocked_terrain.project(unknown).hanging&&
      !blocked_terrain.candidateEdge(left,unknown).candidateAdmissible(),
      "Hanging permission bypassed an unknown or occupied body voxel");
  }
  auto bottomless=std::make_shared<VoxelMap>(.1,Eigen::Vector3d(-4.,-2.,-6.),Eigen::Vector3i(80,40,90));
  bottomless->fill(VoxelStatus::Free);TerrainProjector no_ground(bottomless,p.groundParameters());
  require(no_ground.project(unknown).status==GroundStatus::NoGround&&
    !no_ground.candidateEdge(left,unknown).candidateAdmissible(),
    "Observed absence of ground became speculative support");
}
}
int main(int argc,char** argv){
  try{
    const bool baseline=argc==2&&std::string(argv[1])=="--baseline";
    require(argc==1||baseline,"Only --baseline is supported");
    auto map=scene();const auto p=parameters();TerrainProjector terrain(map,p.groundParameters());
    const StateVec left(-1.5,0.,.8,0.),right(1.5,0.,.8,0.);
    require(terrain.project(left).status==GroundStatus::Supported&&
      terrain.project(right).status==GroundStatus::Supported&&terrain.project(StateVec(0.,0.,.8,0.)).hanging,
      "The fixture must have supported banks and unknown central support");
    const auto strict=terrain.edge(left,right),forward=terrain.candidateEdge(left,right),
               reverse=terrain.candidateEdge(right,left);
    require(!strict.admissible(),"Strict execution crossed the unknown ground");
    GraphManager graph;graph.addVertex(new Vertex(graph.generateVertexID(),left));
    GroundEdgeCache edges;PlanResult counters;
    const bool added=expandGround(graph,map,p,right,counters,p.max_edges,&edges,true);
    if(baseline){
      require(forward.candidateAdmissible()&&reverse.candidateAdmissible()&&
        forward.speculative&&reverse.speculative&&added&&graph.getNumVertices()==2&&
        !graph.getVertex(0)->is_hanging&&!graph.getVertex(1)->is_hanging,
        "The unmodified core did not reproduce the supported-bank bridge");
    }else{
      require(!forward.candidateAdmissible()&&!reverse.candidateAdmissible()&&!added&&
        graph.getNumVertices()==1&&edges.empty(),"Supported banks still authorize an unknown-ground bridge");
      checkFixedBranches(map,p,terrain,left,right);
    }
    std::cout<<"{\"status\":\""<<(baseline?"baseline_reproduced":"passed")
      <<"\",\"both_endpoints_supported\":true,\"strict_rejected\":true"
      <<",\"forward_candidate\":"<<(forward.candidateAdmissible()?"true":"false")
      <<",\"reverse_candidate\":"<<(reverse.candidateAdmissible()?"true":"false")
      <<",\"parent_expanded\":"<<(added?"true":"false")<<",\"vertices\":"<<graph.getNumVertices();
    if(!baseline)std::cout<<",\"neighbor_both_directions_rejected\":true,\"overshoot_permission_fixed\":true"
      <<",\"actual_hanging_lookahead_retained\":true,\"body_and_no_ground_rejected\":true";
    std::cout<<"}\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
