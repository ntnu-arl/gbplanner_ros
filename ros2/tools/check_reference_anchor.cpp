// Targeted production-library check; no ROS executor or simulator.
// Seed historical vertex coordinates explicitly, then exercise the public
// addReferencePath API against real voxel occupancy and TSDF queries.
#include "gbplanner_local/global_memory.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

using namespace gbplanner2;
namespace {
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
bool same(const StateVec& a,const StateVec& b){return (a.array()==b.array()).all();}
Parameters parameters(){
  Parameters p;p.ground_robot=true;p.robot_size={.1,.1,.1};p.overshoot=0.;
  p.edge_min=.05;p.edge_max=1.;p.neighbor_radius=.75;p.neighbor_min=.05;p.neighbor_max=1.;
  p.ground.max_ground_height=.8;p.ground.robot_height=.6;
  p.ground.max_inclination=25.*M_PI/180.;p.ground.max_negative_inclination=p.ground.max_inclination;
  p.ground.interpolate_projection_distance=true;
  p.ground.tsdf_projection_mode=GroundTsdfProjectionMode::ZeroCrossingSimplex;
  p.refinement.enabled=false;p.tangent_yaw=false;p.validate();return p;
}
std::shared_ptr<VoxelMap> terrainMap(double slope=0.){
  auto map=std::make_shared<VoxelMap>(.05,Eigen::Vector3d(-2.,-1.,-1.),Eigen::Vector3i(80,40,50));
  for(int z=0;z<50;++z)for(int y=0;y<40;++y)for(int x=0;x<80;++x){
    const Eigen::Vector3i index(x,y,z);const auto point=map->center(index);
    const double distance=point.z()-slope*point.x();
    map->set(index,distance<0.?VoxelStatus::Occupied:VoxelStatus::Free);
    map->setTsdf(index,distance,2.);
  }
  return map;
}
const StateVec root(0.,0.,.8,.4);
const StateVec bad_nearest(.0131677595552,0.,.81630260962,.1);
std::vector<StateVec> reference(double slope=0.){
  return {root,StateVec(.4,0.,.8+.4*slope,.7)};
}
void seed(GlobalMemory& memory,const std::shared_ptr<const MapQuery>& map,
          const std::vector<StateVec>& states){
  memory.updateMap(map,TerrainQueryReuse::ImmutableSnapshot);
  // The owned graph is mutable. This fixture seeds historical coordinates
  // without reprojecting them through today's map or inventing accepted links.
  auto& graph=const_cast<GraphManager&>(memory.graph());
  for(std::size_t i=0;i<states.size();++i)graph.addVertex(new Vertex(int(i),states[i]));
}
std::string graphState(const GlobalMemory& memory){
  std::ostringstream out;out<<std::hexfloat;
  std::vector<int> ids;for(const auto& item:memory.graph().vertices_map_)ids.push_back(item.first);
  std::sort(ids.begin(),ids.end());
  for(int id:ids){
    const auto& v=*memory.graph().vertices_map_.at(id);
    out<<id<<':'<<v.state.transpose()<<':'<<(v.parent?v.parent->id:-1)<<':'<<v.distance<<':'
       <<int(v.type)<<':'<<v.is_hanging<<':'<<v.children.size()<<';';
    for(const auto* child:v.children)out<<child->id<<',';
  }
  for(const auto& item:memory.graph().edge_map_)for(const auto& edge:item.second)
    out<<item.first<<'>'<<edge.first<<':'<<edge.second<<';';
  out<<memory.snapshotRevision()<<':'<<memory.visitRevision()<<':'<<memory.lastVisitedMarker().transpose();
  for(const auto& state:memory.visitedStates())out<<';'<<state.transpose();
  const auto& changes=memory.lastMapEdgeUpdate();
  out<<':'<<changes.invalidated<<':'<<changes.restored<<':'<<changes.pending;
  return out.str();
}
void accepted(GlobalMemory& memory,const std::vector<StateVec>& path,int anchor){
  const auto original_count=memory.graph().vertices_map_.size();
  std::map<int,StateVec> original;
  for(const auto& item:memory.graph().vertices_map_)original.emplace(item.first,item.second->state);
  const auto visits=memory.visitRevision();ReferencePathDiagnostics diagnostic;int endpoint=-1;
  require(memory.addReferencePath(path,&endpoint,&diagnostic),"Expected reference admission failed");
  require(diagnostic.reason=="ok","Successful admission kept a failure reason");
  const auto* first=memory.graph().vertices_map_.at(int(original_count));
  require(first->parent&&first->parent->id==anchor,"Wrong fallback/nearest anchor selected");
  require(same(first->state,path.front())&&same(memory.graph().vertices_map_.at(endpoint)->state,path.back()),
          "Reference XYZ or yaw changed");
  for(const auto& item:original)require(same(memory.graph().vertices_map_.at(item.first)->state,item.second),
                                      "Historical vertex geometry changed");
  require(memory.visitRevision()==visits&&memory.visitedStates().empty(),"Reference insertion fabricated visits");
}
void rejected(GlobalMemory& memory,const std::vector<StateVec>& path){
  const auto before=graphState(memory);ReferencePathDiagnostics diagnostic;int endpoint=-17;
  require(!memory.addReferencePath(path,&endpoint,&diagnostic),"Invalid connector was admitted");
  require(diagnostic.reason=="anchor_unreachable"&&diagnostic.segment==0&&
          same(diagnostic.from,bad_nearest)&&same(diagnostic.to,path.front()),
          "Failure did not preserve the original nearest-anchor diagnosis");
  require(graphState(memory)==before,"Failed fallback mutated graph state");
  // The existing API clears the caller's output ID at entry, including every
  // rejected request. A sentinel is useful to verify that contract, not to
  // require it to survive a failed admission.
  require(endpoint==-1,"Failed fallback did not clear the output endpoint ID");
}
void admissibleAlternatives(){
  const auto p=parameters();auto map=terrainMap();TerrainProjector terrain(map,p.groundParameters());
  require(terrain.validateFixedEdge(bad_nearest,root,true).status==GroundStatus::Steep&&
          terrain.validateFixedEdge(root,bad_nearest,true).status==GroundStatus::Steep,
          "The 51-degree nearest connector is not rejected in both directions");
  // Equal-distance fallback sites are ordered by ID, not kd-tree traversal.
  GlobalMemory tied(p);seed(tied,map,{StateVec(-.5,0.,.8,.2),bad_nearest,StateVec(0.,-.5,.8,.3)});
  accepted(tied,reference(),0);
  // Distance precedes ID when a later-ID vertex is closer.
  GlobalMemory closer(p);seed(closer,map,{StateVec(-.5,0.,.8,.2),bad_nearest,StateVec(-.3,0.,.8,.3)});
  accepted(closer,reference(),2);
  // A usable nearest site takes the old branch even with other valid anchors.
  GlobalMemory nearest(p);seed(nearest,map,{StateVec(-.5,0.,.8,.2),StateVec(.02,0.,.8,.1)});
  accepted(nearest,reference(),1);
}
void rejectedAlternatives(){
  const auto p=parameters();
  for(const auto status:{VoxelStatus::Occupied,VoxelStatus::Unknown}){
    auto map=terrainMap();const int column=map->index({-.25,0.,0.}).x();
    for(int y=0;y<40;++y)for(int z=0;z<50;++z)map->set({column,y,z},status);
    const StateVec far(-.5,0.,.8,.2);TerrainProjector terrain(map,p.groundParameters());
    const auto expected=status==VoxelStatus::Occupied?GroundStatus::Occupied:GroundStatus::Unknown;
    require(terrain.validateFixedEdge(far,root,true).status==expected,
            "Barrier fixture does not reject its candidate connector");
    GlobalMemory memory(p);seed(memory,map,{far,bad_nearest});rejected(memory,reference());
  }
  auto map=terrainMap();
  const StateVec steep(-.08,0.,.84,.2);TerrainProjector terrain(map,p.groundParameters());
  require(terrain.validateFixedEdge(steep,root,true).status==GroundStatus::Steep,
          "Over-slope fixture does not reject its candidate connector");
  GlobalMemory slope(p);seed(slope,map,{steep,bad_nearest});rejected(slope,reference());
  GlobalMemory outside(p);seed(outside,map,{StateVec(-1.,0.,.8,.2),bad_nearest});
  rejected(outside,reference());
  GlobalMemory alone(p);seed(alone,map,{bad_nearest});rejected(alone,reference());
}
void directionalAlternative(){
  auto p=parameters();p.ground.max_negative_inclination=.1;
  auto map=terrainMap(.25);TerrainProjector terrain(map,p.groundParameters());
  const StateVec far(-.5,0.,.675,.2);
  require(terrain.validateFixedEdge(far,root,true).admissible()&&
          terrain.validateFixedEdge(root,far,true).status==GroundStatus::Steep,
          "Directional fixture must allow uphill but reject downhill");
  GlobalMemory memory(p);seed(memory,map,{far,bad_nearest});accepted(memory,reference(.25),0);
  const auto home=memory.homing(reference(.25).back());
  require(!home.success&&home.reason=="global_target_unreachable",
          "Fallback insertion converted a one-way link into downhill permission");
}
}
int main(){try{
  admissibleAlternatives();rejectedAlternatives();directionalAlternative();
  std::cout<<"{\"status\":\"passed\",\"nearest_51_degree_rejected\":true,"
      "\"fallback_distance_then_id\":true,\"valid_nearest_preserved\":true,"
      "\"reference_and_historical_geometry_preserved\":true,\"obstacle_rejected\":true,"
      "\"unknown_rejected\":true,\"over_slope_rejected\":true,\"radius_preserved\":true,"
      "\"failed_admission_graph_unchanged\":true,\"directional_slope_preserved\":true}\n";
  return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
