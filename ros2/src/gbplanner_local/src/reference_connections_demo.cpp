// The neighbor-edge stage of Rrg::addRefPathToGraph (rrg.cpp:4330-4338).
// Run after building; optionally supply the retained graph snapshot filename.
#include "gbplanner_local/global_memory.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

using namespace gbplanner2;
namespace {
void require(bool condition,const std::string& message){if(!condition)throw std::runtime_error(message);}
Parameters parameters(){
  Parameters p;p.robot_size={.1,.1,.1};p.overshoot=0.;
  p.edge_min=.15;p.edge_max=.8;p.neighbor_radius=.8;p.neighbor_min=.15;p.neighbor_max=.8;
  p.refinement.enabled=false;p.validate();return p;
}
const std::vector<StateVec> kReference{
    {0.,0.,0.,.1},{0.,1.,0.,.2},{1.,1.,0.,.3},{1.,0.,0.,.4},
    {.5,0.,0.,.5},{.5,-1.,0.,.6},{.5,-2.,0.,.7}};
std::shared_ptr<VoxelMap> aerialMap(VoxelStatus shortcut_status=VoxelStatus::Free){
  auto map=std::make_shared<VoxelMap>(.05,Eigen::Vector3d(-.5,-2.5,-.5),Eigen::Vector3i(40,80,20));
  map->fill(VoxelStatus::Free);
  // This cell intersects only the missing H-D shortcut. Every original leg
  // remains clear, including the .1 m robot body at H and D.
  map->set(map->index({.25,0.,0.}),shortcut_status);return map;
}
int exactId(const GlobalMemory& memory,const StateVec& state){
  for(const auto& item:memory.graph().vertices_map_)
    if((item.second->state.array()==state.array()).all())return item.first;
  throw std::runtime_error("Reference XYZ/yaw waypoint was changed");
}
bool linked(const GlobalMemory& memory,int from,int to){
  const auto found=memory.graph().edge_map_.find(from);
  if(found==memory.graph().edge_map_.end())return false;
  for(const auto& edge:found->second)if(edge.first==to)return true;
  return false;
}
void preserved(const GlobalMemory& memory,const std::vector<StateVec>& reference){
  for(std::size_t i=0;i<reference.size();++i){
    const int id=exactId(memory,reference[i]);
    if(i)require(linked(memory,exactId(memory,reference[i-1]),id),"Original reference chain edge was removed");
  }
}
void seed(GlobalMemory& memory,std::shared_ptr<const MapQuery> map,const std::vector<StateVec>& reference){
  memory.updateMap(std::move(map));
  require(memory.setHomingPosition(reference.front()),"Could not set fixture home");
  ReferencePathDiagnostics diagnostic;
  const bool stored=memory.addReferencePath(reference,nullptr,&diagnostic);
  require(stored,"Could not store reference: "+diagnostic.reason);
  preserved(memory,reference);
}
double homeLength(GlobalMemory& memory,const VoxelMap& map,const Parameters& p,double expected){
  const auto result=memory.homing(kReference.back());
  require(result.success,"Homing failed: "+result.reason);
  require(std::abs(result.distance-expected)<1e-9,"Homing length differs from expected "+std::to_string(expected));
  for(std::size_t i=1;i<result.path.size();++i)
    require(map.path(result.path[i-1].head<3>(),result.path[i].head<3>(),p.robot_size)==VoxelStatus::Free,
            "Homing used an occupied or unknown segment");
  require(memory.frontierCount()==0,"Reference insertion invented a frontier");
  return result.distance;
}
double blockedCase(VoxelStatus status,const Parameters& p){
  const auto map=aerialMap(status);GlobalMemory memory(p);seed(memory,map,kReference);
  require(!linked(memory,exactId(memory,kReference.front()),exactId(memory,kReference[4])),
          "Neighbor stage inserted a blocked shortcut");
  return homeLength(memory,*map,p,5.5);
}
struct GroundResult {double uphill_length,downhill_length;std::size_t uphill_knots;};
GroundResult groundCase(){
  // One slope, with a gentle diagonal detour. H-D is uphill-admissible but
  // too steep downhill. The original H-A-D chain remains legal both ways.
  auto map=std::make_shared<VoxelMap>(.02,Eigen::Vector3d(-.5,-.5,-.5),Eigen::Vector3i(75,150,100));
  const auto dims=map->dimensions();
  for(int z=0;z<dims.z();++z)for(int y=0;y<dims.y();++y)for(int x=0;x<dims.x();++x){
    const Eigen::Vector3i index(x,y,z);const auto point=map->center(index);
    map->set(index,point.z()<.5*point.x()?VoxelStatus::Occupied:VoxelStatus::Free);
  }
  auto p=parameters();p.ground_robot=true;p.ground.robot_height=.4;p.ground.max_ground_height=.6;
  p.ground.max_inclination=.9;p.ground.max_negative_inclination=.2;
  TerrainProjector terrain(map,p.groundParameters());
  std::vector<StateVec> reference;
  for(const auto& xy:std::vector<Eigen::Vector2d>{{0.,0.},{0.,2.},{.5,0.}}){
    const auto projected=terrain.project(StateVec(xy.x(),xy.y(),1.,.1+reference.size()));
    require(projected.status==GroundStatus::Supported,"Ground fixture endpoint is unsupported");
    reference.push_back(projected.projected_state);
  }
  const auto uphill=terrain.edge(reference.front(),reference.back());
  const auto downhill=terrain.edge(reference.back(),reference.front());
  require(uphill.admissible()&&uphill.polyline.size()>2&&downhill.status==GroundStatus::Steep,
          "Ground fixture did not isolate directional shortcut admissibility");
  GlobalMemory memory(p);seed(memory,map,reference);
  const int low=exactId(memory,reference.front()),high=exactId(memory,reference.back());
  require(linked(memory,low,high),"Admissible uphill neighbor shortcut was not added");
  const auto gain=[](const StateVec&){return VolumetricGain{};};
  const auto outward=memory.selectFrontier(reference.front(),gain,high);
  const auto returning=memory.selectFrontier(reference.back(),gain,low);
  require(outward.success&&returning.success,"Ground directional routing failed");
  require(outward.distance<.8&&outward.path.size()>2,
          "New ground shortcut was not routed with its ordinary terrain polyline");
  require(returning.distance>4.&&returning.distance<4.2,
          "Dijkstra took a forbidden downhill shortcut instead of the original diagonal detour");
  require(std::abs(outward.distance-uphill.actual_length)<1e-9,
          "Neighbor shortcut lost its terrain arc length");
  preserved(memory,reference);
  memory.updateMap(map);
  require(linked(memory,low,high),"Map refresh removed a one-way-admissible neighbor edge");
  const auto after=memory.selectFrontier(reference.back(),gain,low);
  require(after.success&&std::abs(after.distance-returning.distance)<1e-9,
          "Map refresh lost directional ground routing");
  preserved(memory,reference);
  return {outward.distance,returning.distance,outward.path.size()};
}
}

int main(int argc,char** argv){
  try{
    require(argc<=2,"Usage: reference_connections_demo [SNAPSHOT.gbp2mem]");
    const auto p=parameters();const auto free_map=aerialMap();
    // Recreate the former chain topology with a seeding radius that cannot
    // reach H-D, then restore identical route parameters in both comparisons.
    // No alternate production implementation or map is substituted.
    auto chain_parameters=p;chain_parameters.neighbor_radius=.2;
    GlobalMemory chain(chain_parameters);seed(chain,free_map,kReference);chain.setPlanningParameters(p);
    require(!linked(chain,exactId(chain,kReference.front()),exactId(chain,kReference[4])),
            "Chain baseline unexpectedly contains the shortcut");
    const double before=homeLength(chain,*free_map,p,5.5);
    GlobalMemory connected(p);seed(connected,free_map,kReference);
    const int home=exactId(connected,kReference.front()),junction=exactId(connected,kReference[4]);
    require(linked(connected,home,junction),"Reference insertion did not add the H-D neighbor shortcut");
    const double after=homeLength(connected,*free_map,p,2.5);
    const auto vertex_count=connected.graph().vertices_map_.size();const int edge_count=connected.edgeCount();
    require(connected.addReferencePath(kReference),"Repeated reference insertion failed");
    require(connected.graph().vertices_map_.size()==vertex_count&&connected.edgeCount()==edge_count,
            "Repeated reference insertion changed already aggregated topology");
    const double occupied=blockedCase(VoxelStatus::Occupied,p),unknown=blockedCase(VoxelStatus::Unknown,p);

    const auto blocked=aerialMap(VoxelStatus::Occupied);connected.updateMap(blocked);
    require(!linked(connected,home,junction)&&connected.lastMapEdgeUpdate().invalidated==1,
            "Changed map did not suspend exactly the new shortcut");
    preserved(connected,kReference);const double invalidated=homeLength(connected,*blocked,p,5.5);
    connected.updateMap(free_map);
    require(linked(connected,home,junction)&&connected.lastMapEdgeUpdate().restored==1,
            "Observed-free map did not restore the retained shortcut");
    preserved(connected,kReference);const double recovered=homeLength(connected,*free_map,p,2.5);
    const std::string snapshot=argc==2?argv[1]:"/tmp/gbplanner_reference_connections.gbp2mem";
    const auto saved=connected.saveGraph(snapshot,"map");require(saved.success,"Snapshot save failed: "+saved.reason);
    GlobalMemory restored(p);restored.updateMap(free_map);
    const auto loaded=restored.loadGraph(snapshot,"map");require(loaded.success,"Snapshot load failed: "+loaded.reason);
    preserved(restored,kReference);require(linked(restored,home,junction),"Snapshot lost the new neighbor shortcut");
    const double persisted=homeLength(restored,*free_map,p,2.5);
    const auto ground=groundCase();
    std::cout<<std::setprecision(17)
      <<"{\"status\":\"passed\",\"reference_points\":7,\"baseline_topology\":\"chain seeded with 0.2 m radius, then restored to identical route parameters\""
      <<",\"chain_length_m\":"<<before<<",\"connected_length_m\":"<<after
      <<",\"occupied_shortcut_detour_m\":"<<occupied<<",\"unknown_shortcut_detour_m\":"<<unknown
      <<",\"invalidated_shortcut_detour_m\":"<<invalidated<<",\"recovered_length_m\":"<<recovered
      <<",\"saved_loaded_length_m\":"<<persisted<<",\"original_xyz_yaw_preserved\":true"
      <<",\"reference_chain_edges_preserved\":true,\"repeated_insertion_idempotent\":true"
      <<",\"ground\":{\"uphill_shortcut_length_m\":"<<ground.uphill_length
      <<",\"downhill_detour_length_m\":"<<ground.downhill_length<<",\"uphill_terrain_knots\":"<<ground.uphill_knots
      <<",\"forbidden_reverse_shortcut_avoided\":true,\"one_way_neighbor_survives_map_update\":true}}\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
