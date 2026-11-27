// Focused checks for strict ground-edge reuse in GlobalMemory.
// No alternate planner or loosened collision/support policy is used.
#include "gbplanner_local/global_memory.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace gbplanner2;
namespace {
void require(bool ok,const std::string& message){if(!ok)throw std::runtime_error(message);}
struct Calls {
  std::uint64_t voxel{0},box{0},path{0},ray{0},tsdf{0},simplex{0};
  std::uint64_t total()const{return voxel+box+path+ray+tsdf+simplex;}
};
// Counts calls at the MapQuery boundary, not VoxelMap's internal cell visits.
class CountingMap final:public MapQuery {
 public:
  explicit CountingMap(std::shared_ptr<VoxelMap> map):map(std::move(map)){}
  double resolution()const override{return map->resolution();}
  Eigen::Vector3d minimum()const override{return map->minimum();}
  Eigen::Vector3d maximum()const override{return map->maximum();}
  VoxelStatus voxel(const Eigen::Vector3d& p)const override{++calls.voxel;return map->voxel(p);}
  VoxelStatus box(const Eigen::Vector3d& p,const Eigen::Vector3d& size)const override{
    ++calls.box;return map->box(p,size);
  }
  VoxelStatus path(const Eigen::Vector3d& a,const Eigen::Vector3d& b,const Eigen::Vector3d& size)const override{
    ++calls.path;return map->path(a,b,size);
  }
  void ray(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
           const std::function<bool(const Eigen::Vector3d&,VoxelStatus)>& visit)const override{
    ++calls.ray;map->ray(a,b,visit);
  }
  std::optional<TsdfDistance> tsdfDistance(const Eigen::Vector3d& p)const override{
    ++calls.tsdf;return map->tsdfDistance(p);
  }
  std::optional<TsdfDistance> tsdfDistanceSimplex(const Eigen::Vector3d& p)const override{
    ++calls.simplex;return map->tsdfDistanceSimplex(p);
  }
  void clear()const{calls={};}
  std::shared_ptr<VoxelMap> map;
  mutable Calls calls;
};
Parameters parameters(){
  Parameters p;p.ground_robot=true;p.robot_size={.1,.1,.1};p.overshoot=0.;
  p.edge_min=.15;p.edge_max=.8;p.neighbor_radius=.8;p.neighbor_min=.15;p.neighbor_max=.8;
  p.ground.max_ground_height=.8;p.ground.robot_height=.6;
  p.ground.max_inclination=.9;p.ground.max_negative_inclination=.9;
  p.refinement.enabled=false;p.tangent_yaw=false;p.validate();return p;
}
std::shared_ptr<VoxelMap> flat(){
  auto map=std::make_shared<VoxelMap>(.05,Eigen::Vector3d(-.5,-2.5,-.5),Eigen::Vector3i(40,80,40));
  const auto dims=map->dimensions();
  for(int z=0;z<dims.z();++z)for(int y=0;y<dims.y();++y)for(int x=0;x<dims.x();++x){
    const Eigen::Vector3i index(x,y,z);
    map->set(index,map->center(index).z()<0.?VoxelStatus::Occupied:VoxelStatus::Free);
  }
  return map;
}
std::shared_ptr<VoxelMap> ramp(){
  auto map=std::make_shared<VoxelMap>(.05,Eigen::Vector3d(-2.,-1.,-1.),Eigen::Vector3i(80,40,80));
  const auto dims=map->dimensions();
  for(int z=0;z<dims.z();++z)for(int y=0;y<dims.y();++y)for(int x=0;x<dims.x();++x){
    const Eigen::Vector3i index(x,y,z);const auto point=map->center(index);
    const double height=.5*std::clamp(point.x()+1.,0.,2.);
    map->set(index,point.z()<height?VoxelStatus::Occupied:VoxelStatus::Free);
  }
  return map;
}
planner_msgs::msg::Graph snapshot(GlobalMemory& memory){
  planner_msgs::msg::Graph graph;memory.graphMessage(graph);return graph;
}
int exactId(const GlobalMemory& memory,const StateVec& state){
  for(const auto& item:memory.graph().vertices_map_)
    if((item.second->state.array()==state.array()).all())return item.first;
  throw std::runtime_error("Fixture reference waypoint was changed");
}
bool linked(const GlobalMemory& memory,int from,int to){
  const auto found=memory.graph().edge_map_.find(from);
  if(found==memory.graph().edge_map_.end())return false;
  for(const auto& edge:found->second)if(edge.first==to)return true;
  return false;
}
void sameRoute(const GlobalResult& a,const GlobalResult& b){
  require(a.success==b.success&&a.reason==b.reason&&a.target==b.target&&
          a.distance==b.distance&&a.path.size()==b.path.size(),"Repeated route changed its result");
  for(std::size_t i=0;i<a.path.size();++i)
    require((a.path[i].array()==b.path[i].array()).all(),"Repeated route changed its geometry");
}
struct FlatResult {Calls first_insert,repeated_insert,first_route,repeated_route;double free,blocked,recovered;};
FlatResult flatChecks(){
  auto counted=std::make_shared<CountingMap>(flat());const auto p=parameters();
  TerrainProjector terrain(counted,p.groundParameters());
  std::vector<StateVec> reference;
  for(const auto& xy:std::vector<Eigen::Vector2d>{{0.,0.},{0.,1.},{1.,1.},{1.,0.},{.5,0.},{.5,-1.},{.5,-2.}}){
    const auto projected=terrain.project(StateVec(xy.x(),xy.y(),.8,.1+reference.size()));
    require(projected.status==GroundStatus::Supported,"Flat fixture lacks ground support");
    reference.push_back(projected.projected_state);
  }
  GlobalMemory memory(p);memory.updateMap(counted);
  require(memory.setHomingPosition(reference.front()),"Could not set flat fixture home");
  FlatResult out;counted->clear();
  require(memory.addReferencePath(reference),"First reference insertion failed");out.first_insert=counted->calls;
  const auto graph=snapshot(memory);const int home=exactId(memory,reference.front()),junction=exactId(memory,reference[4]);
  require(linked(memory,home,junction),"Flat fixture did not build the shortcut");
  counted->clear();require(memory.addReferencePath(reference),"Repeated reference insertion failed");
  out.repeated_insert=counted->calls;
  require(snapshot(memory)==graph,"Repeated reference insertion changed graph contents");
  counted->clear();const auto first=memory.homing(reference.back());out.first_route=counted->calls;
  require(first.success,"First flat homing route failed");
  const auto routed_graph=snapshot(memory);
  counted->clear();const auto second=memory.homing(reference.back());out.repeated_route=counted->calls;
  sameRoute(first,second);require(snapshot(memory)==routed_graph,"Repeated route changed graph contents");
  out.free=first.distance;

  // Mutate the SAME backing object, then explicitly notify GlobalMemory.
  // The body obstacle intersects H-C only, not the retained U-shaped chain.
  const auto obstacle=counted->map->index({.25,0.,.6});
  counted->map->set(obstacle,VoxelStatus::Occupied);counted->clear();memory.updateMap(counted);
  require(counted->calls.total()>0,"Changed map did not revalidate ground geometry");
  require(!linked(memory,home,junction),"Cached free shortcut survived a blocking map update");
  const auto blocked=memory.homing(reference.back());
  require(blocked.success&&blocked.distance>out.free+1.,"Blocked shortcut did not force the retained detour");
  out.blocked=blocked.distance;
  counted->map->set(obstacle,VoxelStatus::Free);memory.updateMap(counted);
  require(linked(memory,home,junction),"Previously blocked shortcut was not rechecked after map recovery");
  const auto recovered=memory.homing(reference.back());
  require(recovered.success&&std::abs(recovered.distance-out.free)<1e-12,"Recovered map did not recover the route");
  out.recovered=recovered.distance;return out;
}
void slopeChecks(){
  auto counted=std::make_shared<CountingMap>(ramp());auto p=parameters();
  p.robot_size={.3,.3,.3};p.ground.max_negative_inclination=.35;
  TerrainProjector terrain(counted,p.groundParameters());
  const auto low=terrain.project(StateVec(-1.5,0.,2.,0.)),high=terrain.project(StateVec(1.5,0.,2.,0.));
  require(low.status==GroundStatus::Supported&&high.status==GroundStatus::Supported,"Slope fixture lacks support");
  const auto uphill=terrain.edge(low.projected_state,high.projected_state);
  require(uphill.admissible()&&uphill.polyline.size()>2,"Slope fixture lacks an admissible terrain path");
  // Reference insertion checks its supplied geometry without projecting a
  // straight low-to-high chord onto the piecewise flat/ramp/flat terrain.
  // Seed the already checked terrain polyline, including its endpoint yaw.
  GlobalMemory memory(p);memory.updateMap(counted);
  require(memory.setHomingPosition(low.projected_state)&&
          memory.addReferencePath(uphill.polyline),"Could not seed one-way slope");
  const int high_id=exactId(memory,uphill.polyline.back());
  const auto gain=[](const StateVec&){return VolumetricGain{};};
  require(memory.selectFrontier(low.projected_state,gain,high_id).success,"Allowed uphill route failed");
  const auto rejected=memory.homing(high.projected_state);
  require(!rejected.success&&rejected.reason=="global_target_unreachable","Forbidden downhill route was accepted");
  p.ground.max_negative_inclination=.9;counted->clear();memory.setPlanningParameters(p);
  require(counted->calls.total()>0,"Changed slope policy did not revalidate geometry");
  require(memory.homing(high.projected_state).success,"Old rejecting cache survived a permissive policy change");
  p.ground.max_negative_inclination=.35;memory.setPlanningParameters(p);
  const auto rejected_again=memory.homing(high.projected_state);
  require(!rejected_again.success&&rejected_again.reason=="global_target_unreachable",
          "Old accepting cache survived a restrictive policy change");
  require(memory.selectFrontier(low.projected_state,gain,high_id).success,"Restriction incorrectly removed uphill route");
}
void overshootCheck(){
  // Isolate a neighbor shortcut H-C from the original U-shaped reference.
  // Its true endpoint C is free; the nonzero extension beyond C is occupied.
  auto map=flat();auto p=parameters();p.overshoot=.2;
  const std::vector<StateVec> reference{{0.,0.,.8,0.},{0.,1.,.8,0.},{1.,1.,.8,0.},
                                       {1.,0.,.8,0.},{.5,0.,.8,0.}};
  // The x=.7 point lies on D-C, so use the opposite H-side extension when
  // checking from H toward non-root C: H extends backwards to x=-.2.
  map->set(map->index({-.175,0.,.6}),VoxelStatus::Occupied);
  TerrainProjector terrain(map,p.groundParameters());
  require(terrain.edge(reference.front(),reference.back()).admissible(),
          "Overshoot obstacle also blocks the actual shortcut");
  StateVec extended=reference.front();extended.x()-=p.overshoot;
  require(!terrain.edge(extended,reference.back()).admissible(),"Overshoot obstacle does not block extension");
  // Insert with a small radius first, so only fixed reference links exist.
  auto seed=p;seed.neighbor_radius=.2;
  GlobalMemory memory(seed);memory.updateMap(map);
  require(memory.setHomingPosition(reference.front())&&memory.addReferencePath(reference),"Could not seed overshoot fixture");
  const int home=exactId(memory,reference.front()),target=exactId(memory,reference.back());
  memory.setPlanningParameters(p);
  // Query at H makes connect(H) consider the H->C extension. It must not use
  // an unextended cached admissible edge as proof that this extension is free.
  const auto gain=[](const StateVec&){return VolumetricGain{};};
  const auto route=memory.selectFrontier(reference.front(),gain,target);
  require(route.success,"Retained U-shaped route unexpectedly failed");
  require(!linked(memory,home,target),"Nonzero overshoot was bypassed by the strict base-edge cache");
}
void writeCalls(const Calls& c){
  std::cout<<"{\"total\":"<<c.total()<<",\"voxel\":"<<c.voxel<<",\"box\":"<<c.box
    <<",\"path\":"<<c.path<<",\"ray\":"<<c.ray<<",\"tsdf\":"<<c.tsdf<<",\"simplex\":"<<c.simplex<<'}';
}
}
int main(){try{
  const auto flat=flatChecks();slopeChecks();overshootCheck();
  std::cout<<std::setprecision(17)<<"{\"status\":\"passed\",\"count_scope\":\"MapQuery calls, not internal voxel visits\""
    <<",\"first_reference\":";writeCalls(flat.first_insert);
  std::cout<<",\"repeated_reference\":";writeCalls(flat.repeated_insert);
  std::cout<<",\"first_route\":";writeCalls(flat.first_route);
  std::cout<<",\"repeated_route\":";writeCalls(flat.repeated_route);
  std::cout<<",\"free_distance\":"<<flat.free<<",\"blocked_distance\":"<<flat.blocked
    <<",\"recovered_distance\":"<<flat.recovered
    <<",\"graph_and_path_stable\":true,\"same_map_object_revalidated\":true"
    <<",\"directional_policy_revalidated\":true,\"nonzero_overshoot_obstacle_rejected\":true}\n";
  return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
