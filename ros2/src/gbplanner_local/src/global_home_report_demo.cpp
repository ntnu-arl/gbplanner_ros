// A global request must budget return routes on the graph after current attachment.
#include "gbplanner_local/global_memory.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace gbplanner2;
namespace {
void require(bool condition,const std::string& message){if(!condition)throw std::runtime_error(message);}
Parameters parameters(){
  Parameters p;p.robot_size={.1,.1,.1};p.overshoot=0.;
  p.edge_min=.15;p.edge_max=.8;p.neighbor_radius=.8;p.neighbor_min=.15;p.neighbor_max=.8;
  p.refinement.enabled=false;p.tangent_yaw=false;p.validate();return p;
}
const std::vector<StateVec> kReference{{0.,0.,0.,0.},{0.,1.,0.,0.},{1.,1.,0.,0.},{1.,0.,0.,0.}};
const StateVec kCurrent(.5,0.,0.,0.);
std::shared_ptr<VoxelMap> freeMap(){
  auto map=std::make_shared<VoxelMap>(.05,Eigen::Vector3d(-1.,-1.,-1.),Eigen::Vector3i(60,60,40));
  map->fill(VoxelStatus::Free);return map;
}
bool linked(const GlobalMemory& memory,int from,int to){
  const auto found=memory.graph().edge_map_.find(from);
  if(found==memory.graph().edge_map_.end())return false;
  for(const auto& edge:found->second)if(edge.first==to)return true;
  return false;
}
struct Outcome {
  int target{-1},gain_calls{0},budget_reads{0};
  double outward{0.},home{0.},spare{0.};
};
Outcome run(bool automatic,const std::string& snapshot){
  Outcome out;GlobalMemory memory(parameters());memory.updateMap(freeMap());
  require(memory.setHomingPosition(kReference.front()),"Could not set fixture home");
  out.target=memory.addSemanticPath(kReference,planner_semantic_msgs::msg::SemanticClass::K_DOOR);
  require(out.target==3&&memory.graph().vertices_map_.size()==4&&memory.edgeCount()==3&&
          memory.frontierCount()==1,"Expected one frontier on the four-vertex U-shaped reference chain");
  for(int id=0;id<3;++id)require(linked(memory,id,id+1),"Original reference chain is missing an edge");
  require(!linked(memory,0,out.target),"Initial graph already contains a home shortcut");
  if(!snapshot.empty()){
    // Preserve the graph before the first current attachment. The runtime
    // fixture can load it after receiving odometry, then issue PlannerGlobal.
    const auto saved=memory.saveGraph(snapshot,"map");
    require(saved.success,"Could not save fixture graph: "+saved.reason);
  }
  const auto visit_revision=memory.visitRevision();
  const auto gain=[&](const StateVec&){
    ++out.gain_calls;
    VolumetricGain value;value.reset();value.gain=10.;value.num_unknown_voxels=1;value.is_frontier=true;
    return value;
  };
  GlobalTimeConstraint constraint{22.5,1.,true,[&]{
    ++out.budget_reads;
    BudgetSnapshot budget;budget.valid=true;budget.clock_valid=true;budget.remaining=22.5;budget.reason="ok";
    return budget;
  }};
  // Before attachment: outward .5 + old home 3 = 3.5 seconds, which would
  // violate the 20-second reserve. The current pose adds both .5 m links,
  // yielding .5 outward + 1 home = 1.5 seconds in this very first request.
  double old_spare=0.;
  require(!MissionBudget::sufficient(constraint.remaining,3.5,old_spare),
          "Fixture no longer distinguishes the stale home report");
  const auto selected=memory.selectFrontier(kCurrent,gain,automatic?-1:out.target,&constraint);
  require(selected.success,"First global request rejected the newly connected return route: "+selected.reason);
  require(selected.target==out.target&&selected.path.size()==2&&
          (selected.path.front()-kCurrent).norm()<1e-9&&
          (selected.path.back()-kReference.back()).norm()<1e-9&&
          std::abs(selected.distance-.5)<1e-9,"Selected route changed its actual current pose or destination");
  require(memory.graph().vertices_map_.size()==5&&memory.edgeCount()==5&&
          linked(memory,4,0)&&linked(memory,4,out.target),"Current attachment did not add exactly the two shortcuts");
  require(out.gain_calls==1&&out.budget_reads==(automatic?2:1),
          "Home-report correction repeated gain work or changed live budget checks");
  const auto returning=memory.homing(kReference.back());
  require(returning.success&&std::abs(returning.distance-1.)<1e-9,
          "Selected target does not have the expected one-meter return route");
  out.outward=selected.distance;out.home=returning.distance;
  require(MissionBudget::sufficient(constraint.remaining,(out.outward+out.home)/constraint.speed,out.spare)&&
          std::abs(out.spare-21.)<1e-9,"Accepted route did not preserve the mission reserve");
  require(memory.visitedStates().empty()&&memory.visitRevision()==visit_revision,
          "Global routing invented an odometry or visit event");
  return out;
}
}
int main(int argc,char** argv){
  try{
    require(argc==1||(argc==3&&std::string(argv[1])=="--snapshot"&&std::string(argv[2]).size()>0),
            "Usage: global_home_report_demo [--snapshot SNAPSHOT.gbp2mem]");
    const std::string snapshot=argc==3?argv[2]:"";
    // Each branch starts with a fresh four-point graph. Success cannot rely
    // on a previous failed call having already inserted the current pose.
    const auto explicit_target=run(false,snapshot),automatic=run(true,"");
    require(explicit_target.target==automatic.target,"Fixture target identity differs between branches");
    std::cout<<std::setprecision(15)
      <<"{\"status\":\"passed\",\"explicit_first_request\":true,\"automatic_first_request\":true"
      <<",\"target_id\":"<<explicit_target.target<<",\"snapshot_saved\":"<<(snapshot.empty()?"false":"true")
      <<",\"frame\":\"map\",\"seed_vertices\":4,\"seed_edges\":3,\"connected_vertices\":5,\"connected_edges\":5"
      <<",\"home_before_attachment_m\":3,\"home_after_attachment_m\":"<<explicit_target.home
      <<",\"outward_m\":"<<explicit_target.outward<<",\"old_cost_s\":3.5,\"actual_cost_s\":1.5"
      <<",\"remaining_s\":22.5,\"speed_m_s\":1,\"spare_s\":"<<explicit_target.spare
      <<",\"explicit_budget_reads\":"<<explicit_target.budget_reads
      <<",\"automatic_budget_reads\":"<<automatic.budget_reads<<",\"gain_calls_per_request\":1"
      <<",\"visit_history_unchanged\":true,\"parameters\":{\"sampling.seed\":42,\"robot.size\":[0.1,0.1,0.1]"
      <<",\"planning.edge_min\":0.15,\"planning.edge_max\":0.8,\"planning.neighbor_radius\":0.8"
      <<",\"planning.neighbor_min\":0.15,\"planning.neighbor_max\":0.8,\"planning.overshoot\":0"
      <<",\"planning.path_safety_enhance_enable\":false,\"planning.tangent_yaw\":false}"
      <<",\"map\":{\"resolution\":0.05,\"origin\":[-1,-1,-1],\"dimensions\":[60,60,40]}"
      <<",\"current\":[0.5,0,0,0],\"reference\":[[0,0,0,0],[0,1,0,0],[1,1,0,0],[1,0,0,0]]}\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
