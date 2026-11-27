// Global selection must budget the time after its actual gain/route work.
#include "gbplanner_local/global_memory.hpp"
#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>

using namespace gbplanner2;
namespace {
void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
class ObservedMap final:public MapQuery {
 public:
  ObservedMap():map_(.25,Eigen::Vector3d(-1.,-1.,-1.),Eigen::Vector3i(28,8,8)){
    map_.fill(VoxelStatus::Free);
  }
  double resolution()const override {++queries;return map_.resolution();}
  Eigen::Vector3d minimum()const override {++queries;return map_.minimum();}
  Eigen::Vector3d maximum()const override {++queries;return map_.maximum();}
  VoxelStatus voxel(const Eigen::Vector3d& point)const override {++queries;return map_.voxel(point);}
  VoxelStatus box(const Eigen::Vector3d& point,const Eigen::Vector3d& size)const override {
    ++queries;return map_.box(point,size);
  }
  VoxelStatus path(const Eigen::Vector3d& a,const Eigen::Vector3d& b,const Eigen::Vector3d& size)const override {
    ++queries;if(on_path)on_path(a,b);return map_.path(a,b,size);
  }
  void ray(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
           const std::function<bool(const Eigen::Vector3d&,VoxelStatus)>& visit)const override {
    ++queries;map_.ray(a,b,visit);
  }
  mutable std::size_t queries{0};
  std::function<void(const Eigen::Vector3d&,const Eigen::Vector3d&)> on_path;
 private:
  VoxelMap map_;
};
enum class Mode { Static,GainAges,Paused,FinalRouteAges,ReturnRefinementAges,ReturnRefinementPaused,
                  InvalidClock,StaleBattery,Ignored };
struct Outcome {
  GlobalResult result;
  std::size_t queries{0};
  int gain_calls{0},budget_reads{0};
  double clock{10.};
  bool advanced{false};
};
Outcome run(Mode mode){
  Outcome out;
  Parameters p;p.robot_size={.1,.1,.1};p.overshoot=0.;p.edge_max=1.;
  p.neighbor_radius=1.1;p.neighbor_max=1.1;
  p.refinement.enabled=mode==Mode::ReturnRefinementAges||mode==Mode::ReturnRefinementPaused;
  p.refinement.safety_extension={.5,.5,.5};
  auto map=std::make_shared<ObservedMap>();
  GlobalMemory memory(p);memory.updateMap(map);
  const StateVec home(0.,0.,0.,0.),target(4.75,0.,0.,0.);
  require(memory.setHomingPosition(home),"Could not set fixture home");
  const int target_id=memory.addSemanticPath({home,target},planner_semantic_msgs::msg::SemanticClass::K_DOOR);
  require(target_id>0&&memory.frontierCount()==1,"Could not retain the single fixture frontier");
  MissionBudget mission(30.,.5,mode==Mode::StaleBattery);
  if(mode==Mode::StaleBattery)
    require(mission.updateBattery(30.,out.clock,out.clock),"Could not seed fresh battery observation");
  const auto initial=mission.snapshot(out.clock);
  require(initial.valid&&initial.clock_valid&&initial.remaining==30.,"Fixture did not start with 30 seconds");
  GlobalTimeConstraint constraint{initial.remaining,1.,true};
  if(mode!=Mode::Static)constraint.refresh_budget=[&]{
    ++out.budget_reads;
    if(mode==Mode::Ignored)return BudgetSnapshot{};
    return mission.snapshot(out.clock);
  };
  map->on_path=[&](const Eigen::Vector3d& a,const Eigen::Vector3d& b){
    // Automatic selection must read again after the winning route is built.
    const bool selected_route=mode==Mode::FinalRouteAges&&out.budget_reads==1;
    // This reverse check belongs to the explicit target's return refinement,
    // inside feasible(). An early snapshot at feasible entry would be stale.
    const bool return_refinement=mode==Mode::ReturnRefinementAges&&a.x()>4.7&&b.x()<a.x()-1e-8;
    if(!out.advanced&&(selected_route||return_refinement)){out.clock+=1.;out.advanced=true;}
  };
  const auto gain=[&](const StateVec&){
    ++out.gain_calls;
    if(mode==Mode::Static||mode==Mode::GainAges||mode==Mode::StaleBattery){
      out.clock+=1.;out.advanced=true;
    }else if(mode==Mode::InvalidClock){out.clock-=1.;out.advanced=true;}
    VolumetricGain value;value.reset();value.gain=10.;value.num_unknown_voxels=1;value.is_frontier=true;
    return value;
  };
  map->queries=0;
  const int selected=p.refinement.enabled?target_id:-1;
  out.result=memory.selectFrontier(home,gain,selected,mode==Mode::Ignored?nullptr:&constraint);
  out.queries=map->queries;
  require(out.gain_calls==1,"Budget refresh added or repeated a gain evaluation");
  return out;
}
bool samePath(const GlobalResult& a,const GlobalResult& b){
  if(a.path.size()!=b.path.size())return false;
  for(std::size_t i=0;i<a.path.size();++i)if((a.path[i]-b.path[i]).norm()!=0.)return false;
  return true;
}
}
int main(){
  try{
    const auto fixed=run(Mode::Static),aged=run(Mode::GainAges),paused=run(Mode::Paused);
    require(fixed.result.success&&fixed.budget_reads==0&&
      std::abs(2.*fixed.result.distance-9.5)<1e-9,"Static budget compatibility or 9.5-second fixture changed");
    require(!aged.result.success&&aged.result.path.empty()&&aged.result.reason=="global_time_insufficient"&&
      aged.advanced&&aged.clock==11.&&aged.budget_reads==1,"Gain work was not charged to fresh mission time");
    require(paused.result.success&&paused.clock==10.&&paused.budget_reads==2&&
      samePath(fixed.result,paused.result)&&fixed.queries==paused.queries,
      "An unchanged simulated clock consumed budget, altered the path, or added map queries");
    const auto final=run(Mode::FinalRouteAges);
    require(final.advanced&&final.budget_reads==2&&!final.result.success&&final.result.path.empty()&&
      final.result.reason=="global_time_insufficient","Selected route reused its pre-routing budget");
    const auto returning=run(Mode::ReturnRefinementAges),returning_paused=run(Mode::ReturnRefinementPaused);
    require(returning_paused.result.success&&returning_paused.budget_reads==1&&
      returning.queries==returning_paused.queries&&returning.advanced&&returning.budget_reads==1&&
      !returning.result.success&&returning.result.path.empty()&&
      returning.result.reason=="global_time_insufficient"&&returning.result.refinement.attempted,
      "Explicit target budget was read before its return refinement");
    const auto invalid=run(Mode::InvalidClock),stale=run(Mode::StaleBattery),ignored=run(Mode::Ignored);
    require(!invalid.result.success&&invalid.result.path.empty()&&invalid.result.reason=="mission_clock_invalid",
      "Invalid live clock was allowed to reuse the old valid budget");
    require(!stale.result.success&&stale.result.path.empty()&&stale.result.reason=="battery_status_stale",
      "Invalid live battery snapshot was allowed to reuse the old valid budget");
    require(ignored.result.success&&ignored.budget_reads==0,
      "Disabled/ignored budget invoked a provider or rejected the route");
    std::cout<<"{\"status\":\"passed\",\"initial_remaining_s\":30,\"round_trip_s\":9.5,\"elapsed_ros_s\":1"
      <<",\"gain_refresh_ages_budget\":true,\"paused_clock_preserves_budget\":true"
      <<",\"selected_route_final_recheck\":true,\"explicit_return_refinement_precedes_budget_read\":true"
      <<",\"invalid_clock_rejected\":true,\"stale_battery_rejected\":true,\"ignored_provider_not_called\":true"
      <<",\"static_core_compatibility\":true,\"extra_map_queries\":0,\"gain_calls_per_case\":1"
      <<",\"paired_map_queries\":"<<fixed.queries<<"}\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
