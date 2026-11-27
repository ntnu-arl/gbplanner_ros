// Local buildGraph's sampling budgets, empty-root cutoff and retry boundaries.
#include "gbplanner_local/local_planner.hpp"
#include <iomanip>
#include <iostream>
#include <stdexcept>

using namespace gbplanner2;
namespace {
void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
bool exact(const StateVec& a,const StateVec& b){return (a.array()==b.array()).all();}

// Only sample acceptance and edge collision are controlled. The actual local
// planner still samples, expands its graph, evaluates gain and selects a path.
// Fixed X=4 samples distinguish acceptance checks from the root box at X=0.
class ControlledMap final:public MapQuery {
 public:
  ControlledMap(int reject_samples,bool reject_edges):
      map_(.1,Eigen::Vector3d(-1.,-1.,-.5),Eigen::Vector3i(70,20,25)),
      reject_samples_(reject_samples),reject_edges_(reject_edges){map_.fill(VoxelStatus::Free);}
  double resolution()const override{return map_.resolution();}
  Eigen::Vector3d minimum()const override{return map_.minimum();}
  Eigen::Vector3d maximum()const override{return map_.maximum();}
  VoxelStatus voxel(const Eigen::Vector3d& point)const override{return map_.voxel(point);}
  VoxelStatus box(const Eigen::Vector3d& point,const Eigen::Vector3d& size)const override{
    if(point.x()==4.){
      ++sample_queries;
      if(sample_queries<=reject_samples_)return VoxelStatus::Occupied;
    }
    return map_.box(point,size);
  }
  VoxelStatus path(const Eigen::Vector3d& a,const Eigen::Vector3d& b,const Eigen::Vector3d& size)const override{
    ++edge_queries;
    return reject_edges_?VoxelStatus::Occupied:map_.path(a,b,size);
  }
  void ray(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
           const std::function<bool(const Eigen::Vector3d&,VoxelStatus)>& visit)const override{
    map_.ray(a,b,visit);
  }
  mutable int sample_queries{0},edge_queries{0};
 private:
  VoxelMap map_;
  int reject_samples_;
  bool reject_edges_;
};

Parameters lineParameters(){
  Parameters p;p.local_min={-.5,-.5,-.1};p.local_max={4.5,.5,.1};
  p.robot_size={.1,.1,.1};p.edge_min=.05;p.edge_max=1.;p.overshoot=0.;
  p.max_vertices=10;p.max_attempts=12;p.empty_graph_cutoff=3;p.graph_mode=false;
  p.free_gain=1.;p.path_length_penalty=0.;p.sensor_range=1.;
  p.horizontal_fov=1.;p.horizontal_resolution=1.;p.vertical_fov=.2;p.vertical_resolution=.2;
  p.traverse_length_max=100.;p.traverse_time_max=1000.;p.refinement.enabled=false;
  for(auto& axis:p.axis_sampling){axis.distribution="constant";axis.constant=0.;}
  p.axis_sampling[0].constant=4.;return p;
}

void samePlanAndGraph(const LocalPlanner& a,const PlanResult& x,const LocalPlanner& b,const PlanResult& y){
  require(x.success&&y.success&&x.reason==y.reason&&x.vertices==y.vertices&&x.edges==y.edges&&
          x.attempts==y.attempts&&x.sampling_rounds==y.sampling_rounds&&x.best_id==y.best_id&&
          x.sampling_stop_reason==y.sampling_stop_reason&&
          x.gain==y.gain&&x.gain_evaluations==y.gain_evaluations&&x.reused_gains==y.reused_gains&&
          x.path_length==y.path_length&&x.candidate_path_length==y.candidate_path_length&&
          x.traverse_length==y.traverse_length&&x.traverse_time==y.traverse_time&&
          x.executed_target_id==y.executed_target_id&&x.path.size()==y.path.size(),
          "Default cutoff changed a feasible seeded planning result");
  for(std::size_t i=0;i<x.path.size();++i)require(exact(x.path[i],y.path[i]),"Seeded output pose changed");
  require(a.graph().edge_map_==b.graph().edge_map_,"Seeded graph connectivity or weights changed");
  for(const auto& item:a.graph().vertices_map_){
    const auto* lhs=item.second;const auto* rhs=b.graph().vertices_map_.at(item.first);
    require(exact(lhs->state,rhs->state)&&lhs->type==rhs->type&&
            lhs->vol_gain.gain==rhs->vol_gain.gain&&
            lhs->vol_gain.num_unknown_voxels==rhs->vol_gain.num_unknown_voxels&&
            lhs->vol_gain.num_free_voxels==rhs->vol_gain.num_free_voxels&&
            lhs->vol_gain.num_occupied_voxels==rhs->vol_gain.num_occupied_voxels&&
            lhs->vol_gain.is_frontier==rhs->vol_gain.is_frontier,
            "Seeded graph viewpoint or gain changed");
  }
}
}

int main(){
  try{
    const StateVec root(0.,0.,.6,0.);auto p=lineParameters();
    require(p.sampling_budget_mode=="raw_draws","Default sampling budget no longer preserves raw draws");
    // The unused round budget must not shorten a legacy raw-draw request.
    p.max_sampling_rounds=1;
    auto isolated_map=std::make_shared<ControlledMap>(0,true);
    const auto isolated=LocalPlanner(isolated_map,p).plan(root);
    require(!isolated.success&&isolated.reason=="no_feasible_path"&&isolated.vertices==1&&
            isolated.attempts==3&&isolated.sampling_rounds==3&&isolated.empty_graph_cutoff_reached&&
            isolated.sampling_stop_reason=="empty_graph_cutoff"&&
            isolated_map->sample_queries==3&&isolated_map->edge_queries==3&&!isolated.gain_evaluated,
            "Isolated root did not stop immediately after its third unsuccessful expansion");

    auto growing_map=std::make_shared<ControlledMap>(0,false);
    const auto growing=LocalPlanner(growing_map,p).plan(root);
    require(growing.success&&growing.vertices==5&&growing.attempts==p.max_attempts&&
            growing.sampling_rounds==p.max_attempts&&!growing.empty_graph_cutoff_reached&&
            growing.sampling_stop_reason=="raw_draw_limit",
            "A graph with its first edge was cut off during later unsuccessful expansions");

    // The first complete 1000-draw sampleVertex group fails. The first valid
    // sample is in round 2, already above cutoff=1, but expansion happens first.
    p.empty_graph_cutoff=1;p.max_attempts=1004;
    auto late_free_map=std::make_shared<ControlledMap>(1000,false);
    const auto late_free=LocalPlanner(late_free_map,p).plan(root);
    require(late_free.success&&late_free.vertices==5&&late_free.attempts==1004&&
            late_free.sampling_rounds==5&&!late_free.empty_graph_cutoff_reached&&
            late_free.sampling_stop_reason=="raw_draw_limit"&&
            late_free.collision_rejections==1000&&late_free_map->edge_queries==4,
            "First valid sample above cutoff was rejected before it could establish an edge");
    auto late_blocked_map=std::make_shared<ControlledMap>(1000,true);
    const auto late_blocked=LocalPlanner(late_blocked_map,p).plan(root);
    require(!late_blocked.success&&late_blocked.attempts==1001&&late_blocked.sampling_rounds==2&&
            late_blocked.empty_graph_cutoff_reached&&late_blocked.sampling_stop_reason=="empty_graph_cutoff"&&
            late_blocked_map->edge_queries==1,
            "First valid sample above cutoff did not stop after its failed expansion");
    auto batch_end_map=std::make_shared<ControlledMap>(999,true);
    const auto batch_end=LocalPlanner(batch_end_map,p).plan(root);
    require(!batch_end.success&&batch_end.attempts==1000&&batch_end.sampling_rounds==1&&
            batch_end.empty_graph_cutoff_reached&&batch_end.sampling_stop_reason=="empty_graph_cutoff"&&
            batch_end_map->edge_queries==1,
            "The thousandth accepted draw was counted as the next sampling round");

    // Pure rejection skips the post-expansion cutoff, as upstream. The existing
    // ROS 2 raw-draw cap still terminates it, including a final partial group.
    p.max_attempts=2001;
    auto rejected_map=std::make_shared<ControlledMap>(p.max_attempts,true);
    const auto rejected=LocalPlanner(rejected_map,p).plan(root);
    require(!rejected.success&&rejected.attempts==2001&&rejected.sampling_rounds==3&&
            rejected.vertices==1&&!rejected.empty_graph_cutoff_reached&&
            rejected.sampling_stop_reason=="raw_draw_limit"&&
            rejected.collision_rejections==2001&&rejected_map->edge_queries==0,
            "All-rejected draws escaped or incorrectly shortened the existing total budget");

    // Original buildGraph budgets complete sampleVertex calls. Keeping the
    // raw limit at one proves that it cannot truncate this separate mode.
    auto rounds=lineParameters();rounds.sampling_budget_mode="sampling_rounds";
    rounds.max_sampling_rounds=3;rounds.max_attempts=1;rounds.empty_graph_cutoff=1;
    auto immediate_map=std::make_shared<ControlledMap>(0,false);
    const auto immediate=LocalPlanner(immediate_map,rounds).plan(root);
    require(immediate.success&&immediate.attempts==3&&immediate.sampling_rounds==3&&
            immediate.vertices==4&&immediate.edges==3&&!immediate.empty_graph_cutoff_reached&&
            immediate.sampling_stop_reason=="sampling_round_limit"&&
            immediate_map->sample_queries==3&&immediate_map->edge_queries==3,
            "Three immediately accepted rounds did not stop after exactly three draws");

    auto exhausted_map=std::make_shared<ControlledMap>(3000,true);
    const auto exhausted=LocalPlanner(exhausted_map,rounds).plan(root);
    require(!exhausted.success&&exhausted.reason=="no_feasible_path"&&exhausted.attempts==3000&&
            exhausted.sampling_rounds==3&&exhausted.vertices==1&&exhausted.edges==0&&
            exhausted.collision_rejections==3000&&!exhausted.empty_graph_cutoff_reached&&
            exhausted.sampling_stop_reason=="sampling_round_limit"&&
            exhausted_map->sample_queries==3000&&exhausted_map->edge_queries==0,
            "Three fully rejected rounds did not exhaust 3000 draws without the expansion cutoff");

    auto last_draw_parameters=rounds;last_draw_parameters.max_sampling_rounds=1;
    auto last_draw_map=std::make_shared<ControlledMap>(999,false);
    const auto last_draw=LocalPlanner(last_draw_map,last_draw_parameters).plan(root);
    require(last_draw.success&&last_draw.attempts==1000&&last_draw.sampling_rounds==1&&
            last_draw.vertices==2&&last_draw.edges==1&&last_draw.collision_rejections==999&&
            !last_draw.empty_graph_cutoff_reached&&last_draw.sampling_stop_reason=="sampling_round_limit"&&
            last_draw_map->sample_queries==1000&&last_draw_map->edge_queries==1,
            "The final round ended before its thousandth draw could be accepted and expanded");

    auto round_late_free_map=std::make_shared<ControlledMap>(1000,false);
    const auto round_late_free=LocalPlanner(round_late_free_map,rounds).plan(root);
    require(round_late_free.success&&round_late_free.attempts==1002&&round_late_free.sampling_rounds==3&&
            round_late_free.vertices==3&&round_late_free.edges==2&&
            round_late_free.collision_rejections==1000&&!round_late_free.empty_graph_cutoff_reached&&
            round_late_free.sampling_stop_reason=="sampling_round_limit"&&round_late_free_map->edge_queries==2,
            "A first accepted round above cutoff did not establish its edge before the cutoff check");
    auto round_late_blocked_map=std::make_shared<ControlledMap>(1000,true);
    const auto round_late_blocked=LocalPlanner(round_late_blocked_map,rounds).plan(root);
    require(!round_late_blocked.success&&round_late_blocked.attempts==1001&&
            round_late_blocked.sampling_rounds==2&&round_late_blocked.vertices==1&&
            round_late_blocked.empty_graph_cutoff_reached&&
            round_late_blocked.sampling_stop_reason=="empty_graph_cutoff"&&round_late_blocked_map->edge_queries==1,
            "An accepted round above cutoff did not stop after failing to establish its first edge");

    auto vertex_parameters=rounds;vertex_parameters.max_sampling_rounds=8;vertex_parameters.max_vertices=3;
    const auto vertex_limited=LocalPlanner(std::make_shared<ControlledMap>(0,false),vertex_parameters).plan(root);
    require(vertex_limited.success&&vertex_limited.attempts==2&&vertex_limited.sampling_rounds==2&&
            vertex_limited.vertices==3&&vertex_limited.edges==2&&
            !vertex_limited.empty_graph_cutoff_reached&&vertex_limited.sampling_stop_reason=="vertex_limit",
            "The vertex limit did not terminate sampling before the round budget");
    auto edge_parameters=rounds;edge_parameters.max_sampling_rounds=8;edge_parameters.max_edges=2;
    const auto edge_limited=LocalPlanner(std::make_shared<ControlledMap>(0,false),edge_parameters).plan(root);
    require(edge_limited.success&&edge_limited.attempts==2&&edge_limited.sampling_rounds==2&&
            edge_limited.vertices==3&&edge_limited.edges==2&&
            !edge_limited.empty_graph_cutoff_reached&&edge_limited.sampling_stop_reason=="edge_limit",
            "The edge limit did not terminate sampling before the round budget");

    StateVec occupied_root=root;occupied_root.x()=4.;
    const auto not_started=LocalPlanner(std::make_shared<ControlledMap>(1,false),rounds).plan(occupied_root);
    require(!not_started.success&&not_started.reason=="root_not_free"&&!not_started.graph_build_completed&&
            not_started.attempts==0&&not_started.sampling_rounds==0&&not_started.sampling_stop_reason=="not_started",
            "An invalid root incorrectly reported sampling budget exhaustion");

    // Use the ordinary seeded sampler and occupancy map. A cutoff beyond the
    // complete raw budget makes its branch unreachable, providing the earlier
    // behavior without duplicating or mocking the planning implementation.
    Parameters normal;require(normal.empty_graph_cutoff==1000,"Default cutoff differs from upstream");
    normal.free_gain=1.;
    auto open=std::make_shared<VoxelMap>(.25,Eigen::Vector3d(-10.,-10.,-4.),Eigen::Vector3i(80,80,32));
    open->fill(VoxelStatus::Free);
    auto no_cutoff=normal;no_cutoff.empty_graph_cutoff=normal.max_attempts+1;
    LocalPlanner default_planner(open,normal),reference_planner(open,no_cutoff);
    const auto default_result=default_planner.plan(root),reference_result=reference_planner.plan(root);
    samePlanAndGraph(default_planner,default_result,reference_planner,reference_result);
    require(!default_result.empty_graph_cutoff_reached&&!reference_result.empty_graph_cutoff_reached,
            "Feasible reference unexpectedly stopped on empty graph cutoff");

    std::cout<<std::setprecision(15)<<"{\"status\":\"passed\",\"sample_vertex_draw_limit\":1000"
      <<",\"isolated_attempts\":"<<isolated.attempts<<",\"isolated_sampling_rounds\":"<<isolated.sampling_rounds
      <<",\"growing_attempts\":"<<growing.attempts<<",\"growing_vertices\":"<<growing.vertices
      <<",\"late_accepted_attempts\":"<<late_free.attempts<<",\"late_accepted_sampling_rounds\":"<<late_free.sampling_rounds
      <<",\"late_failed_expansion_attempts\":"<<late_blocked.attempts
      <<",\"late_failed_expansion_sampling_rounds\":"<<late_blocked.sampling_rounds
      <<",\"batch_end_attempts\":"<<batch_end.attempts<<",\"batch_end_sampling_rounds\":"<<batch_end.sampling_rounds
      <<",\"all_rejected_attempts\":"<<rejected.attempts<<",\"all_rejected_sampling_rounds\":"<<rejected.sampling_rounds
      <<",\"round_immediate_attempts\":"<<immediate.attempts<<",\"round_immediate_rounds\":"<<immediate.sampling_rounds
      <<",\"round_rejected_attempts\":"<<exhausted.attempts<<",\"round_rejected_rounds\":"<<exhausted.sampling_rounds
      <<",\"round_last_draw_attempts\":"<<last_draw.attempts<<",\"round_last_draw_rounds\":"<<last_draw.sampling_rounds
      <<",\"round_late_accepted_attempts\":"<<round_late_free.attempts
      <<",\"round_late_failed_expansion_attempts\":"<<round_late_blocked.attempts
      <<",\"vertex_limit_rounds\":"<<vertex_limited.sampling_rounds
      <<",\"edge_limit_rounds\":"<<edge_limited.sampling_rounds
      <<",\"default_seed\":"<<normal.seed<<",\"default_vertices\":"<<default_result.vertices
      <<",\"default_edges\":"<<default_result.edges<<",\"default_attempts\":"<<default_result.attempts
      <<",\"default_sampling_rounds\":"<<default_result.sampling_rounds
      <<",\"feasible_seeded_graph_gain_path_exact\":true,\"raw_attempt_budget_preserved\":true"
      <<",\"sampling_round_budget_verified\":true,\"sampling_stop_reasons_verified\":true}\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
