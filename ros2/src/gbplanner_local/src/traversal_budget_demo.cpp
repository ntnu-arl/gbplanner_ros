// getBestPath traversal limits on actual aerial and projected ground plans.
#include "gbplanner_local/local_planner.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

using namespace gbplanner2;
namespace {
void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
bool close(double a,double b){return std::abs(a-b)<1e-9;}
std::shared_ptr<VoxelMap> scene(bool ground=false){
  auto map=std::make_shared<VoxelMap>(.1,Eigen::Vector3d(-1.,-1.,-.5),Eigen::Vector3i(70,20,25));
  map->fill(VoxelStatus::Free);
  if(ground)for(int z=0;z<5;++z)for(int y=0;y<20;++y)for(int x=0;x<70;++x)
    map->set(Eigen::Vector3i(x,y,z),VoxelStatus::Occupied);
  return map;
}
Parameters lineParameters(){
  Parameters p;p.local_min={-.5,-.5,-.1};p.local_max={4.5,.5,.1};
  p.robot_size={.1,.1,.1};p.edge_min=.05;p.edge_max=1.;p.overshoot=0.;
  p.max_vertices=5;p.max_attempts=10;p.graph_mode=false;
  p.free_gain=1.;p.path_length_penalty=0.;p.sensor_range=1.;
  p.horizontal_fov=1.;p.horizontal_resolution=1.;p.vertical_fov=.2;p.vertical_resolution=.2;
  p.max_velocity=.2;p.traverse_length_max=100.;p.traverse_time_max=1000.;
  for(auto& axis:p.axis_sampling){axis.distribution="constant";axis.constant=0.;}
  p.axis_sampling[0].constant=4.;return p;
}
void isPrefix(const PlanResult& result,const PlanResult& reference){
  require(result.success&&result.truncated_for_budget&&result.path.size()>1&&
          result.path.size()<reference.path.size(),"Traversal limit did not return a strict usable prefix");
  require(result.best_id==reference.best_id&&close(result.candidate_path_length,reference.candidate_path_length)&&
          close(result.gain,reference.gain),"Traversal limit changed graph selection or candidate gain");
  require(result.gain_evaluated&&!result.frontier_exists,"Free-map gain bookkeeping is inconsistent");
  for(std::size_t i=0;i<result.path.size();++i)
    require((result.path[i]-reference.path[i]).norm()<1e-9,"Returned prefix differs from its reference path");
  require(close(result.path_length,result.traverse_length),"Unrefined length differs from traversal accounting");
}
}

int main(){
  try{
    const StateVec root(0.,0.,.6,0.);auto map=scene();auto p=lineParameters();
    const auto reference=LocalPlanner(map,p).plan(root);
    require(reference.success&&reference.path.size()==5&&close(reference.path_length,4.),
            "Deterministic four-edge reference plan failed");

    p.traverse_length_max=1.2;
    LocalPlanner length_planner(map,p);const auto length_limited=length_planner.plan(root);
    isPrefix(length_limited,reference);
    require(close(length_limited.path_length,2.)&&close(length_limited.traverse_time,10.)&&
            length_limited.executed_target_id==2&&
            (length_limited.path.back().head<3>()-
             length_planner.graph().vertices_map_.at(2)->state.head<3>()).norm()<1e-9,
            "Length limit clipped an edge instead of retaining the crossing graph vertex");

    p.traverse_length_max=100.;p.traverse_time_max=6.;
    const auto time_limited=LocalPlanner(map,p).plan(root);isPrefix(time_limited,reference);
    require(close(time_limited.path_length,2.)&&close(time_limited.traverse_time,10.),
            "Traversal time did not use PlanningParams max velocity");

    p.traverse_length_max=1.;p.traverse_time_max=5.;
    const auto equal_limit=LocalPlanner(map,p).plan(root);isPrefix(equal_limit,reference);
    require(close(equal_limit.path_length,2.),"Equality stopped traversal before the strict upstream limit");

    auto short_parameters=lineParameters();short_parameters.max_vertices=2;
    short_parameters.axis_sampling[0].constant=.3;
    const auto short_path=LocalPlanner(map,short_parameters).plan(root);
    require(!short_path.success&&short_path.path.empty()&&short_path.gain_evaluated&&
            short_path.reason=="best_path_too_short"&&close(short_path.candidate_path_length,.3),
            "A candidate at the 0.3 metre minimum was accepted");
    require(short_path.evaluate_ms>0.&&short_path.gain_ms>0.&&
            short_path.gain_ms<=short_path.evaluate_ms&&short_path.refinement_ms==0.,
            "An evaluated early rejection lost its phase timing");
    short_parameters.axis_sampling[0].constant=.31;
    const auto above_minimum=LocalPlanner(map,short_parameters).plan(root);
    require(above_minimum.success&&close(above_minimum.path_length,.31),
            "A candidate above the minimum was rejected");

    auto ground_parameters=lineParameters();ground_parameters.ground_robot=true;
    ground_parameters.ground.max_ground_height=.6;ground_parameters.ground.robot_height=.4;
    auto ground_map=scene(true);
    const auto ground_reference=LocalPlanner(ground_map,ground_parameters).plan(root);
    require(ground_reference.success&&ground_reference.path.size()>10,
            "Projected-ground reference has no complete multi-knot edges");
    ground_parameters.traverse_length_max=.25;
    LocalPlanner ground_planner(ground_map,ground_parameters);
    const auto ground_limited=ground_planner.plan(root);isPrefix(ground_limited,ground_reference);
    const auto& graph_target=ground_planner.graph().vertices_map_.at(ground_limited.executed_target_id)->state;
    require(ground_limited.path.size()>2&&close(ground_limited.path_length,1.)&&
            ground_limited.executed_target_id==1&&
            (ground_limited.path.back().head<3>()-graph_target.head<3>()).norm()<1e-9,
            "Ground traversal stopped at an interior projection knot");
    double actual_length=0.;
    for(std::size_t i=1;i<ground_limited.path.size();++i)
      actual_length+=(ground_limited.path[i]-ground_limited.path[i-1]).head<3>().norm();
    require(close(actual_length,ground_limited.traverse_length),"Ground length lost part of an edge polyline");

    std::cout<<std::setprecision(15)<<"{\"status\":\"passed\",\"reference_length_m\":"<<reference.path_length
      <<",\"length_limit_m\":1.2,\"length_limited_m\":"<<length_limited.path_length
      <<",\"time_limit_s\":6,\"time_limited_s\":"<<time_limited.traverse_time
      <<",\"planning_velocity_mps\":0.2,\"strict_limit_length_m\":"<<equal_limit.path_length
      <<",\"minimum_rejected_m\":"<<short_path.candidate_path_length
      <<",\"rejected_evaluate_ms\":"<<short_path.evaluate_ms
      <<",\"rejected_gain_ms\":"<<short_path.gain_ms
      <<",\"above_minimum_accepted_m\":"<<above_minimum.path_length
      <<",\"ground_limit_m\":0.25,\"ground_complete_edge_m\":"<<ground_limited.path_length
      <<",\"ground_prefix_points\":"<<ground_limited.path.size()
      <<",\"ground_executed_vertex\":"<<ground_limited.executed_target_id
      <<",\"candidate_selection_preserved\":true,\"ground_edge_preserved\":true}\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
