#include "gbplanner_local/local_planner.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

using namespace gbplanner2;
namespace {
std::shared_ptr<VoxelMap> scene(VoxelStatus body=VoxelStatus::Free){
  auto map=std::make_shared<VoxelMap>(.1,Eigen::Vector3d(-4,-3,-1),Eigen::Vector3i(100,60,40));
  map->fill(VoxelStatus::Free);
  for(int z=0;z<40;++z)for(int y=0;y<60;++y)for(int x=0;x<100;++x){
    const Eigen::Vector3i index(x,y,z);const auto point=map->center(index);
    // The body corridor is observed free. Only support beyond x=0 is unknown,
    // unlike an entirely unseen column through which a candidate must not pass.
    if(point.z()<0)map->set(index,point.x()<0?VoxelStatus::Occupied:VoxelStatus::Unknown);
    // A body offset of Y=0.25 puts this patch between the five projection
    // rays at Y=0,+/-0.5, so changing clearance cannot change their support.
    if(body!=VoxelStatus::Free&&point.x()>3.7&&point.x()<4.3&&
       point.y()>.1&&point.y()<.4&&point.z()>.3&&point.z()<.8)map->set(index,body);
  }
  return map;
}
void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
void verifyPrefix(const PlanResult& result,const TerrainProjector& terrain){
  require(result.success&&result.path.size()>1,"No executable ground prefix returned");
  double length=0.;
  for(std::size_t i=0;i<result.path.size();++i){
    const auto projection=terrain.project(result.path[i]);
    require(projection.status==GroundStatus::Supported&&!projection.hanging,"A hanging state leaked into execution");
    require((projection.projected_state-result.path[i]).head<3>().norm()<1e-8,"Execution vertex has shifted ground support");
    if(i){
      require(terrain.edge(result.path[i-1],result.path[i]).admissible(),"An executable segment is speculative");
      length+=(result.path[i]-result.path[i-1]).head<3>().norm();
    }
  }
  require(std::abs(length-result.path_length)<1e-8,"Execution length includes discarded lookahead");
}
void write(const PlanResult& result){
  std::cout<<"{\"success\":true,\"hanging_vertices\":"<<result.hanging_vertices
      <<",\"candidate_hanging_vertices\":"<<result.candidate_hanging_vertices
      <<",\"truncated\":"<<(result.truncated_for_hanging?"true":"false")
      <<",\"candidate_id\":"<<result.best_id<<",\"executed_target_id\":"<<result.executed_target_id
      <<",\"candidate_length\":"<<result.candidate_path_length<<",\"executed_length\":"<<result.path_length
      <<",\"gain_before_hanging_penalty\":"<<result.gain_before_hanging_penalty
      <<",\"gain_after_hanging_penalty\":"<<result.unpenalized_gain<<",\"score\":"<<result.gain
      <<",\"path\":[";
  for(std::size_t i=0;i<result.path.size();++i){
    if(i)std::cout<<',';const auto& point=result.path[i];
    std::cout<<'['<<point[0]<<','<<point[1]<<','<<point[2]<<','<<point[3]<<']';
  }
  std::cout<<"]}";
}
void writeSampling(const PlanResult& result){
  std::cout<<"{\"raw_draws\":"<<result.attempts<<",\"sampling_rounds\":"<<result.sampling_rounds
    <<",\"collision_rejections\":"<<result.collision_rejections<<",\"vertices\":"<<result.vertices
    <<",\"hanging_vertices\":"<<result.hanging_vertices
    <<",\"empty_graph_cutoff_reached\":"<<(result.empty_graph_cutoff_reached?"true":"false")
    <<",\"sampling_stop_reason\":"<<std::quoted(result.sampling_stop_reason)<<'}';
}
}
int main(){
  try{
    auto map=scene();Parameters p;p.ground_robot=true;
    p.ground.max_ground_height=.8;p.ground.robot_height=.55;
    p.robot_size={.3,.3,.3};p.ground.max_inclination=.85;p.ground.max_negative_inclination=.85;
    p.local_min={-.2,-1.5,-.1};p.local_max={5.,1.5,.3};p.max_vertices=180;p.max_edges=5000;
    p.max_attempts=5000;p.seed=42;p.sensor_range=3.;p.vertical_fov=2.4;
    // This scene exercises unknown-support truncation several metres away.
    // Give it an explicit long traversal budget so the newer one-metre
    // production default does not end the route before the hanging boundary.
    p.traverse_length_max=100.;p.traverse_time_max=1000.;
    p.horizontal_resolution=M_PI/12;p.vertical_resolution=.2;p.path_length_penalty=.07;
    p.path_direction_penalty=0.;p.ground.hanging_vertex_penalty=0.;
    TerrainProjector terrain(map,p.groundParameters());
    const StateVec root(-2.,0,.8,0),unseen_support(1.,0,.8,0);
    const auto projected=terrain.project(unseen_support);
    require(projected.hanging,"Unknown support was not classified as hanging");
    require(!terrain.edge(root,unseen_support).admissible(),"Strict edge accepted unknown support");
    const auto lookahead=terrain.candidateEdge(root,unseen_support);
    require(lookahead.candidateAdmissible()&&lookahead.speculative,"Candidate edge lost unknown-support lookahead");

    LocalPlanner unpenalized(map,p);const auto off=unpenalized.plan(root);
    p.ground.hanging_vertex_penalty=5.;LocalPlanner penalized(map,p);const auto on=penalized.plan(root);
    require(off.hanging_vertices>0&&off.hanging_vertices==on.hanging_vertices,"No hanging candidates entered the same graph");
    if(!off.truncated_for_hanging||off.candidate_hanging_vertices<=0){
      std::cerr<<"lookahead reason="<<off.reason<<" hanging="<<off.candidate_hanging_vertices
        <<" budget_truncated="<<off.truncated_for_budget<<" executed="<<off.path_length
        <<" candidate="<<off.candidate_path_length<<'\n';
      throw std::runtime_error("Unpenalized lookahead was not truncated");
    }
    require(off.path_length<off.candidate_path_length,"Candidate length was not separated from execution length");
    require(on.gain<off.gain,"Hanging penalty had no effect on candidate ranking or score");
    verifyPrefix(off,terrain);verifyPrefix(on,terrain);
    planner_msgs::msg::Graph message;unpenalized.graphMessage(message);std::size_t masked=0;
    for(const auto& vertex:message.vertices){
      const auto* local=unpenalized.graph().vertices_map_.at(vertex.id);
      if(local->is_hanging){require(!vertex.is_frontier,"Global export includes a hanging frontier");++masked;}
    }
    require(masked>0,"No hanging vertices were checked in global export");

    // Unknown body clearance and a fully observed lack of support remain
    // rejected even for local candidates.
    auto blocked=scene();blocked->set(blocked->index({.5,0,.55}),VoxelStatus::Unknown);
    TerrainProjector blocked_terrain(blocked,p.groundParameters());
    require(!blocked_terrain.candidateEdge(root,unseen_support).candidateAdmissible(),"Candidate traversed unknown body volume");
    auto bottomless=std::make_shared<VoxelMap>(.1,Eigen::Vector3d(-4,-3,-6),Eigen::Vector3i(100,60,90));
    bottomless->fill(VoxelStatus::Free);TerrainProjector no_ground(bottomless,p.groundParameters());
    require(!no_ground.candidateEdge(root,unseen_support).candidateAdmissible(),"Known absence of ground became a candidate");

    // Exercise the actual LocalSampler path. All draws request the same
    // hanging sample; only its off-ray body patch differs between maps.
    Parameters sampling;sampling.ground_robot=true;
    sampling.ground.max_ground_height=.8;sampling.ground.robot_height=.55;
    sampling.ground.max_inclination=.85;sampling.ground.max_negative_inclination=.85;
    sampling.robot_size={.2,.2,.2};sampling.center_offset={0.,.25,0.};
    sampling.local_min={-7.,-1.,-.2};sampling.local_max={7.,1.,.2};
    sampling.edge_max=7.;sampling.overshoot=0.;sampling.graph_mode=false;
    sampling.max_vertices=10;sampling.max_edges=20;sampling.empty_graph_cutoff=1;
    sampling.sampling_budget_mode="sampling_rounds";sampling.max_sampling_rounds=2;
    sampling.max_attempts=1; // Inactive in round mode, even on accepted draws.
    for(auto& axis:sampling.axis_sampling){axis.distribution="constant";axis.constant=0.;}
    sampling.axis_sampling[0].constant=6.;
    const StateVec fixed_sample(4.,0.,.8,0.);
    const auto sample_body=[&](VoxelStatus status){
      auto sample_map=scene(status);TerrainProjector sample_terrain(sample_map,sampling.groundParameters());
      const auto projection=sample_terrain.project(fixed_sample);
      require(projection.status==GroundStatus::Hanging&&projection.hanging&&!projection.support&&
        sample_map->box(sample_terrain.baseState(projection.projected_state).head<3>()+
                        sampling.center_offset,sampling.robot_size)==status,
        "The body fixture changed projection support instead of only the robot clearance");
      LocalPlanner planner(sample_map,sampling);const auto result=planner.plan(root);
      require(result.graph_build_completed&&result.sampling_rounds==2&&
        !result.empty_graph_cutoff_reached&&result.sampling_stop_reason=="sampling_round_limit",
        "Hanging body sampling did not finish two complete rounds");
      if(status==VoxelStatus::Free){
        require(result.attempts==2&&result.collision_rejections==0&&
          result.vertices==2&&result.hanging_vertices==1,
          "A free-body hanging sample failed to end each batch or enter local lookahead");
      }else{
        require(result.attempts==2000&&result.collision_rejections==2000&&
          result.vertices==1&&result.edges==0&&result.hanging_vertices==0,
          "A non-free hanging body ended a batch early or entered the local graph");
      }
      return result;
    };
    const auto sampling_free=sample_body(VoxelStatus::Free);
    const auto sampling_unknown=sample_body(VoxelStatus::Unknown);
    const auto sampling_occupied=sample_body(VoxelStatus::Occupied);

    std::cout<<std::setprecision(15)<<"{\"seed\":42,\"masked_hanging_vertices\":"<<masked
        <<",\"strict_edges_reject_unknown_support\":true,\"free_body_required\":true,\"off\":";
    write(off);std::cout<<",\"penalty_5\":";write(on);
    std::cout<<",\"sampling_body\":{\"free\":";writeSampling(sampling_free);
    std::cout<<",\"unknown\":";writeSampling(sampling_unknown);
    std::cout<<",\"occupied\":";writeSampling(sampling_occupied);
    std::cout<<"}}\n";return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
