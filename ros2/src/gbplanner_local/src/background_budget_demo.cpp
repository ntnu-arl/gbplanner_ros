// Global background sampling batches and the original soft timer boundary.
#include "gbplanner_local/global_memory.hpp"
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <stdexcept>

using namespace gbplanner2;
namespace {
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
std::shared_ptr<VoxelMap> freeMap(){
  auto map=std::make_shared<VoxelMap>(.25,Eigen::Vector3d(-10,-5,-2),Eigen::Vector3i(200,40,16));
  map->fill(VoxelStatus::Free);return map;
}
VolumetricGain frontier(const StateVec&){VolumetricGain gain;gain.gain=10.;gain.is_frontier=true;return gain;}
Parameters parameters(double sample_x=6.){
  Parameters p;p.local_min={-7,-1,-1};p.local_max={7,1,1};
  p.robot_size={.1,.1,.1};p.edge_max=7.;p.overshoot=0.;p.graph_mode=false;
  for(auto& axis:p.axis_sampling){axis.distribution="constant";axis.constant=0.;}
  p.axis_sampling[0].constant=sample_x;return p;
}

std::shared_ptr<VoxelMap> groundMap(bool unknown_support,VoxelStatus body=VoxelStatus::Free){
  auto map=std::make_shared<VoxelMap>(.1,Eigen::Vector3d(-4,-3,-1),Eigen::Vector3i(100,60,40));
  map->fill(VoxelStatus::Free);
  for(int z=0;z<40;++z)for(int y=0;y<60;++y)for(int x=0;x<100;++x){
    const Eigen::Vector3i index(x,y,z);const auto point=map->center(index);
    if(point.z()<0.)map->set(index,unknown_support&&point.x()>=0.?VoxelStatus::Unknown:VoxelStatus::Occupied);
    // The body's Y offset puts this patch between the five support rays.
    // It changes body clearance without moving the hanging projection upward.
    if(body!=VoxelStatus::Free&&point.x()>3.7&&point.x()<4.3&&
       point.y()>.1&&point.y()<.4&&point.z()>.3&&point.z()<.8)map->set(index,body);
  }
  return map;
}

// Gate only raw point-validity queries. The sampler, graph construction, sparse
// exclusion, steering and edge collision checks remain the production code.
class SamplingMap final:public MapQuery {
 public:
  SamplingMap():base_(freeMap()){}
  double resolution()const override{return base_->resolution();}
  Eigen::Vector3d minimum()const override{return base_->minimum();}
  Eigen::Vector3d maximum()const override{return base_->maximum();}
  VoxelStatus voxel(const Eigen::Vector3d& p)const override{return base_->voxel(p);}
  VoxelStatus box(const Eigen::Vector3d& p,const Eigen::Vector3d& size)const override{return base_->box(p,size);}
  VoxelStatus path(const Eigen::Vector3d& a,const Eigen::Vector3d& b,const Eigen::Vector3d& size)const override{
    if(armed&&(a-b).isZero(0.)){
      sample_x.push_back(a.x());
      if(sample_x.size()<=reject_draws)return VoxelStatus::Occupied;
    }else if(armed&&block_edges)return VoxelStatus::Occupied;
    return base_->path(a,b,size);
  }
  void ray(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
           const std::function<bool(const Eigen::Vector3d&,VoxelStatus)>& visit)const override{
    base_->ray(a,b,visit);
  }
  bool armed{false},block_edges{false};std::size_t reject_draws{0};
  mutable std::vector<double> sample_x;
 private:
  std::shared_ptr<VoxelMap> base_;
};

GlobalMemory::BackgroundClock clockAt(std::initializer_list<int> milliseconds){
  return [times=std::vector<int>(milliseconds),index=std::size_t{0}]()mutable{
    if(index>=times.size())throw std::runtime_error("Unexpected clock check inside a round or sampling batch");
    return std::chrono::steady_clock::time_point{}+std::chrono::milliseconds(times[index++]);
  };
}
void initialize(GlobalMemory& memory,const std::shared_ptr<SamplingMap>& map,bool two_clusters=false){
  memory.updateMap(map);require(memory.recordOdometry(StateVec::Zero()),"Cannot initialize home");
  if(two_clusters){
    require(memory.recordOdometry(StateVec(20,0,0,0)),"Cannot initialize second cluster");
    // The fixture needs two unvisited anchors farther apart than the 10 m
    // clustering radius; real odometry marks its second anchor visited.
    for(const auto& entry:memory.graph().vertices_map_)entry.second->type=VertexType::kUnvisited;
  }
  map->armed=true;
}
void print(const char* name,const BackgroundUpdate& s){
  std::cout<<'"'<<name<<"\":{\"clusters\":"<<s.clusters<<",\"draws\":"<<s.attempts
    <<",\"rounds\":"<<s.rounds<<",\"sampling_batches\":"<<s.sampling_batches
    <<",\"accepted_samples\":"<<s.accepted_samples<<",\"expansion_attempts\":"<<s.expansion_attempts
    <<",\"hanging_rejections\":"<<s.hanging_rejections
    <<",\"added\":"<<s.added<<",\"outside_or_occupied\":"<<s.outside_or_occupied
    <<",\"near_visited\":"<<s.near_visited<<",\"near_graph\":"<<s.near_graph
    <<",\"collision_rejections\":"<<s.collision_rejections<<",\"elapsed_seconds\":"<<s.elapsed_seconds
    <<",\"time_budget_reached\":"<<(s.time_budget_reached?"true":"false")
    <<",\"stop_reason\":\""<<s.stop_reason<<"\"}";
}
}

int main(){
  try{
    const auto p=parameters();
    GlobalMemory fixed(p);auto fixed_map=std::make_shared<SamplingMap>();
    initialize(fixed,fixed_map,true);fixed_map->reject_draws=1000;
    const auto baseline=fixed.expandBackground(64,frontier);
    require(baseline.clusters==2&&baseline.attempts==64&&baseline.sampling_batches==64&&
      baseline.outside_or_occupied==64&&baseline.added==0&&baseline.stop_reason=="draw_budget",
      "The fixed raw-draw budget changed");
    require(fixed_map->sample_x.size()==64&&fixed_map->sample_x[0]!=fixed_map->sample_x[1],
      "Fixed mode stopped rotating clusters after each raw rejection");

    GlobalMemory timed(p);auto timed_map=std::make_shared<SamplingMap>();
    initialize(timed,timed_map,true);timed_map->reject_draws=1000;
    // Clustering/startup has already consumed 0.2 s at the first outer read.
    // Upstream still completes one full sweep, without checking either batch.
    const auto batches=timed.expandBackgroundTimed(.1,frontier,clockAt({0,200,200}));
    require(batches.clusters==2&&batches.rounds==1&&batches.sampling_batches==2&&batches.attempts==1001&&
      batches.outside_or_occupied==1000&&batches.accepted_samples==1&&batches.expansion_attempts==1&&
      batches.added==1&&batches.time_budget_reached&&batches.stop_reason=="time_budget",
      "An expired start or rejected batch prevented the complete cluster sweep");
    require(timed_map->sample_x.size()==1001&&
      std::all_of(timed_map->sample_x.begin(),timed_map->sample_x.begin()+1000,
                  [&](double x){return x==timed_map->sample_x.front();})&&
      timed_map->sample_x.back()!=timed_map->sample_x.front(),
      "A sampling batch changed clusters before exhausting 1000 rejected draws");

    GlobalMemory accepted(p);auto accepted_map=std::make_shared<SamplingMap>();
    initialize(accepted,accepted_map);accepted_map->reject_draws=999;
    const auto sparse=accepted.expandBackgroundTimed(.1,frontier,clockAt({0,0,100,100}));
    require(sparse.rounds==2&&sparse.sampling_batches==2&&sparse.attempts==1001&&
      sparse.outside_or_occupied==999&&sparse.accepted_samples==2&&sparse.expansion_attempts==1&&
      sparse.added==1&&sparse.near_graph==1,
      "Accepted samples did not end their batches before the sparse-graph rejection");

    GlobalMemory visited(parameters(6.));auto visited_map=std::make_shared<SamplingMap>();
    initialize(visited,visited_map);
    // Home alone is not history. Produce an actual E1 event at x=1, then
    // retain the home as this fixture's one unvisited sampling anchor. Its
    // constant x=6 sample is exactly 5 m from the recorded event, before the
    // independent graph-proximity filter is considered.
    visited_map->armed=false;
    require(visited.recordOdometry(StateVec(1.,0.,0.,0.))&&visited.visitedStates().size()==1,
      "Could not establish an actual 1 m history event");
    visited.graph().vertices_map_.at(0)->type=VertexType::kUnvisited;
    visited_map->armed=true;
    const auto history=visited.expandBackgroundTimed(.1,frontier,clockAt({0,0,100,100}));
    require(history.rounds==2&&history.attempts==2&&history.accepted_samples==2&&
      history.near_visited==2&&history.expansion_attempts==0&&history.added==0,
      "A sample at the inclusive 5 m history boundary was retried or expanded");

    GlobalMemory blocked(p);auto blocked_map=std::make_shared<SamplingMap>();
    initialize(blocked,blocked_map);blocked_map->block_edges=true;
    const auto collision=blocked.expandBackgroundTimed(.1,frontier,clockAt({0,0,100,100}));
    require(collision.attempts==2&&collision.accepted_samples==2&&collision.expansion_attempts==2&&
      collision.collision_rejections==2&&collision.added==0,
      "The timed scheduler bypassed shared edge collision checks");

    auto ground_parameters=parameters();ground_parameters.ground_robot=true;
    ground_parameters.ground.max_ground_height=.8;ground_parameters.ground.robot_height=.55;
    ground_parameters.robot_size={.2,.2,.2};ground_parameters.center_offset={0.,.25,0.};
    const StateVec ground_root(-2.,0.,.8,0.),ground_sample(4.,0.,.8,0.);
    auto hanging_map=groundMap(true);
    TerrainProjector hanging_terrain(hanging_map,ground_parameters.groundParameters());
    const auto hanging_projection=hanging_terrain.project(ground_sample);
    require(hanging_projection.status==GroundStatus::Hanging&&hanging_projection.hanging&&
      hanging_map->box(hanging_terrain.baseState(hanging_projection.projected_state).head<3>()+
                      ground_parameters.center_offset,ground_parameters.robot_size)==VoxelStatus::Free,
      "The hanging fixture must have unknown support and observed-free body clearance");
    GlobalMemory hanging(ground_parameters);hanging.updateMap(hanging_map);
    // Set the one unvisited home cluster directly. A displaced odometry event
    // would also run E1's visited-neighborhood update before this check.
    require(hanging.setHomingPosition(ground_root),"Cannot initialize the hanging ground fixture");
    const auto hanging_revision=hanging.snapshotRevision();int hanging_gain_calls=0;
    const auto hanging_gain=[&](const StateVec& state){++hanging_gain_calls;return frontier(state);};
    const auto hanging_timed=hanging.expandBackgroundTimed(.1,hanging_gain,clockAt({0,200,200}));
    require(hanging_timed.clusters==1&&hanging_timed.rounds==1&&hanging_timed.sampling_batches==1&&
      hanging_timed.attempts==1&&hanging_timed.accepted_samples==1&&hanging_timed.hanging_rejections==1&&
      hanging_timed.outside_or_occupied==0&&hanging_timed.near_visited==0&&hanging_timed.near_graph==0&&
      hanging_timed.expansion_attempts==0&&hanging_timed.added==0&&hanging_timed.stop_reason=="time_budget",
      "An accepted hanging sample was retried or reached sparse checks and graph expansion");
    const auto hanging_raw=hanging.expandBackground(2,hanging_gain);
    require(hanging_raw.attempts==2&&hanging_raw.rounds==2&&hanging_raw.sampling_batches==2&&
      hanging_raw.accepted_samples==2&&hanging_raw.hanging_rejections==2&&
      hanging_raw.outside_or_occupied==0&&hanging_raw.expansion_attempts==0&&hanging_raw.added==0&&
      hanging_raw.stop_reason=="draw_budget",
      "Raw mode did not discard accepted hanging samples one draw at a time");
    require(hanging_gain_calls==0&&hanging.graph().vertices_map_.size()==1&&hanging.edgeCount()==0&&
      hanging.snapshotRevision()==hanging_revision,
      "A discarded hanging sample changed persistent graph state or evaluated gain");
    const auto hanging_route=hanging.homing(ground_sample);
    require(!hanging_route.success&&hanging_route.path.empty(),"A hanging sample leaked into a global route");

    const auto rejected_body=[&](VoxelStatus status){
      auto map=groundMap(true,status);TerrainProjector terrain(map,ground_parameters.groundParameters());
      const auto projection=terrain.project(ground_sample);
      require(projection.status==GroundStatus::Hanging&&projection.hanging&&
        map->box(terrain.baseState(projection.projected_state).head<3>()+ground_parameters.center_offset,
                 ground_parameters.robot_size)==status,
        "The body patch changed the support rays instead of only the robot clearance");
      GlobalMemory memory(ground_parameters);memory.updateMap(map);
      require(memory.setHomingPosition(ground_root),"Cannot initialize the rejected-body ground fixture");
      const auto revision=memory.snapshotRevision();
      const auto stats=memory.expandBackgroundTimed(.1,frontier,clockAt({0,200,200}));
      require(stats.clusters==1&&stats.rounds==1&&stats.sampling_batches==1&&stats.attempts==1000&&
        stats.outside_or_occupied==1000&&stats.accepted_samples==0&&stats.hanging_rejections==0&&
        stats.expansion_attempts==0&&stats.added==0&&stats.stop_reason=="time_budget"&&
        memory.graph().vertices_map_.size()==1&&memory.edgeCount()==0&&memory.snapshotRevision()==revision,
        "A hanging sample with a non-free body ended its retry batch early or entered the graph");
      return stats;
    };
    const auto hanging_unknown_body=rejected_body(VoxelStatus::Unknown);
    const auto hanging_occupied_body=rejected_body(VoxelStatus::Occupied);

    auto supported_map=groundMap(false);GlobalMemory supported(ground_parameters);
    supported.updateMap(supported_map);
    require(supported.setHomingPosition(ground_root),"Cannot initialize the supported ground fixture");
    const auto supported_stats=supported.expandBackgroundTimed(.1,frontier,clockAt({0,200,200}));
    require(supported_stats.attempts==1&&supported_stats.accepted_samples==1&&
      supported_stats.hanging_rejections==0&&supported_stats.outside_or_occupied==0&&
      supported_stats.expansion_attempts==1&&supported_stats.added==1&&
      supported.graph().vertices_map_.size()==2&&supported.edgeCount()==1,
      "A supported ground sample stopped expanding after the hanging batch change");
    TerrainProjector supported_terrain(supported_map,ground_parameters.groundParameters());
    for(const auto& entry:supported.graph().vertices_map_){
      require(!entry.second->is_hanging&&
        supported_terrain.project(entry.second->state).status==GroundStatus::Supported,
        "A persistent background vertex lacks known ground support");
    }

    auto real_map=freeMap();Parameters real_parameters=p;real_parameters.axis_sampling={};
    GlobalMemory real(real_parameters);real.updateMap(real_map);
    require(real.recordOdometry(StateVec::Zero()),"Cannot initialize the timed growth scene");
    LocalPlanner evaluator(real_map,real_parameters);
    const auto measured=real.expandBackgroundTimed(.1,[&](const StateVec& state){return evaluator.rayGain(state);});
    require(measured.added>0&&measured.time_budget_reached&&measured.elapsed_seconds>=.1&&
      measured.stop_reason=="time_budget","The real steady-clock run did not grow to its soft deadline");
    for(const auto& vertex:real.graph().vertices_map_)for(const auto& edge:real.graph().edge_map_.at(vertex.first)){
      require(real_map->path(vertex.second->state.head<3>(),
        real.graph().vertices_map_.at(edge.first)->state.head<3>(),real_parameters.robot_size)==VoxelStatus::Free,
        "The timed run produced an occupied graph edge");
    }
    std::cout<<std::setprecision(12)<<"{\"status\":\"passed\",\"sample_vertex_draw_limit\":1000,";
    print("fixed_raw_baseline",baseline);std::cout<<',';print("two_cluster_batch",batches);std::cout<<',';
    print("accepted_then_sparse",sparse);std::cout<<',';print("history_boundary",history);std::cout<<',';
    print("blocked_edges",collision);std::cout<<',';print("hanging_free_body",hanging_timed);std::cout<<',';
    print("hanging_raw_draws",hanging_raw);std::cout<<',';print("hanging_unknown_body",hanging_unknown_body);std::cout<<',';
    print("hanging_occupied_body",hanging_occupied_body);std::cout<<',';print("supported_ground",supported_stats);std::cout<<',';
    print("steady_clock_growth",measured);
    std::cout<<",\"clock_source\":\"steady_clock\",\"deadline_scope\":\"upstream_outer_round\"}\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
