// One frozen native-map request, with and without exact gain reuse.
// The companion Python tool generates parameters from the installed RMF YAML.
#include "gbplanner_local/local_planner.hpp"
#include "gbplanner_local/global_memory.hpp"
#include "gbplanner_local/sparse_voxel_map.hpp"
#include "gbplanner_local/global_bound.hpp"
#include "gbplanner_local/robot_bounds.hpp"
#include <chrono>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <time.h>
#include "rmf_gain_replay_parameters.hpp"

using namespace gbplanner2;
using Clock=std::chrono::steady_clock;
void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
double ms(Clock::time_point t){return std::chrono::duration<double,std::milli>(Clock::now()-t).count();}
std::int64_t threadCpuNanoseconds(){
  timespec value{};
  require(clock_gettime(CLOCK_THREAD_CPUTIME_ID,&value)==0,"Thread CPU clock unavailable");
  return static_cast<std::int64_t>(value.tv_sec)*1000000000+value.tv_nsec;
}
float f32(const unsigned char* p){
  uint32_t bits=uint32_t(p[0])|uint32_t(p[1])<<8|uint32_t(p[2])<<16|uint32_t(p[3])<<24;
  float value;std::memcpy(&value,&bits,4);return value;
}
struct Run {
  PlanResult plan;
  planner_msgs::msg::Graph local,global;
  ExactRayGainCacheStats cache;
  double plan_ms{0},absorb_ms{0},total_ms{0};
  double plan_thread_cpu_ms{0},total_thread_cpu_ms{0};
  bool ray_invariants{false};
  int frontiers{0};
  std::string sampling_budget_mode;
  int max_attempts{0},max_sampling_rounds{0},empty_graph_cutoff{0};
};
Run run(std::shared_ptr<const MapQuery> map,bool reuse,std::optional<bool> reuse_ray_invariants=std::nullopt){
  Run out;auto p=replayParameters();const auto root=replayRoot();
  if(reuse_ray_invariants)p.gain.reuse_ray_invariants=*reuse_ray_invariants;
  out.ray_invariants=p.gain.reuse_ray_invariants;
  out.sampling_budget_mode=p.sampling_budget_mode;out.max_attempts=p.max_attempts;
  out.max_sampling_rounds=p.max_sampling_rounds;out.empty_graph_cutoff=p.empty_graph_cutoff;
  GlobalMemory memory(p);memory.updateMap(map);
  require(memory.setHomingPosition(root),"Replay home could not be set");
  require(memory.recordOdometry(root),"Replay home is not free");
  LocalPlanner planner(map,p,{},reuse?ExactRayGainReuse::ImmutableSnapshot:ExactRayGainReuse::Disabled);
  const auto cpu_begin=threadCpuNanoseconds();
  const auto begin=Clock::now();out.plan=planner.plan(root,replayDirection());out.plan_ms=ms(begin);
  out.plan_thread_cpu_ms=(threadCpuNanoseconds()-cpu_begin)*1.e-6;
  planner.graphMessage(out.local);
  const auto before=Clock::now();
  memory.absorbLocal(out.local,[&](const StateVec& s){return planner.rayGain(s);});
  out.absorb_ms=ms(before);out.total_ms=ms(begin);
  out.total_thread_cpu_ms=(threadCpuNanoseconds()-cpu_begin)*1.e-6;
  memory.graphMessage(out.global);out.frontiers=memory.frontierCount();
  out.cache=planner.exactRayGainCacheStats();return out;
}
void writeRun(const Run& r){
  std::cout<<"{\"success\":"<<(r.plan.success?"true":"false")
    <<",\"reason\":\""<<r.plan.reason<<'"'
    <<",\"graph_build_completed\":"<<(r.plan.graph_build_completed?"true":"false")
    <<",\"gain_evaluated\":"<<(r.plan.gain_evaluated?"true":"false")
    <<",\"vertices\":"<<r.plan.vertices<<",\"edges\":"<<r.plan.edges
    <<",\"draws\":"<<r.plan.attempts<<",\"gain\":"<<r.plan.gain
    <<",\"attempts\":"<<r.plan.attempts<<",\"raw_draws\":"<<r.plan.attempts
    <<",\"sampling_budget_mode\":"<<std::quoted(r.sampling_budget_mode)
    <<",\"max_attempts\":"<<r.max_attempts<<",\"max_sampling_rounds\":"<<r.max_sampling_rounds
    <<",\"sampling_budget_limit\":"<<(r.sampling_budget_mode=="sampling_rounds"?r.max_sampling_rounds:r.max_attempts)
    <<",\"sampling_stop_reason\":"<<std::quoted(r.plan.sampling_stop_reason)
    <<",\"empty_graph_cutoff\":"<<r.empty_graph_cutoff
    <<",\"empty_graph_cutoff_reached\":"<<(r.plan.empty_graph_cutoff_reached?"true":"false")
    <<",\"sampling_rounds\":"<<r.plan.sampling_rounds<<",\"collision_rejections\":"<<r.plan.collision_rejections
    <<",\"gain_evaluations\":"<<r.plan.gain_evaluations
    <<",\"cluster_reuses\":"<<r.plan.reused_gains<<",\"frontiers\":"<<r.frontiers
    <<",\"plan_ms\":"<<r.plan_ms<<",\"absorb_ms\":"<<r.absorb_ms<<",\"total_ms\":"<<r.total_ms
    <<",\"plan_thread_cpu_ms\":"<<r.plan_thread_cpu_ms<<",\"total_thread_cpu_ms\":"<<r.total_thread_cpu_ms
    <<",\"cpu_clock\":\"CLOCK_THREAD_CPUTIME_ID\",\"ray_invariants\":"<<(r.ray_invariants?"true":"false")
    <<",\"build_ms\":"<<r.plan.build_ms<<",\"evaluate_ms\":"<<r.plan.evaluate_ms
    <<",\"gain_ms\":"<<r.plan.gain_ms<<",\"refinement_ms\":"<<r.plan.refinement_ms
    <<",\"evaluate_other_ms\":"<<r.plan.evaluate_ms-r.plan.gain_ms-r.plan.refinement_ms
    <<",\"refinement\":{\"attempted\":"<<(r.plan.refinement.attempted?"true":"false")
    <<",\"modified\":"<<(r.plan.refinement.modified?"true":"false")
    <<",\"reason\":\""<<r.plan.refinement.reason<<"\",\"removed_vertices\":"<<r.plan.refinement.removed_vertices
    <<",\"adjusted_vertices\":"<<r.plan.refinement.adjusted_vertices<<'}'
    <<",\"cache_hits\":"<<r.cache.hits<<",\"cache_evaluations\":"<<r.cache.evaluations
    <<",\"cache_entries\":"<<r.cache.entries<<'}';
}
void requireEqual(const Run& first,const Run& second){
  require(first.local==second.local,"Local graph differs");
  require(first.global==second.global,"Global graph differs");
  require(first.frontiers==second.frontiers&&first.plan.gain==second.plan.gain&&
          first.sampling_budget_mode==second.sampling_budget_mode&&
          first.max_attempts==second.max_attempts&&first.max_sampling_rounds==second.max_sampling_rounds&&
          first.plan.attempts==second.plan.attempts&&first.plan.sampling_rounds==second.plan.sampling_rounds&&
          first.plan.sampling_stop_reason==second.plan.sampling_stop_reason&&
          first.plan.empty_graph_cutoff_reached==second.plan.empty_graph_cutoff_reached&&
          first.plan.collision_rejections==second.plan.collision_rejections&&
          first.plan.gain_evaluations==second.plan.gain_evaluations&&first.plan.reused_gains==second.plan.reused_gains,
          "Planning score or work counts differ");
  require(first.plan.success==second.plan.success&&first.plan.reason==second.plan.reason&&
          first.plan.path_length==second.plan.path_length&&first.plan.candidate_path_length==second.plan.candidate_path_length&&
          first.plan.traverse_length==second.plan.traverse_length&&first.plan.traverse_time==second.plan.traverse_time&&
          first.plan.direction_distance==second.plan.direction_distance&&first.plan.direction_factor==second.plan.direction_factor,
          "Planning result differs");
  require(first.plan.path.size()==second.plan.path.size(),"Path size differs");
  for(std::size_t i=0;i<first.plan.path.size();++i)
    require((first.plan.path[i].array()==second.plan.path[i].array()).all(),"Path differs");
}
int main(int argc,char** argv){try{
  require(argc==2,"Usage: replay_rmf_gain_cache accepted_map.bin");
  std::ifstream file(argv[1],std::ios::binary|std::ios::ate);
  require(bool(file),"Map file missing");const auto bytes=file.tellg();
  require(bytes>0&&bytes%24==0,"Unexpected map record size");
  std::vector<unsigned char> data(static_cast<std::size_t>(bytes));file.seekg(0);
  require(bool(file.read(reinterpret_cast<char*>(data.data()),bytes)),"Incomplete map read");
  auto grid=std::make_shared<SparseVoxelMap>(replayResolution());
  for(std::size_t offset=0;offset<data.size();offset+=24){
    const auto* cell=data.data()+offset;
    const Eigen::Vector3d point(f32(cell),f32(cell+4),f32(cell+8));
    int8_t occupancy;std::memcpy(&occupancy,cell+12,1);
    const auto index=grid->index(point);grid->set(index,static_cast<VoxelStatus>(occupancy));
    grid->setTsdf(index,f32(cell+16),f32(cell+20));
  }
  require(grid->size()==data.size()/24,"Map cells duplicated");
  const auto map=std::make_shared<BoundedMap>(grid,replayBound());
  if(replayCompareRayInvariants()){
    Run baseline,optimized;
    if(replayOptimizedFirst()){optimized=run(map,true,true);baseline=run(map,true,false);}
    else {baseline=run(map,true,false);optimized=run(map,true,true);}
    requireEqual(baseline,optimized);
    require(baseline.cache.hits==optimized.cache.hits&&baseline.cache.evaluations==optimized.cache.evaluations&&
            baseline.cache.entries==optimized.cache.entries,"Exact-pose cache work differs");
    std::cout<<std::setprecision(17)<<"{\"status\":\"passed\",\"scope\":\"One frozen-map endpoint pair; only constant ray work differs, exact pose-gain reuse enabled in both\",\"cells\":"
      <<grid->size()<<",\"exact_path_local_global_graph_score_draws_and_cache_work_equal\":true"
      <<",\"pair_order\":\""<<(replayOptimizedFirst()?"optimized-first":"baseline-first")<<'"'
      <<",\"gain_and_refinement_included_in_evaluate_ms\":true,\"baseline\":";
    writeRun(baseline);std::cout<<",\"optimized\":";writeRun(optimized);std::cout<<"}\n";return 0;
  }
  if(replayAheadMode()){
    const auto cached=run(map,true);
    std::cout<<std::setprecision(17)<<"{\"status\":\"measured\",\"scope\":\"One frozen-map endpoint core probe with explicit seed/direction and a fresh synthetic global graph; not historical PlanAhead service replay\",\"cells\":"
      <<grid->size()<<",\"gain_and_refinement_included_in_evaluate_ms\":true,\"cached\":";
    writeRun(cached);std::cout<<"}\n";return 0;
  }
  const auto ordinary=run(map,false),cached=run(map,true);
  require(ordinary.plan.success&&cached.plan.success,"Replay did not yield a valid path");
  requireEqual(ordinary,cached);
  require(cached.cache.hits>0,"No actual exact-gain reuse observed");
  std::cout<<std::setprecision(17)<<"{\"status\":\"passed\",\"scope\":\"One offline request on the final native map; not a physical flight or original-time decision replay\",\"cells\":"
    <<grid->size()<<",\"exact_path_local_global_graph_score_and_draws_equal\":true,\"baseline\":";
  writeRun(ordinary);std::cout<<",\"cached\":";writeRun(cached);std::cout<<"}\n";
  return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
