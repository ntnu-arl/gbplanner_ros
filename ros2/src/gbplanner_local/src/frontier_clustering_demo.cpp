// Actual absorbLocal decisions with optional, call-local clustering snapshots.
#include "gbplanner_local/global_memory.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <tuple>

using namespace gbplanner2;
namespace {
void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
StateVec pose(double x,double y=0.,double z=.6){return StateVec(x,y,z,0.);}
bool samePath(const std::vector<StateVec>& a,const std::vector<StateVec>& b){
  if(a.size()!=b.size())return false;
  for(std::size_t i=0;i<a.size();++i)if((a[i]-b[i]).norm()>1e-12)return false;
  return true;
}
VolumetricGain frontier(const StateVec& state){
  VolumetricGain gain;gain.is_frontier=true;gain.gain=10.+state.x()+state.y();
  gain.num_unknown_voxels=10;return gain;
}
class CountingMap final:public MapQuery {
 public:
  explicit CountingMap(std::shared_ptr<const MapQuery> map):map_(std::move(map)){}
  double resolution() const override {++queries[0];return map_->resolution();}
  Eigen::Vector3d minimum() const override {++queries[1];return map_->minimum();}
  Eigen::Vector3d maximum() const override {++queries[2];return map_->maximum();}
  VoxelStatus voxel(const Eigen::Vector3d& point) const override {++queries[3];return map_->voxel(point);}
  std::optional<TsdfDistance> tsdfDistance(const Eigen::Vector3d& point) const override {
    ++queries[4];return map_->tsdfDistance(point);
  }
  std::optional<TsdfDistance> tsdfDistanceSimplex(const Eigen::Vector3d& point) const override {
    ++queries[5];return map_->tsdfDistanceSimplex(point);
  }
  VoxelStatus box(const Eigen::Vector3d& point,const Eigen::Vector3d& size) const override {
    ++queries[6];return map_->box(point,size);
  }
  VoxelStatus path(const Eigen::Vector3d& from,const Eigen::Vector3d& to,
                   const Eigen::Vector3d& size) const override {
    ++queries[7];return map_->path(from,to,size);
  }
  bool gainAllowed(const Eigen::Vector3d& point) const override {++queries[8];return map_->gainAllowed(point);}
  void ray(const Eigen::Vector3d& from,const Eigen::Vector3d& to,
           const std::function<bool(const Eigen::Vector3d&,VoxelStatus)>& visit) const override {
    ++queries[9];map_->ray(from,to,visit);
  }
  mutable std::array<std::size_t,10> queries{};
 private:
  std::shared_ptr<const MapQuery> map_;
};
std::shared_ptr<VoxelMap> scene(bool ground=false){
  auto map=std::make_shared<VoxelMap>(.1,Eigen::Vector3d(-2.,-2.,-.5),Eigen::Vector3i(90,90,25));
  map->fill(VoxelStatus::Free);
  if(ground)for(int z=0;z<5;++z)for(int y=0;y<90;++y)for(int x=0;x<90;++x)
    map->set(Eigen::Vector3i(x,y,z),VoxelStatus::Occupied);
  return map;
}
planner_msgs::msg::Graph makeGraph(const std::vector<StateVec>& states,
    const std::vector<std::tuple<int,int,double>>& edges,const std::vector<int>& frontiers){
  GraphManager graph;
  for(std::size_t id=0;id<states.size();++id)graph.addVertex(new Vertex(static_cast<int>(id),states[id]));
  for(const auto& [from,to,weight]:edges)graph.addEdge(graph.getVertex(from),graph.getVertex(to),weight);
  for(int id:frontiers)graph.getVertex(id)->vol_gain=frontier(states[id]);
  planner_msgs::msg::Graph message;graph.convertGraphToMsg(message);return message;
}
planner_msgs::msg::Graph branches(){
  std::vector<StateVec> states{pose(0.)};
  std::vector<std::tuple<int,int,double>> edges;
  for(int id=1;id<=5;++id){states.push_back(pose(id));edges.emplace_back(id-1,id,1.);}
  states.push_back(pose(4.6,.1));edges.emplace_back(4,6,std::sqrt(.37));
  for(int step=1;step<=4;++step){
    const int id=6+step;states.push_back(pose(0.,step));edges.emplace_back(step==1?0:id-1,id,1.);
  }
  return makeGraph(states,edges,{5,6,10});
}
void sameMemory(GlobalMemory& off,GlobalMemory& on){
  planner_msgs::msg::Graph off_message,on_message;off.graphMessage(off_message);on.graphMessage(on_message);
  require(off_message==on_message&&off.graph().edge_map_==on.graph().edge_map_&&
          off.snapshotRevision()==on.snapshotRevision(),"Clustering capture changed the public graph or revision");
  for(const auto& [id,a]:off.graph().vertices_map_){
    const auto* b=on.graph().vertices_map_.at(id);
    require((a->state-b->state).norm()==0.&&a->type==b->type&&a->distance==b->distance&&
            a->cluster_id==b->cluster_id&&a->vol_gain.gain==b->vol_gain.gain&&
            a->vol_gain.accumulative_gain==b->vol_gain.accumulative_gain&&
            a->vol_gain.is_frontier==b->vol_gain.is_frontier,
            "Clustering capture changed stored vertex state or gain");
  }
  const auto& a=off.lastFrontierUpdate();const auto& b=on.lastFrontierUpdate();
  require(std::tie(a.candidates,a.clusters,a.near_existing,a.near_visited,a.invalid_paths,a.added)==
          std::tie(b.candidates,b.clusters,b.near_existing,b.near_visited,b.invalid_paths,b.added),
          "Clustering capture changed the frontier insertion counters");
}
struct Pair {
  std::shared_ptr<CountingMap> off_map,on_map;
  GlobalMemory off,on;
  std::size_t gain_calls{0};
  Pair(const Parameters& p,const std::shared_ptr<VoxelMap>& map)
      :off_map(std::make_shared<CountingMap>(map)),on_map(std::make_shared<CountingMap>(map)),off(p),on(p){
    off.updateMap(off_map);on.updateMap(on_map);
    require(off.setHomingPosition(pose(0.))&&on.setHomingPosition(pose(0.)),"Failed to set comparison homes");
  }
  void absorb(const planner_msgs::msg::Graph& graph,FrontierClusteringDiagnostics& diagnostics){
    off_map->queries={};on_map->queries={};std::vector<StateVec> off_gains,on_gains;
    off.absorbLocal(graph,[&](const StateVec& state){off_gains.push_back(state);return frontier(state);});
    on.absorbLocal(graph,[&](const StateVec& state){on_gains.push_back(state);return frontier(state);},&diagnostics);
    sameMemory(off,on);
    require(samePath(off_gains,on_gains),"Clustering capture changed gain callback count or viewpoints");
    require(off_map->queries==on_map->queries,"Clustering capture performed additional map queries");
    gain_calls+=on_gains.size();
  }
};
const FrontierClusterPath& candidate(const FrontierClusteringDiagnostics& diagnostics,int id){
  const auto found=std::find_if(diagnostics.candidates.begin(),diagnostics.candidates.end(),
      [&](const auto& path){return path.vertex_id==id;});
  require(found!=diagnostics.candidates.end(),"Expected candidate is missing from the snapshot");return *found;
}
void empty(const FrontierClusteringDiagnostics& diagnostics,bool evaluated){
  require(diagnostics.evaluated==evaluated&&diagnostics.candidate_count==0&&
          diagnostics.representative_ids.empty()&&diagnostics.candidates.empty(),
          "A fresh clustering call retained old diagnostic data");
}
}

int main(){
  try{
    Parameters parameters;parameters.robot_size={.1,.1,.1};parameters.overshoot=0.;
    auto map=scene();Pair aerial(parameters,map);FrontierClusteringDiagnostics diagnostics;
    const auto local=branches();aerial.absorb(local,diagnostics);
    require(diagnostics.evaluated&&diagnostics.candidate_count==3&&diagnostics.candidates.size()==3&&
            diagnostics.representative_ids==std::vector<int>({5,10})&&
            candidate(diagnostics,5).cluster_id==5&&candidate(diagnostics,6).cluster_id==5&&
            candidate(diagnostics,10).cluster_id==10&&aerial.on.lastFrontierUpdate().added==2,
            "Actual clustering did not retain the two same/different path groups");
    require(samePath(candidate(diagnostics,6).path,
            {pose(0.),pose(1.),pose(2.),pose(3.),pose(4.),pose(4.6,.1)}),
            "Cluster snapshot differs from the complete local shortest path");
    const auto initial_representatives=diagnostics.representative_ids;
    aerial.absorb(local,diagnostics);
    require(aerial.on.lastFrontierUpdate().near_existing==2&&aerial.on.lastFrontierUpdate().added==0&&
            diagnostics.representative_ids==initial_representatives&&diagnostics.candidates.size()==3,
            "Near-existing rejection removed the clustering decision snapshot");

    Pair visited(parameters,map);
    // Only an actual 1 m event enters history. The representative at x=2 is
    // exactly 3 m from this visit, while remaining >1 m from existing graph.
    require(visited.off.recordOdometry(pose(-1.))&&visited.on.recordOdometry(pose(-1.)),
      "Could not establish the visited-history exclusion fixture");
    visited.absorb(makeGraph({pose(0.),pose(1.),pose(2.)},{{0,1,1.},{1,2,1.}},{2}),diagnostics);
    require(visited.on.lastFrontierUpdate().near_visited==1&&visited.on.lastFrontierUpdate().added==0&&
            diagnostics.representative_ids==std::vector<int>({2})&&candidate(diagnostics,2).path.size()==3,
            "Visited-history rejection removed the pre-exclusion representative");

    Pair short_paths(parameters,map);
    short_paths.absorb(makeGraph({pose(0.),pose(.6)},{{0,1,.6}},{1}),diagnostics);
    require(diagnostics.evaluated&&diagnostics.candidate_count==1&&diagnostics.representative_ids.empty()&&
            candidate(diagnostics,1).cluster_id==1&&candidate(diagnostics,1).path.size()==2&&
            short_paths.on.lastFrontierUpdate().clusters==0,
            "Removed short representative was replaced by a fabricated surviving cluster");
    const auto removed_assignment=candidate(diagnostics,1).cluster_id;
    short_paths.absorb(makeGraph({pose(0.),pose(2.)},{{0,1,2.}},{}),diagnostics);empty(diagnostics,true);

    auto ground_parameters=parameters;ground_parameters.ground_robot=true;
    ground_parameters.ground.max_ground_height=.6;ground_parameters.ground.robot_height=.4;
    Pair ground(ground_parameters,scene(true));
    // Stored undirected weights prefer the detour through vertex 2. Ground
    // absorbLocal computes its actual directed terrain costs and selects 1.
    const auto ground_graph=makeGraph({pose(0.),pose(1.),pose(0.,2.),pose(4.)},
        {{0,1,100.},{1,3,100.},{0,2,.1},{2,3,.1}},{3});
    ground.absorb(ground_graph,diagnostics);
    const auto& ground_candidate=candidate(diagnostics,3);
    require(diagnostics.evaluated&&diagnostics.candidate_count==1&&
            samePath(ground_candidate.path,{pose(0.),pose(1.),pose(4.)})&&
            diagnostics.representative_ids==std::vector<int>({3})&&ground.on.lastFrontierUpdate().added==1,
            "Ground snapshot used stale wire weights, base height or regenerated terrain knots");
    const auto ground_points=ground_candidate.path.size();
    GraphManager wire_graph;wire_graph.convertMsgToGraph(ground_graph);ShortestPathsReport wire_report;
    wire_graph.findShortestPaths(wire_report);std::vector<StateVec> wire_path;
    wire_graph.getShortestPath(3,wire_report,true,wire_path);
    require(samePath(wire_path,{pose(0.),pose(0.,2.),pose(4.)}),"Ground test did not distinguish wire and actual reports");

    // Call-local ownership means reset/load have no stored display snapshot.
    // Reusing an output object still overwrites it on every early return.
    ground.on.reset();ground.on.absorbLocal(ground_graph,frontier,&diagnostics);empty(diagnostics,false);
    GlobalMemory no_map(parameters);diagnostics.evaluated=true;diagnostics.candidate_count=7;
    diagnostics.representative_ids={7};diagnostics.candidates.push_back({7,7,{pose(0.)}});
    no_map.absorbLocal(local,frontier,&diagnostics);empty(diagnostics,false);

    std::cout<<std::setprecision(15)<<"{\"status\":\"passed\",\"candidates\":3,\"representatives\":[5,10]"
      <<",\"same_cluster_members\":[5,6],\"near_existing_preserved\":2,\"near_visited_preserved\":1"
      <<",\"removed_representative_assignment\":"<<removed_assignment
      <<",\"ground_report_points\":"<<ground_points
      <<",\"compared_gain_calls\":"<<aerial.gain_calls+visited.gain_calls+short_paths.gain_calls+ground.gain_calls
      <<",\"global_graph_and_counts_unchanged\":true,\"gain_viewpoints_unchanged\":true"
      <<",\"additional_map_queries\":0,\"no_candidate_clears\":true,\"early_return_clears\":true}\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
