// Finite frozen-map probes using production terrain/body/edge queries.
// No ROS node, simulator, map edits, gain evaluation or private-type inference.
#include "gbplanner_local/local_planner.hpp"
#include "gbplanner_local/sparse_voxel_map.hpp"
#include "gbplanner_local/global_bound.hpp"
#include "gbplanner_local/robot_bounds.hpp"
#include <algorithm>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include "smb_background_diagnostic_parameters.hpp"

using namespace gbplanner2;
namespace {
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
float f32(const unsigned char* p){
  uint32_t bits=uint32_t(p[0])|uint32_t(p[1])<<8|uint32_t(p[2])<<16|uint32_t(p[3])<<24;
  float value;std::memcpy(&value,&bits,4);return value;
}
template<class T>void vectorJson(const T& state){
  std::cout<<'[';for(int i=0;i<state.size();++i){if(i)std::cout<<',';std::cout<<state[i];}std::cout<<']';
}
void numberJson(double value){if(std::isfinite(value))std::cout<<value;else std::cout<<"null";}
const char* occupancyName(VoxelStatus status){
  return status==VoxelStatus::Free?"free":status==VoxelStatus::Occupied?"occupied":"unknown";
}
struct QueryFailure {
  const char* kind;VoxelStatus status;Eigen::Vector3d from,to,size;
};
// Each production query is forwarded exactly once. Missing TSDF reads can be
// normal projection searches, so retain failed body queries, not speculative
// interpretations of missing interpolation samples.
class RecordingMap final:public MapQuery {
 public:
  explicit RecordingMap(std::shared_ptr<const MapQuery> map):map_(std::move(map)){}
  double resolution()const override{return map_->resolution();}
  Eigen::Vector3d minimum()const override{return map_->minimum();}
  Eigen::Vector3d maximum()const override{return map_->maximum();}
  VoxelStatus voxel(const Eigen::Vector3d& p)const override{return map_->voxel(p);}
  std::optional<TsdfDistance> tsdfDistance(const Eigen::Vector3d& p)const override{return map_->tsdfDistance(p);}
  std::optional<TsdfDistance> tsdfDistanceSimplex(const Eigen::Vector3d& p)const override{return map_->tsdfDistanceSimplex(p);}
  VoxelStatus box(const Eigen::Vector3d& p,const Eigen::Vector3d& size)const override{
    const auto status=map_->box(p,size);record("box",status,p,p,size);return status;
  }
  VoxelStatus path(const Eigen::Vector3d& a,const Eigen::Vector3d& b,const Eigen::Vector3d& size)const override{
    const auto status=map_->path(a,b,size);record("path",status,a,b,size);return status;
  }
  void ray(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
           const std::function<bool(const Eigen::Vector3d&,VoxelStatus)>& visit)const override{map_->ray(a,b,visit);}
  bool gainAllowed(const Eigen::Vector3d& p)const override{return map_->gainAllowed(p);}
  mutable std::vector<QueryFailure> failures;
 private:
  void record(const char* kind,VoxelStatus status,const Eigen::Vector3d& a,
              const Eigen::Vector3d& b,const Eigen::Vector3d& size)const{
    if(status!=VoxelStatus::Free&&failures.size()<8)failures.push_back({kind,status,a,b,size});
  }
  std::shared_ptr<const MapQuery> map_;
};
struct Diagnosis {
  std::string stage;int nearest{-1};StateVec nearest_state{StateVec::Zero()},steered{StateVec::Zero()};
  GroundProjection projection;bool has_steering{false},has_edge{false};GroundEdge edge;
};
// Limited read-only branch inspection AFTER shared expandGround fails. This
// does not decide admission or mutate a graph; the shared call supplies the
// actual result, and this inspection identifies its geometric obstruction.
Diagnosis diagnose(GraphManager& graph,const std::shared_ptr<const MapQuery>& map,
                   const Parameters& p,StateVec state){
  Diagnosis d;TerrainProjector terrain(map,p.groundParameters());
  auto projection=terrain.project(state);
  if(projection.status!=GroundStatus::Supported){d.stage="sample_projection";d.projection=projection;return d;}
  state=projection.projected_state;
  if(map->box(terrain.baseState(state).head<3>()+p.center_offset,p.robot_size)!=VoxelStatus::Free){
    d.stage="sample_body";d.projection=projection;return d;
  }
  Vertex* nearest=nullptr;
  if(!graph.getNearestVertex(&state,&nearest)){d.stage="nearest_vertex_missing";return d;}
  d.nearest=nearest->id;d.nearest_state=nearest->state;
  const Eigen::Vector3d direction=(state-nearest->state).head<3>();const double distance=direction.norm();
  if(distance<=p.edge_min){d.stage="sample_edge_too_short";return d;}
  if(distance>p.edge_max)state.head<3>()=nearest->state.head<3>()+p.edge_max*direction/distance;
  d.has_steering=true;d.steered=state;d.projection=terrain.project(state);
  if(d.projection.status!=GroundStatus::Supported){d.stage="steered_projection";return d;}
  state=d.projection.projected_state;
  if(map->box(terrain.baseState(state).head<3>()+p.center_offset,p.robot_size)!=VoxelStatus::Free){
    d.stage="steered_body";return d;
  }
  if((state-nearest->state).head<3>().norm()<=p.edge_min){d.stage="steered_edge_too_short";return d;}
  d.has_edge=true;d.edge=terrain.candidateEdge(nearest->state,state,GroundHangingPermission::Forbid);
  d.stage=d.edge.admissible()?"parent_edge_admissible_check_overshoot_or_other":"parent_edge";
  return d;
}
struct Example {
  int origin_vertex{-1};StateVec raw,projected;
  double visited_distance{0.},graph_distance{0.};bool expanded{false};int collision_rejections{0};
  StateVec endpoint{StateVec::Zero()};Diagnosis diagnosis;std::vector<QueryFailure> failures;
};
void exampleJson(const Example& e){
  std::cout<<"{\"origin_vertex\":"<<e.origin_vertex<<",\"raw\":";vectorJson(e.raw);
  std::cout<<",\"projected_sample\":";vectorJson(e.projected);
  std::cout<<",\"nearest_visited_distance_m\":";numberJson(e.visited_distance);
  std::cout<<",\"nearest_graph_distance_m\":"<<e.graph_distance
    <<",\"expanded\":"<<(e.expanded?"true":"false")<<",\"collision_rejections\":"<<e.collision_rejections;
  if(e.expanded){std::cout<<",\"added_endpoint\":";vectorJson(e.endpoint);}
  else{
    const auto& d=e.diagnosis;
    std::cout<<",\"rejection_stage\":"<<std::quoted(d.stage)<<",\"nearest_vertex\":"<<d.nearest;
    if(d.nearest>=0){std::cout<<",\"nearest_state\":";vectorJson(d.nearest_state);}
    if(d.has_steering){std::cout<<",\"steered_before_projection\":";vectorJson(d.steered);}
    std::cout<<",\"projection_status\":"<<std::quoted(groundStatusName(d.projection.status))
      <<",\"projection_state\":";vectorJson(d.projection.projected_state);
    std::cout<<",\"projection_ground_z\":";numberJson(d.projection.ground_z);
    if(d.has_edge){
      std::cout<<",\"parent_edge_status\":"<<std::quoted(groundStatusName(d.edge.status))
        <<",\"parent_edge_max_uphill_rad\":"<<d.edge.max_uphill
        <<",\"parent_edge_max_downhill_rad\":"<<d.edge.max_downhill<<",\"parent_edge_polyline\":[";
      for(std::size_t i=0;i<d.edge.polyline.size();++i){if(i)std::cout<<',';vectorJson(d.edge.polyline[i]);}std::cout<<']';
    }
  }
  std::cout<<",\"shared_expansion_failed_body_queries\":[";
  for(std::size_t i=0;i<e.failures.size();++i){
    if(i)std::cout<<',';const auto& q=e.failures[i];
    std::cout<<"{\"kind\":"<<std::quoted(q.kind)<<",\"status\":"<<std::quoted(occupancyName(q.status))<<",\"from\":";
    vectorJson(q.from);std::cout<<",\"to\":";vectorJson(q.to);std::cout<<",\"size\":";vectorJson(q.size);std::cout<<'}';
  }
  std::cout<<"]}";
}
}

int main(int argc,char** argv){try{
  require(argc==2,"Usage: diagnose_smb_background accepted_map.bin");
  const auto started=std::chrono::steady_clock::now();
  std::ifstream file(argv[1],std::ios::binary|std::ios::ate);
  require(bool(file),"Saved map missing");const auto bytes=file.tellg();
  require(bytes>0&&bytes%24==0,"Unexpected map record size");
  std::vector<unsigned char> data(static_cast<std::size_t>(bytes));file.seekg(0);
  require(bool(file.read(reinterpret_cast<char*>(data.data()),bytes)),"Incomplete map read");
  auto grid=std::make_shared<SparseVoxelMap>(diagnosticResolution());grid->reserve(data.size()/24);
  for(std::size_t offset=0;offset<data.size();offset+=24){
    const auto* cell=data.data()+offset;const Eigen::Vector3d point(f32(cell),f32(cell+4),f32(cell+8));
    int8_t occupancy;std::memcpy(&occupancy,cell+12,1);
    require(occupancy==-1||occupancy==0||occupancy==100,"Unknown occupancy label");
    const auto index=grid->index(point);grid->set(index,static_cast<VoxelStatus>(occupancy));
    grid->setTsdf(index,f32(cell+16),f32(cell+20));
  }
  require(grid->size()==data.size()/24,"Duplicate map cells");
  const auto p=diagnosticParameters();const auto visits=diagnosticVisits(),vertices=diagnosticVertices();
  const auto centers=diagnosticCenters();const auto center_ids=diagnosticCenterIds();
  const auto message=diagnosticGraph();require(!message.vertices.empty(),"Saved graph is empty");
  const auto map=std::make_shared<BoundedMap>(grid,diagnosticBound());TerrainProjector terrain(map,p.groundParameters());
  GraphManager original;original.convertMsgToGraph(message);
  const auto world_to_sampling=p.worldToSampling();LocalSampler sampler(p);
  std::map<std::string,std::uint64_t> outcomes;std::vector<Example> examples;
  int tested=0,accepted=0,supported=0,outside_visited=0,outside_graph=0,expanded=0;
  for(int i=0;i<diagnosticRandomDraws()&&examples.size()<static_cast<std::size_t>(diagnosticMaxExpansions());++i){
    const auto origin=i%centers.size();const auto& center=centers[origin];
    const StateVec raw=sampler.generate(center);++tested;const auto projection=terrain.project(raw);
    const auto reject=[&](const std::string& reason){++outcomes[reason];};
    if(projection.status!=GroundStatus::Supported&&projection.status!=GroundStatus::Hanging){
      reject(std::string("projection_")+groundStatusName(projection.status));continue;
    }
    const auto sample=projection.projected_state;
    const Eigen::Vector3d relative=world_to_sampling*(sample.head<3>()-center.head<3>());
    if((relative.array()<p.local_min.array()).any()||(relative.array()>p.local_max.array()).any()){
      reject("projected_outside_sampling_bounds");continue;
    }
    const auto body=map->box(terrain.baseState(sample).head<3>()+p.center_offset,p.robot_size);
    if(body!=VoxelStatus::Free){reject(std::string("body_")+occupancyName(body));continue;}
    if(projection.status==GroundStatus::Supported){
      const auto same_site=terrain.edge(sample,sample);
      if(!same_site.admissible()){reject(std::string("same_site_")+groundStatusName(same_site.status));continue;}
    }
    ++accepted;
    if(projection.status==GroundStatus::Hanging){reject("accepted_hanging_discarded");continue;}
    ++supported;Example e;e.origin_vertex=center_ids[origin];e.raw=raw;e.projected=sample;
    e.visited_distance=std::numeric_limits<double>::infinity();e.graph_distance=std::numeric_limits<double>::infinity();
    for(const auto& state:visits)e.visited_distance=std::min(e.visited_distance,(sample-state).head<3>().norm());
    for(const auto& state:vertices)e.graph_distance=std::min(e.graph_distance,(sample-state).head<3>().norm());
    if(e.visited_distance<=5.){reject("near_recorded_visited_state");continue;}++outside_visited;
    if(e.graph_distance<=5.){reject("near_saved_graph");continue;}++outside_graph;
    // Each attempt starts from the same saved graph and immutable map.
    GraphManager graph;graph.convertMsgToGraph(message);PlanResult result;
    auto observed=std::make_shared<RecordingMap>(map);
    e.expanded=expandGround(graph,observed,p,sample,result,100000);
    e.collision_rejections=result.collision_rejections;e.failures=std::move(observed->failures);
    if(e.expanded){++expanded;e.endpoint=graph.getVertex(graph.getNumVertices()-1)->state;reject("shared_expand_ground_added");}
    else{e.diagnosis=diagnose(original,map,p,sample);reject("shared_expand_ground_rejected_"+e.diagnosis.stage);}
    examples.push_back(std::move(e));
  }
  std::cout<<std::setprecision(17)<<"{\"status\":\"measured\",\"cells\":"<<grid->size()
    <<",\"saved_graph_vertices\":"<<original.getNumVertices()<<",\"saved_graph_edges\":"<<original.getNumEdges()
    <<",\"recorded_visited_states\":"<<visits.size()<<",\"grid_scan_performed\":false"
    <<",\"draws\":"<<tested<<",\"accepted_samples\":"<<accepted<<",\"supported_samples\":"<<supported
    <<",\"outside_all_recorded_visits_5m\":"<<outside_visited<<",\"also_outside_graph_5m\":"<<outside_graph
    <<",\"expansion_tested\":"<<examples.size()<<",\"expanded\":"<<expanded<<",\"outcomes\":{";
  bool first=true;for(const auto& entry:outcomes){if(!first)std::cout<<',';first=false;std::cout<<std::quoted(entry.first)<<':'<<entry.second;}
  std::cout<<"},\"expansion_examples\":[";
  for(std::size_t i=0;i<examples.size();++i){if(i)std::cout<<',';exampleJson(examples[i]);}
  std::cout<<"],\"elapsed_seconds\":"<<std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()<<"}\n";
  return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
