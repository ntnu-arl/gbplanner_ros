// Link against the production planner library. No ROS executor or simulation.
// Usage: compare_native_global_update map.bin fixture.txt output_prefix
// Compare one Disabled and one ImmutableSnapshot update of the SAME rebuilt
// public graph. This is not a restore of the unavailable live private metadata.
#include "gbplanner_local/global_memory.hpp"
#include "gbplanner_local/global_bound.hpp"
#include "gbplanner_local/sparse_voxel_map.hpp"
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>

using namespace gbplanner2;
namespace {
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
void vectorIn(std::istream& input,Eigen::Vector3d& value){
  input>>value.x()>>value.y()>>value.z();require(bool(input)&&value.allFinite(),"Invalid vector");
}
struct VertexInput {int id,unknown,free,occupied,frontier;StateVec state;};
struct Fixture {
  double resolution;std::size_t cells;Parameters parameters;Eigen::Vector3d minimum,maximum;
  std::vector<VertexInput> vertices;std::vector<std::tuple<int,int,double>> edges;
};
Fixture readFixture(const char* filename){
  std::ifstream input(filename);std::string version;input>>version;
  require(version=="NATIVE_GLOBAL_UPDATE_V1","Invalid fixture format");Fixture f;
  input>>f.resolution>>f.cells;require(bool(input)&&f.resolution>0.&&f.cells>0,"Invalid map dimensions");
  auto& p=f.parameters;p.ground_robot=true;
  vectorIn(input,p.robot_size);vectorIn(input,p.center_offset);
  auto& g=p.ground;
  input>>g.robot_height>>g.max_ground_height>>g.max_inclination>>g.max_negative_inclination
       >>g.projection_max_length>>g.hanging_vertex_penalty;
  int interpolate;std::string ray,tsdf;input>>interpolate>>ray>>tsdf;
  require(interpolate==0||interpolate==1,"Invalid interpolation policy");
  g.interpolate_projection_distance=interpolate;
  if(ray=="voxel_surface")g.ray_projection_mode=GroundRayProjectionMode::VoxelSurface;
  else if(ray=="upstream_sample")g.ray_projection_mode=GroundRayProjectionMode::UpstreamSample;
  else throw std::runtime_error("Invalid ray projection mode");
  if(tsdf=="upstream_step")g.tsdf_projection_mode=GroundTsdfProjectionMode::UpstreamStep;
  else if(tsdf=="zero_crossing")g.tsdf_projection_mode=GroundTsdfProjectionMode::ZeroCrossing;
  else if(tsdf=="zero_crossing_simplex")g.tsdf_projection_mode=GroundTsdfProjectionMode::ZeroCrossingSimplex;
  else throw std::runtime_error("Invalid TSDF projection mode");
  vectorIn(input,f.minimum);vectorIn(input,f.maximum);p.validate();
  std::size_t vertices,edges;input>>vertices>>edges;
  require(bool(input)&&vertices>0&&vertices<=100000&&edges<=1000000,"Invalid graph counts");
  std::set<int> ids;
  for(std::size_t i=0;i<vertices;++i){
    VertexInput v;input>>v.id>>v.state.x()>>v.state.y()>>v.state.z()>>v.state[3]
      >>v.unknown>>v.free>>v.occupied>>v.frontier;
    require(bool(input)&&v.state.allFinite()&&v.id>=0&&ids.insert(v.id).second&&
            v.unknown>=0&&v.free>=0&&v.occupied>=0&&(v.frontier==0||v.frontier==1),"Invalid vertex");
    f.vertices.push_back(v);
  }
  std::set<std::pair<int,int>> pairs;
  for(std::size_t i=0;i<edges;++i){
    int from,to;double weight;input>>from>>to>>weight;
    require(bool(input)&&ids.count(from)&&ids.count(to)&&from!=to&&std::isfinite(weight)&&weight>=0.&&
            pairs.insert(std::minmax(from,to)).second,"Invalid edge");
    f.edges.emplace_back(from,to,weight);
  }
  std::string extra;require(!(input>>extra),"Trailing fixture data");return f;
}
float floatLE(const std::uint8_t* bytes){
  static_assert(sizeof(float)==4&&std::numeric_limits<float>::is_iec559,"Requires IEEE float32");
  const std::uint32_t bits=std::uint32_t(bytes[0])|(std::uint32_t(bytes[1])<<8)|
    (std::uint32_t(bytes[2])<<16)|(std::uint32_t(bytes[3])<<24);
  float value;std::memcpy(&value,&bits,4);return value;
}
std::shared_ptr<SparseVoxelMap> readMap(const char* filename,const Fixture& fixture){
  require(fixture.cells<=std::size_t(std::numeric_limits<std::streamoff>::max())/24,"Map size overflow");
  const auto length=fixture.cells*24;
  std::ifstream input(filename,std::ios::binary|std::ios::ate);
  require(bool(input)&&input.tellg()==std::streamoff(length),"Map byte length differs from metadata");
  std::vector<std::uint8_t> bytes(length);input.seekg(0);
  require(bool(input.read(reinterpret_cast<char*>(bytes.data()),std::streamsize(length))),"Cannot read map");
  auto map=std::make_shared<SparseVoxelMap>(fixture.resolution);map->reserve(fixture.cells);
  for(std::size_t row=0;row<fixture.cells;++row){
    const auto* data=bytes.data()+row*24;
    const Eigen::Vector3d point(floatLE(data),floatLE(data+4),floatLE(data+8));
    std::int8_t status;std::memcpy(&status,data+12,1);
    map->setObservation(map->index(point),static_cast<VoxelStatus>(status),floatLE(data+16),floatLE(data+20));
  }
  return map; // No invented cells, clearing, filtering, or map changes.
}
void rebuild(GlobalMemory& memory,const Fixture& fixture){
  // Fixture-only construction: do not add a production graph mutation API.
  auto& graph=const_cast<GraphManager&>(memory.graph());
  for(const auto& v:fixture.vertices){
    auto vertex=std::make_unique<Vertex>(v.id,v.state);
    vertex->vol_gain.num_unknown_voxels=v.unknown;vertex->vol_gain.num_free_voxels=v.free;
    vertex->vol_gain.num_occupied_voxels=v.occupied;vertex->vol_gain.is_frontier=v.frontier;
    if(v.frontier)vertex->type=VertexType::kFrontier;
    graph.addVertex(vertex.get());vertex.release();
  }
  for(const auto& [from,to,weight]:fixture.edges)graph.addEdge(graph.getVertex(from),graph.getVertex(to),weight);
}
template<class T>void vectorOut(std::ostream& out,const T& value){
  out<<'[';for(int i=0;i<value.size();++i){if(i)out<<',';out<<value[i];}out<<']';
}
std::string resultGraph(const GlobalMemory& memory){
  // Include every reconstructed vertex and BOTH directed adjacency lists.
  // Serialize internal double costs directly, not rounded Graph.msg float32.
  const auto& graph=memory.graph();std::ostringstream out;out<<std::setprecision(17);
  std::vector<int> ids;for(const auto& item:graph.vertices_map_)ids.push_back(item.first);
  std::sort(ids.begin(),ids.end());out<<"{\"vertices\":[";
  bool first=true;
  for(int id:ids){
    const auto& v=*graph.vertices_map_.at(id);if(!first)out<<',';first=false;
    out<<"{\"id\":"<<id<<",\"state\":";vectorOut(out,v.state);
    out<<",\"unknown\":"<<v.vol_gain.num_unknown_voxels<<",\"free\":"<<v.vol_gain.num_free_voxels
       <<",\"occupied\":"<<v.vol_gain.num_occupied_voxels<<",\"frontier\":"<<(v.vol_gain.is_frontier?"true":"false")
       <<",\"type\":"<<static_cast<int>(v.type)<<'}';
  }
  out<<"],\"directed_adjacency\":[";first=true;
  for(const auto& [from,neighbors]:graph.edge_map_){
    auto sorted=neighbors;std::sort(sorted.begin(),sorted.end());
    for(const auto& [to,weight]:sorted){if(!first)out<<',';first=false;out<<'['<<from<<','<<to<<','<<weight<<']';}
  }
  const auto update=memory.lastMapEdgeUpdate();
  out<<"],\"edge_count\":"<<memory.edgeCount()<<",\"map_edge_update\":{\"invalidated\":"<<update.invalidated
     <<",\"restored\":"<<update.restored<<",\"pending\":"<<update.pending<<"},\"snapshot_revision\":"
     <<memory.snapshotRevision()<<",\"visit_revision\":"<<memory.visitRevision()<<",\"visited_count\":"
     <<memory.visitedStates().size()<<'}';return out.str();
}
double elapsed(std::chrono::steady_clock::time_point start){
  return std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
}
void writeResult(const std::string& path,const std::string& result){
  std::ofstream out(path);out<<result<<'\n';require(bool(out),"Cannot write result graph");
}
} // namespace
int main(int argc,char** argv){try{
  require(argc==4,"Usage: compare_native_global_update map.bin fixture.txt output_prefix");
  const auto fixture=readFixture(argv[2]);auto started=std::chrono::steady_clock::now();
  const auto sparse=readMap(argv[1],fixture);const auto load_seconds=elapsed(started);
  const auto map=std::make_shared<BoundedMap>(sparse,GlobalBound::axisAligned(fixture.minimum,fixture.maximum));
  GlobalMemory disabled(fixture.parameters),enabled(fixture.parameters);
  rebuild(disabled,fixture);rebuild(enabled,fixture);
  require(resultGraph(disabled)==resultGraph(enabled),"Rebuilt input graphs differ");
  started=std::chrono::steady_clock::now();disabled.updateMap(map,TerrainQueryReuse::Disabled);
  const auto disabled_seconds=elapsed(started);
  started=std::chrono::steady_clock::now();enabled.updateMap(map,TerrainQueryReuse::ImmutableSnapshot);
  const auto enabled_seconds=elapsed(started);
  const auto before_result=resultGraph(disabled),after_result=resultGraph(enabled);
  const bool same=before_result==after_result;
  const std::string prefix(argv[3]);writeResult(prefix+"_disabled_graph.json",before_result);
  writeResult(prefix+"_enabled_graph.json",after_result);
  std::cout<<std::setprecision(17)<<"{\"status\":\""<<(same?"passed":"failed")
    <<"\",\"scope\":\"one updateMap per mode on the same reconstructed public graph; not exact live replay\""
    <<",\"exact_live_replay\":false,\"all_reconstructed_edges_use_ordinary_projection\":true"
    <<",\"input_cost_precision\":\"captured Graph.msg float32\",\"result_cost_precision\":\"internal double\""
    <<",\"visited_history_restored\":false,\"map_rows\":"<<fixture.cells<<",\"stored_cells\":"<<sparse->size()
    <<",\"input_vertices\":"<<fixture.vertices.size()<<",\"input_edges\":"<<fixture.edges.size()
    <<",\"map_load_seconds\":"<<load_seconds<<",\"disabled_seconds\":"<<disabled_seconds
    <<",\"enabled_seconds\":"<<enabled_seconds<<",\"speedup\":"<<disabled_seconds/enabled_seconds
    <<",\"complete_reconstructed_graph_equal\":"<<(same?"true":"false")
    <<",\"disabled_result_file\":"<<std::quoted(prefix+"_disabled_graph.json")
    <<",\"enabled_result_file\":"<<std::quoted(prefix+"_enabled_graph.json")<<"}\n";
  return same?0:2;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
