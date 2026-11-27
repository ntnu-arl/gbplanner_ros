// ROS 2 exploration snapshots for Rrg::saveGraph/loadGraph (rrg.h:215-225).
// Graph CDR follows gbplanner_graph; metadata preserves GlobalMemory state
// omitted by planner_msgs/Graph. The ROS 1 source remains unchanged.
#include "gbplanner_local/global_memory.hpp"
#include "gbplanner_local/ros1_graph_reader.hpp"
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <unistd.h>

namespace gbplanner2 {
namespace {
constexpr std::size_t kFileLimit=64*1024*1024;
constexpr std::size_t kVisitedLimit=1000000;
constexpr char kMagic[]="GBP2MEM1";
constexpr char kVisitedMarkerTag[]="E1M1";
constexpr char kFixedGroundEdgesTag[]="FGE1";

// Metadata integers and IEEE-754 doubles use little endian. Graph bytes are
// ordinary ROS 2 CDR with their own encapsulation header. No native structs or
// pointers are written, so snapshots do not depend on padding or pointer size.
struct Writer {
  std::vector<std::uint8_t> data;
  void bytes(const void* value,std::size_t size){
    if(size>kFileLimit-data.size())throw std::runtime_error("Snapshot exceeds 64 MiB");
    if(!size)return;
    const auto* first=static_cast<const std::uint8_t*>(value);
    data.insert(data.end(),first,first+size);
  }
  void integer(std::uint64_t value,int size){
    for(int i=0;i<size;++i){const std::uint8_t byte=value&255u;bytes(&byte,1);value>>=8;}
  }
  void count(std::size_t value){
    if(value>std::numeric_limits<std::uint32_t>::max())throw std::runtime_error("Snapshot count exceeds format limit");
    integer(value,4);
  }
  void signedInt(int value){integer(static_cast<std::uint32_t>(value),4);}
  void real(double value){
    if(!std::isfinite(value))throw std::runtime_error("Non-finite snapshot value");
    static_assert(sizeof(double)==8&&std::numeric_limits<double>::is_iec559);
    std::uint64_t bits;std::memcpy(&bits,&value,8);integer(bits,8);
  }
  void state(const StateVec& value){for(int i=0;i<4;++i)real(value[i]);}
  void string(const std::string& value){count(value.size());bytes(value.data(),value.size());}
};
struct Reader {
  const std::vector<std::uint8_t>& data;std::size_t position{0};
  const std::uint8_t* bytes(std::size_t size){
    if(size>data.size()-position)throw std::runtime_error("Truncated exploration snapshot");
    const auto* value=data.data()+position;position+=size;return value;
  }
  std::uint64_t integer(int size){
    const auto* value=bytes(size);std::uint64_t result=0;
    for(int i=0;i<size;++i)result|=std::uint64_t(value[i])<<(8*i);
    return result;
  }
  std::size_t count(std::size_t limit){
    const auto value=integer(4);
    if(value>limit)throw std::runtime_error("Snapshot count exceeds planner limit");
    return value;
  }
  int signedInt(){
    const auto value=static_cast<std::uint32_t>(integer(4));std::int32_t result;
    std::memcpy(&result,&value,4);return result;
  }
  bool flag(){const auto value=integer(1);if(value>1)throw std::runtime_error("Invalid snapshot flag");return value;}
  double real(){
    const auto bits=integer(8);double result;std::memcpy(&result,&bits,8);
    if(!std::isfinite(result))throw std::runtime_error("Non-finite snapshot value");
    return result;
  }
  StateVec state(){StateVec value;for(int i=0;i<4;++i)value[i]=real();return value;}
  std::string string(){const auto size=count(1024);const auto* value=bytes(size);return {reinterpret_cast<const char*>(value),size};}
};
void validateFrame(const std::string& frame){
  if(frame.empty()||frame.size()>1024||frame.find('\0')!=std::string::npos)
    throw std::runtime_error("A valid world frame is required");
}
std::vector<std::uint8_t> readFile(const std::string& path){
  if(path.empty()||path.find('\0')!=std::string::npos)throw std::runtime_error("Invalid snapshot path");
  std::ifstream input(path,std::ios::binary|std::ios::ate);
  if(!input)throw std::runtime_error("Cannot open exploration snapshot");
  const auto length=input.tellg();
  if(length<12||length>static_cast<std::streamoff>(kFileLimit))throw std::runtime_error("Invalid exploration snapshot size");
  std::vector<std::uint8_t> data(static_cast<std::size_t>(length));input.seekg(0);
  input.read(reinterpret_cast<char*>(data.data()),data.size());
  if(!input)throw std::runtime_error("Cannot read exploration snapshot");
  return data;
}
void writeAtomic(const std::string& path,const std::vector<std::uint8_t>& data){
  if(path.empty()||path.find('\0')!=std::string::npos)throw std::runtime_error("Invalid snapshot path");
  std::string pattern=path+".tmp.XXXXXX";
  std::vector<char> temporary(pattern.begin(),pattern.end());temporary.push_back('\0');
  const int descriptor=mkstemp(temporary.data());
  if(descriptor<0)throw std::runtime_error("Cannot create exploration snapshot beside destination");
  bool open=true;
  try{
    std::size_t written=0;
    while(written<data.size()){
      const auto count=::write(descriptor,data.data()+written,data.size()-written);
      if(count<0&&errno==EINTR)continue;
      if(count<=0)throw std::runtime_error("Cannot write exploration snapshot");
      written+=static_cast<std::size_t>(count);
    }
    if(fsync(descriptor)!=0)throw std::runtime_error("Cannot flush exploration snapshot");
    const int closed=::close(descriptor);open=false;
    if(closed!=0)throw std::runtime_error("Cannot close exploration snapshot");
    if(std::rename(temporary.data(),path.c_str())!=0)throw std::runtime_error("Cannot replace exploration snapshot");
  }catch(...){if(open)::close(descriptor);std::remove(temporary.data());throw;}
}
}

GlobalPersistenceResult GlobalMemory::saveGraph(const std::string& path,const std::string& frame)const{
  GlobalPersistenceResult result;
  try{
    validateFrame(frame);
    if(visited_.size()>kVisitedLimit)throw std::runtime_error("Visited history exceeds snapshot limit");
    if(graph_->getNumVertices()>kMaxVertices||graph_->getNumEdges()+suspended_edges_.size()>kMaxEdges)
      throw std::runtime_error("Retained graph exceeds snapshot planner limit");
    planner_msgs::msg::Graph message;graph_->convertGraphToMsg(message);message.header.frame_id=frame;
    rclcpp::Serialization<planner_msgs::msg::Graph> serializer;
    rclcpp::SerializedMessage cdr;serializer.serialize_message(&message,&cdr);
    const auto& raw=cdr.get_rcl_serialized_message();
    Writer output;output.bytes(kMagic,8);output.string(frame);
    output.integer(p_.ground_robot,1);output.real(p_.groundParameters().max_ground_height);
    output.count(raw.buffer_length);output.bytes(raw.buffer,raw.buffer_length);
    output.count(message.vertices.size());
    // Store exact yaw and non-message properties. Sensor voxel-key lists are
    // map caches and are deliberately rebuilt rather than replayed on a map.
    for(const auto& item:message.vertices){
      const auto* vertex=graph_->getVertex(item.id);
      output.signedInt(vertex->id);output.state(vertex->state);
      output.signedInt(static_cast<int>(vertex->type));
      output.signedInt(vertex->parent?vertex->parent->id:-1);output.real(vertex->distance);
      output.real(vertex->vol_gain.gain);output.real(vertex->vol_gain.accumulative_gain);
      output.signedInt(vertex->vol_gain.num_unknown_surf_voxels);
      output.integer(vertex->is_leaf_vertex,1);output.integer(vertex->is_hanging,1);
      output.signedInt(vertex->cluster_id);output.real(vertex->dm);
      output.signedInt(vertex->semantic_class.value);
    }
    // Graph.msg has float32 weights; keep their exact doubles for restoration.
    output.count(message.edges.size());
    for(const auto& edge:message.edges){
      const auto& neighbors=graph_->edge_map_.at(edge.source_id);
      const auto found=std::find_if(neighbors.begin(),neighbors.end(),[&](const auto& value){return value.first==edge.target_id;});
      if(found==neighbors.end())throw std::runtime_error("Inconsistent exploration graph");
      output.real(found->second);
    }
    output.count(visited_.size());for(const auto& state:visited_)output.state(state);
    output.signedInt(last_odometry_?last_odometry_->id:-1);
    output.count(suspended_edges_.size());
    for(const auto& edge:suspended_edges_){output.signedInt(edge.first.first);output.signedInt(edge.first.second);output.real(edge.second);}
    output.count(suspended_frontiers_.size());for(int id:suspended_frontiers_)output.signedInt(id);
    // Optional extension: old GBP2MEM1 snapshots ended before this tag. The
    // exact E1 marker in old files cannot be recovered from their independent
    // 0.5 m odometry history. Preserve the field for all snapshot generations.
    output.bytes(kVisitedMarkerTag,4);output.state(last_visited_marker_);
    // Reference geometry must retain its meaning after a restart and after
    // map revalidation; old snapshots have no such explicitly marked links.
    if(!fixed_ground_edges_.empty()){
      output.bytes(kFixedGroundEdgesTag,4);output.count(fixed_ground_edges_.size());
      for(const auto& edge:fixed_ground_edges_){output.signedInt(edge.first);output.signedInt(edge.second);}
    }
    result.success=true;result.reason="saved";result.vertices=message.vertices.size();
    result.active_edges=message.edges.size();result.suspended_edges=suspended_edges_.size();result.frontiers=frontierCount();
    writeAtomic(path,output.data);
  }catch(const std::exception& error){result.success=false;result.reason=error.what();}
  return result;
}

GlobalPersistenceResult GlobalMemory::loadGraph(const std::string& path,const std::string& expected_frame,
                                               const GainFunction& gain){
  GlobalPersistenceResult result;
  try{
    validateFrame(expected_frame);const auto data=readFile(path);Reader input{data};
    if(std::memcmp(input.bytes(8),kMagic,8))throw std::runtime_error("Expected GBP2MEM1 exploration snapshot (not ROS 1 or GBP2CDR1)");
    if(input.string()!=expected_frame)throw std::runtime_error("Snapshot world frame does not match");
    const bool ground=input.flag();const double height=input.real();
    if(ground!=p_.ground_robot||(ground&&std::abs(height-p_.groundParameters().max_ground_height)>1e-9))
      throw std::runtime_error("Snapshot robot mode or planning height does not match");
    const auto cdr_size=input.count(kFileLimit);const auto* cdr_data=input.bytes(cdr_size);
    rclcpp::SerializedMessage cdr(cdr_size);auto& raw=cdr.get_rcl_serialized_message();
    raw.buffer_length=cdr_size;std::memcpy(raw.buffer,cdr_data,cdr_size);
    planner_msgs::msg::Graph message;rclcpp::Serialization<planner_msgs::msg::Graph> serializer;
    serializer.deserialize_message(&cdr,&message);
    if(message.header.frame_id!=expected_frame||message.vertices.size()>kMaxVertices||message.edges.size()>kMaxEdges)
      throw std::runtime_error("Invalid snapshot graph header or size");
    // GlobalMemory uses dense nonnegative IDs and reserves zero as home.
    for(std::size_t i=0;i<message.vertices.size();++i){
      const auto& vertex=message.vertices[i];
      if(vertex.id!=static_cast<int>(i)||vertex.num_unknown_voxels<0||vertex.num_free_voxels<0||vertex.num_occupied_voxels<0)
        throw std::runtime_error("Invalid snapshot vertex IDs or gain counts");
    }
    auto edges=std::move(message.edges);message.edges.clear();
    GlobalMemory next(p_);next.graph_->convertMsgToGraph(message);
    if(input.count(kMaxVertices)!=message.vertices.size())throw std::runtime_error("Snapshot vertex metadata mismatch");
    for(std::size_t i=0;i<message.vertices.size();++i){
      const int id=input.signedInt();const auto state=input.state();const int type=input.signedInt(),parent=input.signedInt();
      auto* vertex=next.graph_->getVertex(id);
      if(id!=static_cast<int>(i)||type<0||type>2||parent<-1||parent>=id||
         (state.head<3>()-vertex->state.head<3>()).norm()>1e-9)
        throw std::runtime_error("Invalid snapshot vertex metadata");
      // XYZ matches the already built kd-tree; restore exact yaw separately.
      vertex->state[3]=state[3];vertex->type=static_cast<VertexType>(type);
      vertex->parent=parent<0?nullptr:next.graph_->getVertex(parent);
      if(vertex->parent)vertex->parent->children.push_back(vertex);
      vertex->distance=input.real();vertex->vol_gain.gain=input.real();vertex->vol_gain.accumulative_gain=input.real();
      vertex->vol_gain.num_unknown_surf_voxels=input.signedInt();
      vertex->is_leaf_vertex=input.flag();vertex->is_hanging=input.flag();
      vertex->cluster_id=input.signedInt();vertex->dm=input.real();vertex->semantic_class.value=input.signedInt();
      if(vertex->distance<0||vertex->vol_gain.num_unknown_surf_voxels<0)
        throw std::runtime_error("Invalid snapshot vertex distance or surface count");
    }
    if(input.count(kMaxEdges)!=edges.size())throw std::runtime_error("Snapshot edge metadata mismatch");
    std::set<std::pair<int,int>> active;
    for(const auto& edge:edges){
      const auto key=std::minmax(edge.source_id,edge.target_id);const double weight=input.real();
      if(key.first==key.second||weight<0||!active.emplace(key.first,key.second).second)
        throw std::runtime_error("Invalid or duplicate snapshot edge");
      next.graph_->addEdge(next.graph_->getVertex(edge.source_id),next.graph_->getVertex(edge.target_id),weight);
    }
    const auto visited=input.count(kVisitedLimit);next.visited_.reserve(visited);
    for(std::size_t i=0;i<visited;++i)next.visited_.push_back(input.state());
    const int odometry=input.signedInt();next.last_odometry_=next.graph_->getVertex(odometry);
    if(odometry<-1||(odometry>=0&&!next.last_odometry_)||(message.vertices.empty()&&visited))
      throw std::runtime_error("Invalid snapshot odometry history");
    const auto suspended=input.count(kMaxEdges);
    for(std::size_t i=0;i<suspended;++i){
      const int from=input.signedInt(),to=input.signedInt();const double weight=input.real();
      const auto key=std::make_pair(from,to);
      if(from>=to||weight<0||!next.graph_->getVertex(from)||!next.graph_->getVertex(to)||active.count(key)||
         !next.suspended_edges_.emplace(key,weight).second)
        throw std::runtime_error("Invalid suspended snapshot edge");
    }
    const auto frontiers=input.count(kMaxVertices);
    for(std::size_t i=0;i<frontiers;++i){
      const int id=input.signedInt();
      if(!next.graph_->getVertex(id)||!next.suspended_frontiers_.insert(id).second)
        throw std::runtime_error("Invalid suspended snapshot frontier");
    }
    if(input.position<data.size()){
      if(std::memcmp(input.bytes(4),kVisitedMarkerTag,4))
        throw std::runtime_error("Unexpected exploration snapshot extension");
      next.last_visited_marker_=input.state();
    }else{
      // Legacy files have no E1 state. Reconstruct an approximate threshold
      // marker from their ordered 0.5 m visit samples, starting at the origin.
      // Do not replay retirement on the loaded graph: its stored types stand.
      for(const auto& state:next.visited_)
        if((state-next.last_visited_marker_).head<3>().norm()>=1.0)
          next.last_visited_marker_=state;
    }
    if(input.position<data.size()){
      if(std::memcmp(input.bytes(4),kFixedGroundEdgesTag,4))
        throw std::runtime_error("Unexpected reference-geometry snapshot extension");
      const auto fixed=input.count(kMaxEdges);
      if(fixed&&!ground)throw std::runtime_error("Ground reference links in an aerial snapshot");
      for(std::size_t i=0;i<fixed;++i){
        const int from=input.signedInt(),to=input.signedInt();const auto key=std::make_pair(from,to);
        if(from>=to||!next.graph_->getVertex(from)||!next.graph_->getVertex(to)||
           (!active.count(key)&&!next.suspended_edges_.count(key))||
           !next.fixed_ground_edges_.insert(key).second)
          throw std::runtime_error("Invalid fixed reference snapshot edge");
      }
    }
    if(input.position!=data.size())throw std::runtime_error("Unexpected exploration snapshot data");
    // Do not prune blocked historical links: updateMap moves them to the
    // suspended store and can restore them when a subsequent map clears them.
    // Reserve room for all existing links before this transition.
    if(next.suspended_edges_.size()+active.size()>kMaxEdges)
      throw std::runtime_error("Snapshot retained links exceed planner limit");
    next.updateMap(map_);
    if(gain)next.refresh(gain);
    else for(auto& item:next.graph_->vertices_map_){
      auto* vertex=item.second;
      if(vertex->type==VertexType::kVisited){next.suspended_frontiers_.erase(vertex->id);continue;}
      if(vertex->type==VertexType::kFrontier||next.suspended_frontiers_.count(vertex->id)){
        next.suspended_frontiers_.insert(vertex->id);vertex->type=VertexType::kUnvisited;vertex->vol_gain.reset();
      }
    }
    // No existing graph, history or current map is mutated before this point.
    // The sampler restarts with the configured seed; graph/visited history is
    // the persistent exploration state, not a promise of random replay.
    result.success=true;result.map_revalidated=static_cast<bool>(map_);result.reason=map_?"loaded":"loaded_awaiting_map";
    result.vertices=next.graph_->getNumVertices();result.active_edges=next.graph_->getNumEdges();
    result.suspended_edges=next.suspended_edges_.size();result.frontiers=next.frontierCount();
    graph_.swap(next.graph_);visited_.swap(next.visited_);std::swap(last_odometry_,next.last_odometry_);
    last_visited_marker_.swap(next.last_visited_marker_);
    suspended_edges_.swap(next.suspended_edges_);suspended_frontiers_.swap(next.suspended_frontiers_);
    terrain_.swap(next.terrain_);ground_edges_.swap(next.ground_edges_);
    fixed_ground_edges_.swap(next.fixed_ground_edges_);
    background_sampler_.swap(next.background_sampler_);std::swap(cluster_random_,next.cluster_random_);
    frontier_update_={};map_edge_update_=next.map_edge_update_;
    restored_history_size_=visited_.size();++visit_revision_;++snapshot_revision_;
  }catch(const std::exception& error){result.success=false;result.reason=error.what();}
  return result;
}
GlobalPersistenceResult GlobalMemory::importRos1Graph(const std::string& path,
    const std::string& expected_frame,const GainFunction& gain){
  GlobalPersistenceResult result;
  try{
    const auto source=readRos1Graph(path,expected_frame);
    GlobalMemory next(p_);
    next.graph_->convertMsgToGraph(source.graph);
    // ROS 1 only persisted topology, poses, voxel counts and frontier flags.
    // Parent chains, visit history, semantic labels and fixed reference-edge
    // provenance cannot be reconstructed. Default metadata stays explicitly
    // empty; in particular importing does not claim that the robot visited home.
    for(auto& item:next.graph_->vertices_map_){
      auto* vertex=item.second;
      if(vertex->type==VertexType::kFrontier)next.suspended_frontiers_.insert(vertex->id);
      vertex->type=VertexType::kUnvisited;
      vertex->vol_gain.is_frontier=false;
      vertex->vol_gain.gain=0.;vertex->vol_gain.accumulative_gain=0.;
    }
    // A mapless conversion stores all original links as pending. Normal
    // loadGraph subsequently rechecks them against the current map, including
    // ground support and directed terrain costs, before any route can use them.
    next.updateMap(map_);
    if(gain)next.refresh(gain);
    result.success=true;result.map_revalidated=static_cast<bool>(map_);
    result.reason=map_?"imported_ros1":"imported_ros1_awaiting_map";
    result.vertices=next.graph_->getNumVertices();result.active_edges=next.graph_->getNumEdges();
    result.suspended_edges=next.suspended_edges_.size();result.frontiers=next.frontierCount();
    graph_.swap(next.graph_);visited_.swap(next.visited_);std::swap(last_odometry_,next.last_odometry_);
    last_visited_marker_.swap(next.last_visited_marker_);
    suspended_edges_.swap(next.suspended_edges_);suspended_frontiers_.swap(next.suspended_frontiers_);
    terrain_.swap(next.terrain_);ground_edges_.swap(next.ground_edges_);
    fixed_ground_edges_.swap(next.fixed_ground_edges_);
    background_sampler_.swap(next.background_sampler_);std::swap(cluster_random_,next.cluster_random_);
    frontier_update_={};map_edge_update_=next.map_edge_update_;
    restored_history_size_=0;++visit_revision_;++snapshot_revision_;
  }catch(const std::exception& error){result.success=false;result.reason=error.what();}
  return result;
}

void GlobalMemory::setPlanningParameters(Parameters parameters){
  parameters.validate();
  if(graph_->getNumVertices()&&
     (parameters.ground_robot!=p_.ground_robot||
      (p_.ground_robot&&std::abs(parameters.groundParameters().max_ground_height-
                                 p_.groundParameters().max_ground_height)>1e-9)))
    throw std::invalid_argument("Cannot change stored graph planning-height convention");
  auto sampler=std::make_unique<LocalSampler>(parameters);
  p_=std::move(parameters);background_sampler_=std::move(sampler);
  updateMap(map_);
}
}
