// Native E1 lifecycle: real odometry retires frontiers while graph routes survive.
#include "gbplanner_local/global_memory.hpp"
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <unistd.h>

using namespace gbplanner2;
namespace {
void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
StateVec pose(double x,double y=0.){return StateVec(x,y,0.,0.);}
VolumetricGain gain(const StateVec& state){
  VolumetricGain result;result.is_frontier=true;
  result.gain=state.x()>3.?1000.:10.;result.num_unknown_voxels=100;
  return result;
}
std::shared_ptr<VoxelMap> freeMap(){
  auto map=std::make_shared<VoxelMap>(.25,Eigen::Vector3d(-8,-8,-2),Eigen::Vector3i(64,64,16));
  map->fill(VoxelStatus::Free);return map;
}
planner_msgs::msg::Graph branches(){
  GraphManager graph;graph.addVertex(new Vertex(0,pose(0.)));
  for(int branch=0;branch<2;++branch)for(int i=1;i<=4;++i){
    const int id=branch*4+i,parent=i==1?0:id-1;
    auto* vertex=new Vertex(id,branch?pose(0.,i):pose(i));
    if(i==4)vertex->vol_gain=gain(vertex->state);
    graph.addVertex(vertex);graph.addEdge(graph.getVertex(parent),vertex,1.);
  }
  planner_msgs::msg::Graph result;graph.convertGraphToMsg(result);return result;
}
int at(const GlobalMemory& memory,const StateVec& state){
  for(const auto& item:memory.graph().vertices_map_)
    if((item.second->state-state).head<3>().norm()<1e-9)return item.first;
  throw std::runtime_error("Expected global vertex is missing");
}
VertexType type(const GlobalMemory& memory,int id){return memory.graph().vertices_map_.at(id)->type;}
void seed(GlobalMemory& memory,const std::shared_ptr<VoxelMap>& map){
  memory.updateMap(map);require(memory.recordOdometry(pose(0.)),"Could not record home");
  memory.absorbLocal(branches(),gain);require(memory.frontierCount()==2,"Could not seed separated frontiers");
}
void requireVisited(const GlobalMemory& memory,int id){
  require(type(memory,id)==VertexType::kVisited,"Retired frontier was reactivated");
}
struct SnapshotFiles {
  std::string current="/tmp/gbplanner_e1_"+std::to_string(getpid())+".bin";
  std::string legacy=current+".legacy";
  std::string legacy_exact=current+".legacy_exact",ros1=current+".ros1";
  ~SnapshotFiles(){for(const auto& path:{current,legacy,legacy_exact,ros1})std::remove(path.c_str());}
};
// An independent pre-migration fixture. Do not derive old 0.5 m history by
// stripping an extension from the current writer: current history is 1 m E1.
struct FixtureBytes {
  std::vector<std::uint8_t> data;
  void bytes(const void* value,std::size_t size){
    const auto* first=static_cast<const std::uint8_t*>(value);data.insert(data.end(),first,first+size);
  }
  void integer(std::uint64_t value,int size=4){for(int i=0;i<size;++i){data.push_back(value&255u);value>>=8;}}
  void signedInt(int value){integer(static_cast<std::uint32_t>(value));}
  void real(double value){std::uint64_t bits;std::memcpy(&bits,&value,8);integer(bits,8);}
  void state(const StateVec& value){for(int i=0;i<4;++i)real(value[i]);}
  void string(const std::string& value){integer(value.size());bytes(value.data(),value.size());}
  void save(const std::string& path){
    std::ofstream output(path,std::ios::binary);output.write(reinterpret_cast<const char*>(data.data()),data.size());
    require(bool(output),"Could not write independent history fixture");
  }
};
void writeLegacySnapshot(const std::string& path,const Parameters& p,bool exact_marker){
  GraphManager graph;
  for(int id=0;id<7;++id){
    auto* vertex=new Vertex(id,pose(id==6?1.75:double(id)));vertex->vol_gain=gain(vertex->state);
    vertex->type=id==5?VertexType::kFrontier:id==6?VertexType::kUnvisited:VertexType::kVisited;
    vertex->distance=vertex->state.x();
    if(id)vertex->parent=graph.getVertex(id==6?1:id-1);
    graph.addVertex(vertex);
    if(id)graph.addEdge(vertex,vertex->parent,id==6?.75:1.);
  }
  planner_msgs::msg::Graph message;graph.convertGraphToMsg(message);message.header.frame_id="map";
  rclcpp::Serialization<planner_msgs::msg::Graph> serializer;rclcpp::SerializedMessage cdr;
  serializer.serialize_message(&message,&cdr);const auto& raw=cdr.get_rcl_serialized_message();
  FixtureBytes output;output.bytes("GBP2MEM1",8);output.string("map");
  output.integer(0,1);output.real(p.groundParameters().max_ground_height);
  output.integer(raw.buffer_length);output.bytes(raw.buffer,raw.buffer_length);
  output.integer(message.vertices.size());
  for(const auto& item:message.vertices){
    const auto* v=graph.getVertex(item.id);output.signedInt(v->id);output.state(v->state);
    output.signedInt(static_cast<int>(v->type));output.signedInt(v->parent?v->parent->id:-1);output.real(v->distance);
    output.real(v->vol_gain.gain);output.real(v->vol_gain.accumulative_gain);
    output.signedInt(v->vol_gain.num_unknown_surf_voxels);output.integer(v->is_leaf_vertex,1);output.integer(v->is_hanging,1);
    output.signedInt(v->cluster_id);output.real(v->dm);output.signedInt(v->semantic_class.value);
  }
  output.integer(message.edges.size());
  for(const auto& edge:message.edges){
    const auto& neighbors=graph.edge_map_.at(edge.source_id);
    const auto found=std::find_if(neighbors.begin(),neighbors.end(),[&](const auto& item){return item.first==edge.target_id;});
    require(found!=neighbors.end(),"Legacy fixture edge weight is missing");output.real(found->second);
  }
  output.integer(3);for(const auto& state:{pose(0.),pose(.999),pose(1.75)})output.state(state);
  output.signedInt(6);output.integer(0);output.integer(0); // last odometry; no suspended entries
  if(exact_marker){output.bytes("E1M1",4);output.state(pose(1.));}
  output.save(path);
}
void writeRos1Home(const std::string& path){
  FixtureBytes output;output.integer(0);output.integer(0);output.integer(0);output.string("map");
  output.integer(1);output.signedInt(0);
  for(int i=0;i<6;++i)output.real(0.);
  output.real(1.); // XYZ and unit quaternion
  output.signedInt(0);output.signedInt(0);output.signedInt(0);output.integer(0,1);output.integer(0);
  output.save(path);
}
void requireLegacyPrefix(const GlobalMemory& memory){
  require(memory.restoredHistorySize()==3&&memory.visitedStates().size()>=3,
    "Restored history prefix was not retained");
  const std::array<StateVec,3> expected{pose(0.),pose(.999),pose(1.75)};
  for(std::size_t i=0;i<expected.size();++i)
    require((memory.visitedStates()[i]-expected[i]).norm()==0.,"Old history was resampled or replaced");
}
}

int main(){
  try{
    Parameters p;auto map=freeMap();GlobalMemory memory(p);seed(memory,map);
    require(memory.visitedStates().empty()&&memory.lastVisitedMarker().isZero(0.)&&memory.restoredHistorySize()==0,
      "Home creation inserted history or changed the initial marker");
    const auto initial_visit_revision=memory.visitRevision();
    const auto initial_vertices=memory.graph().vertices_map_.size();
    require(memory.recordOdometry(pose(.5))&&memory.graph().vertices_map_.size()>initial_vertices&&
      memory.visitedStates().empty()&&memory.visitRevision()==initial_visit_revision,
      "The 0.5 m odometry graph spacing was mixed with the 1 m history gate");
    const int near=at(memory,pose(4.)),far=at(memory,pose(0.,4.));
    const double retained_gain=memory.graph().vertices_map_.at(near)->vol_gain.gain;
    const int edges=memory.edgeCount();
    require(memory.recordOdometry(pose(.999)),"Subthreshold odometry failed");
    require(memory.frontierCount()==2,"E1 fired before 1 m displacement");
    require(memory.recordOdometry(pose(1.)),"Threshold odometry failed");
    require(memory.visitedStates().size()==1&&(memory.visitedStates().front()-pose(1.)).norm()==0.&&
      (memory.lastVisitedMarker()-pose(1.)).norm()==0.&&memory.visitRevision()==initial_visit_revision+1,
      "A 1 m event did not append history and update its independent revision");
    requireVisited(memory,near);
    require(type(memory,far)==VertexType::kFrontier,"E1 retired a vertex outside 3 m");
    require(memory.graph().vertices_map_.at(near)->vol_gain.gain==retained_gain,
            "E1 cleared gain instead of changing only the vertex type");
    require(memory.edgeCount()>=edges&&type(memory,0)==VertexType::kVisited,
            "E1 removed topology or failed to mark the home vertex");
    for(int i=0;i<3;++i){
      require(memory.recordOdometry(pose(1.)),"Repeated odometry failed");
      const auto selected=memory.selectFrontier(pose(1.),gain);
      require(selected.success&&selected.target==far,"High stale gain won after E1 retirement");
    }
    const auto home=memory.homing(pose(1.));
    // Homing assigns each waypoint its travel heading, so the final yaw need
    // not equal home's stored yaw. Check the route's identity and XYZ instead.
    require(home.success&&home.target==0&&home.path.size()>=2&&
            (home.path.front().head<3>()-pose(1.).head<3>()).norm()<1e-9&&
            (home.path.back().head<3>()-memory.graph().vertices_map_.at(0)->state.head<3>()).norm()<1e-9&&
            std::abs(home.distance-1.)<1e-9,"E1 broke the route home");
    for(std::size_t i=1;i<home.path.size();++i)
      require(map->path(home.path[i-1].head<3>(),home.path[i].head<3>(),p.robot_size)==VoxelStatus::Free,
              "E1 homing route contains a blocked segment");

    // A temporarily blocked frontier has a separate pending identity. E1 must
    // retire that identity too, so removing the obstacle cannot resurrect it.
    auto blocked_map=freeMap();GlobalMemory blocked(p);seed(blocked,blocked_map);
    const int retired=at(blocked,pose(4.));
    blocked_map->set(blocked_map->index({4.,0.,0.}),VoxelStatus::Occupied);
    blocked.updateMap(blocked_map);blocked.selectFrontier(pose(0.),gain);
    require(type(blocked,retired)==VertexType::kUnvisited,"Blocked frontier was not suspended");
    require(blocked.recordOdometry(pose(.999))&&blocked.recordOdometry(pose(1.)),"Blocked-case odometry failed");
    requireVisited(blocked,retired);
    blocked_map->fill(VoxelStatus::Free);blocked.updateMap(blocked_map);
    const auto restored_choice=blocked.selectFrontier(pose(1.),gain);
    require(restored_choice.success&&restored_choice.target!=retired,"Map recovery revived a visited frontier");
    requireVisited(blocked,retired);
    using Class=planner_semantic_msgs::msg::SemanticClass;
    require(blocked.addSemanticPath({pose(4.)},Class::K_DOOR)==retired,"Semantic annotation failed");
    requireVisited(blocked,retired);

    // New history and E1 share the 1 m gate. An inter-event odometry graph
    // point at 1.75 must not change either history or its last event marker.
    const int next=blocked.addSemanticPath({pose(0.),pose(5.)},Class::K_DOOR);
    require(next>=0&&type(blocked,next)==VertexType::kFrontier,"Next frontier was not retained");
    require(blocked.recordOdometry(pose(1.75)),"Inter-event odometry failed");
    require(blocked.visitedStates().size()==1&&(blocked.lastVisitedMarker()-pose(1.)).norm()==0.,
      "Inter-event odometry leaked into new history");
    SnapshotFiles files;require(blocked.saveGraph(files.current,"map").success,"E1 snapshot save failed");
    GlobalMemory restored(p);restored.updateMap(blocked_map);
    const auto before_load=restored.visitRevision();
    require(restored.loadGraph(files.current,"map",gain).success,"E1 snapshot load failed");
    require(restored.visitRevision()==before_load+1&&restored.restoredHistorySize()==1,
      "Loading a current snapshot lost the restored-prefix metadata");
    requireVisited(restored,retired);
    require(restored.recordOdometry(pose(1.75))&&restored.recordOdometry(pose(1.999)),"Restored odometry failed");
    require(type(restored,next)==VertexType::kFrontier,"Restored E1 marker fired early");
    require(restored.recordOdometry(pose(2.)),"Restored threshold odometry failed");
    require(restored.restoredHistorySize()==1&&restored.visitedStates().size()==2&&
      (restored.visitedStates().back()-pose(2.)).norm()==0.,"New events changed the restored-prefix length");
    requireVisited(restored,next);
    require(restored.homing(pose(2.)).success,"Restored visited graph cannot route home");

    GlobalMemory deferred(p);
    require(deferred.loadGraph(files.current,"map").success,"Deferred E1 snapshot load failed");
    requireVisited(deferred,retired);
    deferred.updateMap(blocked_map);deferred.selectFrontier(pose(1.75),gain);
    requireVisited(deferred,retired);
    require(type(deferred,next)==VertexType::kFrontier,"Unvisited frontier failed to resume after map arrival");

    writeLegacySnapshot(files.legacy,p,false);
    GlobalMemory legacy(p);legacy.updateMap(blocked_map);
    require(legacy.loadGraph(files.legacy,"map",gain).success,"Legacy snapshot became unreadable");
    requireLegacyPrefix(legacy);requireVisited(legacy,4);
    require((legacy.lastVisitedMarker()-pose(1.75)).norm()==0.,"Legacy marker inference changed");
    require(legacy.recordOdometry(pose(2.)),"Legacy odometry failed");
    require(type(legacy,5)==VertexType::kFrontier,"Legacy load did not reconstruct its 1.75 m marker");
    require(legacy.recordOdometry(pose(2.75)),"Legacy threshold odometry failed");
    requireVisited(legacy,5);requireLegacyPrefix(legacy);
    require(legacy.visitedStates().size()==4&&(legacy.visitedStates().back()-pose(2.75)).norm()==0.,
      "Legacy history did not retain its prefix and append the new 1 m event");
    writeLegacySnapshot(files.legacy_exact,p,true);
    GlobalMemory legacy_exact(p);legacy_exact.updateMap(blocked_map);
    require(legacy_exact.loadGraph(files.legacy_exact,"map",gain).success,"Old E1M1 snapshot became unreadable");
    requireLegacyPrefix(legacy_exact);
    require((legacy_exact.lastVisitedMarker()-pose(1.)).norm()==0.,"Explicit old marker was replaced by history");
    require(legacy_exact.recordOdometry(pose(1.75))&&legacy_exact.visitedStates().size()==3&&
      legacy_exact.recordOdometry(pose(2.))&&legacy_exact.visitedStates().size()==4,
      "Old exact marker no longer controls the new event threshold");
    requireLegacyPrefix(legacy_exact);

    const auto before_reset=restored.visitRevision();restored.reset();
    require(restored.visitRevision()==before_reset+1&&restored.visitedStates().empty()&&
      restored.lastVisitedMarker().isZero(0.)&&restored.restoredHistorySize()==0,"Reset retained visited state");
    GlobalMemory explicit_home(p);explicit_home.updateMap(map);const auto before_home=explicit_home.visitRevision();
    require(explicit_home.setHomingPosition(pose(2.))&&explicit_home.visitedStates().empty()&&
      explicit_home.lastVisitedMarker().isZero(0.)&&explicit_home.visitRevision()==before_home,
      "Explicit displaced home fabricated a visit or reset the origin marker");
    require(explicit_home.recordOdometry(pose(2.))&&explicit_home.visitedStates().size()==1,
      "An actual odometry event at a displaced home failed the origin-based gate");
    writeRos1Home(files.ros1);const auto before_import=legacy.visitRevision();
    require(legacy.importRos1Graph(files.ros1,"map").success&&legacy.visitRevision()==before_import+1&&
      legacy.visitedStates().empty()&&legacy.lastVisitedMarker().isZero(0.)&&legacy.restoredHistorySize()==0,
      "ROS 1 import fabricated history or retained replaced snapshot metadata");

    std::cout<<"{\"status\":\"passed\",\"event_spacing_m\":1,\"retirement_radius_m\":3"
      <<",\"threshold_and_radius\":true,\"stale_gain_excluded\":true,\"homing_preserved\":true"
      <<",\"suspended_frontier_retired\":true,\"semantic_annotation_preserves_visit\":true"
      <<",\"exact_marker_reloaded\":true,\"deferred_map_preserves_visit\":true"
      <<",\"legacy_snapshot_loaded\":true,\"history_event_spacing_m\":1,\"home_is_not_history\":true"
      <<",\"restored_prefix_preserved\":true,\"visit_revision_lifecycle\":true,\"reset_import_clear_history\":true}\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
