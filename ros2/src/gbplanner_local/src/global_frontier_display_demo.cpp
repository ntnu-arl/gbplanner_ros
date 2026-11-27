// Public frontier snapshots follow candidate lifecycle without rewriting gain.
#include "gbplanner_local/global_memory.hpp"
#include <cstdio>
#include <iostream>
#include <stdexcept>
#include <unistd.h>

using namespace gbplanner2;
namespace {
void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
StateVec pose(double x){return StateVec(x,0.,0.,0.);}
VolumetricGain gain(const StateVec&){
  VolumetricGain result;result.is_frontier=true;result.gain=100.;result.num_unknown_voxels=100;
  return result;
}
std::shared_ptr<VoxelMap> freeMap(){
  auto map=std::make_shared<VoxelMap>(.25,Eigen::Vector3d(-2.,-2.,-2.),Eigen::Vector3i(32,16,16));
  map->fill(VoxelStatus::Free);return map;
}
int seed(GlobalMemory& memory,const std::shared_ptr<VoxelMap>& map){
  memory.updateMap(map);require(memory.recordOdometry(pose(0.)),"Could not establish home");
  const int target=memory.addSemanticPath({pose(0.),pose(4.)},planner_semantic_msgs::msg::SemanticClass::K_DOOR);
  require(target>0,"Could not retain semantic frontier");return target;
}
planner_msgs::msg::Graph snapshot(GlobalMemory& memory){
  planner_msgs::msg::Graph message;memory.graphMessage(message);
  int frontiers=0;for(const auto& vertex:message.vertices)frontiers+=vertex.is_frontier;
  require(frontiers==memory.frontierCount(),"Public frontier flags disagree with selectable vertex types");
  return message;
}
struct Files {
  std::string active="/tmp/gbplanner_frontier_display_"+std::to_string(getpid())+"_active.bin";
  std::string retired=active+".retired";
  ~Files(){std::remove(active.c_str());std::remove(retired.c_str());}
};
}

int main(){
  try{
    Parameters parameters;auto map=freeMap();GlobalMemory memory(parameters);
    const int target=seed(memory,map);
    const auto* vertex=memory.graph().vertices_map_.at(target);
    // Semantic insertion changes type before a sensor gain is available.
    require(!vertex->vol_gain.is_frontier&&snapshot(memory).vertices.at(target).is_frontier,
            "Public graph ignored an active semantic candidate with unevaluated gain");
    require(memory.selectFrontier(pose(0.),gain).success,"Could not refresh and route to frontier");
    Files files;require(memory.saveGraph(files.active,"map").success,"Could not save active frontier");

    // .999 m creates the final odometry vertex; the subsequent 1 m E1 event
    // changes types only. A snapshot revision must advance without new geometry.
    require(memory.recordOdometry(pose(.999)),"Subthreshold odometry failed");
    const auto before=snapshot(memory);const auto revision_before=memory.snapshotRevision();
    require(memory.recordOdometry(pose(1.)),"Retirement odometry failed");
    const auto retired=snapshot(memory);const auto revision_retired=memory.snapshotRevision();
    require(revision_retired>revision_before&&!retired.vertices.at(target).is_frontier&&
            vertex->type==VertexType::kVisited&&vertex->vol_gain.is_frontier&&
            vertex->vol_gain.num_unknown_voxels==100,
            "Frontier retirement failed to refresh display while retaining historical gain");
    auto comparable=retired;
    require(comparable.vertices.size()==before.vertices.size(),"Type-only event changed vertex count");
    for(std::size_t i=0;i<comparable.vertices.size();++i)
      comparable.vertices[i].is_frontier=before.vertices[i].is_frontier;
    require(comparable==before,"Display qualification changed geometry, voxel counts or edges");
    require(memory.recordOdometry(pose(1.))&&memory.snapshotRevision()==revision_retired,
            "Repeated unchanged odometry invalidated the public graph cache");

    // Saving uses GraphManager's original gain fields, not the display flags.
    require(memory.saveGraph(files.retired,"map").success,"Could not save retired frontier");
    GlobalMemory restored(parameters);restored.updateMap(map);
    require(restored.loadGraph(files.retired,"map",gain).success,"Retired snapshot failed to load");
    const auto* retained=restored.graph().vertices_map_.at(target);
    require(retained->type==VertexType::kVisited&&retained->vol_gain.is_frontier&&
            retained->vol_gain.num_unknown_voxels==100&&
            !snapshot(restored).vertices.at(target).is_frontier,
            "Display flags leaked into persisted historical gain or revived a visit");

    // Deferred restoration first hides unvalidated candidates, then publishes
    // a new eligible frontier after the ordinary current-map gain refresh.
    GlobalMemory deferred(parameters);deferred.updateMap(map);
    require(deferred.loadGraph(files.active,"map").success,"Deferred active snapshot failed to load");
    require(!snapshot(deferred).vertices.at(target).is_frontier,"Unvalidated loaded frontier was displayed as active");
    const auto deferred_revision=deferred.snapshotRevision();
    require(deferred.selectFrontier(pose(0.),gain).success&&
            deferred.snapshotRevision()>deferred_revision&&snapshot(deferred).vertices.at(target).is_frontier,
            "Loaded candidate refresh did not invalidate the display snapshot");

    // These transitions change type while raw gain/counts stay zero. They
    // must not disappear behind the existing gain-only revision comparison.
    GlobalMemory semantic(parameters);const int semantic_id=seed(semantic,map);
    auto revision=semantic.snapshotRevision();
    semantic.selectFrontier(pose(0.),[](const StateVec&){return VolumetricGain{};});
    require(semantic.snapshotRevision()>revision&&!snapshot(semantic).vertices.at(semantic_id).is_frontier,
            "Zero-gain semantic retirement did not invalidate the public snapshot");
    revision=semantic.snapshotRevision();
    require(semantic.addSemanticPath({pose(4.)},planner_semantic_msgs::msg::SemanticClass::K_DOOR)==semantic_id&&
            semantic.snapshotRevision()>revision&&snapshot(semantic).vertices.at(semantic_id).is_frontier,
            "Repeated semantic class failed to publish its newly active frontier type");

    std::cout<<"{\"status\":\"passed\",\"active_before_visit\":1,\"active_after_visit\":0"
      <<",\"historical_unknown_voxels\":"<<retained->vol_gain.num_unknown_voxels
      <<",\"visit_revision_delta\":"<<revision_retired-revision_before
      <<",\"geometry_and_counts_unchanged\":true,\"persistence_preserves_gain\":true"
      <<",\"loaded_frontier_refresh\":true,\"semantic_type_only_transitions\":true}\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
