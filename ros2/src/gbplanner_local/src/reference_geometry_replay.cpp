// Recheck the recorded run2/map77 candidate; never sample a replacement path.
// Input extraction and source hashes: tools/check_reference_geometry.py.
#include "gbplanner_local/global_memory.hpp"
#include "gbplanner_local/global_bound.hpp"
#include "gbplanner_local/sparse_voxel_map.hpp"
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace gbplanner2;
namespace {
constexpr std::size_t kCells=84059,kStride=24;
constexpr double kResolution=.1;
void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
float floatLE(const uint8_t* p){
  static_assert(sizeof(float)==4&&std::numeric_limits<float>::is_iec559,"Requires IEEE float32");
  const uint32_t bits=uint32_t(p[0])|(uint32_t(p[1])<<8)|(uint32_t(p[2])<<16)|(uint32_t(p[3])<<24);
  float value;std::memcpy(&value,&bits,sizeof(value));return value;
}
std::shared_ptr<SparseVoxelMap> readMap(const std::string& filename){
  std::ifstream file(filename,std::ios::binary|std::ios::ate);
  require(bool(file)&&file.tellg()==std::streamoff(kCells*kStride),"Expected exact 84059-cell run2 snapshot");
  std::vector<uint8_t> bytes(kCells*kStride);file.seekg(0);
  require(bool(file.read(reinterpret_cast<char*>(bytes.data()),bytes.size())),"Map read failed");
  auto map=std::make_shared<SparseVoxelMap>(kResolution);
  for(std::size_t i=0;i<kCells;++i){
    const auto* cell=bytes.data()+i*kStride;
    const Eigen::Vector3d point(floatLE(cell),floatLE(cell+4),floatLE(cell+8));
    require(point.allFinite(),"Nonfinite recorded voxel");
    const auto index=map->index(point);const auto center=map->center(index);
    require((center.cast<float>().cast<double>()-point).cwiseAbs().maxCoeff()<1e-6,
            "Snapshot voxel is off the original sparse lattice");
    int8_t occupancy;std::memcpy(&occupancy,cell+12,1);
    map->set(index,static_cast<VoxelStatus>(occupancy));
    map->setTsdf(index,floatLE(cell+16),floatLE(cell+20));
  }
  require(map->size()==kCells,"Duplicate or missing recorded voxels");return map;
}
struct Fixture {StateVec root,diagnostic_from,diagnostic_to;std::vector<StateVec> path;};
Fixture readCandidate(const std::string& filename){
  std::ifstream file(filename);int revision=0,count=0;std::size_t cells=0;
  require(bool(file>>revision>>cells>>count)&&revision==77&&cells==kCells&&count==9,
          "Expected recorded run2 map77 nine-point candidate");
  Fixture fixture;fixture.path.resize(count);
  const auto read=[&](StateVec& state){for(int j=0;j<4;++j)file>>state[j];
    require(bool(file)&&state.allFinite(),"Invalid recorded candidate coordinates");};
  read(fixture.root);read(fixture.diagnostic_from);read(fixture.diagnostic_to);
  for(auto& state:fixture.path)read(state);
  std::string extra;require(!(file>>extra),"Unexpected trailing candidate data");
  require((fixture.diagnostic_to-fixture.path[4]).norm()==0.,"Diagnostic does not identify segment four");
  return fixture;
}
Parameters parameters(){
  // Only geometry/graph settings participate: no local planning, gain or sampling.
  Parameters p;p.ground_robot=true;p.robot_size={1.17,1.17,.25};p.center_offset={0.,0.,.015};
  p.ground.robot_height=.24;p.ground.max_ground_height=.54;
  p.ground.max_inclination=.4363323129985824;p.ground.max_negative_inclination=.4363323129985824;
  p.ground.projection_max_length=1.5;p.ground.interpolate_projection_distance=true;
  p.ground.hanging_vertex_penalty=5.;p.overshoot=0.;p.edge_min=.15;p.edge_max=.8;
  p.neighbor_radius=1.2;p.neighbor_min=.15;p.neighbor_max=1.2;p.refinement.enabled=false;
  p.validate();return p;
}
std::shared_ptr<const MapQuery> bounded(std::shared_ptr<const MapQuery> map){
  return std::make_shared<BoundedMap>(map,GlobalBound::axisAligned({-4.9,-2.9,-.2},{4.9,2.9,2.1}));
}
void state(const StateVec& value){std::cout<<'['<<value[0]<<','<<value[1]<<','<<value[2]<<','<<value[3]<<']';}
void edgeJson(const GroundEdge& edge){
  std::cout<<"{\"status\":\""<<groundStatusName(edge.status)<<"\",\"length_m\":"<<edge.actual_length
    <<",\"max_uphill_rad\":"<<edge.max_uphill<<",\"max_downhill_rad\":"<<edge.max_downhill<<",\"polyline\":[";
  for(std::size_t i=0;i<edge.polyline.size();++i){if(i)std::cout<<',';state(edge.polyline[i]);}
  std::cout<<"]}";
}
void quoted(const std::string& value){
  std::cout<<'"';for(char c:value){if(c=='"'||c=='\\')std::cout<<'\\';
    if(c=='\n')std::cout<<"\\n";else std::cout<<c;}std::cout<<'"';
}
std::vector<int> preservedIds(const GlobalMemory& memory,const std::vector<StateVec>& path){
  std::vector<int> ids;
  for(const auto& waypoint:path){
    int id=-1;for(const auto& item:memory.graph().vertices_map_)
      if((item.second->state.array()==waypoint.array()).all()){id=item.first;break;}
    require(id>=0,"Reference graph did not preserve an original XYZ/yaw waypoint");ids.push_back(id);
  }
  return ids;
}
void requireReferenceEdges(const GlobalMemory& memory,const std::vector<int>& ids){
  for(std::size_t i=1;i<ids.size();++i){
    const auto found=memory.graph().edge_map_.find(ids[i-1]);
    require(found!=memory.graph().edge_map_.end(),"Original reference vertex has no active edges");
    bool present=false;
    for(const auto& edge:found->second)
      if(edge.first==ids[i]&&std::isfinite(edge.second)&&edge.second>=0.)present=true;
    require(present,"An original consecutive reference edge was removed");
  }
}
struct Rejection {std::string name,change;GroundEdge edge;bool reference_rejected;};
Rejection reject(const std::string& name,const std::string& change,
                 std::shared_ptr<const MapQuery> original,std::shared_ptr<const MapQuery> altered,
                 const Parameters& p,const std::vector<StateVec>& path){
  TerrainProjector terrain(altered,p.groundParameters());const auto edge=terrain.validateFixedEdge(path[0],path[1]);
  require(!edge.admissible(),("Fixed edge accepted "+name).c_str());
  GlobalMemory memory(p);memory.updateMap(original);
  require(memory.setHomingPosition(path.front()),"Counterexample home setup failed");
  memory.updateMap(altered);ReferencePathDiagnostics diagnostic;
  const bool stored=memory.addReferencePath(path,nullptr,&diagnostic);
  require(!stored,("Reference insertion accepted "+name).c_str());
  return {name,change,edge,!stored};
}
}

int main(int argc,char** argv){
  try{
    require(argc==3,"Usage: reference_geometry_replay RUN2_MAP.bin EXTRACTED_CANDIDATE.txt");
    const auto observed=readMap(argv[1]);const auto map=bounded(observed);
    const auto fixture=readCandidate(argv[2]);const auto& path=fixture.path;const auto p=parameters();
    TerrainProjector terrain(map,p.groundParameters());

    // Reproduce the former addReferencePath preflight exactly: project root,
    // then feed each newly projected end into the next terrain.edge call.
    const auto first=terrain.project(path.front());
    require(first.status==GroundStatus::Supported,"Original projected root is unsupported");
    StateVec legacy_from=first.projected_state,failed_from=legacy_from;
    std::vector<GroundEdge> legacy;int failed_segment=-1;
    for(std::size_t i=1;i<path.size();++i){
      legacy.push_back(terrain.edge(legacy_from,path[i]));
      if(!legacy.back().admissible()){failed_segment=int(i);failed_from=legacy_from;break;}
      require(!legacy.back().polyline.empty(),"Legacy edge has no endpoint");
      legacy_from=legacy.back().polyline.back();
    }
    require(failed_segment==4&&legacy.back().status==GroundStatus::Occupied,
            "Exact saved candidate did not reproduce segment-four occupied rejection");
    require((failed_from-fixture.diagnostic_from).cwiseAbs().maxCoeff()<1e-8,
            "Legacy failure start differs from the recorded diagnostic");
    const auto diagnostic_edge=terrain.edge(fixture.diagnostic_from,fixture.diagnostic_to);
    require(diagnostic_edge.status==GroundStatus::Occupied,"Recorded segment-four diagnostic no longer reproduces");

    std::vector<GroundEdge> fixed;
    for(std::size_t i=1;i<path.size();++i){
      fixed.push_back(terrain.validateFixedEdge(path[i-1],path[i]));
      require(fixed.back().admissible(),("Original fixed segment "+std::to_string(i)+
              " rejected: "+groundStatusName(fixed.back().status)).c_str());
      require(fixed.back().polyline.size()==2&&
              (fixed.back().polyline.front().array()==path[i-1].array()).all()&&
              (fixed.back().polyline.back().array()==path[i].array()).all(),
              "Fixed validation changed recorded XYZ or yaw");
    }
    GlobalMemory global(p);global.updateMap(map);
    require(global.recordOdometry(fixture.root),"Fresh graph odometry home setup failed");
    const std::size_t before_vertices=global.graph().vertices_map_.size();
    ReferencePathDiagnostics diagnostic;int endpoint=-1;
    const bool stored=global.addReferencePath(path,&endpoint,&diagnostic);
    require(stored,("Original candidate insertion failed: "+diagnostic.reason).c_str());
    const auto preserved=preservedIds(global,path);
    require(preserved.back()==endpoint,"Reference endpoint does not identify the original last waypoint");
    requireReferenceEdges(global,preserved);
    const int edges_before_update=global.edgeCount();
    global.updateMap(map);
    require(global.edgeCount()==edges_before_update&&preservedIds(global,path)==preserved,
            "Same-map update removed or relocated an accepted reference route");
    requireReferenceEdges(global,preserved);
    const auto gain=[](const StateVec&){return VolumetricGain{};};
    const auto route=global.selectFrontier(fixture.root,gain,endpoint);
    require(route.success,("Fixed reference route failed after same-map revalidation: "+route.reason).c_str());
    const std::string snapshot=std::string(argv[2])+".gbp2mem";
    const auto saved=global.saveGraph(snapshot,"map");
    require(saved.success,("Fixed-edge graph save failed: "+saved.reason).c_str());
    GlobalMemory restored(p);restored.updateMap(map);
    const auto loaded=restored.loadGraph(snapshot,"map");
    require(loaded.success,("Fixed-edge graph load failed: "+loaded.reason).c_str());
    require(preservedIds(restored,path)==preserved,"Saved/loaded graph changed original reference waypoints");
    restored.updateMap(map);
    require(preservedIds(restored,path)==preserved,"Updated restored graph changed original reference waypoints");
    requireReferenceEdges(restored,preserved);
    const auto restored_route=restored.selectFrontier(fixture.root,gain,endpoint);
    require(restored_route.success,("Restored reference route failed: "+restored_route.reason).c_str());

    // Four small counterexamples use the same recorded map/parameters. The
    // body cell is lateral to the support ray, so it changes clearance only.
    std::vector<Rejection> rejections;
    const Eigen::Vector3d body=terrain.baseState(path.front()).head<3>()+p.center_offset+Eigen::Vector3d(0.,.3,0.);
    const auto cell=observed->index(body);
    require(observed->voxel(body)==VoxelStatus::Free,"Counterexample body cell was not originally free");
    for(const auto& entry:std::array<std::pair<const char*,VoxelStatus>,2>{{
        {"occupied_body",VoxelStatus::Occupied},{"unknown_body",VoxelStatus::Unknown}}}){
      auto modified=std::make_shared<SparseVoxelMap>(*observed);modified->set(cell,entry.second);
      rejections.push_back(reject(entry.first,"one original free body voxel relabeled; TSDF unchanged",
                                  map,bounded(modified),p,{path[0],path[1]}));
    }
    auto unsupported=std::make_shared<SparseVoxelMap>(*observed);
    const auto column=observed->index(path.front().head<3>());
    for(int x=column.x()-1;x<=column.x()+1;++x)for(int y=column.y()-1;y<=column.y()+1;++y)
      for(int z=column.z()-17;z<=column.z()+1;++z){
        const Eigen::Vector3i index(x,y,z);unsupported->set(index,VoxelStatus::Free);
        unsupported->setTsdf(index,.3,1.);
      }
    rejections.push_back(reject("no_ground","3x3 support column replaced by observed free positive TSDF over the projection range",
                                map,bounded(unsupported),p,{path[0],path[1]}));
    StateVec steep_end=path[3];steep_end.x()+=.01;steep_end.z()+=.02;
    rejections.push_back(reject("steep_segment","same map; from candidate[3], move 1 cm horizontally and 2 cm upward",
                                map,map,p,{path[3],steep_end}));
    require(rejections[0].edge.status==GroundStatus::Occupied&&rejections[1].edge.status==GroundStatus::Unknown&&
            rejections[2].edge.status==GroundStatus::NoGround&&rejections[3].edge.status==GroundStatus::Steep,
            "Counterexamples did not isolate occupied, unknown, absent support and slope checks");

    std::cout<<std::setprecision(17)<<std::boolalpha
      <<"{\"status\":\"passed\",\"map_revision\":77,\"cells\":"<<observed->size()
      <<",\"candidate_points\":"<<path.size()<<",\"candidate_resampled\":false"
      <<",\"global_scope\":\"fresh graph initialized at the saved planning root; no claim to replay historical graph topology\""
      <<",\"legacy\":{\"failed_segment\":"<<failed_segment<<",\"from\":";state(failed_from);
    std::cout<<",\"to\":";state(path[failed_segment]);std::cout<<",\"segments\":[";
    for(std::size_t i=0;i<legacy.size();++i){if(i)std::cout<<',';edgeJson(legacy[i]);}
    std::cout<<"],\"recorded_diagnostic_reproduced\":true},\"fixed_segments\":[";
    for(std::size_t i=0;i<fixed.size();++i){if(i)std::cout<<',';edgeJson(fixed[i]);}
    std::cout<<"],\"global\":{\"reference_stored\":true,\"vertices_before\":"<<before_vertices
      <<",\"vertices_after\":"<<global.graph().vertices_map_.size()<<",\"edges_after\":"<<global.edgeCount()
      <<",\"endpoint\":"<<endpoint<<",\"original_xyz_yaw_preserved\":true,\"original_vertex_ids\":[";
    for(std::size_t i=0;i<preserved.size();++i){if(i)std::cout<<',';std::cout<<preserved[i];}
    std::cout<<"],\"consecutive_reference_edges_preserved\":true,\"same_map_update_preserved\":true,\"route_after_update\":true"
      <<",\"snapshot_saved_and_loaded\":true,\"route_after_load_and_update\":true},\"counterexamples\":[";
    for(std::size_t i=0;i<rejections.size();++i){if(i)std::cout<<',';const auto& r=rejections[i];
      std::cout<<"{\"case\":";quoted(r.name);std::cout<<",\"change\":";quoted(r.change);
      std::cout<<",\"reference_rejected\":"<<r.reference_rejected<<",\"edge\":";edgeJson(r.edge);std::cout<<'}';}
    std::cout<<"]}\n";return 0;
  }catch(const std::exception& error){std::cout<<"{\"status\":\"failed\",\"reason\":";quoted(error.what());
    std::cout<<"}\n";return 1;}
}
