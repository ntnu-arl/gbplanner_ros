// Read-only geometry diagnosis using the production map and terrain classes.
// Reuse the captured-map/fixture reader, but do NOT reconstruct GlobalMemory
// or execute the comparison entry point from the included standalone tool.
#define main unused_native_global_update_comparison_main
#include "compare_native_global_update.cpp"
#undef main
#include <filesystem>

namespace {
void numberOut(std::ostream& out,double value) {
  if(std::isfinite(value))out<<value;else out<<"null";
}
const char* voxelName(VoxelStatus status) {
  switch(status) {
    case VoxelStatus::Free:return "free";
    case VoxelStatus::Occupied:return "occupied";
    case VoxelStatus::Unknown:return "unknown";
  }
  return "invalid";
}
void edgeOut(std::ostream& out,const GroundEdge& edge) {
  out<<"{\"status\":"<<std::quoted(groundStatusName(edge.status))
     <<",\"admissible\":"<<(edge.admissible()?"true":"false")
     <<",\"speculative\":"<<(edge.speculative?"true":"false")
     <<",\"length_m\":";numberOut(out,edge.actual_length);
  out<<",\"max_uphill_rad\":";numberOut(out,edge.max_uphill);
  out<<",\"max_downhill_rad\":";numberOut(out,edge.max_downhill);
  out<<",\"polyline_points\":"<<edge.polyline.size();
  if(!edge.polyline.empty()) {
    out<<",\"first\":";vectorOut(out,edge.polyline.front());
    out<<",\"last\":";vectorOut(out,edge.polyline.back());
  }
  out<<'}';
}
GroundProjection pointOut(std::ostream& out,const VertexInput& vertex,
                          const TerrainProjector& terrain,const MapQuery& map,
                          const GroundParameters& parameters) {
  const auto projected=terrain.project(vertex.state);
  out<<"{\"id\":"<<vertex.id<<",\"state\":";vectorOut(out,vertex.state);
  out<<",\"fixed_self\":";edgeOut(out,terrain.validateFixedEdge(vertex.state,vertex.state,true));
  const Eigen::Vector3d original_body=terrain.baseState(vertex.state).head<3>()+parameters.center_offset;
  out<<",\"original_body_status\":"<<std::quoted(voxelName(map.box(original_body,parameters.robot_size)))
     <<",\"projection\":{\"status\":"<<std::quoted(groundStatusName(projected.status))
     <<",\"support\":"<<(projected.support?"true":"false")
     <<",\"hanging\":"<<(projected.hanging?"true":"false")
     <<",\"state\":";vectorOut(out,projected.projected_state);
  out<<",\"height_shift_m\":";numberOut(out,projected.projected_state.z()-vertex.state.z());
  out<<",\"ground_z\":";numberOut(out,projected.ground_z);
  out<<",\"ray_index\":"<<projected.ray_index
     <<",\"zero_crossing_lower_z\":";numberOut(out,projected.zero_crossing_lower_z);
  out<<",\"zero_crossing_upper_z\":";numberOut(out,projected.zero_crossing_upper_z);
  out<<",\"fixed_self\":";edgeOut(out,terrain.validateFixedEdge(
      projected.projected_state,projected.projected_state,true));
  out<<"}}";
  return projected;
}
struct PairResult {
  bool fixed_out{false},fixed_back{false},ordinary_out{false},ordinary_back{false};
};
PairResult pairOut(std::ostream& out,const VertexInput& from,const VertexInput& to,
                  const TerrainProjector& terrain,const MapQuery& map,
                  const GroundParameters& parameters,bool captured_edge) {
  const auto fixed_out=terrain.validateFixedEdge(from.state,to.state,true);
  const auto fixed_back=terrain.validateFixedEdge(to.state,from.state,true);
  const auto ordinary_out=terrain.edge(from.state,to.state);
  const auto ordinary_back=terrain.edge(to.state,from.state);
  const Eigen::Vector3d delta=(to.state-from.state).head<3>();
  const Eigen::Vector3d body_from=terrain.baseState(from.state).head<3>()+parameters.center_offset;
  const Eigen::Vector3d body_to=terrain.baseState(to.state).head<3>()+parameters.center_offset;
  out<<"{\"from_id\":"<<from.id<<",\"to_id\":"<<to.id
     <<",\"captured_public_edge_present\":"<<(captured_edge?"true":"false")
     <<",\"distance_m\":"<<delta.norm()<<",\"xy_distance_m\":"<<delta.head<2>().norm()
     <<",\"height_difference_m\":"<<delta.z()<<",\"direct_chord_slope_rad\":"
     <<std::atan2(delta.z(),delta.head<2>().norm())
     <<",\"original_body_sweep\":"<<std::quoted(voxelName(map.path(body_from,body_to,parameters.robot_size)))
     <<",\"from\":";
  const auto a=pointOut(out,from,terrain,map,parameters);
  out<<",\"to\":";const auto b=pointOut(out,to,terrain,map,parameters);
  out<<",\"fixed_forward\":";edgeOut(out,fixed_out);
  out<<",\"fixed_reverse\":";edgeOut(out,fixed_back);
  out<<",\"ordinary_forward\":";edgeOut(out,ordinary_out);
  out<<",\"ordinary_reverse\":";edgeOut(out,ordinary_back);
  out<<",\"projected_endpoints_fixed_forward\":";
  edgeOut(out,terrain.validateFixedEdge(a.projected_state,b.projected_state,true));
  out<<",\"projected_endpoints_fixed_reverse\":";
  edgeOut(out,terrain.validateFixedEdge(b.projected_state,a.projected_state,true));
  out<<'}';
  return {fixed_out.admissible(),fixed_back.admissible(),
          ordinary_out.admissible(),ordinary_back.admissible()};
}
}  // namespace

int main(int argc,char** argv) {
  try {
    require(argc==10,"Usage: diagnose_saved_ground_anchors map.bin fixture.txt output.json home_id old_anchor_id root_id radius map_stamp_ns failed_revision");
    require(!std::filesystem::exists(argv[3]),"Refusing to overwrite diagnostic output");
    const int home_id=std::stoi(argv[4]),anchor_id=std::stoi(argv[5]),root_id=std::stoi(argv[6]);
    const double radius=std::stod(argv[7]);
    const auto stamp_ns=std::stoull(argv[8]);const int failed_revision=std::stoi(argv[9]);
    require(std::isfinite(radius)&&radius>0.&&failed_revision>0,"Invalid diagnostic scope");
    const auto fixture=readFixture(argv[2]);const auto sparse=readMap(argv[1],fixture);
    const auto map=std::make_shared<BoundedMap>(sparse,GlobalBound::axisAligned(fixture.minimum,fixture.maximum));
    const auto parameters=fixture.parameters.groundParameters();
    TerrainProjector terrain(map,parameters,TerrainQueryReuse::ImmutableSnapshot);
    const auto vertex=[&](int id)->const VertexInput& {
      for(const auto& candidate:fixture.vertices)if(candidate.id==id)return candidate;
      throw std::runtime_error("Missing requested vertex "+std::to_string(id));
    };
    const auto& home=vertex(home_id);const auto& old_anchor=vertex(anchor_id);const auto& root=vertex(root_id);
    const auto captured_edge=[&](int a,int b) {
      for(const auto& [from,to,weight]:fixture.edges)
        if((from==a&&to==b)||(from==b&&to==a))return true;
      return false;
    };
    std::vector<const VertexInput*> nearby;
    for(const auto& candidate:fixture.vertices)
      if(candidate.id!=home_id&&(candidate.state-home.state).head<3>().norm()<=radius)
        nearby.push_back(&candidate);
    std::sort(nearby.begin(),nearby.end(),[&](const VertexInput* a,const VertexInput* b) {
      const double x=(a->state-home.state).head<3>().squaredNorm();
      const double y=(b->state-home.state).head<3>().squaredNorm();
      return x<y||(x==y&&a->id<b->id);
    });
    std::ostringstream out;out<<std::setprecision(17);
    out<<"{\"status\":\"completed\",\"scope\":\"Read-only production terrain queries on a saved map; no graph rebuilding, mutation or runtime replay\""
       <<",\"exact_live_replay\":false,\"map_acquisition_stamp_ns\":"<<stamp_ns
       <<",\"earlier_failed_revision\":"<<failed_revision
       <<",\"earlier_failed_revision_map_available\":false"
       <<",\"limitations\":[\"The saved map is later than the failed local reference attachment.\","
       <<"\"Captured graph and map were received independently and lack atomic identity.\","
       <<"\"Fixed and ordinary results are separate geometric alternatives; public Graph lacks original fixed-edge metadata.\","
       <<"\"The final root vertex did not establish its existence during earlier failed requests.\","
       <<"\"An admissible pair does not establish a connected route to home or a complete mission.\"]"
       <<",\"map_file\":"<<std::quoted(argv[1])<<",\"fixture_file\":"<<std::quoted(argv[2])
       <<",\"map_rows\":"<<fixture.cells<<",\"stored_cells\":"<<sparse->size()
       <<",\"bound_mode\":3,\"radius_m\":"<<radius<<",\"map_resolution\":"<<fixture.resolution
       <<",\"robot_size\":";vectorOut(out,parameters.robot_size);
    out<<",\"center_offset\":";vectorOut(out,parameters.center_offset);
    out<<",\"ground_robot_height\":"<<parameters.robot_height
       <<",\"ground_max_ground_height\":"<<parameters.max_ground_height
       <<",\"max_inclination_rad\":"<<parameters.max_inclination
       <<",\"max_negative_inclination_rad\":"<<parameters.max_negative_inclination
       <<",\"global_min\":";vectorOut(out,fixture.minimum);
    out<<",\"global_max\":";vectorOut(out,fixture.maximum);
    out<<",\"home_nearby_pairs\":[";
    std::size_t fixed_out=0,fixed_back=0,ordinary_out=0,ordinary_back=0;
    for(std::size_t i=0;i<nearby.size();++i) {
      if(i)out<<',';
      const auto pair=pairOut(out,home,*nearby[i],terrain,*map,parameters,captured_edge(home_id,nearby[i]->id));
      fixed_out+=pair.fixed_out;fixed_back+=pair.fixed_back;
      ordinary_out+=pair.ordinary_out;ordinary_back+=pair.ordinary_back;
    }
    out<<"],\"home_neighbor_summary\":{\"candidate_count\":"<<nearby.size()
       <<",\"fixed_outward_admissible\":"<<fixed_out<<",\"fixed_return_admissible\":"<<fixed_back
       <<",\"ordinary_outward_admissible\":"<<ordinary_out
       <<",\"ordinary_return_admissible\":"<<ordinary_back<<"},\"old_anchor_to_final_root\":";
    pairOut(out,old_anchor,root,terrain,*map,parameters,captured_edge(anchor_id,root_id));
    out<<'}';
    writeResult(argv[3],out.str());
    std::cout<<"{\"status\":\"completed\",\"output\":"<<std::quoted(argv[3])
             <<",\"home_candidates\":"<<nearby.size()<<",\"fixed_outward_admissible\":"<<fixed_out
             <<",\"fixed_return_admissible\":"<<fixed_back
             <<",\"ordinary_outward_admissible\":"<<ordinary_out
             <<",\"ordinary_return_admissible\":"<<ordinary_back<<"}\n";
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
