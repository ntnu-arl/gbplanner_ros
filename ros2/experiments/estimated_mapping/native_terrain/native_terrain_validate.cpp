// Project and validate independent B-reference candidates on a recorded map.
// Reuses production sparse-map, global-bound and terrain implementations.
// No ROS nodes, synthetic occupancy, controller or simulator are created.
#include "gbplanner_local/global_bound.hpp"
#include "gbplanner_local/ground_terrain.hpp"
#include "gbplanner_local/sparse_voxel_map.hpp"
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

using namespace gbplanner2;
namespace {
using Tree=boost::property_tree::ptree;
constexpr std::size_t kStride=24;
void require(bool ok,const std::string& reason){if(!ok)throw std::runtime_error(reason);}
template<int N> Eigen::Matrix<double,N,1> vectorIn(const Tree& tree,const std::string& path=""){
  const auto& values=path.empty()?tree:tree.get_child(path);
  require(values.size()==N,"invalid_vector_size:"+path);
  Eigen::Matrix<double,N,1> value;int i=0;
  for(const auto& item:values){require(item.first.empty(),"expected_array:"+path);value[i++]=item.second.get_value<double>();}
  require(value.allFinite(),"nonfinite_vector:"+path);return value;
}
void stringOut(std::ostream& out,const std::string& value){
  out<<'"';for(unsigned char c:value){
    if(c=='"'||c=='\\')out<<'\\'<<c;
    else if(c<32){const char* hex="0123456789abcdef";out<<"\\u00"<<hex[c>>4]<<hex[c&15];}
    else out<<c;
  }out<<'"';
}
template<class T> void vectorOut(std::ostream& out,const T& value){
  out<<'[';for(int i=0;i<value.size();++i){if(i)out<<',';out<<value[i];}out<<']';
}
void numberOut(std::ostream& out,double value){if(std::isfinite(value))out<<value;else out<<"null";}
float floatLE(const std::uint8_t* b){
  static_assert(sizeof(float)==4&&std::numeric_limits<float>::is_iec559,"Requires IEEE float32");
  const std::uint32_t bits=std::uint32_t(b[0])|(std::uint32_t(b[1])<<8)|
      (std::uint32_t(b[2])<<16)|(std::uint32_t(b[3])<<24);
  float value;std::memcpy(&value,&bits,4);return value;
}
struct LoadedMap {
  std::shared_ptr<SparseVoxelMap> map;
  std::filesystem::path path;
  std::size_t count{0},free{0},occupied{0},unknown{0};
};
LoadedMap loadMap(const Tree& source,const std::filesystem::path& input_dir){
  LoadedMap loaded;loaded.count=source.get<std::size_t>("map.count");
  require(loaded.count>0&&loaded.count<=16000000,"map_count_out_of_range");
  loaded.path=source.get<std::string>("map.path");
  if(loaded.path.is_relative())loaded.path=input_dir/loaded.path;
  require(std::filesystem::file_size(loaded.path)==loaded.count*kStride,"map_byte_count_mismatch");
  const double resolution=source.get<double>("map.resolution");
  loaded.map=std::make_shared<SparseVoxelMap>(resolution);loaded.map->reserve(loaded.count);
  std::ifstream file(loaded.path,std::ios::binary);require(bool(file),"map_open_failed");
  std::array<std::uint8_t,kStride> bytes{};
  for(std::size_t i=0;i<loaded.count;++i){
    require(bool(file.read(reinterpret_cast<char*>(bytes.data()),kStride)),"truncated_map");
    const auto* b=bytes.data();const Eigen::Vector3d point(floatLE(b),floatLE(b+4),floatLE(b+8));
    require(point.allFinite(),"nonfinite_map_coordinate");
    const auto index=loaded.map->index(point);const auto center=loaded.map->center(index);
    require((center.cast<float>().cast<double>()-point).cwiseAbs().maxCoeff()<=resolution*1.e-4,
            "map_coordinate_not_voxel_center");
    std::int8_t occupancy;std::memcpy(&occupancy,b+12,1);
    require(occupancy==-1||occupancy==0||occupancy==100,"invalid_occupancy");
    const double distance=floatLE(b+16),weight=floatLE(b+20);
    require(std::isfinite(distance)&&std::isfinite(weight)&&weight>=0.,"invalid_tsdf_field");
    loaded.map->setObservation(index,static_cast<VoxelStatus>(occupancy),distance,weight);
    if(occupancy==0)++loaded.free;else if(occupancy==100)++loaded.occupied;else ++loaded.unknown;
  }
  require(loaded.map->size()==loaded.count,"duplicate_map_voxel");
  return loaded;
}
GroundParameters groundIn(const Tree& source){
  GroundParameters p;
  p.robot_size=vectorIn<3>(source,"robot.size");
  p.center_offset=vectorIn<3>(source,"robot.center_offset");
  const auto& g=source.get_child("ground");
  p.robot_height=g.get<double>("robot_height");p.max_ground_height=g.get<double>("max_ground_height");
  p.max_inclination=g.get<double>("max_inclination");
  p.max_negative_inclination=g.get<double>("max_negative_inclination");
  p.projection_max_length=g.get<double>("projection_max_length");
  p.hanging_vertex_penalty=g.get<double>("hanging_vertex_penalty",0.);
  p.interpolate_projection_distance=g.get<bool>("interpolate_projection_distance");
  require(p.interpolate_projection_distance,"distance_interpolation_required");
  require(g.get<std::string>("tsdf_projection_mode")=="zero_crossing_simplex","unsupported_tsdf_projection_mode");
  require(g.get<std::string>("ray_projection_mode","voxel_surface")=="voxel_surface","unsupported_ray_projection_mode");
  p.tsdf_projection_mode=GroundTsdfProjectionMode::ZeroCrossingSimplex;
  p.ray_projection_mode=GroundRayProjectionMode::VoxelSurface;
  p.point_robot=false;p.validate();return p;
}
struct CandidateResult {
  std::string id,reason{"unprocessed"},stage{"input"};bool success{false};
  int failure_index{-1};std::size_t input_points{0},projected_points{0},validated_segments{0};
  double max_height_change{0.},minimum_bound_clearance{std::numeric_limits<double>::infinity()};
  std::vector<StateVec> projected;
};
CandidateResult checkCandidate(const Tree& candidate,const std::shared_ptr<const MapQuery>& map,
    const GroundParameters& ground,const GlobalBound& bound){
  CandidateResult result;result.id=candidate.get<std::string>("id");
  try{
    const auto& input=candidate.get_child("base_path");result.input_points=input.size();
    require(!input.empty()&&input.size()<=20000,"invalid_path_size");
    // Immutable map reuse caches native queries only; it cannot add support.
    TerrainProjector terrain(map,ground,TerrainQueryReuse::ImmutableSnapshot);
    std::vector<StateVec> planning;planning.reserve(input.size());result.projected.reserve(input.size());
    int index=0;result.stage="projection";
    for(const auto& item:input){
      result.failure_index=index++;
      require(item.first.empty(),"base_path_must_be_array");
      const StateVec base=vectorIn<4>(item.second);StateVec proposed=base;
      proposed.z()+=ground.max_ground_height-ground.robot_height;
      const auto projection=terrain.project(proposed);
      if(projection.status!=GroundStatus::Supported){
        result.reason=std::string("projection_")+groundStatusName(projection.status);return result;
      }
      if((projection.projected_state-proposed).head<2>().norm()>1.e-9){
        result.reason="projection_changed_xy";return result;
      }
      // Carry the exact submitted XY/yaw into the final reference. Native
      // terrain projection is permitted to update only the floor-derived Z.
      StateVec projected=base;projected.z()=terrain.baseState(projection.projected_state).z();
      StateVec native=projected;native.z()+=ground.max_ground_height-ground.robot_height;
      const Eigen::Vector3d center=projected.head<3>()+ground.center_offset;
      const double clearance=std::min((center-ground.robot_size*.5-bound.minimum()).minCoeff(),
          (bound.maximum()-center-ground.robot_size*.5).minCoeff());
      result.minimum_bound_clearance=std::min(result.minimum_bound_clearance,clearance);
      if(!bound.containsBox(center,ground.robot_size)){result.reason="body_outside_global_bound";return result;}
      result.max_height_change=std::max(result.max_height_change,std::abs(projected.z()-base.z()));
      result.projected.push_back(projected);planning.push_back(native);++result.projected_points;
    }
    result.stage="fixed_segment_validation";
    // Match ValidatePath's final stationary endpoint check as well as every
    // actual segment. No observation-only height attachment applies here.
    for(std::size_t i=0;i<planning.size();++i){
      result.failure_index=static_cast<int>(i);
      const auto edge=terrain.validateFixedEdge(planning[i],planning[std::min(i+1,planning.size()-1)],false);
      if(!edge.admissible()){
        result.reason=std::string("fixed_segment_")+groundStatusName(edge.status);return result;
      }
      ++result.validated_segments;
    }
    result.success=true;result.stage="completed";result.reason="projected_and_validated";result.failure_index=-1;
  }catch(const std::exception& error){result.reason=error.what();}
  return result;
}
void candidateOut(std::ostream& out,const CandidateResult& result){
  out<<"{\"id\":";stringOut(out,result.id);
  out<<",\"success\":"<<(result.success?"true":"false")<<",\"status\":";
  stringOut(out,result.success?"passed":"failed");out<<",\"reason\":";stringOut(out,result.reason);
  out<<",\"stage\":";stringOut(out,result.stage);
  out<<",\"failure_index\":"<<result.failure_index<<",\"input_points\":"<<result.input_points
      <<",\"projected_points\":"<<result.projected_points<<",\"validated_segments_including_terminal_pose\":"
      <<result.validated_segments<<",\"maximum_z_change_m\":"<<result.max_height_change
      <<",\"minimum_global_bound_body_clearance_m\":";
  numberOut(out,result.minimum_bound_clearance);
  out<<",\"xy_and_yaw_preserved\":true,\"allow_measured_height_attachment\":false"
      <<",\"projected_base_path\":[";
  for(std::size_t i=0;i<result.projected.size();++i){if(i)out<<',';vectorOut(out,result.projected[i]);}
  out<<"]}";
}
}

int main(int argc,char** argv){
  try{
    require(argc==3,"Usage: native_terrain_validate INPUT.json OUTPUT.json");
    const auto started=std::chrono::steady_clock::now();
    const auto input=std::filesystem::absolute(argv[1]);Tree source;
    boost::property_tree::read_json(input.string(),source);
    const auto loaded=loadMap(source,input.parent_path());const auto ground=groundIn(source);
    const auto minimum=vectorIn<3>(source,"bounds.min"),maximum=vectorIn<3>(source,"bounds.max");
    const auto bound=GlobalBound::axisAligned(minimum,maximum);
    const auto map=std::make_shared<BoundedMap>(loaded.map,bound);
    require(!source.get<bool>("geofence_enabled",false),"active_geofence_requires_native_service_validation");
    const auto& candidates=source.get_child("candidates");require(!candidates.empty(),"empty_candidate_batch");
    std::vector<CandidateResult> results;std::set<std::string> ids;
    for(const auto& item:candidates){
      require(item.first.empty(),"candidates_must_be_array");
      require(ids.insert(item.second.get<std::string>("id")).second,"duplicate_candidate_id");
      results.push_back(checkCandidate(item.second,map,ground,bound));
    }
    std::ofstream out(argv[2]);require(bool(out),"output_open_failed");out<<std::setprecision(17);
    out<<"{\"status\":\"completed\",\"scope\":\"Native terrain projection and fixed-path validation on saved observed fields; no robot motion\","
       <<"\"final_live_ros_validate_path_required\":true,\"exact_bound\":true,\"clearance_margin_m\":0,"
       <<"\"map\":{\"path\":";stringOut(out,loaded.path.string());
    out<<",\"count\":"<<loaded.count<<",\"resolution\":"<<loaded.map->resolution()
       <<",\"bytes\":"<<loaded.count*kStride<<",\"point_step\":24,\"fields\":[\"x\",\"y\",\"z\",\"occupancy\",\"tsdf_distance\",\"tsdf_weight\"],"
       <<"\"free\":"<<loaded.free<<",\"occupied\":"<<loaded.occupied<<",\"unknown\":"<<loaded.unknown<<"},"
       <<"\"robot\":{\"size\":";vectorOut(out,ground.robot_size);out<<",\"center_offset\":";vectorOut(out,ground.center_offset);
    out<<"},\"bounds\":{\"min\":";vectorOut(out,minimum);out<<",\"max\":";vectorOut(out,maximum);
    out<<"},\"ground\":{\"robot_height\":"<<ground.robot_height<<",\"max_ground_height\":"<<ground.max_ground_height
       <<",\"max_inclination\":"<<ground.max_inclination<<",\"max_negative_inclination\":"<<ground.max_negative_inclination
       <<",\"projection_max_length\":"<<ground.projection_max_length<<",\"interpolate_projection_distance\":true,"
       <<"\"tsdf_projection_mode\":\"zero_crossing_simplex\",\"ray_projection_mode\":\"voxel_surface\"},\"candidates\":[";
    for(std::size_t i=0;i<results.size();++i){if(i)out<<',';candidateOut(out,results[i]);}
    out<<"],\"elapsed_wall_s\":"<<std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()<<"}\n";
    require(bool(out),"output_write_failed");
    const auto passed=std::count_if(results.begin(),results.end(),[](const auto& r){return r.success;});
    std::cout<<"Native terrain batch completed: "<<passed<<'/'<<results.size()<<" candidates valid\n";
    return 0;  // A rejected geometric candidate is a valid diagnostic result.
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
