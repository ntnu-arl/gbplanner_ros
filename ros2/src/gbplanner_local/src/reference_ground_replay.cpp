// Offline diagnosis of the recorded SMB map through the real planning core.
// Usage: reference_ground_replay MAP.bin BASE_X BASE_Y BASE_Z YAW HEADING
// The fixed experiment profile below was checked against
// src/gbplanner_smb/config/exploration.yaml and node defaults on 2026-10-30.
// It is not a reconstruction of the original global graph or map revision.
#include "gbplanner_local/global_memory.hpp"
#include "gbplanner_local/global_bound.hpp"
#include "gbplanner_local/sparse_voxel_map.hpp"
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace gbplanner2;
namespace {
constexpr std::size_t kCells=85067,kStride=24;
constexpr double kResolution=.1;
const Eigen::Vector3d kMinimum(-4.9,-2.9,-.2),kMaximum(4.9,2.9,2.1);

void string(const std::string& value){
  std::cout<<'"';
  for(unsigned char c:value){
    if(c=='"'||c=='\\')std::cout<<'\\'<<char(c);
    else if(c<32)std::cout<<"\\u00"<<std::hex<<std::setw(2)<<std::setfill('0')<<int(c)<<std::dec<<std::setfill(' ');
    else std::cout<<char(c);
  }
  std::cout<<'"';
}
template<class Derived> void vector(const Eigen::MatrixBase<Derived>& value){
  std::cout<<'[';
  for(Eigen::Index i=0;i<value.size();++i){if(i)std::cout<<',';std::cout<<value[i];}
  std::cout<<']';
}
double number(const char* text){
  std::size_t consumed=0;const double result=std::stod(text,&consumed);
  if(text[consumed]!='\0'||!std::isfinite(result))throw std::invalid_argument("Invalid numeric argument");
  return result;
}
float floatLE(const uint8_t* bytes){
  static_assert(sizeof(float)==4&&std::numeric_limits<float>::is_iec559,"Requires IEEE float32");
  const uint32_t bits=uint32_t(bytes[0])|(uint32_t(bytes[1])<<8)|
                      (uint32_t(bytes[2])<<16)|(uint32_t(bytes[3])<<24);
  float result;std::memcpy(&result,&bits,sizeof(result));return result;
}
std::shared_ptr<SparseVoxelMap> readMap(const std::string& path){
  std::ifstream file(path,std::ios::binary|std::ios::ate);
  if(!file||file.tellg()!=std::streamoff(kCells*kStride))
    throw std::runtime_error("Expected the verified run7 85067-cell, 24-byte little-endian snapshot");
  std::vector<uint8_t> bytes(kCells*kStride);file.seekg(0);
  if(!file.read(reinterpret_cast<char*>(bytes.data()),bytes.size()))throw std::runtime_error("Map read failed");
  auto map=std::make_shared<SparseVoxelMap>(kResolution);
  for(std::size_t i=0;i<kCells;++i){
    const auto* cell=bytes.data()+i*kStride;
    const Eigen::Vector3d point(floatLE(cell),floatLE(cell+4),floatLE(cell+8));
    if(!point.allFinite())throw std::runtime_error("Nonfinite recorded voxel position");
    const auto index=map->index(point);const auto center=map->center(index);
    for(int axis=0;axis<3;++axis)
      if(std::abs(double(float(center[axis]))-point[axis])>kResolution*1e-4)
        throw std::runtime_error("Recorded voxel is not aligned with the sparse lattice");
    int8_t occupancy;std::memcpy(&occupancy,cell+12,1);
    map->set(index,static_cast<VoxelStatus>(occupancy));
    map->setTsdf(index,floatLE(cell+16),floatLE(cell+20));
  }
  if(map->size()!=kCells)throw std::runtime_error("Recorded snapshot has duplicate or missing voxels");
  return map;
}
Parameters smbParameters(){
  Parameters p;
  p.local_min={-2.5,-2.,-.08};p.local_max={2.5,2.,.08};
  p.local_rotations.setZero();p.gain_min_extension.setZero();p.gain_max_extension.setZero();
  p.robot_size={1.17,1.17,.25};p.center_offset={0.,0.,.015};p.point_robot=false;
  p.ground_robot=true;p.ground.robot_height=.24;p.ground.max_ground_height=.54;
  p.ground.max_inclination=.4363323129985824;p.ground.max_negative_inclination=.4363323129985824;
  p.ground.projection_max_length=1.5;p.ground.interpolate_projection_distance=true;
  p.ground.hanging_vertex_penalty=5.;
  p.seed=42;p.distribution="uniform";p.sample_mean.setZero();p.sample_scale={1.,1.,.5};
  p.max_vertices=100;p.max_edges=1200;p.max_attempts=3000;
  p.edge_min=.15;p.edge_max=.8;p.overshoot=0.;
  p.max_velocity=.3;p.traverse_length_max=6.;p.traverse_time_max=40.;
  p.neighbor_radius=1.2;p.neighbor_min=.15;p.neighbor_max=1.2;
  p.graph_mode=true;p.tangent_yaw=true;p.leaf_gain_only=false;
  p.path_length_penalty=.07;p.path_direction_penalty=.3;
  p.cluster_vertices_for_gain=true;p.clustering_radius=.4;
  p.adaptive.enabled=false;p.refinement.enabled=false;
  p.refinement.planning_backward=false;p.refinement.homing_backward=false;
  p.gain.use_ray_model=true;p.gain.ray_method="iterative";p.gain.nonuniform_ray_cast=true;
  p.gain.ray_cast_step_size_multiplier=1.;p.gain.exp_gain_voxel_size=.2;
  p.gain.max_ray_samples=1000000;p.gain.max_voxels=1000000;
  p.unknown_gain=1.;p.free_gain=0.;p.occupied_gain=0.;
  SensorParameters sensor;sensor.name="smb_lidar";sensor.type="kLidar";
  sensor.range=6.;sensor.min_range=.1;sensor.center_offset.setZero();sensor.rotations.setZero();
  sensor.horizontal_fov=6.283185307179586;sensor.vertical_fov=1.0471975511965976;
  sensor.horizontal_resolution=.10471975511965977;sensor.vertical_resolution=.17453292519943295;
  sensor.frontier_threshold=.05;p.sensors={sensor};p.validate();return p;
}
void configuration(const Parameters& p){
  std::cout<<"{\"profile\":\"SMB exploration.yaml plus node defaults, checked 2026-10-30\""
    <<",\"bound_mode\":3,\"map_resolution\":"<<kResolution<<",\"global_min\":";vector(kMinimum);
  std::cout<<",\"global_max\":";vector(kMaximum);
  std::cout<<",\"robot_size\":";vector(p.robot_size);std::cout<<",\"center_offset\":";vector(p.center_offset);
  std::cout<<",\"point_robot\":"<<p.point_robot<<",\"ground_robot\":"<<p.ground_robot
    <<",\"ground\":{\"robot_height\":"<<p.ground.robot_height<<",\"max_ground_height\":"<<p.ground.max_ground_height
    <<",\"max_inclination\":"<<p.ground.max_inclination<<",\"max_negative_inclination\":"<<p.ground.max_negative_inclination
    <<",\"projection_max_length\":"<<p.ground.projection_max_length
    <<",\"interpolate_projection_distance\":"<<p.ground.interpolate_projection_distance
    <<",\"hanging_vertex_penalty\":"<<p.ground.hanging_vertex_penalty<<'}'
    <<",\"seed\":"<<p.seed<<",\"local_min\":";vector(p.local_min);std::cout<<",\"local_max\":";vector(p.local_max);
  std::cout<<",\"local_rotations\":";vector(p.local_rotations);
  std::cout<<",\"gain_min_extension\":";vector(p.gain_min_extension);
  std::cout<<",\"gain_max_extension\":";vector(p.gain_max_extension);
  std::cout<<",\"sampling_axes\":[";const auto axes=p.samplingAxes();
  for(std::size_t i=0;i<axes.size();++i){if(i)std::cout<<',';const auto& a=axes[i];
    std::cout<<"{\"distribution\":";string(a.distribution);
    std::cout<<",\"minimum\":"<<a.minimum<<",\"maximum\":"<<a.maximum<<",\"mean\":"<<a.mean<<",\"scale\":"<<a.scale<<'}';}
  std::cout<<"],\"max_vertices\":"<<p.max_vertices<<",\"max_edges\":"<<p.max_edges<<",\"max_attempts\":"<<p.max_attempts
    <<",\"edge_min\":"<<p.edge_min<<",\"edge_max\":"<<p.edge_max<<",\"overshoot\":"<<p.overshoot
    <<",\"max_velocity\":"<<p.max_velocity<<",\"traverse_length_max\":"<<p.traverse_length_max
    <<",\"traverse_time_max\":"<<p.traverse_time_max<<",\"neighbor_radius\":"<<p.neighbor_radius
    <<",\"neighbor_min\":"<<p.neighbor_min<<",\"neighbor_max\":"<<p.neighbor_max
    <<",\"graph_mode\":"<<p.graph_mode<<",\"tangent_yaw\":"<<p.tangent_yaw<<",\"leaf_gain_only\":"<<p.leaf_gain_only
    <<",\"path_length_penalty\":"<<p.path_length_penalty<<",\"path_direction_penalty\":"<<p.path_direction_penalty
    <<",\"cluster_vertices_for_gain\":"<<p.cluster_vertices_for_gain<<",\"clustering_radius\":"<<p.clustering_radius
    <<",\"adaptive_enabled\":"<<p.adaptive.enabled<<",\"refinement_enabled\":"<<p.refinement.enabled
    <<",\"planning_backward\":"<<p.refinement.planning_backward<<",\"homing_backward\":"<<p.refinement.homing_backward
    <<",\"gain\":{\"use_ray_model\":"<<p.gain.use_ray_model<<",\"ray_method\":";string(p.gain.ray_method);
  std::cout<<",\"nonuniform_ray_cast\":"<<p.gain.nonuniform_ray_cast
    <<",\"ray_cast_step_size_multiplier\":"<<p.gain.ray_cast_step_size_multiplier
    <<",\"max_ray_samples\":"<<p.gain.max_ray_samples<<",\"exp_gain_voxel_size\":"<<p.gain.exp_gain_voxel_size
    <<",\"max_voxels\":"<<p.gain.max_voxels<<",\"unknown\":"<<p.unknown_gain
    <<",\"free\":"<<p.free_gain<<",\"occupied\":"<<p.occupied_gain<<'}';
  const auto& s=p.sensors.front();std::cout<<",\"sensor\":{\"name\":";string(s.name);std::cout<<",\"type\":";string(s.type);
  std::cout<<",\"range\":"<<s.range<<",\"min_range\":"<<s.min_range<<",\"center_offset\":";vector(s.center_offset);
  std::cout<<",\"rotations\":";vector(s.rotations);
  std::cout<<",\"horizontal_fov\":"<<s.horizontal_fov<<",\"vertical_fov\":"<<s.vertical_fov
    <<",\"horizontal_resolution\":"<<s.horizontal_resolution<<",\"vertical_resolution\":"<<s.vertical_resolution
    <<",\"frontier_threshold\":"<<s.frontier_threshold<<'}'
    <<",\"geofence_enabled\":false,\"no_gain_zones\":[],\"global_background_enabled\":false,\"mission_enabled\":false}";
}
}

int main(int argc,char** argv){
  try{
    if(argc!=7)throw std::invalid_argument("Usage: reference_ground_replay MAP.bin BASE_X BASE_Y BASE_Z YAW HEADING");
    StateVec base;for(int i=0;i<4;++i)base[i]=number(argv[i+2]);
    const double heading=number(argv[6]);const Parameters p=smbParameters();
    const auto observed=readMap(argv[1]);
    auto map=std::make_shared<BoundedMap>(observed,GlobalBound::axisAligned(kMinimum,kMaximum));
    StateVec root=base;root.z()+=p.ground.max_ground_height-p.ground.robot_height;
    GlobalMemory global(p);global.updateMap(map);
    // Match fresh native auto_home's actual odometry entry point, including
    // its existing projection calls; never invent historical graph vertices.
    const bool home=global.recordOdometry(root);
    LocalPlanner planner(map,p);const auto result=planner.plan(root,heading);
    planner_msgs::msg::Graph graph;planner.graphMessage(graph);
    global.recordOdometry(root);
    global.absorbLocal(graph,[&](const StateVec& state){return planner.rayGain(state);});
    const auto vertices_before=global.graph().vertices_map_.size();const int edges_before=global.edgeCount();
    ReferencePathDiagnostics diagnostics;int endpoint=-1;
    const bool stored=result.success&&global.addReferencePath(result.path,&endpoint,&diagnostics);
    std::cout<<std::setprecision(17)<<std::boolalpha
      <<"{\"status\":\"completed\",\"scope\":\"native core offline replay; fresh odometry home, no historical global graph\""
      <<",\"snapshot_limit\":\"Saved run7 map revision 107; original failure used 106 and candidate waypoints were not recorded\""
      <<",\"map_file\":";string(argv[1]);std::cout<<",\"cells\":"<<observed->size()
      <<",\"map_layout\":\"little-endian, 24-byte stride; xyz float32 at 0/4/8, occupancy int8 at 12, TSDF float32 at 16/20\""
      <<",\"base_root\":";vector(base);std::cout<<",\"planning_root\":";vector(root);
    std::cout<<",\"exploring_direction\":"<<heading<<",\"configuration\":";configuration(p);
    std::cout<<",\"local\":{\"success\":"<<result.success<<",\"reason\":";string(result.reason);
    std::cout<<",\"vertices\":"<<result.vertices<<",\"edges\":"<<result.edges<<",\"attempts\":"<<result.attempts
      <<",\"collision_rejections\":"<<result.collision_rejections<<",\"gain\":"<<result.gain
      <<",\"gain_evaluations\":"<<result.gain_evaluations<<",\"reused_gains\":"<<result.reused_gains
      <<",\"best_id\":"<<result.best_id<<",\"executed_target_id\":"<<result.executed_target_id
      <<",\"candidate_path_length_m\":"<<result.candidate_path_length<<",\"path_length_m\":"<<result.path_length
      <<",\"gain_evaluated\":"<<result.gain_evaluated<<",\"frontier_exists\":"<<result.frontier_exists
      <<",\"direction_distance\":"<<result.direction_distance<<",\"direction_factor\":"<<result.direction_factor
      <<",\"truncated_for_hanging\":"<<result.truncated_for_hanging<<",\"truncated_for_budget\":"<<result.truncated_for_budget
      <<",\"build_ms\":"<<result.build_ms<<",\"evaluate_ms\":"<<result.evaluate_ms<<'}'
      <<",\"candidate_coordinates\":\"planning_height\",\"candidate_path\":[";
    for(std::size_t i=0;i<result.path.size();++i){if(i)std::cout<<',';vector(result.path[i]);}
    std::cout<<"],\"global\":{\"fresh_home_initialized\":"<<home<<",\"vertices_before_reference\":"<<vertices_before
      <<",\"edges_before_reference\":"<<edges_before<<",\"vertices_after_reference\":"<<global.graph().vertices_map_.size()
      <<",\"edges_after_reference\":"<<global.edgeCount()<<",\"reference_stored\":"<<stored<<",\"endpoint\":"<<endpoint<<'}'
      <<",\"reference_path_diagnostics\":{\"reason\":";string(diagnostics.reason);
    std::cout<<",\"segment\":"<<diagnostics.segment<<",\"coordinates\":\"planning_height\",\"from\":";vector(diagnostics.from);
    std::cout<<",\"to\":";vector(diagnostics.to);std::cout<<"}}\n";return 0;
  }catch(const std::exception& error){
    std::cout<<"{\"status\":\"error\",\"reason\":";string(error.what());std::cout<<"}\n";return 1;
  }
}
