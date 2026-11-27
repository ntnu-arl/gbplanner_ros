// Compare the upstream sampled ray with the native projection on fixed input.
// No planner, resampling, thresholds, map cells or recorded poses are changed.
#include "gbplanner_local/ground_terrain.hpp"
#include "gbplanner_local/sparse_voxel_map.hpp"
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <array>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace gbplanner2;
using boost::property_tree::ptree;
namespace {
void require(bool value,const char* reason){if(!value)throw std::runtime_error(reason);}
const char* status(VoxelStatus value){return value==VoxelStatus::Free?"free":value==VoxelStatus::Occupied?"occupied":"unknown";}
void xyz(const Eigen::Vector3d& value){std::cout<<'['<<value.x()<<','<<value.y()<<','<<value.z()<<']';}
struct Query {Eigen::Vector3d point,index,center;VoxelStatus status;};
struct Ray {std::vector<Query> queries;VoxelStatus status{VoxelStatus::Free};double hit_z{0.},distance{0.};};

// Literal scalar counterpart of voxblox_common_impl.cpp:167-230 and
// voxblox/core/common.h:getGridIndexFromPoint. Occupancy is already classified
// by the recorded mapper; no interpolated TSDF is used to classify a query.
Ray sourceRay(const MapQuery& map,const Eigen::Vector3d& start,double length){
  const float voxel_size=map.resolution();const float inv=1.0/voxel_size;
  const Eigen::Vector3d end=start-Eigen::Vector3d(0,0,length),ray=end-start;
  const double ray_length=ray.norm();const Eigen::Vector3d direction=ray/ray_length;
  Ray result;result.hit_z=end.z();result.distance=ray_length;
  for(double d=0;d<ray_length;d+=voxel_size*.9){
    Query q;q.point=start+d*direction;
    const Eigen::Vector3f point=q.point.cast<float>();
    for(int k=0;k<3;++k)q.index[k]=std::floor(point[k]*inv+1.e-6f);
    q.center=(q.index.array()+.5).matrix()*map.resolution();q.status=map.voxel(q.center);
    result.queries.push_back(q);
    if(q.status!=VoxelStatus::Free){result.status=q.status;result.hit_z=q.point.z();
      result.distance=std::abs(start.z()-q.point.z());return result;}
  }
  return result; // End-point metadata does not change the source's Free return.
}
class TraceMap final:public MapQuery {
 public:
  explicit TraceMap(std::shared_ptr<const MapQuery> map):map_(std::move(map)){}
  double resolution()const override{return map_->resolution();}
  Eigen::Vector3d minimum()const override{return map_->minimum();}
  Eigen::Vector3d maximum()const override{return map_->maximum();}
  VoxelStatus voxel(const Eigen::Vector3d& p)const override{
    const auto value=map_->voxel(p);queries.push_back({p,value});return value;
  }
  std::optional<TsdfDistance> tsdfDistance(const Eigen::Vector3d& p)const override{return map_->tsdfDistance(p);}
  VoxelStatus box(const Eigen::Vector3d& p,const Eigen::Vector3d& size)const override{return map_->box(p,size);}
  VoxelStatus path(const Eigen::Vector3d& a,const Eigen::Vector3d& b,const Eigen::Vector3d& size)const override{return map_->path(a,b,size);}
  void ray(const Eigen::Vector3d& a,const Eigen::Vector3d& b,const std::function<bool(const Eigen::Vector3d&,VoxelStatus)>& visit)const override{map_->ray(a,b,visit);}
  mutable std::vector<std::pair<Eigen::Vector3d,VoxelStatus>> queries;
 private:std::shared_ptr<const MapQuery> map_;
};
GroundParameters parameters(){
  GroundParameters p;p.max_ground_height=.54;p.robot_height=.24;p.robot_size={1.17,1.17,.25};
  p.center_offset={0,0,.015};p.max_inclination=p.max_negative_inclination=.4363323129985824;
  p.projection_max_length=1.5;p.ray_projection_mode=GroundRayProjectionMode::UpstreamSample;return p;
}
void compare(const char* label,std::shared_ptr<const MapQuery> map,const StateVec& candidate,const GroundParameters& p){
  const auto source=sourceRay(*map,candidate.head<3>(),p.projection_max_length);
  auto traced=std::make_shared<TraceMap>(map);const auto native=TerrainProjector(traced,p).project(candidate);
  require(traced->queries.size()>=source.queries.size(),"Native ray omitted a source sample");
  for(std::size_t i=0;i<source.queries.size();++i)
    require((traced->queries[i].first.array()==source.queries[i].center.array()).all()&&
            traced->queries[i].second==source.queries[i].status,"Native/source sample index or status differs");
  if(source.status==VoxelStatus::Occupied)
    require(native.support&&native.ray_index==0&&native.ground_z==source.hit_z&&native.projection_distance==source.distance,
            "Native/source occupied sample coordinates differ");
  if(source.status==VoxelStatus::Unknown)
    require(native.hanging&&native.projection_distance==source.distance,"Unknown range was skipped");
  if(source.status==VoxelStatus::Free)require(native.status==GroundStatus::NoGround,"Endpoint became a support hit");
  std::cout<<"{\"case\":\""<<label<<"\",\"start\":";xyz(candidate.head<3>());
  std::cout<<",\"source_status\":\""<<status(source.status)<<"\",\"native_status\":\""<<groundStatusName(native.status)
    <<"\",\"hit_z\":"<<source.hit_z<<",\"samples\":[";
  for(std::size_t i=0;i<source.queries.size();++i){const auto& q=source.queries[i];if(i)std::cout<<',';
    std::cout<<"{\"point\":";xyz(q.point);std::cout<<",\"source_index\":";xyz(q.index);
    std::cout<<",\"native_query_center\":";xyz(traced->queries[i].first);
    std::cout<<",\"status\":\""<<status(q.status)<<"\"}";}
  std::cout<<"]}";
}
float floatLE(const uint8_t* p){
  static_assert(sizeof(float)==4&&std::numeric_limits<float>::is_iec559,"Requires IEEE float32");
  uint32_t bits=uint32_t(p[0])|(uint32_t(p[1])<<8)|(uint32_t(p[2])<<16)|(uint32_t(p[3])<<24);
  float value;std::memcpy(&value,&bits,4);return value;
}
ptree json(const std::string& file){ptree tree;boost::property_tree::read_json(file,tree);return tree;}
std::shared_ptr<SparseVoxelMap> readMap(const std::string& prefix,const ptree& metadata){
  const auto count=metadata.get<std::size_t>("width");
  require(count==84178&&metadata.get<int>("height")==1&&metadata.get<int>("point_step")==24&&
    !metadata.get<bool>("is_bigendian")&&metadata.get<std::size_t>("data_bytes")==24*count,"Expected run5 84178-cell map layout");
  const std::array<std::pair<const char*,int>,6> fields{{{"x",0},{"y",4},{"z",8},{"occupancy",12},{"tsdf_distance",16},{"tsdf_weight",20}}};
  std::size_t field=0;
  for(const auto& item:metadata.get_child("fields")){
    require(field<fields.size()&&item.second.get<std::string>("name")==fields[field].first&&
      item.second.get<int>("offset")==fields[field].second&&item.second.get<int>("datatype")== (field==3?1:7)&&
      item.second.get<int>("count")==1,"Unexpected map field");++field;
  }
  require(field==fields.size(),"Missing map field");
  std::ifstream input(prefix+"_map.bin",std::ios::binary|std::ios::ate);
  require(bool(input)&&input.tellg()==std::streamoff(24*count),"Wrong snapshot length");
  std::vector<uint8_t> bytes(24*count);input.seekg(0);
  require(bool(input.read(reinterpret_cast<char*>(bytes.data()),bytes.size())),"Map read failed");
  auto map=std::make_shared<SparseVoxelMap>(.1);
  for(std::size_t i=0;i<count;++i){const auto* cell=bytes.data()+24*i;
    const Eigen::Vector3d point(floatLE(cell),floatLE(cell+4),floatLE(cell+8));const auto index=map->index(point);
    require((map->center(index).cast<float>().cast<double>()-point).cwiseAbs().maxCoeff()<1e-6,"Map cell is off the world lattice");
    int8_t value;std::memcpy(&value,cell+12,1);map->set(index,static_cast<VoxelStatus>(value));
    map->setTsdf(index,floatLE(cell+16),floatLE(cell+20));
  }
  require(map->size()==count,"Duplicate snapshot cells");return map;
}
StateVec readState(const ptree& array){StateVec state;int i=0;for(const auto& value:array){
  require(i<4,"Too many pose values");state[i++]=value.second.get_value<double>();}
  require(i==4&&state.allFinite(),"Invalid recorded pose");return state;}
class ProjectivePlane final:public MapQuery {
 public:
  enum class Case {Plane,UnknownGap,FallbackJump};
  explicit ProjectivePlane(Case value=Case::Plane):kind_(value){}
  double resolution()const override{return .1;}
  Eigen::Vector3d minimum()const override{return {-10,-10,-10};}
  Eigen::Vector3d maximum()const override{return {10,10,10};}
  VoxelStatus voxel(const Eigen::Vector3d& p)const override{return p.z()<0?VoxelStatus::Occupied:VoxelStatus::Free;}
  std::optional<TsdfDistance> tsdfDistance(const Eigen::Vector3d& p)const override{
    if(kind_==Case::UnknownGap&&p.z()>.15&&p.z()<.25)return std::nullopt;
    if(kind_==Case::FallbackJump)return TsdfDistance{p.z()>0?.2:-.2,1.,false};
    // A projective field with vertical slope 2, clipped at the mapper's
    // truncation distance. Its unique continuous zero is the z=0 plane.
    return TsdfDistance{std::clamp(2.*p.z(),-.3,.3),1.,true};
  }
  VoxelStatus box(const Eigen::Vector3d& p,const Eigen::Vector3d& size)const override{return path(p,p,size);}
  VoxelStatus path(const Eigen::Vector3d& a,const Eigen::Vector3d& b,const Eigen::Vector3d& size)const override{
    return std::min(a.z(),b.z())-.5*size.z()<0.?VoxelStatus::Occupied:VoxelStatus::Free;
  }
  void ray(const Eigen::Vector3d&,const Eigen::Vector3d&,
           const std::function<bool(const Eigen::Vector3d&,VoxelStatus)>&)const override{
    throw std::logic_error("Analytic TSDF fixture does not supply occupancy rays");
  }
 private:Case kind_;
};
void zeroCrossingChecks(){
  auto settings=parameters();settings.interpolate_projection_distance=true;
  auto plane=std::make_shared<ProjectivePlane>();TerrainProjector legacy(plane,settings);
  settings.tsdf_projection_mode=GroundTsdfProjectionMode::ZeroCrossing;
  TerrainProjector crossing(plane,settings);
  std::cout<<",\"zero_crossing_analytic\":{\"field\":\"clamp(2*z,-0.3,0.3)\",\"physical_plane_z\":0,\"starts\":[";
  std::array<double,2> roots{};int index=0;
  for(double height:{.54,.71}){
    const StateVec start(0,0,height,0);const auto old=legacy.project(start),zero=crossing.project(start);
    require(zero.status==GroundStatus::Supported&&std::abs(zero.ground_z)<1e-12&&
      zero.zero_crossing_lower_z<=0.&&zero.zero_crossing_upper_z>=0.&&
      zero.zero_crossing_upper_z-zero.zero_crossing_lower_z<=.0001,"Zero crossing missed analytic plane or bracket bound");
    roots[index]=zero.ground_z;if(index++)std::cout<<',';
    std::cout<<"{\"start_z\":"<<height<<",\"upstream_step_z\":"<<old.ground_z
      <<",\"upstream_step_status\":\""<<groundStatusName(old.status)<<"\",\"zero_crossing_z\":"<<zero.ground_z
      <<",\"map_root_bracket\":["<<zero.zero_crossing_lower_z<<','<<zero.zero_crossing_upper_z<<"]}";
  }
  require(std::abs(roots[0]-roots[1])<1e-12,"Zero-crossing result depends on start height");
  const auto unknown=TerrainProjector(std::make_shared<ProjectivePlane>(ProjectivePlane::Case::UnknownGap),settings).project(StateVec(0,0,.54,0));
  const auto jumped=TerrainProjector(std::make_shared<ProjectivePlane>(ProjectivePlane::Case::FallbackJump),settings).project(StateVec(0,0,.54,0));
  const auto negative=crossing.project(StateVec(0,0,-.01,0));
  const auto fixed=crossing.validateFixedEdge(StateVec(0,0,.59,0),StateVec(0,0,.54,0));
  require(unknown.hanging&&!unknown.support&&jumped.status==GroundStatus::NoGround&&!jumped.support&&
    negative.status==GroundStatus::NoGround&&!negative.support&&fixed.status==GroundStatus::Steep,
    "Zero crossing crossed unknown, accepted a discontinuity/negative start, or relaxed fixed geometry");
  std::cout<<"],\"unknown_gap\":\""<<groundStatusName(unknown.status)<<"\",\"fallback_jump\":\""<<groundStatusName(jumped.status)
    <<"\",\"negative_start\":\""<<groundStatusName(negative.status)<<"\",\"fixed_vertical_edge\":\""<<groundStatusName(fixed.status)<<"\"}";
}
}

int main(int argc,char** argv){
  try{
    require(argc==2,"Usage: ground_ray_projection_demo RUN5_PREFIX (without extension)");
    auto p=parameters();auto flat=std::make_shared<SparseVoxelMap>(.1);
    for(int x=-20;x<=20;++x)for(int y=-20;y<=20;++y)for(int z=-15;z<=25;++z)
      flat->set({x,y,z},z<0?VoxelStatus::Occupied:VoxelStatus::Free);
    std::cout<<std::setprecision(17)<<"{\"status\":\"passed\",\"source_step\":"<<double(float(.1))*.9<<",\"source_comparisons\":[";
    compare("negative_boundary",flat,StateVec(-1e-9,.05,.54,0),p);std::cout<<',';
    compare("shifted_height",flat,StateVec(-1e-9,.05,.59,0),p);std::cout<<',';
    auto unknown=std::make_shared<SparseVoxelMap>(*flat);
    for(int x=-20;x<=20;++x)for(int y=-20;y<=20;++y)unknown->set({x,y,2},VoxelStatus::Unknown);
    compare("unknown_stops_ray",unknown,StateVec(.05,.05,.54,0),p);std::cout<<',';
    auto endpoint=std::make_shared<SparseVoxelMap>(*flat);
    for(int x=-20;x<=20;++x)for(int y=-20;y<=20;++y)endpoint->set({x,y,0},VoxelStatus::Occupied);
    auto short_ray=p;short_ray.projection_max_length=.4;
    compare("endpoint_is_not_hit",endpoint,StateVec(.05,.05,.49,0),short_ray);
    TerrainProjector terrain(flat,p);const StateVec a(.05,.05,.59,0),b(.05,.05,.54,0);
    const auto observed=terrain.validateFixedEdge(a,b,true),fixed=terrain.validateFixedEdge(a,b,false);
    const auto moving=terrain.validateFixedEdge(a,StateVec(.06,.05,.54,0),true);
    require(observed.admissible()&&fixed.status==GroundStatus::Steep&&moving.status==GroundStatus::Steep,
            "Observed-only attachment or fixed/moving slope contract changed");
    const auto phase=terrain.project(StateVec(.05,.05,.54,0));
    const double error=std::abs(terrain.baseState(phase.projected_state).z()-.24);
    std::cout<<"],\"height_attachment\":{\"observed\":\""<<groundStatusName(observed.status)
      <<"\",\"fixed\":\""<<groundStatusName(fixed.status)<<"\",\"nonzero_xy\":\""<<groundStatusName(moving.status)
      <<"\"},\"phase_vs_existing_terminal_tolerance\":{\"height_error_m\":"<<error
      <<",\"terminal_tolerance_m\":0.075,\"within_tolerance\":"<<(error<=.075?"true":"false")<<"}";
    zeroCrossingChecks();

    const std::string prefix=argv[1];const auto metadata=json(prefix+"_map.json");
    const auto report=json(prefix+"_invalidation.json");const auto& inputs=report.get_child("recorded_inputs");
    require(inputs.get<int>("matched_recorded_revision")==91&&inputs.get<int>("map_cells")==84178&&
      inputs.get<int64_t>("map_stamp.sec")==metadata.get<int64_t>("stamp.sec")&&
      inputs.get<int64_t>("map_stamp.nanosec")==metadata.get<int64_t>("stamp.nanosec"),"Snapshot is not the reported run5/map91");
    const auto map=readMap(prefix,metadata);std::vector<StateVec> route;
    for(const auto& item:inputs.get_child("original_commanded_path"))route.push_back(readState(item.second));
    require(route.size()==9,"Expected recorded nine-point route");
    StateVec from=route[route.size()-2],to=route.back();from.z()+=.3;to.z()+=.3;
    const auto original_sweep=map->path(route[route.size()-2].head<3>()+p.center_offset,
      route.back().head<3>()+p.center_offset,p.robot_size);
    std::cout<<",\"map91\":{\"cells\":84178,\"sha256\":\""<<inputs.get<std::string>("map_sha256")
      <<"\",\"scope\":\"Saved map91, recorded final segment; the original failing request was not recorded\",\"original_sweep\":\""
      <<status(original_sweep)<<"\",\"modes\":[";
    for(int mode=0;mode<4;++mode){auto settings=p;settings.interpolate_projection_distance=mode>=2;
      settings.ray_projection_mode=mode==0?GroundRayProjectionMode::VoxelSurface:GroundRayProjectionMode::UpstreamSample;
      settings.tsdf_projection_mode=mode==3?GroundTsdfProjectionMode::ZeroCrossing:GroundTsdfProjectionMode::UpstreamStep;
      TerrainProjector projector(map,settings);const auto first=projector.project(from),last=projector.project(to);
      StateVec nominal=to;nominal.z()=.54;const auto fresh=projector.project(nominal);
      if(mode)std::cout<<',';
      std::cout<<"{\"mode\":\""<<(mode>=2?groundTsdfProjectionModeName(settings.tsdf_projection_mode):groundRayProjectionModeName(settings.ray_projection_mode))
        <<"\",\"from_status\":\""<<groundStatusName(first.status)<<"\",\"to_status\":\""<<groundStatusName(last.status)
        <<"\",\"original_fixed_status\":\""<<groundStatusName(projector.validateFixedEdge(from,to).status)<<'"';
      if(first.support&&last.support){const auto sweep=map->path(projector.baseState(first.projected_state).head<3>()+p.center_offset,
          projector.baseState(last.projected_state).head<3>()+p.center_offset,p.robot_size);
        std::cout<<",\"projected_from_base\":";xyz(projector.baseState(first.projected_state).head<3>());
        std::cout<<",\"projected_to_base\":";xyz(projector.baseState(last.projected_state).head<3>());
        std::cout<<",\"projected_sweep\":\""<<status(sweep)<<'"';}
      if(last.support&&fresh.support)std::cout<<",\"nominal_start_base_z\":"<<projector.baseState(fresh.projected_state).z()
        <<",\"phase_difference_m\":"<<fresh.projected_state.z()-last.projected_state.z();
      if(mode==3&&first.support&&last.support){
        require(first.zero_crossing_upper_z-first.zero_crossing_lower_z<=.0001&&
          last.zero_crossing_upper_z-last.zero_crossing_lower_z<=.0001,"Recorded root bracket exceeds bound");
        std::cout<<",\"from_map_root_bracket\":["<<first.zero_crossing_lower_z<<','<<first.zero_crossing_upper_z
          <<"],\"to_map_root_bracket\":["<<last.zero_crossing_lower_z<<','<<last.zero_crossing_upper_z<<']';
      }
      std::cout<<'}';
    }
    std::cout<<"],\"last_point_source_trace\":";compare("map91_last_point",map,to,p);
    std::cout<<"}}\n";return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
