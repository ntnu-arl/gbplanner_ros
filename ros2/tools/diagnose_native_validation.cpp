// Read-only native route diagnosis; link against the production planner library.
// Usage: diagnose_native_validation map.bin fixture.txt
// Numeric fixture: resolution; row count; size XYZ; offset XYZ;
// robot_height max_ground_height max_inclination max_negative_inclination projection_max_length;
// bounds minXYZ maxXYZ; route count; then name count observedFirst and base XYZYaw rows.
#include "gbplanner_local/path_validation.hpp"
#include "gbplanner_local/sparse_voxel_map.hpp"
#include "gbplanner_local/global_bound.hpp"
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

using namespace gbplanner2;
namespace {
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
void readVector(std::istream& input,Eigen::Vector3d& value){
  input>>value.x()>>value.y()>>value.z();
  require(bool(input)&&value.allFinite(),"Invalid fixture vector");
}
struct Route {std::string name;bool observed_first{false};std::vector<StateVec> base;};
struct Fixture {
  std::size_t cells{0};
  Eigen::Vector3d size,minimum,maximum;
  PathValidationPolicy policy;
  std::vector<Route> routes;
};
Fixture readFixture(const char* filename){
  std::ifstream input(filename);require(bool(input),"Cannot open fixture");Fixture f;
  input>>f.policy.resolution>>f.cells;readVector(input,f.size);readVector(input,f.policy.center_offset);
  auto& ground=f.policy.ground;
  input>>ground.robot_height>>ground.max_ground_height>>ground.max_inclination
    >>ground.max_negative_inclination>>ground.projection_max_length;
  f.policy.ground_robot=true;ground.robot_size=f.size;ground.center_offset=f.policy.center_offset;
  ground.point_robot=f.size.isZero(0.);ground.interpolate_projection_distance=true;
  ground.tsdf_projection_mode=GroundTsdfProjectionMode::ZeroCrossingSimplex;ground.validate();
  readVector(input,f.minimum);readVector(input,f.maximum);
  std::size_t routes=0;input>>routes;require(bool(input)&&routes>0,"Invalid route count");
  for(std::size_t r=0;r<routes;++r){
    Route route;std::size_t count=0;int observed=0;input>>route.name>>count>>observed;
    require(bool(input)&&count>0&&count<=20000&&(observed==0||observed==1),"Invalid route header");
    route.observed_first=observed;
    for(std::size_t i=0;i<count;++i){
      StateVec state;input>>state.x()>>state.y()>>state.z()>>state[3];
      require(bool(input)&&state.allFinite(),"Invalid base pose");route.base.push_back(state);
    }
    f.routes.push_back(std::move(route));
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
  constexpr std::size_t stride=24;
  require(fixture.cells<=std::size_t(std::numeric_limits<std::streamoff>::max())/stride,
          "Map row count overflows byte length");
  const auto length=fixture.cells*stride;
  std::ifstream input(filename,std::ios::binary|std::ios::ate);
  require(bool(input)&&input.tellg()==std::streamoff(length),"Map byte length differs from fixture row count");
  std::vector<std::uint8_t> bytes(length);input.seekg(0);
  require(bool(input.read(reinterpret_cast<char*>(bytes.data()),std::streamsize(length))),"Cannot read map");
  auto map=std::make_shared<SparseVoxelMap>(fixture.policy.resolution);map->reserve(fixture.cells);
  for(std::size_t row=0;row<fixture.cells;++row){
    const auto* data=bytes.data()+row*stride;
    const Eigen::Vector3d point(floatLE(data),floatLE(data+4),floatLE(data+8));
    const auto index=map->index(point);std::int8_t status;std::memcpy(&status,data+12,1);
    map->set(index,static_cast<VoxelStatus>(status));
    map->setTsdf(index,floatLE(data+16),floatLE(data+20));
  }
  // Absent rows and zero-weight Unknown cells remain unobserved; do not fill.
  return map;
}
template<class Value>void vectorJson(const Value& value){
  std::cout<<'[';for(int i=0;i<value.size();++i){if(i)std::cout<<',';std::cout<<value[i];}std::cout<<']';
}
const char* statusName(VoxelStatus status){
  return status==VoxelStatus::Free?"free":status==VoxelStatus::Occupied?"occupied":"unknown";
}
struct Segment {
  std::size_t index,next;bool observed;
  StateVec from,to;
  Eigen::Vector3d body_from,body_to;
  std::string reason,error,body_error;
  VoxelStatus body{VoxelStatus::Unknown};
};
void routeJson(const Route& route,const Fixture& fixture,const PathValidationContext& context,
               const std::shared_ptr<const MapQuery>& map){
  std::vector<Segment> segments;int first_bad=-1,first_bad_body=-1;
  for(std::size_t i=0;i<route.base.size();++i){
    Segment segment;segment.index=i;segment.next=std::min(i+1,route.base.size()-1);
    segment.observed=i==0&&route.observed_first;
    segment.from=route.base[i];segment.to=route.base[segment.next];
    const double height=fixture.policy.ground.max_ground_height-fixture.policy.ground.robot_height;
    segment.from.z()+=height;segment.to.z()+=height;
    // Follow the production base -> planning -> base arithmetic exactly.
    segment.body_from=context.baseState(segment.from).head<3>()+fixture.policy.center_offset;
    segment.body_to=context.baseState(segment.to).head<3>()+fixture.policy.center_offset;
    try {
      segment.reason=segment.observed?
        context.validateObservedGroundConnection(segment.from,segment.to,fixture.size):
        context.validateSegment(segment.from,segment.to,fixture.size);
    }catch(const std::exception& error){segment.reason="invalid_geometry";segment.error=error.what();}
    try {segment.body=map->path(segment.body_from,segment.body_to,fixture.size);}
    catch(const std::exception& error){segment.body_error=error.what();}
    if(!segment.reason.empty()&&first_bad<0)first_bad=static_cast<int>(i);
    if((!segment.body_error.empty()||segment.body!=VoxelStatus::Free)&&first_bad_body<0)
      first_bad_body=static_cast<int>(i);
    segments.push_back(std::move(segment));
  }
  std::cout<<"{\"name\":"<<std::quoted(route.name)<<",\"points\":"<<route.base.size()
    <<",\"first_pose_is_observation\":"<<(route.observed_first?"true":"false")
    <<",\"first_bad_index\":"<<first_bad<<",\"first_bad_body_index\":"<<first_bad_body
    <<",\"segments\":[";
  for(std::size_t i=0;i<segments.size();++i){
    if(i)std::cout<<',';const auto& s=segments[i];
    std::cout<<"{\"index\":"<<s.index<<",\"next_index\":"<<s.next
      <<",\"observed_connection\":"<<(s.observed?"true":"false")
      <<",\"valid\":"<<(s.reason.empty()?"true":"false")
      <<",\"reason\":"<<std::quoted(s.reason.empty()?"ok":s.reason)
      <<",\"body_status\":"<<std::quoted(s.body_error.empty()?statusName(s.body):"invalid_geometry")
      <<",\"validation_exception\":"<<std::quoted(s.error)
      <<",\"body_exception\":"<<std::quoted(s.body_error)<<",\"base_from\":";
    vectorJson(route.base[s.index]);std::cout<<",\"base_to\":";vectorJson(route.base[s.next]);
    std::cout<<",\"planning_from\":";vectorJson(s.from);std::cout<<",\"planning_to\":";vectorJson(s.to);
    std::cout<<",\"body_from\":";vectorJson(s.body_from);std::cout<<",\"body_to\":";vectorJson(s.body_to);
    std::cout<<'}';
  }
  std::cout<<"]}";
}
}  // namespace
int main(int argc,char** argv){try{
  require(argc==3,"Usage: diagnose_native_validation map.bin fixture.txt");
  const auto fixture=readFixture(argv[2]);const auto sparse=readMap(argv[1],fixture);
  const auto map=std::make_shared<BoundedMap>(sparse,GlobalBound::axisAligned(fixture.minimum,fixture.maximum));
  // No captured revision identity is proved by matching the cloud timestamp.
  const auto snapshot=std::make_shared<const PathValidationMap>(PathValidationMap{map,0});
  const PathValidationContext context(fixture.policy,snapshot);
  std::cout<<std::setprecision(17)<<"{\"scope\":\"offline production segment validation on captured matching-stamp cloud; exact revision not proven\""
    <<",\"exact_revision_proven\":false,\"full_validate_path_service_request\":false"
    <<",\"commanded_geometry_changed\":false,\"map_rows\":"<<fixture.cells
    <<",\"stored_cells\":"<<sparse->size()<<",\"resolution\":"<<fixture.policy.resolution
    <<",\"tsdf_projection\":\"zero_crossing_simplex\",\"interpolate_projection_distance\":true"
    <<",\"size\":";vectorJson(fixture.size);std::cout<<",\"offset\":";vectorJson(fixture.policy.center_offset);
  std::cout<<",\"routes\":[";
  for(std::size_t i=0;i<fixture.routes.size();++i){if(i)std::cout<<',';routeJson(fixture.routes[i],fixture,context,map);}
  std::cout<<"]}\n";return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
