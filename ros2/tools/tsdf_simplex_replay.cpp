// Focused offline validation of the explicit ROS 2 observed-simplex terrain field.
// Compile directly with voxel_map/sparse_voxel_map/ground_terrain/geofence sources;
// this does not rebuild/install the production library or start ROS.
#include "gbplanner_local/ground_terrain.hpp"
#include "gbplanner_local/sparse_voxel_map.hpp"
#include "gbplanner_local/global_bound.hpp"
#include "gbplanner_local/geofence.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

using namespace gbplanner2;
namespace {
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
void close(double a,double b,double tolerance,const char* message){require(std::abs(a-b)<=tolerance,message);}
double field(const Eigen::Vector3i& i){return .2*i.x()*i.y()+.07*i.y()*i.z()-.11*i.x()*i.z()+
    .03*i.x()*i.y()*i.z()+.1*i.x();}
double affine(const Eigen::Vector3d& p){return .3*p.x()-.4*p.y()+.5*p.z()+.2;}
struct Checks {int affine_queries{0},continuity_faces{0},old_full_stencil_queries{0};double max_face_gap{0.};};
Checks fieldChecks(){
  Checks checks;VoxelMap dense(1.,Eigen::Vector3d::Zero(),{4,4,4});SparseVoxelMap sparse(1.);
  for(int z=0;z<4;++z)for(int y=0;y<4;++y)for(int x=0;x<4;++x){
    const Eigen::Vector3i i(x,y,z);const double d=affine(dense.center(i));
    dense.setTsdf(i,d,2.);sparse.setTsdf(i,d,2.);
  }
  // One strict interior query per permutation of fractional coordinates.
  std::array<int,3> permutation{{0,1,2}};
  do {Eigen::Vector3d fraction;fraction[permutation[0]]=.8;fraction[permutation[1]]=.5;fraction[permutation[2]]=.2;
    const Eigen::Vector3d p=Eigen::Vector3d::Constant(1.5)+fraction;
    const auto a=dense.tsdfDistanceSimplex(p),b=sparse.tsdfDistanceSimplex(p);
    require(a&&b&&a->interpolated&&b->interpolated,"Known simplex was not interpolated");
    close(a->distance,affine(p),1e-13,"Dense simplex lost affine precision");
    close(b->distance,a->distance,1e-13,"Dense/sparse simplex values differ");++checks.affine_queries;
  }while(std::next_permutation(permutation.begin(),permutation.end()));
  for(int z=0;z<4;++z)for(int y=0;y<4;++y)for(int x=0;x<4;++x){
    const Eigen::Vector3i i(x,y,z);dense.setTsdf(i,field(i),2.);sparse.setTsdf(i,field(i),2.);
  }
  for(const Eigen::Vector3d& fraction:{Eigen::Vector3d(.2,.3,.7),Eigen::Vector3d(.8,.5,.2)}){
    double expected=0.;for(int z=0;z<2;++z)for(int y=0;y<2;++y)for(int x=0;x<2;++x)
      expected+=(x?fraction.x():1.-fraction.x())*(y?fraction.y():1.-fraction.y())*
          (z?fraction.z():1.-fraction.z())*field(Eigen::Vector3i(1+x,1+y,1+z));
    const Eigen::Vector3d p=Eigen::Vector3d::Constant(1.5)+fraction;
    const auto a=dense.tsdfDistance(p),b=sparse.tsdfDistance(p);
    require(a&&b&&a->interpolated&&b->interpolated,"Old full stencil became unavailable");
    close(a->distance,expected,1e-13,"Old dense trilinear value changed");
    close(b->distance,expected,1e-13,"Old sparse trilinear value changed");++checks.old_full_stencil_queries;
  }
  const auto shared_face=[&](Eigen::Vector3d face,const Eigen::Vector3d& normal){
    constexpr double epsilon=1e-9;
    const auto exact=sparse.tsdfDistanceSimplex(face),left=sparse.tsdfDistanceSimplex(face-epsilon*normal),
        right=sparse.tsdfDistanceSimplex(face+epsilon*normal),dense_exact=dense.tsdfDistanceSimplex(face);
    require(exact&&left&&right&&dense_exact,"Shared face interpolation unavailable");
    const double gap=std::max(std::abs(left->distance-exact->distance),std::abs(right->distance-exact->distance));
    require(gap<1e-8,"Simplex field is discontinuous across a shared face");
    close(exact->distance,dense_exact->distance,1e-13,"Dense/sparse shared-face values differ");
    checks.max_face_gap=std::max(checks.max_face_gap,gap);++checks.continuity_faces;
  };
  // Internal tetrahedron faces: tie each pair above and below the third axis.
  for(int a=0;a<3;++a)for(int b=a+1;b<3;++b)for(double other:{.2,.8}){
    Eigen::Vector3d f=Eigen::Vector3d::Constant(other);f[a]=f[b]=.5;
    Eigen::Vector3d normal=Eigen::Vector3d::Zero();normal[a]=1.;normal[b]=-1.;
    shared_face(Eigen::Vector3d::Constant(1.5)+f,normal);
  }
  // Neighboring cubes, including non-affine four-corner face values.
  for(int axis=0;axis<3;++axis){Eigen::Vector3d grid(1.2,1.7,1.3);grid[axis]=2.;
    shared_face(grid+Eigen::Vector3d::Constant(.5),Eigen::Vector3d::Unit(axis));}
  // A face query needs only its nonzero barycentric vertices. Moving off the
  // observed face into a tetrahedron with a missing required corner must fail.
  SparseVoxelMap partial(1.);partial.setTsdf({0,0,0},.2,1.);partial.setTsdf({1,1,1},-.2,1.);
  const auto diagonal=partial.tsdfDistanceSimplex({1.,1.,1.});
  require(diagonal&&diagonal->interpolated,"Zero-weight missing corners invalidated an observed edge");
  require(!partial.tsdfDistanceSimplex({1.1,1.,.9}),"Missing nonzero-weight corner was manufactured");
  require(!partial.tsdfDistanceSimplex({2.1,1.,.9}),"Simplex extrapolated outside observed convex hull");
  partial.setTsdf({1,0,0},.1,1e-7);partial.setTsdf({1,1,0},.1,1.);
  require(!partial.tsdfDistanceSimplex({1.1,1.,.9}),"Subthreshold required corner was accepted");
  auto wrapped=std::make_shared<SparseVoxelMap>(sparse);
  auto bounded=std::make_shared<BoundedMap>(wrapped,GlobalBound::axisAligned({-5,-5,-5},{5,5,5}));
  GeofenceMap fenced(bounded,std::make_shared<GeofenceSet>());
  const Eigen::Vector3d query(1.7,1.8,2.2);const auto direct=sparse.tsdfDistanceSimplex(query),policy=fenced.tsdfDistanceSimplex(query);
  require(direct&&policy,"Policy wrapper lost simplex observations");
  close(direct->distance,policy->distance,0.,"Policy wrapper changed physical TSDF");
  return checks;
}
double hiddenValleyCheck(){
  auto map=std::make_shared<VoxelMap>(1.,Eigen::Vector3d::Zero(),Eigen::Vector3i(2,2,2));map->fill(VoxelStatus::Free);
  const std::array<double,8> values{{.01,.3,.3,.3,.3,.3,-.3,.01}};
  for(int z=0;z<2;++z)for(int y=0;y<2;++y)for(int x=0;x<2;++x)
    map->setTsdf({x,y,z},values[x+2*y+4*z],1.);
  GroundParameters p;p.max_ground_height=p.robot_height=.54;p.robot_size.setZero();p.point_robot=true;
  p.projection_max_length=.6;p.interpolate_projection_distance=true;p.tsdf_projection_mode=GroundTsdfProjectionMode::ZeroCrossingSimplex;
  const auto upper=map->tsdfDistanceSimplex({.7,.8,1.}),knot=map->tsdfDistanceSimplex({.7,.8,.8}),lower=map->tsdfDistanceSimplex({.7,.8,.5});
  require(upper&&knot&&lower&&upper->distance>0.&&lower->distance>0.&&knot->distance<0.,"Valley fixture malformed");
  const auto projected=TerrainProjector(map,p).project(StateVec(.7,.8,1.,0.));
  require(projected.status==GroundStatus::Supported,"Tetrahedron knot first crossing was missed");
  close(projected.ground_z,.8724137931034483,1e-12,"Did not select the first valley zero");
  return projected.ground_z;
}
float floatLE(const uint8_t* p){const uint32_t bits=uint32_t(p[0])|(uint32_t(p[1])<<8)|(uint32_t(p[2])<<16)|(uint32_t(p[3])<<24);
  float value;std::memcpy(&value,&bits,4);return value;}
std::shared_ptr<SparseVoxelMap> readMap(const char* name){
  constexpr std::size_t cells=53388,stride=24;std::ifstream f(name,std::ios::binary|std::ios::ate);
  require(bool(f)&&f.tellg()==std::streamoff(cells*stride),"Expected exact motion6 map155");
  f.seekg(0);std::vector<uint8_t> data(cells*stride);require(bool(f.read(reinterpret_cast<char*>(data.data()),data.size())),"Map read failed");
  auto map=std::make_shared<SparseVoxelMap>(.1);
  for(std::size_t i=0;i<cells;++i){const auto* c=data.data()+i*stride;const Eigen::Vector3d p(floatLE(c),floatLE(c+4),floatLE(c+8));
    int8_t occupancy;std::memcpy(&occupancy,c+12,1);const auto index=map->index(p);
    map->set(index,static_cast<VoxelStatus>(occupancy));map->setTsdf(index,floatLE(c+16),floatLE(c+20));}
  require(map->size()==cells,"Map cells lost or duplicated");return map;
}
GroundParameters parameters(GroundTsdfProjectionMode mode){
  GroundParameters p;p.robot_height=.24;p.max_ground_height=.54;p.robot_size={1.17,1.17,.25};p.center_offset={0,0,.015};
  p.max_inclination=p.max_negative_inclination=.4363323129985824;p.projection_max_length=1.5;
  p.interpolate_projection_distance=true;p.tsdf_projection_mode=mode;return p;
}
StateVec planning(StateVec state){state.z()+=.3;return state;}
void states(const std::vector<StateVec>& path,TerrainProjector& terrain){
  std::cout<<'[';for(std::size_t i=0;i<path.size();++i){if(i)std::cout<<',';
    std::cout<<'"'<<groundStatusName(terrain.validateFixedEdge(path[i],path[i],true).status)<<'"';}std::cout<<']';
}
void segments(const std::vector<StateVec>& path,TerrainProjector& terrain){
  std::cout<<'[';for(std::size_t i=1;i<path.size();++i){if(i>1)std::cout<<',';
    std::cout<<'"'<<groundStatusName(terrain.validateFixedEdge(path[i-1],path[i],true).status)<<'"';}std::cout<<']';
}
}
int main(int argc,char** argv){try{
  require(argc==2,"Usage: tsdf_simplex_replay motion6_accepted.bin");const auto checks=fieldChecks();const double valley=hiddenValleyCheck();
  const auto map=readMap(argv[1]);const auto current=planning(StateVec(.817455530166626,.17979110777378082,.401286780834198,-1.3478479005734028));
  const auto old=planning(StateVec(.8087021112442017,-.0003776156227104366,.4013959467411041,.006418558852157355));
  std::vector<StateVec> route{
    {.8087040185928345,-.00037747513852082193,.4013959467411041,.0064183187900226915},
    {.808882474899292,-.00037553341826424,.38241632829643923,.006422206859996943},
    {.7644591638918234,.19452202357404433,.3961816117654886,1.7948993454016426},
    {.7259526105987799,.36346110071186916,.3944441056821746,1.7948993454016429},
    {.7853414745194236,.5543055399880814,.37605159493222695,1.26910543694949},
    {.8847583178654186,.8737787622462085,.3752801304218374,1.2691054369494905},
    {1.1287079300033755,.8624644189929396,.37215039683941165,-.0463466241573247}};
  for(auto& state:route)state=planning(state);
  std::cout<<std::setprecision(17)<<"{\"status\":\"passed\",\"map_revision\":155,\"cells\":"<<map->size()
    <<",\"checks\":{\"affine_queries\":"<<checks.affine_queries<<",\"shared_faces\":"<<checks.continuity_faces
    <<",\"max_shared_face_epsilon_gap\":"<<checks.max_face_gap<<",\"old_full_stencil_queries\":"<<checks.old_full_stencil_queries
    <<",\"missing_required_corner_rejected\":true,\"outside_observed_hull_rejected\":true,\"subthreshold_weight_rejected\":true,\"policy_forwarding\":true"
    <<",\"first_hidden_valley_zero\":"<<valley<<"},\"modes\":[";
  bool first=true;for(const auto mode:{GroundTsdfProjectionMode::ZeroCrossing,GroundTsdfProjectionMode::ZeroCrossingSimplex}){
    if(!first)std::cout<<',';
    first=false;const auto p=parameters(mode);TerrainProjector terrain(map,p);
    const auto projection=terrain.project(current);const auto connector=terrain.validateFixedEdge(current,old,true);
    std::cout<<"{\"mode\":\""<<groundTsdfProjectionModeName(mode)<<"\",\"current_support\":\""<<groundStatusName(projection.status)
      <<"\",\"current_ground_z\":";if(std::isfinite(projection.ground_z))std::cout<<projection.ground_z;else std::cout<<"null";
    std::cout<<",\"current_ray_index\":"<<projection.ray_index<<",\"frozen_connector\":\""<<groundStatusName(connector.status)
      <<"\",\"route_point_status\":";states(route,terrain);std::cout<<",\"route_segment_status\":";segments(route,terrain);std::cout<<'}';
    if(mode==GroundTsdfProjectionMode::ZeroCrossing){require(projection.status==GroundStatus::NoGround&&connector.status==GroundStatus::NoGround,"Old real-map zero-crossing behavior changed");}
    else require(projection.status==GroundStatus::Supported&&projection.ray_index==0&&connector.admissible(),"Simplex did not restore exact current support/connector");
  }
  const auto p=parameters(GroundTsdfProjectionMode::ZeroCrossingSimplex);
  const auto body_index=map->index(current.head<3>()+Eigen::Vector3d(0,0,-.3));
  auto obstacle=std::make_shared<SparseVoxelMap>(*map);obstacle->set(body_index,VoxelStatus::Occupied);
  require(TerrainProjector(obstacle,p).validateFixedEdge(current,current,true).status==GroundStatus::Occupied,"Occupied body accepted");
  auto unknown=std::make_shared<SparseVoxelMap>(*map);unknown->set(body_index,VoxelStatus::Unknown);
  require(TerrainProjector(unknown,p).validateFixedEdge(current,current,true).status==GroundStatus::Unknown,"Unknown body accepted");
  auto absent=std::make_shared<SparseVoxelMap>(*map);absent->setTsdf({7,1,0},0.,0.);
  require(!absent->tsdfDistanceSimplex({current.x(),current.y(),.14}),"Missing real-map required corner accepted");
  require(TerrainProjector(absent,p).validateFixedEdge(current,current,true).status!=GroundStatus::Admissible,"Missing real-map support accepted");
  std::cout<<"],\"real_map_negative_cases\":{\"occupied_body_rejected\":true,\"unknown_body_rejected\":true,\"missing_required_support_rejected\":true}}\n";
  return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
