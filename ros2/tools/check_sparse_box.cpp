// Differential checks against the unchanged stationary swept-path query.
/* Standalone build (Eigen headers required; no ROS runtime needed):
c++ -std=c++17 -O3 -I src/gbplanner_local/include -I /usr/include/eigen3 \
  tools/check_sparse_box.cpp src/gbplanner_local/src/sparse_voxel_map.cpp \
  -o /tmp/check_sparse_box
*/
// Run without arguments for checks; --benchmark [repetitions] also times
// a small fixed all-free fixture. Timings do not model a full planner request.
#include "gbplanner_local/sparse_voxel_map.hpp"
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace gbplanner2;
namespace {
using Vec=Eigen::Vector3d;
using Index=Eigen::Vector3i;
struct Query {Vec point,size;};
struct Outcome {
  VoxelStatus status{VoxelStatus::Unknown};
  int exception{0};
  std::string message;
  bool operator==(const Outcome& other)const {
    return status==other.status&&exception==other.exception&&message==other.message;
  }
};
Outcome evaluate(const SparseVoxelMap& map,const Query& query,bool oracle) {
  try {
    return {oracle?map.path(query.point,query.point,query.size):map.box(query.point,query.size),0,{}};
  } catch(const std::invalid_argument& error) {return {VoxelStatus::Unknown,1,error.what()};}
    catch(const std::out_of_range& error) {return {VoxelStatus::Unknown,2,error.what()};}
}
class Checks {
 public:
  Outcome compare(const SparseVoxelMap& map,const Query& query,const char* fixture) {
    const auto expected=evaluate(map,query,true),actual=evaluate(map,query,false);
    if(!(actual==expected)) {
      std::ostringstream error;error<<std::setprecision(17)<<fixture<<": box/path mismatch; resolution="
        <<map.resolution()<<", point="<<query.point.transpose()<<", size="<<query.size.transpose()
        <<", box status/exception="<<int(actual.status)<<'/'<<actual.exception
        <<", path status/exception="<<int(expected.status)<<'/'<<expected.exception;
      throw std::runtime_error(error.str());
    }
    ++queries;if(actual.exception)++exceptions;return actual;
  }
  void expect(const SparseVoxelMap& map,const Query& query,VoxelStatus expected,const char* fixture) {
    const auto actual=compare(map,query,fixture);
    if(actual.exception||actual.status!=expected)
      throw std::runtime_error(std::string(fixture)+": explicit contact/state expectation failed");
  }
  std::uint64_t queries{0},exceptions{0};
};
void fillFree(SparseVoxelMap& map,int radius) {
  const std::size_t width=2*radius+1;map.reserve(width*width*width);
  for(int z=-radius;z<=radius;++z)for(int y=-radius;y<=radius;++y)
    for(int x=-radius;x<=radius;++x)map.set({x,y,z},VoxelStatus::Free);
}
void explicitChecks(Checks& checks) {
  SparseVoxelMap map(1.);fillFree(map,3);
  map.set({0,0,0},VoxelStatus::Occupied);
  const double inf=std::numeric_limits<double>::infinity();
  for(const Vec& size:std::vector<Vec>{Vec::Zero(),Vec::Constant(.5)}) {
    const double boundary=1.+size.x()*.5;
    for(const Vec& point:std::vector<Vec>{{boundary,.5,.5},{boundary,boundary,.5},
                                         {boundary,boundary,boundary}}) {
      checks.expect(map,{point,size},VoxelStatus::Occupied,"inclusive face/edge/corner");
      Vec separated=point;separated.x()=std::nextafter(boundary,inf);
      checks.expect(map,{separated,size},VoxelStatus::Free,"one-ULP separation");
      separated.x()=std::nextafter(boundary,-inf);
      checks.expect(map,{separated,size},VoxelStatus::Occupied,"one-ULP overlap");
    }
  }
  map.set({0,0,0},VoxelStatus::Free);map.set({-2,-1,-1},VoxelStatus::Occupied);
  checks.expect(map,{{-1.,-.5,-.5},Vec::Zero()},VoxelStatus::Occupied,"negative contact");
  checks.expect(map,{{std::nextafter(-1.,inf),-.5,-.5},Vec::Zero()},VoxelStatus::Free,
                "negative one-ULP separation");

  // Unknown is encountered before this occupied corner in traversal order.
  SparseVoxelMap mixed(1.);mixed.set({1,1,1},VoxelStatus::Occupied);
  checks.expect(mixed,{{.5,.5,.5},{2.,2.,2.}},VoxelStatus::Occupied,"occupied beats unknown");
  mixed.set({1,1,1},VoxelStatus::Unknown);
  checks.expect(mixed,{{.5,.5,.5},{2.,2.,2.}},VoxelStatus::Unknown,"absent cells stay unknown");

  const Query small{{.5,.5,.5},{.2,.2,.2}};
  mixed.setTsdf({0,0,0},.2,2.);
  checks.expect(mixed,small,VoxelStatus::Unknown,"TSDF alone is not free");
  mixed.set({0,0,0},VoxelStatus::Free);
  checks.expect(mixed,small,VoxelStatus::Free,"unknown to free");
  mixed.set({0,0,0},VoxelStatus::Unknown);
  checks.expect(mixed,small,VoxelStatus::Unknown,"free to TSDF-backed unknown");
  mixed.set({0,0,0},VoxelStatus::Occupied);
  checks.expect(mixed,small,VoxelStatus::Occupied,"unknown to occupied");
  mixed.setTsdf({0,0,0},0.,0.);
  checks.expect(mixed,small,VoxelStatus::Occupied,"TSDF removal preserves occupancy");
  mixed.set({0,0,0},VoxelStatus::Unknown);
  checks.expect(mixed,small,VoxelStatus::Unknown,"erased occupied cell");
}
void boundaryChecks(Checks& checks,double resolution) {
  SparseVoxelMap map(resolution);fillFree(map,6);
  const double inf=std::numeric_limits<double>::infinity();
  const std::vector<Vec> sizes{Vec::Zero(),Vec(.1,.3,.7)*resolution,Vec::Constant(resolution),
                               Vec(2.,3.,1.)*resolution,Vec(.001,2.,.25)*resolution};
  for(const Index& index:std::vector<Index>{{-3,-2,-1},{0,0,0},{2,3,1}}) {
    for(const auto status:{VoxelStatus::Occupied,VoxelStatus::Unknown}) {
      map.set(index,status);
      // Retain a TSDF-backed Unknown alongside absent-cell Unknown fixtures.
      if(status==VoxelStatus::Unknown)map.setTsdf(index,0.,1.);
      for(const auto& size:sizes) {
        const Vec half=size*.5;
        const Vec lo=index.cast<double>()*resolution-half;
        const Vec hi=lo+Vec::Constant(resolution)+size;
        for(int axes=1;axes<8;++axes)for(int side=0;side<2;++side) {
          Vec point=map.center(index);
          for(int axis=0;axis<3;++axis)if(axes&(1<<axis))point[axis]=side?hi[axis]:lo[axis];
          checks.compare(map,{point,size},"expanded-AABB contact");
          for(int axis=0;axis<3;++axis)if(axes&(1<<axis))for(double direction:{-inf,inf}) {
            Vec adjacent=point;adjacent[axis]=std::nextafter(point[axis],direction);
            checks.compare(map,{adjacent,size},"expanded-AABB nextafter");
          }
        }
      }
      map.set(index,VoxelStatus::Free);map.setTsdf(index,0.,0.);
    }
  }
  // Random interiors exercise both all-free scans and mixed sparse occupancy.
  std::mt19937 random(1729);
  std::uniform_real_distribution<double> position(-5.5,5.5),extent(.01,4.);
  for(int fixture=0;fixture<2;++fixture) {
    if(fixture)for(int z=-6;z<=6;++z)for(int y=-6;y<=6;++y)for(int x=-6;x<=6;++x) {
      const int pattern=(x+6)*17+(y+6)*11+(z+6)*7;
      map.set({x,y,z},pattern%13==0?VoxelStatus::Occupied:
                       pattern%7==0?VoxelStatus::Unknown:VoxelStatus::Free);
    }
    for(int n=0;n<250;++n) {
      const Vec point=Vec(position(random),position(random),position(random))*resolution;
      Vec size=Vec::Zero();
      if(n%3)size=Vec(extent(random),extent(random),extent(random))*resolution;
      checks.compare(map,{point,size},fixture?"random mixed":"random free");
    }
  }
}
void domainChecks(Checks& checks,double resolution) {
  SparseVoxelMap map(resolution);
  constexpr int limit=SparseVoxelMap::kIndexLimit;
  for(int sign:{-1,1})for(int z=0;z<3;++z)for(int y=0;y<3;++y)for(int x=0;x<3;++x)
    map.set(Index::Constant(sign*limit)-sign*Index(x,y,z),VoxelStatus::Free);
  map.set(Index::Constant(limit),VoxelStatus::Occupied);
  const double inf=std::numeric_limits<double>::infinity();
  const std::vector<Vec> sizes{Vec::Zero(),Vec::Constant(.5*resolution),Vec::Constant(2.*resolution)};
  for(double boundary:{map.minimum().x(),map.maximum().x()}) {
    for(double coordinate:{std::nextafter(boundary,-inf),boundary,std::nextafter(boundary,inf)})
      for(const auto& size:sizes)for(int axes=1;axes<8;++axes) {
        Vec point=Vec::Zero();
        for(int axis=0;axis<3;++axis)if(axes&(1<<axis))point[axis]=coordinate;
        checks.compare(map,{point,size},"supported-domain boundary");
      }
  }
  for(int sign:{-1,1})for(const auto& size:sizes) {
    const Vec point=map.center(Index::Constant(sign*limit));
    checks.compare(map,{point,size},"outermost voxel center");
  }
  checks.expect(map,{Vec::Zero(),Vec::Constant(3.*limit*resolution)},VoxelStatus::Unknown,
                "body exceeds supported domain");
}
void invalidChecks(Checks& checks) {
  SparseVoxelMap map(.1);
  const double inf=std::numeric_limits<double>::infinity(),nan=std::numeric_limits<double>::quiet_NaN();
  for(const Vec& size:std::vector<Vec>{{0.,.1,.1},{.1,0.,.1},{.1,.1,0.},{-.1,.1,.1},
                                      {-0.,0.,-0.},{inf,1.,1.},{1.,nan,1.}})
    checks.compare(map,{Vec::Zero(),size},"invalid or signed-zero size");
  for(int axis=0;axis<3;++axis)for(double value:{-inf,inf,nan}) {
    Vec point=Vec::Zero();point[axis]=value;
    checks.compare(map,{point,Vec::Zero()},"nonfinite position");
  }
}
struct Timing {double seconds;std::int64_t checksum;};
Timing timed(const SparseVoxelMap& map,const std::vector<Query>& queries,int repetitions,bool oracle) {
  std::int64_t checksum=0;const auto start=std::chrono::steady_clock::now();
  for(int repeat=0;repeat<repetitions;++repeat)for(const auto& query:queries)
    checksum+=1+int(oracle?map.path(query.point,query.point,query.size):map.box(query.point,query.size));
  return {std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count(),checksum};
}
void benchmark(Checks& checks,int repetitions) {
  SparseVoxelMap map(.1);fillFree(map,8);std::vector<Query> queries;
  for(int i=0;i<128;++i) {
    const Vec point(.013*((i*17)%47-23),.011*((i*11)%43-21),.009*((i*7)%41-20));
    Vec size(.65,.45,.3);if(i%4==0)size=Vec::Zero();
    queries.push_back({point,size});checks.compare(map,queries.back(),"timing fixture");
  }
  // Warm each implementation equally; compare timings without a pass threshold.
  timed(map,queries,1,true);timed(map,queries,1,false);
  const auto oracle=timed(map,queries,repetitions,true),specialized=timed(map,queries,repetitions,false);
  if(oracle.checksum!=specialized.checksum)throw std::runtime_error("Timing checksums differ");
  std::cout<<",\"benchmark\":{\"queries_per_implementation\":"<<queries.size()*repetitions
    <<",\"path_seconds\":"<<oracle.seconds<<",\"box_seconds\":"<<specialized.seconds
    <<",\"checksum\":"<<specialized.checksum<<'}';
}
}  // namespace
int main(int argc,char** argv) {
  try {
    int repetitions=0;
    if(argc>1) {
      if(std::string(argv[1])!="--benchmark"||argc>3)
        throw std::invalid_argument("Usage: check_sparse_box [--benchmark [repetitions]]");
      repetitions=100;
      if(argc==3) {
        std::size_t consumed=0;const std::string value=argv[2];repetitions=std::stoi(value,&consumed);
        if(consumed!=value.size()||repetitions<=0||repetitions>100000)
          throw std::invalid_argument("Benchmark repetitions must be between 1 and 100000");
      }
    }
    Checks checks;explicitChecks(checks);invalidChecks(checks);
    const std::array<double,8> resolutions{{std::numeric_limits<double>::denorm_min(),
                                           1e-9,.05,.1,.3,1.,1e6,1e300}};
    for(double resolution:resolutions) {boundaryChecks(checks,resolution);domainChecks(checks,resolution);}
    std::cout<<std::setprecision(17)<<"{\"status\":\"passed\",\"oracle\":\"path(p,p,size)\""
      <<",\"compared_queries\":"<<checks.queries<<",\"matching_exceptions\":"<<checks.exceptions
      <<",\"resolutions\":"<<resolutions.size();
    if(repetitions)benchmark(checks,repetitions);
    std::cout<<"}\n";return 0;
  } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
