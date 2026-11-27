// Focused occupancy-summary checks. Binary resolutions keep dense/sparse
// geometry identical; random queries stay away from the dense map boundary.
/* Standalone build (Eigen headers required):
c++ -std=c++17 -O3 -I src/gbplanner_local/include -I /usr/include/eigen3 \
  tools/check_sparse_free_blocks.cpp src/gbplanner_local/src/sparse_voxel_map.cpp \
  src/gbplanner_local/src/voxel_map.cpp -o /tmp/check_sparse_free_blocks
*/
#include "gbplanner_local/sparse_voxel_map.hpp"
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace gbplanner2;
namespace {
using Vec=Eigen::Vector3d;
using Index=Eigen::Vector3i;
void require(bool value,const char* message) {if(!value)throw std::runtime_error(message);}
struct Fixture {
  Fixture(double resolution,Index lower=Index::Constant(-12),Index dimensions=Index::Constant(24))
      :lower(lower),dimensions(dimensions),sparse(resolution),
       dense(resolution,lower.cast<double>()*resolution,dimensions) {
    sparse.reserve(static_cast<std::size_t>(dimensions.prod()));
    for(int z=0;z<dimensions.z();++z)for(int y=0;y<dimensions.y();++y)
      for(int x=0;x<dimensions.x();++x)set(lower+Index(x,y,z),VoxelStatus::Free);
  }
  void set(const Index& index,VoxelStatus status) {
    sparse.set(index,status);dense.set(index-lower,status);
  }
  void tsdf(const Index& index,double weight) {
    sparse.setTsdf(index,.25,weight);dense.setTsdf(index-lower,.25,weight);
  }
  Index lower,dimensions;
  SparseVoxelMap sparse;
  VoxelMap dense;
};
class Checks {
 public:
  VoxelStatus box(const Fixture& fixture,const Vec& point,const Vec& size,const char* label) {
    const auto sparse=fixture.sparse.box(point,size),dense=fixture.dense.box(point,size);
    same(sparse,dense,point,point,size,label);++boxes;return sparse;
  }
  VoxelStatus path(const Fixture& fixture,const Vec& a,const Vec& b,const Vec& size,const char* label) {
    const auto sparse=fixture.sparse.path(a,b,size),dense=fixture.dense.path(a,b,size);
    same(sparse,dense,a,b,size,label);++paths;return sparse;
  }
  std::uint64_t boxes{0},paths{0};
 private:
  void same(VoxelStatus sparse,VoxelStatus dense,const Vec& a,const Vec& b,const Vec& size,const char* label) {
    if(sparse==dense)return;
    std::ostringstream error;error<<std::setprecision(17)<<label<<": sparse/dense mismatch "
      <<int(sparse)<<'/'<<int(dense)<<", a="<<a.transpose()<<", b="<<b.transpose()
      <<", size="<<size.transpose();throw std::runtime_error(error.str());
  }
};
void blockAndRow(Checks& checks,const Fixture& fixture,const Index& base,const Index& changed,
                 VoxelStatus expected) {
  const double resolution=fixture.sparse.resolution();
  const Vec center=(base.cast<double>()+Vec::Constant(2.))*resolution;
  require(checks.box(fixture,center,Vec::Constant(3.75*resolution),"whole block")==expected,
          "Whole-block state did not follow mutation");
  Vec a=fixture.sparse.center(changed),b=a;
  a.x()=(base.x()+.25)*resolution;b.x()=(base.x()+3.75)*resolution;
  require(checks.path(fixture,a,b,Vec::Constant(.125*resolution),"mutated row")==expected,
          "Path state did not follow mutation");
}
void mutationChecks(Checks& checks) {
  Fixture fixture(.25);
  for(const Index& base:{Index(-4,-4,-4),Index(0,0,0),Index(-8,4,-4)}) {
    for(int z=0;z<4;++z)for(int y=0;y<4;++y)for(int x=0;x<4;++x) {
      const Index index=base+Index(x,y,z);
      blockAndRow(checks,fixture,base,index,VoxelStatus::Free);
      fixture.set(index,VoxelStatus::Occupied);
      blockAndRow(checks,fixture,base,index,VoxelStatus::Occupied);
      fixture.set(index,VoxelStatus::Free);
      blockAndRow(checks,fixture,base,index,VoxelStatus::Free);
      fixture.tsdf(index,1.);fixture.set(index,VoxelStatus::Unknown);
      blockAndRow(checks,fixture,base,index,VoxelStatus::Unknown);
      fixture.set(index,VoxelStatus::Free);fixture.tsdf(index,0.);
      fixture.set(index,VoxelStatus::Unknown);  // Erase, rather than retain TSDF.
      blockAndRow(checks,fixture,base,index,VoxelStatus::Unknown);
      fixture.set(index,VoxelStatus::Free);
      blockAndRow(checks,fixture,base,index,VoxelStatus::Free);
    }
  }
  const Index base(-4,0,4);
  // Clearing every free bit removes the summary. Recreated TSDF-only cells
  // must still leave the whole block unknown until all 64 labels become Free.
  for(int z=0;z<4;++z)for(int y=0;y<4;++y)for(int x=0;x<4;++x) {
    const Index index=base+Index(x,y,z);fixture.set(index,VoxelStatus::Unknown);fixture.tsdf(index,1.);
  }
  blockAndRow(checks,fixture,base,base,VoxelStatus::Unknown);
  for(int z=0;z<4;++z)for(int y=0;y<4;++y)for(int x=0;x<4;++x) {
    if(x==3&&y==3&&z==3)continue;
    fixture.set(base+Index(x,y,z),VoxelStatus::Free);
  }
  const Index last=base+Index::Constant(3);
  blockAndRow(checks,fixture,base,last,VoxelStatus::Unknown);
  fixture.tsdf(last,0.);blockAndRow(checks,fixture,base,last,VoxelStatus::Unknown);
  fixture.set(last,VoxelStatus::Free);blockAndRow(checks,fixture,base,last,VoxelStatus::Free);
}
void copyChecks(Checks& checks) {
  Fixture original(1.);Fixture copied=original;
  const Index base(0,0,0),first(1,1,1),second(2,2,2);
  copied.set(first,VoxelStatus::Occupied);
  blockAndRow(checks,original,base,first,VoxelStatus::Free);
  blockAndRow(checks,copied,base,first,VoxelStatus::Occupied);
  original.set(second,VoxelStatus::Unknown);
  blockAndRow(checks,original,base,second,VoxelStatus::Unknown);
  copied.set(first,VoxelStatus::Free);
  blockAndRow(checks,copied,base,second,VoxelStatus::Free);
  Fixture assigned(.25);assigned=original;
  assigned.set(second,VoxelStatus::Free);
  blockAndRow(checks,assigned,base,second,VoxelStatus::Free);
  blockAndRow(checks,original,base,second,VoxelStatus::Unknown);
  original.set(second,VoxelStatus::Occupied);
  blockAndRow(checks,assigned,base,second,VoxelStatus::Free);
  blockAndRow(checks,original,base,second,VoxelStatus::Occupied);
}
void crossingChecks(Checks& checks) {
  Fixture fixture(1.);
  const Vec a(-7.5,-3.5,.5),b(7.5,-3.5,.5),body=Vec::Constant(.25);
  require(checks.path(fixture,a,b,body,"crossing full blocks")==VoxelStatus::Free,
          "Full free blocks rejected a path");
  fixture.set({-5,-4,0},VoxelStatus::Unknown);fixture.set({4,-4,0},VoxelStatus::Occupied);
  require(checks.path(fixture,a,b,body,"unknown before occupied")==VoxelStatus::Occupied,
          "Unknown masked a later occupied block");
  require(checks.path(fixture,b,a,body,"reverse block crossing")==VoxelStatus::Occupied,
          "Block order changed occupied priority");
  fixture.set({4,-4,0},VoxelStatus::Free);
  require(checks.path(fixture,a,b,body,"unknown in partial block")==VoxelStatus::Unknown,
          "Partial block hid an unknown cell");
  fixture.set({-5,-4,0},VoxelStatus::Free);
  const double inf=std::numeric_limits<double>::infinity();
  for(int boundary:{-8,-4,0,4,8}) {
    const Index obstacle(boundary-1,0,0);fixture.set(obstacle,VoxelStatus::Occupied);
    for(double x:{std::nextafter(double(boundary),-inf),double(boundary),
                  std::nextafter(double(boundary),inf)}) {
      checks.box(fixture,{x,.5,.5},Vec::Zero(),"block-face nextafter");
      checks.path(fixture,{x,.25,.25},{x,.75,.75},Vec::Zero(),"path along block face");
    }
    fixture.set(obstacle,VoxelStatus::Free);
  }
}
void randomChecks(Checks& checks) {
  std::mt19937 random(6971);
  std::uniform_int_distribution<int> cell(-9,9),coordinate(-31,31);
  for(double resolution:{.25,1.}) {
    Fixture fixture(resolution);
    for(int mode=0;mode<2;++mode) {
      if(mode)for(int n=0;n<96;++n) {
        const Index index(cell(random),cell(random),cell(random));
        fixture.set(index,n%3?VoxelStatus::Unknown:VoxelStatus::Occupied);
        if(n%2)fixture.tsdf(index,1.);
      }
      for(int n=0;n<250;++n) {
        const Vec a=Vec(coordinate(random),coordinate(random),coordinate(random))*(.25*resolution);
        const Vec b=Vec(coordinate(random),coordinate(random),coordinate(random))*(.25*resolution);
        Vec size=Vec::Zero();
        if(n%3==1)size=Vec(.25,.75,1.25)*resolution;
        if(n%3==2)size=Vec(2.25,1.75,.5)*resolution;
        checks.box(fixture,a,size,"random box");checks.path(fixture,a,b,size,"random path");
        checks.path(fixture,b,a,size,"random reverse path");
      }
    }
  }
}
void domainChecks(Checks& checks) {
  constexpr int limit=SparseVoxelMap::kIndexLimit;
  Fixture upper(1.,Index(limit-7,-8,-8),Index(8,16,16));
  const Vec residual_center(limit+.5,2.,2.),residual_size(.5,3.75,3.75);
  require(checks.box(upper,residual_center,residual_size,"positive residual block")==VoxelStatus::Free,
          "Valid cells in a domain residual block were lost");
  for(int z=0;z<4;++z)for(int y=0;y<4;++y) {
    const Index index(limit,y,z);Vec a=upper.sparse.center(index),b=a;a.x()-=6.;
    upper.set(index,VoxelStatus::Unknown);upper.tsdf(index,1.);
    require(checks.box(upper,residual_center,residual_size,"residual unknown")==VoxelStatus::Unknown,
            "Residual block hid an unknown cell");
    require(checks.path(upper,a,b,Vec::Constant(.25),"residual path unknown")==VoxelStatus::Unknown,
            "Residual path hid an unknown cell");
    upper.set(index,VoxelStatus::Occupied);
    require(checks.box(upper,residual_center,residual_size,"residual occupied")==VoxelStatus::Occupied,
            "Residual block hid an occupied cell");
    upper.set(index,VoxelStatus::Free);upper.tsdf(index,0.);
  }
  bool rejected=false;
  try {upper.sparse.set({limit+1,0,0},VoxelStatus::Free);}catch(const std::invalid_argument&){rejected=true;}
  require(rejected,"Out-of-domain set was accepted");
  require(checks.box(upper,residual_center,residual_size,"after invalid set")==VoxelStatus::Free,
          "Invalid set corrupted boundary occupancy");
  require(checks.box(upper,{limit+1.,.5,.5},Vec::Zero(),"upper domain edge")==VoxelStatus::Unknown,
          "Upper domain edge became free");
  require(checks.box(upper,{limit+.5,.5,.5},Vec::Ones(),"body reaches upper edge")==VoxelStatus::Unknown,
          "Full block bypassed the body domain check");

  Fixture lower(1.,Index(-limit,-8,-8),Index(8,16,16));
  const Index index(-limit,0,0);const Vec center=lower.sparse.center(index);
  require(checks.box(lower,center,Vec::Ones(),"body reaches lower edge")==VoxelStatus::Free,
          "Lower inclusive domain boundary changed");
  lower.set(index,VoxelStatus::Occupied);
  require(checks.box(lower,center,Vec::Constant(.5),"negative domain cell")==VoxelStatus::Occupied,
          "Negative block indexing lost the boundary cell");
  require(checks.path(lower,center,center+Vec(6.,0.,0.),Vec::Constant(.25),"negative domain path")
          ==VoxelStatus::Occupied,"Negative boundary path lost occupancy");
  lower.set(index,VoxelStatus::Unknown);lower.tsdf(index,1.);
  require(checks.box(lower,center,Vec::Constant(.5),"negative domain TSDF-only")==VoxelStatus::Unknown,
          "Negative boundary TSDF observation became free");
}
}  // namespace
int main() {
  try {
    Checks checks;mutationChecks(checks);copyChecks(checks);crossingChecks(checks);
    randomChecks(checks);domainChecks(checks);
    std::cout<<"{\"status\":\"passed\",\"dense_box_comparisons\":"<<checks.boxes
      <<",\"dense_path_comparisons\":"<<checks.paths
      <<",\"all_64_bit_mutations\":true,\"tsdf_only_unknown\":true"
      <<",\"copy_isolation\":true,\"negative_and_residual_blocks\":true}\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
