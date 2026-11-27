// Combined snapshot records must match the existing set()+setTsdf() sequence.
// Standalone: compile with sparse_voxel_map.cpp and Eigen; no ROS runtime.
#include "gbplanner_local/sparse_voxel_map.hpp"
#include <array>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

using namespace gbplanner2;
namespace {
using Vec=Eigen::Vector3d;
using Index=Eigen::Vector3i;
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
bool bits(double a,double b){return std::memcmp(&a,&b,sizeof(double))==0;}
bool same(const std::optional<TsdfDistance>& a,const std::optional<TsdfDistance>& b){
  return a.has_value()==b.has_value()&&(!a||(bits(a->distance,b->distance)&&
      bits(a->weight,b->weight)&&a->interpolated==b->interpolated));
}
struct Pair {
  SparseVoxelMap old_map{.25},new_map{.25};
  std::size_t updates{0},queries{0};
  void compare(const Vec& point){
    require(old_map.size()==new_map.size(),"Combined record changed sparse storage membership");
    require(old_map.voxel(point)==new_map.voxel(point),"Combined record changed occupancy");
    require(same(old_map.tsdfDistance(point),new_map.tsdfDistance(point)),
            "Combined record changed trilinear/fallback distance or weight bits");
    require(same(old_map.tsdfDistanceSimplex(point),new_map.tsdfDistanceSimplex(point)),
            "Combined record changed simplex distance or weight bits");
    ++queries;
  }
  void apply(const Index& index,VoxelStatus status,double distance,double weight){
    old_map.set(index,status);old_map.setTsdf(index,distance,weight);
    new_map.setObservation(index,status,distance,weight);
    ++updates;compare(old_map.center(index));
  }
  void geometry(const Vec& a,const Vec& b,const Vec& size){
    require(old_map.box(a,size)==new_map.box(a,size),"Combined record changed bitmap box verdict");
    require(old_map.path(a,b,size)==new_map.path(a,b,size),"Combined record changed bitmap path verdict");
    queries+=2;
  }
};
template<class Operation>std::string rejected(Operation operation){
  try{operation();}catch(const std::invalid_argument& error){return error.what();}
  throw std::runtime_error("Invalid snapshot record was accepted");
}
struct Record {Index index;VoxelStatus status;double distance,weight;};
}

int main(){try{
  constexpr int limit=SparseVoxelMap::kIndexLimit;
  const double tiny=std::numeric_limits<double>::denorm_min();
  const std::array<double,7> weights{{0.,-0.,tiny,std::nextafter(1e-6,0.),1e-6,.25,200.}};
  const std::array<double,4> distances{{-1.25,-0.,0.,.375}};
  const std::array<VoxelStatus,3> statuses{{VoxelStatus::Unknown,VoxelStatus::Free,VoxelStatus::Occupied}};
  const std::vector<Index> indices{{0,0,0},{-1,-2,-3},{-4,-4,-4},{3,3,3},{4,4,4},
      {-limit,-limit,-limit},{limit,limit,limit},{-limit,limit,0}};
  Pair values;values.old_map.reserve(indices.size());values.new_map.reserve(indices.size());
  for(const auto& index:indices)for(double distance:distances)for(double weight:weights)
    for(auto status:statuses){
      values.apply(index,status,distance,weight);
      const auto center=values.old_map.center(index);
      values.compare(center+Vec(.03125,-.015625,.046875));
      values.geometry(center,center,Vec::Zero());
    }
  require(values.old_map.size()==values.new_map.size(),"Boundary records changed storage count");

  Pair blocks;
  blocks.old_map.reserve(512);blocks.new_map.reserve(512);
  for(int z=-4;z<4;++z)for(int y=-4;y<4;++y)for(int x=-4;x<4;++x)
    blocks.apply({x,y,z},VoxelStatus::Free,.17*x-.23*y+.11*z,1.+.01*(x+4));
  // All eight 4^3 summaries begin fully free. Changes must invalidate and
  // restore the correct bit, including negative block coordinates.
  for(const auto& target:std::vector<Index>{{1,1,1},{-1,-1,-1},{-4,-4,-4},{3,3,3}}){
    const Vec center=blocks.old_map.center(target),small=Vec::Constant(.1);
    for(const auto& record:std::vector<Record>{{target,VoxelStatus::Occupied,-.2,1.},
          {target,VoxelStatus::Free,.2,0.},{target,VoxelStatus::Unknown,.2,1.},
          {target,VoxelStatus::Free,-0.,tiny},{target,VoxelStatus::Unknown,.3,0.},
          {target,VoxelStatus::Free,.1,1.}}){
      blocks.apply(record.index,record.status,record.distance,record.weight);
      blocks.geometry(center,center+Vec(.03,0.,0.),small);
      require(blocks.new_map.box(center,small)==record.status,
              "TSDF-only/missing voxel was incorrectly marked free in its block");
      blocks.geometry({-.6,-.4,-.2},{.6,.4,.2},small);
      blocks.geometry(Vec::Zero(),Vec::Zero(),Vec::Constant(1.5));
      for(const Vec& fraction:std::vector<Vec>{{.75,.5,.25},{.25,.5,.75},{.5,.5,.5}})
        blocks.compare((target.cast<double>()+fraction+Vec::Constant(.5))*.25);
    }
  }
  // A wholly unobserved record must not allocate a cell. A tiny positive
  // weight is retained even though it remains below the query threshold.
  Pair missing;missing.apply({-3,2,1},VoxelStatus::Unknown,.4,-0.);
  require(missing.new_map.size()==0,"Unknown zero-weight record created a stored cell");
  missing.apply({-3,2,1},VoxelStatus::Unknown,.4,tiny);
  require(missing.new_map.size()==1&&!missing.new_map.tsdfDistance(missing.new_map.center({-3,2,1})),
          "Tiny positive observation changed storage/query threshold semantics");
  missing.apply({-3,2,1},VoxelStatus::Unknown,.4,0.);
  require(missing.new_map.size()==0,"Unknown zero-weight replacement retained a cell");

  const double inf=std::numeric_limits<double>::infinity(),nan=std::numeric_limits<double>::quiet_NaN();
  const std::vector<Record> invalid{{{limit+1,0,0},VoxelStatus::Free,.1,1.},
      {{-limit-1,0,0},VoxelStatus::Free,.1,1.},{{0,0,0},static_cast<VoxelStatus>(1),.1,1.},
      {{0,0,0},static_cast<VoxelStatus>(-2),.1,1.},{{0,0,0},VoxelStatus::Occupied,nan,1.},
      {{0,0,0},VoxelStatus::Unknown,inf,1.},{{0,0,0},VoxelStatus::Free,-inf,1.},
      {{0,0,0},VoxelStatus::Occupied,.1,nan},{{0,0,0},VoxelStatus::Unknown,.1,inf},
      {{0,0,0},VoxelStatus::Occupied,.1,-1.},{{0,0,0},VoxelStatus::Unknown,.1,-tiny}};
  std::size_t rejections=0;
  for(const auto& record:invalid){
    SparseVoxelMap reference(.25),combined(.25);
    reference.set({0,0,0},VoxelStatus::Free);reference.setTsdf({0,0,0},.2,2.);
    combined.setObservation({0,0,0},VoxelStatus::Free,.2,2.);
    const auto center=combined.center({0,0,0});
    const auto before=combined.tsdfDistance(center);
    const auto expected=rejected([&]{reference.set(record.index,record.status);
      reference.setTsdf(record.index,record.distance,record.weight);});
    const auto actual=rejected([&]{combined.setObservation(record.index,record.status,record.distance,record.weight);});
    require(actual==expected,"Combined import changed an invalid-record rejection");
    // The importer discards a malformed snapshot in either path. The combined
    // setter validates the whole record before mutating its private builder.
    require(combined.size()==1&&combined.voxel(center)==VoxelStatus::Free&&
            same(before,combined.tsdfDistance(center))&&
            combined.box(center,Vec::Constant(.1))==VoxelStatus::Free,
            "Rejected combined record partially changed its builder");
    ++rejections;
  }
  std::cout<<"{\"status\":\"passed\",\"updates\":"<<values.updates+blocks.updates+missing.updates
      <<",\"query_comparisons\":"<<values.queries+blocks.queries+missing.queries
      <<",\"invalid_records_rejected\":"<<rejections
      <<",\"bit_exact_tsdf\":true,\"sparse_membership_equal\":true,\"bitmap_queries_equal\":true}\n";
  return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
