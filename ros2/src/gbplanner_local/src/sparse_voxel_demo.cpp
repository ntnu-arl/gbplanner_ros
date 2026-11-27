#include "gbplanner_local/sparse_voxel_map.hpp"
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>

using namespace gbplanner2;
namespace {
void require(bool condition,const char* message) {
  if(!condition)throw std::runtime_error(message);
}
void sameDistance(const MapQuery& a,const MapQuery& b,const Eigen::Vector3d& p) {
  const auto first=a.tsdfDistance(p),second=b.tsdfDistance(p);
  require(bool(first)==bool(second),"Sparse/dense TSDF observation mismatch");
  if(first)require(std::abs(first->distance-second->distance)<1e-10&&
      std::abs(first->weight-second->weight)<1e-10&&first->interpolated==second->interpolated,
      "Sparse/dense TSDF interpolation mismatch");
}
}  // namespace

int main() {
  try {
    constexpr double resolution=.5;
    VoxelMap dense(resolution,Eigen::Vector3d::Constant(-4.),Eigen::Vector3i::Constant(16));
    SparseVoxelMap sparse(resolution);
    for(int z=-8;z<8;++z)for(int y=-8;y<8;++y)for(int x=-8;x<8;++x) {
      const Eigen::Vector3i i(x,y,z),dense_i=i+Eigen::Vector3i::Constant(8);
      auto status=VoxelStatus::Free;
      if(x==1&&y>=0&&z<=2)status=VoxelStatus::Occupied;
      else if(x<=-2&&y==1)status=VoxelStatus::Unknown;
      dense.set(dense_i,status);sparse.set(i,status);
      const Eigen::Vector3d p=sparse.center(i);
      const double distance=2.*p.x()-3.*p.y()+p.z()+.25;
      const double weight=(x==-2&&y==1)?0.:2.;
      dense.setTsdf(dense_i,distance,weight);sparse.setTsdf(i,distance,weight);
    }
    std::mt19937 random(41);std::uniform_real_distribution<double> coordinate(-2.5,2.5);
    constexpr int queries=160;
    for(int n=0;n<queries;++n) {
      const Eigen::Vector3d a(coordinate(random),coordinate(random),coordinate(random));
      const Eigen::Vector3d b(coordinate(random),coordinate(random),coordinate(random));
      Eigen::Vector3d size=Eigen::Vector3d::Zero();
      if(n%3==1)size={.2,.3,.4};
      else if(n%3==2)size={.7,.9,.6};
      require(dense.voxel(a)==sparse.voxel(a),"Sparse/dense voxel mismatch");
      require(dense.box(a,size)==sparse.box(a,size),"Sparse/dense body mismatch");
      require(dense.path(a,b,size)==sparse.path(a,b,size),"Sparse/dense swept path mismatch");
      require(dense.path(b,a,size)==sparse.path(b,a,size),"Sparse/dense reverse path mismatch");
      sameDistance(dense,sparse,a);
    }
    // A TSDF observation does not turn an unknown occupancy cell into free.
    const Eigen::Vector3i missing(-2,1,0);
    require(sparse.voxel(sparse.center(missing))==VoxelStatus::Unknown,
            "Unobserved occupancy became free");
    require(!sparse.tsdfDistance(sparse.center(missing)),"Missing TSDF manufactured a distance");
    sameDistance(dense,sparse,sparse.center(missing+Eigen::Vector3i(0,-1,0)));

    SparseVoxelMap contacts(1.);
    for(int z=-2;z<=2;++z)for(int y=-2;y<=2;++y)for(int x=-2;x<=2;++x)
      contacts.set(Eigen::Vector3i(x,y,z),VoxelStatus::Free);
    const Eigen::Vector3d point_size=Eigen::Vector3d::Zero();
    contacts.set(Eigen::Vector3i(0,-1,0),VoxelStatus::Occupied);
    require(contacts.path({.2,0.,.5},{.8,0.,.5},point_size)==VoxelStatus::Occupied,
            "Point path missed touched voxel face");
    contacts.set(Eigen::Vector3i(0,-1,0),VoxelStatus::Free);
    contacts.set(Eigen::Vector3i(0,-1,-1),VoxelStatus::Occupied);
    require(contacts.path({.2,0.,0.},{.8,0.,0.},point_size)==VoxelStatus::Occupied,
            "Point path missed touched voxel edge");
    contacts.set(Eigen::Vector3i(0,-1,-1),VoxelStatus::Free);
    contacts.set(Eigen::Vector3i(-1,0,0),VoxelStatus::Occupied);
    require(contacts.path({-.5,-.5,-.5},{.5,.5,.5},point_size)==VoxelStatus::Occupied,
            "Point path missed touched voxel corner");

    SparseVoxelMap far(.25);
    const Eigen::Vector3i low=Eigen::Vector3i::Constant(-SparseVoxelMap::kIndexLimit);
    const Eigen::Vector3i high=Eigen::Vector3i::Constant(SparseVoxelMap::kIndexLimit);
    far.set(low,VoxelStatus::Free);far.set(high,VoxelStatus::Occupied);
    require(far.voxel(far.center(low))==VoxelStatus::Free&&
            far.voxel(far.center(high))==VoxelStatus::Occupied,
            "Distant signed lattice coordinates were clipped");
    require((far.index(Eigen::Vector3d::Constant(-.01)).array()==-1).all(),
            "Negative world coordinates did not use floor indexing");
    require(far.voxel(Eigen::Vector3d::Zero())==VoxelStatus::Unknown&&far.size()==2,
            "Sparse gap was allocated or marked free");
    for(int z=0;z<2;++z)for(int y=0;y<2;++y)for(int x=0;x<2;++x) {
      const Eigen::Vector3i delta(x,y,z);const double distance=1.+x+2.*y+4.*z;
      far.setTsdf(low+delta,distance,3.);
      far.setTsdf(high-Eigen::Vector3i::Ones()+delta,distance,3.);
    }
    const Eigen::Vector3d negative_interpolation=(low.cast<double>()+Eigen::Vector3d::Ones())*far.resolution();
    const Eigen::Vector3d positive_interpolation=high.cast<double>()*far.resolution();
    for(const auto& p:{negative_interpolation,positive_interpolation}) {
      const auto value=far.tsdfDistance(p);
      require(value&&value->interpolated&&std::abs(value->distance-4.5)<1e-10,
              "Distant TSDF interpolation failed");
    }
    const auto boundary=far.tsdfDistance(far.center(high));
    require(boundary&&!boundary->interpolated&&boundary->distance==8.,
            "TSDF domain boundary did not use containing-voxel fallback");
    far.setTsdf(high,0.,0.);
    require(!far.tsdfDistance(far.center(high))&&far.voxel(far.center(high))==VoxelStatus::Occupied,
            "Clearing TSDF changed occupancy or retained a distance");

    // A 5 km diagonal crosses only O(length) candidates despite its large
    // enclosing cube. An unknown gap must not mask a later occupied cell.
    SparseVoxelMap diagonal(1.);
    diagonal.set(Eigen::Vector3i::Constant(4500),VoxelStatus::Occupied);
    require(diagonal.path({.5,.5,.5},{5000.5,5000.5,5000.5},{.2,.2,.2})==VoxelStatus::Occupied,
            "Long sparse diagonal skipped an occupied cell");
    diagonal.set(Eigen::Vector3i::Constant(4500),VoxelStatus::Unknown);
    require(diagonal.path({.5,.5,.5},{5000.5,5000.5,5000.5},{.2,.2,.2})==VoxelStatus::Unknown,
            "Long sparse diagonal invented free space");
    std::size_t ray_cells=0;
    far.ray(far.center(low),far.center(low+Eigen::Vector3i(5,3,2)),
            [&](const Eigen::Vector3d&,VoxelStatus){++ray_cells;return true;});
    require(ray_cells==11,"Sparse DDA did not reach its endpoint");
    std::cout<<"Sparse voxel demo passed: "<<queries
             <<" dense comparisons, exact contacts, signed distant TSDF, 5 km diagonal.\n";
    return 0;
  } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
