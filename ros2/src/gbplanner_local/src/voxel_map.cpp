#include "gbplanner_local/voxel_map.hpp"
#include "gbplanner_local/observed_tsdf_simplex.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace gbplanner2 {
namespace {
bool intersects(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
                const Eigen::Vector3d& lo,const Eigen::Vector3d& hi,
                double parallel_tolerance=1e-14) {
  double tmin=0,tmax=1;
  for(int k=0;k<3;++k) {
    double delta=b[k]-a[k];
    if(delta==0.||std::abs(delta)<parallel_tolerance) { if(a[k]<lo[k] || a[k]>hi[k]) return false; }
    else {
      double t0=(lo[k]-a[k])/delta,t1=(hi[k]-a[k])/delta;
      if(t0>t1) std::swap(t0,t1);
      tmin=std::max(tmin,t0);tmax=std::min(tmax,t1);
      if(tmin>tmax) return false;
    }
  }
  return true;
}
bool validStatus(VoxelStatus s) {
  return s==VoxelStatus::Free || s==VoxelStatus::Occupied || s==VoxelStatus::Unknown;
}
}
VoxelMap::VoxelMap(double r,const Eigen::Vector3d& o,const Eigen::Vector3i& d)
 :resolution_(r),origin_(o),dimensions_(d) {
  if(!std::isfinite(r)||r<=0||!o.allFinite()||(d.array()<=0).any()||
     (d.array()>10000).any() || d.cast<int64_t>().prod()>4000000)
    throw std::invalid_argument("Invalid voxel map geometry or excessive cell count");
  if(!maximum().allFinite()) throw std::invalid_argument("Map extent overflow");
  cells_.assign(static_cast<std::size_t>(d.cast<int64_t>().prod()),VoxelStatus::Unknown);
}
bool VoxelMap::inside(const Eigen::Vector3i& i) const {
  return (i.array()>=0).all() && (i.array()<dimensions_.array()).all();
}
Eigen::Vector3i VoxelMap::index(const Eigen::Vector3d& p) const {
  if(!p.allFinite()) throw std::invalid_argument("Nonfinite map coordinate");
  const Eigen::Array3d q=((p-origin_)/resolution_).array().floor();
  if((q.abs()>1000000000).any()) throw std::out_of_range("Map coordinate overflow");
  return q.cast<int>().matrix();
}
std::size_t VoxelMap::offset(const Eigen::Vector3i& i) const {
  return static_cast<std::size_t>(i.x())+static_cast<std::size_t>(dimensions_.x())*
    (static_cast<std::size_t>(i.y())+static_cast<std::size_t>(dimensions_.y())*i.z());
}
Eigen::Vector3d VoxelMap::center(const Eigen::Vector3i& i) const {
  return origin_+(i.cast<double>()+Eigen::Vector3d::Constant(0.5))*resolution_;
}
VoxelStatus VoxelMap::voxel(const Eigen::Vector3d& p) const {
  if(!p.allFinite()||(p.array()<minimum().array()).any()||(p.array()>=maximum().array()).any())
    return VoxelStatus::Unknown;
  return cells_[offset(index(p))];
}
std::optional<TsdfDistance> VoxelMap::tsdfDistance(const Eigen::Vector3d& p) const {
  if(tsdf_weights_.empty()||!p.allFinite()||
     (p.array()<minimum().array()).any()||(p.array()>=maximum().array()).any())return std::nullopt;
  // Voxblox interpolates around voxel centers, not their lower corners. All
  // eight neighboring samples must be observed, even at a zero coefficient.
  const Eigen::Vector3d grid=(p-origin_)/resolution_-Eigen::Vector3d::Constant(.5);
  const Eigen::Vector3i lower=grid.array().floor().cast<int>();
  const Eigen::Vector3d fraction=grid-lower.cast<double>();
  double distance=0.,weight=0.;bool complete=true;
  for(int z=0;z<2&&complete;++z)for(int y=0;y<2&&complete;++y)for(int x=0;x<2;++x){
    const Eigen::Vector3i cell=lower+Eigen::Vector3i(x,y,z);
    if(!inside(cell)){complete=false;break;}
    const auto slot=offset(cell);
    if(tsdf_weights_[slot]<1e-6){complete=false;break;}
    const double coefficient=(x?fraction.x():1.-fraction.x())*
        (y?fraction.y():1.-fraction.y())*(z?fraction.z():1.-fraction.z());
    distance+=coefficient*tsdf_distances_[slot];weight+=coefficient*tsdf_weights_[slot];
  }
  if(complete)return TsdfDistance{distance,weight,true};
  // Match getPointDistance's fallback to the containing voxel when an
  // interpolation neighborhood is incomplete; no occupancy-derived estimate.
  const auto slot=offset(index(p));
  if(tsdf_weights_[slot]<1e-6)return std::nullopt;
  return TsdfDistance{tsdf_distances_[slot],tsdf_weights_[slot],false};
}
std::optional<TsdfDistance> VoxelMap::tsdfDistanceSimplex(const Eigen::Vector3d& p) const {
  if(tsdf_weights_.empty()||!p.allFinite()||
     (p.array()<minimum().array()).any()||(p.array()>=maximum().array()).any())return std::nullopt;
  const Eigen::Vector3d grid=(p-origin_)/resolution_-Eigen::Vector3d::Constant(.5);
  const Eigen::Vector3i lower=grid.array().floor().cast<int>();
  return detail::observedTsdfSimplex(grid-lower.cast<double>(),
      [&](const Eigen::Vector3i& vertex)->std::optional<TsdfDistance>{
        const Eigen::Vector3i cell=lower+vertex;if(!inside(cell))return std::nullopt;
        const auto slot=offset(cell);
        return TsdfDistance{tsdf_distances_[slot],tsdf_weights_[slot],false};
      });
}
void VoxelMap::setTsdf(const Eigen::Vector3i& i,double distance,double weight){
  if(!inside(i)||!std::isfinite(distance)||!std::isfinite(weight)||weight<0.)
    throw std::invalid_argument("Invalid TSDF observation");
  if(tsdf_weights_.empty()){
    tsdf_distances_.assign(cells_.size(),0.);tsdf_weights_.assign(cells_.size(),0.);
  }
  const auto slot=offset(i);tsdf_distances_[slot]=distance;tsdf_weights_[slot]=weight;
}
void VoxelMap::set(const Eigen::Vector3i& i,VoxelStatus s) {
  if(!inside(i)||!validStatus(s)) throw std::invalid_argument("Invalid voxel index/status");
  cells_[offset(i)]=s;
}
void VoxelMap::fill(VoxelStatus s) {
  if(!validStatus(s)) throw std::invalid_argument("Invalid voxel status");
  std::fill(cells_.begin(),cells_.end(),s);
}
VoxelStatus VoxelMap::box(const Eigen::Vector3d& p,const Eigen::Vector3d& s) const {
  return path(p,p,s);
}
VoxelStatus VoxelMap::path(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
                          const Eigen::Vector3d& size) const {
  const bool point=(size.array()==0.).all();
  if(!a.allFinite()||!b.allFinite()||!size.allFinite()||
     (!point&&(size.array()<=0.).any()))
    throw std::invalid_argument("Invalid swept-box query");
  if(point)return pointPath(a,b);
  Eigen::Vector3d half=size*0.5;
  Eigen::Vector3d lo=a.cwiseMin(b)-half,hi=a.cwiseMax(b)+half;
  if((lo.array()<minimum().array()).any()||(hi.array()>=maximum().array()).any())
    return VoxelStatus::Unknown;
  // Exact segment vs expanded voxel AABB: no sampling step can skip a thin wall.
  Eigen::Vector3i imin=index(lo-Eigen::Vector3d::Constant(1e-10)).cwiseMax(Eigen::Vector3i::Zero());
  Eigen::Vector3i imax=index(hi).cwiseMin(dimensions_-Eigen::Vector3i::Ones());
  bool unknown=false;
  for(int z=imin.z();z<=imax.z();++z)for(int y=imin.y();y<=imax.y();++y)for(int x=imin.x();x<=imax.x();++x){
    Eigen::Vector3i i(x,y,z); auto s=cells_[offset(i)];
    if(s==VoxelStatus::Free) continue;
    Eigen::Vector3d cell_lo=origin_+i.cast<double>()*resolution_-half;
    Eigen::Vector3d cell_hi=cell_lo+Eigen::Vector3d::Constant(resolution_)+size;
    if(intersects(a,b,cell_lo,cell_hi)) {
      if(s==VoxelStatus::Occupied) return s;
      unknown=true;
    }
  }
  return unknown?VoxelStatus::Unknown:VoxelStatus::Free;
}
VoxelStatus VoxelMap::pointPath(const Eigen::Vector3d& a,const Eigen::Vector3d& b) const {
  if((a.array()<minimum().array()).any()||(a.array()>=maximum().array()).any()||
     (b.array()<minimum().array()).any()||(b.array()>=maximum().array()).any())
    return VoxelStatus::Unknown;
  bool unknown=false,occupied=false;
  // DDA visits every interior-crossed cell without distance sampling. Its
  // single-axis tie ordering omits some cells touched only along a face,
  // edge or corner, so inspect neighboring cells with an exact segment/AABB
  // intersection. The segment itself is never widened into a tiny box.
  ray(a,b,[&](const Eigen::Vector3d& center,VoxelStatus){
    const Eigen::Vector3i traversed=index(center);
    for(int z=-1;z<=1;++z)for(int y=-1;y<=1;++y)for(int x=-1;x<=1;++x){
      const Eigen::Vector3i cell=traversed+Eigen::Vector3i(x,y,z);
      if(!inside(cell))continue;
      const auto status=cells_[offset(cell)];
      if(status==VoxelStatus::Free)continue;
      const Eigen::Vector3d low=origin_+cell.cast<double>()*resolution_;
      const Eigen::Vector3d high=low+Eigen::Vector3d::Constant(resolution_);
      if(!intersects(a,b,low,high,0.))continue;
      if(status==VoxelStatus::Occupied){occupied=true;return false;}
      unknown=true;
    }
    return true;
  });
  return occupied?VoxelStatus::Occupied:unknown?VoxelStatus::Unknown:VoxelStatus::Free;
}
void VoxelMap::ray(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
                  const std::function<bool(const Eigen::Vector3d&,VoxelStatus)>& visit) const {
  if(!a.allFinite()||!b.allFinite()) throw std::invalid_argument("Invalid ray");
  auto i=index(a); const auto last=index(b);
  Eigen::Vector3d delta=b-a,tmax,tdelta;Eigen::Vector3i step;
  for(int k=0;k<3;++k) {
    step[k]=(delta[k]>0)-(delta[k]<0);
    if(step[k]==0) tmax[k]=tdelta[k]=std::numeric_limits<double>::infinity();
    else {
      double boundary=origin_[k]+(i[k]+(step[k]>0?1:0))*resolution_;
      tmax[k]=(boundary-a[k])/delta[k]; tdelta[k]=resolution_/std::abs(delta[k]);
    }
  }
  const int64_t limit=(last.cast<int64_t>()-i.cast<int64_t>()).cwiseAbs().sum()+4;
  if(limit>100000) throw std::invalid_argument("Ray exceeds traversal limit");
  for(int64_t n=0;n<limit;++n) {
    auto s=inside(i)?cells_[offset(i)]:VoxelStatus::Unknown;
    if(!visit(center(i),s)||i==last) break;
    Eigen::Index axis;tmax.minCoeff(&axis);
    if(tmax[axis]>1.0+1e-10) break;
    i[axis]+=step[axis]; tmax[axis]+=tdelta[axis];
  }
}
}  // namespace gbplanner2
