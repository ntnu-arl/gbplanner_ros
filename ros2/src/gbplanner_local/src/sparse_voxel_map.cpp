#include "gbplanner_local/sparse_voxel_map.hpp"
#include "gbplanner_local/observed_tsdf_simplex.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace gbplanner2 {
namespace {
bool intersects(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
                const Eigen::Vector3d& lo,const Eigen::Vector3d& hi,bool point) {
  double tmin=0.,tmax=1.;
  for(int k=0;k<3;++k) {
    const double delta=b[k]-a[k];
    if(delta==0.||(!point&&std::abs(delta)<1e-14)) {
      if(a[k]<lo[k]||a[k]>hi[k])return false;
    } else {
      double t0=(lo[k]-a[k])/delta,t1=(hi[k]-a[k])/delta;
      if(t0>t1)std::swap(t0,t1);
      tmin=std::max(tmin,t0);tmax=std::min(tmax,t1);
      if(tmin>tmax)return false;
    }
  }
  return true;
}
bool validStatus(VoxelStatus status) {
  return status==VoxelStatus::Unknown||status==VoxelStatus::Free||status==VoxelStatus::Occupied;
}
int floorDiv4(int value) {
  // C++ truncates negative division towards zero; blocks require floor.
  return value/4-(value%4<0);
}
Eigen::Vector3i freeBlockIndex(const Eigen::Vector3i& index) {
  return {floorDiv4(index.x()),floorDiv4(index.y()),floorDiv4(index.z())};
}
VoxelIndexSet::Key tableKey(const Eigen::Vector3i& index) {
  return {index.x(),index.y(),index.z()};
}
}  // namespace

SparseVoxelMap::SparseVoxelMap(double resolution):resolution_(resolution) {
  if(!std::isfinite(resolution_)||resolution_<=0.||!maximum().allFinite())
    throw std::invalid_argument("Invalid sparse voxel resolution");
}
Eigen::Vector3d SparseVoxelMap::minimum() const {
  return Eigen::Vector3d::Constant(-kIndexLimit*resolution_);
}
Eigen::Vector3d SparseVoxelMap::maximum() const {
  return Eigen::Vector3d::Constant((kIndexLimit+1.)*resolution_);
}
bool SparseVoxelMap::inside(const Eigen::Vector3i& i) const {
  return (i.array()>=-kIndexLimit).all()&&(i.array()<=kIndexLimit).all();
}
bool SparseVoxelMap::contains(const Eigen::Vector3d& p) const {
  return p.allFinite()&&(p.array()>=minimum().array()).all()&&
      (p.array()<maximum().array()).all();
}
Eigen::Vector3i SparseVoxelMap::index(const Eigen::Vector3d& p) const {
  if(!p.allFinite())throw std::invalid_argument("Nonfinite sparse map coordinate");
  const Eigen::Array3d grid=(p/resolution_).array().floor();
  if((grid<-kIndexLimit).any()||(grid>kIndexLimit).any())
    throw std::out_of_range("Sparse map coordinate exceeds supported index domain");
  return grid.cast<int>().matrix();
}
Eigen::Vector3d SparseVoxelMap::center(const Eigen::Vector3i& i) const {
  if(!inside(i))throw std::out_of_range("Invalid sparse voxel index");
  return (i.cast<double>()+Eigen::Vector3d::Constant(.5))*resolution_;
}
const SparseVoxelMap::Cell* SparseVoxelMap::cell(const Eigen::Vector3i& i) const {
  return cells_.find(tableKey(i));
}
void SparseVoxelMap::updateFreeBlock(const Eigen::Vector3i& i,VoxelStatus status) {
  const Eigen::Vector3i block=freeBlockIndex(i),local=i-4*block;
  const auto bit=std::uint64_t{1}<<(local.x()+4*local.y()+16*local.z());
  if(status==VoxelStatus::Free)free_blocks_[tableKey(block)]|=bit;
  else {
    auto* found=free_blocks_.find(tableKey(block));
    if(!found)return;
    *found&=~bit;
    if(*found==0)free_blocks_.erase(tableKey(block));
  }
}
bool SparseVoxelMap::fullFreeBlock(const Eigen::Vector3i& block) const {
  const auto* found=free_blocks_.find(tableKey(block));
  // set() rejects out-of-domain cells, so a boundary block with unsupported
  // slots cannot acquire all 64 bits. Const queries never mutate summaries.
  return found&&*found==std::numeric_limits<std::uint64_t>::max();
}
void SparseVoxelMap::set(const Eigen::Vector3i& i,VoxelStatus status) {
  if(!inside(i)||!validStatus(status))throw std::invalid_argument("Invalid sparse voxel index/status");
  if(status==VoxelStatus::Unknown) {
    auto* found=cells_.find(tableKey(i));
    if(!found)return;
    if(found->weight==0.)cells_.erase(tableKey(i));
    else found->status=status;
  } else cells_[tableKey(i)].status=status;
  // Update only after the occupancy mutation succeeds. TSDF changes never
  // promote an Unknown cell to Free and therefore do not touch this index.
  updateFreeBlock(i,status);
}
void SparseVoxelMap::setTsdf(const Eigen::Vector3i& i,double distance,double weight) {
  if(!inside(i)||!std::isfinite(distance)||!std::isfinite(weight)||weight<0.)
    throw std::invalid_argument("Invalid sparse TSDF observation");
  if(weight==0.) {
    auto* found=cells_.find(tableKey(i));
    if(!found)return;
    if(found->status==VoxelStatus::Unknown)cells_.erase(tableKey(i));
    else {found->distance=distance;found->weight=0.;}
  } else {
    auto& observation=cells_[tableKey(i)];observation.distance=distance;observation.weight=weight;
  }
}
void SparseVoxelMap::setObservation(const Eigen::Vector3i& i,VoxelStatus status,
                                    double distance,double weight) {
  if(!inside(i)||!validStatus(status))throw std::invalid_argument("Invalid sparse voxel index/status");
  if(!std::isfinite(distance)||!std::isfinite(weight)||weight<0.)
    throw std::invalid_argument("Invalid sparse TSDF observation");
  // Unknown without a distance observation has no stored cell. Keep tiny
  // positive weights verbatim; the query, not storage, applies the threshold.
  if(status==VoxelStatus::Unknown&&weight==0.) {
    if(!cells_.erase(tableKey(i)))return;
  }else {
    auto& observation=cells_[tableKey(i)];
    // setTsdf() normalizes either signed zero weight to positive zero.
    observation=Cell{status,distance,weight==0.?0.:weight};
  }
  updateFreeBlock(i,status);
}
VoxelStatus SparseVoxelMap::voxel(const Eigen::Vector3d& p) const {
  if(!contains(p))return VoxelStatus::Unknown;
  const auto* observation=cell(index(p));
  return observation?observation->status:VoxelStatus::Unknown;
}
std::optional<TsdfDistance> SparseVoxelMap::tsdfDistance(const Eigen::Vector3d& p) const {
  if(!contains(p))return std::nullopt;
  const Eigen::Vector3d grid=p/resolution_-Eigen::Vector3d::Constant(.5);
  const Eigen::Vector3i lower=grid.array().floor().cast<int>();
  const Eigen::Vector3d fraction=grid-lower.cast<double>();
  double distance=0.,weight=0.;bool complete=true;
  // Match VoxelMap: even a zero interpolation coefficient needs an observed
  // neighbor, and incomplete neighborhoods fall back to the containing voxel.
  for(int z=0;z<2&&complete;++z)for(int y=0;y<2&&complete;++y)for(int x=0;x<2;++x) {
    const auto* observation=cell(lower+Eigen::Vector3i(x,y,z));
    if(!observation||observation->weight<1e-6){complete=false;break;}
    const double coefficient=(x?fraction.x():1.-fraction.x())*
        (y?fraction.y():1.-fraction.y())*(z?fraction.z():1.-fraction.z());
    distance+=coefficient*observation->distance;weight+=coefficient*observation->weight;
  }
  if(complete)return TsdfDistance{distance,weight,true};
  const auto* observation=cell(index(p));
  if(!observation||observation->weight<1e-6)return std::nullopt;
  return TsdfDistance{observation->distance,observation->weight,false};
}
std::optional<TsdfDistance> SparseVoxelMap::tsdfDistanceSimplex(const Eigen::Vector3d& p) const {
  if(!contains(p))return std::nullopt;
  const Eigen::Vector3d grid=p/resolution_-Eigen::Vector3d::Constant(.5);
  const Eigen::Vector3i lower=grid.array().floor().cast<int>();
  return detail::observedTsdfSimplex(grid-lower.cast<double>(),
      [&](const Eigen::Vector3i& vertex)->std::optional<TsdfDistance>{
        const auto* observation=cell(lower+vertex);if(!observation)return std::nullopt;
        return TsdfDistance{observation->distance,observation->weight,false};
      });
}
std::optional<TsdfDistance> SparseVoxelMap::tsdfDistanceSimplexCached(
    const Eigen::Vector3d& p,TsdfQueryContext& context) const {
  if(!contains(p))return std::nullopt;
  const Eigen::Vector3d grid=p/resolution_-Eigen::Vector3d::Constant(.5);
  const Eigen::Vector3i lower=grid.array().floor().cast<int>();
  if(context.owner!=this||(context.lower.array()!=lower.array()).any()){
    context.owner=this;context.lower=lower;context.initialized=0;
  }
  // Keep the interpolation's vertex order and arithmetic unchanged. Only raw
  // cell reads are shared during the caller's immutable, per-ray interval.
  return detail::observedTsdfSimplex(grid-lower.cast<double>(),
      [&](const Eigen::Vector3i& vertex)->std::optional<TsdfDistance>{
        const int slot=vertex.x()|(vertex.y()<<1)|(vertex.z()<<2);
        const auto bit=static_cast<std::uint8_t>(1u<<slot);
        if(context.initialized&bit)++context.hits;
        else {
          ++context.lookups;const auto* observation=cell(lower+vertex);
          if(observation)context.observations[slot]=TsdfDistance{observation->distance,observation->weight,false};
          else context.observations[slot]=std::nullopt;
          context.initialized|=bit;
        }
        return context.observations[slot];
      });
}
VoxelStatus SparseVoxelMap::box(const Eigen::Vector3d& p,const Eigen::Vector3d& size) const {
  const bool point=(size.array()==0.).all();
  if(!p.allFinite()||!size.allFinite()||(!point&&(size.array()<=0.).any()))
    throw std::invalid_argument("Invalid sparse swept-box query");
  const Eigen::Vector3d half=size*.5;
  const Eigen::Vector3d lo=p-half,hi=p+half;
  if((lo.array()<minimum().array()).any()||(hi.array()>=maximum().array()).any())
    return VoxelStatus::Unknown;
  const Eigen::Vector3i radius=((half/resolution_).array().floor()+1.).cast<int>();
  // Match the stationary ray's candidate origin, including its center/index
  // round trip at extremely small resolutions. No DDA or callback is needed.
  const Eigen::Vector3i current=index(center(index(p)));
  Eigen::Vector3i low=(current-radius).cwiseMax(Eigen::Vector3i::Constant(-kIndexLimit));
  Eigen::Vector3i high=(current+radius).cwiseMin(Eigen::Vector3i::Constant(kIndexLimit));
  const auto touches=[&](int axis,int coordinate) {
    // Keep path()'s expanded-AABB arithmetic and inclusive contact test.
    // Replacing these bounds with floor((p +/- half)/resolution) changes
    // some nextafter-sized gaps at decimal resolutions.
    const double cell_lo=static_cast<double>(coordinate)*resolution_-half[axis];
    const double cell_hi=cell_lo+resolution_+size[axis];
    return !(p[axis]<cell_lo||p[axis]>cell_hi);
  };
  // Each axis's intersecting cells form a contiguous interval. Clip once
  // before looking up occupancy instead of testing every expanded 3-D cell.
  for(int axis=0;axis<3;++axis) {
    while(low[axis]<=high[axis]&&!touches(axis,low[axis]))++low[axis];
    if(low[axis]>high[axis])return VoxelStatus::Free;
    while(high[axis]>low[axis]&&!touches(axis,high[axis]))--high[axis];
  }
  bool unknown=false;
  const Eigen::Vector3i block_low=freeBlockIndex(low),block_high=freeBlockIndex(high);
  for(int bz=block_low.z();bz<=block_high.z();++bz)
    for(int by=block_low.y();by<=block_high.y();++by)
      for(int bx=block_low.x();bx<=block_high.x();++bx) {
        const Eigen::Vector3i block(bx,by,bz);
        if(fullFreeBlock(block))continue;
        const Eigen::Vector3i begin=low.cwiseMax(4*block);
        const Eigen::Vector3i end=high.cwiseMin(4*block+Eigen::Vector3i::Constant(3));
        for(int z=begin.z();z<=end.z();++z)for(int y=begin.y();y<=end.y();++y)
          for(int x=begin.x();x<=end.x();++x) {
            const auto* observation=cell(Eigen::Vector3i(x,y,z));
            const auto status=observation?observation->status:VoxelStatus::Unknown;
            if(status==VoxelStatus::Occupied)return VoxelStatus::Occupied;
            if(status==VoxelStatus::Unknown)unknown=true;
          }
      }
  return unknown?VoxelStatus::Unknown:VoxelStatus::Free;
}
VoxelStatus SparseVoxelMap::path(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
                               const Eigen::Vector3d& size) const {
  const bool point=(size.array()==0.).all();
  if(!a.allFinite()||!b.allFinite()||!size.allFinite()||
     (!point&&(size.array()<=0.).any()))
    throw std::invalid_argument("Invalid sparse swept-box query");
  const Eigen::Vector3d half=size*.5;
  const Eigen::Vector3d lo=a.cwiseMin(b)-half,hi=a.cwiseMax(b)+half;
  if((lo.array()<minimum().array()).any()||(hi.array()>=maximum().array()).any())
    return VoxelStatus::Unknown;
  // DDA follows the centerline. A local neighborhood covers the robot body;
  // exact segment/expanded-AABB tests retain face, edge and corner contacts.
  // Only the entering slab is examined after each DDA step, so a long diagonal
  // costs O(ray length * body cross-section), not its enclosing box's volume.
  const Eigen::Vector3i radius=((half/resolution_).array().floor()+1.).cast<int>();
  bool unknown=false,occupied=false,first=true;
  Eigen::Vector3i previous=Eigen::Vector3i::Zero();
  ray(a,b,[&](const Eigen::Vector3d& position,VoxelStatus) {
    const Eigen::Vector3i current=index(position);
    Eigen::Vector3i low=current-radius,high=current+radius;
    if(!first) {
      for(int k=0;k<3;++k)if(current[k]!=previous[k]) {
        low[k]=high[k]=current[k]+(current[k]>previous[k]?radius[k]:-radius[k]);
        break;
      }
    }
    first=false;previous=current;
    low=low.cwiseMax(Eigen::Vector3i::Constant(-kIndexLimit));
    high=high.cwiseMin(Eigen::Vector3i::Constant(kIndexLimit));
    const Eigen::Vector3i block_low=freeBlockIndex(low),block_high=freeBlockIndex(high);
    for(int bz=block_low.z();bz<=block_high.z();++bz)
      for(int by=block_low.y();by<=block_high.y();++by)
        for(int bx=block_low.x();bx<=block_high.x();++bx) {
          const Eigen::Vector3i block(bx,by,bz);
          if(fullFreeBlock(block))continue;
          const Eigen::Vector3i begin=low.cwiseMax(4*block);
          const Eigen::Vector3i end=high.cwiseMin(4*block+Eigen::Vector3i::Constant(3));
          for(int z=begin.z();z<=end.z();++z)for(int y=begin.y();y<=end.y();++y)
            for(int x=begin.x();x<=end.x();++x) {
              const Eigen::Vector3i i(x,y,z);const auto* observation=cell(i);
              const auto status=observation?observation->status:VoxelStatus::Unknown;
              if(status==VoxelStatus::Free)continue;
              const Eigen::Vector3d cell_lo=i.cast<double>()*resolution_-half;
              const Eigen::Vector3d cell_hi=cell_lo+Eigen::Vector3d::Constant(resolution_)+size;
              if(!intersects(a,b,cell_lo,cell_hi,point))continue;
              if(status==VoxelStatus::Occupied){occupied=true;return false;}
              unknown=true;
            }
        }
    return true;
  });
  return occupied?VoxelStatus::Occupied:unknown?VoxelStatus::Unknown:VoxelStatus::Free;
}
void SparseVoxelMap::ray(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
    const std::function<bool(const Eigen::Vector3d&,VoxelStatus)>& visit) const {
  if(!a.allFinite()||!b.allFinite())throw std::invalid_argument("Invalid sparse ray");
  auto i=index(a);const auto last=index(b);
  const Eigen::Vector3d delta=b-a;Eigen::Vector3d tmax;Eigen::Vector3i step;
  for(int k=0;k<3;++k) {
    step[k]=(delta[k]>0.)-(delta[k]<0.);
    if(i[k]==last[k])tmax[k]=std::numeric_limits<double>::infinity();
    else {
      const double boundary=(i[k]+(step[k]>0?1:0))*resolution_;
      tmax[k]=(boundary-a[k])/delta[k];
    }
  }
  const int64_t limit=(last.cast<int64_t>()-i.cast<int64_t>()).cwiseAbs().sum()+4;
  for(int64_t n=0;n<limit;++n) {
    const auto* observation=cell(i);
    const auto status=observation?observation->status:VoxelStatus::Unknown;
    if(!visit(center(i),status)||(i.array()==last.array()).all())break;
    Eigen::Index axis;tmax.minCoeff(&axis);
    i[axis]+=step[axis];
    // Recompute instead of accumulating tdelta: rays can span the full sparse
    // domain, and accumulated round-off must not omit their final cells.
    if(i[axis]==last[axis])tmax[axis]=std::numeric_limits<double>::infinity();
    else {
      const double boundary=(i[axis]+(step[axis]>0?1:0))*resolution_;
      tmax[axis]=(boundary-a[axis])/delta[axis];
    }
  }
}
}  // namespace gbplanner2
