#pragma once

#include "gbplanner_local/voxel_map.hpp"
#include <algorithm>
#include <array>

namespace gbplanner2::detail {

// Every voxel-center cube uses the same 000->111 Freudenthal diagonal.
// Sorting u>=v>=w selects exactly one of its six tetrahedra. Adjacent
// tetrahedra/cubes have identical triangulations and values on shared faces.
// This field is deliberately not mixed pointwise with trilinear interpolation:
// bilinear and triangular face values need not agree on non-affine data.
template<class Sample>
std::optional<TsdfDistance> observedTsdfSimplex(const Eigen::Vector3d& fraction,
                                             Sample sample) {
  std::array<int,3> axes{{0,1,2}};
  std::stable_sort(axes.begin(),axes.end(),[&](int a,int b){return fraction[a]>fraction[b];});
  const std::array<double,4> coefficients{{1.-fraction[axes[0]],
      fraction[axes[0]]-fraction[axes[1]],fraction[axes[1]]-fraction[axes[2]],fraction[axes[2]]}};
  Eigen::Vector3i vertex=Eigen::Vector3i::Zero();
  double distance=0.,weight=0.;
  for(std::size_t i=0;i<coefficients.size();++i){
    if(i)vertex[axes[i-1]]=1;
    const double coefficient=coefficients[i];
    // Exact zeros lie on the convex hull of the remaining observed vertices.
    // Do not round small positive coefficients down to zero to fill a hole.
    if(coefficient==0.)continue;
    const auto observation=sample(vertex);
    if(!observation||observation->weight<1e-6)return std::nullopt;
    distance+=coefficient*observation->distance;weight+=coefficient*observation->weight;
  }
  return TsdfDistance{distance,weight,true};
}

}  // namespace gbplanner2::detail
