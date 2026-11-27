// Four-axis sampling policy from GBPlanner 2 RandomSamplerBase / RandomSampler.
// Copyright (c) 2020 UNR Autonomous Robots Lab. BSD-3-Clause.
#pragma once
#include <Eigen/Core>
#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>

namespace gbplanner2 {
struct AxisSamplingParameters {
  std::string distribution{"uniform"};
  // Ranges bound uniform draws only. As in the original generate(), normal
  // and Cauchy samples are not truncated here; the planner clips its workspace.
  double minimum{-1.},maximum{1.},mean{0.},scale{1.},constant{0.};
  void validate()const{
    if(distribution!="uniform"&&distribution!="normal"&&
       distribution!="cauchy"&&distribution!="constant")
      throw std::invalid_argument("Invalid axis sampling distribution");
    if(!std::isfinite(minimum)||!std::isfinite(maximum)||!std::isfinite(mean)||
       !std::isfinite(scale)||!std::isfinite(constant)||
       (distribution=="uniform"&&minimum>=maximum)||
       ((distribution=="normal"||distribution=="cauchy")&&scale<=0.))
      throw std::invalid_argument("Invalid axis sampling parameters");
  }
};
using SamplingAxes=std::array<AxisSamplingParameters,4>;

// Unset fields inherit the existing XYZ settings (Heading: uniform [-pi,pi]).
// The separate adaptive profile is layered over the exploration profile.
struct AxisSamplingOverride {
  std::optional<std::string> distribution;
  std::optional<double> minimum,maximum,mean,scale,constant;
  AxisSamplingParameters apply(AxisSamplingParameters parameters)const{
    if(distribution)parameters.distribution=*distribution;
    if(minimum)parameters.minimum=*minimum;
    if(maximum)parameters.maximum=*maximum;
    if(mean)parameters.mean=*mean;
    if(scale)parameters.scale=*scale;
    if(constant)parameters.constant=*constant;
    return parameters;
  }
};
using SamplingOverrides=std::array<AxisSamplingOverride,4>;

// Rrg::buildGraph calls setBound and setDistributionParams on every XYZ axis
// after constructing the PCA box. Those calls replace even manual moments and
// ranges, but never change the distribution type, a constant, or Heading.
inline void applyAdaptiveSamplingGeometry(SamplingOverrides& axes,
    const Eigen::Vector3d& minimum,const Eigen::Vector3d& maximum,
    const Eigen::Vector3d& mean,const Eigen::Vector3d& scale){
  for(int i=0;i<3;++i){
    axes[i].minimum=minimum[i];axes[i].maximum=maximum[i];
    axes[i].mean=mean[i];axes[i].scale=scale[i];
  }
}
}  // namespace gbplanner2
