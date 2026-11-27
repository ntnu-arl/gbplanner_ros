// Adapted from RobotParams::getPlanningSize and Rrg::setBoundMode in GBPlanner 2.
// Copyright (c) 2020 UNR Autonomous Robots Lab, BSD-3-Clause.
#pragma once

#include <Eigen/Core>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace gbplanner2 {

struct RobotBounds {
  // Values match PlannerSrv / PlannerSearch and upstream BoundModeType.
  enum Mode { Extended=0,Relaxed=1,Minimum=2,Exact=3,Point=4 };
  Eigen::Vector3d physical_size{.4,.4,.4};
  // Extensions are additions to the FULL side length, not per-side margins.
  Eigen::Vector3d extension{0.,0.,0.},min_extension{0.,0.,0.};
  double relax_ratio{.5};
  int bound_mode{Extended};

  static bool validMode(int mode){return mode>=Extended&&mode<=Point;}
  static bool pointMode(int mode){
    requireMode(mode);return mode==Point;
  }
  static const char* modeName(int mode){
    switch(mode){
      case Extended:return "kExtendedBound";
      case Relaxed:return "kRelaxedBound";
      case Minimum:return "kMinBound";
      case Exact:return "kExactBound";
      case Point:return "kNoBound";
      default:throw std::invalid_argument("Invalid robot bound mode");
    }
  }
  void validate()const{
    if(!physical_size.allFinite()||(physical_size.array()<=0.).any()||
       !extension.allFinite()||!min_extension.allFinite()||
       (min_extension.array()<0.).any()||(extension.array()<min_extension.array()).any()||
       !std::isfinite(relax_ratio)||relax_ratio<0.||relax_ratio>1.||!validMode(bound_mode))
      throw std::invalid_argument("Invalid robot bound parameters");
  }
  void setBoundMode(int mode){requireMode(mode);bound_mode=mode;}

  Eigen::Vector3d size(int mode)const{
    validate();requireMode(mode);
    switch(mode){
      case Extended:return physical_size+extension;
      case Relaxed:return physical_size+relax_ratio*min_extension+(1.-relax_ratio)*extension;
      case Minimum:return physical_size+min_extension;
      case Exact:return physical_size;
      // MapQuery routes all-zero size to exact point/segment collision checks.
      case Point:return Eigen::Vector3d::Zero();
      default:throw std::invalid_argument("Invalid robot bound mode");
    }
  }
  Eigen::Vector3d size()const{return size(bound_mode);}
  double collisionRadius(int mode)const{return .5*size(mode).norm();}
  double collisionRadius()const{return collisionRadius(bound_mode);}

  static double budgetMultiplier(int mode){
    requireMode(mode);
    if(mode==Relaxed)return 1.3;
    if(mode==Minimum)return 1.3*1.3;
    // Exact/Point had no assignment in the upstream switch and could inherit
    // the preceding call's budget. Keep them at the configured base budget.
    return 1.;
  }
  static int scaledBudget(int base,int mode){
    const double expanded=static_cast<double>(base)*budgetMultiplier(mode);
    if(base<=0||expanded>std::numeric_limits<int>::max())
      throw std::invalid_argument("Invalid robot bound graph budget");
    return static_cast<int>(expanded);
  }

 private:
  static void requireMode(int mode){
    if(!validMode(mode))throw std::invalid_argument("Invalid robot bound mode");
  }
};

}  // namespace gbplanner2
