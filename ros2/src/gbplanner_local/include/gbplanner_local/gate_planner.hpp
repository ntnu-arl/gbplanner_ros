#pragma once
#include "gbplanner_local/voxel_map.hpp"
#include <planner_common/graph_base.h>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace gbplanner2 {
struct Parameters;
struct GateParameters {
  bool enabled{false};
  Eigen::Vector3d center{4.,0.,1.};
  double heading{0.},center_search_radius{.5},center_search_step{.05};
  double line_search_length{5.},line_search_range{M_PI/18.},line_search_step{M_PI/180.};
  // Optional explicit extended/relaxed/minimum boxes, in that order. Empty
  // means only the configured exact robot.size, with no implicit shrinking.
  std::vector<Eigen::Vector3d> robot_sizes;
  void validate()const;
};
struct GateResult {
  bool success{false};
  std::string reason;
  std::vector<StateVec> path;
  double path_length{0.},heading{0.};
  Eigen::Vector3d center{Eigen::Vector3d::Zero()},robot_size{Eigen::Vector3d::Zero()};
  int bound_level{-1},bound_mode{3},centers_tested{0},lines_tested{0};
};

// Direct port of Rrg::searchPathToPassGate / searchPathThroughCenterPoint.
// Geometry only: TF lookup and preserving the path in global memory belong
// to the ROS boundary. The original method is an aerial straight-line search.
class GatePlanner {
 public:
  GatePlanner(std::shared_ptr<const MapQuery> map,const Parameters& robot,GateParameters parameters);
  GateResult plan(const StateVec& current)const;
  GateResult plan(const StateVec& current,const Eigen::Vector3d& center,double heading)const;
 private:
  bool throughCenter(const StateVec& current,const Eigen::Vector3d& center,
                      double heading,const Eigen::Vector3d& size,GateResult& result)const;
  std::shared_ptr<const MapQuery> map_;
  GateParameters p_;
  Eigen::Vector3d robot_size_,center_offset_;
  bool ground_robot_{false};
};
}
