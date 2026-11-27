#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <string>
#include <vector>

namespace gbplanner2 {
class MapQuery;

struct AdaptiveParameters {
  bool enabled{false};
  std::string type{"kPca"};
  double range{15.},size_max{30.},scale_max{10.},leaf{1.};
  void validate() const;
};

struct AdaptiveBoundResult {
  bool applied{false};
  std::string reason{"not_computed"};
  std::size_t raw_points{0},filtered_points{0};
  Eigen::Vector3d min{Eigen::Vector3d::Zero()},max{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rotations{Eigen::Vector3d::Zero()},mean{Eigen::Vector3d::Zero()};
  Eigen::Vector3d scale{Eigen::Vector3d::Zero()},centroid{Eigen::Vector3d::Zero()};
  // Raw PCL eigenvalues, before the upstream /(N-1) and axis permutation.
  Eigen::Vector3d eigenvalues{Eigen::Vector3d::Zero()};
  // World-to-sampling matrix. rotations is its yaw/pitch/roll Euler form.
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
};

// Adapted from the pinned adaptive_obb_ros PCL implementation. The public
// interface carries only Eigen values; PCL and ROS 1 types do not cross it.
class AdaptiveBound {
 public:
  explicit AdaptiveBound(AdaptiveParameters parameters);
  AdaptiveBoundResult fromMap(const MapQuery& map,const Eigen::Vector3d& root) const;
  AdaptiveBoundResult fromCloud(const std::vector<Eigen::Vector3d>& cloud,
                               const Eigen::Vector3d& root=Eigen::Vector3d::Zero()) const;
 private:
  AdaptiveParameters p_;
};

}  // namespace gbplanner2
