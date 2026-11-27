#pragma once
#include "gbplanner_local/ground_terrain.hpp"
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace gbplanner2 {
struct Parameters;
struct PathRefinementParameters {
  bool enabled{false};
  // Heading policies also apply when geometric path refinement is disabled.
  bool planning_backward{false},homing_backward{false};
  // RobotParams has no upstream fallback; these values are the RMF config.
  Eigen::Vector3d safety_extension{3.,4.,3.};
  double interpolation_distance{.5},relaxed_corridor_multiplier{1.};
  bool capture_geometry{false};
  void validate()const;
};
struct RefinementPlane {
  // World half-space: normal.dot(point) <= offset. Keep the algorithm's
  // unnormalized coefficients, including its dimensionless 0.05 tolerance.
  Eigen::Vector3d normal{Eigen::Vector3d::Zero()},tangent{Eigen::Vector3d::Zero()};
  double offset{0.};
  bool obstacle{false};
};
struct RefinementAttemptGeometry {
  std::size_t segment_index{0};
  Eigen::Vector3d start{Eigen::Vector3d::Zero()},target{Eigen::Vector3d::Zero()},center{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
  // modify() output before any ground projection or later path checks.
  Eigen::Vector3d adjusted{Eigen::Vector3d::Zero()},centroid{Eigen::Vector3d::Zero()};
  bool centroid_valid{false},modify_attempted{false},modify_succeeded{false},locally_accepted{false};
  std::string reason{"target_unavailable"};
  std::vector<RefinementPlane> planes;
  // Samples already queried Free in this attempted endpoint cross-section.
  // They are diagnostic samples, not a collision-free corridor certificate.
  std::vector<Eigen::Vector3d> free_points;
};
struct RefinementGeometry {
  static constexpr std::size_t kMaxAttempts=64,kMaxPlanes=1024,kMaxFreePoints=8192;
  bool captured{false},path_accepted{false},truncated{false};
  // Totals include omitted geometry; dropped counts explain every cap.
  std::size_t attempt_count{0},dropped_attempts{0},plane_count{0},dropped_planes{0},
      free_point_count{0},dropped_free_points{0};
  std::vector<RefinementAttemptGeometry> attempts;
};
struct PathRefinementDiagnostics {
  bool attempted{false},modified{false};
  std::string reason{"disabled"};
  int removed_vertices{0},adjusted_vertices{0};
  RefinementGeometry geometry;
  // Complete adopted refinement, before caller-side reference insertion and
  // output interpolation. Empty when capture is off or refinement rejected.
  // Keep every original StateVec; display decimation would lose terrain knots.
  std::vector<StateVec> accepted_path;
};
struct PathRefinementResult {
  bool success{false},modified{false};
  std::string reason;
  std::vector<StateVec> path;
  double path_length{0.};
  int removed_vertices{0},adjusted_vertices{0};
  RefinementGeometry geometry;
};

// Rrg::improveFreePath / modifyPath: short-edge pruning followed by an
// ellipsoid-tangent corridor and a sampled endpoint cross-section centroid.
// All poses use planning height. Failure retains the complete input path.
class PathRefiner {
 public:
  PathRefiner(std::shared_ptr<const MapQuery> map,const Parameters& robot,
              PathRefinementParameters parameters);
  PathRefinementResult refine(const std::vector<StateVec>& input,
                               bool relaxed=false,bool preserve_endpoint=true)const;
 private:
  bool safe(const StateVec& start,const StateVec& end)const;
  bool projected(StateVec& state)const;
  bool densify(const std::vector<StateVec>& input,std::vector<StateVec>& output)const;
  bool obstacles(const Eigen::Vector3d& center,const Eigen::Vector3d& axis,
                  const Eigen::Vector3d& size,std::vector<Eigen::Vector3d>& points)const;
  bool modify(const std::vector<Eigen::Vector3d>& obstacles,
              const Eigen::Vector3d& start,const Eigen::Vector3d& target,
              Eigen::Vector3d& adjusted,RefinementGeometry* geometry,
              RefinementAttemptGeometry* attempt)const;
  std::shared_ptr<const MapQuery> map_;
  PathRefinementParameters p_;
  Eigen::Vector3d robot_size_,center_offset_;
  bool tangent_yaw_{true};
  std::unique_ptr<TerrainProjector> terrain_;
};

// Apply only a complete checked refinement. Any rejected refinement or
// recoverable computation failure keeps the caller's safe path and length.
PathRefinementDiagnostics refinePlannedPath(std::shared_ptr<const MapQuery> map,
    const Parameters& parameters,std::vector<StateVec>& path,double& path_length,
    bool relaxed,bool preserve_endpoint);

// Final spatial discretization from Rrg::getBestPath/getGlobalPath/
// getHomingPath/runGlobalPlanner, after traversal clipping, optional geometric
// refinement and reference-path insertion. Use on the response copy: graph
// vertex IDs, candidate gains and complete-edge traversal accounting stay as-is.
// As in the upstream Pose overload, spacing restarts at each original segment
// and each inserted point inherits its successor's yaw. Unlike that overload,
// every original StateVec is retained exactly (including first/last yaw and
// coincident pure-yaw knots); in particular, the final endpoint is not omitted.
// No terrain re-projection or shortcut is performed. Existing checked terrain
// knots remain in order, and each new subsegment lies on its original segment.
// This is independent of PathRefinementParameters::enabled. Invalid values or
// a point-budget overflow throw without modifying the caller's input.
std::vector<StateVec> interpolateOutputPath(const std::vector<StateVec>& input,
    double spacing,std::size_t max_points=20000);
}
