#pragma once
#include "gbplanner_local/voxel_map.hpp"
#include <planner_common/graph_base.h>
#include <memory>
#include <limits>
#include <vector>

namespace gbplanner2 {
enum class GroundRayProjectionMode { VoxelSurface,UpstreamSample };
const char* groundRayProjectionModeName(GroundRayProjectionMode mode);
enum class GroundTsdfProjectionMode { UpstreamStep,ZeroCrossing,ZeroCrossingSimplex };
const char* groundTsdfProjectionModeName(GroundTsdfProjectionMode mode);
struct GroundParameters {
  double max_ground_height{1.2},robot_height{1.0};
  Eigen::Vector3d center_offset{0,0,0},robot_size{.4,.4,.4};
  // Only body clearance becomes a point; ground support and slope still apply.
  bool point_robot{false};
  double max_inclination{.52},max_negative_inclination{.37};
  double projection_max_length{5.0},hanging_vertex_penalty{0.0};
  bool interpolate_projection_distance{false};
  // Noninterpolated support: the old cell-top estimate or the upstream
  // float-indexed 0.9-voxel sample. UpstreamSample requires a world-zero grid.
  GroundRayProjectionMode ray_projection_mode{GroundRayProjectionMode::VoxelSurface};
  // Both zero-crossing fields are explicit ROS 2 adaptations, not upstream
  // behavior. Simplex uses one continuous observed-vertex field for the ray.
  GroundTsdfProjectionMode tsdf_projection_mode{GroundTsdfProjectionMode::UpstreamStep};
  void validate()const;
};

enum class GroundStatus { Supported,Admissible,Hanging,NoGround,Steep,Unknown,Occupied,Invalid,OffTerrain };
const char* groundStatusName(GroundStatus status);

struct GroundProjection {
  GroundStatus status{GroundStatus::Invalid};
  bool support{false},hanging{false};
  StateVec projected_state{StateVec::Zero()};
  // Supported projections use the selected cell-top/ray-sample estimate,
  // or the TSDF surface estimate when distance interpolation is enabled.
  // A hanging projection has no observed ground_z (NaN), only a candidate.
  double ground_z{0.},projection_distance{0.};
  // Both zero-crossing modes populate this bracket. It bounds a zero of the map's
  // continuous interpolated field, not the error relative to physical ground.
  double zero_crossing_lower_z{std::numeric_limits<double>::quiet_NaN()};
  double zero_crossing_upper_z{std::numeric_limits<double>::quiet_NaN()};
  int ray_index{-1};
};

struct GroundEdge {
  GroundStatus status{GroundStatus::Invalid};
  std::vector<StateVec> polyline;
  double actual_length{0.},max_uphill{0.},max_downhill{0.},mean_inclination{0.};
  bool speculative{false};
  bool admissible()const{return status==GroundStatus::Admissible;}
  bool candidateAdmissible()const{return admissible()||(status==GroundStatus::Hanging&&speculative);}
};

// Candidate lookahead is permitted by the logical edge endpoints, not by an
// unknown patch discovered midway or at a virtual overshoot endpoint.
enum class GroundHangingPermission { FromEndpoints,Forbid,OriginalEndpointHanging };

// Opt in only when this projector's map/policy are an immutable snapshot for
// one synchronous request. A const MapQuery alias alone is not that promise.
enum class TerrainQueryReuse { Disabled,ImmutableSnapshot };
struct TerrainQueryReuseStats {
  std::size_t projection_hits{0},projection_evaluations{0};
  std::size_t support_hits{0},support_evaluations{0};
};

// Terrain projection from both branches of Rrg::projectSample and
// getProjectedEdgeStatus, using observed voxel surfaces rather than a 2D map.
// Planning states live ground+max_ground_height above each local surface;
// the actual base_link trajectory lives ground+robot_height above that surface.
class TerrainProjector {
 public:
  TerrainProjector(std::shared_ptr<const MapQuery> map,GroundParameters parameters);
  TerrainProjector(std::shared_ptr<const MapQuery> map,GroundParameters parameters,
                   TerrainQueryReuse reuse);
  void clearQueryReuse();
  TerrainQueryReuseStats queryReuseStats()const;
  bool queryReuseEnabled()const{return static_cast<bool>(query_cache_);}
  GroundProjection project(const StateVec& candidate)const;
  GroundEdge edge(const StateVec& from,const StateVec& to)const;
  // Validate an existing commanded segment without changing its XYZ/yaw.
  // Projection creates a terrain path; validation checks the supplied path.
  // The optional same-site height attachment is only for measured odometry,
  // using the existing sub-voxel attachment rule, never for reference edges.
  GroundEdge validateFixedEdge(const StateVec& from,const StateVec& to,
                               bool allow_height_attachment=false)const;
  // By default at least one projected endpoint must be hanging before unknown
  // intermediate support may inform lookahead. Graph callers explicitly carry
  // the original endpoints' permission through overshoot and reverse checks.
  // Body clearance stays observed-free; speculative edges are never executable.
  GroundEdge candidateEdge(const StateVec& from,const StateVec& to,
      GroundHangingPermission permission=GroundHangingPermission::FromEndpoints)const;
  StateVec baseState(const StateVec& planning_state)const;
  const GroundParameters& parameters()const{return p_;}
 private:
  struct Hit {
    VoxelStatus status{VoxelStatus::Free};double surface_z{0.},distance{0.};
    bool rejected{false};
    double lower_z{std::numeric_limits<double>::quiet_NaN()};
    double upper_z{std::numeric_limits<double>::quiet_NaN()};
  };
  Hit castDown(const Eigen::Vector3d& start)const;
  Hit castDownUncached(const Eigen::Vector3d& start)const;
  Hit castDownUpstream(const Eigen::Vector3d& start)const;
  Hit castDownDistance(const Eigen::Vector3d& start)const;
  Hit castDownZeroCrossing(const Eigen::Vector3d& start)const;
  GroundStatus bodyStatus(const StateVec& planning_state)const;
  GroundProjection projectUncached(const StateVec& candidate)const;
  GroundEdge edgeImpl(const StateVec& from,const StateVec& to,GroundHangingPermission permission)const;
  std::shared_ptr<const MapQuery> map_;
  GroundParameters p_;
  struct QueryCache;
  std::shared_ptr<QueryCache> query_cache_;
};
}
