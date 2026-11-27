#pragma once
#include "gbplanner_local/parameters.hpp"
#include "gbplanner_local/voxel_map.hpp"
#include "gbplanner_local/gain_raycast.hpp"
#include "gbplanner_local/gain_diagnostics.hpp"
#include "gbplanner_local/ground_edge_diagnostics.hpp"
#include <planner_common/graph_manager.h>
#include <array>
#include <cstdint>
#include <memory>
#include <map>
#include <optional>
#include <string>

namespace gbplanner2 {
struct PlanningDiagnosticsOptions {
  bool samples{false},projected_graph{false},failed_edges{false};
  bool paths{false};
  bool gains{false};
  int gain_vertex_id{20}; // -1 selects the first vertex actually evaluated.
  bool ground_edges{false};
};
struct PlanningPathVertex {
  int id{-1},parent{-1};
  StateVec state{StateVec::Zero()};
  bool leaf{false};
};
// Bounded snapshots of geometry already evaluated by the planner. Leaving an
// option disabled keeps its vectors empty and performs no extra map queries.
struct PlanningDiagnostics {
  PlanningDiagnosticsOptions options;
  static constexpr std::size_t kMaxSamples=4000,kMaxFailedEdges=100,kMaxProjectedVertices=20000;
  std::vector<Eigen::Vector3d> valid_samples,invalid_samples,projected_vertices;
  std::vector<std::array<Eigen::Vector3d,2>> projected_segments,failed_edges;
  // Actual reachable Dijkstra tree and complete scored candidate, before
  // executable-prefix extraction and refinement. Root is its own parent.
  std::vector<PlanningPathVertex> shortest_vertices;
  std::vector<StateVec> best_path;
  int best_id{-1};
  double best_gain{0};
  GainGeometry gains;
  GroundEdgeDiagnostics ground_edges;
  std::size_t dropped_samples{0},dropped_failed_edges{0},dropped_projected_edges{0};
  void reset();
  void sample(const StateVec& state,bool valid);
  void failedEdge(const Eigen::Vector3d& from,const Eigen::Vector3d& to);
  void projectedEdge(const GroundEdge& edge);
};
struct PlanResult {
  bool success{false};
  // Rrg::buildGraph advances its lifetime trigger count after the sampling
  // loop, even when evaluation cannot subsequently produce a usable path.
  // Invalid roots and other returns before that loop completes do not count.
  bool graph_build_completed{false};
  std::string reason;
  std::vector<StateVec> path;
  int vertices{0},edges{0},attempts{0},collision_rejections{0},best_id{-1};
  // attempts counts raw sampler draws; sampling_rounds counts upstream-style
  // sampleVertex calls, which stop on acceptance or after 1000 rejected draws.
  int sampling_rounds{0};
  // Explains graph construction termination independently of path evaluation.
  // Graph limits take precedence when reached on the final budgeted sample.
  std::string sampling_stop_reason{"not_started"};
  bool empty_graph_cutoff_reached{false};
  int gain_evaluations{0},reused_gains{0};
  int hanging_vertices{0},candidate_hanging_vertices{0},executed_target_id{-1};
  bool truncated_for_hanging{false},truncated_for_budget{false};
  bool gain_evaluated{false},frontier_exists{false};
  double gain{0},path_length{0},build_ms{0},evaluate_ms{0};
  // Subsets of evaluate_ms, not extra work to add to the total. Recorded on
  // completed phases only; gain_evaluated/refinement describe execution.
  double gain_ms{0},refinement_ms{0};
  // Length/time before path refinement, using complete executable graph edges.
  double traverse_length{0},traverse_time{0};
  double unpenalized_gain{0},direction_distance{0},exploring_direction{0};
  double direction_factor{1};
  double candidate_path_length{0},gain_before_hanging_penalty{0};
  PathRefinementDiagnostics refinement;
  // Geometry actually used by this call, also filled from configured values
  // when adaptive bounds are disabled or map-based construction fails.
  AdaptiveBoundResult adaptive;
  StateVec sampling_root{StateVec::Zero()};
  std::string gain_mode{"ray"};
  double gain_voxel_size{0};
  std::size_t gain_voxel_count{0},gain_fov_voxel_count{0},gain_visibility_queries{0};
};
// Shared aerial expandGraph branch. Both local planning and background global
// expansion use the exact same steering, overshoot and neighbor-edge checks.
bool expandAerial(GraphManager& graph,const std::shared_ptr<const MapQuery>& map,
                  const Parameters& parameters,StateVec state,PlanResult& result,int max_edges,
                  PlanningDiagnostics* diagnostics=nullptr);
using GroundEdgeCache=std::map<std::pair<int,int>,GroundEdge>;
bool expandGround(GraphManager& graph,const std::shared_ptr<const MapQuery>& map,
                  const Parameters& parameters,StateVec state,PlanResult& result,int max_edges,
                  GroundEdgeCache* edges=nullptr,bool allow_hanging=false,
                  PlanningDiagnostics* diagnostics=nullptr);
// Opt-in only for a single synchronous planning request whose map and policy
// snapshots have no mutable writers. Other public query callers keep live-map
// semantics by default, even when they hold a shared_ptr<const MapQuery> alias.
enum class ExactRayGainReuse { Disabled, ImmutableSnapshot };
struct ExactRayGainCacheStats {
  std::size_t hits{0},evaluations{0},entries{0};
};
class LocalPlanner {
 public:
  LocalPlanner(std::shared_ptr<const MapQuery> map,Parameters p,PlanningDiagnosticsOptions diagnostics={},
               ExactRayGainReuse gain_reuse=ExactRayGainReuse::Disabled);
  // A snapshot of actual motion history; planning a hypothetical root must
  // never advance the exploration-direction estimator.
  PlanResult plan(const StateVec& root,double exploring_direction=0.0);
  const GraphManager& graph() const { return graph_; }
  const PlanningDiagnostics& diagnostics() const {return diagnostics_;}
  void graphMessage(planner_msgs::msg::Graph& message);
  // Cost-only reuse within this object's immutable planning snapshot. The
  // caller must supply the same map/policy and exact directed graph endpoints.
  // Candidate/hanging or mismatched entries return no value for strict fallback.
  // No cached polyline is exported: graph yaw may have changed during scoring.
  std::optional<double> validatedGroundLength(const std::shared_ptr<const MapQuery>& map,
      const GroundParameters& parameters,int from_id,int to_id,
      const StateVec& from,const StateVec& to) const;
  VolumetricGain gain(const StateVec& state) const;
  std::vector<SensorGain> sensorGains(const StateVec& state) const;
  std::vector<SensorGain> raySensorGains(const StateVec& state) const;
  std::vector<SensorGain> voxelSensorGains(const StateVec& state) const;
  VolumetricGain rayGain(const StateVec& state) const;
  // Rrg::computeVolumetricGainRayModelNoBound: always use rays and retain
  // the global workspace boundary, but ignore local no-gain zones.
  VolumetricGain globalRayGain(const StateVec& state) const;
  ExactRayGainCacheStats exactRayGainCacheStats() const {
    return {exact_ray_gain_hits_,exact_ray_gain_evaluations_,
            exact_ray_gains_.size()+exact_global_ray_gains_.size()};
  }
  TerrainQueryReuseStats terrainQueryReuseStats()const {
    return terrain_?terrain_->queryReuseStats():TerrainQueryReuseStats{};
  }
  const std::vector<SensorParameters>& sensorParameters() const {return sensors_;}
  std::vector<Eigen::Vector3d> sensorEndpoints(std::size_t index,const StateVec& state) const;
 private:
  bool expand(StateVec state,PlanResult& result);
  bool inside(const StateVec& state,const StateVec& root) const;
  void computeExplorationGain(PlanResult& result);
  using ExactRayGainKey=std::array<std::uint64_t,4>;
  static ExactRayGainKey exactRayGainKey(const StateVec& state);
  void rememberExactRayGain(const StateVec& state,const VolumetricGain& gain,
                            bool exclude_no_gain_zones=true) const;
  VolumetricGain cachedRayGain(const StateVec& state,bool exclude_no_gain_zones) const;
  std::vector<SensorGain> sensorGains(const StateVec& state,GainGeometry* capture) const;
  std::vector<SensorGain> raySensorGains(const StateVec& state,bool exclude_no_gain_zones,
                                       GainGeometry* capture=nullptr) const;
  std::vector<SensorGain> voxelSensorGains(const StateVec& state,GainGeometry* capture) const;
  VolumetricGain sumSensorGains(const std::vector<SensorGain>& gains) const;
  void groundShortestPaths(ShortestPathsReport& report);
  std::vector<StateVec> groundPath(int target,const ShortestPathsReport& report,
                                  bool executable_only=false,int* executed_target=nullptr);
  std::vector<StateVec> executablePath(int target,const ShortestPathsReport& report,PlanResult& result);
  std::shared_ptr<const MapQuery> map_;
  const Parameters configured_p_;
  Parameters p_;
  Eigen::Matrix3d world_to_sampling_;
  GraphManager graph_;
  std::unique_ptr<TerrainProjector> terrain_;
  GroundEdgeCache ground_edges_;
  PlanningDiagnostics diagnostics_;
  std::vector<SensorParameters> sensors_;
  std::vector<std::vector<Eigen::Vector3d>> rays_;
  std::vector<std::pair<std::size_t,std::size_t>> ray_grid_sizes_;
  Eigen::Vector3d gain_root_{Eigen::Vector3d::Zero()};
  bool gain_root_valid_{false};
  const bool exact_ray_gain_reuse_;
  // A full cache falls back to ordinary evaluation; capacity never changes
  // ray sampling, graph size or gain quality. This cache belongs to this object.
  static constexpr std::size_t kMaxExactRayGains=20000;
  mutable std::map<ExactRayGainKey,VolumetricGain> exact_ray_gains_;
  // Separate policy results only when zones exist. Without zones the two
  // upstream evaluators are identical and share the local exact cache.
  mutable std::map<ExactRayGainKey,VolumetricGain> exact_global_ray_gains_;
  mutable std::size_t exact_ray_gain_hits_{0},exact_ray_gain_evaluations_{0};
};
}  // namespace gbplanner2
