#pragma once
#include "gbplanner_local/local_planner.hpp"

namespace gbplanner2 {
// Defaults from upstream RandomSamplingParams. Searching has its own graph
// and fixed sampling settings; exploration gain and adaptive PCA are unused.
struct SearchParameters {
  int max_vertices{500},max_edges{10000},max_attempts{100000};
  // findPathToConnect checks its zero-based loop_count after expansion and
  // before incrementing. Rejected sampleVertex batches do not advance it;
  // max_attempts separately retains the ROS 2 cap on every raw sampler draw.
  int empty_graph_cutoff{2000},max_target_candidates{5};
  double target_radius{2.};
  bool check_collision_at_source{true};
  // Upstream search returns directly without improveFreePath. Keep that
  // behavior unless this ROS 2 extension is explicitly requested as well as
  // Parameters::refinement.enabled; both route endpoints remain fixed.
  bool refine_path{false};
  void validate() const;
};
struct SearchResult {
  bool success{false},exact_target{false};
  std::string reason;
  std::vector<StateVec> path;
  // Distance to the requested target, before terrain projection. Ground exact
  // arrival allows at most one map resolution of Z projection, with exact XY.
  double endpoint_distance{0.},path_length{0.},build_ms{0.};
  int vertices{0},edges{0},attempts{0},collision_rejections{0};
  // Accepted sampling batches that reached expandGraph, including failed
  // expansions. Unlike local buildGraph, wholly rejected batches do not count.
  // cutoff=N therefore stops an isolated root after N+1 such expansions.
  int sampling_rounds{0};
  bool empty_graph_cutoff_reached{false};
  int target_candidates{0},target_id{-1};
  PathRefinementDiagnostics refinement;
};

class SearchPlanner {
 public:
  SearchPlanner(std::shared_ptr<const MapQuery> map,Parameters parameters,
                SearchParameters search_parameters={});
  SearchResult search(const StateVec& source,const StateVec& target);
  void graphMessage(planner_msgs::msg::Graph& message);
  const GraphManager& graph() const {return graph_;}
 private:
  bool project(const StateVec& input,StateVec& output,bool allow_relocation=false) const;
  bool inside(const StateVec& state,const StateVec& source) const;
  bool expand(const StateVec& state,PlanResult& counters);
  void shortestPaths(ShortestPathsReport& report);
  std::vector<StateVec> path(int target,const ShortestPathsReport& report);
  std::shared_ptr<const MapQuery> map_;
  Parameters p_;
  SearchParameters search_;
  GraphManager graph_;
  std::unique_ptr<TerrainProjector> terrain_;
  GroundEdgeCache ground_edges_;
  Eigen::Matrix3d world_to_sampling_;
};
}  // namespace gbplanner2
