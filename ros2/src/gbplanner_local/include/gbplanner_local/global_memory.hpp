#pragma once
#include "gbplanner_local/local_planner.hpp"
#include "gbplanner_local/trajectory.hpp"
#include "gbplanner_local/ground_terrain.hpp"
#include "gbplanner_local/ground_path_reconnection.hpp"
#include <functional>
#include <chrono>
#include <cstdint>
#include <map>
#include <set>
#include "gbplanner_local/mission_budget.hpp"

namespace gbplanner2 {
struct HomingPathDiagnostics {
  bool captured{false};
  // Current-to-home vertices from the existing route report, before terrain
  // expansion, reconnection, heading changes or refinement. Planning height.
  std::vector<int> vertex_ids;
  std::vector<StateVec> vertices;
};
struct GlobalPathDiagnostics {
  bool captured{false};
  std::string home_reason{"not_captured"};
  // Ordered raw graph chains, before terrain expansion, reconnection or
  // corridor refinement. Ground vertices retain planning height. The return
  // chain describes graph reachability, not a promised future homing path.
  std::vector<int> current_to_frontier_ids,frontier_to_home_ids;
  std::vector<StateVec> current_to_frontier,frontier_to_home;
};
struct GlobalResult {
  bool success{false};std::string reason;std::vector<StateVec> path;
  int target{-1};double score{0},distance{0};
  PathRefinementDiagnostics refinement;
  GroundReconnectionDiagnostics reconnection;
  GlobalPathDiagnostics graph_paths;
};
struct FrontierUpdate {
  int candidates{0},clusters{0},near_existing{0},near_visited{0},invalid_paths{0},added{0};
};
struct FrontierClusterPath {
  int vertex_id{-1},cluster_id{-1};
  // Root-to-candidate vertices from the report used by this clustering call.
  // Ground coordinates retain planning height; these are graph polylines,
  // not terrain-refined execution paths.
  std::vector<StateVec> path;
};
struct FrontierClusteringDiagnostics {
  bool evaluated{false};
  std::size_t candidate_count{0};
  std::vector<int> representative_ids;
  // Keep the clusterer's actual assignment even if refinement removed every
  // representative. Such a cluster_id need not occur in representative_ids.
  std::vector<FrontierClusterPath> candidates;
};
struct MapEdgeUpdate {
  std::size_t invalidated{0},restored{0},pending{0};
};
struct HomeEdgeDiagnostics {
  int neighbor{-1};
  StateVec state{StateVec::Zero()};
  bool active{false},fixed{false};
  GroundStatus forward{GroundStatus::Invalid},reverse{GroundStatus::Invalid};
  double forward_length{0.},reverse_length{0.};
};
struct HomeConnectivityDiagnostics {
  bool present{false},ground{false};
  StateVec home{StateVec::Zero()};
  std::size_t active_degree{0};
  VoxelStatus body{VoxelStatus::Unknown};
  GroundProjection projected;
  GroundParameters parameters;
  // Active and suspended incident edges, once each in ascending neighbor ID.
  // Forward is home -> neighbor; reverse is neighbor -> home.
  std::vector<HomeEdgeDiagnostics> edges;
};
struct BackgroundUpdate {
  int clusters{0};
  // attempts retains its original meaning: every raw sampler draw.
  std::uint64_t attempts{0},outside_or_occupied{0},near_visited{0},near_graph{0},
      near_frontier{0},collision_rejections{0},added{0},frontiers_added{0};
  std::uint64_t rounds{0},sampling_batches{0},accepted_samples{0},expansion_attempts{0};
  // Accepted free-body samples with unknown support finish their batch, then
  // are discarded before sparse-area checks or persistent graph expansion.
  std::uint64_t hanging_rejections{0};
  double elapsed_seconds{0.};
  bool time_budget_reached{false};
  std::string stop_reason{"not_started"};
};
struct GlobalPersistenceResult {
  bool success{false},map_revalidated{false};
  std::string reason;
  std::size_t vertices{0},active_edges{0},suspended_edges{0},frontiers{0};
};
struct ReferencePathDiagnostics {
  std::string reason{"not_attempted"};
  // Original path endpoint index for a failed segment; 0 denotes the root
  // or its graph attachment, and -1 a failure before any segment was checked.
  int segment{-1};
  StateVec from{StateVec::Zero()},to{StateVec::Zero()};
};
// Incremental aerial/ground addFrontiers, reference paths and global routing.
// Ground inputs, graph vertices and output polylines all use planning height;
// the ROS node converts base_link poses only at the external boundary.
class GlobalMemory {
 public:
  using GainFunction=std::function<VolumetricGain(const StateVec&)>;
  using BackgroundClock=std::function<std::chrono::steady_clock::time_point()>;
  explicit GlobalMemory(Parameters parameters);
  void reset();
  void updateMap(std::shared_ptr<const MapQuery> map);
  // ImmutableSnapshot promises that the map and its policy wrappers remain
  // unchanged for every query until the next updateMap(), not just this call.
  // Every update rebuilds terrain caches, even when the map pointer is reused.
  // The original overload keeps query reuse disabled for existing callers.
  void updateMap(std::shared_ptr<const MapQuery> map,TerrainQueryReuse reuse);
  // Call-local tracing includes old-map/cache destruction as its own phase.
  // Keep the existing overloads and object layout for already built clients.
  void updateMap(std::shared_ptr<const MapQuery> map,TerrainQueryReuse reuse,
                 const std::function<void(const char*)>& progress);
  // Cooperative process-shutdown checkpoints between retained-edge queries.
  // A throwing checkpoint abandons this update without rollback: its owner
  // must tear down the GlobalMemory rather than use its partially rechecked
  // graph. Existing overloads perform the entire update as before.
  void updateMap(std::shared_ptr<const MapQuery> map,TerrainQueryReuse reuse,
                 const std::function<void(const char*)>& progress,
                 const std::function<void()>& shutdown_checkpoint);
  // Recheck retained links after a robot-bound change while retaining home,
  // visits and suspended topology. Existing states keep their height convention.
  void setPlanningParameters(Parameters parameters);
  bool recordOdometry(const StateVec& state);
  bool setHomingPosition(const StateVec& state);
  // Preserve an accepted execution route independently of frontier filtering.
  // Checked graph links only: never changes visited poses, odometry parent or home.
  bool addReferencePath(const std::vector<StateVec>& path,int* endpoint_id=nullptr,
                        ReferencePathDiagnostics* diagnostics=nullptr);
  // Store the complete checked connection, then label its exact endpoint.
  // Return its graph ID so PCI can request the ordinary global route to it.
  int addSemanticPath(const std::vector<StateVec>& path,int semantic_class);
  // Optional call-local snapshot, overwritten even on an early return. The
  // default path retains no diagnostics across reset, load or later requests.
  void absorbLocal(const planner_msgs::msg::Graph& local_graph,const GainFunction& gain,
                   FrontierClusteringDiagnostics* diagnostics=nullptr);
  // Keep the existing entry point for built clients. This diagnostic overload
  // reports phase boundaries only and never changes the graph work.
  void absorbLocal(const planner_msgs::msg::Graph& local_graph,const GainFunction& gain,
                   FrontierClusteringDiagnostics* diagnostics,
                   const std::function<void(const char*)>& progress);
  // Optional producer reuse applies only to strictly admissible scalar costs
  // on the same immutable map/policy and exact directed XYZ endpoints.
  void absorbLocal(const planner_msgs::msg::Graph& local_graph,const GainFunction& gain,
                   FrontierClusteringDiagnostics* diagnostics,
                   const std::function<void(const char*)>& progress,
                   const LocalPlanner* producer);
  GlobalResult selectFrontier(const StateVec& current,const GainFunction& gain,int target=-1,const GlobalTimeConstraint* time=nullptr,
                              bool require_frontier=false,bool capture_refinement=false,bool capture_graph_paths=false);
  GlobalResult homing(const StateVec& current,bool capture_refinement=false);
  // Optional call-local capture; retain the original overload and result
  // layout for already built clients. No additional shortest-path search.
  GlobalResult homing(const StateVec& current,bool capture_refinement,HomingPathDiagnostics* diagnostic);
  // Fresh read-only checks on the current map/policy, using a separate terrain
  // projector. Does not populate routing caches or change graph/history state.
  // Like other graph accessors, call while graph/map mutations are serialized.
  HomeConnectivityDiagnostics inspectHomeConnectivity() const;
  // Reposition to the global vertex nearest the requested waypoint. The
  // waypoint selects a graph destination; it is not appended to the route.
  // As with other routing methods, ground poses use planning height.
  GlobalResult routeToWaypoint(const StateVec& current,const StateVec& target,
                               double max_target_distance=15.0,bool capture_refinement=false);
  const GraphManager& graph() const {return *graph_;}
  // Actual 1 m E1 event states, not the graph's 0.5 m odometry spacing.
  // A loaded prefix is retained exactly as saved; subsequent entries always
  // follow the current event threshold. Ground states use planning height.
  const std::vector<StateVec>& visitedStates() const {return visited_;}
  const StateVec& lastVisitedMarker() const {return last_visited_marker_;}
  std::uint64_t visitRevision() const {return visit_revision_;}
  std::size_t restoredHistorySize() const {return restored_history_size_;}
  // Changes only when the public Graph fields or semantic target geometry/
  // class may differ. Retiring an active frontier changes its public flag;
  // visits to other vertices and routing-only metadata do not invalidate it.
  // Keep this monotonic across reset/load; publishers can retain their cache.
  std::uint64_t snapshotRevision() const {return snapshot_revision_;}
  // Public frontier flags describe current candidate type, not stale sensor
  // gain. Persistence separately writes the original GraphManager gain data.
  void graphMessage(planner_msgs::msg::Graph& message);
  int frontierCount() const;
  int edgeCount() const {return graph_->getNumEdges();}
  const MapEdgeUpdate& lastMapEdgeUpdate() const {return map_edge_update_;}
  const FrontierUpdate& lastFrontierUpdate() const {return frontier_update_;}
  BackgroundUpdate expandBackground(int attempts,const GainFunction& gain);
  // Upstream soft budget: start before clustering; each entered round reads
  // elapsed time, then finishes every cluster's <=1000-draw sampling batch.
  // The NEXT condition uses that reading, so crossing the deadline can still
  // start one final complete round. No clock checks split a round or batch.
  // steady_clock replaces ROS time; the optional clock supports deterministic
  // scheduling checks and must advance monotonically. Spatial checks and graph
  // size caps are the same as the fixed raw-draw entry point above.
  BackgroundUpdate expandBackgroundTimed(double time_budget_seconds,const GainFunction& gain,
                                          const BackgroundClock& clock={});
  // GBP2MEM1 stores ROS 2 Graph CDR plus exploration metadata in one atomic
  // snapshot. It is distinct from GraphManager's graph-only GBP2CDR1 and from
  // ROS 1 serialization. The frame must match; poses use planning height.
  GlobalPersistenceResult saveGraph(const std::string& path,const std::string& frame) const;
  // Build and revalidate separately, then swap only on success. With no map,
  // all links/frontiers remain suspended until a current snapshot arrives.
  // Without a gain callback frontier identities remain pending for refresh().
  GlobalPersistenceResult loadGraph(const std::string& path,const std::string& expected_frame,
                                    const GainFunction& gain = {});
  // Explicit migration of the fixed upstream ROS 1 Graph wire format. Frame
  // and robot/planning-height conventions are supplied by the caller: the old
  // file contains neither robot metadata nor visited/semantic mission state.
  // Historical links and frontier identities await current-map validation.
  GlobalPersistenceResult importRos1Graph(const std::string& path,const std::string& expected_frame,
                                         const GainFunction& gain = {});
 private:
  bool safe(const StateVec& a,const StateVec& b) const;
  bool projectState(const StateVec& input,StateVec& projected) const;
  double directedLength(const StateVec& a,const StateVec& b,bool preserve_geometry=false) const;
  double linkLength(const StateVec& a,const StateVec& b,bool preserve_geometry=false) const;
  bool fixedGroundEdge(int from,int to) const;
  void markFixedGroundEdge(int from,int to);
  const GroundEdge& groundEdge(int from,int to) const;
  bool shortestPaths(GraphManager& graph,int source,ShortestPathsReport& report,
                     bool reverse=false,const LocalPlanner* producer=nullptr) const;
  Vertex* connect(const StateVec& state);
  Vertex* append(const StateVec& state,Vertex* parent,bool preserve_geometry=false);
  void neighborEdges(Vertex* vertex);
  void applyVisitedEvent(StateVec state);
  void refresh(const GainFunction& gain);
  std::vector<StateVec> backgroundCenters();
  bool drawBackgroundSample(const StateVec& center,const Eigen::Matrix3d& world_to_sampling,
                            StateVec& sample,bool& hanging,BackgroundUpdate& stats);
  void expandBackgroundSample(const StateVec& sample,const GainFunction& gain,BackgroundUpdate& stats);
  bool addPath(const std::vector<Vertex*>& path,const LocalPlanner* producer=nullptr);
  // The three upstream route producers differ in their heading/refinement
  // order. Only Homing also reconnects the ground return polyline.
  enum class RouteKind {GlobalFrontier,Homing,Waypoint};
  GlobalResult route(const StateVec& current,int target,RouteKind kind,bool capture_refinement,bool capture_graph_paths=false,
                     HomingPathDiagnostics* homing_diagnostic=nullptr);
  Parameters p_;std::shared_ptr<const MapQuery> map_;
  std::unique_ptr<TerrainProjector> terrain_;
  // Undirected topology retains a link if either direction is traversable.
  // Routing evaluates each ordered pair independently on the current map.
  mutable std::map<std::pair<int,int>,GroundEdge> ground_edges_;
  // Only reference links retain their checked geometry. New maps revalidate
  // the same segments; ordinary sampled/odometry links retain projection.
  // Keys are canonical undirected pairs and include suspended reference links.
  std::set<std::pair<int,int>> fixed_ground_edges_;
  std::shared_ptr<GraphManager> graph_;
  std::uint64_t snapshot_revision_{0};
  std::vector<StateVec> visited_;
  // Independent of Graph.msg: history/E1 can change without public graph fields.
  std::uint64_t visit_revision_{0};
  std::size_t restored_history_size_{0};
  Vertex* last_odometry_{nullptr};
  // E1 uses its own 1 m displacement threshold, independently of the 0.5 m
  // odometry graph spacing. As in Rrg, the initial marker is the world origin.
  StateVec last_visited_marker_{StateVec::Zero()};
  FrontierUpdate frontier_update_;
  // Only edges previously accepted by the planner may be recovered. IDs are
  // local to this graph generation; reset must clear this store with the graph.
  std::map<std::pair<int,int>,double> suspended_edges_;
  MapEdgeUpdate map_edge_update_;
  // A temporarily blocked former frontier must be reconsidered once free.
  // It is not an active frontier and contributes no stale gain while paused.
  std::set<int> suspended_frontiers_;
  std::unique_ptr<LocalSampler> background_sampler_;
  std::mt19937 cluster_random_;
  static constexpr int kMaxVertices=20000;
  static constexpr int kMaxEdges=100000;
};
}
