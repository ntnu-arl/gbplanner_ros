#pragma once

#include <geometry_msgs/Point.h>
#include <ros/message_traits.h>
#include <ros/serialization.h>

#include "planner_common/params.h"
#include "planner_semantic_msgs/SemanticClass.h"

// namespace explorer {

struct SampleStatistic {
  SampleStatistic()
      : num_vertices_fail(0),
        num_edges_fail(0),
        build_graph_time(0),
        compute_exp_gain_time(0),
        shortest_path_time(0),
        evaluate_graph_time(0),
        total_time(0) {}
  StateVec current_state;
  int num_vertices_fail;
  int num_edges_fail;
  // Sampled state in free space, but got rejected due to collision edges.
  std::vector<std::vector<double>> edges_fail;
  double build_graph_time;
  double compute_exp_gain_time;
  double shortest_path_time;
  double evaluate_graph_time;
  double total_time;
  void init(StateVec& state) { current_state = state; }

  void printTime() {
    total_time = build_graph_time + compute_exp_gain_time + shortest_path_time +
                 evaluate_graph_time;
    ROS_INFO_COND(
        global_verbosity >= Verbosity::INFO,
        "Time statistics:\n \
        Build graph    : %3.3f (s)\n \
        Compute gain   : %3.3f (s)\n \
        Dijkstra       : %3.3f (s)\n \
        Evaluate graph : %3.3f (s)\n \
        Total          : %3.3f (s)",
        build_graph_time, compute_exp_gain_time, shortest_path_time,
        evaluate_graph_time, total_time);
  }
  void printTime(std::string title) {
    total_time = build_graph_time + compute_exp_gain_time + shortest_path_time +
                 evaluate_graph_time;
    ROS_INFO_COND(
        global_verbosity >= Verbosity::INFO,
        "Time statistics (%s):\n \
        Build graph    : %3.3f (ms)\n \
        Compute gain   : %3.3f (ms)\n \
        Dijkstra       : %3.3f (ms)\n \
        Evaluate graph : %3.3f (ms)\n \
        Total          : %3.3f (ms)",
        title.c_str(), build_graph_time, compute_exp_gain_time,
        shortest_path_time, evaluate_graph_time, total_time);
  }
};

struct VolumetricGain {
  VolumetricGain()
      : gain(0),
        accumulative_gain(0),
        num_unknown_voxels(0),
        num_free_voxels(0),
        num_occupied_voxels(0),
        num_unknown_surf_voxels(0) {}

  void reset() {
    gain = 0;
    accumulative_gain = 0;
    num_unknown_voxels = 0;
    num_free_voxels = 0;
    num_occupied_voxels = 0;
    num_unknown_surf_voxels = 0;
    is_frontier = false;
    unseen_voxel_hash_keys.clear();
  }

  double gain;
  double accumulative_gain;
  int num_unknown_voxels;
  int num_free_voxels;
  int num_occupied_voxels;
  int num_unknown_surf_voxels;
  std::vector<std::size_t> unseen_voxel_hash_keys;

  bool is_frontier;

  void printGain() {
    std::cout << "Gains: " << gain << ", " << num_unknown_voxels << ", "
              << num_occupied_voxels << ", " << num_free_voxels << std::endl;
  }
};

struct BoundingBoxType {
  void reset() {
    min_val = min_val_default;
    max_val = max_val_default;
  }

  void setDefault(Eigen::Vector3d& v_min, Eigen::Vector3d& v_max) {
    min_val_default = v_min;
    max_val_default = v_max;

    min_val = v_min;
    max_val = v_max;
  }

  // Some interfaces to set/get.
  void set(Eigen::Vector3d& v_min, Eigen::Vector3d& v_max) {
    min_val = v_min;
    max_val = v_max;
  }

  void set(geometry_msgs::Point& p_min, geometry_msgs::Point& p_max,
           bool set_z_val = false) {
    min_val.x() = p_min.x;
    min_val.y() = p_min.y;
    if (set_z_val) min_val.z() = p_min.z;

    max_val.x() = p_max.x;
    max_val.y() = p_max.y;
    if (set_z_val) max_val.z() = p_max.z;
  }

  void get(geometry_msgs::Point& p_min, geometry_msgs::Point& p_max) {
    p_min.x = min_val.x();
    p_min.y = min_val.y();
    p_min.z = min_val.z();
    p_max.x = max_val.x();
    p_max.y = max_val.y();
    p_max.z = max_val.z();
  }

  void get(Eigen::Vector3d& v_min, Eigen::Vector3d& v_max) {
    v_min = min_val;
    v_max = max_val;
  }

 private:
  Eigen::Vector3d min_val;  // [x,y,z] (m)
  Eigen::Vector3d max_val;  // [x,y,z] (m)

  Eigen::Vector3d min_val_default;  // [x,y,z] (m)
  Eigen::Vector3d max_val_default;  // [x,y,z] (m)
};

// Mark the property of the vertex.
enum struct VertexType {
  kUnvisited =
      0,  // Detect as a collision free vertex to visit but hasn't done yet.
  kVisited = 1,  // Free and visisted.
  kFrontier = 2  // Potential frontier vertex to explore, hasn't visited yet.
};

struct Vertex {
  Vertex(int v_id, StateVec v_state) {
    id = v_id;
    state << v_state[0], v_state[1], v_state[2], v_state[3];
    vol_gain.reset();
    parent = NULL;
    distance = 0;
    is_leaf_vertex = true;
    is_hanging = false;
    type = VertexType::kUnvisited;
    cluster_id = 0;
    dm = 0;
    semantic_class.value = planner_semantic_msgs::SemanticClass::kNone;
  }

  // Unique id for each vertex (root: 0, others: positive number).
  int id;
  // State of the vertex: x, y, z, yaw.
  StateVec state;
  // Volumetric gain for exploration.
  VolumetricGain vol_gain;
  // NBVP legacy, keeping for now if wants to build a tree to compare.
  Vertex* parent;
  std::vector<Vertex*> children;
  // Distance to root.
  double distance;
  // Set true if this is a leaf in the simplified tree from the graph.
  bool is_leaf_vertex;
  // If there is ground plane below the vertex (only for legged robots)
  bool is_hanging;
  // Type of vertex, used for global graph.
  VertexType type;
  // Cluster id: used for clustering.
  int cluster_id;
  // Distance to closest obstacle in the map
  double dm;
  // Semantic
  planner_semantic_msgs::SemanticClass semantic_class;
};

struct SerializeVertex {
  SerializeVertex() {
    // vol_gain.reset();
    // type = 1;
  }

  // Unique id for each vertex (root: 0, others: positive number).
  int id;
  // State of the vertex: x, y, z, yaw.
  StateVec state;
  // Volumetric gain for exploration.
  VolumetricGain vol_gain;
  // NBVP legacy, keeping for now if wants to build a tree to compare.
  int parent_id;
  // kFrontier=0, kUnvisited=1, kVisited=2
  int type;

  void printit() {
    std::cout << "x: " << this->state[0] << " y: " << this->state[1]
              << " z: " << this->state[2] << std::endl;
    std::cout << "ID: " << id << " Paren: " << parent_id << std::endl;
  }
};

enum class ExpandGraphStatus {
  kSuccess = 0,
  kErrorKdTree,
  kErrorCollisionEdge,
  kErrorShortEdge,
  kErrorGeofenceViolated,
  kNull
};

struct ExpandGraphReport {
  ExpandGraphReport()
      : status(ExpandGraphStatus::kNull),
        num_vertices_added(0),
        num_edges_added(0),
        vertex_added(NULL) {}
  ExpandGraphStatus status;
  int num_vertices_added;
  int num_edges_added;
  Vertex* vertex_added;
};

enum class ConnectStatus {
  kSuccess = 0,
  kErrorCollisionAtSource,  // Failed to connect due to collision at source.
  kErrorNoFeasiblePath      // Could not find any feasible path from source to
                            // target
};
struct RandomSamplingParams {
  RandomSamplingParams() {
    // All magic numbers.
    num_vertices_max = 500;
    num_edges_max = 10000;
    num_loops_cutoff = 2000;
    num_loops_max = 100000;
    reached_target_radius = 2.0;
    check_collision_at_source = true;
    num_paths_to_target_max = 5;
  }
  int num_vertices_max;
  int num_edges_max;
  double reached_target_radius;
  bool check_collision_at_source;
  int num_paths_to_target_max;
  double num_loops_cutoff;
  double num_loops_max;
};

/* Serialization of structs */
namespace ros {
namespace serialization {
// VolumetricGain
template <>
struct Serializer<VolumetricGain> {
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m) {
    stream.next(m.gain);
    stream.next(m.accumulative_gain);
    stream.next(m.num_unknown_voxels);
    stream.next(m.num_free_voxels);
    stream.next(m.num_occupied_voxels);
    stream.next(m.is_frontier);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};
// StateVec
template <>
struct Serializer<StateVec> {
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m) {
    stream.next(m[0]);
    stream.next(m[1]);
    stream.next(m[2]);
    stream.next(m[3]);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};

// Serialize Vertex
template <>
struct Serializer<SerializeVertex> {
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m) {
    stream.next(m.id);
    stream.next(m.state);
    stream.next(m.vol_gain);
    stream.next(m.parent_id);
    stream.next(m.type);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};

}  // namespace serialization
}  // namespace ros
