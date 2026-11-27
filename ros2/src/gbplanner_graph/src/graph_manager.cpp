// Port of planner_common/src/graph_manager.cpp, GBPlanner 2 (BSD-3-Clause).
#include "planner_common/graph_manager.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <set>
#include <stdexcept>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

GraphManager::GraphManager() : kd_tree_(nullptr) { reset(); }
GraphManager::~GraphManager() {
  if (kd_tree_) kd_free(kd_tree_);
  for (auto& item : vertices_map_) delete item.second;
}
void GraphManager::reset() {
  if (kd_tree_) kd_free(kd_tree_);
  kd_tree_ = kd_create(3);
  if (!kd_tree_) throw std::bad_alloc();
  graph_ = std::make_shared<Graph>();
  for (auto& item : vertices_map_) delete item.second;
  vertices_map_.clear(); edge_map_.clear(); local_id_map_.clear();
  subgraph_ind_ = -1; id_count_ = -1;
}
int GraphManager::generateSubgraphIndex() { return ++subgraph_ind_; }
int GraphManager::generateVertexID() { return ++id_count_; }
void GraphManager::addVertex(Vertex* vertex) {
  if (!vertex || vertex->id < 0 || !vertex->state.allFinite() ||
      vertices_map_.count(vertex->id))
    throw std::invalid_argument("Invalid or duplicate vertex");
  if (kd_insert3(kd_tree_, vertex->state.x(), vertex->state.y(),
                 vertex->state.z(), vertex) != 0) throw std::bad_alloc();
  if (vertex->id == 0) graph_->addSourceVertex(0);
  else graph_->addVertex(vertex->id);
  vertices_map_[vertex->id] = vertex;
  id_count_ = std::max(id_count_, vertex->id);
}
void GraphManager::addEdge(Vertex* a, Vertex* b, double weight) {
  if (!a || !b || getVertex(a->id) != a || getVertex(b->id) != b)
    throw std::invalid_argument("Unknown edge endpoint");
  auto result = graph_->addEdge(a->id, b->id, weight);
  if (result.second) {
    edge_map_[a->id].emplace_back(b->id, weight);
    edge_map_[b->id].emplace_back(a->id, weight);
  }
}
void GraphManager::removeEdge(Vertex* a, Vertex* b) {
  if (!a || !b) return;
  graph_->removeEdge(a->id, b->id);
  auto erase_neighbor = [this](int u, int v) {
    auto& edges = edge_map_[u];
    edges.erase(std::remove_if(edges.begin(), edges.end(),
                [v](const auto& e) { return e.first == v; }), edges.end());
  };
  erase_neighbor(a->id, b->id); erase_neighbor(b->id, a->id);
}
bool GraphManager::getNearestVertex(const StateVec* state, Vertex** result) {
  if (!state || !result || !state->allFinite() || !getNumVertices()) return false;
  *result = nullptr;
  auto* nearest = kd_nearest3(kd_tree_, state->x(), state->y(), state->z());
  if (!nearest) return false;
  if (kd_res_size(nearest)) *result = static_cast<Vertex*>(kd_res_item_data(nearest));
  kd_res_free(nearest);
  return *result != nullptr;
}
bool GraphManager::getNearestVertexInRange(const StateVec* state, double range, Vertex** result) {
  if (!std::isfinite(range) || range < 0 || !getNearestVertex(state, result)) return false;
  if ((state->head<3>() - (*result)->state.head<3>()).norm() <= range) return true;
  *result = nullptr; return false;
}
bool GraphManager::getNearestVertices(const StateVec* state, double range, std::vector<Vertex*>* result) {
  if (!result) return false;
  result->clear();
  if (!state || !state->allFinite() || !std::isfinite(range) || range < 0) return false;
  auto* nearest = kd_nearest_range3(kd_tree_, state->x(), state->y(), state->z(), range);
  if (!nearest) return false;
  for (; !kd_res_end(nearest); kd_res_next(nearest))
    result->push_back(static_cast<Vertex*>(kd_res_item_data(nearest)));
  kd_res_free(nearest);
  return !result->empty();
}
bool GraphManager::existVertexInRange(const StateVec* state, double range) {
  Vertex* result = nullptr; return getNearestVertexInRange(state, range, &result);
}
bool GraphManager::findShortestPaths(ShortestPathsReport& report) {
  return findShortestPaths(0, report);
}
bool GraphManager::findShortestPaths(int source, ShortestPathsReport& report) {
  return graph_->findDijkstraShortestPaths(source, report);
}
void GraphManager::getShortestPath(int target, const ShortestPathsReport& report,
                                 bool forward, std::vector<int>& path) {
  path.clear();
  if (!report.status || !report.parent_id_map.count(target)) return;
  int current = target;
  // A disconnected vertex is its own predecessor; a bounded walk also
  // protects callers from stale or externally constructed predecessor maps.
  for (std::size_t count = 0; count <= report.parent_id_map.size(); ++count) {
    path.push_back(current);
    if (current == report.source_id) {
      if (forward) std::reverse(path.begin(), path.end());
      return;
    }
    auto it = report.parent_id_map.find(current);
    if (it == report.parent_id_map.end() || it->second == current) break;
    current = it->second;
  }
  path.clear();
}
void GraphManager::getShortestPath(int target, const ShortestPathsReport& report,
                                 bool forward, std::vector<Vertex*>& path) {
  path.clear(); std::vector<int> ids; getShortestPath(target, report, forward, ids);
  for (int id : ids) { auto* v = getVertex(id); if (!v) { path.clear(); return; } path.push_back(v); }
}
void GraphManager::getShortestPath(int target, const ShortestPathsReport& report,
                                 bool forward, std::vector<StateVec>& path) {
  path.clear(); std::vector<Vertex*> vertices; getShortestPath(target, report, forward, vertices);
  for (auto* v : vertices) path.push_back(v->state);
}
void GraphManager::getShortestPath(int target, const ShortestPathsReport& report,
                                 bool forward, std::vector<Eigen::Vector3d>& path) {
  path.clear(); std::vector<Vertex*> vertices; getShortestPath(target, report, forward, vertices);
  for (auto* v : vertices) path.push_back(v->state.head<3>());
}
double GraphManager::getShortestDistance(int target, const ShortestPathsReport& report) {
  auto it = report.distance_map.find(target);
  return report.status && it != report.distance_map.end() ? it->second : std::numeric_limits<double>::max();
}
int GraphManager::getParentIDFromShortestPath(int target, const ShortestPathsReport& report) {
  auto it = report.parent_id_map.find(target);
  return report.status && it != report.parent_id_map.end() ? it->second : target;
}
void GraphManager::findLeafVertices(const ShortestPathsReport& report) {
  for (auto& item : vertices_map_) item.second->is_leaf_vertex = true;
  if (!report.status) return;
  for (const auto& item : report.parent_id_map)
    if (item.first != item.second && getVertex(item.second)) getVertex(item.second)->is_leaf_vertex = false;
}
void GraphManager::getLeafVertices(std::vector<Vertex*>& vertices) {
  vertices.clear();
  for (auto& item : vertices_map_) if (item.second->is_leaf_vertex) vertices.push_back(item.second);
}
void GraphManager::updateVertexTypeInRange(StateVec& state, double range) {
  std::vector<Vertex*> vertices; getNearestVertices(&state, range, &vertices);
  for (auto* v : vertices) v->type = VertexType::kVisited;
}
void GraphManager::convertGraphToMsg(planner_msgs::msg::Graph& message) {
  message.vertices.clear(); message.edges.clear();
  std::vector<int> ids;
  for (auto& item : vertices_map_) ids.push_back(item.first);
  std::sort(ids.begin(), ids.end());
  for (int id : ids) {
    auto* v = getVertex(id); planner_msgs::msg::Vertex m;
    m.id = id; m.pose.position.x = v->state.x(); m.pose.position.y = v->state.y();
    m.pose.position.z = v->state.z(); m.pose.orientation.z = std::sin(v->state[3] / 2);
    m.pose.orientation.w = std::cos(v->state[3] / 2);
    m.num_unknown_voxels = v->vol_gain.num_unknown_voxels;
    m.num_free_voxels = v->vol_gain.num_free_voxels;
    m.num_occupied_voxels = v->vol_gain.num_occupied_voxels;
    m.is_frontier = v->vol_gain.is_frontier; message.vertices.push_back(m);
  }
  std::pair<Graph::GraphType::edge_iterator, Graph::GraphType::edge_iterator> edges;
  graph_->getEdgeIterator(edges);
  for (auto it = edges.first; it != edges.second; ++it) {
    planner_msgs::msg::Edge edge;
    std::tie(edge.source_id, edge.target_id, edge.weight) = graph_->getEdgeProperty(it);
    message.edges.push_back(edge);
  }
}
void GraphManager::convertMsgToGraph(const planner_msgs::msg::Graph& message) {
  // Construct separately: rejecting an invalid update must leave the old graph usable.
  GraphManager next;
  for (const auto& m : message.vertices) {
    const auto& q = m.pose.orientation;
    const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (!std::isfinite(norm) || norm < 1e-12) throw std::invalid_argument("Invalid quaternion");
    const double yaw = std::atan2(2*(q.w*q.z + q.x*q.y), norm - 2*(q.y*q.y + q.z*q.z));
    StateVec state(m.pose.position.x, m.pose.position.y, m.pose.position.z, yaw);
    auto vertex = std::make_unique<Vertex>(m.id, state);
    vertex->vol_gain.num_unknown_voxels = m.num_unknown_voxels;
    vertex->vol_gain.num_free_voxels = m.num_free_voxels;
    vertex->vol_gain.num_occupied_voxels = m.num_occupied_voxels;
    vertex->vol_gain.is_frontier = m.is_frontier;
    if (m.is_frontier) vertex->type = VertexType::kFrontier;
    next.addVertex(vertex.get()); vertex.release();
  }
  for (const auto& e : message.edges)
    next.addEdge(next.getVertex(e.source_id), next.getVertex(e.target_id), e.weight);
  std::swap(kd_tree_, next.kd_tree_); graph_.swap(next.graph_);
  vertices_map_.swap(next.vertices_map_); edge_map_.swap(next.edge_map_);
  local_id_map_.swap(next.local_id_map_);
  std::swap(id_count_, next.id_count_); std::swap(subgraph_ind_, next.subgraph_ind_);
}
void GraphManager::saveGraph(const std::string& path) {
  planner_msgs::msg::Graph message; convertGraphToMsg(message);
  rclcpp::Serialization<planner_msgs::msg::Graph> serializer;
  rclcpp::SerializedMessage bytes; serializer.serialize_message(&message, &bytes);
  std::ofstream output(path, std::ios::binary);
  if (!output) throw std::runtime_error("Cannot open graph file for writing");
  const auto& raw = bytes.get_rcl_serialized_message();
  output.write("GBP2CDR1", 8);
  output.write(reinterpret_cast<const char*>(raw.buffer), raw.buffer_length);
  output.close();
  if (!output) throw std::runtime_error("Graph write failed");
}
void GraphManager::loadGraph(const std::string& path) {
  std::ifstream input(path, std::ios::binary | std::ios::ate);
  if (!input) throw std::runtime_error("Cannot open graph file");
  const auto length = input.tellg();
  if (length < 12 || length > 64*1024*1024) throw std::runtime_error("Invalid graph file size");
  input.seekg(0); char magic[8]; input.read(magic, 8);
  if (std::memcmp(magic, "GBP2CDR1", 8)) throw std::runtime_error("Not a ROS 2 graph file");
  rclcpp::SerializedMessage bytes(static_cast<std::size_t>(length) - 8);
  auto& raw = bytes.get_rcl_serialized_message(); raw.buffer_length = static_cast<std::size_t>(length) - 8;
  input.read(reinterpret_cast<char*>(raw.buffer), raw.buffer_length);
  if (!input) throw std::runtime_error("Truncated graph file");
  planner_msgs::msg::Graph message;
  rclcpp::Serialization<planner_msgs::msg::Graph> serializer;
  serializer.deserialize_message(&bytes, &message);
  convertMsgToGraph(message);
}
