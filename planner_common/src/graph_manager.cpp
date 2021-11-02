#include "planner_common/graph_manager.h"

#include <ros/message_traits.h>
#include <ros/serialization.h>

#include "ros/ros.h"

GraphManager::GraphManager() {
  kd_tree_ = NULL;
  reset();
}

void GraphManager::reset() {
  // Reset kdtree first.
  if (kd_tree_) kd_free(kd_tree_);
  kd_tree_ = kd_create(3);

  // Reset graph.
  graph_.reset(new Graph());

  // Vertex mapping.
  // Clear memory for all vertex pointers
  if (vertices_map_.size() > 0) {
    for (int i = 0; i < vertices_map_.size(); ++i) {
      delete vertices_map_[i];
    }
  }
  vertices_map_.clear();
  edge_map_.clear();

  // Other params.
  subgraph_ind_ = -1;
  id_count_ = -1;
}

void GraphManager::addVertex(Vertex* v) {
  kd_insert3(kd_tree_, v->state.x(), v->state.y(), v->state.z(), v);
  if (v->id == 0)
    graph_->addSourceVertex(0);
  else
    graph_->addVertex(v->id);
  vertices_map_[v->id] = v;
}

void GraphManager::addEdge(Vertex* v, Vertex* u, double weight) {
  graph_->addEdge(v->id, u->id, weight);
  edge_map_[v->id].push_back(std::make_pair(u->id, weight));
  edge_map_[u->id].push_back(std::make_pair(v->id, weight));
}

void GraphManager::removeEdge(Vertex* v, Vertex* u) {
  graph_->removeEdge(v->id, u->id);
}

bool GraphManager::getNearestVertex(const StateVec* state, Vertex** v_res) {
  if (getNumVertices() <= 0) return false;
  kdres* nearest = kd_nearest3(kd_tree_, state->x(), state->y(), state->z());
  if (kd_res_size(nearest) <= 0) {
    kd_res_free(nearest);
    return false;
  }
  *v_res = (Vertex*)kd_res_item_data(nearest);
  kd_res_free(nearest);
  return true;
}

bool GraphManager::getNearestVertexInRange(const StateVec* state, double range,
                                           Vertex** v_res) {
  if (getNumVertices() <= 0) return false;
  kdres* nearest = kd_nearest3(kd_tree_, state->x(), state->y(), state->z());
  if (kd_res_size(nearest) <= 0) {
    kd_res_free(nearest);
    return false;
  }
  *v_res = (Vertex*)kd_res_item_data(nearest);
  Eigen::Vector3d dist;
  dist << state->x() - (*v_res)->state.x(), state->y() - (*v_res)->state.y(),
      state->z() - (*v_res)->state.z();
  kd_res_free(nearest);
  if (dist.norm() > range) return false;
  return true;
}

bool GraphManager::getNearestVertices(const StateVec* state, double range,
                                      std::vector<Vertex*>* v_res) {
  // Notice that this might include the same vertex in the result.
  // if that vertex is added to the tree before.
  // Use the distance 0 or small threshold to filter out.
  kdres* neighbors =
      kd_nearest_range3(kd_tree_, state->x(), state->y(), state->z(), range);
  int neighbors_size = kd_res_size(neighbors);
  if (neighbors_size <= 0) return false;
  v_res->clear();
  for (int i = 0; i < neighbors_size; ++i) {
    Vertex* new_neighbor = (Vertex*)kd_res_item_data(neighbors);
    v_res->push_back(new_neighbor);
    if (kd_res_next(neighbors) <= 0) break;
  }
  kd_res_free(neighbors);
  return true;
}

bool GraphManager::findShortestPaths(ShortestPathsReport& rep) {
  return graph_->findDijkstraShortestPaths(0, rep);
}

bool GraphManager::findShortestPaths(int source_id, ShortestPathsReport& rep) {
  return graph_->findDijkstraShortestPaths(source_id, rep);
}

void GraphManager::getShortestPath(int target_id,
                                   const ShortestPathsReport& rep,
                                   bool source_to_target_order,
                                   std::vector<Vertex*>& path) {
  std::vector<int> path_id;
  getShortestPath(target_id, rep, source_to_target_order, path_id);
  for (auto p = path_id.begin(); p != path_id.end(); ++p) {
    path.push_back(vertices_map_[*p]);
  }
}

void GraphManager::getShortestPath(int target_id,
                                   const ShortestPathsReport& rep,
                                   bool source_to_target_order,
                                   std::vector<Eigen::Vector3d>& path) {
  std::vector<int> path_id;
  getShortestPath(target_id, rep, source_to_target_order, path_id);
  for (auto p = path_id.begin(); p != path_id.end(); ++p) {
    path.emplace_back(Eigen::Vector3d(vertices_map_[*p]->state.x(),
                                      vertices_map_[*p]->state.y(),
                                      vertices_map_[*p]->state.z()));
  }
}

void GraphManager::getShortestPath(int target_id,
                                   const ShortestPathsReport& rep,
                                   bool source_to_target_order,
                                   std::vector<StateVec>& path) {
  std::vector<int> path_id;
  getShortestPath(target_id, rep, source_to_target_order, path_id);
  for (auto p = path_id.begin(); p != path_id.end(); ++p) {
    path.emplace_back(vertices_map_[*p]->state);
  }
}

void GraphManager::getShortestPath(int target_id,
                                   const ShortestPathsReport& rep,
                                   bool source_to_target_order,
                                   std::vector<int>& path) {
  path.clear();
  if (!rep.status) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Shortest paths report is not valid");
    return;
  }

  if (rep.parent_id_map.size() <= target_id) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Vertext with ID [%d] doesn't exist in the graph", target_id);
    return;
  }

  if (target_id == rep.source_id) {
    path.push_back(target_id);
    return;
  }

  int parent_id = rep.parent_id_map.at(target_id);
  if (parent_id == target_id) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Vertex with ID [%d] is isolated from the graph", target_id);
    return;
  }

  path.push_back(target_id);  // current vertex id first
  path.push_back(
      parent_id);  // its first parent, the rest is recursively looked up
  while (parent_id != rep.source_id) {
    parent_id = rep.parent_id_map.at(parent_id);
    path.push_back(parent_id);
  }

  // Initially, the path follows target to source order. Reverse if required.
  if (source_to_target_order) {
    std::reverse(path.begin(), path.end());
  }
}

double GraphManager::getShortestDistance(int target_id,
                                         const ShortestPathsReport& rep) {
  double dist = std::numeric_limits<double>::max();

  if (!rep.status) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Shortest paths report is not valid");
    return dist;
  }
  if (rep.parent_id_map.size() <= target_id) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Vertex with ID [%d] doesn't exist in the graph", target_id);
    return dist;
  }
  dist = rep.distance_map.at(target_id);
  return dist;
}

int GraphManager::getParentIDFromShortestPath(int target_id,
                                              const ShortestPathsReport& rep) {
  if (!rep.status) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Shortest paths report is not valid");
    return target_id;
  }

  if (rep.parent_id_map.size() <= target_id) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Vertex with ID [%d] doesn't exist in the graph", target_id);
    return target_id;
  }

  return rep.parent_id_map.at(target_id);
}

void GraphManager::getLeafVertices(std::vector<Vertex*>& leaf_vertices) {
  leaf_vertices.clear();
  for (int id = 0; id < getNumVertices(); ++id) {
    Vertex* v = getVertex(id);
    if (v->is_leaf_vertex) leaf_vertices.push_back(v);
  }
}

void GraphManager::findLeafVertices(const ShortestPathsReport& rep) {
  int num_vertices = getNumVertices();
  for (int id = 0; id < num_vertices; ++id) {
    int pid = getParentIDFromShortestPath(id, rep);
    getVertex(pid)->is_leaf_vertex = false;
  }
}

int GraphManager::generateSubgraphIndex() { return ++subgraph_ind_; }

int GraphManager::generateVertexID() { return ++id_count_; }

void GraphManager::updateVertexTypeInRange(StateVec& state, double range) {
  std::vector<Vertex*> nearest_vertices;
  getNearestVertices(&state, range, &nearest_vertices);
  for (auto& v : nearest_vertices) {
    v->type = VertexType::kVisited;
  }
}

void GraphManager::convertGraphToMsg(planner_msgs::Graph& graph_msg) {
  // Get all the vertices
  for (auto& v : vertices_map_) {
    planner_msgs::Vertex vertex;
    vertex.id = v.second->id;

    // convertStateToPoseMsg
    vertex.pose.position.x = v.second->state[0];
    vertex.pose.position.y = v.second->state[1];
    vertex.pose.position.z = v.second->state[2];
    double yawhalf = v.second->state[3] * 0.5;
    vertex.pose.orientation.x = 0.0;
    vertex.pose.orientation.y = 0.0;
    vertex.pose.orientation.z = sin(yawhalf);
    vertex.pose.orientation.w = cos(yawhalf);

    vertex.num_unknown_voxels = v.second->vol_gain.num_unknown_voxels;
    vertex.num_occupied_voxels = v.second->vol_gain.num_occupied_voxels;
    vertex.num_free_voxels = v.second->vol_gain.num_free_voxels;
    vertex.is_frontier = v.second->vol_gain.is_frontier;
    graph_msg.vertices.emplace_back(vertex);
  }

  // Get all the edges through iterator of the boost graph lib.
  std::pair<Graph::GraphType::edge_iterator, Graph::GraphType::edge_iterator>
      ei;
  graph_->getEdgeIterator(ei);
  for (Graph::GraphType::edge_iterator it = ei.first; it != ei.second; ++it) {
    planner_msgs::Edge edge;
    std::tie(edge.source_id, edge.target_id, edge.weight) =
        graph_->getEdgeProperty(it);
    graph_msg.edges.emplace_back(edge);
  }
}

void GraphManager::convertMsgToGraph(const planner_msgs::Graph& graph_msg) {
  // Add all the vertices first
  for (auto& v : graph_msg.vertices) {
    generateVertexID();  // must call this one to increase the count inside
    StateVec state;
    state[0] = v.pose.position.x;
    state[1] = v.pose.position.y;
    state[2] = v.pose.position.z;
    state[3] = tf::getYaw(v.pose.orientation);
    Vertex* vertex = new Vertex(v.id, state);
    // Copy other info
    vertex->vol_gain.num_unknown_voxels = v.num_unknown_voxels;
    vertex->vol_gain.num_occupied_voxels = v.num_occupied_voxels;
    vertex->vol_gain.num_free_voxels = v.num_free_voxels;
    vertex->vol_gain.is_frontier = v.is_frontier;
    if (v.is_frontier) vertex->type = VertexType::kFrontier;
    addVertex(vertex);
  }

  // Add all edges
  for (auto& e : graph_msg.edges) {
    addEdge(getVertex(e.source_id), getVertex(e.target_id), e.weight);
  }
}

void GraphManager::saveGraph(const std::string& path) {
  // Convert data to the msg type first
  planner_msgs::Graph graph_msg;
  convertGraphToMsg(graph_msg);

  // Serialize the data, then write to a file
  uint32_t serial_size = ros::serialization::serializationLength(graph_msg);
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  ros::serialization::OStream stream(buffer.get(), serial_size);
  ros::serialization::serialize(stream, graph_msg);

  // Write to a file
  std::ofstream wrt_file(path, std::ios::out | std::ios::binary);
  wrt_file.write((char*)buffer.get(), serial_size);
  wrt_file.close();
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Save the graph with [%d] vertices and [%d] edges to a file: %s",
           getNumVertices(), getNumEdges(), path.c_str());
}

void GraphManager::loadGraph(const std::string& path) {
  // Fill buffer with a serialized UInt32
  std::ifstream read_file(path, std::ifstream::binary);
  // Get the total number of bytes:
  // http://www.cplusplus.com/reference/fstream/ifstream/rdbuf/
  std::filebuf* pbuf = read_file.rdbuf();
  std::size_t size = pbuf->pubseekoff(0, read_file.end, read_file.in);
  pbuf->pubseekpos(0, read_file.in);
  // Read the whole file to a buffer
  boost::shared_array<uint8_t> buffer(new uint8_t[size]);
  // char* buffer1=new char[size];
  pbuf->sgetn((char*)buffer.get(), size);
  read_file.close();

  // Deserialize data into msg
  planner_msgs::Graph graph_msg;
  ros::serialization::IStream stream_in(buffer.get(), size);
  ros::serialization::deserialize(stream_in, graph_msg);

  // Reconstruct the graph
  reset();
  convertMsgToGraph(graph_msg);
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Load the graph with [%d] vertices and [%d] edges from a file: %s",
           getNumVertices(), getNumEdges(), path.c_str());
}