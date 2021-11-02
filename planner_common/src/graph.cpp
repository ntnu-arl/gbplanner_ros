#include "planner_common/graph.h"
// #include "planner_common/graph_base.h"

// namespace explorer {

Graph::Graph() {}
Graph::~Graph() {}

Graph::VertexDescriptor Graph::addSourceVertex(int id) {
  source_ = boost::add_vertex(id, graph_);
  vertex_descriptors_[id] = source_;
  return source_;
}

Graph::VertexDescriptor Graph::addVertex(int id) {
  VertexDescriptor v = boost::add_vertex(id, graph_);
  vertex_descriptors_[id] = v;
  return v;
}

Graph::EdgeDescriptorPair Graph::addEdge(int u_id, int v_id, double weight) {
  if (vertex_descriptors_.find(u_id) == vertex_descriptors_.end()) {
    addVertex(u_id);
  }
  if (vertex_descriptors_.find(v_id) == vertex_descriptors_.end()) {
    addVertex(v_id);
  }
  return boost::add_edge(vertex_descriptors_[u_id], vertex_descriptors_[v_id],
                         weight, graph_);
}

void Graph::removeEdge(int u_id, int v_id) {
  if (vertex_descriptors_.find(u_id) == vertex_descriptors_.end()) {
    return;
  }
  if (vertex_descriptors_.find(v_id) == vertex_descriptors_.end()) {
    return;
  }

  boost::remove_edge(vertex_descriptors_[u_id], vertex_descriptors_[v_id],
                     graph_);
}

bool Graph::findDijkstraShortestPaths(int src_id, ShortestPathsReport& rep) {
  rep.source_id = src_id;
  rep.status = false;

  if (vertex_descriptors_.size() <= src_id) {
    std::cout << "Source index [" << src_id << "] is not in the graph."
              << std::endl;
    return false;
  }

  std::vector<VertexDescriptor> shortest_paths;
  std::vector<double> shortest_distances;
  if (findDijkstraShortestPaths(vertex_descriptors_[src_id], shortest_paths,
                                shortest_distances)) {
    rep.status = true;
    rep.parent_id_map.clear();
    rep.distance_map.clear();
    int num_vertices = getNumVertices();
    for (int ind = 0; ind < num_vertices; ++ind) {
      rep.parent_id_map[ind] = getVertexID(shortest_paths[ind]);
      rep.distance_map[ind] = shortest_distances[ind];
    }
  }

  return rep.status;
}

bool Graph::findDijkstraShortestPaths(
    VertexDescriptor& source, std::vector<VertexDescriptor>& shortest_paths,
    std::vector<double>& shortest_distances) {
  num_vertices_ = boost::num_vertices(graph_);
  if (num_vertices_ < 2) return false;

  shortest_paths.clear();
  shortest_distances.clear();
  shortest_paths.resize(num_vertices_);
  shortest_distances.resize(num_vertices_);

  auto v_index = boost::get(boost::vertex_index, graph_);
  auto weight = boost::get(boost::edge_weight, graph_);
  boost::dijkstra_shortest_paths(
      graph_, source,
      boost::predecessor_map(
          boost::make_iterator_property_map(shortest_paths.begin(),
                                            get(boost::vertex_index, graph_)))
          .distance_map(boost::make_iterator_property_map(
              shortest_distances.begin(), get(boost::vertex_index, graph_))));
  return true;
}

int Graph::getVertexID(VertexDescriptor v) {
  return boost::get(boost::get(boost::vertex_index, graph_), v);
}

int Graph::getNumVertices() { return boost::num_vertices(graph_); }

int Graph::getNumEdges() { return boost::num_edges(graph_); }

void Graph::printResults(const ShortestPathsReport& rep) {
  std::cout << "Shortest paths:" << std::endl;
  for (int i = 0; i < num_vertices_; ++i) {
    int id = rep.parent_id_map.at(i);
    std::cout << "Path: "
              << "[cost: " << rep.distance_map.at(i) << "] " << i << "<-" << id;
    if (id == 0) {
    } else if (id == i)
      std::cout << "[isolated-vertex]";
    else {
      int id_parent = id;
      while (id_parent != 0) {
        id_parent = rep.parent_id_map.at(id_parent);
        std::cout << "<-" << id_parent;
      }
    }
    std::cout << std::endl;
  }
}

void Graph::getEdgeIterator(
    std::pair<GraphType::edge_iterator, GraphType::edge_iterator>& ei) {
  ei = boost::edges(graph_);
}

std::tuple<int, int, double> Graph::getEdgeProperty(
    GraphType::edge_iterator e) {
  return std::make_tuple(
      getVertexID(boost::source(*e, graph_)),
      getVertexID(boost::target(*e, graph_)),
      boost::get(boost::get(boost::edge_weight, graph_), *e));
}

void Graph::getVertexIterator(
    std::pair<GraphType::vertex_iterator, GraphType::vertex_iterator>& vi) {
  vi = boost::vertices(graph_);
}

int Graph::getVertexProperty(GraphType::vertex_iterator v) {
  return boost::get(boost::get(boost::vertex_index, graph_), *v);
}