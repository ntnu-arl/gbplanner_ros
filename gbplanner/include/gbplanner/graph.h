#ifndef GRAPH_H_
#define GRAPH_H_

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <unordered_map>
//#include <boost/graph/undirected_graph.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/property_map/transform_value_property_map.hpp>

namespace explorer {
namespace gbplanner {

// Result of the Djisktra shortest path calculation, one to all vertices.
struct ShortestPathsReport {
  ShortestPathsReport() : status(false), source_id(0) {}
  void reset() { status = false; }
  bool status;    // False if can not run the shortest path algorithm.
  int source_id;  // ID of the source vertex.
  // Parent ID related to the shortest path from each ID in the id_list.
  std::unordered_map<int, int> parent_id_map;
  // Shortest distance to source corresponding to each id from id_list.
  std::unordered_map<int, double> distance_map;
};

/* Wrapper for the Boost Graph Library.
 */
class Graph {
 public:
  // Edge is set to setS type to avoid duplication in adding new edges.
  typedef boost::adjacency_list<boost::setS, boost::listS, boost::undirectedS,
                                boost::property<boost::vertex_index_t, int>,
                                boost::property<boost::edge_weight_t, double>,
                                boost::no_property>
      GraphType;
  using VertexDescriptor = typename GraphType::vertex_descriptor;
  using EdgeDescriptor = typename GraphType::edge_descriptor;
  using EdgeDescriptorPair = typename std::pair<EdgeDescriptor, bool>;

  Graph();
  ~Graph();

  VertexDescriptor addSourceVertex(int id);
  VertexDescriptor addVertex(int id);
  EdgeDescriptorPair addEdge(int u_id, int v_id, double weight);
  void removeEdge(int u_id, int v_id);

  int getVertexID(VertexDescriptor v);
  int getNumVertices();
  int getNumEdges();

  void getEdgeIterator(
      std::pair<GraphType::edge_iterator, GraphType::edge_iterator> &ei);
  std::tuple<int, int, double> getEdgeProperty(GraphType::edge_iterator e);
  void getVertexIterator(
      std::pair<GraphType::vertex_iterator, GraphType::vertex_iterator> &vi);
  int getVertexProperty(GraphType::vertex_iterator v);

  bool findDijkstraShortestPaths(int src_id, ShortestPathsReport &rep);

  void printResults(const ShortestPathsReport &rep);
  void clear();

 private:
  GraphType graph_;
  VertexDescriptor source_;
  std::unordered_map<int, VertexDescriptor> vertex_descriptors_;
  std::vector<VertexDescriptor> vertices_;
  int num_vertices_;

  bool findDijkstraShortestPaths(VertexDescriptor &source,
                                 std::vector<VertexDescriptor> &shortest_paths,
                                 std::vector<double> &shortest_distances);
};
}  // namespace gbplanner
}  // namespace explorer

#endif
