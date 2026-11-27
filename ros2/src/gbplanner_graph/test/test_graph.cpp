#include <gtest/gtest.h>
#include <fstream>
#include <filesystem>
#include <random>
#include <rclcpp/serialization.hpp>
#include "planner_common/graph_manager.h"

static Vertex* add(GraphManager& g, int id, double x, double y = 0, double yaw = 0) {
  auto p = std::make_unique<Vertex>(id, StateVec(x, y, 0, yaw));
  g.addVertex(p.get()); return p.release();
}
TEST(Graph, WeightedRouteAndDisconnected) {
  GraphManager g; auto a=add(g,0,0), b=add(g,1,1), c=add(g,2,2); add(g,3,9);
  g.addEdge(a,b,1); g.addEdge(b,c,1); g.addEdge(a,c,5);
  ShortestPathsReport r; ASSERT_TRUE(g.findShortestPaths(r));
  std::vector<int> p; g.getShortestPath(2,r,true,p);
  EXPECT_EQ(p,(std::vector<int>{0,1,2})); EXPECT_DOUBLE_EQ(g.getShortestDistance(2,r),2);
  g.getShortestPath(3,r,true,p); EXPECT_TRUE(p.empty());
  g.getShortestPath(99,r,true,p); EXPECT_TRUE(p.empty());
}
TEST(Graph, SparseIdsAndNonzeroSource) {
  GraphManager g; auto a=add(g,10,0), b=add(g,42,1), c=add(g,100,2);
  g.addEdge(a,b,1);g.addEdge(b,c,2);
  ShortestPathsReport r; ASSERT_TRUE(g.findShortestPaths(100,r));
  std::vector<int> p; g.getShortestPath(10,r,true,p);
  EXPECT_EQ(p,(std::vector<int>{100,42,10})); EXPECT_DOUBLE_EQ(r.distance_map.at(10),3);
}
TEST(Graph, EmptySingleVertexAndInvalidSource) {
  GraphManager g; ShortestPathsReport r; EXPECT_FALSE(g.findShortestPaths(r));
  add(g,42,0); ASSERT_TRUE(g.findShortestPaths(42,r));
  std::vector<int> p; g.getShortestPath(42,r,true,p); EXPECT_EQ(p,(std::vector<int>{42}));
  EXPECT_FALSE(g.findShortestPaths(99,r)); EXPECT_TRUE(r.parent_id_map.empty());
}
TEST(Graph, EdgeRemovalUpdatesBothRepresentations) {
  GraphManager g; auto a=add(g,0,0),b=add(g,1,1),c=add(g,2,2);
  g.addEdge(a,b,1);g.addEdge(a,b,1);g.addEdge(b,c,1);g.addEdge(a,c,5);
  EXPECT_EQ(g.getNumEdges(),3); EXPECT_EQ(g.edge_map_[0].size(),2u);
  g.removeEdge(b,c); EXPECT_EQ(g.edge_map_[1].size(),1u);
  ShortestPathsReport r; g.findShortestPaths(r); EXPECT_DOUBLE_EQ(r.distance_map.at(2),5);
}
TEST(Graph, RejectNegativeAndNanWeights) {
  GraphManager g; auto a=add(g,0,0),b=add(g,1,1);
  EXPECT_THROW(g.addEdge(a,b,-1),std::invalid_argument);
  EXPECT_THROW(g.addEdge(a,b,std::numeric_limits<double>::quiet_NaN()),std::invalid_argument);
  EXPECT_THROW(g.addEdge(a,nullptr,1),std::invalid_argument);
  EXPECT_EQ(g.getNumEdges(),0);
}
TEST(Graph, NearestAndRadius) {
  GraphManager g; add(g,0,0);add(g,1,2);add(g,2,5);
  StateVec q(2.1,0,0,0); Vertex* v=nullptr;
  ASSERT_TRUE(g.getNearestVertex(&q,&v));EXPECT_EQ(v->id,1);
  EXPECT_FALSE(g.getNearestVertexInRange(&q,0.01,&v)); EXPECT_EQ(v,nullptr);
  std::vector<Vertex*> all; ASSERT_TRUE(g.getNearestVertices(&q,2.2,&all)); EXPECT_EQ(all.size(),2u);
  q.x()=100; EXPECT_FALSE(g.getNearestVertices(&q,0.1,&all)); EXPECT_TRUE(all.empty());
}
TEST(Graph, MessageRoundTripAndNoAccumulation) {
  GraphManager g; auto a=add(g,8,0,1,0.7),b=add(g,19,2,3,-0.4); g.addEdge(a,b,2.5);
  a->vol_gain.is_frontier=true;a->vol_gain.num_unknown_voxels=7;
  planner_msgs::msg::Graph msg;g.convertGraphToMsg(msg);g.convertGraphToMsg(msg);
  ASSERT_EQ(msg.vertices.size(),2u);ASSERT_EQ(msg.edges.size(),1u);
  GraphManager h;h.convertMsgToGraph(msg);EXPECT_NEAR(h.getVertex(8)->state[3],0.7,1e-12);
  EXPECT_TRUE(h.getVertex(8)->vol_gain.is_frontier);EXPECT_EQ(h.getVertex(8)->vol_gain.num_unknown_voxels,7);
  EXPECT_EQ(h.generateVertexID(),20);
}
TEST(Graph, InvalidMessagePreservesPreviousGraph) {
  GraphManager g;add(g,0,0);
  planner_msgs::msg::Graph msg;g.convertGraphToMsg(msg);msg.vertices.push_back(msg.vertices[0]);
  EXPECT_THROW(g.convertMsgToGraph(msg),std::invalid_argument);EXPECT_EQ(g.getNumVertices(),1);
  msg.vertices.resize(1); planner_msgs::msg::Edge edge;edge.source_id=0;edge.target_id=7;edge.weight=1;msg.edges.push_back(edge);
  EXPECT_THROW(g.convertMsgToGraph(msg),std::invalid_argument);EXPECT_EQ(g.getNumVertices(),1);
}
TEST(Graph, CdrFileAndInvalidFile) {
  GraphManager g;auto a=add(g,0,0),b=add(g,4,1);g.addEdge(a,b,1);
  const auto file=(std::filesystem::temp_directory_path()/"gbplanner_graph_test.cdr").string();
  g.saveGraph(file);GraphManager h;h.loadGraph(file);EXPECT_EQ(h.getNumVertices(),2);
  ShortestPathsReport r;ASSERT_TRUE(h.findShortestPaths(r));EXPECT_DOUBLE_EQ(r.distance_map.at(4),1);
  { std::ofstream bad(file);bad<<"not a valid graph"; }
  EXPECT_THROW(h.loadGraph(file),std::runtime_error); EXPECT_EQ(h.getNumVertices(),2);
  std::filesystem::remove(file);
  EXPECT_THROW(h.loadGraph(file),std::runtime_error);
}
TEST(Graph, RepeatedResetAndLeafRecalculation) {
  GraphManager g;for(int i=0;i<30;++i){auto a=add(g,0,0),b=add(g,1,1);g.addEdge(a,b,1);
    ShortestPathsReport r;g.findShortestPaths(r);g.findLeafVertices(r);
    EXPECT_FALSE(a->is_leaf_vertex);EXPECT_TRUE(b->is_leaf_vertex);
    g.reset();EXPECT_EQ(g.getNumVertices(),0);}
}
TEST(Graph, RandomGraphsMatchIndependentFloydWarshall) {
  std::mt19937 rng(42);std::uniform_real_distribution<double> weights(0.1,10);
  for(int run=0;run<20;++run){
    const int n=12; GraphManager g;std::vector<Vertex*> vertices;
    std::vector<std::vector<double>> d(n,std::vector<double>(n,1e20));
    for(int i=0;i<n;++i){vertices.push_back(add(g,i*3+5,i));d[i][i]=0;}
    for(int i=0;i<n;++i)for(int j=i+1;j<n;++j)if(rng()%3==0){double w=weights(rng);g.addEdge(vertices[i],vertices[j],w);d[i][j]=d[j][i]=w;}
    for(int k=0;k<n;++k)for(int i=0;i<n;++i)for(int j=0;j<n;++j)d[i][j]=std::min(d[i][j],d[i][k]+d[k][j]);
    for(int i=0;i<n;++i){ShortestPathsReport r;ASSERT_TRUE(g.findShortestPaths(i*3+5,r));
      for(int j=0;j<n;++j){double actual=g.getShortestDistance(j*3+5,r);
        if(d[i][j]<1e19)EXPECT_NEAR(actual,d[i][j],1e-10);else EXPECT_GT(actual,1e19);}}
  }
}
