#include <gtest/gtest.h>
#include "gbplanner_local/exploration_direction.hpp"
#include "gbplanner_local/local_planner.hpp"
#include "gbplanner_local/trajectory.hpp"
#include "gbplanner_local/path_heading.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

using namespace gbplanner2;
namespace {
using Path=std::vector<Eigen::Vector3d>;

// Independent transcription of the pinned original trajectory.cpp:74-102,
// 151-179 and 262-288. It deliberately calls no migrated trajectory helpers.
double originalDirection(const Path& path,double heading,double step=.2,bool scale=true){
  if(path.empty()||step<=0)return std::numeric_limits<double>::max();
  Path points;
  if(path.size()==1)points.push_back(path[0]);
  for(std::size_t i=0;i+1<path.size();++i){
    const Eigen::Vector3d segment=path[i+1]-path[i];const double length=segment.norm();
    if(std::abs(length)<.01)points.push_back(path[i]);
    else{
      const int count=static_cast<int>(length/step);
      const Eigen::Vector3d delta=segment/length*step;
      for(int j=0;j<=count;++j)points.push_back(path[i]+j*delta);
    }
  }
  double length=0.;for(std::size_t i=1;i<points.size();++i)length+=(points[i]-points[i-1]).norm();
  if(length==0)return std::numeric_limits<double>::max();
  const Eigen::Vector3d delta=Eigen::AngleAxisd(heading,Eigen::Vector3d::UnitZ())*Eigen::Vector3d(1,0,0)*step;
  const int count=static_cast<int>(length/step);
  Path reference;for(int i=0;i<count;++i)reference.push_back(path[0]+delta*count);
  std::vector<std::vector<double>> cost(points.size()+1,
      std::vector<double>(reference.size()+1,std::numeric_limits<double>::infinity()));
  cost[0][0]=0.;
  for(std::size_t i=1;i<=points.size();++i)for(std::size_t j=1;j<=reference.size();++j)
    cost[i][j]=(points[i-1]-reference[j-1]).norm()+
        std::min({cost[i-1][j-1],cost[i-1][j],cost[i][j-1]});
  double result=cost.back().back();if(scale)result/=(length*length);return result;
}

class UniformGainMap:public MapQuery{
 public:
  double resolution()const override{return .25;}
  Eigen::Vector3d minimum()const override{return {-20,-20,-20};}
  Eigen::Vector3d maximum()const override{return {20,20,20};}
  VoxelStatus voxel(const Eigen::Vector3d&)const override{return VoxelStatus::Free;}
  VoxelStatus box(const Eigen::Vector3d&,const Eigen::Vector3d&)const override{return VoxelStatus::Free;}
  VoxelStatus path(const Eigen::Vector3d&,const Eigen::Vector3d&,const Eigen::Vector3d&)const override{return VoxelStatus::Free;}
  void ray(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
      const std::function<bool(const Eigen::Vector3d&,VoxelStatus)>& visit)const override{
    visit((a+b)*.5,VoxelStatus::Unknown);
  }
};
class EastUnknownGainMap final:public UniformGainMap{
 public:
  void ray(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
      const std::function<bool(const Eigen::Vector3d&,VoxelStatus)>& visit)const override{
    // A directional observation distinguishes gain evaluation before and after
    // reversing yaw; all geometry remains traversable for both planners.
    visit((a+b)*.5,b.x()-a.x()>.5?VoxelStatus::Unknown:VoxelStatus::Free);
  }
};
Parameters example(){
  Parameters p;p.max_vertices=60;p.max_attempts=500;p.seed=42;
  p.local_min={-2.,-2.,-.1};p.local_max={2.,2.,.1};
  p.horizontal_fov=.1;p.vertical_fov=.1;p.horizontal_resolution=.1;p.vertical_resolution=.1;
  return p;
}
struct Expected {int id{-1};double score{0},raw{0},distance{0};};
Expected originalWinner(const LocalPlanner& planner,const Parameters& p,double heading){
  const auto& source=planner.graph();GraphManager graph;
  // Copy the in-memory graph directly: Graph.msg stores edge weights as
  // float32, which changes shortest-path costs before the reference scoring.
  std::vector<int> ids;
  for(const auto& item:source.vertices_map_)ids.push_back(item.first);
  std::sort(ids.begin(),ids.end());
  for(int id:ids){
    const auto* original=source.vertices_map_.at(id);
    auto* vertex=new Vertex(id,original->state);vertex->vol_gain=original->vol_gain;
    graph.addVertex(vertex);
  }
  std::pair<Graph::GraphType::edge_iterator,Graph::GraphType::edge_iterator> edges;
  source.graph_->getEdgeIterator(edges);
  for(auto edge=edges.first;edge!=edges.second;++edge){
    const auto [from,to,weight]=source.graph_->getEdgeProperty(edge);
    graph.addEdge(graph.getVertex(from),graph.getVertex(to),weight);
  }
  ShortestPathsReport report;graph.findShortestPaths(report);graph.findLeafVertices(report);Expected best;
  for(int id=1;id<graph.getNumVertices();++id){
    if(!graph.getVertex(id)->is_leaf_vertex)continue;
    std::vector<Vertex*> vertices;graph.getShortestPath(id,report,true,vertices);
    double raw=0.;Path path;
    for(const auto* vertex:vertices){
      // Recompute scalar gain independently from the stored voxel counts.
      const auto& counts=vertex->vol_gain;
      const double gain=counts.num_unknown_voxels*p.unknown_gain+
          counts.num_free_voxels*p.free_gain+counts.num_occupied_voxels*p.occupied_gain;
      raw+=gain*std::exp(-p.path_length_penalty*graph.getShortestDistance(vertex->id,report));
      path.push_back(vertex->state.head<3>());
    }
    const double distance=originalDirection(path,heading);
    const double score=p.path_direction_penalty==0?raw:raw*std::exp(-p.path_direction_penalty*distance);
    if(score>best.score)best={id,score,raw,distance};
  }
  return best;
}
}

TEST(PathHeading,HomingUsesInitialHeadingAndRetainsUpstreamShortPathFallback){
  std::vector<StateVec> forward{{0,0,0,0},{3,0,1,1},{3,2,1,1}};
  applyHomingPathHeading(forward,true,true);
  EXPECT_DOUBLE_EQ(forward.front()[3],0.);
  EXPECT_DOUBLE_EQ(forward[1][3],0.);
  EXPECT_NEAR(forward[2][3],M_PI/2,1e-12);

  std::vector<StateVec> retreat{{0,0,0,0},{-3,0,1,1},{-3,-2,1,1}};
  applyHomingPathHeading(retreat,true,true);
  EXPECT_DOUBLE_EQ(retreat.front()[3],0.);
  EXPECT_NEAR(retreat[1][3],0.,1e-12);
  EXPECT_NEAR(retreat[2][3],M_PI/2,1e-12);
  applyHomingPathHeading(retreat,true,false);
  EXPECT_NEAR(std::abs(retreat[1][3]),M_PI,1e-12);
  EXPECT_NEAR(retreat[2][3],-M_PI/2,1e-12);

  // Strictly greater than 2 m, not cumulative distance, and z is ignored.
  const std::vector<StateVec> short_path{{0,0,0,0},{-2,0,8,0}};
  EXPECT_TRUE(comparePathWithDirectionApproximately(short_path,0.));
  EXPECT_FALSE(comparePathWithDirectionApproximately(short_path,M_PI));
  const std::vector<StateVec> across_wrap{{0,0,0,0},{-3,-.01,0,0}};
  EXPECT_TRUE(comparePathWithDirectionApproximately(across_wrap,M_PI-.01));
  EXPECT_TRUE(comparePathWithDirectionApproximately(forward,M_PI/2));
  EXPECT_FALSE(comparePathWithDirectionApproximately(forward,M_PI/2+.001));
}

TEST(PathHeading,DisabledTangentsAndZeroHorizontalSegmentsKeepAssignedYaw){
  std::vector<StateVec> path{{0,0,0,.3},{0,0,0,.7},{0,0,1,1.1},{1,0,1,1.4}};
  const auto original=path;
  applyPathHeading(path,false,true);EXPECT_EQ(path,original);
  applyHomingPathHeading(path,false,true);EXPECT_EQ(path,original);
  applyPathHeading(path,true,true);
  EXPECT_DOUBLE_EQ(path[0][3],.3);EXPECT_DOUBLE_EQ(path[1][3],.7);
  EXPECT_DOUBLE_EQ(path[2][3],1.1);EXPECT_NEAR(std::abs(path[3][3]),M_PI,1e-12);
}

TEST(PathHeading,LocalBackwardOutputDoesNotRequirePathRefinement){
  auto map=std::make_shared<UniformGainMap>();auto p=example();
  p.refinement.enabled=false;p.refinement.planning_backward=true;
  LocalPlanner planner(map,p);const StateVec root(0,0,0,.4);
  const auto result=planner.plan(root);ASSERT_TRUE(result.success);
  EXPECT_FALSE(result.refinement.attempted);EXPECT_DOUBLE_EQ(result.path.front()[3],root[3]);
  ASSERT_GT(result.path.size(),1u);
  for(std::size_t i=1;i<result.path.size();++i){
    const auto delta=(result.path[i]-result.path[i-1]).head<2>().eval();
    if(delta.norm()<=1e-10)continue;
    const Eigen::Vector2d facing(std::cos(result.path[i][3]),std::sin(result.path[i][3]));
    EXPECT_NEAR(facing.dot(delta.normalized()),-1.,1e-12);
  }
}

TEST(PathHeading,LocalBackwardYawChangesObservationGainBeforeSelectingAPath){
  auto map=std::make_shared<EastUnknownGainMap>();auto p=example();
  p.local_min={2.,-.001,-.001};p.local_max={3.,.001,.001};
  p.max_vertices=3;p.refinement.enabled=false;
  const StateVec root(0,0,0,M_PI/2);
  LocalPlanner forward(map,p);const auto outward=forward.plan(root);
  ASSERT_TRUE(outward.success);EXPECT_GT(outward.gain,0.);
  p.refinement.planning_backward=true;
  LocalPlanner backward(map,p);const auto retreat=backward.plan(root);
  EXPECT_FALSE(retreat.success);EXPECT_EQ(retreat.reason,"no_gain");
  EXPECT_GT(retreat.vertices,1);EXPECT_DOUBLE_EQ(retreat.gain,0.);
}

TEST(DirectionPenalty,MatchesOriginalMetricForStraightReverseAndBentPaths){
  const std::vector<Path> paths{
      {{0,0,0},{1,0,0}},{{0,0,0},{-1,0,0}},
      {{1,-2,.3},{1.7,-1.8,.4},{1.4,-.2,1.1},{-.8,.1,.7}},
      {{0,0,0},{.003,0,0},{.73,.2,0},{.73,-.6,.4}}};
  for(const auto& path:paths)for(double heading:{0.,M_PI,-2.4,.8})for(bool scale:{false,true}){
    EXPECT_NEAR(Trajectory::computeDistanceBetweenTrajectoryAndDirection(path,heading,.2,scale),
                originalDirection(path,heading,.2,scale),1e-11);
  }
  // Replacing the repeated endpoint with a sampled straight reference would
  // give zero for this forward path and silently change the original metric.
  EXPECT_NEAR(Trajectory::computeDistanceBetweenTrajectoryAndDirection(paths[0],0.,.2,true),3.,1e-12);
  EXPECT_NEAR(Trajectory::computeDistanceBetweenTrajectoryAndDirection(paths[1],0.,.2,true),9.,1e-12);
}

TEST(ExplorationDirection,ThresholdEmaWrapAndBoundedHistoryFollowOriginal){
  ExplorationDirection direction;
  EXPECT_FALSE(direction.update(StateVec(0,0,0,1.8)));
  EXPECT_EQ(direction.samples(),1u);EXPECT_EQ(direction.updates(),0u);EXPECT_DOUBLE_EQ(direction.heading(),0.);
  EXPECT_FALSE(direction.update(StateVec(.74,0,0,0)));
  EXPECT_TRUE(direction.update(StateVec(.75,0,0,0)));
  EXPECT_TRUE(direction.update(StateVec(.75,.75,0,0)));
  EXPECT_NEAR(direction.heading(),.7*M_PI/2,1e-12);EXPECT_EQ(direction.updates(),2u);

  ExplorationDirection crossing;StateVec state=StateVec::Zero();crossing.update(state);
  for(int i=0;i<4;++i){state.x()-=1.;EXPECT_TRUE(crossing.update(state));}
  const double before=crossing.heading();
  state.x()+=std::cos(-3.);state.y()+=std::sin(-3.);ASSERT_TRUE(crossing.update(state));
  const double turn=std::atan2(std::sin(-3.-before),std::cos(-3.-before));
  const double expected=std::atan2(std::sin(before+.7*turn),std::cos(before+.7*turn));
  EXPECT_NEAR(crossing.heading(),expected,1e-12);EXPECT_LT(crossing.heading(),-3.);
  for(int i=0;i<20;++i){state.x()+=1.;ASSERT_TRUE(crossing.update(state));}
  EXPECT_EQ(crossing.samples(),10u);EXPECT_EQ(crossing.updates(),25u);
}

TEST(ExplorationDirection,MeasuresTranslationWithoutUsingBodyYawOrAltitude){
  ExplorationDirection a,b;
  a.update(StateVec(0,0,0,0));b.update(StateVec(0,0,50,2.7));
  EXPECT_FALSE(b.update(StateVec(0,0,-100,-2.9)));
  EXPECT_EQ(b.samples(),1u);EXPECT_DOUBLE_EQ(b.heading(),0.);
  for(int i=1;i<=6;++i){
    a.update(StateVec(i*.8,i*.3,0,0));b.update(StateVec(i*.8,i*.3,i*20,-i*.4));
    EXPECT_DOUBLE_EQ(a.heading(),b.heading());EXPECT_EQ(a.updates(),b.updates());
  }
}

TEST(DirectionPenalty,ZeroWeightPreservesLegacyWinnerAndDoesNotChangeMotionHistory){
  auto map=std::make_shared<UniformGainMap>();auto p=example();LocalPlanner a(map,p),b(map,p);
  const StateVec root=StateVec::Zero();const auto first=a.plan(root),second=b.plan(root,2.4);
  ASSERT_TRUE(first.success);ASSERT_TRUE(second.success);
  const auto expected=originalWinner(a,p,0.);
  EXPECT_EQ(first.best_id,expected.id);EXPECT_DOUBLE_EQ(first.gain,expected.raw);
  EXPECT_EQ(first.best_id,second.best_id);EXPECT_DOUBLE_EQ(first.gain,second.gain);
  EXPECT_DOUBLE_EQ(first.unpenalized_gain,first.gain);EXPECT_DOUBLE_EQ(first.direction_factor,1.);
  ASSERT_EQ(first.path.size(),second.path.size());
  for(std::size_t i=0;i<first.path.size();++i)EXPECT_EQ(first.path[i],second.path[i]);
  ExplorationDirection history;history.update(root);history.update(StateVec(0,1,0,0));
  const double heading=history.heading();const auto samples=history.samples(),updates=history.updates();
  ASSERT_TRUE(a.plan(StateVec(4,0,0,-1.),history.heading()).success);
  EXPECT_DOUBLE_EQ(history.heading(),heading);EXPECT_EQ(history.samples(),samples);EXPECT_EQ(history.updates(),updates);
}

TEST(DirectionPenalty,NonzeroWeightChangesSelectionAndMatchesEveryCandidateFormula){
  auto map=std::make_shared<UniformGainMap>();auto p=example();LocalPlanner baseline(map,p);
  const auto original=baseline.plan(StateVec::Zero());ASSERT_TRUE(original.success);
  p.path_direction_penalty=2.;bool changed=false;
  for(double heading:{0.,M_PI,M_PI/2,-M_PI/2}){
    LocalPlanner planner(map,p);const auto result=planner.plan(StateVec::Zero(),heading);ASSERT_TRUE(result.success);
    const auto expected=originalWinner(planner,p,heading);
    EXPECT_EQ(result.best_id,expected.id);EXPECT_NEAR(result.gain,expected.score,1e-12);
    EXPECT_NEAR(result.unpenalized_gain,expected.raw,1e-12);EXPECT_NEAR(result.direction_distance,expected.distance,1e-12);
    EXPECT_NEAR(result.direction_factor,std::exp(-p.path_direction_penalty*expected.distance),1e-12);
    EXPECT_DOUBLE_EQ(result.exploring_direction,heading);EXPECT_LE(result.gain,result.unpenalized_gain);
    changed=changed||result.best_id!=original.best_id;
  }
  EXPECT_TRUE(changed);
}
