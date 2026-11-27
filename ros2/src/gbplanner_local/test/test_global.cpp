#include <gtest/gtest.h>
#include "gbplanner_local/global_memory.hpp"
#include <cmath>
#include <limits>
#include "upstream_reference.hpp"

using namespace gbplanner2;
namespace {
std::shared_ptr<VoxelMap> map(){
  auto result=std::make_shared<VoxelMap>(.25,Eigen::Vector3d(-8,-8,-2),Eigen::Vector3i(64,64,16));
  result->fill(VoxelStatus::Free);return result;
}
VolumetricGain frontier(const StateVec&){VolumetricGain g;g.is_frontier=true;g.gain=10;g.num_unknown_voxels=10;return g;}
planner_msgs::msg::Graph branches(){
  GraphManager graph;graph.addVertex(new Vertex(0,StateVec(0,0,0,0)));
  for(int i=1;i<=4;++i){
    graph.addVertex(new Vertex(i,StateVec(i,0,0,0)));graph.addEdge(graph.getVertex(i-1),graph.getVertex(i),1);
  }
  for(int i=1;i<=4;++i){
    graph.addVertex(new Vertex(i+4,StateVec(0,i,0,0)));
    graph.addEdge(graph.getVertex(i==1?0:i+3),graph.getVertex(i+4),1);
  }
  graph.getVertex(4)->vol_gain=frontier(StateVec::Zero());graph.getVertex(8)->vol_gain=frontier(StateVec::Zero());
  planner_msgs::msg::Graph message;graph.convertGraphToMsg(message);return message;
}
}
TEST(Trajectory,ExtractedDtwAndDegeneratePaths){
  Trajectory::PathType a{{0,0,0},{1,0,0},{2,0,0}},b{{0,1,0},{1,1,0},{2,1,0}};
  EXPECT_DOUBLE_EQ(Trajectory::computeDTWDistance(a,a),0);
  EXPECT_DOUBLE_EQ(Trajectory::computeDTWDistance(a,b),3);
  EXPECT_DOUBLE_EQ(Trajectory::computeDistanceBetweenTwoTrajectories(a,a),0);
  Trajectory::PathType singleton{{0,0,0}},other{{1,0,0}};
  EXPECT_DOUBLE_EQ(Trajectory::computeDistanceBetweenTwoTrajectories(singleton,singleton),0);
  EXPECT_TRUE(std::isinf(Trajectory::computeDistanceBetweenTwoTrajectories(singleton,other)));
  EXPECT_TRUE(std::isinf(Trajectory::computeDistanceBetweenTwoTrajectories({},a)));
}
TEST(GlobalMemory,PreservesSeparatedFrontiersAndOriginalScore){
  auto grid=map();Parameters p;GlobalMemory memory(p);memory.updateMap(grid);
  ASSERT_TRUE(memory.recordOdometry(StateVec::Zero()));memory.absorbLocal(branches(),frontier);
  ASSERT_EQ(memory.frontierCount(),2);
  EXPECT_EQ(memory.lastFrontierUpdate().added,2);
  auto gain=[](const StateVec& state){auto g=frontier(state);g.gain=state.x()>3?20:10;return g;};
  auto selected=memory.selectFrontier(StateVec::Zero(),gain);
  ASSERT_TRUE(selected.success)<<selected.reason;
  EXPECT_NEAR(selected.path.back().x(),4,1e-9);
  EXPECT_NEAR(selected.distance,4,1e-9);
  EXPECT_NEAR(selected.score,20*std::exp(-.01*4),1e-9);
  for(std::size_t i=1;i<selected.path.size();++i)
    EXPECT_EQ(grid->path(selected.path[i-1].head<3>(),selected.path[i].head<3>(),p.robot_size),VoxelStatus::Free);
  int count=memory.graph().vertices_map_.size();memory.absorbLocal(branches(),gain);
  EXPECT_EQ(memory.graph().vertices_map_.size(),static_cast<std::size_t>(count));
}
TEST(GlobalMemory,RefreshClearsExploredFrontiers){
  GlobalMemory memory{Parameters{}};memory.updateMap(map());memory.recordOdometry(StateVec::Zero());
  memory.absorbLocal(branches(),frontier);
  auto result=memory.selectFrontier(StateVec::Zero(),[](const StateVec&){return VolumetricGain{};});
  EXPECT_FALSE(result.success);EXPECT_EQ(result.reason,"no_global_frontier");EXPECT_EQ(memory.frontierCount(),0);
}
TEST(GlobalMemory,VisitedStateExclusionMatchesUpstreamRadius){
  GlobalMemory memory{Parameters{}};memory.updateMap(map());
  memory.recordOdometry(StateVec::Zero());memory.recordOdometry(StateVec(3,0,0,0));
  memory.absorbLocal(branches(),frontier);
  EXPECT_EQ(memory.frontierCount(),1);
  auto result=memory.selectFrontier(StateVec::Zero(),frontier);ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.path.back().y(),4,1e-9);
}
TEST(GlobalMemory,HomingKeepsExactCurrentPoseAndRejectsBlockedEdge){
  auto grid=map();Parameters p;GlobalMemory memory(p);memory.updateMap(grid);
  for(double x:{0.0,1.0,2.0})ASSERT_TRUE(memory.recordOdometry(StateVec(x,0,0,0)));
  auto result=memory.homing(StateVec(2.02,0,0,1));ASSERT_TRUE(result.success)<<result.reason;
  EXPECT_NEAR(result.path.front().x(),2.02,1e-12);EXPECT_NEAR(result.path.front()[3],1,1e-12);
  EXPECT_NEAR(result.path.back().head<3>().norm(),0,1e-12);EXPECT_NEAR(result.distance,2.02,1e-12);
  for(int y=0;y<grid->dimensions().y();++y)for(int z=0;z<grid->dimensions().z();++z)
    grid->set({grid->index({1.5,0,0}).x(),y,z},VoxelStatus::Occupied);
  memory.updateMap(grid);result=memory.homing(StateVec(2.02,0,0,1));
  EXPECT_FALSE(result.success);EXPECT_EQ(result.reason,"global_target_unreachable");
}
TEST(GlobalMemory,MapResetAndOccupiedOdometry){
  auto grid=map();GlobalMemory memory{Parameters{}};memory.updateMap(grid);
  EXPECT_FALSE(memory.homing(StateVec::Zero()).success);
  ASSERT_TRUE(memory.recordOdometry(StateVec::Zero()));
  EXPECT_FALSE(memory.recordOdometry(StateVec(20,0,0,0)));
  memory.reset();EXPECT_TRUE(memory.graph().vertices_map_.empty());
  EXPECT_FALSE(memory.selectFrontier(StateVec::Zero(),frontier).success);
  grid->fill(VoxelStatus::Unknown);memory.updateMap(grid);EXPECT_FALSE(memory.recordOdometry(StateVec::Zero()));
}
TEST(GlobalMemory,ExplicitHomeRequiresEmptyGraphAndPreservesExistingRoutes){
  GlobalMemory memory{Parameters{}};auto grid=map();memory.updateMap(grid);
  const StateVec home(-1,0,0,.7);
  ASSERT_TRUE(memory.setHomingPosition(home));
  ASSERT_TRUE(memory.recordOdometry(StateVec(0,0,0,0)));
  ASSERT_TRUE(memory.recordOdometry(StateVec(1,0,0,0)));
  const auto count=memory.graph().vertices_map_.size();
  EXPECT_FALSE(memory.setHomingPosition(StateVec(1,0,0,0)));
  EXPECT_EQ(memory.graph().vertices_map_.size(),count);
  auto result=memory.homing(StateVec(1,0,0,0));ASSERT_TRUE(result.success)<<result.reason;
  EXPECT_NEAR(result.path.back().x(),-1,1e-12);EXPECT_NEAR(result.distance,2,1e-12);
  EXPECT_NEAR((memory.graph().vertices_map_.at(0)->state-home).norm(),0,1e-12);
  memory.reset();ASSERT_TRUE(memory.setHomingPosition(StateVec(1,0,0,0)));
  EXPECT_NEAR(memory.graph().vertices_map_.at(0)->state.x(),1,1e-12);
}
TEST(GlobalMemory,ExplicitHomeRejectsMissingUnknownOccupiedAndInvalidState){
  GlobalMemory memory{Parameters{}};
  EXPECT_FALSE(memory.setHomingPosition(StateVec::Zero()));
  auto grid=map();grid->fill(VoxelStatus::Unknown);memory.updateMap(grid);
  EXPECT_FALSE(memory.setHomingPosition(StateVec::Zero()));
  grid->fill(VoxelStatus::Occupied);EXPECT_FALSE(memory.setHomingPosition(StateVec::Zero()));
  grid->fill(VoxelStatus::Free);
  StateVec invalid=StateVec::Zero();invalid.x()=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(memory.setHomingPosition(invalid));
  EXPECT_FALSE(memory.setHomingPosition(StateVec(20,0,0,0)));
  EXPECT_TRUE(memory.graph().vertices_map_.empty());
}
TEST(GlobalMemory,UnreachableHighGainFrontierDoesNotHideReachableBranch){
  auto grid=map();Parameters p;GlobalMemory memory(p);memory.updateMap(grid);
  ASSERT_TRUE(memory.recordOdometry(StateVec::Zero()));memory.absorbLocal(branches(),frontier);
  ASSERT_EQ(memory.frontierCount(),2);
  // Separate the x branch while retaining the y branch and both frontier sites.
  for(int y=0;y<grid->dimensions().y();++y)for(int z=0;z<grid->dimensions().z();++z)
    grid->set({grid->index({2,0,0}).x(),y,z},VoxelStatus::Occupied);
  memory.updateMap(grid);
  auto gain=[](const StateVec& state){auto g=frontier(state);g.gain=state.x()>3?1000:10;return g;};
  auto result=memory.selectFrontier(StateVec::Zero(),gain);
  ASSERT_TRUE(result.success)<<result.reason;EXPECT_NEAR(result.path.back().y(),4,1e-9);
  EXPECT_NEAR(result.score,10*std::exp(-.04),1e-9);
}
TEST(GlobalMemory,UnknownMapInvalidatesEdgesAndOccupiedStartCannotReturnHome){
  auto grid=map();Parameters p;GlobalMemory memory(p);memory.updateMap(grid);
  for(double x:{0.,1.,2.,3.})ASSERT_TRUE(memory.recordOdometry(StateVec(x,0,0,0)));
  for(int y=0;y<grid->dimensions().y();++y)for(int z=0;z<grid->dimensions().z();++z)
    grid->set({grid->index({1.5,0,0}).x(),y,z},VoxelStatus::Unknown);
  memory.updateMap(grid);auto result=memory.homing(StateVec(3,0,0,0));
  EXPECT_FALSE(result.success);EXPECT_EQ(result.reason,"global_target_unreachable");
  grid->set(grid->index({3,0,0}),VoxelStatus::Occupied);memory.updateMap(grid);
  result=memory.homing(StateVec(3,0,0,0));
  EXPECT_FALSE(result.success);EXPECT_EQ(result.reason,"current_not_connected");
}
TEST(GlobalMemory,ChangedGainRejectsStaleIncomingFrontiers){
  GlobalMemory memory{Parameters{}};memory.updateMap(map());memory.recordOdometry(StateVec::Zero());
  memory.absorbLocal(branches(),[](const StateVec&){return VolumetricGain{};});
  EXPECT_EQ(memory.frontierCount(),0);EXPECT_EQ(memory.lastFrontierUpdate().candidates,0);
}
TEST(GlobalBackground,SmallSamplingWindowPreservesOriginalFiveMeterExclusion){
  GlobalMemory memory{Parameters{}};memory.updateMap(map());memory.recordOdometry(StateVec::Zero());
  auto result=memory.expandBackground(100,frontier);
  EXPECT_EQ(result.attempts,100);EXPECT_EQ(result.added,0);EXPECT_EQ(result.near_visited,100);
  EXPECT_EQ(memory.graph().vertices_map_.size(),1u);
}
TEST(GlobalBackground,LargerWindowAddsCollisionFreeVerticesRepeatably){
  Parameters p;p.local_min={-7,-7,-1};p.local_max={7,7,1};auto grid=map();
  GlobalMemory first(p),second(p);
  for(auto* memory:{&first,&second}){memory->updateMap(grid);ASSERT_TRUE(memory->recordOdometry(StateVec::Zero()));}
  auto a=first.expandBackground(200,frontier),b=second.expandBackground(200,frontier);
  ASSERT_GT(a.added,0);EXPECT_EQ(a.added,b.added);EXPECT_EQ(a.attempts,200);
  EXPECT_EQ(a.frontiers_added,a.added);EXPECT_EQ(first.frontierCount(),a.added);
  ASSERT_EQ(first.graph().vertices_map_.size(),second.graph().vertices_map_.size());
  for(const auto& item:first.graph().vertices_map_){
    auto other=second.graph().vertices_map_.at(item.first);EXPECT_NEAR((item.second->state-other->state).norm(),0,1e-12);
  }
  for(const auto& item:first.graph().edge_map_)for(const auto& edge:item.second){
    auto* from=first.graph().vertices_map_.at(item.first);auto* to=first.graph().vertices_map_.at(edge.first);
    EXPECT_EQ(grid->path(from->state.head<3>(),to->state.head<3>(),p.robot_size),VoxelStatus::Free);
  }
  first.reset();first.recordOdometry(StateVec::Zero());auto repeated=first.expandBackground(200,frontier);
  EXPECT_EQ(repeated.added,a.added);EXPECT_EQ(repeated.near_graph,a.near_graph);
}
TEST(GlobalBackground,UnknownSpaceAndBudgetsCannotCreateUnsafeExpansion){
  Parameters p;p.local_min={-7,-7,-1};p.local_max={7,7,1};auto grid=map();
  GlobalMemory memory(p);memory.updateMap(grid);memory.recordOdometry(StateVec::Zero());
  grid->fill(VoxelStatus::Unknown);memory.updateMap(grid);
  auto result=memory.expandBackground(100,frontier);EXPECT_EQ(result.added,0);EXPECT_EQ(result.outside_or_occupied,100);
  EXPECT_EQ(memory.expandBackground(0,frontier).attempts,0);
  EXPECT_THROW(memory.expandBackground(-1,frontier),std::invalid_argument);
  EXPECT_THROW(memory.expandBackground(100001,frontier),std::invalid_argument);
}
TEST(GlobalBackground,SharedExpansionMatchesOriginalVertexOverload){
  for(uint32_t seed=0;seed<10;++seed)for(bool graph_mode:{false,true}){
    auto grid=map();Parameters p;p.seed=seed;p.graph_mode=graph_mode;
    p.local_min={-7,-7,-1};p.local_max={7,7,1};
    // Put a full-height wall through the right half to exercise rejection too.
    for(int y=0;y<64;++y)for(int z=0;z<16;++z)grid->set({grid->index({2,0,0}).x(),y,z},VoxelStatus::Occupied);
    GraphManager port;auto original=std::make_shared<GraphManager>();
    port.addVertex(new Vertex(0,StateVec::Zero()));original->addVertex(new Vertex(0,StateVec::Zero()));
    upstream_reference::ReferenceRrg reference(grid,p);LocalSampler sampler(p);
    for(int i=0;i<300;++i){
      auto sample=sampler.generate(StateVec::Zero());
      if(grid->box(sample.head<3>(),p.robot_size)!=VoxelStatus::Free)continue;
      PlanResult result;expandAerial(port,grid,p,sample,result,100000);
      Vertex vertex(-1,sample);ExpandGraphReport report;reference.expandVertex(original,vertex,report);
    }
    ASSERT_EQ(port.getNumVertices(),original->getNumVertices());ASSERT_EQ(port.getNumEdges(),original->getNumEdges());
    for(int id=0;id<port.getNumVertices();++id){
      EXPECT_LT((port.getVertex(id)->state-original->getVertex(id)->state).norm(),1e-12);
      auto a=port.edge_map_[id],b=original->edge_map_[id];std::sort(a.begin(),a.end());std::sort(b.begin(),b.end());
      ASSERT_EQ(a.size(),b.size());
      for(std::size_t k=0;k<a.size();++k){EXPECT_EQ(a[k].first,b[k].first);EXPECT_NEAR(a[k].second,b[k].second,1e-12);}
    }
  }
}

TEST(MissionBudget,MinimumClockAndBatteryAges){
  MissionBudget budget(100,5);budget.observeClock(10);
  EXPECT_TRUE(budget.updateBattery(30,12,12));
  auto state=budget.snapshot(14);ASSERT_TRUE(state.valid);
  EXPECT_DOUBLE_EQ(state.elapsed,4);EXPECT_DOUBLE_EQ(state.remaining,28);
  MissionBudget clock(8,5);clock.observeClock(10);
  ASSERT_TRUE(clock.updateBattery(100,12,12));
  EXPECT_DOUBLE_EQ(clock.snapshot(14).remaining,4);
}
TEST(MissionBudget,ExactReserveAndHomingThreshold){
  double spare=0;
  EXPECT_TRUE(MissionBudget::sufficient(28,8,spare));EXPECT_DOUBLE_EQ(spare,20);
  EXPECT_FALSE(MissionBudget::sufficient(27.999,8,spare));
  EXPECT_FALSE(MissionBudget::shouldReturn(28,8,1));
  EXPECT_TRUE(MissionBudget::shouldReturn(27.999,8,1));
  EXPECT_FALSE(MissionBudget::shouldReturn(0,8,1));
}
TEST(MissionBudget,RejectsInvalidStaleAndReplayedReports){
  MissionBudget budget(100,2);budget.observeClock(10);
  EXPECT_FALSE(budget.updateBattery(std::nan(""),10,10));
  EXPECT_FALSE(budget.updateBattery(-1,10,10));
  EXPECT_FALSE(budget.updateBattery(50,0,10));
  EXPECT_FALSE(budget.updateBattery(50,7,10));
  EXPECT_FALSE(budget.updateBattery(50,11,10));
  ASSERT_TRUE(budget.updateBattery(25,10,10));
  EXPECT_FALSE(budget.updateBattery(100,10,11));
  EXPECT_DOUBLE_EQ(budget.snapshot(11).remaining,24);
}
TEST(MissionBudget,MissingStaleAndRecovery){
  MissionBudget budget(100,2,true);budget.observeClock(10);
  EXPECT_EQ(budget.snapshot(10).reason,"battery_status_missing");
  ASSERT_TRUE(budget.updateBattery(30,10,10));
  EXPECT_TRUE(budget.snapshot(12).valid);
  EXPECT_EQ(budget.snapshot(12.01).reason,"battery_status_stale");
  ASSERT_TRUE(budget.updateBattery(20,13,13));
  EXPECT_TRUE(budget.snapshot(13).valid);
}
TEST(MissionBudget,ClockRollbackLatchesAndZeroWaits){
  MissionBudget budget;budget.observeClock(0);
  EXPECT_EQ(budget.snapshot(0).reason,"mission_clock_unavailable");
  EXPECT_TRUE(budget.snapshot(10).valid);
  EXPECT_EQ(budget.snapshot(9).reason,"mission_clock_invalid");
  EXPECT_FALSE(budget.snapshot(20).valid);
}
TEST(MissionBudget,ExhaustionAndConfigurationValidation){
  MissionBudget budget(10);budget.observeClock(5);
  auto ended=budget.snapshot(20);ASSERT_TRUE(ended.valid);
  EXPECT_DOUBLE_EQ(ended.remaining,0);EXPECT_EQ(ended.reason,"mission_budget_exhausted");
  EXPECT_THROW(MissionBudget(0),std::invalid_argument);
  EXPECT_THROW(MissionBudget(10,-1),std::invalid_argument);
  double spare=0;EXPECT_FALSE(MissionBudget::sufficient(INFINITY,1,spare));
  EXPECT_FALSE(MissionBudget::sufficient(30,-1,spare));
}
TEST(GlobalMemory,TimeRequiresOutboundAndHomeAndExactReserve){
  GlobalMemory memory{Parameters{}};memory.updateMap(map());memory.recordOdometry(StateVec::Zero());
  memory.absorbLocal(branches(),frontier);
  const auto unrestricted=memory.selectFrontier(StateVec::Zero(),frontier);
  ASSERT_TRUE(unrestricted.success);const int target=unrestricted.target;
  GlobalTimeConstraint enough{28.,1.,true};
  auto selected=memory.selectFrontier(StateVec::Zero(),frontier,target,&enough);
  EXPECT_TRUE(selected.success)<<selected.reason;
  enough.remaining=27.999;
  selected=memory.selectFrontier(StateVec::Zero(),frontier,target,&enough);
  EXPECT_FALSE(selected.success);EXPECT_TRUE(selected.path.empty());
  EXPECT_EQ(selected.reason,"global_time_insufficient");
  // This would pass the upstream precedence bug, which only counted home.
  enough.remaining=25.;
  EXPECT_FALSE(memory.selectFrontier(StateVec::Zero(),frontier,-1,&enough).success);
  enough.include_home=false;
  EXPECT_TRUE(memory.selectFrontier(StateVec::Zero(),frontier,-1,&enough).success);
  EXPECT_TRUE(memory.selectFrontier(StateVec::Zero(),frontier,target).success);
}
TEST(GlobalMemory,TimeScoreUsesSpareAndRejectsInvalidConstraints){
  GlobalMemory memory{Parameters{}};memory.updateMap(map());memory.recordOdometry(StateVec::Zero());
  memory.absorbLocal(branches(),frontier);
  GlobalTimeConstraint enough{40.,1.,true};
  auto selected=memory.selectFrontier(StateVec::Zero(),frontier,-1,&enough);
  ASSERT_TRUE(selected.success)<<selected.reason;
  EXPECT_NEAR(selected.score,10*std::exp(-.04)*32,1e-9);
  enough.speed=0;
  EXPECT_EQ(memory.selectFrontier(StateVec::Zero(),frontier,-1,&enough).reason,"invalid_time_constraint");
}

TEST(GlobalRecovery,ClearedObstacleRestoresOriginalEdgesAndWeights){
  auto grid=map();GlobalMemory memory{Parameters{}};memory.updateMap(grid);
  for(double x:{0.,1.,2.,3.,4.})ASSERT_TRUE(memory.recordOdometry(StateVec(x,0,0,0)));
  const auto original=memory.graph().edge_map_;
  const auto count=memory.graph().vertices_map_.size();
  auto set_barrier=[&](VoxelStatus status){
    for(int y=0;y<grid->dimensions().y();++y)for(int z=0;z<grid->dimensions().z();++z)
      grid->set({grid->index({1.5,0,0}).x(),y,z},status);
    memory.updateMap(grid);
  };
  set_barrier(VoxelStatus::Occupied);
  ASSERT_GT(memory.lastMapEdgeUpdate().invalidated,0u);
  EXPECT_FALSE(memory.homing(StateVec(4,0,0,0)).success);
  set_barrier(VoxelStatus::Unknown);
  EXPECT_EQ(memory.lastMapEdgeUpdate().restored,0u);
  EXPECT_FALSE(memory.homing(StateVec(4,0,0,0)).success);
  set_barrier(VoxelStatus::Free);
  ASSERT_GT(memory.lastMapEdgeUpdate().restored,0u);
  EXPECT_EQ(memory.lastMapEdgeUpdate().pending,0u);
  EXPECT_EQ(memory.graph().vertices_map_.size(),count);
  for(const auto& item:original){
    auto expected=item.second,actual=memory.graph().edge_map_.at(item.first);
    std::sort(expected.begin(),expected.end());std::sort(actual.begin(),actual.end());
    EXPECT_EQ(expected,actual);
  }
  auto result=memory.homing(StateVec(4,0,0,0));ASSERT_TRUE(result.success)<<result.reason;
  EXPECT_NEAR(result.distance,4.,1e-12);
  const auto edge_count=memory.edgeCount();
  set_barrier(VoxelStatus::Free);EXPECT_EQ(memory.lastMapEdgeUpdate().restored,0u);
  EXPECT_EQ(memory.edgeCount(),edge_count);
  set_barrier(VoxelStatus::Occupied);EXPECT_FALSE(memory.homing(StateVec(4,0,0,0)).success);
  set_barrier(VoxelStatus::Free);EXPECT_TRUE(memory.homing(StateVec(4,0,0,0)).success);
  EXPECT_EQ(memory.edgeCount(),edge_count);
}
TEST(GlobalRecovery,RestorationRequiresSweptBodyClearance){
  auto grid=map();Parameters p;p.robot_size={.4,.8,.4};GlobalMemory memory(p);memory.updateMap(grid);
  ASSERT_TRUE(memory.recordOdometry(StateVec(0,0,0,0)));
  ASSERT_TRUE(memory.recordOdometry(StateVec(1,0,0,0)));
  // The centerline remains clear but the robot's side overlaps this voxel.
  const auto obstacle=grid->index({.5,.3,0});
  grid->set(obstacle,VoxelStatus::Occupied);memory.updateMap(grid);
  ASSERT_GT(memory.lastMapEdgeUpdate().pending,0u);
  EXPECT_FALSE(memory.homing(StateVec(1,0,0,0)).success);
  grid->set(obstacle,VoxelStatus::Unknown);memory.updateMap(grid);
  EXPECT_EQ(memory.lastMapEdgeUpdate().restored,0u);
  grid->set(obstacle,VoxelStatus::Free);memory.updateMap(grid);
  EXPECT_EQ(memory.lastMapEdgeUpdate().pending,0u);
  EXPECT_TRUE(memory.homing(StateVec(1,0,0,0)).success);
}
TEST(GlobalRecovery,ResetDropsSuspendedEdgesBeforeIdsAreReused){
  auto grid=map();GlobalMemory memory{Parameters{}};memory.updateMap(grid);
  for(double x:{0.,1.,2.})ASSERT_TRUE(memory.recordOdometry(StateVec(x,0,0,0)));
  grid->fill(VoxelStatus::Unknown);memory.updateMap(grid);
  ASSERT_GT(memory.lastMapEdgeUpdate().pending,0u);
  memory.reset();EXPECT_EQ(memory.lastMapEdgeUpdate().pending,0u);
  grid->fill(VoxelStatus::Free);memory.updateMap(grid);
  ASSERT_TRUE(memory.setHomingPosition(StateVec(-4,1,0,0)));
  ASSERT_TRUE(memory.recordOdometry(StateVec(-3,1,0,0)));
  const auto count=memory.edgeCount();memory.updateMap(grid);
  EXPECT_EQ(memory.edgeCount(),count);EXPECT_EQ(memory.lastMapEdgeUpdate().restored,0u);
  EXPECT_EQ(memory.graph().vertices_map_.size(),2u);
  auto result=memory.homing(StateVec(-3,1,0,0));ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.path.back().x(),-4.,1e-12);
}

TEST(PlanAheadBudget,CountsArrivalLocalReversalHomeAndReserve){
  double spare=0.;
  // 3 s to old endpoint + 2 m new route + (2 + 4) m return, at 1 m/s.
  EXPECT_TRUE(MissionBudget::sufficientAhead(31.,3.,2.,4.,1.,true,spare));
  EXPECT_DOUBLE_EQ(spare,20.);
  EXPECT_FALSE(MissionBudget::sufficientAhead(30.999,3.,2.,4.,1.,true,spare));
  // Without automatic homing, old arrival and outbound travel still count.
  EXPECT_TRUE(MissionBudget::sufficientAhead(25.,3.,2.,4.,1.,false,spare));
  EXPECT_DOUBLE_EQ(spare,20.);
  EXPECT_FALSE(MissionBudget::sufficientAhead(24.999,3.,2.,4.,1.,false,spare));
}
TEST(PlanAheadBudget,RecheckedSnapshotChargesPlanningBeyondArrival){
  MissionBudget budget(31.);budget.observeClock(10.);double spare=0.;
  EXPECT_TRUE(MissionBudget::sufficientAhead(budget.snapshot(10.).remaining,3.,2.,4.,1.,true,spare));
  // Two seconds spent planning overlap the old route's travel.
  EXPECT_TRUE(MissionBudget::sufficientAhead(budget.snapshot(12.).remaining,1.,2.,4.,1.,true,spare));
  // Once the old route has ended, further planning consumes the reserve.
  EXPECT_FALSE(MissionBudget::sufficientAhead(budget.snapshot(14.).remaining,0.,2.,4.,1.,true,spare));
}
TEST(PlanAheadBudget,RejectsInvalidTimesAndUsesConfiguredSpeed){
  double spare=0.;
  EXPECT_FALSE(MissionBudget::sufficientAhead(100.,-1.,2.,4.,1.,true,spare));
  EXPECT_FALSE(MissionBudget::sufficientAhead(100.,1.,-2.,4.,1.,true,spare));
  EXPECT_FALSE(MissionBudget::sufficientAhead(100.,1.,2.,-4.,1.,true,spare));
  EXPECT_FALSE(MissionBudget::sufficientAhead(100.,1.,2.,4.,0.,true,spare));
  EXPECT_FALSE(MissionBudget::sufficientAhead(100.,INFINITY,2.,4.,1.,true,spare));
  EXPECT_FALSE(MissionBudget::sufficientAhead(31.,3.,2.,4.,.5,true,spare));
  EXPECT_TRUE(MissionBudget::sufficientAhead(39.,3.,2.,4.,.5,true,spare));
}

TEST(PlanAheadBudget,HypotheticalHomingPreservesActualHistoryAndHome){
  GlobalMemory memory{Parameters{}};memory.updateMap(map());
  ASSERT_TRUE(memory.recordOdometry(StateVec(0,0,0,0)));
  ASSERT_TRUE(memory.recordOdometry(StateVec(.5,0,0,0)));
  memory.absorbLocal(branches(),frontier);
  auto hypothetical=memory.homing(StateVec(4,0,0,0));
  ASSERT_TRUE(hypothetical.success)<<hypothetical.reason;
  EXPECT_NEAR(memory.graph().vertices_map_.at(0)->state.x(),0.,1e-12);
  const int previous_count=memory.graph().vertices_map_.size();
  ASSERT_TRUE(memory.recordOdometry(StateVec(1,0,0,0)));
  ASSERT_EQ(memory.graph().vertices_map_.size(),previous_count+1u);
  const auto* latest=memory.graph().vertices_map_.at(previous_count);
  ASSERT_NE(latest,nullptr);ASSERT_NE(latest->parent,nullptr);
  // Only the actually measured .5 m pose may become its odometry parent.
  EXPECT_NEAR(latest->parent->state.x(),.5,1e-12);
}

TEST(ReferencePath,PreservesShortExplorationRouteWithoutInventingVisits){
  Parameters p;p.neighbor_radius=1.;
  GlobalMemory memory(p);memory.updateMap(map());
  ASSERT_TRUE(memory.recordOdometry(StateVec(0,0,0,0)));
  ASSERT_TRUE(memory.recordOdometry(StateVec(.5,0,0,0)));
  // Under the 3 m visited-frontier exclusion, but beyond the graph connector.
  const std::vector<StateVec> path{{.5,0,0,0},{.5,2.5,0,0},{1.,2.5,0,0}};
  EXPECT_FALSE(memory.homing(path.back()).success);
  ASSERT_TRUE(memory.addReferencePath(path));
  EXPECT_EQ(memory.frontierCount(),0);
  auto home=memory.homing(path.back());ASSERT_TRUE(home.success)<<home.reason;
  // homing() may add other checked neighbor links that shorten the route.
  EXPECT_LE(home.distance,3.5+1e-12);
  EXPECT_GE(home.distance,path.back().head<3>().norm()-1e-12);
  EXPECT_NEAR(home.path.back().head<3>().norm(),0.,1e-12);
  for(const auto& item:memory.graph().edge_map_)for(const auto& edge:item.second)
    EXPECT_LE(edge.second,1.+1e-12);
  const auto before=memory.graph().vertices_map_.size();
  const int edges_before=memory.edgeCount();
  ASSERT_TRUE(memory.addReferencePath(path));
  EXPECT_EQ(memory.graph().vertices_map_.size(),before);
  EXPECT_EQ(memory.edgeCount(),edges_before);
  // Recording the next actual pose must still branch from the last actual
  // odometry (.5,0,0), never from the reference endpoint (1,2.5,0).
  ASSERT_TRUE(memory.recordOdometry(StateVec(1.,0,0,0)));
  ASSERT_EQ(memory.graph().vertices_map_.size(),before+1u);
  const auto* measured=memory.graph().vertices_map_.at(static_cast<int>(before));
  ASSERT_NE(measured->parent,nullptr);
  EXPECT_NEAR((measured->parent->state-StateVec(.5,0,0,0)).norm(),0.,1e-12);
}
TEST(ReferencePath,BlockedLaterSegmentRejectsWholeRoute){
  auto grid=map();GlobalMemory memory{Parameters{}};memory.updateMap(grid);
  ASSERT_TRUE(memory.recordOdometry(StateVec(0,0,0,0)));
  const std::vector<StateVec> path{{0,0,0,0},{1,0,0,0},{2,0,0,0}};
  grid->set(grid->index({1.5,0,0}),VoxelStatus::Occupied);memory.updateMap(grid);
  const auto count=memory.graph().vertices_map_.size();const int edges=memory.edgeCount();
  EXPECT_FALSE(memory.addReferencePath(path));
  EXPECT_EQ(memory.graph().vertices_map_.size(),count);EXPECT_EQ(memory.edgeCount(),edges);
  grid->set(grid->index({1.5,0,0}),VoxelStatus::Unknown);memory.updateMap(grid);
  EXPECT_FALSE(memory.addReferencePath(path));EXPECT_EQ(memory.graph().vertices_map_.size(),count);
  grid->set(grid->index({1.5,0,0}),VoxelStatus::Free);memory.updateMap(grid);
  EXPECT_TRUE(memory.addReferencePath(path));EXPECT_TRUE(memory.homing(path.back()).success);
}
