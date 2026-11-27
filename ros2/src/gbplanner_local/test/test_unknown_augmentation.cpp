#include <gtest/gtest.h>
#include "gbplanner_local/local_planner.hpp"
#include "gbplanner_local/unknown_augmentation_schedule.hpp"
#include <limits>

using namespace gbplanner2;

TEST(UnknownAugmentationSchedule,CountsCompletedEmptyBuildsAcrossGraphResets){
  auto map=std::make_shared<VoxelMap>(.25,Eigen::Vector3d(-2,-2,-2),Eigen::Vector3i(16,16,16));
  map->fill(VoxelStatus::Free);
  Parameters p;p.local_min={-.2,-.2,-.2};p.local_max={.2,.2,.2};
  // Samples are free but too close to create an edge: buildGraph reaches
  // its empty-graph cutoff and must still consume one augmentation round.
  p.edge_min=1.;p.edge_max=2.;p.max_attempts=8;p.empty_graph_cutoff=2;
  LocalPlanner planner(map,p);UnknownAugmentationSchedule schedule(2);
  const StateVec root(0,0,0,0);
  EXPECT_TRUE(schedule.odometryWindowOpen());
  for(std::uint64_t completed=1;completed<=3;++completed){
    const auto invalid=planner.plan(StateVec(std::numeric_limits<double>::quiet_NaN(),0,0,0));
    ASSERT_EQ(invalid.reason,"invalid_root");EXPECT_FALSE(invalid.graph_build_completed);
    schedule.recordGraphBuild(invalid.graph_build_completed);
    EXPECT_EQ(schedule.completedBuilds(),completed-1);
    const auto built=planner.plan(root);
    ASSERT_EQ(built.reason,"no_feasible_path");EXPECT_FALSE(built.success);
    ASSERT_TRUE(built.empty_graph_cutoff_reached);ASSERT_TRUE(built.graph_build_completed);
    schedule.recordGraphBuild(built.graph_build_completed);
    EXPECT_EQ(schedule.completedBuilds(),completed);
    EXPECT_EQ(schedule.odometryWindowOpen(),completed<2);
  }
}

TEST(UnknownAugmentationSchedule,ZeroLimitClosesOnlyTheOdometryWindow){
  UnknownAugmentationSchedule defaults,disabled(0);
  EXPECT_EQ(defaults.buildLimit(),5u);EXPECT_TRUE(defaults.odometryWindowOpen());
  EXPECT_FALSE(disabled.odometryWindowOpen());
  disabled.recordGraphBuild(false);EXPECT_EQ(disabled.completedBuilds(),0u);
  // Counting remains lifetime accounting even when the odometry window is
  // closed. The caller may still augment an independently checked root box.
  disabled.recordGraphBuild(true);EXPECT_EQ(disabled.completedBuilds(),1u);
  EXPECT_FALSE(disabled.odometryWindowOpen());
}
