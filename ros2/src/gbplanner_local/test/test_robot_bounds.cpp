#include <gtest/gtest.h>
#include "gbplanner_local/robot_bounds.hpp"

using gbplanner2::RobotBounds;

TEST(RobotBounds, SourceSizesPointModeAndPerRequestBudgets){
  RobotBounds bounds;
  bounds.physical_size={.4,.5,.6};
  bounds.extension={.6,.8,1.0};
  bounds.min_extension={.2,.3,.4};
  bounds.relax_ratio=.25;
  EXPECT_TRUE(bounds.size(RobotBounds::Extended).isApprox(Eigen::Vector3d(1.,1.3,1.6)));
  EXPECT_TRUE(bounds.size(RobotBounds::Relaxed).isApprox(Eigen::Vector3d(.9,1.175,1.45)));
  EXPECT_TRUE(bounds.size(RobotBounds::Minimum).isApprox(Eigen::Vector3d(.6,.8,1.)));
  EXPECT_TRUE(bounds.size(RobotBounds::Exact).isApprox(bounds.physical_size));
  bounds.setBoundMode(RobotBounds::Point);
  EXPECT_TRUE(bounds.size().isZero());
  EXPECT_TRUE(RobotBounds::pointMode(bounds.bound_mode));
  EXPECT_EQ(bounds.collisionRadius(),0.);
  EXPECT_EQ(RobotBounds::scaledBudget(100,RobotBounds::Relaxed),130);
  EXPECT_EQ(RobotBounds::scaledBudget(100,RobotBounds::Minimum),169);
  EXPECT_EQ(RobotBounds::scaledBudget(100,RobotBounds::Exact),100);
  EXPECT_EQ(RobotBounds::scaledBudget(100,RobotBounds::Point),100);
  EXPECT_FALSE(RobotBounds::validMode(5));
  EXPECT_THROW(bounds.size(5),std::invalid_argument);
}
