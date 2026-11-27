#include <gtest/gtest.h>

#include <cmath>

#include "gbplanner_voxblox/transform_queue.hpp"

namespace {
constexpr float kPi = 3.14159265358979323846f;

voxblox::Transformation pose(float x, float y, float z, float yaw) {
  return voxblox::Transformation(
      voxblox::Rotation(Eigen::Quaternionf(Eigen::AngleAxisf(yaw, voxblox::Point::UnitZ()))),
      voxblox::Point(x, y, z));
}

void expectPosition(const voxblox::Transformation& value, float x, float y, float z) {
  EXPECT_NEAR(value.getPosition().x(), x, 1e-5f);
  EXPECT_NEAR(value.getPosition().y(), y, 1e-5f);
  EXPECT_NEAR(value.getPosition().z(), z, 1e-5f);
}
}  // namespace

TEST(TransformQueue, InterpolatesTranslationAndYawAndKeepsFutureSamples) {
  gbplanner2::TransformQueue queue(1);
  queue.push({1000000000, pose(1.f, 2.f, 3.f, kPi / 2.f)});
  queue.push({5000000000, pose(5.f, 6.f, 7.f, kPi)});
  queue.push({9000000000, pose(9.f, 10.f, 11.f, kPi)});
  voxblox::Transformation result;
  ASSERT_TRUE(queue.lookup(2000000000, &result));
  // Independent expected geometry at alpha=.25: linear world translation,
  // plus 22.5 degrees of rotation relative to the initial 90-degree pose.
  expectPosition(result, 2.f, 3.f, 4.f);
  const auto direction = result.getRotation().rotate(voxblox::Point::UnitX());
  EXPECT_NEAR(direction.x(), std::cos(5.f * kPi / 8.f), 1e-5f);
  EXPECT_NEAR(direction.y(), std::sin(5.f * kPi / 8.f), 1e-5f);
  EXPECT_EQ(queue.size(), 3u);  // Old interpolation endpoint remains reusable.

  ASSERT_TRUE(queue.lookup(5000000000, &result));
  expectPosition(result, 5.f, 6.f, 7.f);
  EXPECT_EQ(queue.size(), 2u);
  ASSERT_TRUE(queue.lookup(7000000000, &result));
  expectPosition(result, 7.f, 8.f, 9.f);
  EXPECT_EQ(queue.size(), 2u);
  ASSERT_TRUE(queue.lookup(9000000000, &result));
  expectPosition(result, 9.f, 10.f, 11.f);
  EXPECT_EQ(queue.size(), 1u);
}

TEST(TransformQueue, BodyDynamicAndSensorExtrinsicsHaveTheOriginalOrder) {
  const auto T_G_D = pose(10.f, 0.f, 0.f, kPi / 2.f);
  const auto T_B_D = pose(1.f, 0.f, 0.f, kPi / 2.f);
  const auto T_B_C = pose(0.f, 2.f, 0.f, kPi);
  const auto T_G_C = gbplanner2::sensorTransform(T_G_D, T_B_D, T_B_C);
  // D<-C has translation (2,1,0), then G<-D rotates it to (-1,2,0).
  expectPosition(T_G_C, 9.f, 2.f, 0.f);
  const auto transformed = T_G_C * voxblox::Point(1.f, 0.f, 0.f);
  EXPECT_NEAR(transformed.x(), 8.f, 1e-5f);
  EXPECT_NEAR(transformed.y(), 2.f, 1e-5f);
  EXPECT_NEAR(transformed.z(), 0.f, 1e-5f);
}

TEST(TransformQueue, FirstToleranceHitStrictBoundaryAndNoExtrapolation) {
  gbplanner2::TransformQueue queue(10);
  queue.push({100, pose(1.f, 0.f, 0.f, 0.f)});
  queue.push({108, pose(9.f, 0.f, 0.f, 0.f)});
  queue.push({200, pose(20.f, 0.f, 0.f, 0.f)});
  voxblox::Transformation result;
  ASSERT_TRUE(queue.lookup(107, &result));
  expectPosition(result, 1.f, 0.f, 0.f);  // First within 10 ns, despite a closer 108.
  EXPECT_EQ(queue.size(), 3u);
  ASSERT_TRUE(queue.lookup(117, &result));
  expectPosition(result, 9.f, 0.f, 0.f);
  EXPECT_EQ(queue.size(), 2u);
  ASSERT_TRUE(queue.lookup(154, &result));
  expectPosition(result, 14.5f, 0.f, 0.f);
  EXPECT_EQ(queue.size(), 2u);
  ASSERT_TRUE(queue.lookup(200, &result));
  EXPECT_EQ(queue.size(), 1u);
  EXPECT_FALSE(queue.lookup(210, &result));  // Equality is not within tolerance.
  EXPECT_FALSE(queue.lookup(190, &result));  // No old interpolation endpoint.
  expectPosition(result, 20.f, 0.f, 0.f);
  EXPECT_EQ(queue.size(), 1u);
  ASSERT_TRUE(queue.lookup(209, &result));  // A tolerant endpoint match is allowed.
  expectPosition(result, 20.f, 0.f, 0.f);
}

TEST(TransformQueue, RetainsZeroToleranceAndArrivalOrderBehavior) {
  gbplanner2::TransformQueue zero_tolerance(0);
  zero_tolerance.push({100, pose(1.f, 0.f, 0.f, 0.f)});
  zero_tolerance.push({200, pose(2.f, 0.f, 0.f, 0.f)});
  voxblox::Transformation result = pose(9.f, 0.f, 0.f, 0.f);
  // Upstream's strict < comparison with zero tolerance does not directly
  // match an equal stamp. An old endpoint can still interpolate with alpha=0.
  ASSERT_TRUE(zero_tolerance.lookup(100, &result));
  expectPosition(result, 1.f, 0.f, 0.f);
  EXPECT_FALSE(zero_tolerance.lookup(200, &result));
  EXPECT_EQ(zero_tolerance.size(), 2u);
  expectPosition(result, 1.f, 0.f, 0.f);

  gbplanner2::TransformQueue arrival_order(1);
  arrival_order.push({200, pose(2.f, 0.f, 0.f, 0.f)});
  arrival_order.push({100, pose(1.f, 0.f, 0.f, 0.f)});
  EXPECT_FALSE(arrival_order.lookup(150, &result));
  EXPECT_EQ(arrival_order.size(), 2u);  // push did not silently sort the samples.
  arrival_order.clear();
  EXPECT_EQ(arrival_order.size(), 0u);
}
