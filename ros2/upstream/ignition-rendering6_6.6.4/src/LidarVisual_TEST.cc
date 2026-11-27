/* * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gtest/gtest.h>

#include <ignition/common/Console.hh>

#include "test_config.h"  // NOLINT(build/include)
#include "ignition/rendering/RenderEngine.hh"
#include "ignition/rendering/RenderingIface.hh"
#include "ignition/rendering/LidarVisual.hh"
#include "ignition/rendering/Scene.hh"

using namespace ignition;
using namespace rendering;
using namespace std::chrono_literals;

class LidarVisualTest : public testing::Test,
                   public testing::WithParamInterface<const char *>
{
  public: void LidarVisual(const std::string &_renderEngine);
};

/////////////////////////////////////////////////
void LidarVisualTest::LidarVisual(const std::string &_renderEngine)
{
#ifdef __APPLE__                                                                                                                                                                              
  std::cerr << "Skipping test for apple, see issue #847." << std::endl;
  return;
#endif
  if (_renderEngine == "optix")
  {
    igndbg << "LidarVisual not supported yet in rendering engine: "
            << _renderEngine << std::endl;
    return;
  }


  RenderEngine *engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    FAIL() << "Engine '" << _renderEngine
           << "' is not supported" << std::endl;
    return;
  }

  // check scene creation
  ScenePtr scene = engine->CreateScene("scene");
  VisualPtr root = scene->RootVisual();
  EXPECT_NE(scene, nullptr);
  EXPECT_NE(root, nullptr);

  // check that root visual has no child visuals
  EXPECT_EQ(root->ChildCount(), 0u);
  unsigned int count = root->ChildCount();

  // check lidar creation
  LidarVisualPtr lidar = scene->CreateLidarVisual();
  ASSERT_NE(nullptr, lidar);

  // check update in child count of root visual
  root->AddChild(lidar);
  ASSERT_NE(root->ChildCount(), count);
  ASSERT_EQ(root->ChildCount(), 1u);

  // check API
  lidar->SetMinVerticalAngle(0.54);
  EXPECT_DOUBLE_EQ(lidar->MinVerticalAngle(), 0.54);
  lidar->SetMaxVerticalAngle(5.30);
  EXPECT_DOUBLE_EQ(lidar->MaxVerticalAngle(), 5.30);

  lidar->SetMinHorizontalAngle(1.30);
  EXPECT_DOUBLE_EQ(lidar->MinHorizontalAngle(), 1.30);
  lidar->SetMaxHorizontalAngle(9.30);
  EXPECT_DOUBLE_EQ(lidar->MaxHorizontalAngle(), 9.30);

  lidar->SetVerticalRayCount(5);
  EXPECT_EQ(lidar->VerticalRayCount(), 5u);
  lidar->SetHorizontalRayCount(10);
  EXPECT_EQ(lidar->HorizontalRayCount(), 10u);

  lidar->SetMaxRange(50.50);
  EXPECT_DOUBLE_EQ(lidar->MaxRange(), 50.50);
  lidar->SetMinRange(0.54);
  EXPECT_DOUBLE_EQ(lidar->MinRange(), 0.54);

  lidar->SetDisplayNonHitting(false);
  EXPECT_EQ(lidar->DisplayNonHitting(), false);
  lidar->SetDisplayNonHitting(true);
  EXPECT_EQ(lidar->DisplayNonHitting(), true);

  lidar->SetType(LVT_NONE);
  EXPECT_EQ(lidar->Type(), LVT_NONE);
  lidar->SetType(LVT_POINTS);
  EXPECT_EQ(lidar->Type(), LVT_POINTS);
  lidar->SetType(LVT_POINTS);
  EXPECT_EQ(lidar->Type(), LVT_POINTS);
  lidar->SetType(LVT_TRIANGLE_STRIPS);
  EXPECT_EQ(lidar->Type(), LVT_TRIANGLE_STRIPS);

  EXPECT_DOUBLE_EQ(1.0, lidar->Size());
  lidar->SetSize(12.0);
  EXPECT_DOUBLE_EQ(12.0, lidar->Size());

  ignition::math::Pose3d p(0.5, 2.56, 3.67, 1.4, 2, 4.5);
  lidar->SetOffset(p);
  EXPECT_EQ(lidar->Offset(), p);

  std::vector<double> pts{10.0, 15.0, 15.0,
                          15.0, INFINITY, INFINITY, INFINITY,
                          10, 3.5};
  lidar->SetPoints(pts);
  EXPECT_EQ(pts.size(), lidar->PointCount());

  lidar->PreRender();

  lidar->ClearPoints();
  EXPECT_EQ(lidar->PointCount(), 0u);


  // Clean up
  engine->DestroyScene(scene);
  rendering::unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
TEST_P(LidarVisualTest, LidarVisual)
{
  LidarVisual(GetParam());
}

INSTANTIATE_TEST_CASE_P(LidarVisual, LidarVisualTest,
    RENDER_ENGINE_VALUES,
    ignition::rendering::PrintToStringParam());

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
