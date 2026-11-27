/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <chrono>
#include <cmath>

#include <gz/rendering/MoveToHelper.hh>
#include <gz/common/Console.hh>

#include "test_config.h"  // NOLINT(build/include)

#include "gz/rendering/RenderEngine.hh"
#include "gz/rendering/RenderingIface.hh"
#include "gz/rendering/Scene.hh"

using namespace gz;
using namespace rendering;

class MoveToHelperTest : public testing::Test,
                 public testing::WithParamInterface<const char *>
{
  public: void MoveTo(const std::string &_renderEngine);

  public: void OnMoveToComplete();

  public: void checkIsCompleted(double timeout);

  public: MoveToHelper moveToHelper;

  public: bool isMoveCompleted = false;
};

void MoveToHelperTest::OnMoveToComplete()
{
  this->isMoveCompleted = true;
}

void MoveToHelperTest::checkIsCompleted(double timeout)
{
  isMoveCompleted = false;

  std::chrono::time_point<std::chrono::system_clock> startTime;
  std::chrono::time_point<std::chrono::system_clock> endTime;

  startTime = std::chrono::system_clock::now();

  while (!isMoveCompleted)
  {
    moveToHelper.AddTime(0.01);
    endTime = std::chrono::system_clock::now();

    auto seconds =
      std::chrono::duration_cast<std::chrono::milliseconds>(
        endTime - startTime).count() / 1000.0;
    if (seconds > timeout)
      break;
  }
}

void MoveToHelperTest::MoveTo(const std::string &_renderEngine)
{
#ifdef __APPLE__                                                                              
  std::cerr << "Skipping test for apple, see issue #847." << std::endl;                       
  return;
#endif
  // create and populate scene
  RenderEngine *engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    FAIL() << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }
  ScenePtr scene = engine->CreateScene("scene");
  ASSERT_NE(nullptr, scene);

  VisualPtr root = scene->RootVisual();
  ASSERT_NE(nullptr, root);

  // create center visual
  VisualPtr center = scene->CreateVisual("center");
  ASSERT_NE(nullptr, center);
  center->AddGeometry(scene->CreateSphere());
  center->SetLocalPosition(30, 0, 0);
  center->SetLocalScale(0.1, 0.1, 0.1);
  root->AddChild(center);

  CameraPtr camera(scene->CreateCamera());
  ASSERT_NE(nullptr, camera);

  camera->SetLocalPosition(0.0, 0.0, 0.0);
  camera->SetLocalRotation(0.0, 0.0, 0.0);
  camera->SetImageWidth(800);
  camera->SetImageHeight(600);
  camera->SetAntiAliasing(2);
  camera->SetAspectRatio(1.333);
  camera->SetHFOV(IGN_PI / 2);
  root->AddChild(camera);

  ASSERT_NE(nullptr, camera);

  NodePtr target = scene->NodeByName("center");

  moveToHelper.SetInitCameraPose(math::Pose3d());

  // This call should return, there is no camera
  moveToHelper.AddTime(0.1);

  moveToHelper.MoveTo(camera, target, 0.5,
    std::bind(&MoveToHelperTest::OnMoveToComplete, this));
  EXPECT_FALSE(moveToHelper.Idle());
  checkIsCompleted(0.5);
  EXPECT_TRUE(moveToHelper.Idle());
  EXPECT_EQ(math::Vector3d(28.5, 0.0, 0.0), camera->LocalPosition());

  moveToHelper.MoveTo(camera, math::Pose3d(0.0, 0.0, 0.0, 0, 0, 0), 0.5,
    std::bind(&MoveToHelperTest::OnMoveToComplete, this));
  EXPECT_FALSE(moveToHelper.Idle());
  checkIsCompleted(0.5);
  EXPECT_EQ(math::Vector3d(0.0, 0.0, 0.0), camera->LocalPosition());
  EXPECT_TRUE(moveToHelper.Idle());

  math::Vector3d lookAt(0, 0, 1);
  math::Vector3d viewAngleDirection(0, 1, 1);
  moveToHelper.LookDirection(camera,
      viewAngleDirection, lookAt,
      0.5, std::bind(&MoveToHelperTest::OnMoveToComplete, this));
  EXPECT_FALSE(moveToHelper.Idle());
  checkIsCompleted(0.5);
  EXPECT_TRUE(moveToHelper.Idle());
  EXPECT_EQ(math::Vector3d(0.0, -1, 0.0), camera->LocalPosition());
  EXPECT_EQ(math::Quaterniond(0.0, -0.785398, 1.5708), camera->LocalRotation());

  moveToHelper.LookDirection(camera,
      math::Vector3d::Zero, lookAt,
      0.5, std::bind(&MoveToHelperTest::OnMoveToComplete, this));
  EXPECT_FALSE(moveToHelper.Idle());
  checkIsCompleted(0.5);
  EXPECT_TRUE(moveToHelper.Idle());
  EXPECT_EQ(math::Vector3d(0.0, 0, 0.0), camera->LocalPosition());
  EXPECT_EQ(math::Quaterniond(0.0, 0, 0), camera->LocalRotation());

  moveToHelper.MoveTo(camera, math::Pose3d(INFINITY, 0.0, 0.0, 0, 0, 0), 0.5,
    std::bind(&MoveToHelperTest::OnMoveToComplete, this));
  EXPECT_FALSE(moveToHelper.Idle());
  checkIsCompleted(0.5);
  EXPECT_EQ(math::Vector3d(0.0, 0, 0.0), camera->LocalPosition());
  EXPECT_TRUE(moveToHelper.Idle());

  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
TEST_P(MoveToHelperTest, MoveToHelper)
{
  MoveTo(GetParam());
}

INSTANTIATE_TEST_CASE_P(MoveToHelper, MoveToHelperTest,
    RENDER_ENGINE_VALUES,
    PrintToStringParam());

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
