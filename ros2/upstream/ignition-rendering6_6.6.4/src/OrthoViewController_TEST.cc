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

#include <ignition/common/Console.hh>

#include "test_config.h"  // NOLINT(build/include)
#include "ignition/rendering/Camera.hh"
#include "ignition/rendering/OrthoViewController.hh"
#include "ignition/rendering/RenderEngine.hh"
#include "ignition/rendering/RenderingIface.hh"
#include "ignition/rendering/Scene.hh"

using namespace ignition;
using namespace rendering;

class OrthoViewControllerTest : public testing::Test,
                         public testing::WithParamInterface<const char *>
{
  /// \brief Test basic api
  public: void OrthoViewControl(const std::string &_renderEngine);

  /// \brief test zoom, pan, orbit
  public: void Control(const std::string &_renderEngine);
};

/////////////////////////////////////////////////
void OrthoViewControllerTest::OrthoViewControl(const std::string &_renderEngine)
{
#ifdef __APPLE__                                                                                                                                                                              
  std::cerr << "Skipping test for apple, see issue #847." << std::endl;
  return;
#endif
  RenderEngine *engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    FAIL() << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }
  ScenePtr scene = engine->CreateScene("scene");
  EXPECT_NE(nullptr, scene);

  CameraPtr camera =  scene->CreateCamera("camera");
  EXPECT_NE(nullptr, camera);

  OrthoViewController viewControl;

  // verify intial values
  EXPECT_EQ(nullptr, viewControl.Camera());
  EXPECT_EQ(math::Vector3d::Zero, viewControl.Target());

  // test setting caream
  viewControl.SetCamera(camera);
  EXPECT_EQ(camera, viewControl.Camera());

  viewControl.SetCamera(CameraPtr());
  EXPECT_EQ(CameraPtr(), viewControl.Camera());

  // test setting target
  math::Vector3d target(1, 0, 0);
  viewControl.SetTarget(target);
  EXPECT_EQ(target, viewControl.Target());

  target.Set(-0.3, -5, 1);
  viewControl.SetTarget(target);
  EXPECT_EQ(target, viewControl.Target());

  // Clean up
  engine->DestroyScene(scene);
  rendering::unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
void OrthoViewControllerTest::Control(const std::string &_renderEngine)
{
#ifdef __APPLE__                                                                                                                                                                              
  std::cerr << "Skipping test for apple, see issue #847." << std::endl;
  return;
#endif
  RenderEngine *engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    FAIL() << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }
  ScenePtr scene = engine->CreateScene("scene");
  ASSERT_NE(nullptr, scene);

  CameraPtr camera =  scene->CreateCamera("camera");
  ASSERT_NE(nullptr, camera);

  camera->SetImageWidth(320);
  camera->SetImageHeight(240);

  math::Pose3d initialPose = camera->WorldPose();
  EXPECT_EQ(math::Pose3d::Zero, initialPose);

  OrthoViewController viewControl;

  // test setting camera
  viewControl.SetCamera(camera);
  EXPECT_EQ(camera, viewControl.Camera());

  // test setting target
  math::Vector3d target(1, 0, 0);
  viewControl.SetTarget(target);
  EXPECT_EQ(target, viewControl.Target());

  // test zoom
  math::Matrix4d initialProjectionMatrix = camera->ProjectionMatrix();
  viewControl.Zoom(0);
  EXPECT_EQ(math::Pose3d::Zero, initialPose);
  viewControl.Zoom(0.1);
  // we do not change the camera pos when zooming in orthographic view
  // we update the projection matrix
  EXPECT_EQ(initialPose.Pos(), camera->WorldPose().Pos());
  EXPECT_EQ(initialPose.Rot(), camera->WorldPose().Rot());
  math::Matrix4d zoomProjectionMatrix = camera->ProjectionMatrix();
  EXPECT_NE(initialProjectionMatrix, zoomProjectionMatrix);
  viewControl.Zoom(-5);
  math::Matrix4d zoomProjectionMatrix2 = camera->ProjectionMatrix();
  EXPECT_EQ(initialPose.Pos(), camera->WorldPose().Pos());
  EXPECT_EQ(initialPose.Rot(), camera->WorldPose().Rot());
  EXPECT_NE(initialProjectionMatrix, zoomProjectionMatrix2);
  EXPECT_NE(zoomProjectionMatrix, zoomProjectionMatrix2);

  // reset camera pose
  camera->SetWorldPose(initialPose);
  EXPECT_EQ(math::Pose3d::Zero, initialPose);

  // test zero pan
  viewControl.Pan(math::Vector2d(0, 0));
  EXPECT_EQ(math::Pose3d::Zero, initialPose);

  // test pan in viewport x
  viewControl.Pan(math::Vector2d(2, 0));
  EXPECT_DOUBLE_EQ(initialPose.Pos().X(), camera->WorldPose().Pos().X());
  EXPECT_LT(initialPose.Pos().Y(), camera->WorldPose().Pos().Y());
  EXPECT_DOUBLE_EQ(initialPose.Pos().Z(), camera->WorldPose().Pos().Z());
  EXPECT_EQ(initialPose.Rot(), camera->WorldPose().Rot());
  viewControl.Pan(math::Vector2d(-8, 0));
  EXPECT_DOUBLE_EQ(initialPose.Pos().X(), camera->WorldPose().Pos().X());
  EXPECT_GT(initialPose.Pos().Y(), camera->WorldPose().Pos().Y());
  EXPECT_DOUBLE_EQ(initialPose.Pos().Z(), camera->WorldPose().Pos().Z());
  EXPECT_EQ(initialPose.Rot(), camera->WorldPose().Rot());

  // reset camera pose
  camera->SetWorldPose(initialPose);
  EXPECT_EQ(math::Pose3d::Zero, initialPose);

  // test pan in viewport y
  viewControl.Pan(math::Vector2d(0, 7));
  EXPECT_DOUBLE_EQ(initialPose.Pos().X(), camera->WorldPose().Pos().X());
  EXPECT_DOUBLE_EQ(initialPose.Pos().Y(), camera->WorldPose().Pos().Y());
  EXPECT_LT(initialPose.Pos().Z(), camera->WorldPose().Pos().Z());
  EXPECT_EQ(initialPose.Rot(), camera->WorldPose().Rot());
  viewControl.Pan(math::Vector2d(0, -25));
  EXPECT_DOUBLE_EQ(initialPose.Pos().X(), camera->WorldPose().Pos().X());
  EXPECT_DOUBLE_EQ(initialPose.Pos().Y(), camera->WorldPose().Pos().Y());
  EXPECT_GT(initialPose.Pos().Z(), camera->WorldPose().Pos().Z());
  EXPECT_EQ(initialPose.Rot(), camera->WorldPose().Rot());

  // reset camera pose
  camera->SetWorldPose(initialPose);
  EXPECT_EQ(math::Pose3d::Zero, initialPose);

  // test zero orbit
  viewControl.Orbit(math::Vector2d(0, 0));
  EXPECT_EQ(math::Pose3d::Zero, initialPose);

  // test orbit in viewport x (yaw)
  viewControl.Orbit(math::Vector2d(100, 0));
  EXPECT_LT(initialPose.Pos().X(), camera->WorldPose().Pos().X());
  EXPECT_LT(initialPose.Pos().Y(), camera->WorldPose().Pos().Y());
  EXPECT_DOUBLE_EQ(initialPose.Pos().Z(), camera->WorldPose().Pos().Z());
  EXPECT_DOUBLE_EQ(initialPose.Rot().X(), camera->WorldPose().Rot().X());
  EXPECT_DOUBLE_EQ(initialPose.Rot().Y(), camera->WorldPose().Rot().Y());
  EXPECT_GT(initialPose.Rot().Z(), camera->WorldPose().Rot().Z());

  camera->SetWorldPose(initialPose);
  EXPECT_EQ(math::Pose3d::Zero, initialPose);

  viewControl.Orbit(math::Vector2d(-80, 0));
  EXPECT_LT(initialPose.Pos().X(), camera->WorldPose().Pos().X());
  EXPECT_GT(initialPose.Pos().Y(), camera->WorldPose().Pos().Y());
  EXPECT_DOUBLE_EQ(initialPose.Pos().Z(), camera->WorldPose().Pos().Z());
  EXPECT_DOUBLE_EQ(initialPose.Rot().X(), camera->WorldPose().Rot().X());
  EXPECT_DOUBLE_EQ(initialPose.Rot().Y(), camera->WorldPose().Rot().Y());
  EXPECT_LT(initialPose.Rot().Z(), camera->WorldPose().Rot().Z());

  // reset camera pose
  camera->SetWorldPose(initialPose);
  EXPECT_EQ(math::Pose3d::Zero, initialPose);

  // test orbit in viewport y (pitch)
  viewControl.Orbit(math::Vector2d(0, 80));
  EXPECT_LT(initialPose.Pos().X(), camera->WorldPose().Pos().X());
  EXPECT_DOUBLE_EQ(initialPose.Pos().Y(), camera->WorldPose().Pos().Y());
  EXPECT_LT(initialPose.Pos().Z(), camera->WorldPose().Pos().Z());
  EXPECT_DOUBLE_EQ(initialPose.Rot().X(), camera->WorldPose().Rot().X());
  EXPECT_LT(initialPose.Rot().Y(), camera->WorldPose().Rot().Y());
  EXPECT_DOUBLE_EQ(initialPose.Rot().Z(), camera->WorldPose().Rot().Z());

  camera->SetWorldPose(initialPose);
  EXPECT_EQ(math::Pose3d::Zero, initialPose);

  viewControl.Orbit(math::Vector2d(0, -90));
  EXPECT_LT(initialPose.Pos().X(), camera->WorldPose().Pos().X());
  EXPECT_DOUBLE_EQ(initialPose.Pos().Y(), camera->WorldPose().Pos().Y());
  EXPECT_GT(initialPose.Pos().Z(), camera->WorldPose().Pos().Z());
  EXPECT_DOUBLE_EQ(initialPose.Rot().X(), camera->WorldPose().Rot().X());
  EXPECT_GT(initialPose.Rot().Y(), camera->WorldPose().Rot().Y());
  EXPECT_DOUBLE_EQ(initialPose.Rot().Z(), camera->WorldPose().Rot().Z());

  // Clean up
  engine->DestroyScene(scene);
  rendering::unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
TEST_P(OrthoViewControllerTest, OrthoViewController)
{
  OrthoViewControl(GetParam());
}

/////////////////////////////////////////////////
TEST_P(OrthoViewControllerTest, Control)
{
  Control(GetParam());
}

INSTANTIATE_TEST_CASE_P(OrthoViewController, OrthoViewControllerTest,
    RENDER_ENGINE_VALUES,
    ignition::rendering::PrintToStringParam());

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
