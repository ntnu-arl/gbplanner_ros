/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <gz/common/Console.hh>

#include "test_config.h"  // NOLINT(build/include)

#include "gz/rendering/Camera.hh"
#include "gz/rendering/RenderEngine.hh"
#include "gz/rendering/RenderingIface.hh"
#include "gz/rendering/Scene.hh"
#include "gz/rendering/TransformController.hh"

using namespace gz;
using namespace rendering;

class TransformControllerTest : public testing::Test,
                         public testing::WithParamInterface<const char *>
{
  /// \brief Test basic api
  public: void TransformControl(const std::string &_renderEngine);

  /// \brief test rotate, translate, scale transformations in world space
  public: void WorldSpace(const std::string &_renderEngine);

  /// \brief test rotate, translate, scale transformations in local space
  public: void LocalSpace(const std::string &_renderEngine);

  /// \brief test rotate, translate, scale transformations from 2d movements
  public: void Control2d(const std::string &_renderEngine);
};

/////////////////////////////////////////////////
void TransformControllerTest::TransformControl(const std::string &_renderEngine)
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
  EXPECT_NE(scene, nullptr);

  CameraPtr camera =  scene->CreateCamera("camera");
  EXPECT_NE(camera, nullptr);

  TransformController transformControl;

  // verify intial values
  EXPECT_EQ(nullptr, transformControl.Camera());
  EXPECT_EQ(nullptr, transformControl.Node());
  EXPECT_FALSE(transformControl.Active());
  EXPECT_EQ(TransformMode::TM_NONE, transformControl.Mode());
  EXPECT_EQ(TransformSpace::TS_LOCAL, transformControl.Space());
  EXPECT_EQ(math::Vector3d::Zero, transformControl.ActiveAxis());

  // create visual node for testing
  VisualPtr visual = scene->CreateVisual();
  ASSERT_NE(nullptr, visual);

  // test attaching / detaching node
  transformControl.Attach(visual);
  EXPECT_EQ(visual, transformControl.Node());

  transformControl.Detach();
  EXPECT_EQ(nullptr, transformControl.Node());

  // attach node again for subsequent tests
  transformControl.Attach(visual);

  // test setting camera
  transformControl.SetCamera(camera);
  EXPECT_EQ(camera, transformControl.Camera());

  // test setting transform space
  transformControl.SetTransformSpace(TransformSpace::TS_WORLD);
  EXPECT_EQ(TransformSpace::TS_WORLD, transformControl.Space());

  // test setting transform axis
  transformControl.SetActiveAxis(math::Vector3d::UnitZ);
  EXPECT_EQ(math::Vector3d::UnitZ, transformControl.ActiveAxis());

  // test setting transform mode
  transformControl.SetTransformMode(TransformMode::TM_ROTATION);
  EXPECT_EQ(TransformMode::TM_ROTATION, transformControl.Mode());

  // verify active state
  transformControl.Start();
  EXPECT_TRUE(transformControl.Active());
  transformControl.Stop();
  EXPECT_FALSE(transformControl.Active());

  // test axis conversion
  EXPECT_EQ(math::Vector3d::UnitX,
      transformControl.ToAxis(TransformAxis::TA_TRANSLATION_X));
  EXPECT_EQ(math::Vector3d::UnitY,
      transformControl.ToAxis(TransformAxis::TA_TRANSLATION_Y));
  EXPECT_EQ(math::Vector3d::UnitZ,
      transformControl.ToAxis(TransformAxis::TA_TRANSLATION_Z));
  EXPECT_EQ(math::Vector3d::UnitX,
      transformControl.ToAxis(TransformAxis::TA_ROTATION_X));
  EXPECT_EQ(math::Vector3d::UnitY,
      transformControl.ToAxis(TransformAxis::TA_ROTATION_Y));
  EXPECT_EQ(math::Vector3d::UnitZ,
      transformControl.ToAxis(TransformAxis::TA_ROTATION_Z));
  EXPECT_EQ(math::Vector3d::UnitX,
      transformControl.ToAxis(TransformAxis::TA_SCALE_X));
  EXPECT_EQ(math::Vector3d::UnitY,
      transformControl.ToAxis(TransformAxis::TA_SCALE_Y));
  EXPECT_EQ(math::Vector3d::UnitZ,
      transformControl.ToAxis(TransformAxis::TA_SCALE_Z));

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
void TransformControllerTest::WorldSpace(const std::string &_renderEngine)
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

  TransformController transformControl;

  // test setting camera
  transformControl.SetCamera(camera);
  EXPECT_EQ(camera, transformControl.Camera());

  // create visual node for testing
  VisualPtr visual = scene->CreateVisual();
  ASSERT_NE(nullptr, visual);
  transformControl.Attach(visual);
  EXPECT_EQ(visual, transformControl.Node());

  // test translation in world space
  transformControl.SetTransformMode(TransformMode::TM_TRANSLATION);
  transformControl.SetTransformSpace(TransformSpace::TS_WORLD);
  transformControl.SetActiveAxis(math::Vector3d::UnitZ);
  transformControl.Translate(math::Vector3d(0, 0, 2));
  EXPECT_EQ(visual->WorldPosition(), math::Vector3d(0, 0, 2));
  EXPECT_EQ(visual->WorldRotation(), math::Quaterniond::Identity);
  EXPECT_EQ(visual->WorldScale(), math::Vector3d::One);

  // test rotation in world space
  transformControl.SetTransformMode(TransformMode::TM_ROTATION);
  transformControl.SetTransformSpace(TransformSpace::TS_WORLD);
  transformControl.SetActiveAxis(math::Vector3d::UnitX);
  transformControl.Rotate(math::Quaterniond(IGN_PI, 0, 0));
  EXPECT_EQ(visual->WorldPosition(), math::Vector3d(0, 0, 2));
  EXPECT_EQ(visual->WorldRotation(), math::Quaterniond(IGN_PI, 0, 0));
  EXPECT_EQ(visual->WorldScale(), math::Vector3d::One);

  // test scaling in world space
  transformControl.SetTransformMode(TransformMode::TM_SCALE);
  transformControl.SetTransformSpace(TransformSpace::TS_WORLD);
  transformControl.SetActiveAxis(math::Vector3d::UnitY);
  transformControl.Scale(math::Vector3d(1.0, 0.3, 1.0));
  EXPECT_EQ(visual->WorldPosition(), math::Vector3d(0, 0, 2));
  EXPECT_EQ(visual->WorldRotation(), math::Quaterniond(IGN_PI, 0, 0));
  EXPECT_EQ(visual->WorldScale(), math::Vector3d(1.0, 0.3, 1.0));

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
void TransformControllerTest::LocalSpace(const std::string &_renderEngine)
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

  TransformController transformControl;

  // test invalid callas and make sure no exceptions are thrown
  EXPECT_NO_THROW(transformControl.SetCamera(nullptr));
  EXPECT_NO_THROW(transformControl.Attach(nullptr));
  EXPECT_NO_THROW(transformControl.Start());
  EXPECT_NO_THROW(transformControl.Translate(math::Vector3d::Zero));
  EXPECT_NO_THROW(transformControl.Rotate(math::Quaterniond::Identity));
  EXPECT_NO_THROW(transformControl.Translate(math::Vector3d::One));
  EXPECT_EQ(math::Vector3d::Zero, transformControl.AxisById(0u));

  // test setting camera
  transformControl.SetCamera(camera);
  EXPECT_EQ(camera, transformControl.Camera());

  // create a visual node and intiialize it with a rotation
  // for testing transfoms in local space
  VisualPtr visual = scene->CreateVisual();
  ASSERT_NE(nullptr, visual);
  math::Quaterniond initialRot(IGN_PI * 0.5, 0, 0);
  visual->SetLocalRotation(initialRot);
  EXPECT_EQ(initialRot, visual->WorldRotation());
  transformControl.Attach(visual);
  EXPECT_EQ(visual, transformControl.Node());

  // test translation in local space
  transformControl.SetTransformMode(TransformMode::TM_TRANSLATION);
  transformControl.SetTransformSpace(TransformSpace::TS_LOCAL);
  transformControl.SetActiveAxis(math::Vector3d::UnitZ);
  transformControl.Translate(math::Vector3d(0, 0, 2));
  transformControl.Update();
  EXPECT_EQ(math::Vector3d(0, -2, 0), visual->WorldPosition());
  EXPECT_EQ(initialRot, visual->WorldRotation());
  EXPECT_EQ(math::Vector3d::One, visual->WorldScale());

  // test translation when snapping is enabled
  transformControl.SetActiveAxis(math::Vector3d::UnitY);
  transformControl.SetTransformSpace(TransformSpace::TS_WORLD);
  transformControl.Translate(math::Vector3d(0, 1, 0), true);
  transformControl.Update();
  EXPECT_EQ(math::Vector3d(0, -1, 0), visual->WorldPosition());
  EXPECT_EQ(initialRot, visual->WorldRotation());
  EXPECT_EQ(math::Vector3d::One, visual->WorldScale());

  // test rotation in local space
  transformControl.SetTransformMode(TransformMode::TM_ROTATION);
  transformControl.SetTransformSpace(TransformSpace::TS_LOCAL);
  transformControl.SetActiveAxis(math::Vector3d::UnitX);
  transformControl.Rotate(math::Quaterniond(IGN_PI, 0, 0));
  transformControl.Update();
  EXPECT_EQ(math::Vector3d(0, -1, 0), visual->WorldPosition());
  EXPECT_EQ(math::Quaterniond(IGN_PI, 0, 0) * initialRot,
      visual->WorldRotation());
  EXPECT_EQ(math::Vector3d::One, visual->WorldScale());

  // test rotation when snapping is enabled
  transformControl.SetActiveAxis(math::Vector3d::UnitY);
  transformControl.SetTransformSpace(TransformSpace::TS_WORLD);
  transformControl.Rotate(math::Quaterniond(0, IGN_PI, 0), true);
  transformControl.Update();
  EXPECT_EQ( math::Vector3d(0, -1, 0), visual->WorldPosition());
  EXPECT_EQ(math::Quaterniond(0, IGN_PI, 0) * math::Quaterniond(IGN_PI, 0, 0) *
      initialRot, visual->WorldRotation());
  EXPECT_EQ(math::Vector3d::One, visual->WorldScale());

  // test scaling in local space
  transformControl.SetTransformMode(TransformMode::TM_SCALE);
  transformControl.SetTransformSpace(TransformSpace::TS_LOCAL);
  transformControl.SetActiveAxis(math::Vector3d::UnitY);
  transformControl.Scale(math::Vector3d(1.0, 0.3, 1.0));
  transformControl.Update();
  EXPECT_EQ(math::Vector3d(0, -1, 0), visual->WorldPosition());
  EXPECT_EQ(math::Quaterniond(0, IGN_PI, 0) * math::Quaterniond(IGN_PI, 0, 0) *
      initialRot, visual->WorldRotation());

  auto expectedScale = math::Vector3d(1.0, 0.3, 1.0);
  EXPECT_EQ(expectedScale, visual->WorldScale());

  // test scaling when snapping is enabled
  auto newScale = math::Vector3d(2.0, 6.0, 1.2);
  transformControl.Scale(newScale, true);
  transformControl.Update();
  EXPECT_EQ(math::Vector3d(0, -1, 0), visual->WorldPosition());
  EXPECT_EQ(math::Quaterniond(0, IGN_PI, 0) * math::Quaterniond(IGN_PI, 0, 0) *
      initialRot, visual->WorldRotation());
  math::Vector3d snappedScale(std::round(newScale.X() * expectedScale.X()),
                              std::round(newScale.Y() * expectedScale.Y()),
                              std::round(newScale.Z() * expectedScale.Z()));
  EXPECT_EQ(snappedScale, visual->WorldScale());

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
void TransformControllerTest::Control2d(const std::string &_renderEngine)
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
  camera->SetWorldPosition(-5, 0, 0);
  EXPECT_EQ(math::Vector3d(-5, 0, 0), camera->WorldPosition());

  camera->SetImageWidth(320);
  camera->SetImageHeight(240);

  TransformController transformControl;

  // test translation and scale without a node
  math::Vector2d start0(0.5, 0.5);
  math::Vector2d end0(0.5, 0.8);
  EXPECT_EQ(math::Vector3d::Zero,
      transformControl.TranslationFrom2d(math::Vector3d::UnitZ, start0, end0));
  EXPECT_EQ(math::Vector3d::Zero,
      transformControl.ScaleFrom2d(math::Vector3d::UnitY, start0, end0));

  // create a dummy node visual node and attach to the controller
  VisualPtr visual = scene->CreateVisual();
  ASSERT_NE(nullptr, visual);
  transformControl.Attach(visual);
  EXPECT_EQ(visual, transformControl.Node());

  // test translation and scale without a camera
  EXPECT_EQ(math::Vector3d::Zero,
      transformControl.TranslationFrom2d(math::Vector3d::UnitZ, start0, end0));
  EXPECT_EQ(math::Vector3d::Zero,
      transformControl.ScaleFrom2d(math::Vector3d::UnitY, start0, end0));

  // test setting camera
  transformControl.SetCamera(camera);
  EXPECT_EQ(camera, transformControl.Camera());

  // test translation from 2d
  transformControl.SetTransformMode(TransformMode::TM_TRANSLATION);
  transformControl.SetTransformSpace(TransformSpace::TS_LOCAL);
  transformControl.SetActiveAxis(math::Vector3d::UnitZ);
  transformControl.Start();
  math::Vector2d start(0.5, 0.5);
  math::Vector2d end(0.5, 0.8);
  // translation in z
  math::Vector3d translation =
      transformControl.TranslationFrom2d(math::Vector3d::UnitZ, start, end);
  transformControl.Stop();
  EXPECT_DOUBLE_EQ(translation.X(), 0);
  EXPECT_DOUBLE_EQ(translation.Y(), 0);
  EXPECT_GT(translation.Z(), 0);

  // translation in y
  transformControl.SetActiveAxis(math::Vector3d::UnitY);
  transformControl.Start();
  math::Vector2d starty(0.5, 0.5);
  math::Vector2d endy(0.2, 0.5);
  translation =
      transformControl.TranslationFrom2d(math::Vector3d::UnitY, starty, endy);
  transformControl.Stop();
  EXPECT_DOUBLE_EQ(translation.X(), 0);
  EXPECT_GT(translation.Y(), 0);
  EXPECT_DOUBLE_EQ(translation.Z(), 0);

  // test rotation from 2d
  transformControl.SetTransformMode(TransformMode::TM_ROTATION);
  transformControl.SetTransformSpace(TransformSpace::TS_LOCAL);
  transformControl.SetActiveAxis(math::Vector3d::UnitX);
  transformControl.Start();
  start = math::Vector2d(0.5, 0.5);
  end = math::Vector2d(0.5, -0.8);
  math::Quaterniond rotation =
      transformControl.RotationFrom2d(math::Vector3d::UnitX, start, end);
  transformControl.Stop();
  math::Vector3d euler = rotation.Euler();
  EXPECT_GT(euler.X(), 0);
  EXPECT_DOUBLE_EQ(euler.Y(), 0);
  EXPECT_DOUBLE_EQ(euler.Z(), 0);

  // test scaling from 2d
  transformControl.SetTransformMode(TransformMode::TM_SCALE);
  transformControl.SetTransformSpace(TransformSpace::TS_LOCAL);
  transformControl.SetActiveAxis(math::Vector3d::UnitY);
  transformControl.Start();
  math::Vector3d scale =
      transformControl.ScaleFrom2d(math::Vector3d::UnitY, start, end);
  transformControl.Stop();
  EXPECT_DOUBLE_EQ(scale.X(), 1);
  EXPECT_GT(scale.Y(), 0);
  EXPECT_DOUBLE_EQ(scale.Z(), 1);

  // test snapping with invalid args
  EXPECT_EQ(math::Vector3d::Zero,
      transformControl.SnapPoint(math::Vector3d::One, -1));
  EXPECT_EQ(math::Vector3d::Zero,
      transformControl.SnapPoint(math::Vector3d::One, 1, -1));

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}
/////////////////////////////////////////////////
TEST_P(TransformControllerTest, TransformControl)
{
  TransformControl(GetParam());
}

/////////////////////////////////////////////////
TEST_P(TransformControllerTest, WorldSpace)
{
  WorldSpace(GetParam());
}

/////////////////////////////////////////////////
TEST_P(TransformControllerTest, LocalSpace)
{
  LocalSpace(GetParam());
}

/////////////////////////////////////////////////
TEST_P(TransformControllerTest, Control2d)
{
  Control2d(GetParam());
}

INSTANTIATE_TEST_CASE_P(TransformController, TransformControllerTest,
    RENDER_ENGINE_VALUES,
    PrintToStringParam());

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
