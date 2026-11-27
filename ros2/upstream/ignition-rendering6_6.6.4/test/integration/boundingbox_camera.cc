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
#include <ignition/common/Filesystem.hh>
#include <ignition/common/Event.hh>

#include <ignition/math/Color.hh>

#include "test_config.h"  // NOLINT(build/include)

#include "ignition/rendering/RenderEngine.hh"
#include "ignition/rendering/RenderingIface.hh"
#include "ignition/rendering/Scene.hh"
#include "ignition/rendering/BoundingBoxCamera.hh"

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
class BoundingBoxCameraTest: public testing::Test,
  public testing::WithParamInterface<const char *>
{
  /// \brief Test 2d boxes (full boxes and visible boxes modes)
  /// with a scene that contains overlaped / occluded boxes
  public: void OccludedBoxes(const std::string &_renderEngine);

  /// \brief Test 2d boxes with a scene with 2 boxes besides
  public: void SimpleBoxes(const std::string &_renderEngine);

  /// \brief Test 3d oriented boxes with a scene with single box
  public: void Oriented3dBoxes(const std::string &_renderEngine);

  // Documentation inherited
  protected: void SetUp() override
  {
    ignition::common::Console::SetVerbosity(4);
  }
};

/// \brief mutex for thread safety
std::mutex g_mutex;

/// \brief bounding boxes from the camera
std::vector<BoundingBox> g_boxes;

/// \brief callback to receive boxes from the camera
void OnNewBoundingBoxes(const std::vector<BoundingBox> &boxes)
{
  g_mutex.lock();
  g_boxes.clear();

  for (auto box : boxes)
    g_boxes.push_back(box);

  g_mutex.unlock();
}

/// \brief Build the scene with 2 boxes besides
void BuildSimpleScene(rendering::ScenePtr scene)
{
  math::Vector3d leftPosition(3, 1.5, 0);
  math::Vector3d rightPosition(3, -1.5, 0);

  rendering::VisualPtr root = scene->RootVisual();

  // create front box visual (the smaller box)
  rendering::VisualPtr leftBox = scene->CreateVisual();
  leftBox->AddGeometry(scene->CreateBox());
  leftBox->SetOrigin(0.0, 0.0, 0.0);
  leftBox->SetLocalPosition(leftPosition);
  leftBox->SetLocalRotation(0, 0, 0);
  leftBox->SetUserData("label", 1);
  root->AddChild(leftBox);

  // create occluded box visual (the bigger box)
  rendering::VisualPtr rightBox = scene->CreateVisual();
  rightBox->AddGeometry(scene->CreateBox());
  rightBox->SetOrigin(0.0, 0.0, 0.0);
  rightBox->SetLocalPosition(rightPosition);
  rightBox->SetLocalRotation(0, 0, 0);
  rightBox->SetUserData("label", 2);
  root->AddChild(rightBox);
}

/// \brief Build the scene with 3 boxes 2 overlapping boxes which 1
/// is behind the other, and the 3rd box is invisible behind them
void BuildScene(rendering::ScenePtr scene)
{
  math::Vector3d occludedPosition(4, 1, 0);
  math::Vector3d frontPosition(2, 0, 0);
  math::Vector3d invisiblePosition(5, 0, 0);

  rendering::VisualPtr root = scene->RootVisual();

  // create front box visual (the smaller box)
  rendering::VisualPtr occludedBox = scene->CreateVisual();
  occludedBox->AddGeometry(scene->CreateBox());
  occludedBox->SetOrigin(0.0, 0.0, 0.0);
  occludedBox->SetLocalPosition(occludedPosition);
  occludedBox->SetLocalRotation(0, 0, 0);
  occludedBox->SetUserData("label", 1);
  root->AddChild(occludedBox);

  // create occluded box visual (the bigger box)
  rendering::VisualPtr frontBox = scene->CreateVisual();
  frontBox->AddGeometry(scene->CreateBox());
  frontBox->SetOrigin(0.0, 0.0, 0.0);
  frontBox->SetLocalPosition(frontPosition);
  frontBox->SetLocalRotation(0, 0, 0);
  frontBox->SetUserData("label", 2);
  root->AddChild(frontBox);

  rendering::VisualPtr invisibleBox = scene->CreateVisual();
  invisibleBox->AddGeometry(scene->CreateBox());
  invisibleBox->SetOrigin(0.0, 0.0, 0.0);
  invisibleBox->SetLocalPosition(invisiblePosition);
  invisibleBox->SetLocalRotation(0, 0, 0);
  invisibleBox->SetUserData("label", 2);
  root->AddChild(invisibleBox);
}

/// \brief Build a scene with a single oriented box for 3D test
void Build3dBoxScene(rendering::ScenePtr scene)
{
  math::Vector3d leftPosition(3, 1.5, 0);
  math::Vector3d rightPosition(3, -1.5, 0);

  rendering::VisualPtr root = scene->RootVisual();

  // create front box visual (the smaller box)
  rendering::VisualPtr box = scene->CreateVisual();
  box->AddGeometry(scene->CreateBox());
  box->SetOrigin(0.0, 0.0, 0.0);
  box->SetLocalPosition(math::Vector3d(3, 0, 0));
  box->SetLocalRotation(0.1, 0, 0.7);
  box->SetUserData("label", 1);
  root->AddChild(box);
}

//////////////////////////////////////////////////
void BoundingBoxCameraTest::SimpleBoxes(
  const std::string &_renderEngine)
{
  // Not all engines are supported
  if (_renderEngine.compare("optix") == 0 ||
      _renderEngine.compare("ogre") == 0)
  {
    igndbg << "Engine '" << _renderEngine
           << "' doesn't support bounding box cameras" << std::endl;
    return;
  }

  // Setup ign-rendering with an empty scene
  auto *engine = ignition::rendering::engine(_renderEngine);
  if (!engine)
  {
    igndbg << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }

  ignition::rendering::ScenePtr scene = engine->CreateScene("scene");
  ASSERT_NE(nullptr, scene);
  BuildSimpleScene(scene);

  // Create BoundingBox camera
  auto camera = scene->CreateBoundingBoxCamera("BoundingBoxCamera");
  ASSERT_NE(camera, nullptr);

  camera->SetLocalPosition(0.0, 0.0, 0.0);
  camera->SetLocalRotation(0.0, 0.0, 0.0);

  unsigned int width = 320;
  unsigned int height = 240;

  camera->SetImageWidth(width);
  camera->SetImageHeight(height);
  camera->SetAspectRatio(1.333);
  camera->SetHFOV(IGN_PI / 2);
  camera->SetBoundingBoxType(BoundingBoxType::BBT_VISIBLEBOX2D);

  EXPECT_EQ(camera->ImageWidth(), width);
  EXPECT_EQ(camera->ImageHeight(), height);

  // add camera to the scene
  scene->RootVisual()->AddChild(camera);

  // Set a callback on the  camera sensor to get a BoundingBox camera frame
  ignition::common::ConnectionPtr connection =
    camera->ConnectNewBoundingBoxes(
      std::bind(OnNewBoundingBoxes, std::placeholders::_1));
  EXPECT_NE(nullptr, connection);

  // Update once to create image
  camera->Update();

  g_mutex.lock();
  EXPECT_EQ(g_boxes.size(), std::size_t(2));

  BoundingBox leftBox = g_boxes[0];
  BoundingBox rightBox = g_boxes[1];

  double middleScreen = 0.5 * width;
  unsigned int leftLabel = 1;
  unsigned int rightLabel = 2;

  // check if the left box has x positions < middle screen
  EXPECT_LT(leftBox.Center().X() - leftBox.Size().X() / 2, middleScreen);
  EXPECT_LT(leftBox.Center().X() + leftBox.Size().X() / 2, middleScreen);
  EXPECT_EQ(leftBox.Label(), leftLabel);

  // check if the right box has x positions > middle screen
  EXPECT_GT(rightBox.Center().X() - leftBox.Size().X() / 2, middleScreen);
  EXPECT_GT(rightBox.Center().X() + leftBox.Size().X() / 2, middleScreen);
  EXPECT_EQ(rightBox.Label(), rightLabel);

  g_mutex.unlock();

  // Clean up
  engine->DestroyScene(scene);
  ignition::rendering::unloadEngine(engine->Name());
}

//////////////////////////////////////////////////
void BoundingBoxCameraTest::OccludedBoxes(
  const std::string &_renderEngine)
{
  // accepted error with +/- in pixels in comparing the box coordinates
  int marginError = 2;

  // Not all engines are supported
  if (_renderEngine.compare("optix") == 0 ||
      _renderEngine.compare("ogre") == 0)
  {
    igndbg << "Engine '" << _renderEngine
           << "' doesn't support bounding box cameras" << std::endl;
    return;
  }

  // Setup ign-rendering with an empty scene
  auto *engine = ignition::rendering::engine(_renderEngine);
  if (!engine)
  {
    igndbg << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }

  ignition::rendering::ScenePtr scene = engine->CreateScene("scene");
  ASSERT_NE(nullptr, scene);
  BuildScene(scene);

  // Create BoundingBox camera
  auto camera = scene->CreateBoundingBoxCamera("BoundingBoxCamera");
  ASSERT_NE(camera, nullptr);

  camera->SetLocalPosition(0.0, 0.0, 0.0);
  camera->SetLocalRotation(0.0, 0.0, 0.0);

  camera->SetBoundingBoxType(BoundingBoxType::BBT_VISIBLEBOX2D);
  EXPECT_EQ(camera->Type(), BoundingBoxType::BBT_VISIBLEBOX2D);

  unsigned int width = 320;
  unsigned int height = 240;

  camera->SetImageWidth(width);
  camera->SetImageHeight(height);
  camera->SetAspectRatio(1.333);
  camera->SetHFOV(IGN_PI / 2);

  EXPECT_EQ(camera->ImageWidth(), width);
  EXPECT_EQ(camera->ImageHeight(), height);

  // add camera to the scene
  scene->RootVisual()->AddChild(camera);

  // Set a callback on the  camera sensor to get a BoundingBox camera frame
  ignition::common::ConnectionPtr connection =
    camera->ConnectNewBoundingBoxes(
      std::bind(OnNewBoundingBoxes, std::placeholders::_1));
  EXPECT_NE(nullptr, connection);

  // Update once to create image
  camera->Update();

  // Visible Type Test
  g_mutex.lock();

  // check if the invisible 3rd box is not exist
  EXPECT_EQ(g_boxes.size(), size_t(2));

  BoundingBox occludedBox = g_boxes[0];
  BoundingBox frontBox = g_boxes[1];

  unsigned int occludedLabel = 1;
  unsigned int frontLabel = 2;

  // hard-coded comparasion with acceptable error
  EXPECT_NEAR(occludedBox.Center().X(), 98, marginError);
  EXPECT_NEAR(occludedBox.Center().Y(), 119, marginError);
  EXPECT_NEAR(occludedBox.Size().X(), 15, marginError);
  EXPECT_NEAR(occludedBox.Size().Y(), 45, marginError);
  EXPECT_EQ(occludedBox.Label(), occludedLabel);

  EXPECT_NEAR(frontBox.Center().X(), 159, marginError);
  EXPECT_NEAR(frontBox.Center().Y(), 119, marginError);
  EXPECT_NEAR(frontBox.Size().X(), 105, marginError);
  EXPECT_NEAR(frontBox.Size().Y(), 105, marginError);
  EXPECT_EQ(frontBox.Label(), frontLabel);

  g_mutex.unlock();

  // Full Boxes Type Test
  camera->SetBoundingBoxType(BoundingBoxType::BBT_FULLBOX2D);
  // Update once to create image
  camera->Update();

  // test
  g_mutex.lock();

  // check the hidden box
  EXPECT_EQ(g_boxes.size(), size_t(2));

  BoundingBox occludedFullBox = g_boxes[0];
  BoundingBox frontFullBox = g_boxes[1];

  // coordinates of partially occluded object is bigger
  EXPECT_NEAR(occludedFullBox.Center().X(), 116, marginError);
  EXPECT_NEAR(occludedFullBox.Center().Y(), 119, marginError);
  EXPECT_NEAR(occludedFullBox.Size().X(), 51, marginError);
  EXPECT_NEAR(occludedFullBox.Size().Y(), 45, marginError);
  EXPECT_EQ(occludedFullBox.Label(), occludedLabel);

  EXPECT_NEAR(frontFullBox.Center().X(), 159, marginError);
  EXPECT_NEAR(frontFullBox.Center().Y(), 119, marginError);
  EXPECT_NEAR(frontFullBox.Size().X(), 108, marginError);
  EXPECT_NEAR(frontFullBox.Size().Y(), 108, marginError);

  EXPECT_EQ(frontFullBox.Label(), frontLabel);

  g_mutex.unlock();

  // Clean up
  engine->DestroyScene(scene);
  ignition::rendering::unloadEngine(engine->Name());
}

//////////////////////////////////////////////////
void BoundingBoxCameraTest::Oriented3dBoxes(
  const std::string &_renderEngine)
{
  // Not all engines are supported
  if (_renderEngine.compare("optix") == 0 ||
      _renderEngine.compare("ogre") == 0)
  {
    igndbg << "Engine '" << _renderEngine
           << "' doesn't support bounding box cameras" << std::endl;
    return;
  }

  // Setup ign-rendering with an empty scene
  auto *engine = ignition::rendering::engine(_renderEngine);
  if (!engine)
  {
    igndbg << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }

  ignition::rendering::ScenePtr scene = engine->CreateScene("scene");
  ASSERT_NE(nullptr, scene);
  Build3dBoxScene(scene);

  // Create BoundingBox camera
  auto camera = scene->CreateBoundingBoxCamera("BoundingBoxCamera");
  ASSERT_NE(camera, nullptr);

  camera->SetLocalPosition(0.0, 0.0, 0.0);
  camera->SetLocalRotation(0.0, 0.0, 0.0);

  unsigned int width = 320;
  unsigned int height = 240;

  camera->SetImageWidth(width);
  camera->SetImageHeight(height);
  camera->SetAspectRatio(1.333);
  camera->SetHFOV(IGN_PI / 2);
  camera->SetBoundingBoxType(BoundingBoxType::BBT_BOX3D);

  EXPECT_EQ(camera->ImageWidth(), width);
  EXPECT_EQ(camera->ImageHeight(), height);

  // add camera to the scene
  scene->RootVisual()->AddChild(camera);

  // Set a callback on the  camera sensor to get a BoundingBox camera frame
  ignition::common::ConnectionPtr connection =
    camera->ConnectNewBoundingBoxes(
      std::bind(OnNewBoundingBoxes, std::placeholders::_1));
  EXPECT_NE(nullptr, connection);

  // Update once to create image
  camera->Update();

  g_mutex.lock();
  EXPECT_EQ(g_boxes.size(), std::size_t(1));

  BoundingBox box = g_boxes[0];

  // accepted error with +/- in pixels in comparing the box coordinates
  double marginError = 0.1;

  EXPECT_NEAR(box.Center().X(), 0, marginError);
  EXPECT_NEAR(box.Center().Y(), 0, marginError);
  EXPECT_NEAR(box.Center().Z(), -3, marginError);

  EXPECT_NEAR(box.Orientation().Roll(), 1.6708, marginError);
  EXPECT_NEAR(box.Orientation().Pitch(), 0.870796, marginError);
  EXPECT_NEAR(box.Orientation().Yaw(), -3.14159, marginError);

  EXPECT_EQ(box.Label(), 1u);
  g_mutex.unlock();

  // Clean up
  engine->DestroyScene(scene);
  ignition::rendering::unloadEngine(engine->Name());
}

TEST_P(BoundingBoxCameraTest, SimpleBoxes)
{
  SimpleBoxes(GetParam());
}

TEST_P(BoundingBoxCameraTest, OccludedBoxes)
{
  OccludedBoxes(GetParam());
}

TEST_P(BoundingBoxCameraTest, Oriented3dBoxes)
{
  Oriented3dBoxes(GetParam());
}

INSTANTIATE_TEST_CASE_P(BoundingBoxCamera, BoundingBoxCameraTest,
    RENDER_ENGINE_VALUES, ignition::rendering::PrintToStringParam());

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
