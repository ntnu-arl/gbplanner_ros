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
#include "ignition/rendering/SegmentationCamera.hh"

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
class SegmentationCameraTest: public testing::Test,
  public testing::WithParamInterface<const char *>
{
  public: void SegmentationCameraBoxes(const std::string &_renderEngine);

  // Documentation inherited
  protected: void SetUp() override
  {
    ignition::common::Console::SetVerbosity(4);
  }
};

/// \brief mutex for thread safety
std::mutex g_mutex;

/// \brief Segmentation buffer
uint8_t *g_buffer = nullptr;

/// \brief counter of received segmentation msgs
int g_counter = 0;

//////////////////////////////////////////////////
/// \brief callback to get the segmentation buffer
void OnNewSegmentationFrame(const uint8_t *_data,
                    unsigned int _width, unsigned int _height,
                    unsigned int /*_channels*/,
                    const std::string &/*_format*/)
{
  g_mutex.lock();
  auto bufferSize = _width * _height * 3;

  if (!g_buffer)
    g_buffer = new uint8_t[bufferSize];

  memcpy(g_buffer, _data, bufferSize);

  g_counter++;
  g_mutex.unlock();
}

//////////////////////////////////////////////////
/// \brief Build the scene with 3 boxes besides each other
/// the 2 outer boxes have the same label & the middle is different
void BuildScene(rendering::ScenePtr scene)
{
  math::Vector3d leftPosition(3, 1.5, 0);
  math::Vector3d rightPosition(3, -1.5, 0);
  math::Vector3d middlePosition(3, 0, 0);

  rendering::VisualPtr root = scene->RootVisual();

  // create box visual
  rendering::VisualPtr box = scene->CreateVisual("box_left");
  box->AddGeometry(scene->CreateBox());
  box->SetOrigin(0.0, 0.0, 0.0);
  box->SetLocalPosition(leftPosition);
  box->SetLocalRotation(0, 0, 0);
  box->SetUserData("label", 1);
  root->AddChild(box);

  // create box visual of same label
  rendering::VisualPtr box1 = scene->CreateVisual("box_right");
  box1->AddGeometry(scene->CreateBox());
  box1->SetOrigin(0.0, 0.0, 0.0);
  box1->SetLocalPosition(rightPosition);
  box1->SetLocalRotation(0, 0, 0);
  box1->SetUserData("label", 1);
  root->AddChild(box1);

  // create box visual of different label
  ignition::rendering::VisualPtr box2 = scene->CreateVisual("box_mid");
  box2->AddGeometry(scene->CreateBox());
  box2->SetOrigin(0.0, 0.0, 0.0);
  box2->SetLocalPosition(middlePosition);
  box2->SetLocalRotation(0, 0, 0);
  box2->SetUserData("label", 2);
  root->AddChild(box2);
}

//////////////////////////////////////////////////
void SegmentationCameraTest::SegmentationCameraBoxes(
  const std::string &_renderEngine)
{
  // Currently, only ogre2 supports segmentation cameras
  if (_renderEngine.compare("ogre2") != 0)
  {
    ignerr << "Engine '" << _renderEngine
              << "' doesn't support segmentation cameras" << std::endl;
    return;
  }

  // Setup ign-rendering with an empty scene
  auto *engine = ignition::rendering::engine(_renderEngine);
  if (!engine)
  {
    ignerr << "Engine '" << _renderEngine
              << "' was unable to be retrieved" << std::endl;
    return;
  }
  ignition::rendering::ScenePtr scene = engine->CreateScene("scene");
  ASSERT_NE(nullptr, scene);
  BuildScene(scene);

  // Create Segmentation camera
  auto camera = scene->CreateSegmentationCamera("SegmentationCamera");
  ASSERT_NE(camera, nullptr);

  camera->SetLocalPosition(0.0, 0.0, 0.0);
  camera->SetLocalRotation(0.0, 0.0, 0.0);

  int backgroundLabel = 23;
  camera->SetBackgroundLabel(backgroundLabel);
  EXPECT_EQ(camera->BackgroundLabel(), backgroundLabel);

  // We will test semantic segmentation first
  camera->SetSegmentationType(SegmentationType::ST_SEMANTIC);
  EXPECT_EQ(camera->Type(), SegmentationType::ST_SEMANTIC);

  camera->EnableColoredMap(false);
  EXPECT_FALSE(camera->IsColoredMap());

  int width = 320;
  int height = 240;
  double aspectRatio = static_cast<double>(width) / static_cast<double>(height);

  camera->SetAspectRatio(aspectRatio);
  camera->SetImageWidth(width);
  camera->SetImageHeight(height);
  camera->SetHFOV(IGN_PI / 2);

  // add camera to the scene
  scene->RootVisual()->AddChild(camera);

  // Set a callback on the  camera sensor to get a Segmentation camera frame
  ignition::common::ConnectionPtr connection =
      camera->ConnectNewSegmentationFrame(
          std::bind(OnNewSegmentationFrame,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));
  ASSERT_NE(nullptr, connection);

  // Update once to create image
  camera->Update();
  EXPECT_EQ(1, g_counter);

  // get the center of each box, the percentages locates the center
  math::Vector2d leftProj(width * 0.25, height * 0.5);
  math::Vector2d rightProj(width * 0.75, height * 0.5);
  math::Vector2d middleProj(width * 0.5, height * 0.5);

  // get their index in the buffer
  auto leftIndex =
    static_cast<uint32_t>((leftProj.Y() * width + leftProj.X()) * 3);
  auto rightIndex =
    static_cast<uint32_t>((rightProj.Y() * width + rightProj.X()) * 3);
  auto middleIndex =
    static_cast<uint32_t>((middleProj.Y() * width + middleProj.X()) * 3);

  // cast the unsigned char to unsigned int to read it
  uint8_t leftLabel =   g_buffer[leftIndex];
  uint8_t rightLabel =  g_buffer[rightIndex];
  uint8_t middleLabel = g_buffer[middleIndex];

  // check the label
  EXPECT_EQ(leftLabel  , 1);
  EXPECT_EQ(middleLabel, 2);
  EXPECT_EQ(rightLabel , 1);

  // check if the first pixel(background) = the background label
  int background = g_buffer[0];
  EXPECT_EQ(background, backgroundLabel);

  // Instance/Panoptic test
  camera->SetSegmentationType(SegmentationType::ST_PANOPTIC);

  // Update once to create image
  g_counter = 0;
  camera->Update();
  EXPECT_EQ(1, g_counter);

  // the label is in the last channel
  leftLabel =   g_buffer[leftIndex   + 2];
  rightLabel =  g_buffer[rightIndex  + 2];
  middleLabel = g_buffer[middleIndex + 2];

  // the instances count in the first channel
  uint8_t leftCount =   g_buffer[leftIndex];
  uint8_t rightCount =  g_buffer[rightIndex];
  uint8_t middleCount = g_buffer[middleIndex];

  // check the label
  EXPECT_EQ(1, leftLabel);
  EXPECT_EQ(2, middleLabel);
  EXPECT_EQ(1, rightLabel);

  // instance count
  // the instance count for the right box is 1 and the instance count for the
  // left box is 2 because of how items are sorted when material switching:
  // https://github.com/ignitionrobotics/ign-rendering/blob/c4e06851605bda75e2ca45a35f0e9bd86fbd7f2f/ogre2/src/Ogre2SegmentationMaterialSwitcher.cc#L171-L179
  EXPECT_EQ(1, middleCount);
  EXPECT_EQ(1, rightCount);
  EXPECT_EQ(2, leftCount);

  // Clean up
  engine->DestroyScene(scene);
  ignition::rendering::unloadEngine(engine->Name());
}

TEST_P(SegmentationCameraTest, SegmentationCameraBoxes)
{
  SegmentationCameraBoxes(GetParam());
}

INSTANTIATE_TEST_CASE_P(SegmentationCamera, SegmentationCameraTest,
    RENDER_ENGINE_VALUES, ignition::rendering::PrintToStringParam());

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
