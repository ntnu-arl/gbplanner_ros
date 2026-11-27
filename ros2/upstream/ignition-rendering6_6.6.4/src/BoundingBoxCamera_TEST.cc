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
#include "ignition/rendering/BoundingBoxCamera.hh"
#include "ignition/rendering/RenderEngine.hh"
#include "ignition/rendering/RenderingIface.hh"
#include "ignition/rendering/Scene.hh"

using namespace ignition;
using namespace rendering;

class BoundingBoxCameraTest : public testing::Test,
                          public testing::WithParamInterface<const char *>
{
  /// \brief Test basic api
  public: void BoundingBoxCamera(const std::string &_renderEngine);
};

/////////////////////////////////////////////////
void BoundingBoxCameraTest::BoundingBoxCamera(const std::string &_renderEngine)
{
  // Not all engines are supported
  if (_renderEngine.compare("optix") == 0 ||
      _renderEngine.compare("ogre") == 0)
  {
    igndbg << "Engine '" << _renderEngine
           << "' doesn't support bounding box cameras" << std::endl;
    return;
  }

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

  BoundingBoxCameraPtr camera(scene->CreateBoundingBoxCamera());
  ASSERT_NE(nullptr, camera);
  camera->SetBoundingBoxType(BoundingBoxType::BBT_VISIBLEBOX2D);

  // Test Draw Bounding Box
  int width = 200;
  int height = 200;
  int channels = 3;

  unsigned char *data = new unsigned char[width * height * channels];

  BoundingBox box;
  box.SetCenter({125, 125, 0});
  box.SetSize({50, 50, 0});

  camera->SetImageWidth(width);
  camera->SetImageHeight(height);
  camera->DrawBoundingBox(data, ignition::math::Color::Green, box);

  // test the boundaries (min & max box's points) color
  uint32_t minX = uint32_t(box.Center().X() - box.Size().X() / 2);
  uint32_t minY = uint32_t(box.Center().Y() - box.Size().Y() / 2);
  uint32_t maxX = uint32_t(box.Center().X() + box.Size().X() / 2);
  uint32_t maxY = uint32_t(box.Center().Y() + box.Size().Y() / 2);
  int minIndex = (minY * width + minX) * channels;
  int maxIndex = (maxY * width + (maxX-1)) * channels;

  // color is green (middle value = 255)
  EXPECT_EQ(data[minIndex + 1], 255);
  EXPECT_EQ(data[maxIndex + 1], 255);

  // Test Bounding Box Type
  EXPECT_EQ(camera->Type(), BoundingBoxType::BBT_VISIBLEBOX2D);
  camera->SetBoundingBoxType(BoundingBoxType::BBT_FULLBOX2D);
  EXPECT_EQ(camera->Type(), BoundingBoxType::BBT_FULLBOX2D);

  // Clean up
  engine->DestroyScene(scene);
  rendering::unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
TEST_P(BoundingBoxCameraTest, BoundingBoxCamera)
{
  BoundingBoxCamera(GetParam());
}

INSTANTIATE_TEST_CASE_P(BoundingBoxCamera, BoundingBoxCameraTest,
    RENDER_ENGINE_VALUES,
    ignition::rendering::PrintToStringParam());

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
