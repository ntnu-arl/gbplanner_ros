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
#include "gz/rendering/ThermalCamera.hh"
#include "gz/rendering/RenderEngine.hh"
#include "gz/rendering/RenderingIface.hh"
#include "gz/rendering/Scene.hh"

using namespace gz;
using namespace rendering;

class ThermalCameraTest : public testing::Test,
                          public testing::WithParamInterface<const char *>
{
  /// \brief Test basic api
  public: void ThermalCamera(const std::string &_renderEngine);
};

/////////////////////////////////////////////////
void ThermalCameraTest::ThermalCamera(const std::string &_renderEngine)
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

  ThermalCameraPtr camera(scene->CreateThermalCamera());
  EXPECT_TRUE(camera != nullptr);

  float ambient = 310.5;
  camera->SetAmbientTemperature(ambient);
  EXPECT_FLOAT_EQ(ambient, camera->AmbientTemperature());

  float range = 3.35f;
  camera->SetAmbientTemperatureRange(range);
  EXPECT_FLOAT_EQ(range, camera->AmbientTemperatureRange());

  float minTemp = 250.05f;
  camera->SetMinTemperature(minTemp);
  EXPECT_FLOAT_EQ(minTemp, camera->MinTemperature());

  float maxTemp = 380.06f;
  camera->SetMaxTemperature(maxTemp);
  EXPECT_FLOAT_EQ(maxTemp, camera->MaxTemperature());

  float resolution = 0.04f;
  camera->SetLinearResolution(resolution);
  EXPECT_FLOAT_EQ(resolution, camera->LinearResolution());

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
TEST_P(ThermalCameraTest, ThermalCamera)
{
  ThermalCamera(GetParam());
}

INSTANTIATE_TEST_CASE_P(ThermalCamera, ThermalCameraTest,
    RENDER_ENGINE_VALUES,
    PrintToStringParam());

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
