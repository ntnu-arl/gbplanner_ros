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
#include <string>

#include <ignition/common/Console.hh>

#include "test_config.h"  // NOLINT(build/include)

#include "ignition/rendering/LightVisual.hh"
#include "ignition/rendering/RenderEngine.hh"
#include "ignition/rendering/RenderingIface.hh"
#include "ignition/rendering/Scene.hh"

using namespace ignition;
using namespace rendering;

class LightVisualTest : public testing::Test,
                        public testing::WithParamInterface<const char *>
{
  /// \brief Test basic API
  public: void LightVisual(const std::string &_renderEngine);
};

/////////////////////////////////////////////////
void LightVisualTest::LightVisual(const std::string &_renderEngine)
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

  // create visual
  LightVisualPtr lightVisual = scene->CreateLightVisual();
  ASSERT_NE(nullptr, lightVisual);

  // check initial values
  EXPECT_EQ(LightVisualType::LVT_EMPTY, lightVisual->Type());
  EXPECT_DOUBLE_EQ(0, lightVisual->InnerAngle());
  EXPECT_DOUBLE_EQ(0, lightVisual->OuterAngle());

  lightVisual->SetType(LightVisualType::LVT_POINT);
  lightVisual->SetInnerAngle(0.7);
  lightVisual->SetOuterAngle(1.3);
  EXPECT_EQ(LightVisualType::LVT_POINT, lightVisual->Type());
  EXPECT_DOUBLE_EQ(0.7, lightVisual->InnerAngle());
  EXPECT_DOUBLE_EQ(1.3, lightVisual->OuterAngle());

  // Clean up
  engine->DestroyScene(scene);
  rendering::unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
TEST_P(LightVisualTest, LightVisual)
{
  LightVisual(GetParam());
}

INSTANTIATE_TEST_CASE_P(Visual, LightVisualTest,
    RENDER_ENGINE_VALUES,
    ignition::rendering::PrintToStringParam());

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
