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

#include "gz/rendering/InertiaVisual.hh"
#include "gz/rendering/RenderEngine.hh"
#include "gz/rendering/RenderingIface.hh"
#include "gz/rendering/Scene.hh"

using namespace ignition;
using namespace rendering;

class InertiaVisualTest : public testing::Test,
                        public testing::WithParamInterface<const char *>
{
  /// \brief Test basic API
  public: void InertiaVisual(const std::string &_renderEngine);
};

/////////////////////////////////////////////////
void InertiaVisualTest::InertiaVisual(const std::string &_renderEngine)
{
  RenderEngine *engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    FAIL() << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }

  ScenePtr scene = engine->CreateScene("scene");

  // create visual
  InertiaVisualPtr inertiaVisual = scene->CreateInertiaVisual();
  ASSERT_NE(nullptr, inertiaVisual);

  // check initial values
  EXPECT_EQ(nullptr, inertiaVisual->BoxVisual());

  ignition::math::MassMatrix3d massMatrix(
      2.0, {2.0, 1.5, 1.0}, {0.0, 0.0, 0.0});
  ignition::math::Pose3d p(0.0, 1.0, 2.5, 1.0, 0.4, 0.4);
  ignition::math::Inertiald inertial;

  inertiaVisual->SetInertial(inertial);
  EXPECT_EQ(nullptr, inertiaVisual->BoxVisual());

  inertial.SetMassMatrix(massMatrix);
  inertial.SetPose(p);
  inertiaVisual->SetInertial(inertial);
  EXPECT_NE(nullptr, inertiaVisual->BoxVisual());

  // Clean up
  engine->DestroyScene(scene);
  rendering::unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
TEST_P(InertiaVisualTest, InertiaVisual)
{
  InertiaVisual(GetParam());
}

INSTANTIATE_TEST_CASE_P(Visual, InertiaVisualTest,
    RENDER_ENGINE_VALUES,
    ignition::rendering::PrintToStringParam());

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
