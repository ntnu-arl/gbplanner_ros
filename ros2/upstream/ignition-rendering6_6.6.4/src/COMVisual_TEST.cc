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

#include "ignition/rendering/COMVisual.hh"
#include "ignition/rendering/RenderEngine.hh"
#include "ignition/rendering/RenderingIface.hh"
#include "ignition/rendering/Scene.hh"

using namespace ignition;
using namespace rendering;

class COMVisualTest : public testing::Test,
                        public testing::WithParamInterface<const char *>
{
  /// \brief Test basic API
  public: void COMVisual(const std::string &_renderEngine);
};

/////////////////////////////////////////////////
void COMVisualTest::COMVisual(const std::string &_renderEngine)
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
  COMVisualPtr comVisual = scene->CreateCOMVisual();
  ASSERT_NE(nullptr, comVisual);

  ignition::math::MassMatrix3d massMatrix(
      0.0, {2.0, 1.5, 1.0}, {0.0, 0.0, 0.0});
  ignition::math::Pose3d p(0.0, 1.0, 2.5, 1.0, 0.4, 0.4);
  ignition::math::Inertiald inertial;
  inertial.SetMassMatrix(massMatrix);
  inertial.SetPose(p);

  // check initial values
  EXPECT_EQ(nullptr, comVisual->SphereVisual());
  EXPECT_EQ(ignition::math::Pose3d::Zero, comVisual->InertiaPose());
  EXPECT_DOUBLE_EQ(1.0, comVisual->Mass());

  // set invalid mass
  comVisual->SetMass(-1.0);
  comVisual->PreRender();
  EXPECT_EQ(nullptr, comVisual->SphereVisual());
  EXPECT_DOUBLE_EQ(1.0, comVisual->Mass());
  EXPECT_EQ(ignition::math::Pose3d::Zero, comVisual->InertiaPose());

  // set invalid inertial
  comVisual->SetInertial(inertial);
  comVisual->PreRender();
  EXPECT_EQ(nullptr, comVisual->SphereVisual());
  EXPECT_DOUBLE_EQ(1.0, comVisual->Mass());
  EXPECT_EQ(inertial.Pose(), comVisual->InertiaPose());

  // set valid mass
  comVisual->SetMass(2.0);
  EXPECT_EQ(nullptr, comVisual->SphereVisual());
  EXPECT_DOUBLE_EQ(2.0, comVisual->Mass());

  // set parent
  VisualPtr parentVisual = scene->CreateVisual("parent_visual");
  parentVisual->AddChild(comVisual);
  comVisual->PreRender();
  EXPECT_NE(nullptr, comVisual->SphereVisual());

  // Clean up
  engine->DestroyScene(scene);
  rendering::unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
TEST_P(COMVisualTest, COMVisual)
{
  COMVisual(GetParam());
}

INSTANTIATE_TEST_CASE_P(Visual, COMVisualTest,
    RENDER_ENGINE_VALUES,
    ignition::rendering::PrintToStringParam());

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
