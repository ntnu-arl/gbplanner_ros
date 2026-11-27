/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include "ignition/rendering/ArrowVisual.hh"
#include "ignition/rendering/AxisVisual.hh"
#include "ignition/rendering/RenderEngine.hh"
#include "ignition/rendering/RenderingIface.hh"
#include "ignition/rendering/Scene.hh"
#include "ignition/rendering/Visual.hh"

using namespace ignition;
using namespace rendering;

class AxisVisualTest : public testing::Test,
                       public testing::WithParamInterface<const char *>
{
  /// \brief Test basic API
  public: void AxisVisual(const std::string &_renderEngine);

  /// \brief Test gizmo material
  public: void Material(const std::string &_renderEngine);
};

/////////////////////////////////////////////////
void AxisVisualTest::AxisVisual(const std::string &_renderEngine)
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

  // create axis visual
  AxisVisualPtr visual = scene->CreateAxisVisual();
  ASSERT_NE(nullptr, visual);

  // check scale
  EXPECT_TRUE(visual->InheritScale());
  EXPECT_EQ(math::Vector3d::One, visual->LocalScale());
  EXPECT_EQ(math::Vector3d::One, visual->WorldScale());

  visual->SetLocalScale(0.2, 0.3, 0.4);
  EXPECT_EQ(math::Vector3d(0.2, 0.3, 0.4), visual->LocalScale());

  // check children and geometry
  EXPECT_EQ(3u, visual->ChildCount());

  for (unsigned int i = 0; i < 3u; ++i)
  {
    NodePtr node = visual->ChildByIndex(i);
    ArrowVisualPtr arrow = std::dynamic_pointer_cast<ArrowVisual>(node);
    ASSERT_NE(nullptr, arrow);

    EXPECT_EQ(3u, arrow->ChildCount());
    NodePtr childNode = arrow->ChildByIndex(0u);
    VisualPtr child = std::dynamic_pointer_cast<Visual>(childNode);
    EXPECT_EQ(1u, child->GeometryCount());

    childNode = arrow->ChildByIndex(1u);
    child = std::dynamic_pointer_cast<Visual>(childNode);
    EXPECT_EQ(1u, child->GeometryCount());

    childNode = arrow->ChildByIndex(2u);
    child = std::dynamic_pointer_cast<Visual>(childNode);
    EXPECT_EQ(1u, child->GeometryCount());
  }

  // create visual
  AxisVisualPtr axis_name = scene->CreateAxisVisual("axis_name");
  EXPECT_NE(nullptr, axis_name);

  AxisVisualPtr axis_id = scene->CreateAxisVisual(101);
  EXPECT_NE(nullptr, axis_id);

  // Clean up
  engine->DestroyScene(scene);
  rendering::unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
TEST_P(AxisVisualTest, AxisVisual)
{
  AxisVisual(GetParam());
}

INSTANTIATE_TEST_CASE_P(AxisVisual, AxisVisualTest,
    RENDER_ENGINE_VALUES,
    ignition::rendering::PrintToStringParam());

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
