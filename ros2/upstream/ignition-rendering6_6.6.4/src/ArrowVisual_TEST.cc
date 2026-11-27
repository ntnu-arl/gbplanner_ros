/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include "gz/rendering/ArrowVisual.hh"
#include "gz/rendering/RenderEngine.hh"
#include "gz/rendering/RenderingIface.hh"
#include "gz/rendering/Scene.hh"
#include "gz/rendering/Visual.hh"

using namespace ignition;
using namespace rendering;

class ArrowVisualTest : public testing::Test,
                        public testing::WithParamInterface<const char *>
{
  /// \brief Test basic API
  public: void ArrowVisual(const std::string &_renderEngine);

  /// \brief Test gizmo material
  public: void Material(const std::string &_renderEngine);
};

/////////////////////////////////////////////////
void ArrowVisualTest::ArrowVisual(const std::string &_renderEngine)
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
  }

  ScenePtr scene = engine->CreateScene("scene");

  // create arrow visual
  ArrowVisualPtr visual = scene->CreateArrowVisual();
  ASSERT_NE(nullptr, visual);

  // check scale
  EXPECT_TRUE(visual->InheritScale());
  EXPECT_EQ(math::Vector3d::One, visual->LocalScale());
  EXPECT_EQ(math::Vector3d::One, visual->WorldScale());

  visual->SetLocalScale(0.2, 0.3, 0.4);
  EXPECT_EQ(math::Vector3d(0.2, 0.3, 0.4), visual->LocalScale());

  // check children and geometry
  EXPECT_EQ(3u, visual->ChildCount());

  NodePtr node = visual->ChildByIndex(0u);
  VisualPtr child = std::dynamic_pointer_cast<Visual>(node);
  ASSERT_NE(nullptr, child);
  EXPECT_EQ(1u, child->GeometryCount());
  EXPECT_EQ(node, visual->Rotation());

  node = visual->ChildByIndex(1u);
  child = std::dynamic_pointer_cast<Visual>(node);
  ASSERT_NE(nullptr, child);
  EXPECT_EQ(1u, child->GeometryCount());
  EXPECT_EQ(node, visual->Shaft());

  node = visual->ChildByIndex(2u);
  child = std::dynamic_pointer_cast<Visual>(node);
  ASSERT_NE(nullptr, child);
  EXPECT_EQ(1u, child->GeometryCount());
  EXPECT_EQ(node, visual->Head());

  // test destroy
  ArrowVisualPtr visual2 = scene->CreateArrowVisual();
  ASSERT_NE(nullptr, visual2);
  visual2->Destroy();
  EXPECT_EQ(0u, visual2->ChildCount());

  // Clean up
  engine->DestroyScene(scene);
  rendering::unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
TEST_P(ArrowVisualTest, ArrowVisual)
{
  ArrowVisual(GetParam());
}

INSTANTIATE_TEST_CASE_P(ArrowVisual, ArrowVisualTest,
    RENDER_ENGINE_VALUES,
    ignition::rendering::PrintToStringParam());

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
