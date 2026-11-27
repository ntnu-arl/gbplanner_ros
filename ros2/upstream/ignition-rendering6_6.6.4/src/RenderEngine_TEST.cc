/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include "gz/rendering/RenderingIface.hh"
#include "gz/rendering/Scene.hh"
#include "gz/rendering/RenderEngine.hh"

using namespace gz;
using namespace rendering;

class RenderEngineTest : public testing::Test,
                  public testing::WithParamInterface<const char *>
{
  public: void RenderEngine(const std::string &_renderEngine);
};

/////////////////////////////////////////////////
void RenderEngineTest::RenderEngine(const std::string &_renderEngine)
{
  // Get engine
  auto engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    FAIL() << "Engine '" << _renderEngine
           << "' is not supported" << std::endl;
    return;
  }

  EXPECT_EQ(_renderEngine, engine->Name());
  EXPECT_TRUE(engine->IsEnabled());
  EXPECT_TRUE(engine->IsLoaded());

  engine->AddResourcePath("none");

  // Check there are no scenes
  EXPECT_EQ(0u, engine->SceneCount());
  EXPECT_FALSE(engine->HasSceneName("scene1"));
  EXPECT_EQ(nullptr, engine->SceneByName("scene1"));
  EXPECT_EQ(nullptr, engine->SceneByIndex(0));

  // Create a scene
  auto scene1 = engine->CreateScene("scene1");
  ASSERT_NE(nullptr, scene1);
  EXPECT_EQ(engine->SceneCount(), 1u);
  EXPECT_TRUE(engine->HasSceneName("scene1"));
  EXPECT_TRUE(engine->HasSceneId(scene1->Id()));
  EXPECT_TRUE(engine->HasScene(scene1));
  EXPECT_EQ(scene1, engine->SceneByName("scene1"));
  EXPECT_EQ(scene1, engine->SceneById(scene1->Id()));
  EXPECT_EQ(scene1, engine->SceneByIndex(0));

  // Create a scene with the same name
  EXPECT_EQ(nullptr, engine->CreateScene("scene1"));
  EXPECT_EQ(engine->SceneCount(), 1u);

  // Create a scene with the same id
  EXPECT_EQ(nullptr, engine->CreateScene(scene1->Id(), "new_scene"));
  EXPECT_EQ(engine->SceneCount(), 1u);

  // Create another scene
  auto scene2 = engine->CreateScene("scene2");
  ASSERT_NE(nullptr, scene2);
  EXPECT_EQ(engine->SceneCount(), 2u);
  EXPECT_TRUE(engine->HasSceneName("scene2"));
  EXPECT_TRUE(engine->HasSceneId(scene2->Id()));
  EXPECT_TRUE(engine->HasScene(scene2));
  EXPECT_EQ(scene2, engine->SceneByName("scene2"));
  EXPECT_EQ(scene2, engine->SceneById(scene2->Id()));
  EXPECT_EQ(scene2, engine->SceneByIndex(1));

  // Destroy the first scene
  engine->DestroyScene(scene1);
  EXPECT_EQ(engine->SceneCount(), 1u);
  EXPECT_FALSE(engine->HasSceneName("scene1"));
  EXPECT_FALSE(engine->HasSceneId(scene1->Id()));
  EXPECT_FALSE(engine->HasScene(scene1));
  EXPECT_EQ(nullptr, engine->SceneByName("scene1"));
  EXPECT_EQ(nullptr, engine->SceneById(scene1->Id()));

  // FIXME: shared pointers are not released when scene is destroyed
  // See issue #13
  // EXPECT_EQ(1u, scene1.use_count());

  // Check scene2 is now the first one
  EXPECT_EQ(scene2, engine->SceneByIndex(0));
  EXPECT_EQ(nullptr, engine->SceneByIndex(1));

  // Create more scenes so we can test destroy's
  auto scene3 = engine->CreateScene(12345, "scene3");
  ASSERT_NE(nullptr, scene3);
  EXPECT_EQ(engine->SceneCount(), 2u);

  auto scene4 = engine->CreateScene("scene4");
  ASSERT_NE(nullptr, scene4);
  EXPECT_EQ(engine->SceneCount(), 3u);

  auto scene5 = engine->CreateScene("scene5");
  ASSERT_NE(nullptr, scene5);
  EXPECT_EQ(engine->SceneCount(), 4u);

  auto scene6 = engine->CreateScene("scene6");
  ASSERT_NE(nullptr, scene6);
  EXPECT_EQ(engine->SceneCount(), 5u);

  // Destroy scenes
  engine->DestroySceneById(scene2->Id());
  EXPECT_EQ(engine->SceneCount(), 4u);

  engine->DestroySceneByName("scene3");
  EXPECT_EQ(engine->SceneCount(), 3u);

  engine->DestroySceneByIndex(0);
  EXPECT_EQ(engine->SceneCount(), 2u);

  engine->DestroyScenes();
  EXPECT_EQ(engine->SceneCount(), 0u);

  // Clean up
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
TEST_P(RenderEngineTest, RenderEngine)
{
  RenderEngine(GetParam());
}

INSTANTIATE_TEST_CASE_P(RenderEngine, RenderEngineTest,
    RENDER_ENGINE_VALUES,
    PrintToStringParam());

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
