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

#include <ignition/common/Console.hh>

#include "test_config.h"  // NOLINT(build/include)

#include "ignition/rendering/RenderingIface.hh"
#include "ignition/rendering/RenderEngine.hh"
#include "ignition/rendering/RenderEngineManager.hh"

using namespace ignition;
using namespace rendering;

class RenderEngineManagerTest : public testing::Test,
                  public testing::WithParamInterface<const char *>
{
  public: void RenderEngineManager(const std::string &_renderEngine);
};

/////////////////////////////////////////////////
void RenderEngineManagerTest::RenderEngineManager(
  const std::string &_renderEngine)
{
  // Tested missing APIs
  EXPECT_FALSE(RenderEngineManager::Instance()->HasEngine("invalid"));
  EXPECT_EQ(nullptr, RenderEngineManager::Instance()->Engine("invalid"));
  EXPECT_FALSE(RenderEngineManager::Instance()->UnloadEngine("invalid"));

  EXPECT_FALSE(RenderEngineManager::Instance()->UnloadEngineAt(100));

  RenderEngineManager::Instance()->RegisterEngine("invalid", nullptr);

  auto engine = rendering::engine(_renderEngine);
  RenderEngineManager::Instance()->RegisterEngine("ogre2", engine);

  RenderEngineManager::Instance()->UnregisterEngine(engine);
  EXPECT_FALSE(RenderEngineManager::Instance()->UnloadEngineAt(0));

  std::list<std::string> paths = {"path1", "path2"};
  RenderEngineManager::Instance()->SetPluginPaths(paths);
}

/////////////////////////////////////////////////
TEST_P(RenderEngineManagerTest, RenderEngineManager)
{
  RenderEngineManager(GetParam());
}

INSTANTIATE_TEST_CASE_P(RenderEngineManager, RenderEngineManagerTest,
    RENDER_ENGINE_VALUES,
    PrintToStringParam());

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
