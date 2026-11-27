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

#include "test_config.h"  // NOLINT(build/include)

#include <gz/common/Console.hh>

#include "gz/rendering/config.hh"
#include "gz/rendering/RenderEngine.hh"
#include "gz/rendering/RenderingIface.hh"

using namespace gz;
using namespace rendering;

/////////////////////////////////////////////////
// get the number of default engines available
unsigned int defaultEnginesForTest()
{
  unsigned int count = 0u;
#if HAVE_OGRE
  count++;
#endif
#if HAVE_OGRE2
  count++;
#endif
#if HAVE_OPTIX
  count++;
#endif
  return count;
}

/////////////////////////////////////////////////
TEST(RenderingIfaceTest, GetEngine)
{
#ifdef __APPLE__                                                                              
  std::cerr << "Skipping test for apple, see issue #847." << std::endl;                       
  return;
#endif
  common::Console::SetVerbosity(4);

  unsigned int count = defaultEnginesForTest();
  EXPECT_GT(count, 0u);

  EXPECT_EQ(count, engineCount());

  EXPECT_TRUE(loadedEngines().empty());
  EXPECT_FALSE(isEngineLoaded("ogre"));
  EXPECT_FALSE(isEngineLoaded("ogre2"));
  EXPECT_FALSE(isEngineLoaded("optix"));
  EXPECT_FALSE(isEngineLoaded("no_such_engine"));
  EXPECT_EQ(nullptr, sceneFromFirstRenderEngine());

  // check get engine
  for (unsigned int i = 0; i < count; ++i)
  {
    RenderEngine *eng = engine(i, std::map<std::string, std::string>(),
        IGN_RENDERING_TEST_PLUGIN_PATH);
    ASSERT_NE(nullptr, eng);
    EXPECT_TRUE(isEngineLoaded(eng->Name()));
    EXPECT_TRUE(hasEngine(eng->Name()));
    EXPECT_EQ(eng, engine(eng->Name()));

#if HAVE_OGRE && HAVE_OGRE2
    // TODO(anyone): ogre and ogre2 cannot be loaded at the same time
    // so for now only test rendering engine API with one ogre version
    if (eng->Name() == "ogre" || eng->Name() == "ogre2")
      ++i;
#endif

#ifndef _WIN32
    // Windows CI fails with
    // Ogre::RenderingAPIException::RenderingAPIException: OpenGL 1.5 is not
    // supported in GLRenderSystem::initialiseContext
    auto scene = eng->CreateScene("scene");
    EXPECT_NE(nullptr, scene);

    EXPECT_EQ(scene, sceneFromFirstRenderEngine());
#endif

    ASSERT_EQ(1u, loadedEngines().size());
    EXPECT_EQ(eng->Name(), loadedEngines()[0]);

    unloadEngine(eng->Name());
    EXPECT_FALSE(isEngineLoaded(eng->Name()));
  }

  EXPECT_TRUE(loadedEngines().empty());
  EXPECT_EQ(nullptr, sceneFromFirstRenderEngine());

  // non-existent engine
  EXPECT_EQ(nullptr, engine("no_such_engine"));
  EXPECT_EQ(nullptr, engine(1000000));
}

/////////////////////////////////////////////////
TEST(RenderingIfaceTest, RegisterEngine)
{
  common::Console::SetVerbosity(4);

  unsigned int count = defaultEnginesForTest();

  if (count == 0)
    return;

  // unregister existing engine by index
  RenderEngine *eng = engine(0u);
  EXPECT_TRUE(hasEngine(eng->Name()));
  EXPECT_NO_THROW(unregisterEngine(0u));
  EXPECT_FALSE(hasEngine(eng->Name()));

  // register engine back with a different name
  EXPECT_NO_THROW(registerEngine("my_new_engine", eng));
  EXPECT_TRUE(hasEngine("my_new_engine"));

  // unregister engine by name
  EXPECT_NO_THROW(unregisterEngine("my_new_engine"));
  EXPECT_FALSE(hasEngine("my_new_engine"));

  // unregister non-existent engine
  EXPECT_NO_THROW(unregisterEngine("no_such_engine"));
  EXPECT_NO_THROW(unregisterEngine(1000000u));
  EXPECT_NO_THROW(unregisterEngine(nullptr));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
