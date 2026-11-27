/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

// The following is needed to enable the GetMemInfo function for OSX
#ifdef __MACH__
# include <mach/mach.h>
#endif  // __MACH__

#include <gtest/gtest.h>

#include <gz/common/Console.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "test_config.h"  // NOLINT(build/include)

#include "gz/rendering/RenderEngine.hh"
#include "gz/rendering/RenderingIface.hh"
#include "gz/rendering/Scene.hh"

using namespace gz;
using namespace rendering;


/// \brief Create and destroy objects using Scene, profile memory usage and
/// check for memory leak
class SceneFactoryTest: public testing::Test,
                     public testing::WithParamInterface<const char *>
{
  /// \brief Test creating and destroying materials
  public: void MaterialMemoryLeak(const std::string &_renderEngine);

  /// \brief Test creating and destroying visuals
  public: void VisualMemoryLeak(const std::string &_renderEngine);
};


/////////////////////////////////////////////////
void getMemInfo(double &_resident, double &_share)
{
#ifdef __linux__
  int totalSize, residentPages, sharePages;
  totalSize = residentPages = sharePages = 0;

  std::ifstream buffer("/proc/self/statm");
  buffer >> totalSize >> residentPages >> sharePages;
  buffer.close();

  // in case x86-64 is configured to use 2MB pages
  int64_t pageSizeKb = sysconf(_SC_PAGE_SIZE) / 1024;

  _resident = residentPages * pageSizeKb;
  _share = sharePages * pageSizeKb;
#elif __MACH__
  // /proc is only available on Linux
  // for OSX, use task_info to get resident and virtual memory
  struct task_basic_info t_info;
  mach_msg_type_number_t t_info_count = TASK_BASIC_INFO_COUNT;
  if (KERN_SUCCESS != task_info(mach_task_self(),
                                TASK_BASIC_INFO,
                                (task_info_t)&t_info,
                                &t_info_count))
  {
    ignerr << "failure calling task_info\n";
    return;
  }
  _resident = static_cast<double>(t_info.resident_size/1024);
  _share = static_cast<double>(t_info.virtual_size/1024);
#else
  ignerr << "Unsupported architecture\n";
  return;
#endif
}

/////////////////////////////////////////////////
void checkMemLeak(const std::string &_renderEngine,
    const std::function<void(ScenePtr)> &_cb)
{
  auto engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    igndbg << "Engine '" << _renderEngine << "' is not supported" << std::endl;
    return;
  }

  auto scene = engine->CreateScene("scene");
  ASSERT_NE(nullptr, scene);

  // max memory change allowed
  const double resMaxPercentChange = 3.0;
  const double shareMaxPercentChange = 1.0;

  // get initial memory usage
  double residentStart = 0;
  double shareStart = 0;
  getMemInfo(residentStart, shareStart);

  // run the code
  _cb(scene);

  // get final memory usage
  double residentEnd = 0;
  double shareEnd = 0;
  getMemInfo(residentEnd, shareEnd);

  // Calculate the percent change from the initial resident and shared memory
  double resPercentChange = (residentEnd - residentStart) / residentStart;
  double sharePercentChange = (shareEnd - shareStart) / shareStart;

  igndbg << "ResPercentChange[" << resPercentChange << "] "
    << "ResMaxPercentChange[" << resMaxPercentChange << "]" << std::endl;
  igndbg << "SharePercentChange[" << sharePercentChange << "] "
    << "ShareMaxPercentChange[" << shareMaxPercentChange << "]" << std::endl;

  EXPECT_LT(resPercentChange, resMaxPercentChange);
  EXPECT_LT(sharePercentChange, shareMaxPercentChange);

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
void SceneFactoryTest::MaterialMemoryLeak(const std::string &_renderEngine)
{
  auto function = [](ScenePtr _scene)
  {
    const unsigned int numMaterials = 5000;
    for (unsigned int i = 0; i < numMaterials; ++i)
    {
      MaterialPtr mat = _scene->CreateMaterial();
      _scene->DestroyMaterial(mat);
    }
  };

  checkMemLeak(_renderEngine, function);
}

/////////////////////////////////////////////////
void SceneFactoryTest::VisualMemoryLeak(const std::string &_renderEngine)
{
  auto function = [](ScenePtr _scene)
  {
    const unsigned int numCycles = 4;
    const unsigned int numVisuals = 5000;

    for (unsigned int j = 0; j < numCycles; ++j)
    {
      // parent visual
      auto parent = _scene->CreateVisual("parent");
      // Create a N visuals
      for (unsigned int i = 0; i < numVisuals; ++i)
      {
        std::stringstream ss;
        ss << "child" << i;
        auto child = _scene->CreateVisual(ss.str());
        auto box = _scene->CreateBox();
        child->AddGeometry(box);
        parent->AddChild(child);
      }
      // Recursive destroy - all child visuals should also be destroyed
      _scene->DestroyVisual(parent, true);
    }
  };

  checkMemLeak(_renderEngine, function);
}

/////////////////////////////////////////////////
TEST_P(SceneFactoryTest,
       IGN_UTILS_TEST_DISABLED_ON_WIN32(MaterialMemoryLeak))
{
  MaterialMemoryLeak(GetParam());
}

/////////////////////////////////////////////////
TEST_P(SceneFactoryTest,
       IGN_UTILS_TEST_DISABLED_ON_WIN32(VisualMemoryLeak))
{
  VisualMemoryLeak(GetParam());
}

INSTANTIATE_TEST_CASE_P(SceneFactory, SceneFactoryTest,
    RENDER_ENGINE_VALUES,
    PrintToStringParam());

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
