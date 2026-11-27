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
#include "gz/rendering/DistortionPass.hh"
#include "gz/rendering/GaussianNoisePass.hh"
#include "gz/rendering/RenderEngine.hh"
#include "gz/rendering/RenderingIface.hh"
#include "gz/rendering/RenderPassSystem.hh"

using namespace gz;
using namespace rendering;

class RenderPassSystemTest : public testing::Test,
                             public testing::WithParamInterface<const char*>
{
  /// \brief Test creating render passes
  public: void RenderPassSystem(const std::string &_renderEngine);
};

/////////////////////////////////////////////////
void RenderPassSystemTest::RenderPassSystem(const std::string &_renderEngine)
{
  //  get engine
  RenderEngine *engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    FAIL() << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }

  // get the render pass system
  RenderPassSystemPtr rpSystem = engine->RenderPassSystem();
  if (!rpSystem)
  {
    ignwarn << "Render engin '" << _renderEngine << "' does not support "
            << "render pass system" << std::endl;
    return;
  }

  // test creating abstract render pass
  RenderPassPtr nullPass = rpSystem->Create<RenderPass>();
  EXPECT_EQ(nullptr, nullPass);

  // test passing invalid class type
  RenderPassPtr invalidPass = rpSystem->Create<RenderEngine>();
  EXPECT_EQ(nullptr, invalidPass);

  // create valid gaussian noise pass
  RenderPassPtr pass = rpSystem->Create<GaussianNoisePass>();
  EXPECT_NE(nullptr, pass);
  GaussianNoisePassPtr noisePass =
      std::dynamic_pointer_cast<GaussianNoisePass>(pass);
  EXPECT_NE(nullptr, noisePass);

  // create distortion pass
  RenderPassPtr dpass = rpSystem->Create<DistortionPass>();
  if (_renderEngine == "ogre")
  {
    EXPECT_NE(nullptr, dpass);
    DistortionPassPtr distortionPass =
        std::dynamic_pointer_cast<DistortionPass>(dpass);
    EXPECT_NE(nullptr, distortionPass);
  }
}

/////////////////////////////////////////////////
TEST_P(RenderPassSystemTest, RenderPassSystem)
{
  RenderPassSystem(GetParam());
}

INSTANTIATE_TEST_CASE_P(GaussianNoise, RenderPassSystemTest,
    RENDER_ENGINE_VALUES,
    PrintToStringParam());

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
