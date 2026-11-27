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
#include <cmath>

#include <gz/common/Console.hh>

#include "test_config.h"  // NOLINT(build/include)
#include "gz/rendering/GaussianNoisePass.hh"
#include "gz/rendering/RenderEngine.hh"
#include "gz/rendering/RenderingIface.hh"
#include "gz/rendering/RenderPassSystem.hh"

using namespace gz;
using namespace rendering;

class GaussianNoisePassTest : public testing::Test,
                              public testing::WithParamInterface<const char*>
{
  /// \brief Test Gaussian noise pass properties
  public: void GaussianNoise(const std::string &_renderEngine);
};

/////////////////////////////////////////////////
void GaussianNoisePassTest::GaussianNoise(const std::string &_renderEngine)
{
  // get engine
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
  RenderPassPtr pass = rpSystem->Create<GaussianNoisePass>();
  GaussianNoisePassPtr noisePass =
      std::dynamic_pointer_cast<GaussianNoisePass>(pass);
  EXPECT_NE(nullptr, noisePass);

  // verify initial values
  EXPECT_DOUBLE_EQ(0u, noisePass->Mean());
  EXPECT_DOUBLE_EQ(0u, noisePass->StdDev());
  EXPECT_DOUBLE_EQ(0u, noisePass->Bias());

  // mean
  double mean = 0.23;
  noisePass->SetMean(mean);
  EXPECT_DOUBLE_EQ(mean, noisePass->Mean());

  // std dev
  double stdDev = 0.069;
  noisePass->SetStdDev(stdDev);
  EXPECT_DOUBLE_EQ(stdDev, noisePass->StdDev());

  // bias
  double biasMean = 0.1;
  noisePass->SetBiasMean(biasMean);
  EXPECT_DOUBLE_EQ(biasMean, std::fabs(noisePass->Bias()));
  std::cerr << "bias mean " << noisePass->Bias() << std::endl;

  double biasStdDev = 0.007;
  noisePass->SetBiasStdDev(biasStdDev);
  // expect bias to be within 3-sigma
  // Note, tol relaxed to 4-sigma to fix flaky test
  EXPECT_LE(std::fabs(noisePass->Bias()), biasMean + biasStdDev*4);
  EXPECT_GE(std::fabs(noisePass->Bias()), biasMean - biasStdDev*4);
}

/////////////////////////////////////////////////
TEST_P(GaussianNoisePassTest, GaussianNoise)
{
  GaussianNoise(GetParam());
}

INSTANTIATE_TEST_CASE_P(GaussianNoise, GaussianNoisePassTest,
    RENDER_ENGINE_VALUES,
    PrintToStringParam());

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
