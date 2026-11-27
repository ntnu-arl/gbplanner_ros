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
#include <ignition/math/Color.hh>

#include "test_config.h"  // NOLINT(build/include)

#include "ignition/rendering/ParticleEmitter.hh"
#include "ignition/rendering/RenderEngine.hh"
#include "ignition/rendering/RenderingIface.hh"
#include "ignition/rendering/Scene.hh"

using namespace ignition;
using namespace rendering;

/// \brief The test fixture.
class ParticleEmitterTest : public testing::Test,
                            public testing::WithParamInterface<const char *>
{
  /// \brief How to set up the environment.
  public: bool SetUp(const std::string &_renderEngine);

  /// \brief Check setters/getters.
  public: void CheckBasicAPI();

  /// \brief How to tear down the environment.
  public: void TearDown();

  /// \brief A directory under test/ with some textures.
  protected: const std::string TEST_MEDIA_PATH =
      common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test", "media", "materials", "textures");

  /// \brief The rendering engine used.
  protected: RenderEngine *engine;

  /// \brief The scene.
  protected: ScenePtr scene;
};

/////////////////////////////////////////////////
bool ParticleEmitterTest::SetUp(const std::string &_renderEngine)
{
  this->engine = rendering::engine(_renderEngine);
  if (!this->engine)
  {
    igndbg << "Engine '" << _renderEngine << "' is not supported" << std::endl;
    return false;
  }

  this->scene = this->engine->CreateScene("scene");
  return this->scene != nullptr;
}

/////////////////////////////////////////////////
void ParticleEmitterTest::CheckBasicAPI()
{
  // Create a particle emitter.
  ParticleEmitterPtr particleEmitter = this->scene->CreateParticleEmitter();

  // Default values.
  EmitterType    expectedEmitterType     = EmitterType::EM_POINT;
  math::Vector3d expectedEmitterSize     = ignition::math::Vector3d::One;
  double         expectedRate            = 10.0;
  double         expectedDuration        = 0;
  bool           expectedEmitting        = false;
  math::Vector3d expectedParticleSize    = {1, 1, 1};
  double         expectedLifetime        = 5;
  MaterialPtr    expectedMaterial        = nullptr;
  double         expectedMinVel          = 1;
  double         expectedMaxVel          = 1;
  math::Color    expectedColorStart      = ignition::math::Color::White;
  math::Color    expectedColorEnd        = ignition::math::Color::White;
  double         expectedScaleRate       = 1;
  std::string    expectedColorRangeImage = "";
  float          particleScatterRatio    = 0.65f;

  // Check default expectations.
  EXPECT_EQ(expectedEmitterType,      particleEmitter->Type());
  EXPECT_EQ(expectedEmitterSize,      particleEmitter->EmitterSize());
  EXPECT_DOUBLE_EQ(expectedRate,      particleEmitter->Rate());
  EXPECT_DOUBLE_EQ(expectedDuration,  particleEmitter->Duration());
  EXPECT_EQ(expectedEmitting,         particleEmitter->Emitting());
  EXPECT_EQ(expectedParticleSize,     particleEmitter->ParticleSize());
  EXPECT_DOUBLE_EQ(expectedLifetime,  particleEmitter->Lifetime());
  EXPECT_EQ(expectedMaterial,         particleEmitter->Material());
  EXPECT_DOUBLE_EQ(expectedMinVel,    particleEmitter->MinVelocity());
  EXPECT_DOUBLE_EQ(expectedMaxVel,    particleEmitter->MaxVelocity());
  EXPECT_EQ(expectedColorStart,       particleEmitter->ColorStart());
  EXPECT_EQ(expectedColorEnd,         particleEmitter->ColorEnd());
  EXPECT_DOUBLE_EQ(expectedScaleRate, particleEmitter->ScaleRate());
  EXPECT_EQ(expectedColorRangeImage,  particleEmitter->ColorRangeImage());
  EXPECT_FLOAT_EQ(particleScatterRatio,
      particleEmitter->ParticleScatterRatio());

  // Modify values.
  expectedEmitterType     = EmitterType::EM_BOX;
  expectedEmitterSize     = {0.2, 0.2, 0.2};
  expectedRate            = 5.0;
  expectedDuration        = 30;
  expectedEmitting        = true;
  expectedParticleSize    = {200, 300, 400};
  expectedLifetime        = 10;
  expectedMaterial        = nullptr;
  expectedMinVel          = 2;
  expectedMaxVel          = 3;
  expectedColorStart      = ignition::math::Color::Red;
  expectedColorEnd        = ignition::math::Color::Blue;
  expectedScaleRate       = 10;
  expectedColorRangeImage = common::joinPaths(TEST_MEDIA_PATH, "texture.png");
  float expectedScatterRatio  = 0.24f;

  // Modify attributes.
  particleEmitter->SetType(expectedEmitterType);
  particleEmitter->SetEmitterSize(expectedEmitterSize);
  particleEmitter->SetRate(expectedRate);
  particleEmitter->SetDuration(expectedDuration);
  particleEmitter->SetEmitting(expectedEmitting);
  particleEmitter->SetParticleSize(expectedParticleSize);
  particleEmitter->SetLifetime(expectedLifetime);
  particleEmitter->SetMaterial(expectedMaterial);
  particleEmitter->SetVelocityRange(expectedMinVel, expectedMaxVel);
  particleEmitter->SetColorRange(expectedColorStart, expectedColorEnd);
  particleEmitter->SetScaleRate(expectedScaleRate);
  particleEmitter->SetColorRangeImage(expectedColorRangeImage);
  particleEmitter->SetParticleScatterRatio(expectedScatterRatio);

  // Check getters.
  EXPECT_EQ(expectedEmitterType,      particleEmitter->Type());
  EXPECT_EQ(expectedEmitterSize,      particleEmitter->EmitterSize());
  EXPECT_DOUBLE_EQ(expectedRate,      particleEmitter->Rate());
  EXPECT_DOUBLE_EQ(expectedDuration,  particleEmitter->Duration());
  EXPECT_EQ(expectedEmitting,         particleEmitter->Emitting());
  EXPECT_EQ(expectedParticleSize,     particleEmitter->ParticleSize());
  EXPECT_DOUBLE_EQ(expectedLifetime,  particleEmitter->Lifetime());
  EXPECT_EQ(expectedMaterial,         particleEmitter->Material());
  EXPECT_DOUBLE_EQ(expectedMinVel,    particleEmitter->MinVelocity());
  EXPECT_DOUBLE_EQ(expectedMaxVel,    particleEmitter->MaxVelocity());
  EXPECT_EQ(expectedColorStart,       particleEmitter->ColorStart());
  EXPECT_EQ(expectedColorEnd,         particleEmitter->ColorEnd());
  EXPECT_DOUBLE_EQ(expectedScaleRate, particleEmitter->ScaleRate());
  EXPECT_EQ(expectedColorRangeImage,  particleEmitter->ColorRangeImage());
  EXPECT_FLOAT_EQ(expectedScatterRatio,
      particleEmitter->ParticleScatterRatio());
}

/////////////////////////////////////////////////
void ParticleEmitterTest::TearDown()
{
  // Clean up
  if (this->scene && this->engine)
  {
    this->engine->DestroyScene(this->scene);
    rendering::unloadEngine(this->engine->Name());
  }
}

/////////////////////////////////////////////////
TEST_P(ParticleEmitterTest, ParticleEmitter)
{
  if (SetUp(GetParam()))
  {
    this->CheckBasicAPI();
  }
  // TearDown() happens automatically.
}

INSTANTIATE_TEST_CASE_P(ParticleEmitter, ParticleEmitterTest,
    RENDER_ENGINE_VALUES,
    ignition::rendering::PrintToStringParam());

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
