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

#include <gtest/gtest.h>

#include <gz/common/Console.hh>

#include "test_config.h"  // NOLINT(build/include)
#include "gz/rendering/Light.hh"
#include "gz/rendering/RenderEngine.hh"
#include "gz/rendering/RenderingIface.hh"
#include "gz/rendering/Scene.hh"

using namespace gz;
using namespace rendering;

class LightTest : public testing::Test,
                  public testing::WithParamInterface<const char*>
{
  /// \brief Test light APIs
  public: void Light(const std::string &_renderEngine);
};


/////////////////////////////////////////////////
void LightTest::Light(const std::string &_renderEngine)
{
#ifdef __APPLE__                                                                              
  std::cerr << "Skipping test for apple, see issue #847." << std::endl;                       
  return;
#endif
  // create and populate scene
  RenderEngine *engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    FAIL() << "Engine '" << _renderEngine
           << "' is not supported" << std::endl;
    return;
  }
  ScenePtr scene = engine->CreateScene("scene");
  ASSERT_NE(nullptr, scene);

  // Point light
  LightPtr light = scene->CreatePointLight();
  EXPECT_TRUE(light != nullptr);
  EXPECT_EQ(1u, scene->LightCount());
  EXPECT_TRUE(scene->HasLight(light));

  // diffuse
  light->SetDiffuseColor(1, 0, 0);
  EXPECT_EQ(math::Color(1, 0, 0), light->DiffuseColor());
  light->SetDiffuseColor(math::Color(0, 1, 0));
  EXPECT_EQ(math::Color(0, 1, 0), light->DiffuseColor());

  // specular
  light->SetSpecularColor(1, 1, 1);
  EXPECT_EQ(math::Color(1, 1, 1), light->SpecularColor());
  light->SetSpecularColor(math::Color(1, 1, 0));
  EXPECT_EQ(math::Color(1, 1, 0), light->SpecularColor());

  // cast shadows
  light->SetCastShadows(true);
  EXPECT_TRUE(light->CastShadows());
  light->SetCastShadows(false);
  EXPECT_FALSE(light->CastShadows());

  // intensity
  light->SetIntensity(1.25);
  EXPECT_NEAR(1.25, light->Intensity(), 1e-6);

  // attenuation
  // Checking near because Ogre stores it as float
  light->SetAttenuationConstant(0.6);
  EXPECT_NEAR(0.6, light->AttenuationConstant(), 1e-6);
  light->SetAttenuationLinear(0.2);
  EXPECT_NEAR(0.2, light->AttenuationLinear(), 1e-6);
  light->SetAttenuationQuadratic(0.01);
  EXPECT_NEAR(0.01, light->AttenuationQuadratic(), 1e-6);
  light->SetAttenuationRange(10);
  EXPECT_DOUBLE_EQ(10, light->AttenuationRange());

  // Directional light
  DirectionalLightPtr dirLight = scene->CreateDirectionalLight();
  EXPECT_TRUE(dirLight != nullptr);
  EXPECT_EQ(2u, scene->LightCount());
  EXPECT_TRUE(scene->HasLightName(dirLight->Name()));

  math::Vector3d dir = math::Vector3d(-0.2, -0.1, -0.9).Normalize();
  dirLight->SetDirection(dir.X(), dir.Y(), dir.Z());
  EXPECT_EQ(dir, dirLight->Direction());
  dir = math::Vector3d(0.4, 0.3, 0.2).Normalize();
  dirLight->SetDirection(dir);
  EXPECT_EQ(dir, dirLight->Direction());

  dirLight->SetLocalPose(math::Pose3d(10, 0, 0, 0, 0, 0));
  EXPECT_EQ(math::Pose3d(10, 0, 0, 0, 0, 0), dirLight->LocalPose());
  EXPECT_EQ(dir, dirLight->Direction());

  // Spot light
  SpotLightPtr spotLight = scene->CreateSpotLight();
  EXPECT_TRUE(spotLight != nullptr);
  EXPECT_EQ(3u, scene->LightCount());
  EXPECT_TRUE(scene->HasLightId(dirLight->Id()));

  dir = math::Vector3d(-0.2, -0.1, -0.9).Normalize();
  spotLight->SetDirection(dir.X(), dir.Y(), dir.Z());
  EXPECT_EQ(dir, spotLight->Direction());
  dir = math::Vector3d(0.4, 0.3, 0.2).Normalize();
  spotLight->SetDirection(dir);
  EXPECT_EQ(dir, spotLight->Direction());
  spotLight->SetInnerAngle(1.1);
  EXPECT_EQ(math::Angle(1.1), spotLight->InnerAngle());
  spotLight->SetInnerAngle(math::Angle(0.1));
  EXPECT_EQ(math::Angle(0.1), spotLight->InnerAngle());
  spotLight->SetOuterAngle(2.2);
  EXPECT_EQ(math::Angle(2.2), spotLight->OuterAngle());
  spotLight->SetOuterAngle(math::Angle(0.2));
  EXPECT_EQ(math::Angle(0.2), spotLight->OuterAngle());
  spotLight->SetFalloff(0.2);
  EXPECT_NEAR(0.2, spotLight->Falloff(), 1e-6);

  // remove lights
  scene->DestroyLightById(light->Id());
  EXPECT_EQ(2u, scene->LightCount());
  scene->DestroyLightByName(dirLight->Name());
  EXPECT_EQ(1u, scene->LightCount());
  scene->DestroyLight(spotLight);
  EXPECT_EQ(0u, scene->LightCount());

  // Verify we can repeatedly create and remove lights
  for (unsigned i = 0; i < 5u; ++i)
  {
    DirectionalLightPtr lightx = scene->CreateDirectionalLight();
    EXPECT_TRUE(lightx != nullptr);
    EXPECT_EQ(1u, scene->LightCount());
    scene->DestroyLight(lightx);
    EXPECT_EQ(0u, scene->LightCount());
  }

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
TEST_P(LightTest, Light)
{
  Light(GetParam());
}

INSTANTIATE_TEST_CASE_P(Light, LightTest,
    RENDER_ENGINE_VALUES,
    PrintToStringParam());

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
