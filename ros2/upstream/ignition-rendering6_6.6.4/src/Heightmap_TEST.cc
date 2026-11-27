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

#include <ignition/common/Console.hh>
#include <ignition/common/ImageHeightmap.hh>
#include <ignition/utils/ExtraTestMacros.hh>

#include "test_config.h"  // NOLINT(build/include)
#include "ignition/rendering/RenderEngine.hh"
#include "ignition/rendering/RenderingIface.hh"
#include "ignition/rendering/Heightmap.hh"
#include "ignition/rendering/Scene.hh"

using namespace ignition;
using namespace rendering;

class HeightmapTest : public testing::Test,
                 public testing::WithParamInterface<const char *>
{
  // Documentation inherited
  protected: void SetUp() override
  {
    common::Console::SetVerbosity(4);
  }

  /// \brief Path to test media files.
  public: const std::string TEST_MEDIA_PATH{
        common::joinPaths(std::string(PROJECT_SOURCE_PATH),
        "test", "media")};
};

/////////////////////////////////////////////////
// ogre1 not supported on Windows
TEST_P(HeightmapTest, IGN_UTILS_TEST_ENABLED_ONLY_ON_LINUX(Heightmap))
{
  std::string renderEngine{this->GetParam()};

  auto engine = rendering::engine(renderEngine);
  if (!engine)
  {
    FAIL() << "Engine '" << renderEngine
           << "' is not supported" << std::endl;
    return;
  }

  auto scene = engine->CreateScene("scene");
  ASSERT_NE(nullptr, scene);

  // Heightmap data
  auto heightImage = common::joinPaths(TEST_MEDIA_PATH, "heightmap_bowl.png");
  math::Vector3d size{17, 17, 10};
  math::Vector3d position{1, 2, 3};
  auto textureImage = common::joinPaths(TEST_MEDIA_PATH, "materials",
      "textures", "texture.png");
  auto normalImage = common::joinPaths(TEST_MEDIA_PATH, "materials",
      "textures", "flat_normal.png");

  auto data = std::make_shared<common::ImageHeightmap>();
  data->Load(heightImage);

  EXPECT_EQ(heightImage, data->Filename());

  HeightmapDescriptor desc;
  desc.SetData(data);
  desc.SetSize(size);
  desc.SetPosition(position);
  desc.SetUseTerrainPaging(true);
  desc.SetSampling(4u);

  HeightmapTexture textureA;
  textureA.SetSize(0.5);
  textureA.SetDiffuse(textureImage);
  textureA.SetNormal(normalImage);
  desc.AddTexture(textureA);

  HeightmapBlend blendA;
  blendA.SetMinHeight(2.0);
  blendA.SetFadeDistance(5.0);
  desc.AddBlend(blendA);

  HeightmapTexture textureB;
  textureB.SetSize(0.5);
  textureB.SetDiffuse(textureImage);
  textureB.SetNormal(normalImage);
  desc.AddTexture(textureB);

  HeightmapBlend blendB;
  blendB.SetMinHeight(4.0);
  blendB.SetFadeDistance(5.0);
  desc.AddBlend(blendB);

  HeightmapTexture textureC;
  textureC.SetSize(0.5);
  textureC.SetDiffuse(textureImage);
  textureC.SetNormal(normalImage);
  desc.AddTexture(textureC);

  auto heightmap = scene->CreateHeightmap(desc);
  ASSERT_NE(nullptr, heightmap);

  EXPECT_NE(nullptr, heightmap->Descriptor().Data());
  EXPECT_EQ(size, heightmap->Descriptor().Size());
  EXPECT_EQ(position, heightmap->Descriptor().Position());
  EXPECT_TRUE(heightmap->Descriptor().UseTerrainPaging());
  EXPECT_EQ(4u, heightmap->Descriptor().Sampling());

  EXPECT_EQ(3u, heightmap->Descriptor().TextureCount());
  for (auto i = 0u; i < heightmap->Descriptor().TextureCount(); ++i)
  {
    auto texture = heightmap->Descriptor().TextureByIndex(i);
    ASSERT_NE(nullptr, texture);
    EXPECT_EQ(textureImage, texture->Diffuse());
    EXPECT_EQ(normalImage, texture->Normal());
    EXPECT_DOUBLE_EQ(0.5, texture->Size());
  }

  ASSERT_EQ(2u, heightmap->Descriptor().BlendCount());

  auto blend = heightmap->Descriptor().BlendByIndex(0);
  ASSERT_NE(nullptr, blend);
  EXPECT_DOUBLE_EQ(2.0, blend->MinHeight());
  EXPECT_DOUBLE_EQ(5.0, blend->FadeDistance());

  blend = heightmap->Descriptor().BlendByIndex(1);
  ASSERT_NE(nullptr, blend);
  EXPECT_DOUBLE_EQ(4.0, blend->MinHeight());
  EXPECT_DOUBLE_EQ(5.0, blend->FadeDistance());

  // Add to a visual
  auto vis = scene->CreateVisual();
  EXPECT_EQ(0u, vis->GeometryCount());

  vis->AddGeometry(heightmap);
  EXPECT_EQ(1u, vis->GeometryCount());
  EXPECT_TRUE(vis->HasGeometry(heightmap));
  EXPECT_EQ(heightmap, vis->GeometryByIndex(0));

  scene->RootVisual()->AddChild(vis);

  // \todo(iche033) this should not be needed once Ogre2Heightmap::Destroy is
  // implemented.
  if (renderEngine == "ogre2")
  {
    vis->Destroy();
    heightmap.reset();
  }

  // Clean up
  engine->DestroyScene(scene);
  rendering::unloadEngine(engine->Name());
}

//////////////////////////////////////////////////
TEST_P(HeightmapTest, MoveConstructor)
{
  HeightmapDescriptor descriptor;
  descriptor.SetSize({0.1, 0.2, 0.3});
  descriptor.SetPosition({0.5, 0.6, 0.7});
  descriptor.SetUseTerrainPaging(true);
  descriptor.SetSampling(123u);

  HeightmapDescriptor descriptor2(std::move(descriptor));
  EXPECT_EQ(ignition::math::Vector3d(0.1, 0.2, 0.3), descriptor2.Size());
  EXPECT_EQ(ignition::math::Vector3d(0.5, 0.6, 0.7), descriptor2.Position());
  EXPECT_TRUE(descriptor2.UseTerrainPaging());
  EXPECT_EQ(123u, descriptor2.Sampling());

  HeightmapTexture texture;
  texture.SetSize(123.456);
  texture.SetDiffuse("diffuse");
  texture.SetNormal("normal");

  HeightmapTexture texture2(std::move(texture));
  EXPECT_DOUBLE_EQ(123.456, texture2.Size());
  EXPECT_EQ("diffuse", texture2.Diffuse());
  EXPECT_EQ("normal", texture2.Normal());

  HeightmapBlend blend;
  blend.SetMinHeight(123.456);
  blend.SetFadeDistance(456.123);

  HeightmapBlend blend2(std::move(blend));
  EXPECT_DOUBLE_EQ(123.456, blend2.MinHeight());
  EXPECT_DOUBLE_EQ(456.123, blend2.FadeDistance());
}

/////////////////////////////////////////////////
TEST_P(HeightmapTest, CopyConstructor)
{
  HeightmapDescriptor descriptor;
  descriptor.SetSize({0.1, 0.2, 0.3});
  descriptor.SetPosition({0.5, 0.6, 0.7});
  descriptor.SetUseTerrainPaging(true);
  descriptor.SetSampling(123u);

  HeightmapDescriptor descriptor2(descriptor);
  EXPECT_EQ(ignition::math::Vector3d(0.1, 0.2, 0.3), descriptor2.Size());
  EXPECT_EQ(ignition::math::Vector3d(0.5, 0.6, 0.7), descriptor2.Position());
  EXPECT_TRUE(descriptor2.UseTerrainPaging());
  EXPECT_EQ(123u, descriptor2.Sampling());

  HeightmapTexture texture;
  texture.SetSize(123.456);
  texture.SetDiffuse("diffuse");
  texture.SetNormal("normal");

  HeightmapTexture texture2(texture);
  EXPECT_DOUBLE_EQ(123.456, texture2.Size());
  EXPECT_EQ("diffuse", texture2.Diffuse());
  EXPECT_EQ("normal", texture2.Normal());

  HeightmapBlend blend;
  blend.SetMinHeight(123.456);
  blend.SetFadeDistance(456.123);

  HeightmapBlend blend2(blend);
  EXPECT_DOUBLE_EQ(123.456, blend2.MinHeight());
  EXPECT_DOUBLE_EQ(456.123, blend2.FadeDistance());
}

/////////////////////////////////////////////////
TEST_P(HeightmapTest, CopyAssignmentOperator)
{
  HeightmapDescriptor descriptor;
  descriptor.SetSize({0.1, 0.2, 0.3});
  descriptor.SetPosition({0.5, 0.6, 0.7});
  descriptor.SetUseTerrainPaging(true);
  descriptor.SetSampling(123u);

  HeightmapDescriptor descriptor2;
  descriptor2 = descriptor;
  EXPECT_EQ(ignition::math::Vector3d(0.1, 0.2, 0.3), descriptor2.Size());
  EXPECT_EQ(ignition::math::Vector3d(0.5, 0.6, 0.7), descriptor2.Position());
  EXPECT_TRUE(descriptor2.UseTerrainPaging());
  EXPECT_EQ(123u, descriptor2.Sampling());

  HeightmapTexture texture;
  texture.SetSize(123.456);
  texture.SetDiffuse("diffuse");
  texture.SetNormal("normal");

  HeightmapTexture texture2;
  texture2 = texture;
  EXPECT_DOUBLE_EQ(123.456, texture2.Size());
  EXPECT_EQ("diffuse", texture2.Diffuse());
  EXPECT_EQ("normal", texture2.Normal());

  HeightmapBlend blend;
  blend.SetMinHeight(123.456);
  blend.SetFadeDistance(456.123);

  HeightmapBlend blend2;
  blend2 = blend;
  EXPECT_DOUBLE_EQ(123.456, blend2.MinHeight());
  EXPECT_DOUBLE_EQ(456.123, blend2.FadeDistance());
}

/////////////////////////////////////////////////
TEST_P(HeightmapTest, MoveAssignmentOperator)
{
  HeightmapDescriptor descriptor;
  descriptor.SetSize({0.1, 0.2, 0.3});
  descriptor.SetPosition({0.5, 0.6, 0.7});
  descriptor.SetUseTerrainPaging(true);
  descriptor.SetSampling(123u);

  HeightmapDescriptor descriptor2;
  descriptor2 = std::move(descriptor);
  EXPECT_EQ(ignition::math::Vector3d(0.1, 0.2, 0.3), descriptor2.Size());
  EXPECT_EQ(ignition::math::Vector3d(0.5, 0.6, 0.7), descriptor2.Position());
  EXPECT_TRUE(descriptor2.UseTerrainPaging());
  EXPECT_EQ(123u, descriptor2.Sampling());

  HeightmapTexture texture;
  texture.SetSize(123.456);
  texture.SetDiffuse("diffuse");
  texture.SetNormal("normal");

  HeightmapTexture texture2;
  texture2 = std::move(texture);
  EXPECT_DOUBLE_EQ(123.456, texture2.Size());
  EXPECT_EQ("diffuse", texture2.Diffuse());
  EXPECT_EQ("normal", texture2.Normal());

  HeightmapBlend blend;
  blend.SetMinHeight(123.456);
  blend.SetFadeDistance(456.123);

  HeightmapBlend blend2;
  blend2 = std::move(blend);
  EXPECT_DOUBLE_EQ(123.456, blend2.MinHeight());
  EXPECT_DOUBLE_EQ(456.123, blend2.FadeDistance());
}

/////////////////////////////////////////////////
TEST_P(HeightmapTest, CopyAssignmentAfterMove)
{
  HeightmapDescriptor descriptor1;
  descriptor1.SetSampling(123u);

  HeightmapDescriptor descriptor2;
  descriptor2.SetSampling(456u);

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  HeightmapDescriptor tmp = std::move(descriptor1);
  descriptor1 = descriptor2;
  descriptor2 = tmp;

  EXPECT_EQ(456u, descriptor1.Sampling());
  EXPECT_EQ(123u, descriptor2.Sampling());

  HeightmapTexture texture1;
  texture1.SetSize(123.456);

  HeightmapTexture texture2;
  texture2.SetSize(456.123);

  HeightmapTexture tmpTexture = std::move(texture1);
  texture1 = texture2;
  texture2 = tmpTexture;

  EXPECT_DOUBLE_EQ(456.123, texture1.Size());
  EXPECT_DOUBLE_EQ(123.456, texture2.Size());

  HeightmapBlend blend1;
  blend1.SetMinHeight(123.456);

  HeightmapBlend blend2;
  blend2.SetMinHeight(456.123);

  HeightmapBlend tmpBlend = std::move(blend1);
  blend1 = blend2;
  blend2 = tmpBlend;

  EXPECT_DOUBLE_EQ(456.123, blend1.MinHeight());
  EXPECT_DOUBLE_EQ(123.456, blend2.MinHeight());
}

INSTANTIATE_TEST_CASE_P(Heightmap, HeightmapTest,
    RENDER_ENGINE_VALUES,
    ignition::rendering::PrintToStringParam());

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
