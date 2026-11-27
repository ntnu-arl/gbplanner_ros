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

#include "gz/rendering/Camera.hh"
#include "gz/rendering/RenderEngine.hh"
#include "gz/rendering/RenderingIface.hh"
#include "gz/rendering/Scene.hh"

using namespace gz;
using namespace rendering;

class SceneTest: public testing::Test,
                 public testing::WithParamInterface<const char *>
{
  // Documentation inherited
  public: void SetUp() override
  {
    ignition::common::Console::SetVerbosity(4);
  }

  // Test adding and removing visuals
  // Simulates 'levels' where visuals are added and removed throughout
  // frame updates
  public: void AddRemoveVisuals(const std::string &_renderEngine);

  // Test and verify camera tracking
  public: void VisualAt(const std::string &_renderEngine);
};

/////////////////////////////////////////////////
void SceneTest::AddRemoveVisuals(const std::string &_renderEngine)
{
  // create and populate scene
  RenderEngine *engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    igndbg << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }

  ScenePtr scene = engine->CreateScene("scene");
  ASSERT_TRUE(scene != nullptr);

  VisualPtr root = scene->RootVisual();

  // create camera
  CameraPtr camera = scene->CreateCamera("camera");
  ASSERT_TRUE(camera != nullptr);
  camera->SetLocalPosition(0.0, 0.0, 0.0);
  camera->SetLocalRotation(0.0, 0.0, 0.0);
  camera->SetImageWidth(800);
  camera->SetImageHeight(600);
  camera->SetAntiAliasing(2);
  camera->SetAspectRatio(1.333);
  camera->SetHFOV(IGN_PI / 2);
  root->AddChild(camera);

  // create material assigned to all geoms
  math::Color ambient(0.5f, 0.2f, 0.4f, 1.0f);
  math::Color diffuse(0.1f, 0.9f, 0.3f, 1.0f);
  math::Color specular(0.8f, 0.7f, 0.0f, 1.0f);
  double transparency = 0.3;
  MaterialPtr material = scene->CreateMaterial("mat");
  ASSERT_NE(nullptr, material);
  EXPECT_TRUE(scene->MaterialRegistered("mat"));
  material->SetAmbient(ambient);
  material->SetDiffuse(diffuse);
  material->SetSpecular(specular);
  material->SetTransparency(transparency);

  std::vector<VisualPtr> visuals;
  for (unsigned int i = 0u; i < 10u; ++i)
  {
    // create 10 visuals and add to front of vector
    for (unsigned int j = 0u; j < 10u; ++j)
    {
      // create box
      std::string name = "box" + std::to_string(i) + std::to_string(j);
      VisualPtr box = scene->CreateVisual(name);
      ASSERT_TRUE(box != nullptr);
      EXPECT_TRUE(scene->HasVisualName(name));
      GeometryPtr boxGeom = scene->CreateBox();
      boxGeom->SetMaterial(material);
      box->AddGeometry(boxGeom);
      box->SetLocalPosition(j*i + j, 0, 0);
      root->AddChild(box);
      visuals.insert(visuals.begin(), box);

      // verify box material properties
      MaterialPtr boxMat = boxGeom->Material();
      std::string boxMatName = boxMat->Name();
      EXPECT_TRUE(scene->MaterialRegistered(boxMatName));
      EXPECT_NE(material->Name(), boxMatName);
      EXPECT_EQ(material->Type(), boxMat->Type());
      EXPECT_EQ(ambient, boxMat->Ambient());
      EXPECT_EQ(diffuse, boxMat->Diffuse());
      EXPECT_EQ(specular, boxMat->Specular());
      EXPECT_DOUBLE_EQ(transparency, boxMat->Transparency());
    }

    // render a frame
    camera->Update();

    // remove second half of visuals in vector
    int size = visuals.size();
    while (visuals.size() > static_cast<unsigned int>(size/2.0))
    {
      // remove box visual and verify
      VisualPtr box = visuals.back();
      visuals.pop_back();
      std::string name = box->Name();
      std::string boxMatName = box->GeometryByIndex(0u)->Name();
      scene->DestroyVisual(box);
      EXPECT_FALSE(scene->HasVisualName(name));
      EXPECT_FALSE(scene->MaterialRegistered(boxMatName));
    }

    // render a frame
    camera->Update();
  }

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
void SceneTest::VisualAt(const std::string &_renderEngine)
{
  if (_renderEngine == "optix")
  {
    igndbg << "RayQuery not supported yet in rendering engine: "
            << _renderEngine << std::endl;
    return;
  }

  // create and populate scene
  RenderEngine *engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    igndbg << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }

  ScenePtr scene = engine->CreateScene("scene");
  ASSERT_TRUE(scene != nullptr);

  VisualPtr root = scene->RootVisual();

  // create box visual
  VisualPtr box = scene->CreateVisual("box");
  ASSERT_TRUE(box != nullptr);
  box->AddGeometry(scene->CreateBox());
  box->SetOrigin(0.0, 0.5, 0.0);
  box->SetLocalPosition(3, 0, 0);
  box->SetLocalRotation(IGN_PI / 4, 0, IGN_PI / 3);
  box->SetLocalScale(1, 2.5, 1);
  root->AddChild(box);

  // create sphere visual
  VisualPtr sphere = scene->CreateVisual("sphere");
  ASSERT_TRUE(sphere != nullptr);
  sphere->AddGeometry(scene->CreateSphere());
  sphere->SetOrigin(0.0, -0.5, 0.0);
  sphere->SetLocalPosition(3, 0, 0);
  sphere->SetLocalRotation(0, 0, 0);
  sphere->SetLocalScale(1, 2.5, 1);
  root->AddChild(sphere);

  // create camera
  CameraPtr camera = scene->CreateCamera("camera");
  ASSERT_TRUE(camera != nullptr);
  camera->SetLocalPosition(0.0, 0.0, 0.0);
  camera->SetLocalRotation(0.0, 0.0, 0.0);
  camera->SetImageWidth(800);
  camera->SetImageHeight(600);
  camera->SetAntiAliasing(2);
  camera->SetAspectRatio(1.333);
  camera->SetHFOV(IGN_PI / 2);
  root->AddChild(camera);

  // render a frame
  camera->Update();

  // test get sphere object
  math::Vector2i spherePosition(220, 307);
  VisualPtr sphere_visual = scene->VisualAt(camera, spherePosition);
  ASSERT_TRUE(sphere_visual != nullptr);
  EXPECT_EQ("sphere", sphere_visual->Name());

  // test get box object
  math::Vector2i boxPosition(452, 338);
  VisualPtr box_visual = scene->VisualAt(camera, boxPosition);
  ASSERT_TRUE(box_visual != nullptr);
  EXPECT_EQ("box", box_visual->Name());

  // test get no object
  math::Vector2i emptyPosition(300, 150);
  VisualPtr empty_visual = scene->VisualAt(camera, emptyPosition);
  ASSERT_TRUE(empty_visual == nullptr);

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
TEST_P(SceneTest, AddRemoveVisuals)
{
  AddRemoveVisuals(GetParam());
}

/////////////////////////////////////////////////
TEST_P(SceneTest, VisualAt)
{
  VisualAt(GetParam());
}

// It doesn't suppot optix just yet
INSTANTIATE_TEST_CASE_P(Scene, SceneTest,
    RENDER_ENGINE_VALUES,
    PrintToStringParam());

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
