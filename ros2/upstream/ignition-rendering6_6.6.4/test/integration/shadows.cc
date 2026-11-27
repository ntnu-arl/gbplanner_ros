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
#include <gz/common/Image.hh>

#include "test_config.h"  // NOLINT(build/include)

#include "gz/rendering/Camera.hh"
#include "gz/rendering/Image.hh"
#include "gz/rendering/PixelFormat.hh"
#include "gz/rendering/RenderEngine.hh"
#include "gz/rendering/RenderingIface.hh"
#include "gz/rendering/Scene.hh"

using namespace gz;
using namespace rendering;

class ShadowsTest: public testing::Test,
                   public testing::WithParamInterface<const char *>
{
  // Test and verify shadows are generated
  public: void Shadows(const std::string &_renderEngine);
};

/////////////////////////////////////////////////
void ShadowsTest::Shadows(const std::string &_renderEngine)
{
  // override and make sure not to look for resources in installed share dir
  std::string projectSrcPath = PROJECT_SOURCE_PATH;
  std::string env = "IGN_RENDERING_RESOURCE_PATH=" + projectSrcPath;
  putenv(const_cast<char *>(env.c_str()));

  // create and populate scene
  RenderEngine *engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    igndbg << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }

  // add resources in build dir
  engine->AddResourcePath(
      common::joinPaths(std::string(PROJECT_BUILD_PATH), "src"));

  ScenePtr scene = engine->CreateScene("scene");
  ASSERT_TRUE(scene != nullptr);
  scene->SetAmbientLight(0.3, 0.3, 0.3);

  VisualPtr root = scene->RootVisual();

  CameraPtr camera = scene->CreateCamera();
  ASSERT_TRUE(camera != nullptr);

  // downward looking camera
  camera->SetImageWidth(10);
  camera->SetImageHeight(10);
  camera->SetLocalRotation(0, 1.57, 0);
  root->AddChild(camera);


  // create downward directional light
  DirectionalLightPtr light = scene->CreateDirectionalLight();
  light->SetDirection(0.0, 0.0, -1);
  light->SetDiffuseColor(0.5, 0.5, 0.5);
  light->SetSpecularColor(0.5, 0.5, 0.5);
  root->AddChild(light);

  MaterialPtr white = scene->CreateMaterial();
  white->SetAmbient(1.0, 1.0, 1.0);
  white->SetDiffuse(1.0, 1.0, 1.0);
  white->SetSpecular(1.0, 1.0, 1.0);
  white->SetCastShadows(true);

  // create box that casts shadows
  VisualPtr boxTop = scene->CreateVisual();
  boxTop->AddGeometry(scene->CreateBox());
  boxTop->SetLocalPosition(0.0, 0.5, 0.55);
  boxTop->SetMaterial(white, false);
  root->AddChild(boxTop);

  // create green material
  MaterialPtr green = scene->CreateMaterial();
  green->SetAmbient(0.0, 0.5, 0.0);
  green->SetDiffuse(0.0, 0.7, 0.0);
  green->SetSpecular(0.5, 0.5, 0.5);
  green->SetShininess(50);
  green->SetReflectivity(0);

  // create box at origin that receives shadows
  VisualPtr boxBottom = scene->CreateVisual();
  boxBottom->AddGeometry(scene->CreateBox());
  boxBottom->SetLocalPosition(0.0, 0.0, -1.0);
  boxBottom->SetMaterial(green);
  root->AddChild(boxBottom);

  Image image = camera->CreateImage();
  unsigned int height = camera->ImageHeight();
  unsigned int width = camera->ImageWidth();
  unsigned int bpp = PixelUtil::BytesPerPixel(camera->ImageFormat());
  unsigned int step = width * bpp;

  // verify shadows are generated for a number of images
  for (unsigned int k = 0; k < 10; ++k)
  {
    // sum of shaded / unshaded pixel values
    unsigned shaded = 0;
    unsigned unshaded = 0;

    camera->Capture(image);
    unsigned char *data = image.Data<unsigned char>();
    for (unsigned int i = 0; i < height; ++i)
    {
      for (unsigned int j = 0; j < step; j+=bpp)
      {
        unsigned int idx = i * step + j;
        unsigned int r = data[idx];
        unsigned int g = data[idx+1];
        unsigned int b = data[idx+2];

        // color should be a shade of green
        EXPECT_GT(g, r);
        EXPECT_GT(g, b);

        if (j < step /2)
          shaded += r + g + b;
        else
          unshaded += r + g + b;
      }
    }
    // Test currently fails on macOS
#ifndef __APPLE__
    // left side of image should be significanly darker than the right side
    EXPECT_LT(shaded, unshaded);
#endif
  }

  // disable shadows and verify shades
  white->SetCastShadows(false);
  boxTop->SetMaterial(white);


  // verify no shadows are generated for a number of images
  for (unsigned int k = 0; k < 10; ++k)
  {
    // sum of left / right pixel values
    unsigned left = 0;
    unsigned right = 0;

    camera->Capture(image);
    unsigned char *data = image.Data<unsigned char>();
    for (unsigned int i = 0; i < height; ++i)
    {
      for (unsigned int j = 0; j < step; j+=bpp)
      {
        unsigned int idx = i * step + j;
        unsigned int r = data[idx];
        unsigned int g = data[idx+1];
        unsigned int b = data[idx+2];

        // color should be a shade of green
        EXPECT_GT(g, r);
        EXPECT_GT(g, b);

        if (j < step /2)
          left += r + g + b;
        else
          right += r + g + b;
      }
    }
    // Test currently fails on macOS
#ifndef __APPLE__
    // left side of image should be similar to right side
    EXPECT_NEAR(left, right, 5);
#endif
  }

  // Clean up materials
  scene->DestroyMaterial(white);
  scene->DestroyMaterial(green);

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
TEST_P(ShadowsTest, Shadows)
{
  Shadows(GetParam());
}

INSTANTIATE_TEST_CASE_P(Shadows, ShadowsTest,
    RENDER_ENGINE_VALUES,
    PrintToStringParam());

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
