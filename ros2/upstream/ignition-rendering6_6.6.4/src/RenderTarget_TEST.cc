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

#include "test_config.h"  // NOLINT(build/include)

#include "gz/rendering/Camera.hh"
#include "gz/rendering/GaussianNoisePass.hh"
#include "gz/rendering/RenderEngine.hh"
#include "gz/rendering/RenderingIface.hh"
#include "gz/rendering/RenderPassSystem.hh"
#include "gz/rendering/RenderTarget.hh"
#include "gz/rendering/Scene.hh"

using namespace gz;
using namespace rendering;

class RenderTargetTest : public testing::Test,
                         public testing::WithParamInterface<const char*>
{
  /// \brief test RenderTexture properties
  public: void RenderTexture(const std::string &_renderEngine);

  /// \brief test RenderWindow properties
  public: void RenderWindow(const std::string &_renderEngine);

  /// \brief test adding and removing render passes
  public: void AddRemoveRenderPass(const std::string &_renderEngine);
};

/////////////////////////////////////////////////
void RenderTargetTest::RenderTexture(const std::string &_renderEngine)
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

  RenderTexturePtr renderTexture = scene->CreateRenderTexture();

  // default properties
  EXPECT_EQ(scene->BackgroundColor(), renderTexture->BackgroundColor());
  EXPECT_EQ(0u, renderTexture->GLId());

  // test basic properties
  renderTexture->SetFormat(PF_R8G8B8);
  renderTexture->SetWidth(800u);
  renderTexture->SetHeight(600u);

  EXPECT_EQ(PF_R8G8B8, renderTexture->Format());
  EXPECT_EQ(800u, renderTexture->Width());
  EXPECT_EQ(600u, renderTexture->Height());

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
void RenderTargetTest::RenderWindow(const std::string &_renderEngine)
{
#ifdef __APPLE__                                                                                                                                                                              
  std::cerr << "Skipping test for apple, see issue #847." << std::endl;
  return;
#endif
  if (_renderEngine != "ogre")
  {
    igndbg << "RenderWindow not supported yet in rendering engine: "
            << _renderEngine << std::endl;
    return;
  }

  // create and populate scene
  RenderEngine *engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    FAIL() << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }
  ScenePtr scene = engine->CreateScene("scene");

  CameraPtr camera =  scene->CreateCamera("camera");

  RenderWindowPtr renderWindow = camera->CreateRenderWindow();

  renderWindow->SetWidth(320);
  renderWindow->SetHeight(240);
  renderWindow->SetHandle("windowId");
  renderWindow->SetDevicePixelRatio(1.0);

  EXPECT_EQ(320u, renderWindow->Width());
  EXPECT_EQ(240u, renderWindow->Height());
  EXPECT_EQ("windowId", renderWindow->Handle());
  EXPECT_DOUBLE_EQ(1.0, renderWindow->DevicePixelRatio());
  EXPECT_EQ(math::Color::Black, renderWindow->BackgroundColor());

  // change render window properties and verify
  renderWindow->SetWidth(640);
  renderWindow->SetHeight(480);
  scene->SetBackgroundColor(math::Color::Red);

  EXPECT_EQ(640u, renderWindow->Width());
  EXPECT_EQ(480u, renderWindow->Height());
  EXPECT_EQ(math::Color::Red, renderWindow->BackgroundColor());

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
void RenderTargetTest::AddRemoveRenderPass(const std::string &_renderEngine)
{
#ifdef __APPLE__                                                                                                                                                                              
  std::cerr << "Skipping test for apple, see issue #847." << std::endl;
  return;
#endif
  if (_renderEngine != "ogre")
  {
    igndbg << "RenderWindow not supported yet in rendering engine: "
            << _renderEngine << std::endl;
    return;
  }

  // create and populate scene
  RenderEngine *engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    FAIL() << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }
  ScenePtr scene = engine->CreateScene("scene");

  // create a render texture and verify no render pass exists
  RenderTexturePtr renderTexture = scene->CreateRenderTexture();
  EXPECT_EQ(0u, renderTexture->RenderPassCount());

  // get the render pass system
  RenderPassSystemPtr rpSystem = engine->RenderPassSystem();
  if (!rpSystem)
  {
    ignwarn << "Render engin '" << _renderEngine << "' does not support "
            << "render pass system" << std::endl;
    return;
  }
  RenderPassPtr pass1 = rpSystem->Create<GaussianNoisePass>();
  EXPECT_NE(nullptr, pass1);

  // test adding a render pass
  renderTexture->AddRenderPass(pass1);
  EXPECT_EQ(1u, renderTexture->RenderPassCount());
  EXPECT_EQ(pass1, renderTexture->RenderPassByIndex(0u));

  // test adding another render pass
  RenderPassPtr pass2 = rpSystem->Create<GaussianNoisePass>();
  EXPECT_NE(nullptr, pass2);
  renderTexture->AddRenderPass(pass2);
  EXPECT_EQ(2u, renderTexture->RenderPassCount());
  EXPECT_EQ(pass1, renderTexture->RenderPassByIndex(0u));
  EXPECT_EQ(pass2, renderTexture->RenderPassByIndex(1u));

  // test removing render pass
  renderTexture->RemoveRenderPass(pass1);
  EXPECT_EQ(1u, renderTexture->RenderPassCount());
  EXPECT_EQ(pass2, renderTexture->RenderPassByIndex(0u));

  // Clean up
  engine->DestroyScene(scene);
  rendering::unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
TEST_P(RenderTargetTest, RenderTexture)
{
  RenderTexture(GetParam());
}

/////////////////////////////////////////////////
TEST_P(RenderTargetTest, RenderWindow)
{
  RenderWindow(GetParam());
}

/////////////////////////////////////////////////
TEST_P(RenderTargetTest, AddRemoveRenderPass)
{
  AddRemoveRenderPass(GetParam());
}

INSTANTIATE_TEST_CASE_P(RenderTarget, RenderTargetTest,
    RENDER_ENGINE_VALUES,
    PrintToStringParam());

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
