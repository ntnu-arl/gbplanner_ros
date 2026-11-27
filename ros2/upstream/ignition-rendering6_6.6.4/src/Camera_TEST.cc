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
#include "gz/rendering/Scene.hh"
#include "gz/rendering/Utils.hh"

using namespace gz;
using namespace rendering;

class CameraTest : public testing::Test,
                   public testing::WithParamInterface<const char*>
{
  /// \brief Test camera view and projection matrix
  public: void ViewProjectionMatrix(const std::string &_renderEngine);

  /// \brief Test updating camera render texture properties
  public: void RenderTexture(const std::string &_renderEngine);

  /// \brief Test camera tracking and camera following.
  public: void TrackFollow(const std::string &_renderEngine);

  /// \brief Test adding and removing render passes
  public: void AddRemoveRenderPass(const std::string &_renderEngine);

  /// \brief Test setting visibility mask
  public: void VisibilityMask(const std::string &_renderEngine);

  /// \brief Test setting intrinsic matrix
  public: void IntrinsicMatrix(const std::string &_renderEngine);
};

/////////////////////////////////////////////////
void CameraTest::ViewProjectionMatrix(const std::string &_renderEngine)
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

  CameraPtr camera(scene->CreateCamera());
  EXPECT_TRUE(camera != nullptr);

  // projection parameters
  math::Matrix4d projMatrix = camera->ProjectionMatrix();

  EXPECT_GT(camera->HFOV(), 0);
  math::Angle hfov(1.57);
  camera->SetHFOV(hfov);
  EXPECT_NEAR(hfov.Radian(), camera->HFOV().Radian(), 1e-6);

  EXPECT_GT(camera->AspectRatio(), 0);
  camera->SetAspectRatio(1.7777);
  EXPECT_NEAR(1.7777, camera->AspectRatio(), 1e-6);

  camera->SetAntiAliasing(1u);
  EXPECT_EQ(1u, camera->AntiAliasing());

  EXPECT_GT(camera->NearClipPlane(), 0);
  camera->SetNearClipPlane(0.1);
  EXPECT_DOUBLE_EQ(0.1, camera->NearClipPlane());

  EXPECT_GT(camera->FarClipPlane(), 0);
  camera->SetFarClipPlane(800);
  EXPECT_DOUBLE_EQ(800, camera->FarClipPlane());

  EXPECT_NE(projMatrix, camera->ProjectionMatrix());

  // view matrix
  math::Matrix4d viewMatrix = camera->ViewMatrix();
  EXPECT_EQ(math::Vector3d::Zero, camera->LocalPosition());
  EXPECT_EQ(math::Quaterniond::Identity, camera->LocalRotation());

  math::Vector3d pos(3.0, -2.0, 5.0);
  math::Quaterniond rot(0.0, 1.5, 3.14);
  camera->SetLocalPosition(pos);
  EXPECT_EQ(pos, camera->LocalPosition());
  camera->SetLocalRotation(rot);
  EXPECT_EQ(rot, camera->LocalRotation());

  EXPECT_NE(viewMatrix, camera->ViewMatrix());

  // projection type
  math::Matrix4d initialProjectionMatrix = camera->ProjectionMatrix();
  math::Matrix4d initialViewMatrix = camera->ViewMatrix();
  EXPECT_EQ(CameraProjectionType::CPT_PERSPECTIVE, camera->ProjectionType());
  camera->SetProjectionType(CameraProjectionType::CPT_ORTHOGRAPHIC);
  EXPECT_EQ(CameraProjectionType::CPT_ORTHOGRAPHIC, camera->ProjectionType());
  EXPECT_NE(initialProjectionMatrix, camera->ProjectionMatrix());
  EXPECT_EQ(initialViewMatrix, camera->ViewMatrix());

  camera->SetProjectionType(CameraProjectionType::CPT_PERSPECTIVE);
  EXPECT_EQ(CameraProjectionType::CPT_PERSPECTIVE, camera->ProjectionType());
  EXPECT_EQ(initialProjectionMatrix, camera->ProjectionMatrix());
  EXPECT_EQ(initialViewMatrix, camera->ViewMatrix());

  // project 3d to 2d
  unsigned int width = 320u;
  unsigned int height = 240u;
  camera->SetImageWidth(width);
  camera->SetImageHeight(height);
  camera->SetLocalPosition(math::Vector3d::Zero);
  camera->SetLocalRotation(math::Quaterniond::Identity);

  math::Vector2i pos2d = camera->Project(math::Vector3d(2.0, 0, 0));
  EXPECT_EQ(static_cast<int>(width * 0.5), pos2d.X());
  EXPECT_EQ(static_cast<int>(height * 0.5), pos2d.Y());

  pos2d = camera->Project(math::Vector3d(3.0, 0, 0));
  EXPECT_EQ(static_cast<int>(width * 0.5), pos2d.X());
  EXPECT_EQ(static_cast<int>(height * 0.5), pos2d.Y());

  pos2d = camera->Project(math::Vector3d(2.0, 1, 0));
  EXPECT_GT(static_cast<int>(width * 0.5), pos2d.X());
  EXPECT_EQ(static_cast<int>(height * 0.5), pos2d.Y());

  pos2d = camera->Project(math::Vector3d(2.0, 1, 1));
  EXPECT_GT(static_cast<int>(width * 0.5), pos2d.X());
  EXPECT_GT(static_cast<int>(height * 0.5), pos2d.Y());

  pos2d = camera->Project(math::Vector3d(2.0, -1, -1));
  EXPECT_LT(static_cast<int>(width * 0.5), pos2d.X());
  EXPECT_LT(static_cast<int>(height * 0.5), pos2d.Y());

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
void CameraTest::RenderTexture(const std::string &_renderEngine)
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

  CameraPtr camera = scene->CreateCamera();
  EXPECT_TRUE(camera != nullptr);

  // render texture parameters
  EXPECT_GT(camera->ImageWidth(), 0u);
  EXPECT_GT(camera->ImageHeight(), 0u);

  unsigned int height = 80;
  camera->SetImageHeight(height);
  EXPECT_EQ(height, camera->ImageHeight());
  double aspectRatio =
    static_cast<double>(camera->ImageWidth()) / static_cast<double>(height);
  EXPECT_NEAR(aspectRatio, camera->AspectRatio(), 1e-6);

  unsigned int width = 100;
  camera->SetImageWidth(width);
  EXPECT_EQ(width, camera->ImageWidth());
  aspectRatio =
    static_cast<double>(width) / static_cast<double>(camera->ImageHeight());
  EXPECT_NEAR(aspectRatio, camera->AspectRatio(), 1e-6);

  EXPECT_NE(PixelFormat::PF_UNKNOWN, camera->ImageFormat());
  camera->SetImageFormat(PixelFormat::PF_B8G8R8);
  EXPECT_EQ(PixelFormat::PF_B8G8R8, camera->ImageFormat());
  EXPECT_EQ(width*height*3u, camera->ImageMemorySize());


  // verify render texture GL Id
  EXPECT_EQ(0u, camera->RenderTextureGLId());
#ifdef HAVE_OPENGL
  // PreRender - creates the render texture
  camera->PreRender();
  EXPECT_NE(0u, camera->RenderTextureGLId());
#endif

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
void CameraTest::TrackFollow(const std::string &_renderEngine)
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

  CameraPtr camera = scene->CreateCamera();
  EXPECT_TRUE(camera != nullptr);

  VisualPtr visual = scene->CreateVisual();

  // track node
  EXPECT_EQ(nullptr, camera->TrackTarget());
  EXPECT_EQ(math::Vector3d::Zero, camera->TrackOffset());

  camera->SetTrackTarget(nullptr);
  EXPECT_EQ(nullptr, camera->TrackTarget());
  EXPECT_EQ(math::Vector3d::Zero, camera->TrackOffset());

  camera->SetTrackTarget(visual);
  EXPECT_EQ(visual, camera->TrackTarget());
  EXPECT_EQ(math::Vector3d::Zero, camera->TrackOffset());

  math::Vector3d trackOffset(1.3, 30.4, -1.3);
  camera->SetTrackTarget(visual, trackOffset, false);
  EXPECT_EQ(visual, camera->TrackTarget());
  EXPECT_EQ(trackOffset, camera->TrackOffset());

  math::Vector3d newTrackOffset(-1.2, 9.4, 1.7);
  camera->SetTrackOffset(newTrackOffset);
  EXPECT_EQ(newTrackOffset, camera->TrackOffset());

  camera->SetTrackPGain(0.234);
  EXPECT_DOUBLE_EQ(0.234, camera->TrackPGain());

  // follow node
  EXPECT_EQ(nullptr, camera->FollowTarget());
  EXPECT_EQ(math::Vector3d::Zero, camera->FollowOffset());

  camera->SetFollowTarget(nullptr);
  EXPECT_EQ(nullptr, camera->FollowTarget());
  EXPECT_EQ(math::Vector3d::Zero, camera->FollowOffset());

  camera->SetFollowTarget(visual);
  EXPECT_EQ(visual, camera->FollowTarget());
  EXPECT_EQ(math::Vector3d::Zero, camera->FollowOffset());

  math::Vector3d followOffset(7.2, -3.8, 9.3);
  camera->SetFollowTarget(visual, followOffset, true);
  EXPECT_EQ(visual, camera->FollowTarget());
  EXPECT_EQ(followOffset, camera->FollowOffset());

  math::Vector3d newFollowOffset(-0.2, 0.4, 0.7);
  camera->SetFollowOffset(newFollowOffset);
  EXPECT_EQ(newFollowOffset, camera->FollowOffset());

  camera->SetFollowPGain(0.4);
  EXPECT_DOUBLE_EQ(0.4, camera->FollowPGain());

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
void CameraTest::AddRemoveRenderPass(const std::string &_renderEngine)
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

  CameraPtr camera = scene->CreateCamera();
  EXPECT_TRUE(camera != nullptr);

  // verify no render pass exists for camera
  EXPECT_EQ(0u, camera->RenderPassCount());

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
  camera->AddRenderPass(pass1);
  EXPECT_EQ(1u, camera->RenderPassCount());
  EXPECT_EQ(pass1, camera->RenderPassByIndex(0u));

  // test adding another render pass
  RenderPassPtr pass2 = rpSystem->Create<GaussianNoisePass>();
  EXPECT_NE(nullptr, pass2);
  camera->AddRenderPass(pass2);
  EXPECT_EQ(2u, camera->RenderPassCount());
  EXPECT_EQ(pass1, camera->RenderPassByIndex(0u));
  EXPECT_EQ(pass2, camera->RenderPassByIndex(1u));

  // test removing render pass
  camera->RemoveRenderPass(pass1);
  EXPECT_EQ(1u, camera->RenderPassCount());
  EXPECT_EQ(pass2, camera->RenderPassByIndex(0u));

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
void CameraTest::VisibilityMask(const std::string &_renderEngine)
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

  CameraPtr camera = scene->CreateCamera();
  EXPECT_TRUE(camera != nullptr);

  // chek initial value
  EXPECT_EQ(static_cast<uint32_t>(IGN_VISIBILITY_ALL),
      camera->VisibilityMask());

  // check setting new values
  camera->SetVisibilityMask(0x00000010u);
  EXPECT_EQ(0x00000010u, camera->VisibilityMask());

  camera->SetVisibilityMask(0u);
  EXPECT_EQ(0u, camera->VisibilityMask());

  // Clean up
  engine->DestroyScene(scene);
  rendering::unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
void CameraTest::IntrinsicMatrix(const std::string &_renderEngine)
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

  CameraPtr camera = scene->CreateCamera();
  EXPECT_TRUE(camera != nullptr);

  unsigned int width = 320;
  unsigned int height = 240;
  double hfov = 1.047;

  camera->SetImageHeight(height);
  camera->SetImageWidth(width);
  camera->SetHFOV(hfov);

  double error = 1e-1;
  EXPECT_EQ(camera->ImageHeight(), height);
  EXPECT_EQ(camera->ImageWidth(), width);
  EXPECT_NEAR(camera->HFOV().Radian(), hfov, error);

  // Verify focal length and optical center from intrinsics
  auto cameraIntrinsics = projectionToCameraIntrinsic(
      camera->ProjectionMatrix(),
      camera->ImageWidth(),
      camera->ImageHeight()
    );
  EXPECT_NEAR(cameraIntrinsics(0, 0), 277.1913, error);
  EXPECT_NEAR(cameraIntrinsics(1, 1), 277.1913, error);
  EXPECT_DOUBLE_EQ(cameraIntrinsics(0, 2), 160);
  EXPECT_DOUBLE_EQ(cameraIntrinsics(1, 2), 120);

  // Verify rest of the intrinsics
  EXPECT_DOUBLE_EQ(cameraIntrinsics(0, 1), 0);
  EXPECT_DOUBLE_EQ(cameraIntrinsics(1, 0), 0);
  EXPECT_DOUBLE_EQ(cameraIntrinsics(0, 1), 0);
  EXPECT_DOUBLE_EQ(cameraIntrinsics(2, 0), 0);
  EXPECT_DOUBLE_EQ(cameraIntrinsics(2, 1), 0);
  EXPECT_DOUBLE_EQ(cameraIntrinsics(2, 2), 1);

  // Verify that changing camera size changes intrinsics
  height = 1000;
  width = 1000;
  camera->SetImageHeight(height);
  camera->SetImageWidth(width);
  camera->SetHFOV(hfov);

  EXPECT_EQ(camera->ImageHeight(), height);
  EXPECT_EQ(camera->ImageWidth(), width);
  EXPECT_NEAR(camera->HFOV().Radian(), hfov, error);

  // Verify if intrinsics have changed
  cameraIntrinsics = projectionToCameraIntrinsic(
      camera->ProjectionMatrix(),
      camera->ImageWidth(),
      camera->ImageHeight()
    );
  EXPECT_NEAR(cameraIntrinsics(0, 0), 866.223, error);
  EXPECT_NEAR(cameraIntrinsics(1, 1), 866.223, error);
  EXPECT_DOUBLE_EQ(cameraIntrinsics(0, 2), 500);
  EXPECT_DOUBLE_EQ(cameraIntrinsics(1, 2), 500);

  // Clean up
  engine->DestroyScene(scene);
}

/////////////////////////////////////////////////
TEST_P(CameraTest, ViewProjectionMatrix)
{
  ViewProjectionMatrix(GetParam());
}

/////////////////////////////////////////////////
TEST_P(CameraTest, RenderTexture)
{
  RenderTexture(GetParam());
}

/////////////////////////////////////////////////
TEST_P(CameraTest, TrackFollow)
{
  TrackFollow(GetParam());
}

/////////////////////////////////////////////////
TEST_P(CameraTest, AddRemoveRenderPass)
{
  AddRemoveRenderPass(GetParam());
}

/////////////////////////////////////////////////
TEST_P(CameraTest, VisibilityMask)
{
  VisibilityMask(GetParam());
}

/////////////////////////////////////////////////
TEST_P(CameraTest, IntrinsicMatrix)
{
  IntrinsicMatrix(GetParam());
}

INSTANTIATE_TEST_CASE_P(Camera, CameraTest,
    RENDER_ENGINE_VALUES,
    PrintToStringParam());

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
