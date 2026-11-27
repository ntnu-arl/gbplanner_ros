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
#include <gz/utils/ExtraTestMacros.hh>

#include "test_config.h"  // NOLINT(build/include)

#include "gz/rendering/Camera.hh"
#include "gz/rendering/GpuRays.hh"
#include "gz/rendering/RenderEngine.hh"
#include "gz/rendering/RenderingIface.hh"
#include "gz/rendering/Scene.hh"
#include "gz/rendering/SegmentationCamera.hh"
#include "gz/rendering/ShaderParams.hh"
#include "gz/rendering/ThermalCamera.hh"

using namespace gz;
using namespace rendering;

class CameraTest: public testing::Test,
                  public testing::WithParamInterface<const char *>
{
  // Documentation inherited
  public: void SetUp() override
  {
    common::Console::SetVerbosity(4);
  }

  // Test and verify camera tracking
  public: void Track(const std::string &_renderEngine);

  // Test and verify camera following
  public: void Follow(const std::string &_renderEngine);

  // Test and verify camera visibilty mask and visual visibility flags
  public: void Visibility(const std::string &_renderEngine);

  // Test and verify camera select function method using Selection Buffer
  public: void VisualAt(const std::string &_renderEngine);

  // Test selecting visual with custom shader
  public: void ShaderSelection(const std::string &_renderEngine);

  // Path to test media directory
  public: const std::string TEST_MEDIA_PATH =
          ignition::common::joinPaths(std::string(PROJECT_SOURCE_PATH),
                "test", "media");
};

/////////////////////////////////////////////////
void CameraTest::Track(const std::string &_renderEngine)
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

#if IGNITION_RENDERING_MAJOR_VERSION <= 6
  // HACK: Tell ign-rendering6 to listen to SetTime calls
  scene->SetTime(std::chrono::nanoseconds(-1));
#endif

  VisualPtr root = scene->RootVisual();

  CameraPtr camera = scene->CreateCamera();
  ASSERT_TRUE(camera != nullptr);
  root->AddChild(camera);

  // create visual to be tracked
  VisualPtr visual = scene->CreateVisual();
  visual->AddGeometry(scene->CreateBox());
  visual->SetWorldPosition(0.0, 0.0, 0.0);
  // rotate visual to test tracking in local and world frames
  visual->SetWorldRotation(0.0, 0.0, 3.14);
  root->AddChild(visual);

  // set camera initial pose
  math::Vector3d initPos(-2, 0.0, 5.0);
  math::Quaterniond initRot = math::Quaterniond::Identity;
  camera->SetWorldPosition(initPos);
  EXPECT_EQ(initPos, camera->WorldPosition());
  EXPECT_EQ(initRot, camera->WorldRotation());

  // track visual
  camera->SetTrackTarget(visual);
  EXPECT_EQ(visual, camera->TrackTarget());

  // render a frame
  camera->Update();
  scene->SetTime(scene->Time() + std::chrono::milliseconds(16));

  EXPECT_EQ(initPos, camera->WorldPosition());
  EXPECT_NE(initRot, camera->WorldRotation());
  math::Vector3d rot = camera->WorldRotation().Euler();
  EXPECT_DOUBLE_EQ(rot.X(), 0.0);
  EXPECT_GT(rot.Y(), 0.0);
  EXPECT_DOUBLE_EQ(rot.Z(), 0.0);

  // store result that will be compared against camera tracking with p gain
  // later in the test
  math::Pose3d camPoseTrackNormal = camera->WorldPose();

  // track target with offset in world frame
  math::Vector3d trackOffset(0.0, 1.0, 0.0);
  camera->SetTrackTarget(visual, trackOffset, true);
  EXPECT_EQ(visual, camera->TrackTarget());
  EXPECT_EQ(trackOffset, camera->TrackOffset());

  // render a frame
  camera->Update();
  scene->SetTime(scene->Time() + std::chrono::milliseconds(16));

  // verify camera orientation when tracking target with offset
  // in world frame
  // camera should be looking down and to the left
  EXPECT_EQ(initPos, camera->WorldPosition());
  EXPECT_NE(initRot, camera->WorldRotation());
  rot = camera->WorldRotation().Euler();
  EXPECT_NEAR(0.0, rot.X(), 1e-6);
  EXPECT_GT(rot.Y(), 0.0);
  EXPECT_GT(rot.Z(), 0.0);

  // track visual with offset in local frame
  camera->SetTrackTarget(visual, trackOffset, false);
  EXPECT_EQ(visual, camera->TrackTarget());
  EXPECT_EQ(trackOffset, camera->TrackOffset());

  // render a frame
  camera->Update();
  scene->SetTime(scene->Time() + std::chrono::milliseconds(16));
  // verify camera orientation when tracking target with offset
  // in local frame
  // camera should be looking down and to the right
  EXPECT_EQ(initPos, camera->WorldPosition());
  EXPECT_NE(initRot, camera->WorldRotation());
  rot = camera->WorldRotation().Euler();
  EXPECT_NEAR(0.0, rot.X(), 1e-6);
  EXPECT_GT(rot.Y(), 0.0);
  EXPECT_LT(rot.Z(), 0.0);

  // disable target tracking
  camera->SetTrackTarget(nullptr);
  EXPECT_EQ(nullptr, camera->TrackTarget());

  // render a frame
  camera->Update();
  scene->SetTime(scene->Time() + std::chrono::milliseconds(16));

  // reset camera pose
  camera->SetWorldPosition(initPos);
  camera->SetWorldRotation(initRot);
  EXPECT_EQ(initPos, camera->WorldPosition());
  EXPECT_EQ(initRot, camera->WorldRotation());

  // track visual with p gain
  camera->SetTrackPGain(0.01);
  EXPECT_DOUBLE_EQ(0.01, camera->TrackPGain());
  camera->SetTrackTarget(visual);
  EXPECT_EQ(visual, camera->TrackTarget());

  // render a frame
  camera->Update();
  scene->SetTime(scene->Time() + std::chrono::milliseconds(16));

  // verify camera rotaion has pitch component
  // but not as large as before without p gain
  EXPECT_EQ(initPos, camera->WorldPosition());
  EXPECT_NE(initRot, camera->WorldRotation());
  rot = camera->WorldRotation().Euler();
  EXPECT_DOUBLE_EQ(0.0, rot.X());
  EXPECT_GT(rot.Y(), 0.0);
  EXPECT_LT(rot.Y(), camPoseTrackNormal.Rot().Euler().Y());
  EXPECT_DOUBLE_EQ(0.0, rot.Z());

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
void CameraTest::VisualAt(const std::string &_renderEngine)
{
  if (_renderEngine == "optix")
  {
    igndbg << "VisualAt not supported yet in rendering engine: "
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
  box->SetOrigin(0.0, 0.7, 0.0);
  box->SetLocalPosition(2, 0, 0);
  root->AddChild(box);

  // create sphere visual
  VisualPtr sphere = scene->CreateVisual("sphere");
  ASSERT_TRUE(sphere != nullptr);
  sphere->AddGeometry(scene->CreateSphere());
  sphere->SetOrigin(0.0, -0.7, 0.0);
  sphere->SetLocalPosition(2, 0, 0);
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

  // render a few frames
  for (auto i = 0; i < 30; ++i)
  {
    camera->Update();
    scene->SetTime(scene->Time() + std::chrono::milliseconds(16));
  }

  EXPECT_EQ(800u, camera->ImageWidth());
  EXPECT_EQ(600u, camera->ImageHeight());

  for (auto x = 0u; x < camera->ImageWidth(); x = x + 100)
  {
    auto vis = camera->VisualAt(math::Vector2i(x, camera->ImageHeight() / 2));

    if (x <= 100)
    {
      EXPECT_EQ(nullptr, vis)
          << "Found [" << vis->Name() << "] at X [" << x << "]";
    }
    else if (x > 100 && x <= 300)
    {
      // Don't end test here on failure, this condition is flaky
      EXPECT_NE(nullptr, vis) << "X: " << x;
      if (vis)
      {
        EXPECT_EQ("sphere", vis->Name());
      }
    }
    else if (x > 300 && x <= 400)
    {
      EXPECT_EQ(nullptr, vis)
          << "Found [" << vis->Name() << "] at X [" << x << "]";
    }
    else if (x > 400 && x <= 700)
    {
      // Don't end test here on failure, this condition is flaky
      EXPECT_NE(nullptr, vis) << "X: " << x;
      if (vis)
      {
        EXPECT_EQ("box", vis->Name());
      }
    }
    else
    {
      EXPECT_EQ(nullptr, vis);
    }
  }

  // change camera size
  camera->SetImageWidth(1200);
  camera->SetImageHeight(800);

  // render a few frames
  for (auto i = 0; i < 30; ++i)
  {
    camera->Update();
    scene->SetTime(scene->Time() + std::chrono::milliseconds(16));
  }

  // test that VisualAt still works after resize
  {
    unsigned int x = 300u;
    auto vis = camera->VisualAt(math::Vector2i(x, camera->ImageHeight() / 2));
    EXPECT_NE(nullptr, vis) << "X: " << x;
    if (vis)
    {
      EXPECT_EQ("sphere", vis->Name());
    }
  }

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
void CameraTest::Follow(const std::string &_renderEngine)
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

  CameraPtr camera = scene->CreateCamera();
  ASSERT_TRUE(camera != nullptr);
  root->AddChild(camera);

  // create to be followed
  VisualPtr visual = scene->CreateVisual();
  visual->AddGeometry(scene->CreateBox());
  visual->SetWorldPosition(0.0, 0.0, 0.0);
  // give visual a rotation for testing camera following in target
  // node's local frame.
  visual->SetWorldRotation(0.0, 0.0, 0.8);
  root->AddChild(visual);

  // Set camera initial pose
  math::Vector3d initPos(-20, 0.0, 5.0);
  math::Quaterniond initRot = math::Quaterniond::Identity;
  camera->SetWorldPosition(initPos);
  EXPECT_EQ(initPos, camera->WorldPosition());
  EXPECT_EQ(initRot, camera->WorldRotation());

  // track visual
  camera->SetFollowTarget(visual);
  EXPECT_EQ(visual, camera->FollowTarget());

  // render a frame
  camera->Update();
  scene->SetTime(scene->Time() + std::chrono::milliseconds(16));

  // verify camera is at same location as visual because
  // no offset is given
  EXPECT_EQ(visual->WorldPosition(), camera->WorldPosition());
  EXPECT_EQ(initRot, camera->WorldRotation());

  // follow target with offset in world frame
  math::Vector3d followOffset(-3.0, 2.0, 1.0);
  camera->SetFollowTarget(visual, followOffset, true);
  EXPECT_EQ(visual, camera->FollowTarget());
  EXPECT_EQ(followOffset, camera->FollowOffset());

  // render a frame
  camera->Update();
  scene->SetTime(scene->Time() + std::chrono::milliseconds(16));

  // verify camera pose when following target with offset
  // in world frame
  EXPECT_NE(initPos, camera->WorldPosition());
  EXPECT_EQ(initRot, camera->WorldRotation());
  math::Vector3d cameraFollowPosWorld = visual->WorldPosition()
      + followOffset;
  EXPECT_EQ(cameraFollowPosWorld, camera->WorldPosition());

  // follow target with offset in target local frame
  camera->SetFollowTarget(visual, followOffset, false);
  EXPECT_EQ(visual, camera->FollowTarget());
  EXPECT_EQ(followOffset, camera->FollowOffset());

  // render a frame
  camera->Update();
  scene->SetTime(scene->Time() + std::chrono::milliseconds(16));

  // verify camera pose when following target with offset
  // in local frame
  EXPECT_NE(initPos, camera->WorldPosition());
  EXPECT_EQ(initRot, camera->WorldRotation());
  math::Vector3d cameraFollowPosLocal =
      visual->WorldPosition() + visual->WorldRotation()*followOffset;
  EXPECT_EQ(cameraFollowPosLocal, camera->WorldPosition());
  EXPECT_NE(cameraFollowPosWorld, cameraFollowPosLocal);

  // disable target following
  camera->SetFollowTarget(nullptr);
  EXPECT_EQ(nullptr, camera->FollowTarget());

  // render a frame
  camera->Update();
  scene->SetTime(scene->Time() + std::chrono::milliseconds(16));

  // reset camera pose
  camera->SetWorldPosition(initPos);
  camera->SetWorldRotation(initRot);
  EXPECT_EQ(camera->WorldPosition(), initPos);
  EXPECT_EQ(camera->WorldRotation(), initRot);

  // follow visual with p gain
  camera->SetFollowPGain(0.03);
  EXPECT_DOUBLE_EQ(0.03, camera->FollowPGain());
  camera->SetFollowTarget(visual, followOffset, true);
  EXPECT_EQ(visual, camera->FollowTarget());

  // render a frame
  camera->Update();
  scene->SetTime(scene->Time() + std::chrono::milliseconds(16));

  // verify camera position has changed but
  // but not as close to the target as before without p gain
  EXPECT_NE(cameraFollowPosWorld, camera->WorldPosition());
  EXPECT_EQ(initRot, camera->WorldRotation());
  math::Vector3d offset = camera->WorldPosition() - visual->WorldPosition();
  EXPECT_GT(offset.Length(), followOffset.Length());

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
void CameraTest::Visibility(const std::string &_renderEngine)
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
  scene->SetBackgroundColor(0, 0, 0);
  scene->SetAmbientLight(1, 1, 1);

  VisualPtr root = scene->RootVisual();

  CameraPtr cameraA = scene->CreateCamera();
  ASSERT_TRUE(cameraA != nullptr);
  cameraA->SetWorldPosition(-1, 0, 0);
  cameraA->SetVisibilityMask(0x01);
  root->AddChild(cameraA);

  CameraPtr cameraB = scene->CreateCamera();
  ASSERT_TRUE(cameraB != nullptr);
  cameraB->SetWorldPosition(-1, 0, 0);
  cameraB->SetVisibilityMask(0x02);
  root->AddChild(cameraB);

  // create visuals with different visibility flags
  VisualPtr visualA = scene->CreateVisual();
  visualA->AddGeometry(scene->CreateBox());
  visualA->SetWorldPosition(0.0, 0.0, 0.0);
  visualA->SetVisibilityFlags(0x01);
  root->AddChild(visualA);

  VisualPtr visualB = scene->CreateVisual();
  visualB->AddGeometry(scene->CreateBox());
  visualB->SetWorldPosition(0.0, 0.0, 0.0);
  visualB->SetVisibilityFlags(0x02);
  root->AddChild(visualB);

  // create green and red material and assign to visualA and visualB
  MaterialPtr green = scene->CreateMaterial();
  green->SetAmbient(0.0, 1.0, 0.0);
  green->SetDiffuse(0.0, 1.0, 0.0);
  green->SetSpecular(0.0, 1.0, 0.0);
  visualA->SetMaterial(green);

  MaterialPtr red = scene->CreateMaterial();
  red->SetAmbient(1.0, 0.0, 0.0);
  red->SetDiffuse(1.0, 0.0, 0.0);
  red->SetSpecular(1.0, 0.0, 0.0);
  visualB->SetMaterial(red);

  // create images to store camera data
  Image imageA = cameraA->CreateImage();
  Image imageB = cameraB->CreateImage();

  unsigned int height = cameraA->ImageHeight();
  unsigned int width = cameraA->ImageWidth();
  unsigned int bpp = PixelUtil::BytesPerPixel(cameraA->ImageFormat());
  unsigned int step = width * bpp;

  ASSERT_GT(height, 0u);
  ASSERT_GT(width, 0u);
  ASSERT_GT(bpp, 0u);

  unsigned int rASum = 0u;
  unsigned int gASum = 0u;
  unsigned int bASum = 0u;
  unsigned int rBSum = 0u;
  unsigned int gBSum = 0u;
  unsigned int bBSum = 0u;

  // verify that cameraA only sees visualA and cameraB only sees VisualB
  for (unsigned int k = 0; k < 10; ++k)
  {
    cameraA->Capture(imageA);
    cameraB->Capture(imageB);

    unsigned char *dataA = imageA.Data<unsigned char>();
    unsigned char *dataB = imageB.Data<unsigned char>();

    for (unsigned int i = 0; i < height; ++i)
    {
      for (unsigned int j = 0; j < step; j+=bpp)
      {
        unsigned int idx = i * step + j;
        unsigned int rA = dataA[idx];
        unsigned int gA = dataA[idx+1];
        unsigned int bA = dataA[idx+2];

        // color should be a shade of green
        EXPECT_GT(gA, rA);
        EXPECT_GT(gA, bA);

        rASum += rA;
        gASum += gA;
        bASum += bA;

        unsigned int rB = dataB[idx];
        unsigned int gB = dataB[idx+1];
        unsigned int bB = dataB[idx+2];

        // color should be a shade of red
        EXPECT_GT(rB, gB);
        EXPECT_GT(rB, bB);

        rBSum += rB;
        gBSum += gB;
        bBSum += bB;
      }
    }
  }

  // one last test:  verify sums of rgb for bother cameras
  EXPECT_EQ(rASum, 0u);
  EXPECT_GT(gASum, 0u);
  EXPECT_EQ(bASum, 0u);

  EXPECT_GT(rBSum, 0u);
  EXPECT_EQ(gBSum, 0u);
  EXPECT_EQ(bBSum, 0u);

  // Clean up
  engine->DestroyScene(scene);
  rendering::unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
void CameraTest::ShaderSelection(const std::string &_renderEngine)
{
  if (_renderEngine == "optix")
  {
    igndbg << "Custom shaders are not supported yet in rendering engine: "
           << _renderEngine << std::endl;
    return;
  }
  else if (_renderEngine == "ogre2")
  {
    // \todo(anyone) test fails on github action (Bionic) but pass on other
    // builds. Need to investigate further.
    // Github action sets the MESA_GL_VERSION_OVERRIDE variable
    // so check for this variable and disable test if it is set.
#ifdef __linux__
    std::string value;
    bool result = common::env("MESA_GL_VERSION_OVERRIDE", value, true);
    if (result && value == "3.3")
    {
      igndbg << "Test is run on machine with software rendering or mesa driver "
             << "Skipping test. " << std::endl;
      return;
    }
#endif
  }

  // This test checks that custom shaders are being rendering correctly in
  // camera view. It also verifies that visual selection is working and the
  // visual's material remains the same after selection.

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
  scene->SetAmbientLight(1, 1, 1);

  VisualPtr root = scene->RootVisual();

  // create directional light
  DirectionalLightPtr light = scene->CreateDirectionalLight();
  light->SetDirection(1.0, 0.0, -1);
  light->SetDiffuseColor(0.5, 0.5, 0.5);
  light->SetSpecularColor(0.5, 0.5, 0.5);
  root->AddChild(light);

  std::string vertexShaderFile;
  std::string fragmentShaderFile;
  if (_renderEngine == "ogre2")
  {
    vertexShaderFile = "simple_color_330_vs.glsl";
    fragmentShaderFile = "simple_color_330_fs.glsl";
  }
  else if (_renderEngine == "ogre")
  {
    vertexShaderFile = "simple_color_vs.glsl";
    fragmentShaderFile = "simple_color_fs.glsl";
  }

  // create shader materials
  // path to look for vertex and fragment shader parameters
  std::string vertexShaderPath = ignition::common::joinPaths(
      TEST_MEDIA_PATH, "materials", "programs", vertexShaderFile);
  std::string fragmentShaderPath = ignition::common::joinPaths(
      TEST_MEDIA_PATH, "materials", "programs", fragmentShaderFile);

  // create shader material
  ignition::rendering::MaterialPtr shader = scene->CreateMaterial();
  shader->SetVertexShader(vertexShaderPath);
  shader->SetFragmentShader(fragmentShaderPath);

  // create visual
  VisualPtr visual = scene->CreateVisual("box");
  visual->AddGeometry(scene->CreateBox());
  visual->SetWorldPosition(2.0, 0.0, 0.0);
  visual->SetWorldRotation(0.0, 0.0, 0.0);
  visual->SetMaterial(shader);
  root->AddChild(visual);
  // for thermal camera
  visual->SetUserData("temperature", 310.0f);
  // for segmentation camera
  visual->SetUserData("label", 1);

  // visual will clone and create a unique material
  // so destroy this one
  scene->DestroyMaterial(shader);

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

  // Create a gpu ray
  // laser retro material switching may also affect shader materials
  const double hMinAngle = -IGN_PI/2.0;
  const double hMaxAngle = IGN_PI/2.0;
  const double minRange = 0.1;
  const double maxRange = 10.0;
  const int hRayCount = 320;
  const int vRayCount = 1;
  GpuRaysPtr gpuRays = scene->CreateGpuRays("gpu_rays");
  gpuRays->SetWorldPosition(0, 0, 0);
  gpuRays->SetNearClipPlane(minRange);
  gpuRays->SetFarClipPlane(maxRange);
  gpuRays->SetAngleMin(hMinAngle);
  gpuRays->SetAngleMax(hMaxAngle);
  gpuRays->SetRayCount(hRayCount);
  gpuRays->SetVerticalRayCount(vRayCount);
  root->AddChild(gpuRays);

  // Create thermal camera
  // heat map material switching may also affect shader materials
  auto thermalCamera = scene->CreateThermalCamera("ThermalCamera");
  ASSERT_NE(thermalCamera, nullptr);
  thermalCamera->SetAmbientTemperature(296.0f);
  thermalCamera->SetAspectRatio(1.333);
  thermalCamera->SetImageWidth(320);
  thermalCamera->SetImageHeight(240);
  thermalCamera->SetHFOV(IGN_PI_2);
  root->AddChild(thermalCamera);

  // Currently, only ogre2 supports segmentation cameras
  SegmentationCameraPtr segmentationCamera;
  if (_renderEngine == "ogre2")
  {
    // Create segmentation camera
    // segmentation material switching may also affect shader materials
    segmentationCamera =
        scene->CreateSegmentationCamera("SegmentationCamera");
    ASSERT_NE(camera, nullptr);
    segmentationCamera->SetLocalPosition(0.0, 0.0, 0.0);
    segmentationCamera->SetLocalRotation(0.0, 0.0, 0.0);
    segmentationCamera->SetBackgroundLabel(23);
    segmentationCamera->SetSegmentationType(SegmentationType::ST_SEMANTIC);
    segmentationCamera->EnableColoredMap(false);
    segmentationCamera->SetAspectRatio(1.333);
    segmentationCamera->SetImageWidth(320);
    segmentationCamera->SetImageHeight(240);
    segmentationCamera->SetHFOV(IGN_PI_2);
    root->AddChild(segmentationCamera);

    // worldviewproj_matrix is a constant defined by ogre.
    // Here we add a line to add this constant to the params.
    // The specified value is ignored as it will be auto bound to the
    // correct type and value.
    auto params = visual->Material()->VertexShaderParams();
    (*params)["worldviewproj_matrix"] = 1;

    // check setting invalid param - this should print a warning msg and
    // not cause the program to crash.
    (*params)["worldviewproj_matrix_invalid"] = 1;
  }

  // render a few frames
  for (auto i = 0; i < 30; ++i)
  {
    camera->Update();
    gpuRays->Update();
    thermalCamera->Update();
    if (segmentationCamera)
      segmentationCamera->Update();
    scene->SetTime(scene->Time() + std::chrono::milliseconds(16));
  }

  // capture a frame
  Image image = camera->CreateImage();
  camera->Capture(image);

  // verify correct visual is returned
  VisualPtr vis = camera->VisualAt(
      math::Vector2i(camera->ImageWidth() / 2, camera->ImageHeight() / 2));
  EXPECT_NE(nullptr, vis);
  EXPECT_EQ("box", vis->Name());

  // capture another frame
  Image image2 = camera->CreateImage();
  camera->Capture(image2);

  unsigned char *data = image.Data<unsigned char>();
  unsigned char *data2 = image2.Data<unsigned char>();
  unsigned int height = camera->ImageHeight();
  unsigned int width = camera->ImageWidth();

  // verify that camera sees red color before and after selection
  int mid = (height / 2 * width * 3u) + (width / 2 - 1) * 3u;
  int r = static_cast<int>(data[mid]);
  int g = static_cast<int>(data[mid+1]);
  int b = static_cast<int>(data[mid+2]);
  int r2 = static_cast<int>(data2[mid]);
  int g2 = static_cast<int>(data2[mid+1]);
  int b2 = static_cast<int>(data2[mid+2]);

  EXPECT_EQ(r, r2);
  EXPECT_EQ(g, g2);
  EXPECT_EQ(b, b2);

  EXPECT_GT(r, g);
  EXPECT_GT(r, b);
  EXPECT_EQ(g, b);

  // Clean up
  engine->DestroyScene(scene);

  ASSERT_EQ(1u, camera.use_count());
  ASSERT_EQ(1u, gpuRays.use_count());
  ASSERT_EQ(1u, thermalCamera.use_count());
  if (segmentationCamera)
  {
    ASSERT_EQ(1u, segmentationCamera.use_count());
  }

  rendering::unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
TEST_P(CameraTest, Track)
{
  Track(GetParam());
}

/////////////////////////////////////////////////
TEST_P(CameraTest, Follow)
{
  Follow(GetParam());
}

/////////////////////////////////////////////////
TEST_P(CameraTest, Visibility)
{
  Visibility(GetParam());
}

/////////////////////////////////////////////////
TEST_P(CameraTest, VisualAt)
{
  VisualAt(GetParam());
}

/////////////////////////////////////////////////
// See: https://github.com/gazebosim/gz-rendering/issues/654
TEST_P(CameraTest,
       IGN_UTILS_TEST_DISABLED_ON_MAC(ShaderSelection))
{
  ShaderSelection(GetParam());
}

INSTANTIATE_TEST_CASE_P(Camera, CameraTest,
    RENDER_ENGINE_VALUES,
    PrintToStringParam());

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
