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
#include <gz/common/Filesystem.hh>
#include <gz/common/Event.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "test_config.h"  // NOLINT(build/include)

#include "gz/rendering/DepthCamera.hh"
#include "gz/rendering/ParticleEmitter.hh"
#include "gz/rendering/RenderEngine.hh"
#include "gz/rendering/RenderingIface.hh"
#include "gz/rendering/Scene.hh"

#define DEPTH_TOL 1e-4
#define DOUBLE_TOL 1e-6

unsigned int g_depthCounter = 0;
unsigned int g_pointCloudCounter = 0;

void OnNewDepthFrame(float *_scanDest, const float *_scan,
                  unsigned int _width, unsigned int _height,
                  unsigned int _channels,
                  const std::string &/*_format*/)
{
  float f;
  int size =  _width * _height * _channels;
  memcpy(_scanDest, _scan, size * sizeof(f));
  g_depthCounter++;
}

void OnNewRgbPointCloud(float *_scanDest, const float *_scan,
                  unsigned int _width, unsigned int _height,
                  unsigned int _channels,
                  const std::string &/*_format*/)
{
  float f;
  int size =  _width * _height * _channels;
  memcpy(_scanDest, _scan, size * sizeof(f));
  g_pointCloudCounter++;
}

class DepthCameraTest: public testing::Test,
  public testing::WithParamInterface<const char *>
{
  // Create a Camera sensor from a SDF and gets a image message
  public: void DepthCameraBoxes(const std::string &_renderEngine);

  // Compare depth camera image before and after adding particles
  // in the scene
  public: void DepthCameraParticles(const std::string &_renderEngine);
};

void DepthCameraTest::DepthCameraBoxes(
    const std::string &_renderEngine)
{
  int imgWidth_ = 256;
  int imgHeight_ = 256;
  double aspectRatio_ = imgWidth_/imgHeight_;

  double unitBoxSize = 1.0;
  gz::math::Vector3d boxPosition(1.8, 0.0, 0.0);

  // Optix is not supported
  if (_renderEngine.compare("optix") == 0)
  {
    igndbg << "Engine '" << _renderEngine
              << "' doesn't support depth cameras" << std::endl;
    return;
  }

  // Setup ign-rendering with an empty scene
  auto *engine = gz::rendering::engine(_renderEngine);
  if (!engine)
  {
    igndbg << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }

  gz::rendering::ScenePtr scene = engine->CreateScene("scene");

  // red background
  scene->SetBackgroundColor(1.0, 0.0, 0.0);

#if IGNITION_RENDERING_MAJOR_VERSION <= 6
  // HACK: Tell ign-rendering6 to listen to SetTime calls
  scene->SetTime(std::chrono::nanoseconds(-1));
#endif

  // Create an scene with a box in it
  scene->SetAmbientLight(1.0, 1.0, 1.0);
  gz::rendering::VisualPtr root = scene->RootVisual();

  // create blue material
  gz::rendering::MaterialPtr blue = scene->CreateMaterial();
  blue->SetAmbient(0.0, 0.0, 1.0);
  blue->SetDiffuse(0.0, 0.0, 1.0);
  blue->SetSpecular(0.0, 0.0, 1.0);

  // create box visual
  gz::rendering::VisualPtr box = scene->CreateVisual();
  box->AddGeometry(scene->CreateBox());
  box->SetOrigin(0.0, 0.0, 0.0);
  box->SetLocalPosition(boxPosition);
  box->SetLocalRotation(0, 0, 0);
  box->SetLocalScale(unitBoxSize, unitBoxSize, unitBoxSize);
  box->SetMaterial(blue);
  root->AddChild(box);
  {
    double farDist = 10.0;
    double nearDist = 0.15;
    double hfov_ = 1.05;
    // Create depth camera
    auto depthCamera = scene->CreateDepthCamera("DepthCamera");
    ASSERT_NE(depthCamera, nullptr);

    gz::math::Pose3d testPose(gz::math::Vector3d(0, 0, 0),
        gz::math::Quaterniond::Identity);
    depthCamera->SetLocalPose(testPose);

    // Configure depth camera
    depthCamera->SetImageWidth(imgWidth_);
    EXPECT_EQ(depthCamera->ImageWidth(),
      static_cast<unsigned int>(imgWidth_));
    depthCamera->SetImageHeight(imgHeight_);
    EXPECT_EQ(depthCamera->ImageHeight(),
      static_cast<unsigned int>(imgHeight_));
    depthCamera->SetFarClipPlane(farDist);
    EXPECT_NEAR(depthCamera->FarClipPlane(), farDist, DOUBLE_TOL);
    depthCamera->SetNearClipPlane(nearDist);
    EXPECT_NEAR(depthCamera->NearClipPlane(), nearDist, DOUBLE_TOL);
    depthCamera->SetAspectRatio(aspectRatio_);
    EXPECT_NEAR(depthCamera->AspectRatio(), aspectRatio_, DOUBLE_TOL);
    depthCamera->SetHFOV(hfov_);
    EXPECT_NEAR(depthCamera->HFOV().Radian(), hfov_, DOUBLE_TOL);

    depthCamera->CreateDepthTexture();
    scene->RootVisual()->AddChild(depthCamera);

    // Set a callback on the  camera sensor to get a depth camera frame
    float *scan = new float[imgHeight_ * imgWidth_];
    gz::common::ConnectionPtr connection =
      depthCamera->ConnectNewDepthFrame(
          std::bind(&::OnNewDepthFrame, scan,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
            std::placeholders::_4, std::placeholders::_5));

    // rgb point cloud data callback
    unsigned int pointCloudChannelCount = 4u;
    float *pointCloudData = new float[
        imgHeight_ * imgWidth_ * pointCloudChannelCount];
    gz::common::ConnectionPtr connection2 =
      depthCamera->ConnectNewRgbPointCloud(
          std::bind(&::OnNewRgbPointCloud, pointCloudData,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
            std::placeholders::_4, std::placeholders::_5));

    // update and verify we get new data
    g_depthCounter = 0u;
    g_pointCloudCounter = 0u;
    depthCamera->Update();
    scene->SetTime(scene->Time() + std::chrono::milliseconds(16));
    EXPECT_EQ(1u, g_depthCounter);
    EXPECT_EQ(1u, g_pointCloudCounter);

    // compute mid, left, and right indices to be used later for retrieving data
    // from depth and point cloud image

    // depth image indices
    int midWidth = static_cast<int>(depthCamera->ImageWidth() * 0.5);
    int midHeight = static_cast<int>(depthCamera->ImageHeight() * 0.5);
    int mid = midHeight * depthCamera->ImageWidth() + midWidth -1;
    double expectedRangeAtMidPoint = boxPosition.X() - unitBoxSize * 0.5;
    int left = midHeight * depthCamera->ImageWidth();
    int right = (midHeight+1) * depthCamera->ImageWidth() - 1;

    // point cloud image indices
    int pcMid = midHeight * depthCamera->ImageWidth() * pointCloudChannelCount
        + (midWidth-1) * pointCloudChannelCount;
    int pcLeft = midHeight * depthCamera->ImageWidth() * pointCloudChannelCount;
    int pcRight = (midHeight+1)
        * (depthCamera->ImageWidth() * pointCloudChannelCount)
        - pointCloudChannelCount;

    float minVal = -gz::math::INF_D;
    float maxVal = gz::math::INF_D;

    // Verify Depth
    // Depth sensor should see box in the middle of the image
    EXPECT_NEAR(expectedRangeAtMidPoint, scan[mid], DEPTH_TOL);
    // The left and right side of the depth frame should be max value
    EXPECT_FLOAT_EQ(maxVal, scan[left]);
    EXPECT_FLOAT_EQ(maxVal, scan[right]);

    // Verify Point Cloud XYZ values
    {
      // check mid point
      float mx = pointCloudData[pcMid];
      float my = pointCloudData[pcMid + 1];
      float mz = pointCloudData[pcMid + 2];
      EXPECT_FLOAT_EQ(scan[mid], mx);

      // check left and right points
      float lx = pointCloudData[pcLeft];
      float ly = pointCloudData[pcLeft + 1];
      float lz = pointCloudData[pcLeft + 2];
      EXPECT_FLOAT_EQ(maxVal, ly);
      EXPECT_FLOAT_EQ(maxVal, lz);
      EXPECT_FLOAT_EQ(scan[left], lx);

      float rx = pointCloudData[pcRight];
      float ry = pointCloudData[pcRight + 1];
      float rz = pointCloudData[pcRight + 2];
      EXPECT_FLOAT_EQ(maxVal, rx);
      EXPECT_FLOAT_EQ(maxVal, ry);
      EXPECT_FLOAT_EQ(maxVal, rz);
      EXPECT_FLOAT_EQ(scan[right], rx);

      // point to the left of mid point should have larger y value than mid
      // point, which in turn should have large y value than point to the
      // right of mid point
      float midLeftY = pointCloudData[pcMid + 1 - pointCloudChannelCount];
      float midRightY = pointCloudData[pcMid + 1 + pointCloudChannelCount];
      EXPECT_GT(midLeftY, my);
      EXPECT_GT(my, midRightY);
      EXPECT_GT(midLeftY, 0.0);
      EXPECT_LT(midRightY, 0.0);

      // all points on the box should have the same z position
      float midLeftZ = pointCloudData[pcMid + 2 - pointCloudChannelCount];
      float midRightZ = pointCloudData[pcMid + 2 + pointCloudChannelCount];
      EXPECT_NEAR(mz, midLeftZ, DEPTH_TOL);
      EXPECT_NEAR(mz, midRightZ, DEPTH_TOL);

      // Verify Point Cloud RGB values
      // The mid point should be blue
      float mc = pointCloudData[pcMid + 3];
      uint32_t *mrgba = reinterpret_cast<uint32_t *>(&mc);
      unsigned int mr = *mrgba >> 24 & 0xFF;
      unsigned int mg = *mrgba >> 16 & 0xFF;
      unsigned int mb = *mrgba >> 8 & 0xFF;
      unsigned int ma = *mrgba >> 0 & 0xFF;
      EXPECT_EQ(0u, mr);
      EXPECT_EQ(0u, mg);
      EXPECT_GT(mb, 0u);

      // Far left and right points should be red (background color)
      float lc = pointCloudData[pcLeft + 3];
      uint32_t *lrgba = reinterpret_cast<uint32_t *>(&lc);
      unsigned int lr = *lrgba >> 24 & 0xFF;
      unsigned int lg = *lrgba >> 16 & 0xFF;
      unsigned int lb = *lrgba >> 8 & 0xFF;
      unsigned int la = *lrgba >> 0 & 0xFF;

      EXPECT_EQ(255u, lr);
      EXPECT_EQ(0u, lg);
      EXPECT_EQ(0u, lb);

      float rc = pointCloudData[pcRight + 3];
      uint32_t *rrgba = reinterpret_cast<uint32_t *>(&rc);
      unsigned int rr = *rrgba >> 24 & 0xFF;
      unsigned int rg = *rrgba >> 16 & 0xFF;
      unsigned int rb = *rrgba >> 8 & 0xFF;
      unsigned int ra = *rrgba >> 0 & 0xFF;

      EXPECT_EQ(255u, rr);
      EXPECT_EQ(0u, rg);
      EXPECT_EQ(0u, rb);

      // Note: internal texture format used is RGB with no alpha channel
      // We observed the values can be either 255 or 0 but graphics card
      // drivers are free to fill it with any value they want.
      // This should be fixed in ogre 2.2 in ign-rendering6 which forbids
      // the use of RGB format.
      // see https://github.com/ignitionrobotics/ign-rendering/issues/315
      EXPECT_TRUE(255u == ma || 0u == ma);
      EXPECT_TRUE(255u == la || 0u == la);
      EXPECT_TRUE(255u == ra || 0u == ra);
    }

    // Check that for a box really close it returns it is not seen
    gz::math::Vector3d boxPositionNear(
        unitBoxSize * 0.5 + nearDist * 0.5, 0.0, 0.0);
    box->SetLocalPosition(boxPositionNear);

    // update and verify we get new data
    g_depthCounter = 0u;
    g_pointCloudCounter = 0u;
    depthCamera->Update();
    scene->SetTime(scene->Time() + std::chrono::milliseconds(16));
    EXPECT_EQ(1u, g_depthCounter);
    EXPECT_EQ(1u, g_pointCloudCounter);

    // Verify Depth
    // box not detected
    EXPECT_FLOAT_EQ(minVal, scan[mid]);
    EXPECT_FLOAT_EQ(minVal, scan[left]);
    EXPECT_FLOAT_EQ(minVal, scan[right]);

    // Verify Point Cloud XYZ
    {
      // all points should be min val
      for (unsigned int i = 0; i < depthCamera->ImageHeight(); ++i)
      {
        unsigned int step = i*depthCamera->ImageWidth()*pointCloudChannelCount;
        for (unsigned int j = 0; j < depthCamera->ImageWidth(); ++j)
        {
          float x = pointCloudData[step + j*pointCloudChannelCount];
          float y = pointCloudData[step + j*pointCloudChannelCount + 1];
          float z = pointCloudData[step + j*pointCloudChannelCount + 2];
          EXPECT_FLOAT_EQ(minVal, x);
          EXPECT_FLOAT_EQ(minVal, y);
          EXPECT_FLOAT_EQ(minVal, z);
         }
      }

      // Verify Point Cloud RGB
      // all points should be red (background color)
      for (unsigned int i = 0; i < depthCamera->ImageHeight(); ++i)
      {
        unsigned int step = i*depthCamera->ImageWidth()*pointCloudChannelCount;
        for (unsigned int j = 0; j < depthCamera->ImageWidth(); ++j)
        {
          float color = pointCloudData[step + j*pointCloudChannelCount + 3];
          uint32_t *rgba = reinterpret_cast<uint32_t *>(&color);
          unsigned int r = *rgba >> 24 & 0xFF;
          unsigned int g = *rgba >> 16 & 0xFF;
          unsigned int b = *rgba >> 8 & 0xFF;
          unsigned int a = *rgba >> 0 & 0xFF;
          EXPECT_EQ(255u, r);
          EXPECT_EQ(0u, g);
          EXPECT_EQ(0u, b);
          EXPECT_EQ(255u, a);
        }
      }
    }

    // Check that for a box really far it returns max val
    gz::math::Vector3d boxPositionFar(
        unitBoxSize * 0.5 + farDist * 1.5, 0.0, 0.0);
    box->SetLocalPosition(boxPositionFar);

    // update and verify we get new data
    g_depthCounter = 0u;
    g_pointCloudCounter = 0u;
    depthCamera->Update();
    scene->SetTime(scene->Time() + std::chrono::milliseconds(16));
    EXPECT_EQ(1u, g_depthCounter);
    EXPECT_EQ(1u, g_pointCloudCounter);

    // Verify Depth
    {
      // box not detected so all should return max val
      EXPECT_FLOAT_EQ(maxVal, scan[mid]);
      EXPECT_FLOAT_EQ(maxVal, scan[left]);
      EXPECT_FLOAT_EQ(maxVal, scan[right]);
    }

    // Verify Point Cloud XYZ
    {
      // all points should be maxVal
      for (unsigned int i = 0; i < depthCamera->ImageHeight(); ++i)
      {
        unsigned int step = i*depthCamera->ImageWidth()*pointCloudChannelCount;
        for (unsigned int j = 0; j < depthCamera->ImageWidth(); ++j)
        {
          float x = pointCloudData[step + j*pointCloudChannelCount];
          float y = pointCloudData[step + j*pointCloudChannelCount + 1];
          float z = pointCloudData[step + j*pointCloudChannelCount + 2];
          EXPECT_FLOAT_EQ(maxVal, x);
          EXPECT_FLOAT_EQ(maxVal, y);
          EXPECT_FLOAT_EQ(maxVal, z);
        }
      }

      // Verify Point Cloud RGB
      // all points should be red (background color)
      for (unsigned int i = 0; i < depthCamera->ImageHeight(); ++i)
      {
        unsigned int step = i*depthCamera->ImageWidth()*pointCloudChannelCount;
        for (unsigned int j = 0; j < depthCamera->ImageWidth(); ++j)
        {
          float color = pointCloudData[step + j*pointCloudChannelCount + 3];
          uint32_t *rgba = reinterpret_cast<uint32_t *>(&color);
          unsigned int r = *rgba >> 24 & 0xFF;
          unsigned int g = *rgba >> 16 & 0xFF;
          unsigned int b = *rgba >> 8 & 0xFF;
          unsigned int a = *rgba >> 0 & 0xFF;
          EXPECT_EQ(255u, r);
          EXPECT_EQ(0u, g);
          EXPECT_EQ(0u, b);
          EXPECT_EQ(255u, a);
        }
      }
    }

    // Check that the depth values for a box do not warp.
    gz::math::Vector3d boxPositionFillFrame(
        unitBoxSize * 0.5 + 0.2, 0.0, 0.0);
    box->SetLocalPosition(boxPositionFillFrame);

    // update and verify we get new data
    g_depthCounter = 0u;
    g_pointCloudCounter = 0u;
    depthCamera->Update();
    scene->SetTime(scene->Time() + std::chrono::milliseconds(16));
    EXPECT_EQ(1u, g_depthCounter);
    EXPECT_EQ(1u, g_pointCloudCounter);

    double expectedRange = boxPositionFillFrame.X() - unitBoxSize * 0.5;

    // Verify Depth
    {
      // all points should have the same depth value
      EXPECT_FLOAT_EQ(expectedRange, scan[mid]);
      EXPECT_FLOAT_EQ(expectedRange, scan[left]);
      EXPECT_FLOAT_EQ(expectedRange, scan[right]);
    }
    // Verify Point Cloud XYZ
    {
      // all points should have the same X value
      for (unsigned int i = 0; i < depthCamera->ImageHeight(); ++i)
      {
        unsigned int step = i*depthCamera->ImageWidth()*pointCloudChannelCount;
        for (unsigned int j = 0; j < depthCamera->ImageWidth(); ++j)
        {
          float x = pointCloudData[step + j*pointCloudChannelCount];
          EXPECT_NEAR(expectedRange, x, DOUBLE_TOL);
        }
      }

      // Verify Point Cloud RGB
      // all points should be blue
      for (unsigned int i = 0; i < depthCamera->ImageHeight(); ++i)
      {
        unsigned int step = i*depthCamera->ImageWidth()*pointCloudChannelCount;
        for (unsigned int j = 0; j < depthCamera->ImageWidth(); ++j)
        {
          float color = pointCloudData[step + j*pointCloudChannelCount + 3];
          uint32_t *rgba = reinterpret_cast<uint32_t *>(&color);
          unsigned int r = *rgba >> 24 & 0xFF;
          unsigned int g = *rgba >> 16 & 0xFF;
          unsigned int b = *rgba >> 8 & 0xFF;
          unsigned int a = *rgba >> 0 & 0xFF;
          EXPECT_EQ(0u, r);
          EXPECT_EQ(0u, g);
          EXPECT_GT(b, 0u);
          EXPECT_EQ(255u, a);
        }
      }
    }

    // Clean up
    connection.reset();
    delete [] scan;
    if (pointCloudData)
      delete [] pointCloudData;
  }

  engine->DestroyScene(scene);
  gz::rendering::unloadEngine(engine->Name());
}

void DepthCameraTest::DepthCameraParticles(
    const std::string &_renderEngine)
{
  int imgWidth_ = 256;
  int imgHeight_ = 256;
  double aspectRatio_ = imgWidth_ / imgHeight_;

  // box should fill camera view
  // we will add particle emitter in between box and depth camera later
  ignition::math::Vector3d boxSize(1.0, 10.0, 10.0);
  ignition::math::Vector3d boxPosition(1.8, 0.0, 0.0);

  // particle emitter is only supported in ogre2
  if (_renderEngine.compare("ogre2") != 0)
  {
    igndbg << "Engine '" << _renderEngine
              << "' doesn't support depth cameras" << std::endl;
    return;
  }

  // Setup ign-rendering with an empty scene
  auto *engine = ignition::rendering::engine(_renderEngine);
  if (!engine)
  {
    igndbg << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }

  ignition::rendering::ScenePtr scene = engine->CreateScene("scene");

  // red background
  scene->SetBackgroundColor(1.0, 0.0, 0.0);

#if IGNITION_RENDERING_MAJOR_VERSION <= 6
  // HACK: Tell ign-rendering6 to listen to SetTime calls
  scene->SetTime(std::chrono::nanoseconds(-1));
#endif

  // Create an scene with a box in it
  scene->SetAmbientLight(1.0, 1.0, 1.0);
  ignition::rendering::VisualPtr root = scene->RootVisual();

  // create blue material
  ignition::rendering::MaterialPtr blue = scene->CreateMaterial();
  blue->SetAmbient(0.0, 0.0, 1.0);
  blue->SetDiffuse(0.0, 0.0, 1.0);
  blue->SetSpecular(0.0, 0.0, 1.0);

  // create box visual
  ignition::rendering::VisualPtr box = scene->CreateVisual();
  box->AddGeometry(scene->CreateBox());
  box->SetOrigin(0.0, 0.0, 0.0);
  box->SetLocalPosition(boxPosition);
  box->SetLocalRotation(0, 0, 0);
  box->SetLocalScale(boxSize);
  box->SetMaterial(blue);
  root->AddChild(box);
  {
    double farDist = 10.0;
    double nearDist = 0.01;
    double hfov_ = 1.05;
    // Create depth camera
    auto depthCamera = scene->CreateDepthCamera("DepthCamera");
    ASSERT_NE(depthCamera, nullptr);

    ignition::math::Pose3d testPose(ignition::math::Vector3d(0, 0, 0),
        ignition::math::Quaterniond::Identity);
    depthCamera->SetLocalPose(testPose);

    // Configure depth camera
    depthCamera->SetImageWidth(imgWidth_);
    EXPECT_EQ(depthCamera->ImageWidth(),
      static_cast<unsigned int>(imgWidth_));
    depthCamera->SetImageHeight(imgHeight_);
    EXPECT_EQ(depthCamera->ImageHeight(),
      static_cast<unsigned int>(imgHeight_));
    depthCamera->SetFarClipPlane(farDist);
    EXPECT_DOUBLE_EQ(depthCamera->FarClipPlane(), farDist);
    depthCamera->SetNearClipPlane(nearDist);
    EXPECT_DOUBLE_EQ(depthCamera->NearClipPlane(), nearDist);
    depthCamera->SetAspectRatio(aspectRatio_);
    EXPECT_DOUBLE_EQ(depthCamera->AspectRatio(), aspectRatio_);
    depthCamera->SetHFOV(hfov_);
    EXPECT_DOUBLE_EQ(depthCamera->HFOV().Radian(), hfov_);

    depthCamera->CreateDepthTexture();
    scene->RootVisual()->AddChild(depthCamera);

    // Set a callback on the camera sensor to get a depth camera frame
    float *scan = new float[imgHeight_ * imgWidth_];
    ignition::common::ConnectionPtr connection =
      depthCamera->ConnectNewDepthFrame(
          std::bind(&::OnNewDepthFrame, scan,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
            std::placeholders::_4, std::placeholders::_5));

    // rgb point cloud data callback
    unsigned int pointCloudChannelCount = 4u;
    float *pointCloudData = new float[
        imgHeight_ * imgWidth_ * pointCloudChannelCount];
    ignition::common::ConnectionPtr connection2 =
      depthCamera->ConnectNewRgbPointCloud(
          std::bind(&::OnNewRgbPointCloud, pointCloudData,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
            std::placeholders::_4, std::placeholders::_5));

    // update and verify we get new data
    g_depthCounter = 0u;
    g_pointCloudCounter = 0u;
    depthCamera->Update();
    scene->SetTime(scene->Time() + std::chrono::milliseconds(16));
    EXPECT_EQ(1u, g_depthCounter);
    EXPECT_EQ(1u, g_pointCloudCounter);

    double expectedDepth = boxPosition.X() - boxSize.X() * 0.5;

    double pointAvg = 0.0;
    double depthAvg = 0.0;
    // Verify depth and point cloud data before particle effects
    for (unsigned int i = 0u; i < depthCamera->ImageHeight(); ++i)
    {
      unsigned int step =
          i * depthCamera->ImageWidth() * pointCloudChannelCount;
      for (unsigned int j = 0u; j < depthCamera->ImageWidth(); ++j)
      {
        float x = pointCloudData[step + j * pointCloudChannelCount];
        float y = pointCloudData[step + j * pointCloudChannelCount + 1];
        float z = pointCloudData[step + j * pointCloudChannelCount + 2];
        EXPECT_NEAR(expectedDepth, x, DEPTH_TOL);
        float d = scan[i * depthCamera->ImageWidth() + j];
        EXPECT_NEAR(expectedDepth, d, DEPTH_TOL);

        pointAvg += ignition::math::Vector3d(x, y, z).Length();
        depthAvg += d;
      }
    }

    // create particle emitter between depth camera and box
    ignition::math::Vector3d particlePosition(1.0, 0, 0);
    ignition::math::Quaterniond particleRotation(
        ignition::math::Vector3d(0, -1.57, 0));
    ignition::math::Vector3d particleSize(0.2, 0.2, 0.2);
    ignition::rendering::ParticleEmitterPtr emitter =
        scene->CreateParticleEmitter();
    emitter->SetLocalPosition(particlePosition);
    emitter->SetLocalRotation(particleRotation);
    emitter->SetParticleSize(particleSize);
    emitter->SetRate(100);
    emitter->SetLifetime(2);
    emitter->SetVelocityRange(0.1, 0.1);
    emitter->SetScaleRate(0.0);
    emitter->SetColorRange(ignition::math::Color::Red,
        ignition::math::Color::Black);
    emitter->SetEmitting(true);
    root->AddChild(emitter);

    // update and verify we get new data
    // make sure to update for a few frames for particles for flow into
    // camera view.
    g_depthCounter = 0u;
    g_pointCloudCounter = 0u;
    for (unsigned int i = 0; i < 100; ++i)
    {
      depthCamera->Update();
      scene->SetTime(scene->Time() + std::chrono::milliseconds(16));
    }
    EXPECT_EQ(100u, g_depthCounter);
    EXPECT_EQ(100u, g_pointCloudCounter);

    double pointParticleAvg = 0.0;
    double depthParticleAvg = 0.0;

    // set a larger tol for particle depth
    // depth noise is computed based on particle size
    double depthNoiseTol = particleSize.X() + particleSize.X() * 0.5;
    double expectedParticleDepth = particlePosition.X();

    // Verify depth and point cloud data after particle effects
    for (unsigned int i = 0u; i < depthCamera->ImageHeight(); ++i)
    {
      unsigned int step =
          i * depthCamera->ImageWidth() * pointCloudChannelCount;
      for (unsigned int j = 0u; j < depthCamera->ImageWidth(); ++j)
      {
        float x = pointCloudData[step + j * pointCloudChannelCount];
        float y = pointCloudData[step + j * pointCloudChannelCount + 1];
        float z = pointCloudData[step + j * pointCloudChannelCount + 2];

        double xd = static_cast<double>(x);
        // depth camera sees only certain percentage of particles
        // so the values should be either
        //   * box depth (depth camera does not see particles), or
        //   * noisy particle depth (depth camera see particles but values
        //     are affected by noise)
        EXPECT_TRUE(
            ignition::math::equal(expectedParticleDepth, xd, depthNoiseTol) ||
            ignition::math::equal(expectedDepth, xd, DEPTH_TOL))
            << "actual vs expected particle depth: "
            << xd << " vs " << expectedParticleDepth;
        float depth = scan[i * depthCamera->ImageWidth() + j];
        double depthd = static_cast<double>(depth);
        EXPECT_TRUE(
            ignition::math::equal(expectedParticleDepth, depthd, depthNoiseTol)
            || ignition::math::equal(expectedDepth, depthd, DEPTH_TOL))
            << "actual vs expected particle depth: "
            << depthd << " vs " << expectedParticleDepth;

        pointParticleAvg += ignition::math::Vector3d(x, y, z).Length();
        depthParticleAvg += depthd;
      }
    }

    // compare point and depth data before and after particle effects
    // the avg point length and depth values in the image with particle effects
    // should be lower than the image without particle effects
    double pixelCount = depthCamera->ImageWidth() * depthCamera->ImageHeight();
    pointAvg /= pixelCount;
    depthAvg /= pixelCount;
    pointParticleAvg /= pixelCount;
    depthParticleAvg /= pixelCount;
    EXPECT_LT(pointParticleAvg, pointAvg);
    EXPECT_LT(depthParticleAvg, depthAvg);

    // test setting particle scatter ratio
    // reduce particle scatter ratio - this creates a "less dense" particle
    // emitter so we should have larger depth values on avg since fewers
    // depth readings are occluded by particles
    emitter->SetParticleScatterRatio(0.1f);

    g_depthCounter = 0u;
    g_pointCloudCounter = 0u;
    for (unsigned int i = 0; i < 100; ++i)
    {
      depthCamera->Update();
      scene->SetTime(scene->Time() + std::chrono::milliseconds(16));
    }
    EXPECT_EQ(100u, g_depthCounter);
    EXPECT_EQ(100u, g_pointCloudCounter);

    double pointParticleLowScatterAvg = 0.0;
    double depthParticleLowScatterAvg = 0.0;

    // Verify depth and point cloud data after setting particle scatter ratio
    for (unsigned int i = 0u; i < depthCamera->ImageHeight(); ++i)
    {
      unsigned int step =
          i * depthCamera->ImageWidth() * pointCloudChannelCount;
      for (unsigned int j = 0u; j < depthCamera->ImageWidth(); ++j)
      {
        float x = pointCloudData[step + j * pointCloudChannelCount];
        float y = pointCloudData[step + j * pointCloudChannelCount + 1];
        float z = pointCloudData[step + j * pointCloudChannelCount + 2];

        double xd = static_cast<double>(x);
        // depth camera sees only certain percentage of particles
        // so the values should be either
        //   * box depth (depth camera does not see particles), or
        //   * noisy particle depth (depth camera see particles but values
        //     are affected by noise)
        EXPECT_TRUE(
            ignition::math::equal(expectedParticleDepth, xd, depthNoiseTol) ||
            ignition::math::equal(expectedDepth, xd, DEPTH_TOL))
            << "actual vs expected particle depth: "
            << xd << " vs " << expectedParticleDepth;
        float depth = scan[i * depthCamera->ImageWidth() + j];
        double depthd = static_cast<double>(depth);
        EXPECT_TRUE(
            ignition::math::equal(expectedParticleDepth, depthd, depthNoiseTol)
            || ignition::math::equal(expectedDepth, depthd, DEPTH_TOL))
            << "actual vs expected particle depth: "
            << depthd << " vs " << expectedParticleDepth;

        pointParticleLowScatterAvg +=
            ignition::math::Vector3d(x, y, z).Length();
        depthParticleLowScatterAvg += depthd;
      }
    }

    // compare point and depth data before and after setting particle scatter
    // ratio. The avg point length and depth values in the image with low
    // particle scatter ratio should be should be higher than the previous
    // images with particle effects
    pointParticleLowScatterAvg /= pixelCount;
    depthParticleLowScatterAvg /= pixelCount;
    EXPECT_LT(pointParticleAvg, pointParticleLowScatterAvg);
    EXPECT_LT(depthParticleAvg, depthParticleLowScatterAvg);

    // Clean up
    connection.reset();
    delete [] scan;
    if (pointCloudData)
      delete [] pointCloudData;
  }

  engine->DestroyScene(scene);
  ignition::rendering::unloadEngine(engine->Name());
}

TEST_P(DepthCameraTest,
  IGN_UTILS_TEST_DISABLED_ON_WIN32(DepthCameraBoxes))
{
  DepthCameraBoxes(GetParam());
}

TEST_P(DepthCameraTest, DepthCameraParticles)
{
  DepthCameraParticles(GetParam());
}

INSTANTIATE_TEST_CASE_P(DepthCamera, DepthCameraTest,
    RENDER_ENGINE_VALUES, gz::rendering::PrintToStringParam());

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
