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

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/common/Event.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include <gz/math/Color.hh>

#include "test_config.h"  // NOLINT(build/include)

#include "gz/rendering/ParticleEmitter.hh"
#include "gz/rendering/PixelFormat.hh"
#include "gz/rendering/RenderEngine.hh"
#include "gz/rendering/RenderingIface.hh"
#include "gz/rendering/Scene.hh"
#include "gz/rendering/ThermalCamera.hh"

#define DEPTH_TOL 1e-4
#define DOUBLE_TOL 1e-6

unsigned int g_thermalCounter = 0;

//////////////////////////////////////////////////
void OnNewThermalFrame(uint16_t *_scanDest, const uint16_t *_scan,
                  unsigned int _width, unsigned int _height,
                  unsigned int _channels,
                  const std::string &_format)
{
  EXPECT_TRUE(_format == "L16" || _format == "L8");
  EXPECT_EQ(50u, _width);
  EXPECT_EQ(50u, _height);
  EXPECT_EQ(1u, _channels);

  uint16_t u;
  int size =  _width * _height * _channels;
  memcpy(_scanDest, _scan, size * sizeof(u));
}

//////////////////////////////////////////////////
class ThermalCameraTest: public testing::Test,
  public testing::WithParamInterface<const char *>
{
  // Create a Camera sensor from a SDF and gets a image message.
  // If _useHeatSignature is false, uniform surface temperature is tested
  // (if _useHeatSignature is true, applying a heat signature is tested)
  public: void ThermalCameraBoxes(const std::string &_renderEngine,
              const bool _useHeatSignature);

  // Test 8 bit thermal camera output
  public: void ThermalCameraBoxes8Bit(const std::string &_renderEngine);

  // Test that particles do not appear in thermal camera image
  public: void ThermalCameraParticles(const std::string &_renderEngine);

  // Path to test textures
  public: const std::string TEST_MEDIA_PATH =
          ignition::common::joinPaths(std::string(PROJECT_SOURCE_PATH),
                "test", "media", "materials", "textures");

  // Documentation inherited
  protected: void SetUp() override
  {
    gz::common::Console::SetVerbosity(4);
  }
};

//////////////////////////////////////////////////
void ThermalCameraTest::ThermalCameraBoxes(
    const std::string &_renderEngine, const bool _useHeatSignature)
{
  int imgWidth = 50;
  int imgHeight = 50;
  double aspectRatio = imgWidth/imgHeight;

  double unitBoxSize = 1.0;
  gz::math::Vector3d boxPosition(1.8, 0.0, 0.0);

  // Optix is not supported
  if (_renderEngine.compare("optix") == 0)
  {
    igndbg << "Engine '" << _renderEngine
              << "' doesn't support thermal cameras" << std::endl;
    return;
  }
  // Only ogre2 supports heat signatures
  else if (_useHeatSignature && (_renderEngine.compare("ogre2") != 0))
  {
    igndbg << "Engine '" << _renderEngine
              << "' doesn't support heat signatures" << std::endl;
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

  // Create an scene with a box in it
  scene->SetAmbientLight(1.0, 1.0, 1.0);
  gz::rendering::VisualPtr root = scene->RootVisual();

  // create box visual
  gz::rendering::VisualPtr box = scene->CreateVisual();
  box->AddGeometry(scene->CreateBox());
  box->SetOrigin(0.0, 0.0, 0.0);
  box->SetLocalPosition(boxPosition);
  box->SetLocalRotation(0, 0, 0);
  box->SetLocalScale(unitBoxSize, unitBoxSize, unitBoxSize);

  // set box temperature
  float boxTemp = 310.0;
  box->SetUserData("temperature", boxTemp);
  if (_useHeatSignature)
  {
    std::string textureName =
      ignition::common::joinPaths(TEST_MEDIA_PATH, "gray_texture.png");
    box->SetUserData("temperature", textureName);
    box->SetUserData("minTemp", 100.0f);
    box->SetUserData("maxTemp", 200.0f);
    // (the heat signature is just a texture of gray pixels,
    // so the box's temperature should be midway between minTemp and maxTemp)
    boxTemp = 150.0f;
  }

  root->AddChild(box);
  {
    // range is hardcoded in shaders
    float boxTempRange = 3.0;
    double farDist = 10.0;
    double nearDist = 0.15;
    double hfov = 1.05;
    // Create thermal camera
    auto thermalCamera = scene->CreateThermalCamera("ThermalCamera");
    ASSERT_NE(thermalCamera, nullptr);

    gz::math::Pose3d testPose(gz::math::Vector3d(0, 0, 0),
        gz::math::Quaterniond::Identity);
    thermalCamera->SetLocalPose(testPose);

    // Configure thermal camera
    thermalCamera->SetImageWidth(imgWidth);
    EXPECT_EQ(thermalCamera->ImageWidth(),
      static_cast<unsigned int>(imgWidth));
    thermalCamera->SetImageHeight(imgHeight);
    EXPECT_EQ(thermalCamera->ImageHeight(),
      static_cast<unsigned int>(imgHeight));
    thermalCamera->SetFarClipPlane(farDist);
    EXPECT_NEAR(thermalCamera->FarClipPlane(), farDist, DOUBLE_TOL);
    thermalCamera->SetNearClipPlane(nearDist);
    EXPECT_NEAR(thermalCamera->NearClipPlane(), nearDist, DOUBLE_TOL);
    thermalCamera->SetAspectRatio(aspectRatio);
    EXPECT_NEAR(thermalCamera->AspectRatio(), aspectRatio, DOUBLE_TOL);
    thermalCamera->SetHFOV(hfov);
    EXPECT_NEAR(thermalCamera->HFOV().Radian(), hfov, DOUBLE_TOL);

    // thermal-specific params
    // set room temperature: 294 ~ 298 Kelvin
    float ambientTemp = 296.0f;
    float ambientTempRange = 4.0f;
    float linearResolution = 0.01f;
    thermalCamera->SetAmbientTemperature(ambientTemp);
    EXPECT_FLOAT_EQ(ambientTemp, thermalCamera->AmbientTemperature());
    thermalCamera->SetAmbientTemperatureRange(ambientTempRange);
    EXPECT_FLOAT_EQ(ambientTempRange, thermalCamera->AmbientTemperatureRange());
    thermalCamera->SetLinearResolution(linearResolution);
    EXPECT_FLOAT_EQ(linearResolution, thermalCamera->LinearResolution());
    thermalCamera->SetHeatSourceTemperatureRange(boxTempRange);
    EXPECT_FLOAT_EQ(boxTempRange, thermalCamera->HeatSourceTemperatureRange());
    scene->RootVisual()->AddChild(thermalCamera);

    // Set a callback on the  camera sensor to get a thermal camera frame
    uint16_t *thermalData = new uint16_t[imgHeight * imgWidth];
    gz::common::ConnectionPtr connection =
      thermalCamera->ConnectNewThermalFrame(
          std::bind(&::OnNewThermalFrame, thermalData,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
            std::placeholders::_4, std::placeholders::_5));
    EXPECT_NE(nullptr, connection);

    // Update once to create image
    thermalCamera->Update();

    // thermal image indices
    int midWidth = static_cast<int>(thermalCamera->ImageWidth() * 0.5);
    int midHeight = static_cast<int>(thermalCamera->ImageHeight() * 0.5);
    int mid = midHeight * thermalCamera->ImageWidth() + midWidth -1;
    int left = midHeight * thermalCamera->ImageWidth();
    int right = (midHeight+1) * thermalCamera->ImageWidth() - 1;

    // verify temperature
    // Box should be in the middle of image and return box temp
    // Left and right side of the image frame should be ambient temp
    EXPECT_NEAR(ambientTemp, thermalData[left] * linearResolution,
        ambientTempRange);
    EXPECT_NEAR(ambientTemp, thermalData[right] * linearResolution,
        ambientTempRange);
    EXPECT_FLOAT_EQ(thermalData[right], thermalData[left]);
    EXPECT_NEAR(boxTemp, thermalData[mid] * linearResolution, boxTempRange);

    // move box in front of near clip plane and verify the thermal
    // image returns all box temperature values
    gz::math::Vector3d boxPositionNear(
        unitBoxSize * 0.5 + nearDist * 0.5, 0.0, 0.0);
    box->SetLocalPosition(boxPositionNear);
    thermalCamera->Update();

    for (unsigned int i = 0; i < thermalCamera->ImageHeight(); ++i)
    {
      unsigned int step = i * thermalCamera->ImageWidth();
      for (unsigned int j = 0; j < thermalCamera->ImageWidth(); ++j)
      {
        float temp = thermalData[step + j] * linearResolution;
#ifndef __APPLE__
        // https://github.com/ignitionrobotics/ign-rendering/issues/253
        EXPECT_NEAR(boxTemp, temp, boxTempRange);
#endif
      }
    }

    // move box beyond far clip plane and verify the thermal
    // image returns all ambient temperature values
    gz::math::Vector3d boxPositionFar(
        unitBoxSize * 0.5 + farDist * 1.5, 0.0, 0.0);
    box->SetLocalPosition(boxPositionFar);
    thermalCamera->Update();

    for (unsigned int i = 0; i < thermalCamera->ImageHeight(); ++i)
    {
      unsigned int step = i * thermalCamera->ImageWidth();
      for (unsigned int j = 0; j < thermalCamera->ImageWidth(); ++j)
      {
        float temp = thermalData[step + j] * linearResolution;
        EXPECT_NEAR(ambientTemp, temp, ambientTempRange);
      }
    }

    // Clean up
    connection.reset();
    delete [] thermalData;
  }

  engine->DestroyScene(scene);
  gz::rendering::unloadEngine(engine->Name());
}

//////////////////////////////////////////////////
void ThermalCameraTest::ThermalCameraBoxes8Bit(
    const std::string &_renderEngine)
{
  int imgWidth = 50;
  int imgHeight = 50;
  double aspectRatio = imgWidth / imgHeight;

  double unitBoxSize = 1.0;
  ignition::math::Vector3d boxPosition(1.8, 0.0, 0.0);

  // Only ogre2 supports 8 bit image format
  if (_renderEngine.compare("ogre2") != 0)
  {
    igndbg << "Engine '" << _renderEngine
              << "' doesn't support 8 bit thermal cameras" << std::endl;
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

  // Create an scene with a box in it
  scene->SetAmbientLight(1.0, 1.0, 1.0);
  ignition::rendering::VisualPtr root = scene->RootVisual();

  // create box visual
  ignition::rendering::VisualPtr box = scene->CreateVisual();
  box->AddGeometry(scene->CreateBox());
  box->SetOrigin(0.0, 0.0, 0.0);
  box->SetLocalPosition(boxPosition);
  box->SetLocalRotation(0, 0, 0);
  box->SetLocalScale(unitBoxSize, unitBoxSize, unitBoxSize);

  // set box temperature
  float boxTemp = 310.0;
  box->SetUserData("temperature", boxTemp);

  root->AddChild(box);
  {
    // range is hardcoded in shaders
    float boxTempRange = 3.0f;
    double farDist = 10.0;
    double nearDist = 0.15;
    double hfov = 1.05;
    // set min max values based on thermal camera spec
    // using the Vividia HTi HT-301 camera as example:
    // https://hti-instrument.com/products/ht-301-mobile-phone-thermal-imager
    // The range is ~= -20 to 400 degree Celsius
    double minTemp = 253.0;
    double maxTemp = 673.0;
    // Create thermal camera
    auto thermalCamera = scene->CreateThermalCamera("ThermalCamera");
    ASSERT_NE(thermalCamera, nullptr);

    ignition::math::Pose3d testPose(ignition::math::Vector3d(0, 0, 0),
        ignition::math::Quaterniond::Identity);
    thermalCamera->SetLocalPose(testPose);

    // Configure thermal camera
    thermalCamera->SetImageWidth(imgWidth);
    EXPECT_EQ(thermalCamera->ImageWidth(),
      static_cast<unsigned int>(imgWidth));
    thermalCamera->SetImageHeight(imgHeight);
    EXPECT_EQ(thermalCamera->ImageHeight(),
      static_cast<unsigned int>(imgHeight));
    thermalCamera->SetFarClipPlane(farDist);
    EXPECT_NEAR(thermalCamera->FarClipPlane(), farDist, DOUBLE_TOL);
    thermalCamera->SetNearClipPlane(nearDist);
    EXPECT_NEAR(thermalCamera->NearClipPlane(), nearDist, DOUBLE_TOL);
    thermalCamera->SetAspectRatio(aspectRatio);
    EXPECT_NEAR(thermalCamera->AspectRatio(), aspectRatio, DOUBLE_TOL);
    thermalCamera->SetHFOV(hfov);
    EXPECT_NEAR(thermalCamera->HFOV().Radian(), hfov, DOUBLE_TOL);

    // set bit depth
    thermalCamera->SetImageFormat(ignition::rendering::PF_L8);
    EXPECT_EQ(ignition::rendering::PF_L8, thermalCamera->ImageFormat());

    // set min max temp
    thermalCamera->SetMinTemperature(minTemp);
    EXPECT_DOUBLE_EQ(minTemp, thermalCamera->MinTemperature());
    thermalCamera->SetMaxTemperature(maxTemp);
    EXPECT_DOUBLE_EQ(maxTemp, thermalCamera->MaxTemperature());

    // thermal-specific params
    // set room temperature: 294 ~ 298 Kelvin
    float ambientTemp = 296.0f;
    float ambientTempRange = 4.0f;

    // 8 bit format so higher number here (lower resolution)
    // +- 3 degrees
    float linearResolution = 3.0f;
    thermalCamera->SetAmbientTemperature(ambientTemp);
    EXPECT_FLOAT_EQ(ambientTemp, thermalCamera->AmbientTemperature());
    thermalCamera->SetAmbientTemperatureRange(ambientTempRange);
    EXPECT_FLOAT_EQ(ambientTempRange, thermalCamera->AmbientTemperatureRange());
    thermalCamera->SetLinearResolution(linearResolution);
    EXPECT_FLOAT_EQ(linearResolution, thermalCamera->LinearResolution());
    thermalCamera->SetHeatSourceTemperatureRange(boxTempRange);
    EXPECT_FLOAT_EQ(boxTempRange, thermalCamera->HeatSourceTemperatureRange());
    scene->RootVisual()->AddChild(thermalCamera);

    // Set a callback on the camera sensor to get a thermal camera frame
    // todo(anyone) change this to uint8_t when thermal cameras supports a
    // ConnectNewThermalFrame event that provides this format
    uint16_t *thermalData = new uint16_t[imgHeight * imgWidth];
    ignition::common::ConnectionPtr connection =
      thermalCamera->ConnectNewThermalFrame(
          std::bind(&::OnNewThermalFrame, thermalData,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
            std::placeholders::_4, std::placeholders::_5));
    EXPECT_NE(nullptr, connection);

    // Update once to create image
    thermalCamera->Update();

    // thermal image indices
    int midWidth = static_cast<int>(thermalCamera->ImageWidth() * 0.5);
    int midHeight = static_cast<int>(thermalCamera->ImageHeight() * 0.5);
    int mid = midHeight * thermalCamera->ImageWidth() + midWidth -1;
    int left = midHeight * thermalCamera->ImageWidth();
    int right = (midHeight+1) * thermalCamera->ImageWidth() - 1;

    // verify temperature
    // Box should be in the middle of image and return box temp
    // Left and right side of the image frame should be ambient temp
    EXPECT_NEAR(ambientTemp, thermalData[left] * linearResolution,
        ambientTempRange);
    EXPECT_NEAR(ambientTemp, thermalData[right] * linearResolution,
        ambientTempRange);
    EXPECT_FLOAT_EQ(thermalData[right], thermalData[left]);
    EXPECT_NEAR(boxTemp, thermalData[mid] * linearResolution, boxTempRange);

    // move box in front of near clip plane and verify the thermal
    // image returns all box temperature values
    ignition::math::Vector3d boxPositionNear(
        unitBoxSize * 0.5 + nearDist * 0.5, 0.0, 0.0);
    box->SetLocalPosition(boxPositionNear);
    thermalCamera->Update();

    for (unsigned int i = 0; i < thermalCamera->ImageHeight(); ++i)
    {
      unsigned int step = i * thermalCamera->ImageWidth();
      for (unsigned int j = 0; j < thermalCamera->ImageWidth(); ++j)
      {
        float temp = thermalData[step + j] * linearResolution;
        EXPECT_NEAR(boxTemp, temp, boxTempRange);
      }
    }

    // move box beyond far clip plane and verify the thermal
    // image returns all ambient temperature values
    ignition::math::Vector3d boxPositionFar(
        unitBoxSize * 0.5 + farDist * 1.5, 0.0, 0.0);
    box->SetLocalPosition(boxPositionFar);
    thermalCamera->Update();

    for (unsigned int i = 0; i < thermalCamera->ImageHeight(); ++i)
    {
      unsigned int step = i * thermalCamera->ImageWidth();
      for (unsigned int j = 0; j < thermalCamera->ImageWidth(); ++j)
      {
        float temp = thermalData[step + j] * linearResolution;
        EXPECT_NEAR(ambientTemp, temp, ambientTempRange);
      }
    }

    // Clean up
    connection.reset();
    delete [] thermalData;
  }

  engine->DestroyScene(scene);
  ignition::rendering::unloadEngine(engine->Name());
}

//////////////////////////////////////////////////
void ThermalCameraTest::ThermalCameraParticles(
    const std::string &_renderEngine)
{
  int imgWidth = 50;
  int imgHeight = 50;
  double aspectRatio = imgWidth / imgHeight;

  double unitBoxSize = 1.0;
  ignition::math::Vector3d boxPosition(1.8, 0.0, 0.0);

  // Only ogre2 supports 8 bit image format
  if (_renderEngine.compare("ogre2") != 0)
  {
    igndbg << "Engine '" << _renderEngine
           << "' doesn't support 8 bit thermal cameras" << std::endl;
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

  // Create an scene with a box in it
  scene->SetAmbientLight(1.0, 1.0, 1.0);
  ignition::rendering::VisualPtr root = scene->RootVisual();

  // create box visual
  ignition::rendering::VisualPtr box = scene->CreateVisual();
  box->AddGeometry(scene->CreateBox());
  box->SetOrigin(0.0, 0.0, 0.0);
  box->SetLocalPosition(boxPosition);
  box->SetLocalRotation(0, 0, 0);
  box->SetLocalScale(unitBoxSize, unitBoxSize, unitBoxSize);

  // set box temperature
  float boxTemp = 310.0;
  box->SetUserData("temperature", boxTemp);

  root->AddChild(box);

  // create particle emitter between camera and box
  ignition::rendering::ParticleEmitterPtr emitter =
      scene->CreateParticleEmitter();
  emitter->SetLocalPosition({0.5, 0, 0});
  emitter->SetRate(10);
  emitter->SetParticleSize({1, 1, 1});
  emitter->SetLifetime(2);
  emitter->SetVelocityRange(0.1, 0.5);
  emitter->SetColorRange(ignition::math::Color::Red,
      ignition::math::Color::Black);
  emitter->SetScaleRate(1);
  emitter->SetEmitting(true);

  root->AddChild(emitter);

  {
    double farDist = 10.0;
    double nearDist = 0.15;
    double hfov = 1.05;
    // set min max values based on thermal camera spec
    // using the Vividia HTi HT-301 camera as example:
    // https://hti-instrument.com/products/ht-301-mobile-phone-thermal-imager
    // The range is ~= -20 to 400 degree Celsius
    double minTemp = 253.0;
    double maxTemp = 673.0;
    // Create thermal camera
    auto thermalCamera = scene->CreateThermalCamera("ThermalCamera");
    ASSERT_NE(thermalCamera, nullptr);

    ignition::math::Pose3d testPose(ignition::math::Vector3d(0, 0, 0),
        ignition::math::Quaterniond::Identity);
    thermalCamera->SetLocalPose(testPose);

    // Configure thermal camera
    thermalCamera->SetImageWidth(imgWidth);
    EXPECT_EQ(thermalCamera->ImageWidth(),
      static_cast<unsigned int>(imgWidth));
    thermalCamera->SetImageHeight(imgHeight);
    EXPECT_EQ(thermalCamera->ImageHeight(),
      static_cast<unsigned int>(imgHeight));
    thermalCamera->SetFarClipPlane(farDist);
    EXPECT_DOUBLE_EQ(thermalCamera->FarClipPlane(), farDist);
    thermalCamera->SetNearClipPlane(nearDist);
    EXPECT_DOUBLE_EQ(thermalCamera->NearClipPlane(), nearDist);
    thermalCamera->SetAspectRatio(aspectRatio);
    EXPECT_DOUBLE_EQ(thermalCamera->AspectRatio(), aspectRatio);
    thermalCamera->SetHFOV(hfov);
    EXPECT_DOUBLE_EQ(thermalCamera->HFOV().Radian(), hfov);

    // set bit depth
    thermalCamera->SetImageFormat(ignition::rendering::PF_L8);
    EXPECT_EQ(ignition::rendering::PF_L8, thermalCamera->ImageFormat());

    // set min max temp
    thermalCamera->SetMinTemperature(minTemp);
    EXPECT_DOUBLE_EQ(minTemp, thermalCamera->MinTemperature());
    thermalCamera->SetMaxTemperature(maxTemp);
    EXPECT_DOUBLE_EQ(maxTemp, thermalCamera->MaxTemperature());

    // thermal-specific params
    // set room temperature: 294 ~ 298 Kelvin
    float ambientTemp = 296.0f;
    float ambientTempRange = 4.0f;

    // 8 bit format so higher number here (lower resolution)
    // +- 3 degrees
    float linearResolution = 3.0f;
    thermalCamera->SetAmbientTemperature(ambientTemp);
    EXPECT_FLOAT_EQ(ambientTemp, thermalCamera->AmbientTemperature());
    thermalCamera->SetAmbientTemperatureRange(ambientTempRange);
    EXPECT_FLOAT_EQ(ambientTempRange, thermalCamera->AmbientTemperatureRange());
    thermalCamera->SetLinearResolution(linearResolution);
    EXPECT_FLOAT_EQ(linearResolution, thermalCamera->LinearResolution());
    scene->RootVisual()->AddChild(thermalCamera);

    // Set a callback on the camera sensor to get a thermal camera frame
    // todo(anyone) change this to uint8_t when thermal cameras supports a
    // ConnectNewThermalFrame event that provides this format
    uint16_t *thermalData = new uint16_t[imgHeight * imgWidth];
    ignition::common::ConnectionPtr connection =
      thermalCamera->ConnectNewThermalFrame(
          std::bind(&::OnNewThermalFrame, thermalData,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
            std::placeholders::_4, std::placeholders::_5));
    EXPECT_NE(nullptr, connection);

    // thermal image indices
    int midWidth = static_cast<int>(thermalCamera->ImageWidth() * 0.5);
    int midHeight = static_cast<int>(thermalCamera->ImageHeight() * 0.5);
    int mid = midHeight * thermalCamera->ImageWidth() + midWidth -1;
    int left = midHeight * thermalCamera->ImageWidth();
    int right = (midHeight+1) * thermalCamera->ImageWidth() - 1;

    // Update a few times to make sure the flow of particles do not affect
    // the readings
    for (unsigned int i = 0; i < 100u; ++i)
    {
      thermalCamera->Update();

      // verify temperature
      // Box should be in the middle of image and return box temp
      // Left and right side of the image frame should be ambient temp
      EXPECT_NEAR(ambientTemp, thermalData[left] * linearResolution,
          ambientTempRange);
      EXPECT_NEAR(ambientTemp, thermalData[right] * linearResolution,
          ambientTempRange);
      EXPECT_FLOAT_EQ(thermalData[right], thermalData[left]);
      EXPECT_NEAR(boxTemp, thermalData[mid] * linearResolution,
          linearResolution);
    }

    // Clean up
    connection.reset();
    delete [] thermalData;
  }

  engine->DestroyScene(scene);
  ignition::rendering::unloadEngine(engine->Name());
}

// See: https://github.com/gazebosim/gz-rendering/issues/654
TEST_P(ThermalCameraTest,
       IGN_UTILS_TEST_ENABLED_ONLY_ON_LINUX(ThermalCameraBoxesUniformTemp))
{
  ThermalCameraBoxes(GetParam(), false);
}

TEST_P(ThermalCameraTest,
       IGN_UTILS_TEST_DISABLED_ON_WIN32(ThermalCameraBoxesHeatSignature))
{
  ThermalCameraBoxes(GetParam(), true);
}

TEST_P(ThermalCameraTest,
       IGN_UTILS_TEST_DISABLED_ON_WIN32(ThermalCameraBoxesUniformTemp8Bit))
{
  ThermalCameraBoxes8Bit(GetParam());
}

TEST_P(ThermalCameraTest, ThermalCameraParticles)
{
  ThermalCameraParticles(GetParam());
}

INSTANTIATE_TEST_CASE_P(ThermalCamera, ThermalCameraTest,
    RENDER_ENGINE_VALUES, gz::rendering::PrintToStringParam());

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
