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
#include <ignition/common/Image.hh>
#include <ignition/common/Filesystem.hh>

#include "test_config.h"  // NOLINT(build/include)

#include "ignition/rendering/GpuRays.hh"
#include "ignition/rendering/LidarVisual.hh"
#include "ignition/rendering/RenderEngine.hh"
#include "ignition/rendering/RenderingIface.hh"
#include "ignition/rendering/Scene.hh"

#define LASER_TOL 2e-4
#define DOUBLE_TOL 1e-6

// vertical range values seem to be less accurate
#define VERTICAL_LASER_TOL 1e-3

using namespace ignition;
using namespace rendering;

void OnNewGpuRaysFrame(float *_scanDest, const float *_scan,
                  unsigned int _width, unsigned int _height,
                  unsigned int _channels,
                  const std::string &/*_format*/)
{
  float f;
  int size =  _width * _height * _channels;
  memcpy(_scanDest, _scan, size * sizeof(f));
}

class LidarVisualTest: public testing::Test,
                  public testing::WithParamInterface<const char *>
{
  // Test and verify lidar visual properties setters and getters
  public: void Configure(const std::string &_renderEngine);

  // Test boxes detection
  public: void RaysUnitBox(const std::string &_renderEngine);

  // Test vertical measurements
  public: void LaserVertical(const std::string &_renderEngine);
};

/////////////////////////////////////////////////
/// \brief Test LidarVisual configuraions
void LidarVisualTest::Configure(const std::string &_renderEngine)
{
  if (_renderEngine == "optix")
  {
    igndbg << "LidarVisual not supported yet in rendering engine: "
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

  LidarVisualPtr lidarVis = scene->CreateLidarVisual();
  ASSERT_TRUE(lidarVis != nullptr);
  root->AddChild(lidarVis);

  // set initial pose for lidar
  math::Vector3d initPos(-2, 0.0, 5.0);
  math::Quaterniond initRot = math::Quaterniond::Identity;
  lidarVis->SetWorldPosition(initPos);
  EXPECT_EQ(initPos, lidarVis->WorldPosition());
  EXPECT_EQ(initRot, lidarVis->WorldRotation());

  // The following tests all the getters and setters
  {
    lidarVis->SetMinVerticalAngle(0.1);
    EXPECT_DOUBLE_EQ(lidarVis->MinVerticalAngle(), 0.1);

    lidarVis->SetMaxVerticalAngle(251.20);
    EXPECT_DOUBLE_EQ(lidarVis->MaxVerticalAngle(), 251.20);

    lidarVis->SetMinRange(0.05);
    EXPECT_DOUBLE_EQ(lidarVis->MinRange(), 0.05);

    lidarVis->SetMaxRange(105.20);
    EXPECT_DOUBLE_EQ(lidarVis->MaxRange(), 105.20);

    lidarVis->SetMinHorizontalAngle(-1.5136);
    EXPECT_DOUBLE_EQ(lidarVis->MinHorizontalAngle(), -1.5136);

    lidarVis->SetMaxHorizontalAngle(2.513);
    EXPECT_DOUBLE_EQ(lidarVis->MaxHorizontalAngle(), 2.513);

    lidarVis->SetHorizontalRayCount(360);
    EXPECT_EQ(lidarVis->HorizontalRayCount(), 360u);

    lidarVis->SetVerticalRayCount(30);
    EXPECT_EQ(lidarVis->VerticalRayCount(), 30u);

    ignition::math::Pose3d offset(1.5, 3.6, 2.9, 1.1, -5.3, -2.9);
    lidarVis->SetOffset(offset);
    EXPECT_EQ(lidarVis->Offset(), offset);

    lidarVis->SetType(LVT_NONE);
    EXPECT_EQ(lidarVis->Type(), LVT_NONE);
    lidarVis->SetType(LVT_POINTS);
    EXPECT_EQ(lidarVis->Type(), LVT_POINTS);
    lidarVis->SetType(LVT_POINTS);
    EXPECT_EQ(lidarVis->Type(), LVT_POINTS);
    lidarVis->SetType(LVT_TRIANGLE_STRIPS);
    EXPECT_EQ(lidarVis->Type(), LVT_TRIANGLE_STRIPS);

    lidarVis->SetDisplayNonHitting(true);
    EXPECT_EQ(lidarVis->DisplayNonHitting(), true);
    lidarVis->SetDisplayNonHitting(false);
    EXPECT_EQ(lidarVis->DisplayNonHitting(), false);

    std::vector<double> pts = {2, 14, 15, 3, 5, 10, 3};
    lidarVis->SetPoints(pts);
    EXPECT_EQ(lidarVis->PointCount(), pts.size());
    lidarVis->ClearPoints();
    EXPECT_EQ(lidarVis->PointCount(), 0u);
  }

  // Clean up
  engine->DestroyScene(scene);
  rendering::unloadEngine(engine->Name());
}


/////////////////////////////////////////////////
/// \brief Test detection of different boxes
void LidarVisualTest::RaysUnitBox(const std::string &_renderEngine)
{
#ifdef __APPLE__
  std::cerr << "Skipping test for apple, see issue #35." << std::endl;
  return;
#endif

  if (_renderEngine == "optix")
  {
    igndbg << "LidarVisual not supported yet in rendering engine: "
            << _renderEngine << std::endl;
    return;
  }

  // Test lidar visual with 3 boxes in the world, using reading from GPU rays
  // First GPU rays at identity orientation, second at 90 degree roll
  // First place 2 of 3 boxes within range and verify range values from lidar.
  // then move all 3 boxes out of range and verify range values from lidar

  const double hMinAngle = -IGN_PI/2.0;
  const double hMaxAngle = IGN_PI/2.0;
  const double minRange = 0.1;
  const double maxRange = 10.0;
  const int hRayCount = 320;
  const int vRayCount = 1;

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

  // Create first ray caster
  ignition::math::Pose3d testPose(ignition::math::Vector3d(0, 0, 0.1),
      ignition::math::Quaterniond::Identity);

  GpuRaysPtr gpuRays = scene->CreateGpuRays("gpu_rays_1");
  gpuRays->SetWorldPosition(testPose.Pos());
  gpuRays->SetWorldRotation(testPose.Rot());
  gpuRays->SetNearClipPlane(minRange);
  gpuRays->SetFarClipPlane(maxRange);
  gpuRays->SetAngleMin(hMinAngle);
  gpuRays->SetAngleMax(hMaxAngle);
  gpuRays->SetRayCount(hRayCount);
  gpuRays->SetVerticalRayCount(vRayCount);
  root->AddChild(gpuRays);

  // Create first lidar
  LidarVisualPtr lidarVis = scene->CreateLidarVisual();
  lidarVis->SetWorldPosition(testPose.Pos());
  lidarVis->SetWorldRotation(testPose.Rot());
  lidarVis->SetMinRange(minRange);
  lidarVis->SetMaxRange(maxRange);
  lidarVis->SetMinHorizontalAngle(hMinAngle);
  lidarVis->SetMaxHorizontalAngle(hMaxAngle);
  lidarVis->SetHorizontalRayCount(hRayCount);
  lidarVis->SetVerticalRayCount(vRayCount);
  lidarVis->SetType(LidarVisualType::LVT_TRIANGLE_STRIPS);
  lidarVis->SetDisplayNonHitting(true);
  root->AddChild(lidarVis);

  // Create a second ray caster rotated
  ignition::math::Pose3d testPose2(ignition::math::Vector3d(0, 0, 0.1),
      ignition::math::Quaterniond(IGN_PI/2.0, 0, 0));

  GpuRaysPtr gpuRays2 = scene->CreateGpuRays("gpu_rays_2");
  gpuRays2->SetWorldPosition(testPose2.Pos());
  gpuRays2->SetWorldRotation(testPose2.Rot());
  gpuRays2->SetNearClipPlane(minRange);
  gpuRays2->SetFarClipPlane(maxRange);
  gpuRays2->SetClamp(true);
  gpuRays2->SetAngleMin(hMinAngle);
  gpuRays2->SetAngleMax(hMaxAngle);
  gpuRays2->SetRayCount(hRayCount);
  gpuRays2->SetVerticalRayCount(vRayCount);
  root->AddChild(gpuRays2);

  // Create a second lidar rotated
  LidarVisualPtr lidarVis2 = scene->CreateLidarVisual();
  lidarVis2->SetWorldPosition(testPose2.Pos());
  lidarVis2->SetWorldRotation(testPose2.Rot());
  lidarVis2->SetMinRange(minRange);
  lidarVis2->SetMaxRange(maxRange);
  lidarVis2->SetMinHorizontalAngle(hMinAngle);
  lidarVis2->SetMaxHorizontalAngle(hMaxAngle);
  lidarVis2->SetHorizontalRayCount(hRayCount);
  lidarVis2->SetVerticalRayCount(vRayCount);
  lidarVis2->SetType(LidarVisualType::LVT_TRIANGLE_STRIPS);
  lidarVis2->SetDisplayNonHitting(true);
  root->AddChild(lidarVis2);

  // Create testing boxes
  // box in the center
  ignition::math::Pose3d box01Pose(ignition::math::Vector3d(3, 0, 0.5),
                                   ignition::math::Quaterniond::Identity);
  VisualPtr visualBox1 = scene->CreateVisual("UnitBox1");
  visualBox1->AddGeometry(scene->CreateBox());
  visualBox1->SetWorldPosition(box01Pose.Pos());
  visualBox1->SetWorldRotation(box01Pose.Rot());
  root->AddChild(visualBox1);

  // box on the right of the first gpu rays caster
  ignition::math::Pose3d box02Pose(ignition::math::Vector3d(0, -5, 0.5),
                                   ignition::math::Quaterniond::Identity);
  VisualPtr visualBox2 = scene->CreateVisual("UnitBox2");
  visualBox2->AddGeometry(scene->CreateBox());
  visualBox2->SetWorldPosition(box02Pose.Pos());
  visualBox2->SetWorldRotation(box02Pose.Rot());
  root->AddChild(visualBox2);

  // box on the left of the rays caster 1 but out of range
  ignition::math::Pose3d box03Pose(
      ignition::math::Vector3d(0, maxRange + 1, 0.5),
      ignition::math::Quaterniond::Identity);
  VisualPtr visualBox3 = scene->CreateVisual("UnitBox3");
  visualBox3->AddGeometry(scene->CreateBox());
  visualBox3->SetWorldPosition(box03Pose.Pos());
  visualBox3->SetWorldRotation(box03Pose.Rot());
  root->AddChild(visualBox3);

  // Verify rays caster 1 range readings
  // listen to new gpu rays frames
  unsigned int channels = gpuRays->Channels();
  float *scan = new float[hRayCount * vRayCount * channels];
  common::ConnectionPtr c =
    gpuRays->ConnectNewGpuRaysFrame(
        std::bind(&::OnNewGpuRaysFrame, scan,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));

  gpuRays->Update();

  std::vector<double> pts;
  for (int j = 0; j < vRayCount; j++)
  {
    for (int i = 0; i < gpuRays->RayCount(); ++i)
    {
      pts.push_back(scan[j*channels*gpuRays->RayCount() + i * channels]);
    }
  }
  lidarVis->SetPoints(pts);
  lidarVis->Update();

  // receive points from lidar to check values
  std::vector<double> pts_back = lidarVis->Points();

  int mid = static_cast<int>(hRayCount/2);
  int last = (hRayCount - 1);
  double unitBoxSize = 1.0;
  double expectedRangeAtMidPointBox1 = abs(box01Pose.Pos().X()) - unitBoxSize/2;
  double expectedRangeAtMidPointBox2 = abs(box02Pose.Pos().Y()) - unitBoxSize/2;

  // rays caster values received from lidar should see box01 and box02
  EXPECT_NEAR(pts_back[mid], expectedRangeAtMidPointBox1, LASER_TOL);
  EXPECT_NEAR(pts_back[0], expectedRangeAtMidPointBox2, LASER_TOL);
  EXPECT_FLOAT_EQ(pts_back[last], ignition::math::INF_F);

  // Verify rays caster 2 range readings
  // listen to new gpu rays frames
  float *scan2 = new float[hRayCount * vRayCount * 3];

  gpuRays2->Update();
  // Test Copy method instead of using the callback for the second rays caster
  gpuRays2->Copy(scan2);

  // copy ray caster values to lidar data
  std::vector<double> pts2;
  for (int j = 0; j < vRayCount; j++)
  {
    for (int i = 0; i < gpuRays->RayCount(); ++i)
    {
      pts2.push_back(scan2[j*channels*gpuRays->RayCount() + i * channels]);
    }
  }
  lidarVis2->SetPoints(pts2);
  lidarVis2->Update();
  std::vector<double> pts_back2 = lidarVis2->Points();

  // Only box01 should be visible to rays caster 2
  EXPECT_FLOAT_EQ(pts_back2[0], maxRange);
  EXPECT_NEAR(pts_back2[mid], expectedRangeAtMidPointBox1, LASER_TOL);
  EXPECT_FLOAT_EQ(pts_back2[last], maxRange);

  // Move all boxes out of range
  visualBox1->SetWorldPosition(
      ignition::math::Vector3d(maxRange + 1, 0, 0));
  visualBox1->SetWorldRotation(box01Pose.Rot());
  visualBox2->SetWorldPosition(
      ignition::math::Vector3d(0, -(maxRange + 1), 0));
  visualBox2->SetWorldRotation(box02Pose.Rot());

  gpuRays->Update();
  gpuRays2->Update();
  gpuRays2->Copy(scan2);

  pts.clear();
  for (int j = 0; j < vRayCount; j++)
  {
    for (int i = 0; i < gpuRays->RayCount(); ++i)
    {
      pts.push_back(scan[j*channels*gpuRays->RayCount() + i * channels]);
    }
  }

  pts2.clear();
  for (int j = 0; j < vRayCount; j++)
  {
    for (int i = 0; i < gpuRays->RayCount(); ++i)
    {
      pts2.push_back(scan2[j*channels*gpuRays->RayCount() + i * channels]);
    }
  }

  for (unsigned int i = 0; i < lidarVis->HorizontalRayCount(); ++i)
    EXPECT_FLOAT_EQ(pts[i], ignition::math::INF_F);

  for (unsigned int i = 0; i < lidarVis->HorizontalRayCount(); ++i)
    EXPECT_FLOAT_EQ(pts2[i], maxRange);

  c.reset();

  delete [] scan;
  delete [] scan2;

  scan = nullptr;
  scan2 = nullptr;

  // Clean up
  engine->DestroyScene(scene);
  rendering::unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
/// \brief Test GPU rays vertical component
void LidarVisualTest::LaserVertical(const std::string &_renderEngine)
{
#ifdef __APPLE__
  std::cerr << "Skipping test for apple, see issue #35." << std::endl;
  return;
#endif

  if (_renderEngine == "optix")
  {
    igndbg << "LidarVisual not supported yet in rendering engine: "
            << _renderEngine << std::endl;
    return;
  }

  // Test a rays that has a vertical range component.
  // Place a box within range and verify range values,
  // then move the box out of range and verify range values

  double hMinAngle = -IGN_PI/2.0;
  double hMaxAngle = IGN_PI/2.0;
  double vMinAngle = -IGN_PI/4.0;
  double vMaxAngle = IGN_PI/4.0;
  double minRange = 0.1;
  double maxRange = 5.0;
  unsigned int hRayCount = 640;
  unsigned int vRayCount = 4;

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

  // Create first ray caster
  ignition::math::Pose3d testPose(ignition::math::Vector3d(0.25, 0, 0.5),
      ignition::math::Quaterniond::Identity);

  GpuRaysPtr gpuRays = scene->CreateGpuRays("vertical_gpu_rays");
  gpuRays->SetWorldPosition(testPose.Pos());
  gpuRays->SetWorldRotation(testPose.Rot());
  gpuRays->SetNearClipPlane(minRange);
  gpuRays->SetFarClipPlane(maxRange);
  gpuRays->SetAngleMin(hMinAngle);
  gpuRays->SetAngleMax(hMaxAngle);
  gpuRays->SetVerticalAngleMin(vMinAngle);
  gpuRays->SetVerticalAngleMax(vMaxAngle);
  gpuRays->SetRayCount(hRayCount);
  gpuRays->SetVerticalRayCount(vRayCount);
  root->AddChild(gpuRays);

  LidarVisualPtr lidarVis = scene->CreateLidarVisual();
  lidarVis->SetWorldPosition(testPose.Pos());
  lidarVis->SetWorldRotation(testPose.Rot());
  lidarVis->SetMinRange(minRange);
  lidarVis->SetMaxRange(maxRange);
  lidarVis->SetMinHorizontalAngle(hMinAngle);
  lidarVis->SetMaxHorizontalAngle(hMaxAngle);
  lidarVis->SetHorizontalRayCount(hRayCount);
  lidarVis->SetVerticalRayCount(vRayCount);
  lidarVis->SetType(LidarVisualType::LVT_TRIANGLE_STRIPS);
  lidarVis->SetDisplayNonHitting(true);
  root->AddChild(lidarVis);

  // Create testing boxes
  // box in front of ray sensor
  ignition::math::Pose3d box01Pose(ignition::math::Vector3d(1, 0, 0.5),
      ignition::math::Quaterniond::Identity);
  VisualPtr visualBox1 = scene->CreateVisual("VerticalTestBox1");
  visualBox1->AddGeometry(scene->CreateBox());
  visualBox1->SetWorldPosition(box01Pose.Pos());
  visualBox1->SetWorldRotation(box01Pose.Rot());
  root->AddChild(visualBox1);

  unsigned int channels = gpuRays->Channels();
  float *scan = new float[hRayCount * vRayCount * channels];
  common::ConnectionPtr c =
    gpuRays->ConnectNewGpuRaysFrame(
        std::bind(&::OnNewGpuRaysFrame, scan,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));

  gpuRays->Update();
  std::vector<double> pts;
  for (unsigned int j = 0; j < vRayCount; j++)
  {
    for (int i = 0; i < gpuRays->RayCount(); ++i)
    {
      pts.push_back(scan[j*channels*gpuRays->RayCount() + i * channels]);
    }
  }
  lidarVis->SetPoints(pts);
  lidarVis->Update();

  unsigned int mid = hRayCount / 2;
  double unitBoxSize = 1.0;
  double expectedRangeAtMidPoint = box01Pose.Pos().X() - unitBoxSize/2
      - testPose.Pos().X();

  double vAngleStep = (vMaxAngle - vMinAngle) / (vRayCount-1);
  double angleStep = vMinAngle;

  // all vertical laser planes should sense box
  for (unsigned int i = 0; i < vRayCount; ++i)
  {
    double expectedRange = expectedRangeAtMidPoint / cos(angleStep);

    EXPECT_NEAR(pts[i * hRayCount + mid],
        expectedRange, VERTICAL_LASER_TOL);

    angleStep += vAngleStep;

    // check that the values in the extremes are infinity
    EXPECT_FLOAT_EQ(pts[i * hRayCount ],
        ignition::math::INF_F);
    EXPECT_FLOAT_EQ(pts[(i * hRayCount + (hRayCount - 1))],
        ignition::math::INF_F);
  }

  // Move box out of range
  visualBox1->SetWorldPosition(
      ignition::math::Vector3d(maxRange + 1, 0, 0));
  visualBox1->SetWorldRotation(
      ignition::math::Quaterniond::Identity);

  // after a few more laser scans
  gpuRays->Update();
  pts.clear();
  for (unsigned int j = 0; j < vRayCount; j++)
  {
    for (int i = 0; i < gpuRays->RayCount(); ++i)
    {
      pts.push_back(scan[j*channels*gpuRays->RayCount() + i * channels]);
    }
  }
  lidarVis->SetPoints(pts);
  lidarVis->Update();


  for (unsigned int j = 0; j < lidarVis->VerticalRayCount(); ++j)
  {
    for (unsigned int i = 0; i < lidarVis->HorizontalRayCount(); ++i)
    {
      EXPECT_FLOAT_EQ(pts[j * lidarVis->HorizontalRayCount()+ i],
          ignition::math::INF_F);
    }
  }

  c.reset();

  delete [] scan;
  scan = nullptr;

  // Clean up
  engine->DestroyScene(scene);
  rendering::unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
TEST_P(LidarVisualTest, Configure)
{
  Configure(GetParam());
}

/////////////////////////////////////////////////
TEST_P(LidarVisualTest, RaysUnitBox)
{
  RaysUnitBox(GetParam());
}

/////////////////////////////////////////////////
TEST_P(LidarVisualTest, LaserVertical)
{
  LaserVertical(GetParam());
}

INSTANTIATE_TEST_CASE_P(LidarVisual, LidarVisualTest,
    RENDER_ENGINE_VALUES,
    ignition::rendering::PrintToStringParam());

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
