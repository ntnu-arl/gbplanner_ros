/* * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "gz/rendering/Camera.hh"
#include "gz/rendering/RayQuery.hh"
#include "gz/rendering/RenderEngine.hh"
#include "gz/rendering/RenderingIface.hh"
#include "gz/rendering/Scene.hh"
#include "gz/rendering/Utils.hh"
#include "gz/rendering/Visual.hh"

#include "test_config.h"  // NOLINT(build/include)

using namespace gz;
using namespace rendering;

class UtilTest : public testing::Test,
                 public testing::WithParamInterface<const char *>
{
  // Documentation inherited
  public: void SetUp() override
  {
    common::Console::SetVerbosity(4);
  }

  public: void ClickToScene(const std::string &_renderEngine);
};

void UtilTest::ClickToScene(const std::string &_renderEngine)
{
  RenderEngine *engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    FAIL() << "Engine '" << _renderEngine
           << "' is not supported" << std::endl;
    return;
  }
  ScenePtr scene = engine->CreateScene("scene");

  CameraPtr camera(scene->CreateCamera());
  EXPECT_TRUE(camera != nullptr);

  camera->SetLocalPosition(0.0, 0.0, 15);
  camera->SetLocalRotation(0.0, IGN_PI / 2, 0.0);

  unsigned int width = 640u;
  unsigned int height = 480u;
  camera->SetImageWidth(width);
  camera->SetImageHeight(height);

  const int halfWidth  = static_cast<int>(width / 2);
  const int halfHeight = static_cast<int>(height / 2);
  math::Vector2i centerClick(halfWidth, halfHeight);

  RayQueryPtr rayQuery = scene->CreateRayQuery();
  EXPECT_TRUE(rayQuery != nullptr);

  // screenToPlane
  math::Vector3d result = screenToPlane(centerClick, camera, rayQuery);

  EXPECT_NEAR(0.0, result.Z(), 1e-10);
  EXPECT_NEAR(0.0, result.X(), 2e-6);
  EXPECT_NEAR(0.0, result.Y(), 2e-6);

  // call with non-zero plane offset
  result = screenToPlane(centerClick, camera, rayQuery, 5.0);

  EXPECT_NEAR(5.0, result.Z(), 1e-10);
  EXPECT_NEAR(0.0, result.X(), 2e-6);
  EXPECT_NEAR(0.0, result.Y(), 2e-6);

  // screenToScene
  // API without RayQueryResult and default max distance
  result = screenToScene(centerClick, camera, rayQuery);

  // No objects currently in the scene, so return a point max distance in
  // front of camera
  // The default max distance is 10 meters away
  EXPECT_NEAR(5.0 - camera->NearClipPlane(), result.Z(), 4e-6);
  EXPECT_NEAR(0.0, result.X(), 2e-6);
  EXPECT_NEAR(0.0, result.Y(), 2e-6);

  // Try with different max distance
  RayQueryResult rayResult;
  result = screenToScene(centerClick, camera, rayQuery, rayResult, 20.0);

  EXPECT_NEAR(-5.0 - camera->NearClipPlane(), result.Z(), 4e-6);
  EXPECT_NEAR(0.0, result.X(), 4e-6);
  EXPECT_NEAR(0.0, result.Y(), 4e-6);
  EXPECT_FALSE(rayResult);
  EXPECT_EQ(0u, rayResult.objectId);

  VisualPtr root = scene->RootVisual();

  // create box visual to collide with the ray
  VisualPtr box = scene->CreateVisual();
  box->AddGeometry(scene->CreateBox());
  box->SetOrigin(0.0, 0.0, 0.0);
  box->SetLocalPosition(0.0, 0.0, 0.0);
  box->SetLocalRotation(0.0, 0.0, 0.0);
  box->SetLocalScale(1.0, 1.0, 1.0);
  root->AddChild(box);

  // add camera and render one frame
  root->AddChild(camera);
  camera->Update();

  // \todo(anyone)
  // the centerClick var above is set to a screen pos of (width/2, height/2).
  // This is off-by-1. The actual center pos should be at
  // (width/2 - 1, height/2 - 1) so the result.X() and result.Y() is a bit off
  // from the expected position. However, fixing the centerClick above caused
  // the screenToPlane tests to fail so only modifying the pos here, and the
  // cause of test failure need to be investigated.
  if (_renderEngine == "ogre2")
    centerClick = ignition::math::Vector2i(halfWidth-1, halfHeight-1);

  // API without RayQueryResult and default max distance
  result = screenToScene(centerClick, camera, rayQuery, rayResult);

  // high tol is used for z due to depth buffer precision.
  // Do not merge the tol changes forward to ign-rendering6.
  EXPECT_NEAR(0.5, result.Z(), 1e-3);
  EXPECT_NEAR(0.0, result.X(), 2e-6);
  EXPECT_NEAR(0.0, result.Y(), 2e-6);
  EXPECT_TRUE(rayResult);
  EXPECT_NEAR(14.5 - camera->NearClipPlane(), rayResult.distance, 1e-3);
  EXPECT_EQ(box->Id(), rayResult.objectId);

  result = screenToScene(centerClick, camera, rayQuery, rayResult, 20.0);

  EXPECT_NEAR(0.5, result.Z(), 1e-3);
  EXPECT_NEAR(0.0, result.X(), 2e-6);
  EXPECT_NEAR(0.0, result.Y(), 2e-6);
  EXPECT_TRUE(rayResult);
  EXPECT_NEAR(14.5 - camera->NearClipPlane(), rayResult.distance, 1e-3);
  EXPECT_EQ(box->Id(), rayResult.objectId);

  // Move camera closer to box
  camera->SetLocalPosition(0.0, 0.0, 7.0);
  camera->SetLocalRotation(0.0, IGN_PI / 2, 0.0);

  result = screenToScene(centerClick, camera, rayQuery, rayResult);

  EXPECT_NEAR(0.5, result.Z(), 1e-3);
  EXPECT_NEAR(0.0, result.X(), 2e-6);
  EXPECT_NEAR(0.0, result.Y(), 2e-6);
  EXPECT_TRUE(rayResult);
  EXPECT_NEAR(6.5 - camera->NearClipPlane(), rayResult.distance, 1e-4);
  EXPECT_EQ(box->Id(), rayResult.objectId);
}

/////////////////////////////////////////////////
TEST_P(UtilTest, ClickToScene)
{
  ClickToScene(GetParam());
}

INSTANTIATE_TEST_CASE_P(ClickToScene, UtilTest,
    RENDER_ENGINE_VALUES,
    PrintToStringParam());

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
