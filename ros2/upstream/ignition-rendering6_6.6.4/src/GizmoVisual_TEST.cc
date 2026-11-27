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
#include <string>

#include <gz/common/Console.hh>

#include "test_config.h"  // NOLINT(build/include)

#include "gz/rendering/GizmoVisual.hh"
#include "gz/rendering/RenderEngine.hh"
#include "gz/rendering/RenderingIface.hh"
#include "gz/rendering/Scene.hh"

using namespace gz;
using namespace rendering;

class GizmoVisualTest : public testing::Test,
                        public testing::WithParamInterface<const char *>
{
  /// \brief Test basic API
  public: void GizmoVisual(const std::string &_renderEngine);

  /// \brief Test gizmo material
  public: void Material(const std::string &_renderEngine);
};

/////////////////////////////////////////////////
void GizmoVisualTest::GizmoVisual(const std::string &_renderEngine)
{
  RenderEngine *engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    FAIL() << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }

  ScenePtr scene = engine->CreateScene("scene");

  // create visual
  GizmoVisualPtr gizmo = scene->CreateGizmoVisual();
  ASSERT_NE(nullptr, gizmo);

  // check initial values
  EXPECT_EQ(TransformMode::TM_NONE, gizmo->Mode());
  EXPECT_EQ(math::Vector3d::Zero, gizmo->ActiveAxis());

  // test setting mode
  gizmo->SetTransformMode(TransformMode::TM_ROTATION);
  EXPECT_EQ(TransformMode::TM_ROTATION, gizmo->Mode());

  // test setting active axis
  gizmo->SetActiveAxis(math::Vector3d::UnitZ);
  EXPECT_EQ(math::Vector3d::UnitZ, gizmo->ActiveAxis());

  // verify all axis visuals exist
  VisualPtr xtrans = gizmo->ChildByAxis(TransformAxis::TA_TRANSLATION_X);
  EXPECT_NE(nullptr, xtrans);
  EXPECT_EQ(TransformAxis::TA_TRANSLATION_X, gizmo->AxisById(xtrans->Id()));

  VisualPtr ytrans = gizmo->ChildByAxis(TransformAxis::TA_TRANSLATION_Y);
  EXPECT_NE(nullptr, ytrans);
  EXPECT_EQ(TransformAxis::TA_TRANSLATION_Y, gizmo->AxisById(ytrans->Id()));

  VisualPtr ztrans = gizmo->ChildByAxis(TransformAxis::TA_TRANSLATION_Z);
  EXPECT_NE(nullptr, ztrans);
  EXPECT_EQ(TransformAxis::TA_TRANSLATION_Z, gizmo->AxisById(ztrans->Id()));

  VisualPtr xrot = gizmo->ChildByAxis(TransformAxis::TA_ROTATION_X);
  EXPECT_NE(nullptr, xrot);
  EXPECT_EQ(TransformAxis::TA_ROTATION_X, gizmo->AxisById(xrot->Id()));

  VisualPtr yrot = gizmo->ChildByAxis(TransformAxis::TA_ROTATION_Y);
  EXPECT_NE(nullptr, yrot);
  EXPECT_EQ(TransformAxis::TA_ROTATION_Y, gizmo->AxisById(yrot->Id()));

  VisualPtr zrot = gizmo->ChildByAxis(TransformAxis::TA_ROTATION_Z);
  EXPECT_NE(nullptr, zrot);
  EXPECT_EQ(TransformAxis::TA_ROTATION_Z, gizmo->AxisById(zrot->Id()));

  VisualPtr xscale = gizmo->ChildByAxis(TransformAxis::TA_SCALE_X);
  EXPECT_NE(nullptr, xscale);
  EXPECT_EQ(TransformAxis::TA_SCALE_X, gizmo->AxisById(xscale->Id()));

  VisualPtr yscale = gizmo->ChildByAxis(TransformAxis::TA_SCALE_Y);
  EXPECT_NE(nullptr, yscale);
  EXPECT_EQ(TransformAxis::TA_SCALE_Y, gizmo->AxisById(yscale->Id()));

  VisualPtr zscale = gizmo->ChildByAxis(TransformAxis::TA_SCALE_Z);
  EXPECT_NE(nullptr, zscale);
  EXPECT_EQ(TransformAxis::TA_SCALE_Z, gizmo->AxisById(zscale->Id()));

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
void GizmoVisualTest::Material(const std::string &_renderEngine)
{
  RenderEngine *engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    FAIL() << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }

  ScenePtr scene = engine->CreateScene("scene");

  // create visual
  GizmoVisualPtr gizmo = scene->CreateGizmoVisual();
  ASSERT_NE(nullptr, gizmo);

  // get all original axis materials
  VisualPtr xtrans = gizmo->ChildByAxis(TransformAxis::TA_TRANSLATION_X);
  EXPECT_NE(nullptr, xtrans);
  EXPECT_EQ(TransformAxis::TA_TRANSLATION_X, gizmo->AxisById(xtrans->Id()));
  MaterialPtr xMat = xtrans->Material();
  EXPECT_NE(nullptr, xMat);

  VisualPtr ytrans = gizmo->ChildByAxis(TransformAxis::TA_TRANSLATION_Y);
  EXPECT_NE(nullptr, ytrans);
  EXPECT_EQ(TransformAxis::TA_TRANSLATION_Y, gizmo->AxisById(ytrans->Id()));
  MaterialPtr yMat = ytrans->Material();
  EXPECT_NE(nullptr, yMat);

  VisualPtr ztrans = gizmo->ChildByAxis(TransformAxis::TA_TRANSLATION_Z);
  EXPECT_NE(nullptr, ztrans);
  EXPECT_EQ(TransformAxis::TA_TRANSLATION_Z, gizmo->AxisById(ztrans->Id()));
  MaterialPtr zMat = ztrans->Material();
  EXPECT_NE(nullptr, zMat);

  // set mode and active axis
  gizmo->SetTransformMode(TransformMode::TM_TRANSLATION);
  EXPECT_EQ(TransformMode::TM_TRANSLATION, gizmo->Mode());
  gizmo->SetActiveAxis(math::Vector3d::UnitZ);
  EXPECT_EQ(math::Vector3d::UnitZ, gizmo->ActiveAxis());

  // pre-render to verify that material of active axis changed.
  gizmo->PreRender();
  MaterialPtr xMat2 = xtrans->Material();
  MaterialPtr yMat2 = ytrans->Material();
  MaterialPtr zMat2 = ztrans->Material();
  EXPECT_EQ(xMat, xMat2);
  EXPECT_EQ(yMat, yMat2);
  EXPECT_NE(zMat, zMat2);

  MaterialPtr activeMat = zMat2;

  // set different active axis
  gizmo->SetActiveAxis(math::Vector3d::UnitX);
  EXPECT_EQ(math::Vector3d::UnitX, gizmo->ActiveAxis());

  // pre-render to verify that material of active axis.
  gizmo->PreRender();
  MaterialPtr xMat3 = xtrans->Material();
  MaterialPtr yMat3 = ytrans->Material();
  MaterialPtr zMat3 = ztrans->Material();
  EXPECT_NE(xMat, xMat3);
  EXPECT_EQ(yMat, yMat3);
  EXPECT_EQ(zMat, zMat3);

  // the active axis material should be the same as before
  EXPECT_EQ(activeMat, xMat3);

  // reset mode and verify all axes now have the original materials
  gizmo->SetTransformMode(TransformMode::TM_NONE);
  EXPECT_EQ(TransformMode::TM_NONE, gizmo->Mode());
  gizmo->PreRender();
  MaterialPtr xMat4 = xtrans->Material();
  MaterialPtr yMat4 = ytrans->Material();
  MaterialPtr zMat4 = ztrans->Material();
  EXPECT_EQ(xMat, xMat4);
  EXPECT_EQ(yMat, yMat4);
  EXPECT_EQ(zMat, zMat4);

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
TEST_P(GizmoVisualTest, GizmoVisual)
{
  GizmoVisual(GetParam());
}

/////////////////////////////////////////////////
TEST_P(GizmoVisualTest, Material)
{
  Material(GetParam());
}

INSTANTIATE_TEST_CASE_P(Visual, GizmoVisualTest,
    RENDER_ENGINE_VALUES,
    PrintToStringParam());

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
