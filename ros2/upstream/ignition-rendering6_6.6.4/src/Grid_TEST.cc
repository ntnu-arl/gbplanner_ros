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
#include "gz/rendering/RenderEngine.hh"
#include "gz/rendering/RenderingIface.hh"
#include "gz/rendering/Grid.hh"
#include "gz/rendering/Scene.hh"

using namespace gz;
using namespace rendering;

class GridTest : public testing::Test,
                 public testing::WithParamInterface<const char *>
{
  public: void Grid(const std::string &_renderEngine);
};

/////////////////////////////////////////////////
void GridTest::Grid(const std::string &_renderEngine)
{
#ifdef __APPLE__                                                                              
  std::cerr << "Skipping test for apple, see issue #847." << std::endl;                       
  return;
#endif
  if (_renderEngine != "ogre" && _renderEngine != "ogre2")
  {
    igndbg << "Grid not supported yet in rendering engine: "
            << _renderEngine << std::endl;
    return;
  }

  RenderEngine *engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    FAIL() << "Engine '" << _renderEngine
           << "' is not supported" << std::endl;
    return;
  }

  ScenePtr scene = engine->CreateScene("scene");

  GridPtr grid = scene->CreateGrid();
  ASSERT_NE(nullptr, grid);

  EXPECT_GT(grid->CellCount(), 0u);
  EXPECT_GT(grid->CellLength(), 0u);
  EXPECT_EQ(0u, grid->VerticalCellCount());

  grid->SetCellCount(15u);
  EXPECT_EQ(15u, grid->CellCount());

  grid->SetCellLength(3.25);
  EXPECT_DOUBLE_EQ(3.25, grid->CellLength());

  grid->SetVerticalCellCount(2u);
  EXPECT_EQ(2u, grid->VerticalCellCount());

  grid->PreRender();

  // create material
  MaterialPtr mat = scene->CreateMaterial();
  mat->SetAmbient(0.6, 0.7, 0.8);
  mat->SetDiffuse(0.3, 0.8, 0.2);
  mat->SetSpecular(0.4, 0.9, 1.0);

  grid->SetMaterial(mat);
  MaterialPtr gridMat = grid->Material();
  ASSERT_NE(nullptr, gridMat);
  EXPECT_EQ(math::Color(0.6f, 0.7f, 0.8f), gridMat->Ambient());
  EXPECT_EQ(math::Color(0.3f, 0.8f, 0.2f), gridMat->Diffuse());
  EXPECT_EQ(math::Color(0.4f, 0.9f, 1.0f), gridMat->Specular());

  MaterialPtr matNull = nullptr;
  grid->SetMaterial(matNull, false);
  gridMat = grid->Material();
  ASSERT_NE(nullptr, gridMat);

  grid->SetVerticalCellCount(3u);
  grid->PreRender();

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
TEST_P(GridTest, Grid)
{
  Grid(GetParam());
}

INSTANTIATE_TEST_CASE_P(Grid, GridTest,
    RENDER_ENGINE_VALUES,
    PrintToStringParam());

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
