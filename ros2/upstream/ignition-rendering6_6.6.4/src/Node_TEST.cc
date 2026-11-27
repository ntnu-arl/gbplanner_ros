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
#include <string>

#include <gz/common/Console.hh>

#include "test_config.h"  // NOLINT(build/include)

#include "gz/rendering/Node.hh"
#include "gz/rendering/RenderEngine.hh"
#include "gz/rendering/RenderingIface.hh"
#include "gz/rendering/Scene.hh"
#include "gz/rendering/Visual.hh"

using namespace gz;
using namespace rendering;

class NodeTest : public testing::Test,
                 public testing::WithParamInterface<const char *>
{
  /// \brief Test visual material
  public: void Pose(const std::string &_renderEngine);
};

/////////////////////////////////////////////////
void NodeTest::Pose(const std::string &_renderEngine)
{
#ifdef __APPLE__                                                                              
  std::cerr << "Skipping test for apple, see issue #847." << std::endl;                       
  return;
#endif
  RenderEngine *engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    FAIL() << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }

  ScenePtr scene = engine->CreateScene("scene");

  // create visual
  NodePtr node = scene->CreateVisual();
  ASSERT_NE(nullptr, node);

  // check initial pose
  EXPECT_EQ(math::Pose3d(), node->LocalPose());
  EXPECT_EQ(math::Vector3d(), node->LocalPosition());
  EXPECT_EQ(math::Quaterniond(), node->LocalRotation());

  EXPECT_EQ(math::Pose3d(), node->WorldPose());
  EXPECT_EQ(math::Vector3d(), node->WorldPosition());
  EXPECT_EQ(math::Quaterniond(), node->WorldRotation());

  // set node pose, position, and quaternion
  node->SetLocalPose(math::Pose3d(1, 2, 3, 0, 1.57, 1.57));
  EXPECT_EQ(math::Pose3d(1, 2, 3, 0, 1.57, 1.57), node->LocalPose());
  EXPECT_EQ(math::Pose3d(1, 2, 3, 0, 1.57, 1.57), node->WorldPose());

  node->SetLocalPosition(math::Vector3d(3, 4, 5));
  EXPECT_EQ(math::Vector3d(3, 4, 5), node->LocalPosition());
  EXPECT_EQ(math::Vector3d(3, 4, 5), node->WorldPosition());

  node->SetLocalRotation(math::Quaterniond(math::Vector3d(0.3, 0.1, 0.2)));
  EXPECT_EQ(math::Quaterniond(math::Vector3d(0.3, 0.1, 0.2)),
      node->LocalRotation());
  EXPECT_EQ(math::Quaterniond(math::Vector3d(0.3, 0.1, 0.2)),
      node->WorldRotation());

  node->SetWorldPose(math::Pose3d(-1, -2, -3, 0, -1.57, -1.57));
  EXPECT_EQ(math::Pose3d(-1, -2, -3, 0, -1.57, -1.57), node->WorldPose());
  EXPECT_EQ(math::Pose3d(-1, -2, -3, 0, -1.57, -1.57), node->LocalPose());

  node->SetWorldPosition(math::Vector3d(-3, -4, -5));
  EXPECT_EQ(math::Vector3d(-3, -4, -5), node->WorldPosition());
  EXPECT_EQ(math::Vector3d(-3, -4, -5), node->LocalPosition());

  node->SetWorldRotation(math::Quaterniond(math::Vector3d(-0.3, -0.1, -0.2)));
  EXPECT_EQ(math::Quaterniond(math::Vector3d(-0.3, -0.1, -0.2)),
      node->WorldRotation());
  EXPECT_EQ(math::Quaterniond(math::Vector3d(-0.3, -0.1, -0.2)),
      node->LocalRotation());

  // set NAN and inf values. verify they are not set
  node->SetLocalPose(math::Pose3d(1, NAN, 3, 0, math::INF_D, 1.57));
  EXPECT_EQ(math::Pose3d(-3, -4, -5, -0.3, -0.1, -0.2), node->LocalPose());
  EXPECT_EQ(math::Pose3d(-3, -4, -5, -0.3, -0.1, -0.2), node->WorldPose());
  node->SetWorldPose(math::Pose3d(1, NAN, 3, 0, math::INF_D, 1.57));
  EXPECT_EQ(math::Pose3d(-3, -4, -5, -0.3, -0.1, -0.2), node->LocalPose());
  EXPECT_EQ(math::Pose3d(-3, -4, -5, -0.3, -0.1, -0.2), node->WorldPose());

  node->SetLocalPosition(math::Vector3d(NAN, 4, 5));
  EXPECT_EQ(math::Vector3d(-3, -4, -5), node->WorldPosition());
  EXPECT_EQ(math::Vector3d(-3, -4, -5), node->LocalPosition());
  node->SetWorldPosition(math::Vector3d(NAN, 4, 5));
  EXPECT_EQ(math::Vector3d(-3, -4, -5), node->WorldPosition());
  EXPECT_EQ(math::Vector3d(-3, -4, -5), node->LocalPosition());

  node->SetLocalRotation(math::Quaterniond(math::Vector3d(NAN, 0.4, 1.5)));
  EXPECT_EQ(math::Quaterniond(math::Vector3d(-0.3, -0.1, -0.2)),
      node->LocalRotation());
  EXPECT_EQ(math::Quaterniond(math::Vector3d(-0.3, -0.1, -0.2)),
      node->WorldRotation());
  node->SetWorldRotation(math::Quaterniond(math::Vector3d(NAN, 0.4, 1.5)));
  EXPECT_EQ(math::Quaterniond(math::Vector3d(-0.3, -0.1, -0.2)),
      node->WorldRotation());
  EXPECT_EQ(math::Quaterniond(math::Vector3d(-0.3, -0.1, -0.2)),
      node->LocalRotation());

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
TEST_P(NodeTest, Pose)
{
  Pose(GetParam());
}

INSTANTIATE_TEST_CASE_P(Node, NodeTest,
    RENDER_ENGINE_VALUES,
    PrintToStringParam());

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
