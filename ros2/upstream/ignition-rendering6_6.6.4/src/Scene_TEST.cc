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
#include "gz/rendering/RenderTarget.hh"
#include "gz/rendering/RenderingIface.hh"
#include "gz/rendering/Scene.hh"

using namespace gz;
using namespace rendering;

class SceneTest : public testing::Test,
                  public testing::WithParamInterface<const char *>
{
  // Documentation inherited
  public: void SetUp() override
  {
    ignition::common::Console::SetVerbosity(4);
  }

  public: void Scene(const std::string &_renderEngine);
  public: void Nodes(const std::string &_renderEngine);

  /// \brief Test removing nodes
  public: void RemoveNodes(const std::string &_renderEngine);

  /// \brief Test destroying nodes
  public: void DestroyNodes(const std::string &_renderEngine);

  /// \brief Test if node cycles (child pointing to parent) are handled properly
  public: void NodeCycle(const std::string &_renderEngine);

  /// \brief Test creating and destroying materials
  public: void Materials(const std::string &_renderEngine);

  public: const std::string TEST_MEDIA_PATH =
        common::joinPaths(std::string(PROJECT_SOURCE_PATH),
        "test", "media", "skeleton");

  /// \brief Test setting and getting Time
  public: void Time(const std::string &_renderEngine);

  /// \brief Test background material
  public: void BackgroundMaterial(const std::string &_renderEngine);

  /// \brief Test enablng sky
  public: void Sky(const std::string &_renderEngine);
};

/////////////////////////////////////////////////
void SceneTest::Scene(const std::string &_renderEngine)
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

  // Check background color
  EXPECT_EQ(math::Color::Black, scene->BackgroundColor());
  scene->SetBackgroundColor(0, 1, 0, 1);
  EXPECT_EQ(math::Color(0, 1, 0, 1), scene->BackgroundColor());
  math::Color red(1, 0, 0, 1);
  scene->SetBackgroundColor(red);
  EXPECT_EQ(red, scene->BackgroundColor());

  // TODO(anyone) gradient background color and render window only supported
  // by ogre
  if (_renderEngine == "ogre2")
  {
    EXPECT_FALSE(scene->IsGradientBackgroundColor());

    // Check gradient background color
    std::array<math::Color, 4> gradientBackgroundColor =
        scene->GradientBackgroundColor();
    for (auto i = 0u; i < 4; ++i)
      EXPECT_EQ(math::Color::Black, gradientBackgroundColor[i]);
    gradientBackgroundColor[0] = math::Color::Red;
    gradientBackgroundColor[1] = math::Color::Green;
    gradientBackgroundColor[2] = math::Color::Blue;
    gradientBackgroundColor[3] = math::Color::Black;
    scene->SetGradientBackgroundColor(gradientBackgroundColor);
    EXPECT_TRUE(scene->IsGradientBackgroundColor());
    auto currentGradientBackgroundColor = scene->GradientBackgroundColor();
    EXPECT_EQ(math::Color::Red, currentGradientBackgroundColor[0]);
    EXPECT_EQ(math::Color::Green, currentGradientBackgroundColor[1]);
    EXPECT_EQ(math::Color::Blue, currentGradientBackgroundColor[2]);
    EXPECT_EQ(math::Color::Black, currentGradientBackgroundColor[3]);
    gradientBackgroundColor[0] = math::Color::White;
    scene->SetGradientBackgroundColor(gradientBackgroundColor);
    currentGradientBackgroundColor = scene->GradientBackgroundColor();
    EXPECT_EQ(math::Color::White, currentGradientBackgroundColor[0]);
    EXPECT_EQ(math::Color::Green, currentGradientBackgroundColor[1]);
    EXPECT_EQ(math::Color::Blue, currentGradientBackgroundColor[2]);
    EXPECT_EQ(math::Color::Black, currentGradientBackgroundColor[3]);
    scene->RemoveGradientBackgroundColor();
    EXPECT_FALSE(scene->IsGradientBackgroundColor());

    // test creating render window from scene
    RenderWindowPtr renderWindow = scene->CreateRenderWindow();
    EXPECT_NE(nullptr, renderWindow->Scene());
    EXPECT_EQ(scene, renderWindow->Scene());
  }

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
void SceneTest::Nodes(const std::string &_renderEngine)
{
#ifdef __APPLE__                                                                              
  std::cerr << "Skipping test for apple, see issue #847." << std::endl;                       
  return;
#endif
  auto engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    FAIL() << "Engine '" << _renderEngine << "' is not supported" << std::endl;
    return;
  }

  auto scene = engine->CreateScene("scene");
  ASSERT_NE(nullptr, scene);

  auto root = scene->RootVisual();
  ASSERT_NE(nullptr, root);

  // No nodes
  EXPECT_EQ(0u, scene->NodeCount());

  // Box visual
  auto box = scene->CreateVisual();
  ASSERT_NE(nullptr, box);

  box->AddGeometry(scene->CreateBox());
  root->AddChild(box);

  // Has node
  EXPECT_EQ(1u, scene->NodeCount());
  EXPECT_TRUE(scene->HasNode(box));
  EXPECT_TRUE(scene->HasNodeId(box->Id()));
  EXPECT_TRUE(scene->HasNodeName(box->Name()));

  // Get node
  EXPECT_EQ(box, scene->NodeByIndex(0));
  EXPECT_EQ(box, scene->NodeById(box->Id()));
  EXPECT_EQ(box, scene->NodeByName(box->Name()));

  // Has visuals
  EXPECT_EQ(1u, scene->VisualCount());
  EXPECT_TRUE(scene->HasVisual(box));
  EXPECT_TRUE(scene->HasVisualId(box->Id()));
  EXPECT_TRUE(scene->HasVisualName(box->Name()));

  // Get visual
  EXPECT_EQ(box, scene->VisualByIndex(0));
  EXPECT_EQ(box, scene->VisualById(box->Id()));
  EXPECT_EQ(box, scene->VisualByName(box->Name()));

  // child visual
  auto child = scene->CreateVisual("child");
  ASSERT_NE(nullptr, child);
  auto geom = scene->CreateBox();
  child->AddGeometry(geom);
  EXPECT_TRUE(child->HasGeometry(geom));
  EXPECT_TRUE(scene->HasVisual(child));

  // scene visuals
  EXPECT_EQ(2u, scene->VisualCount());

  // visual tree: root > box > child
  box->AddChild(child);

  // Has child
  EXPECT_TRUE(box->HasChild(child));
  EXPECT_TRUE(box->HasChildId(child->Id()));
  EXPECT_TRUE(box->HasChildName(child->Name()));
  EXPECT_EQ(1u, box->ChildCount());

  // Get child
  EXPECT_EQ(child, box->ChildById(child->Id()));
  EXPECT_EQ(child, box->ChildByName(child->Name()));
  EXPECT_EQ(child, box->ChildByIndex(0u));

  // Has parent
  EXPECT_TRUE(child->HasParent());

  // Get parent
  EXPECT_EQ(box, child->Parent());

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
void SceneTest::RemoveNodes(const std::string &_renderEngine)
{
#ifdef __APPLE__                                                                              
  std::cerr << "Skipping test for apple, see issue #847." << std::endl;                       
  return;
#endif
  auto engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    FAIL() << "Engine '" << _renderEngine << "' is not supported" << std::endl;
    return;
  }

  auto scene = engine->CreateScene("scene");
  ASSERT_NE(nullptr, scene);

  auto root = scene->RootVisual();
  ASSERT_NE(nullptr, root);

  // No nodes
  EXPECT_EQ(0u, scene->NodeCount());

  // parent visual
  auto parent = scene->CreateVisual("parent");
  ASSERT_NE(nullptr, parent);
  EXPECT_TRUE(scene->HasVisual(parent));

  // Create and add child visuals to parent
  auto child = scene->CreateVisual("child");
  ASSERT_NE(nullptr, child);
  child->AddGeometry(scene->CreateBox());
  EXPECT_TRUE(scene->HasVisual(child));
  parent->AddChild(child);

  auto child02 = scene->CreateVisual("child_02");
  ASSERT_NE(nullptr, child02);
  child02->AddGeometry(scene->CreateCylinder());
  parent->AddChild(child02);

  auto child03 = scene->CreateVisual("child_03");
  ASSERT_NE(nullptr, child03);
  child03->AddGeometry(scene->CreateSphere());
  parent->AddChild(child03);

  auto child04 = scene->CreateVisual("child_04");
  ASSERT_NE(nullptr, child04);
  child04->AddGeometry(scene->CreateSphere());
  parent->AddChild(child04);

  EXPECT_TRUE(scene->HasVisual(child02));
  EXPECT_TRUE(scene->HasVisual(child03));
  EXPECT_TRUE(scene->HasVisual(child04));
  EXPECT_TRUE(parent->HasChild(child02));
  EXPECT_TRUE(parent->HasChild(child03));
  EXPECT_TRUE(parent->HasChild(child04));
  EXPECT_EQ(4u, parent->ChildCount());
  EXPECT_EQ(5u, scene->VisualCount());

  // Remove child - this detaches the child visual
  parent->RemoveChild(child);
  EXPECT_FALSE(parent->HasChild(child));
  EXPECT_EQ(3u, parent->ChildCount());
  EXPECT_EQ(5u, scene->VisualCount());

  // Remove child by index
  parent->RemoveChildByIndex(0u);
  EXPECT_FALSE(parent->HasChild(child02));
  EXPECT_EQ(2u, parent->ChildCount());
  EXPECT_EQ(5u, scene->VisualCount());

  // Remove child by Id
  parent->RemoveChildById(child03->Id());
  EXPECT_FALSE(parent->HasChild(child03));
  EXPECT_EQ(1u, parent->ChildCount());
  EXPECT_EQ(5u, scene->VisualCount());

  // Remove child by name
  parent->RemoveChildByName(child04->Name());
  EXPECT_FALSE(parent->HasChild(child04));
  EXPECT_EQ(0u, parent->ChildCount());
  EXPECT_EQ(5u, scene->VisualCount());

  // Verify that child visuals can be re-attached
  parent->AddChild(child);
  EXPECT_TRUE(parent->HasChild(child));
  EXPECT_EQ(1u, parent->ChildCount());

  parent->AddChild(child02);
  EXPECT_TRUE(parent->HasChild(child02));
  EXPECT_EQ(2u, parent->ChildCount());

  parent->AddChild(child03);
  EXPECT_TRUE(parent->HasChild(child03));
  EXPECT_EQ(3u, parent->ChildCount());

  parent->AddChild(child04);
  EXPECT_TRUE(parent->HasChild(child04));
  EXPECT_EQ(4u, parent->ChildCount());
  EXPECT_EQ(5u, scene->VisualCount());

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
void SceneTest::DestroyNodes(const std::string &_renderEngine)
{
#ifdef __APPLE__                                                                              
  std::cerr << "Skipping test for apple, see issue #847." << std::endl;                       
  return;
#endif
  auto engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    FAIL() << "Engine '" << _renderEngine << "' is not supported" << std::endl;
    return;
  }

  auto scene = engine->CreateScene("scene");
  ASSERT_NE(nullptr, scene);

  auto root = scene->RootVisual();
  ASSERT_NE(nullptr, root);

  // No nodes
  EXPECT_EQ(0u, scene->NodeCount());

  // parent visual
  auto parent = scene->CreateVisual("parent");
  ASSERT_NE(nullptr, parent);
  EXPECT_TRUE(scene->HasVisual(parent));

  // Create and add child visuals to parent
  // visual tree: root > parent > child
  //                            > child_02
  //                            > child_03
  //                            > child_04
  auto child = scene->CreateVisual("child");
  ASSERT_NE(nullptr, child);
  child->AddGeometry(scene->CreateBox());
  EXPECT_TRUE(scene->HasVisual(child));
  parent->AddChild(child);

  auto child02 = scene->CreateVisual("child_02");
  ASSERT_NE(nullptr, child02);
  child02->AddGeometry(scene->CreateCylinder());
  parent->AddChild(child02);

  auto child03 = scene->CreateVisual("child_03");
  ASSERT_NE(nullptr, child03);
  child03->AddGeometry(scene->CreateSphere());
  parent->AddChild(child03);

  auto child04 = scene->CreateVisual("child_04");
  ASSERT_NE(nullptr, child04);
  child04->AddGeometry(scene->CreateSphere());
  parent->AddChild(child04);

  EXPECT_TRUE(scene->HasVisual(child02));
  EXPECT_TRUE(scene->HasVisual(child03));
  EXPECT_TRUE(scene->HasVisual(child04));
  EXPECT_TRUE(parent->HasChild(child02));
  EXPECT_TRUE(parent->HasChild(child03));
  EXPECT_TRUE(parent->HasChild(child04));
  EXPECT_EQ(4u, parent->ChildCount());
  EXPECT_EQ(5u, scene->VisualCount());

  // Destroy a child visual
  scene->DestroyVisual(child);
  EXPECT_FALSE(parent->HasChild(child));
  EXPECT_FALSE(scene->HasVisual(child));
  EXPECT_EQ(3u, parent->ChildCount());
  EXPECT_EQ(4u, scene->VisualCount());

  // Destroy a child visual by index
  scene->DestroyVisualByIndex(0u);
  EXPECT_FALSE(parent->HasChild(child02));
  EXPECT_FALSE(scene->HasVisual(child02));
  EXPECT_EQ(2u, parent->ChildCount());
  EXPECT_EQ(3u, scene->VisualCount());

  // Destroy a child visual by id
  scene->DestroyVisualById(child03->Id());
  EXPECT_FALSE(parent->HasChild(child03));
  EXPECT_FALSE(scene->HasVisual(child03));
  EXPECT_EQ(1u, parent->ChildCount());
  EXPECT_EQ(2u, scene->VisualCount());

  // Destroy a child visual by name
  scene->DestroyVisualByName(child04->Name());
  EXPECT_FALSE(parent->HasChild(child04));
  EXPECT_FALSE(scene->HasVisual(child04));
  EXPECT_EQ(0u, parent->ChildCount());
  EXPECT_EQ(1u, scene->VisualCount());

  // Create and add more child visuals to parent
  // visual tree: root > parent > child_a > child_aa
  //                            > child_b
  auto childA = scene->CreateVisual("child_a");
  ASSERT_NE(nullptr, childA);
  childA->AddGeometry(scene->CreateBox());
  EXPECT_TRUE(scene->HasVisual(childA));
  parent->AddChild(childA);

  auto childB = scene->CreateVisual("child_b");
  ASSERT_NE(nullptr, childB);
  childB->AddGeometry(scene->CreateSphere());
  parent->AddChild(childB);

  auto childAA = scene->CreateVisual("child_aa");
  ASSERT_NE(nullptr, childAA);
  childAA->AddGeometry(scene->CreateCylinder());
  childA->AddChild(childAA);

  EXPECT_TRUE(parent->HasChild(childA));
  EXPECT_TRUE(parent->HasChild(childB));
  EXPECT_TRUE(childA->HasChild(childAA));
  EXPECT_EQ(2u, parent->ChildCount());
  EXPECT_EQ(1u, childA->ChildCount());
  EXPECT_EQ(4u, scene->VisualCount());

  // Destroy parent visual - this should cause all child visuals to be
  // detached but not destroyed
  scene->DestroyVisual(parent);
  EXPECT_FALSE(scene->HasVisual(parent));
  EXPECT_TRUE(scene->HasVisual(childA));
  EXPECT_TRUE(scene->HasVisual(childB));
  EXPECT_TRUE(scene->HasVisual(childAA));
  EXPECT_EQ(1u, childA->ChildCount());
  EXPECT_EQ(3u, scene->VisualCount());

  // Create another parent and attach all child visuals
  auto parent02 = scene->CreateVisual("parent_02");
  ASSERT_NE(nullptr, parent02);
  EXPECT_TRUE(scene->HasVisual(parent02));
  parent02->AddChild(childA);
  parent02->AddChild(childB);
  EXPECT_TRUE(parent02->HasChild(childA));
  EXPECT_TRUE(parent02->HasChild(childB));
  EXPECT_EQ(2u, parent02->ChildCount());
  EXPECT_EQ(1u, childA->ChildCount());
  EXPECT_EQ(4u, scene->VisualCount());

  // Recursive destroy - all child visuals should also be destroyed
  scene->DestroyVisual(parent02, true);
  EXPECT_FALSE(scene->HasVisual(parent02));
  EXPECT_FALSE(scene->HasVisual(childA));
  EXPECT_FALSE(scene->HasVisual(childB));
  EXPECT_FALSE(scene->HasVisual(childAA));

  auto gizmoVisual = scene->CreateGizmoVisual("gizmo_visual");
  auto planeVisual = scene->CreatePlane();
  auto meshVisual = scene->CreateMesh(
    common::joinPaths(TEST_MEDIA_PATH, "walk.dae"));

  common::MeshPtr mesh(new common::Mesh());
  meshVisual = scene->CreateMesh(mesh.get());

  scene->DestroyVisuals();

  common::Material mat(math::Color(1.0f, 0.5f, 0.2f, 1.0f));
  auto materialVisual = scene->CreateMaterial(mat);
  scene->DestroyMaterial(materialVisual);
  scene->DestroyMaterial(MaterialPtr());

  auto depthCameraSensor = scene->CreateDepthCamera();
  scene->DestroySensors();
  depthCameraSensor = scene->CreateDepthCamera("camera_depth");
  scene->DestroySensorByName("camera_depth");
  depthCameraSensor = scene->CreateDepthCamera("camera_depth");
  EXPECT_FALSE(scene->HasSensor(ConstSensorPtr()));
  EXPECT_FALSE(scene->HasSensorId(8));
  EXPECT_FALSE(scene->HasSensorName("invalid"));
  EXPECT_TRUE(scene->HasSensorName("camera_depth"));

  auto dCSensor = scene->SensorByName("camera_depth");
  EXPECT_TRUE(scene->HasSensor(dCSensor));
  EXPECT_TRUE(scene->HasSensorName("camera_depth"));
  scene->DestroySensor(dCSensor, false);

  depthCameraSensor = scene->CreateDepthCamera(76);
  dCSensor = scene->SensorById(76);
  EXPECT_TRUE(scene->HasSensor(dCSensor));
  EXPECT_TRUE(scene->HasSensorId(76));
  scene->DestroySensor(dCSensor, true);

  depthCameraSensor = scene->CreateDepthCamera(76);
  scene->DestroySensorById(76);

  depthCameraSensor = scene->CreateDepthCamera();
  scene->DestroySensorByIndex(0);

  // lights
  auto spotLight = scene->CreateSpotLight();
  auto pointLight = scene->CreatePointLight("point_light");
  auto directionalLight = scene->CreateDirectionalLight(99);

  auto directionalLight2 = scene->LightById(99);
  EXPECT_EQ(directionalLight, directionalLight2);

  auto directionalLight3 = scene->CreateDirectionalLight("directional_light");

  auto pointLight2 = scene->LightByName("point_light");
  EXPECT_EQ(pointLight, pointLight2);

  scene->DestroyLightByIndex(0);
  scene->DestroyLight(pointLight, true);
  scene->DestroyLights();

  spotLight = scene->CreateSpotLight();
  scene->DestroyNodeByIndex(0);

  spotLight = scene->CreateSpotLight("light_node");
  scene->DestroyNodeByName("light_node");

  spotLight = scene->CreateSpotLight(56);
  scene->DestroyNodeById(56);

  EXPECT_EQ(0u, scene->VisualCount());
  EXPECT_EQ(0u, scene->SensorCount());
  EXPECT_EQ(0u, scene->LightCount());

  EXPECT_EQ(std::chrono::milliseconds(0), scene->Time());
  scene->SetTime(std::chrono::milliseconds(3550));
  EXPECT_EQ(std::chrono::milliseconds(3550), scene->Time());

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
void SceneTest::NodeCycle(const std::string &_renderEngine)
{
#ifdef __APPLE__                                                                              
  std::cerr << "Skipping test for apple, see issue #847." << std::endl;                       
  return;
#endif
  auto engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    FAIL() << "Engine '" << _renderEngine << "' is not supported" << std::endl;
    return;
  }

  auto scene = engine->CreateScene("scene");
  ASSERT_NE(nullptr, scene);

  auto root = scene->RootVisual();
  ASSERT_NE(nullptr, root);

  // No nodes
  EXPECT_EQ(0u, scene->NodeCount());

  {
    // parent visual
    auto parent = scene->CreateVisual("parent");
    ASSERT_NE(nullptr, parent);
    EXPECT_TRUE(scene->HasVisual(parent));

    // Set child = parent on purpose to create a cycle of size 1
    const auto child = parent;
    ASSERT_NE(nullptr, child);
    parent->AddChild(child);
    // Adding the child should have failed
    EXPECT_FALSE(parent->HasChild(child));

    // Try Removing child. This should do nothing
    parent->RemoveChild(child);
    ASSERT_NE(nullptr, parent);

    // add child again and try to destroy
    parent->AddChild(child);
    scene->DestroyVisual(parent, true);
    EXPECT_EQ(0u, scene->VisualCount());
  }

  {
    // Add another parent and create a longer cycle
    auto parent = scene->CreateVisual("parent");
    ASSERT_NE(nullptr, parent);
    EXPECT_TRUE(scene->HasVisual(parent));

    auto childA = scene->CreateVisual("child_A");
    ASSERT_NE(nullptr, childA);
    parent->AddChild(childA);

    // set childAA to parent so the cycle is "parent->childA->parent"
    auto childAA = parent;
    childA->AddChild(childAA);

    // This should not crash
    scene->DestroyVisual(parent, true);
    EXPECT_EQ(0u, scene->VisualCount());
  }

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
void SceneTest::Materials(const std::string &_renderEngine)
{
#ifdef __APPLE__                                                                              
  std::cerr << "Skipping test for apple, see issue #847." << std::endl;                       
  return;
#endif
  auto engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    FAIL() << "Engine '" << _renderEngine << "' is not supported" << std::endl;
    return;
  }

  auto scene = engine->CreateScene("scene");
  ASSERT_NE(nullptr, scene);

  // create and destroy material
  MaterialPtr mat = scene->CreateMaterial();
  ASSERT_NE(nullptr, mat);
  std::string matName = mat->Name();
  ASSERT_TRUE(scene->MaterialRegistered(matName));
  ASSERT_EQ(mat, scene->Material(matName));
  scene->DestroyMaterial(mat);
  ASSERT_FALSE(scene->MaterialRegistered(matName));

  // create and destroy material with user-specified name
  std::string mat2Name = "another_material";
  MaterialPtr mat2 = scene->CreateMaterial(mat2Name);
  ASSERT_NE(nullptr, mat2);
  ASSERT_EQ(mat2Name, mat2->Name());
  ASSERT_TRUE(scene->MaterialRegistered(mat2Name));
  ASSERT_EQ(mat2, scene->Material(mat2Name));
  scene->DestroyMaterial(mat2);
  ASSERT_FALSE(scene->MaterialRegistered(mat2Name));

  // Create mesh for testing
  auto root = scene->RootVisual();
  ASSERT_NE(nullptr, root);
  VisualPtr visual = scene->CreateVisual();
  ASSERT_NE(nullptr, visual);
  MeshPtr mesh = std::dynamic_pointer_cast<Mesh>(scene->CreateBox());
  visual->AddGeometry(mesh);
  root->AddChild(visual);

  // verify mesh default material is registered with scene
  ASSERT_EQ(1u, mesh->SubMeshCount());
  SubMeshPtr submesh = mesh->SubMeshByIndex(0u);
  ASSERT_NE(nullptr, submesh);
  MaterialPtr defaultMeshMat = submesh->Material();
  ASSERT_NE(nullptr, defaultMeshMat);
  std::string defaultMeshMatName = defaultMeshMat->Name();
  ASSERT_TRUE(scene->MaterialRegistered(defaultMeshMatName));

  // create new material for testing
  std::string newMeshMatName = "mesh_material";
  MaterialPtr newMeshMat = scene->CreateMaterial(newMeshMatName);
  ASSERT_NE(nullptr, newMeshMat);

  // test assigning material to mesh. The second param (false) tells the mesh
  // not to clone the material
  mesh->SetMaterial(newMeshMat, false);
  MaterialPtr retMeshMat = submesh->Material();
  ASSERT_EQ(newMeshMat, retMeshMat);

  // verify default mesh material is removed from scene
  ASSERT_FALSE(scene->MaterialRegistered(defaultMeshMatName));

  // create another material for testing
  std::string newMeshMat2Name = "mesh_material2";
  MaterialPtr newMeshMat2 = scene->CreateMaterial(newMeshMat2Name);
  ASSERT_NE(nullptr, newMeshMat2);

  // test assigning material to mesh. The second param (true) tells the mesh
  // to make a unique copy of the material and the mesh will take ownership of
  // the cloned material
  mesh->SetMaterial(newMeshMat2, true);
  MaterialPtr retMeshMat2 = submesh->Material();
  ASSERT_NE(newMeshMat2, retMeshMat2);
  ASSERT_NE(nullptr, retMeshMat2);

  // verify previous mesh material is not removed from scene
  ASSERT_TRUE(scene->MaterialRegistered(newMeshMatName));

  // create another material for testing
  std::string subMeshMatName = "submesh_material";
  MaterialPtr subMeshMat = scene->CreateMaterial(subMeshMatName);
  ASSERT_NE(nullptr, subMeshMat);

  // test assigning material to submesh. The second param (false) tells the
  // submesh not to clone the material
  submesh->SetMaterial(subMeshMat, false);
  MaterialPtr retSubMeshMat = submesh->Material();
  ASSERT_EQ(subMeshMat, retSubMeshMat);

  // verify parent mesh material is not removed from scene as the parent mesh
  // material is shared with other sibling submeshes
  ASSERT_TRUE(scene->MaterialRegistered(newMeshMatName));

  // create another material for testing
  std::string subMeshMat2Name = "submesh_material2";
  MaterialPtr subMeshMat2 = scene->CreateMaterial(subMeshMat2Name);
  ASSERT_NE(nullptr, subMeshMat2);

  // test assigning material to submesh. The second param (true) tells the
  // submesh to make a unique copy of the material and submesh will take
  // ownership of the cloned material
  submesh->SetMaterial(subMeshMat2, true);
  MaterialPtr retSubMeshMat2 = submesh->Material();
  ASSERT_NE(subMeshMat2, retSubMeshMat2);
  ASSERT_NE(nullptr, retSubMeshMat2);

  // verify previous submesh material is not removed from scene
  ASSERT_TRUE(scene->MaterialRegistered(subMeshMatName));

  // remove visual and its mesh and submesh
  // verify cloned materials are also be removed from the scene as they are
  // unique to the mesh and submesh
  scene->DestroyVisual(visual);
  ASSERT_FALSE(scene->MaterialRegistered(retMeshMat2->Name()));
  ASSERT_FALSE(scene->MaterialRegistered(retSubMeshMat2->Name()));

  // destroy all scene materials and verify
  scene->DestroyMaterials();
  ASSERT_FALSE(scene->MaterialRegistered(newMeshMatName));
  ASSERT_FALSE(scene->MaterialRegistered(newMeshMat2Name));
  ASSERT_FALSE(scene->MaterialRegistered(subMeshMatName));
  ASSERT_FALSE(scene->MaterialRegistered(subMeshMat2Name));

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
void SceneTest::Time(const std::string &_renderEngine)
{
#ifdef __APPLE__                                                                                                                                                                              
  std::cerr << "Skipping test for apple, see issue #847." << std::endl;
  return;
#endif
  auto engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    FAIL() << "Engine '" << _renderEngine << "' is not supported" << std::endl;
    return;
  }

  auto scene = engine->CreateScene("scene");
  ASSERT_NE(nullptr, scene);

  std::chrono::steady_clock::duration duration =
    std::chrono::steady_clock::duration::zero();

#if IGNITION_RENDERING_MAJOR_VERSION <= 6
  // -1 is a hack and is the only value supposed to
  // be ignored by ign-rendering6.
  //
  // We must call it and EXPECT_EQ(duration, scene->Time());
  // should still be pass in ign-rendering6.
  scene->SetTime(std::chrono::nanoseconds(-1));
#endif

  EXPECT_EQ(duration, scene->Time());

  duration = std::chrono::seconds(23);

  scene->SetTime(duration);
  EXPECT_EQ(duration, scene->Time());

  duration = std::chrono::seconds(1) + std::chrono::milliseconds(123);
  scene->SetTime(duration);
  EXPECT_EQ(duration, scene->Time());

  duration = std::chrono::hours(24) +
             std::chrono::seconds(6) +
             std::chrono::milliseconds(123);
  scene->SetTime(duration);
  EXPECT_EQ(duration, scene->Time());

  // Clean up
  engine->DestroyScene(scene);
  rendering::unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
void SceneTest::BackgroundMaterial(const std::string &_renderEngine)
{
#ifdef __APPLE__                                                                                                                                                                              
  std::cerr << "Skipping test for apple, see issue #847." << std::endl;
  return;
#endif
  auto engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    FAIL() << "Engine '" << _renderEngine << "' is not supported" << std::endl;
    return;
  }

  auto scene = engine->CreateScene("scene");
  ASSERT_NE(nullptr, scene);

  EXPECT_EQ(nullptr, scene->BackgroundMaterial());

  rendering::MaterialPtr mat = scene->CreateMaterial("test_mat");
  scene->SetBackgroundMaterial(mat);
  EXPECT_EQ(mat, scene->BackgroundMaterial());

  scene->SetBackgroundMaterial(nullptr);
  EXPECT_EQ(nullptr, scene->BackgroundMaterial());

  // Clean up
  engine->DestroyScene(scene);
  rendering::unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
void SceneTest::Sky(const std::string &_renderEngine)
{
#ifdef __APPLE__                                                                                                                                                                              
  std::cerr << "Skipping test for apple, see issue #847." << std::endl;
  return;
#endif
  auto engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    FAIL() << "Engine '" << _renderEngine << "' is not supported" << std::endl;
    return;
  }

  if (_renderEngine != "ogre2")
  {
    igndbg << "Sky not supported yet in rendering engine: "
            << _renderEngine << std::endl;
    return;
  }

  auto scene = engine->CreateScene("scene");
  ASSERT_NE(nullptr, scene);

  EXPECT_FALSE(scene->SkyEnabled());

  scene->SetSkyEnabled(false);
  EXPECT_FALSE(scene->SkyEnabled());

  scene->SetSkyEnabled(true);
  EXPECT_TRUE(scene->SkyEnabled());

  scene->SetSkyEnabled(false);
  EXPECT_FALSE(scene->SkyEnabled());

  // set background material and verify sky remains disabled
  rendering::MaterialPtr mat = scene->CreateMaterial("test_mat");
  scene->SetBackgroundMaterial(mat);
  EXPECT_EQ(mat, scene->BackgroundMaterial());
  EXPECT_FALSE(scene->SkyEnabled());

  // enable sky and verify it is not affected by background material
  scene->SetSkyEnabled(true);
  EXPECT_TRUE(scene->SkyEnabled());
  scene->SetBackgroundMaterial(nullptr);
  EXPECT_TRUE(scene->SkyEnabled());

  // Clean up
  engine->DestroyScene(scene);
  rendering::unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
TEST_P(SceneTest, Scene)
{
  Scene(GetParam());
}

/////////////////////////////////////////////////
TEST_P(SceneTest, Nodes)
{
  Nodes(GetParam());
}

/////////////////////////////////////////////////
TEST_P(SceneTest, RemoveNodes)
{
  RemoveNodes(GetParam());
}

/////////////////////////////////////////////////
TEST_P(SceneTest, DestroyNodes)
{
  DestroyNodes(GetParam());
}

/////////////////////////////////////////////////
TEST_P(SceneTest, NodeCycle)
{
  NodeCycle(GetParam());
}

/////////////////////////////////////////////////
TEST_P(SceneTest, Materials)
{
  Materials(GetParam());
}

/////////////////////////////////////////////////
TEST_P(SceneTest, Time)
{
  Time(GetParam());
}

/////////////////////////////////////////////////
TEST_P(SceneTest, BackgroundMaterial)
{
  BackgroundMaterial(GetParam());
}

/////////////////////////////////////////////////
TEST_P(SceneTest, Sky)
{
  Sky(GetParam());
}

INSTANTIATE_TEST_CASE_P(Scene, SceneTest,
    RENDER_ENGINE_VALUES,
    PrintToStringParam());

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
