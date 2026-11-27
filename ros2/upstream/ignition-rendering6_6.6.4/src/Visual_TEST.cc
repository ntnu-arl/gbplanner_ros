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
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/AxisAlignedBox.hh>

#include "test_config.h"  // NOLINT(build/include)

#include "gz/rendering/Geometry.hh"
#include "gz/rendering/RenderEngine.hh"
#include "gz/rendering/RenderingIface.hh"
#include "gz/rendering/Scene.hh"
#include "gz/rendering/Visual.hh"

using namespace gz;
using namespace rendering;

class VisualTest : public testing::Test,
                   public testing::WithParamInterface<const char *>
{
  /// \brief Test visual material
  public: void Material(const std::string &_renderEngine);

  /// \brief Test adding removing children
  public: void Children(const std::string &_renderEngine);

  /// \brief Test visual scale
  public: void Scale(const std::string &_renderEngine);

  /// \brief User data
  public: void UserData(const std::string &_renderEngine);

  /// \brief Test adding removing geometry
  public: void Geometry(const std::string &_renderEngine);

  /// \brief Test setting visibility flags
  public: void VisibilityFlags(const std::string &_renderEngine);

  /// \brief Test getting setting bounding boxes
  public: void BoundingBox(const std::string &_renderEngine);

  /// \brief Test changing to wireframe
  public: void Wireframe(const std::string &_renderEngine);

  /// \brief Test cloning visuals
  public: void Clone(const std::string &_renderEngine);
};

/////////////////////////////////////////////////
void VisualTest::Material(const std::string &_renderEngine)
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
  VisualPtr visual = scene->CreateVisual();
  ASSERT_NE(nullptr, visual);

  // check initial material
  EXPECT_EQ(nullptr, visual->Material());

  // create material
  math::Color ambient(0.5f, 0.2f, 0.4f, 1.0f);
  math::Color diffuse(0.1f, 0.9f, 0.3f, 1.0f);
  math::Color specular(0.8f, 0.7f, 0.0f, 1.0f);
  double transparency = 0.3;
  MaterialPtr material = scene->CreateMaterial("unique");
  ASSERT_NE(nullptr, material);
  EXPECT_TRUE(scene->MaterialRegistered("unique"));
  material->SetAmbient(ambient);
  material->SetDiffuse(diffuse);
  material->SetSpecular(specular);
  material->SetTransparency(transparency);

  // Set visual material but do not clone the material
  visual->SetMaterial(material, false);
  EXPECT_EQ(material, visual->Material());

  // Set visual material and clone the material
  visual->SetMaterial(material, true);
  MaterialPtr cloneMat = visual->Material();
  EXPECT_NE(material, cloneMat);
  EXPECT_NE(material->Name(), cloneMat->Name());

  // verify clone material properties
  EXPECT_EQ(material->Type(), cloneMat->Type());
  EXPECT_EQ(ambient, cloneMat->Ambient());
  EXPECT_EQ(diffuse, cloneMat->Diffuse());
  EXPECT_EQ(specular, cloneMat->Specular());
  EXPECT_DOUBLE_EQ(transparency, cloneMat->Transparency());

  // create another material
  math::Color ambient2(0.0f, 0.0f, 1.0f, 1.0f);
  math::Color diffuse2(1.0f, 0.0f, 1.0f, 1.0f);
  math::Color specular2(0.0f, 1.0f, 0.0f, 1.0f);
  double transparency2 = 0;
  MaterialPtr material2 = scene->CreateMaterial("unique2");
  ASSERT_NE(nullptr, material2);
  EXPECT_TRUE(scene->MaterialRegistered("unique2"));
  material2->SetAmbient(ambient2);
  material2->SetDiffuse(diffuse2);
  material2->SetSpecular(specular2);
  material2->SetTransparency(transparency2);

  // Set material to the same visual using its name and verify material changed
  visual->SetMaterial("unique2", true);
  MaterialPtr cloneMat2 = visual->Material();
  EXPECT_NE(material2, cloneMat);
  EXPECT_NE(material2->Name(), cloneMat2->Name());
  EXPECT_EQ(material2->Type(), cloneMat2->Type());
  EXPECT_EQ(ambient2, cloneMat2->Ambient());
  EXPECT_EQ(diffuse2, cloneMat2->Diffuse());
  EXPECT_EQ(specular2, cloneMat2->Specular());
  EXPECT_DOUBLE_EQ(transparency2, cloneMat2->Transparency());

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
TEST_P(VisualTest, Material)
{
  Material(GetParam());
}

/////////////////////////////////////////////////
void VisualTest::Children(const std::string &_renderEngine)
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

  ScenePtr scene = engine->CreateScene("scene2");

  // create visual
  VisualPtr visual = scene->CreateVisual();
  ASSERT_NE(nullptr, visual);

  VisualPtr child = scene->CreateVisual();
  ASSERT_NE(nullptr, child);

  // no child by default
  EXPECT_EQ(0u, visual->ChildCount());

  // attach child and verify
  visual->AddChild(child);
  EXPECT_EQ(1u, visual->ChildCount());
  EXPECT_TRUE(visual->HasChild(child));
  EXPECT_TRUE(visual->HasChildId(child->Id()));
  EXPECT_TRUE(visual->HasChildName(child->Name()));

  EXPECT_EQ(child, visual->ChildById(child->Id()));
  EXPECT_EQ(child, visual->ChildByName(child->Name()));
  EXPECT_EQ(child, visual->ChildByIndex(0u));

  // attempt to attach itself and verify self not added
  visual->AddChild(visual);
  EXPECT_EQ(1u, visual->ChildCount());
  EXPECT_FALSE(visual->HasChild(visual));

  // attach more than one child
  VisualPtr child2 = scene->CreateVisual();
  ASSERT_NE(nullptr, child2);
  visual->AddChild(child2);
  EXPECT_EQ(2u, visual->ChildCount());
  VisualPtr child3 = scene->CreateVisual();
  ASSERT_NE(nullptr, child3);
  visual->AddChild(child3);
  EXPECT_EQ(3u, visual->ChildCount());

  // test different child removal methods
  EXPECT_EQ(child, visual->RemoveChild(child));
  EXPECT_EQ(child2, visual->RemoveChildById(child2->Id()));
  EXPECT_EQ(child3, visual->RemoveChildByName(child3->Name()));

  // attach previously removed child and remove again
  visual->AddChild(child);
  EXPECT_EQ(child, visual->RemoveChildByIndex(0u));

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
TEST_P(VisualTest, Children)
{
  Children(GetParam());
}

/////////////////////////////////////////////////
void VisualTest::Scale(const std::string &_renderEngine)
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

  ScenePtr scene = engine->CreateScene("scene2");

  // create visual
  VisualPtr visual = scene->CreateVisual();
  ASSERT_NE(nullptr, visual);

  VisualPtr child = scene->CreateVisual();
  ASSERT_NE(nullptr, child);

  // verify initial scale properties
  EXPECT_TRUE(visual->InheritScale());
  EXPECT_EQ(math::Vector3d::One, visual->LocalScale());
  EXPECT_EQ(math::Vector3d::One, visual->WorldScale());

  // set visual scale and verify
  math::Vector3d scale(4, 2, 3);
  visual->SetLocalScale(scale);
  EXPECT_EQ(scale, visual->LocalScale());
  EXPECT_EQ(scale, visual->WorldScale());

  math::Vector3d worldScale(6, 1, 4);
  visual->SetWorldScale(worldScale);
  EXPECT_EQ(worldScale, visual->LocalScale());
  EXPECT_EQ(worldScale, visual->WorldScale());

  // attach child and verify
  visual->AddChild(child);
  EXPECT_EQ(1u, visual->ChildCount());
  EXPECT_EQ(child, visual->ChildById(child->Id()));

  // verify child initial scale
  EXPECT_TRUE(child->InheritScale());
  EXPECT_EQ(math::Vector3d::One, child->LocalScale());
  EXPECT_EQ(worldScale, child->WorldScale());

  // Set child scale and verify
  math::Vector3d childScale(5, 1, 9);
  child->SetLocalScale(childScale);
  EXPECT_EQ(childScale, child->LocalScale());
  EXPECT_EQ(childScale * worldScale, child->WorldScale());

  math::Vector3d childWorldScale(5, 1, 9);
  child->SetWorldScale(childWorldScale);
  math::Vector3d childLocalScale = childWorldScale / worldScale;
  EXPECT_EQ(childLocalScale, child->LocalScale());
  EXPECT_EQ(childWorldScale, child->WorldScale());

  // Set visual scale and verify visual and child scale
  math::Vector3d scale2(3, 5, 9);
  visual->SetLocalScale(scale2);
  EXPECT_EQ(scale2, visual->LocalScale());
  EXPECT_EQ(scale2, visual->WorldScale());
  EXPECT_EQ(childLocalScale, child->LocalScale());
  EXPECT_EQ(scale2 * childLocalScale, child->WorldScale());

  // test setting child inherit scale to false
  child->SetInheritScale(false);
  EXPECT_FALSE(child->InheritScale());
  EXPECT_EQ(childLocalScale, child->LocalScale());
  EXPECT_EQ(childLocalScale, child->WorldScale());

  // set parent visual scale and verify the child is not affected.
  math::Vector3d scale3(1, 9, 8);
  visual->SetLocalScale(scale3);
  EXPECT_EQ(scale3, visual->LocalScale());
  EXPECT_EQ(scale3, visual->WorldScale());
  EXPECT_EQ(childLocalScale, child->LocalScale());
  EXPECT_EQ(childLocalScale, child->WorldScale());

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
TEST_P(VisualTest, Scale)
{
  Scale(GetParam());
}

/////////////////////////////////////////////////
void VisualTest::UserData(const std::string &_renderEngine)
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

  ScenePtr scene = engine->CreateScene("scene3");

  // create visual
  VisualPtr visual = scene->CreateVisual();
  ASSERT_NE(nullptr, visual);

  // int
  std::string intKey = "int";
  int intValue = 1998;
  EXPECT_FALSE(visual->HasUserData(intKey));
  visual->SetUserData(intKey, intValue);
  EXPECT_TRUE(visual->HasUserData(intKey));
  Variant value = visual->UserData(intKey);
  EXPECT_EQ(intValue, std::get<int>(value));

  // float
  std::string floatKey = "float";
  float floatValue = 1.345f;
  EXPECT_FALSE(visual->HasUserData(floatKey));
  visual->SetUserData(floatKey, floatValue);
  EXPECT_TRUE(visual->HasUserData(floatKey));
  value = visual->UserData(floatKey);
  EXPECT_FLOAT_EQ(floatValue, std::get<float>(value));

  // double
  std::string doubleKey = "double";
  double doubleValue = -0.123;
  EXPECT_FALSE(visual->HasUserData(doubleKey));
  visual->SetUserData(doubleKey, doubleValue);
  EXPECT_TRUE(visual->HasUserData(doubleKey));
  value = visual->UserData(doubleKey);
  EXPECT_DOUBLE_EQ(doubleValue, std::get<double>(value));

  // string
  std::string stringKey = "string";
  std::string stringValue = "test_string";
  EXPECT_FALSE(visual->HasUserData(stringKey));
  visual->SetUserData(stringKey, stringValue);
  EXPECT_TRUE(visual->HasUserData(stringKey));
  value = visual->UserData(stringKey);
  EXPECT_EQ(stringValue, std::get<std::string>(value));

  // bool
  std::string boolKey = "bool";
  bool boolValue = true;
  EXPECT_FALSE(visual->HasUserData(boolKey));
  visual->SetUserData(boolKey, boolValue);
  EXPECT_TRUE(visual->HasUserData(boolKey));
  value = visual->UserData(boolKey);
  EXPECT_EQ(boolValue, std::get<bool>(value));

  // unsigned int
  std::string unsignedIntKey = "unsignedInt";
  unsigned int unsignedIntValue = 5u;
  EXPECT_FALSE(visual->HasUserData(unsignedIntKey));
  visual->SetUserData(unsignedIntKey, unsignedIntValue);
  EXPECT_TRUE(visual->HasUserData(unsignedIntKey));
  value = visual->UserData(unsignedIntKey);
  EXPECT_EQ(unsignedIntValue, std::get<unsigned int>(value));

  // test a key that does not exist (should return no data)
  value = visual->UserData("invalidKey");
  EXPECT_FALSE(std::holds_alternative<int>(value));
  EXPECT_FALSE(std::holds_alternative<float>(value));
  EXPECT_FALSE(std::holds_alternative<double>(value));
  EXPECT_FALSE(std::holds_alternative<std::string>(value));
  EXPECT_FALSE(std::holds_alternative<bool>(value));
  EXPECT_FALSE(std::holds_alternative<unsigned int>(value));
  EXPECT_EQ(0u, value.index());

  // test invalid access
  EXPECT_THROW(
  {
    auto res = std::get<int>(value);
    igndbg << res << std::endl;
  }, std::bad_variant_access);

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
TEST_P(VisualTest, UserData)
{
  UserData(GetParam());
}

/////////////////////////////////////////////////
void VisualTest::Geometry(const std::string &_renderEngine)
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

  ScenePtr scene = engine->CreateScene("scene4");

  // create visual
  VisualPtr visual = scene->CreateVisual();
  ASSERT_NE(nullptr, visual);

  // Add geometries
  GeometryPtr box = scene->CreateBox();
  visual->AddGeometry(box);
  EXPECT_EQ(1u, visual->GeometryCount());
  EXPECT_TRUE(visual->HasGeometry(box));
  EXPECT_EQ(box, visual->GeometryByIndex(0u));
  EXPECT_TRUE(box->HasParent());
  EXPECT_EQ(visual, box->Parent());

  GeometryPtr cylinder = scene->CreateCylinder();
  visual->AddGeometry(cylinder);
  EXPECT_EQ(2u, visual->GeometryCount());
  EXPECT_TRUE(visual->HasGeometry(cylinder));
  EXPECT_EQ(cylinder, visual->GeometryByIndex(1u));
  EXPECT_TRUE(cylinder->HasParent());
  EXPECT_EQ(visual, cylinder->Parent());

  GeometryPtr sphere = scene->CreateSphere();
  visual->AddGeometry(sphere);
  EXPECT_EQ(3u, visual->GeometryCount());
  EXPECT_TRUE(visual->HasGeometry(sphere));
  EXPECT_EQ(sphere, visual->GeometryByIndex(2u));
  EXPECT_TRUE(sphere->HasParent());
  EXPECT_EQ(visual, sphere->Parent());

  // Remove geometries
  GeometryPtr removed = visual->RemoveGeometryByIndex(0u);
  EXPECT_EQ(2u, visual->GeometryCount());
  EXPECT_EQ(box, removed);
  EXPECT_FALSE(box->HasParent());
  EXPECT_EQ(nullptr, box->Parent());

  GeometryPtr removed2 = visual->RemoveGeometry(cylinder);
  EXPECT_EQ(1u, visual->GeometryCount());
  EXPECT_EQ(cylinder, removed2);
  EXPECT_FALSE(cylinder->HasParent());
  EXPECT_EQ(nullptr, cylinder->Parent());

  visual->RemoveGeometries();
  EXPECT_EQ(0u, visual->GeometryCount());
  EXPECT_FALSE(sphere->HasParent());
  EXPECT_EQ(nullptr, sphere->Parent());

  // remove non-existent geometries
  GeometryPtr removed3 = visual->RemoveGeometryByIndex(0u);
  EXPECT_EQ(nullptr, removed3);

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
TEST_P(VisualTest, Geometry)
{
  Geometry(GetParam());
}

/////////////////////////////////////////////////
void VisualTest::VisibilityFlags(const std::string &_renderEngine)
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

  ScenePtr scene = engine->CreateScene("scene5");

  // create visual
  VisualPtr visual = scene->CreateVisual();
  ASSERT_NE(nullptr, visual);

  // check initial value
  EXPECT_EQ(static_cast<uint32_t>(IGN_VISIBILITY_ALL),
      visual->VisibilityFlags());

  // check setting new values
  visual->SetVisibilityFlags(0x00000001u);
  EXPECT_EQ(0x00000001u, visual->VisibilityFlags());

  visual->AddVisibilityFlags(0x10000000u);
  EXPECT_EQ(0x10000001u, visual->VisibilityFlags());

  visual->RemoveVisibilityFlags(0x00000001u);
  EXPECT_EQ(0x10000000u, visual->VisibilityFlags());

  VisualPtr visual2 = scene->CreateVisual();
  ASSERT_NE(nullptr, visual2);
  EXPECT_EQ(static_cast<uint32_t>(IGN_VISIBILITY_ALL),
      visual2->VisibilityFlags());

  // check setting visibility flags to visual with child nodes
  visual->AddChild(visual2);

  visual->SetVisibilityFlags(0x00000001u);
  EXPECT_EQ(0x00000001u, visual->VisibilityFlags());
  EXPECT_EQ(0x00000001u, visual2->VisibilityFlags());

  visual->AddVisibilityFlags(0x01000000u);
  EXPECT_EQ(0x01000001u, visual->VisibilityFlags());
  EXPECT_EQ(0x01000001u, visual2->VisibilityFlags());

  visual->RemoveVisibilityFlags(0x00000001u);
  EXPECT_EQ(0x01000000u, visual->VisibilityFlags());
  EXPECT_EQ(0x01000000u, visual2->VisibilityFlags());

  // set child node's visibility flag only
  visual2->SetVisibilityFlags(0x00000010u);
  EXPECT_EQ(0x01000000u, visual->VisibilityFlags());
  EXPECT_EQ(0x00000010u, visual2->VisibilityFlags());

  // Clean up
  engine->DestroyScene(scene);
  rendering::unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
TEST_P(VisualTest, VisibilityFlags)
{
  VisibilityFlags(GetParam());
}

/////////////////////////////////////////////////
void VisualTest::BoundingBox(const std::string &_renderEngine)
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

  ScenePtr scene = engine->CreateScene("scene6");

  // create visual
  VisualPtr visual = scene->CreateVisual();
  ASSERT_NE(nullptr, visual);

  // Add geometries
  GeometryPtr box = scene->CreateBox();
  visual->AddGeometry(box);
  visual->SetWorldPosition(1.0, 2.0, 3.0);

  ignition::math::AxisAlignedBox localBoundingBox = visual->LocalBoundingBox();
  ignition::math::AxisAlignedBox boundingBox = visual->BoundingBox();

  EXPECT_EQ(ignition::math::Vector3d(-0.5, -0.5, -0.5), localBoundingBox.Min());
  EXPECT_EQ(ignition::math::Vector3d(0.5, 0.5, 0.5), localBoundingBox.Max());
  EXPECT_EQ(ignition::math::Vector3d(0.5, 1.5, 2.5), boundingBox.Min());
  EXPECT_EQ(ignition::math::Vector3d(1.5, 2.5, 3.5), boundingBox.Max());

  // Clean up
  engine->DestroyScene(scene);
  rendering::unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
TEST_P(VisualTest, BoundingBox)
{
  BoundingBox(GetParam());
}

/////////////////////////////////////////////////
void VisualTest::Wireframe(const std::string &_renderEngine)
{
#ifdef __APPLE__                                                                                                                                                                              
  std::cerr << "Skipping test for apple, see issue #847." << std::endl;
  return;
#endif
  RenderEngine *engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    FAIL() << "Engine '" << _renderEngine
              << "' is not supported\n";
    return;
  }

  ScenePtr scene = engine->CreateScene("scene7");

  // create visual
  VisualPtr visual = scene->CreateVisual();
  ASSERT_NE(nullptr, visual);
  EXPECT_EQ(false, visual->Wireframe());

  // set wireframe
  visual->SetWireframe(true);
  EXPECT_EQ(true, visual->Wireframe());
}

/////////////////////////////////////////////////
TEST_P(VisualTest, Wireframe)
{
  Wireframe(GetParam());
}

/////////////////////////////////////////////////
void VisualTest::Clone(const std::string &_renderEngine)
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

  ScenePtr scene = engine->CreateScene("scene8");
  ASSERT_NE(nullptr, scene);

  VisualPtr parent = scene->CreateVisual();
  ASSERT_NE(nullptr, parent);

  // add descendant visuals (one child, one grandchild)
  VisualPtr child = scene->CreateVisual();
  ASSERT_NE(nullptr, child);
  parent->AddChild(child);
  VisualPtr grandChild = scene->CreateVisual();
  ASSERT_NE(nullptr, grandChild);
  child->AddChild(grandChild);
  EXPECT_EQ(1u, parent->ChildCount());
  EXPECT_EQ(1u, child->ChildCount());
  EXPECT_EQ(0u, grandChild->ChildCount());

  // create geometries
  GeometryPtr parentBox = scene->CreateBox();
  parent->AddGeometry(parentBox);
  GeometryPtr childCylinder = scene->CreateCylinder();
  child->AddGeometry(childCylinder);
  GeometryPtr grandChildSphere = scene->CreateSphere();
  grandChild->AddGeometry(grandChildSphere);

  // create material
  math::Color ambient(0.5f, 0.2f, 0.4f, 1.0f);
  math::Color diffuse(0.1f, 0.9f, 0.3f, 1.0f);
  math::Color specular(0.8f, 0.7f, 0.0f, 1.0f);
  double transparency = 0.3;
  MaterialPtr material = scene->CreateMaterial("unique");
  ASSERT_NE(nullptr, material);
  EXPECT_TRUE(scene->MaterialRegistered("unique"));
  material->SetAmbient(ambient);
  material->SetDiffuse(diffuse);
  material->SetSpecular(specular);
  material->SetTransparency(transparency);
  // this SetMaterial call applies the material to all of the geometries for the
  // visual (including geometries of child visuals)
  parent->SetMaterial(material);

  // set scale
  const math::Vector3d parentLocalScale(1, 2, 3);
  parent->SetLocalScale(parentLocalScale);
  const math::Vector3d childLocalScale(4, 5, 6);
  child->SetLocalScale(childLocalScale);
  child->SetInheritScale(true);
  const math::Vector3d grandChildLocalScale(7, 8, 9);
  grandChild->SetLocalScale(grandChildLocalScale);
  grandChild->SetInheritScale(false);

  // set user data
  const auto parentUserData = "parent";
  parent->SetUserData(parentUserData, parentUserData);
  const auto childUserData = "child";
  child->SetUserData(childUserData, childUserData);
  const auto grandChildUserData = "grandChild";
  grandChild->SetUserData(grandChildUserData, grandChildUserData);

  // set visibility flags
  const uint32_t visibilityFlags = 0x00000001u;
  parent->SetVisibilityFlags(visibilityFlags);
  const uint32_t grandChildVisibilityFlags = 0x01000000u;
  grandChild->SetVisibilityFlags(grandChildVisibilityFlags);
  EXPECT_EQ(visibilityFlags, parent->VisibilityFlags());
  EXPECT_EQ(visibilityFlags, child->VisibilityFlags());
  EXPECT_EQ(grandChildVisibilityFlags, grandChild->VisibilityFlags());

  // set pose
  const math::Pose3d parentPose(1, 1, 1, 0, 0, 0);
  parent->SetWorldPose(parentPose);
  const math::Pose3d grandChildPoseOffset(1, 1, 1, 0, 0, 1);
  grandChild->SetLocalPose(grandChildPoseOffset);
  EXPECT_EQ(parentPose, parent->WorldPose());
  EXPECT_EQ(parentPose, child->WorldPose());
  EXPECT_EQ(parentPose * grandChildPoseOffset, grandChild->WorldPose());

  // set wireframe
  parent->SetWireframe(true);
  child->SetWireframe(false);
  grandChild->SetWireframe(true);

  // clone the parent visual
  const auto preCloneNodeCount = scene->NodeCount();
  const auto clonedVisualName = "clonedVisual";
  auto clonedVisual = parent->Clone(clonedVisualName, parent->Parent());
  ASSERT_NE(nullptr, clonedVisual);
  EXPECT_GT(scene->NodeCount(), preCloneNodeCount);

  // check the clone
  EXPECT_EQ(clonedVisualName, clonedVisual->Name());
  EXPECT_NE(clonedVisual->Name(), parent->Name());
  EXPECT_NE(clonedVisual->Id(), parent->Id());
  EXPECT_EQ(clonedVisual->Scene(), parent->Scene());
  EXPECT_EQ(clonedVisual->ChildCount(), parent->ChildCount());
  EXPECT_EQ(clonedVisual->LocalScale(), parent->LocalScale());
  EXPECT_EQ(clonedVisual->WorldScale(), parent->WorldScale());
  EXPECT_EQ(clonedVisual->UserData(parentUserData),
      parent->UserData(parentUserData));
  EXPECT_EQ(clonedVisual->VisibilityFlags(), parent->VisibilityFlags());
  EXPECT_EQ(clonedVisual->WorldPose(), parent->WorldPose());
  EXPECT_EQ(clonedVisual->LocalPose(), parent->LocalPose());
  EXPECT_EQ(clonedVisual->Wireframe(), parent->Wireframe());

  // compare materials (the material is cloned, so the name is different but
  // the properties of the material are the same)
  auto clonedVisualMaterial = clonedVisual->Material();
  auto originalVisualMaterial = parent->Material();
  ASSERT_NE(nullptr, clonedVisualMaterial);
  ASSERT_NE(nullptr, originalVisualMaterial);
  EXPECT_NE(clonedVisualMaterial, originalVisualMaterial);
  EXPECT_NE(clonedVisualMaterial->Name(), originalVisualMaterial->Name());
  EXPECT_EQ(clonedVisualMaterial->Type(), originalVisualMaterial->Type());
  EXPECT_EQ(clonedVisualMaterial->Ambient(), originalVisualMaterial->Ambient());
  EXPECT_EQ(clonedVisualMaterial->Diffuse(), originalVisualMaterial->Diffuse());
  EXPECT_EQ(clonedVisualMaterial->Specular(),
      originalVisualMaterial->Specular());
  EXPECT_DOUBLE_EQ(clonedVisualMaterial->Transparency(),
      originalVisualMaterial->Transparency());

  // TODO(anyone) compare child visuals that were cloned

  // clean up
  engine->DestroyScene(scene);
  rendering::unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
TEST_P(VisualTest, Clone)
{
  Clone(GetParam());
}


INSTANTIATE_TEST_CASE_P(Visual, VisualTest,
    RENDER_ENGINE_VALUES,
    PrintToStringParam());

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
