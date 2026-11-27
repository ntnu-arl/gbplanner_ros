/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "example_config.hh"
#include "SceneBuilder.hh"

using namespace gz;
using namespace rendering;

const std::string RESOURCE_PATH = std::string(PROJECT_BINARY_PATH) + "/media";


//////////////////////////////////////////////////
//////////////////////////////////////////////////

unsigned int SceneBuilder::tick = 0;

//////////////////////////////////////////////////
SceneBuilder::SceneBuilder()
{
}

//////////////////////////////////////////////////
SceneBuilder::~SceneBuilder()
{
}

//////////////////////////////////////////////////
void SceneBuilder::SetScenes(const SceneList &_scenes)
{
  this->scenes = _scenes;
  this->RegisterMaterials();
}

//////////////////////////////////////////////////
void SceneBuilder::SetCameras(const CameraList &_cameras)
{
  this->cameras = _cameras;
}

//////////////////////////////////////////////////
void SceneBuilder::BuildScenes()
{
  for (auto scene : this->scenes)
  {
    this->ClearScene(scene);
    this->BuildScene(scene);
  }
}

//////////////////////////////////////////////////
void SceneBuilder::ResetCameras()
{
  for (auto camera : this->cameras)
  {
    this->ResetCamera(camera);
  }
}

//////////////////////////////////////////////////
void SceneBuilder::UpdateScenes()
{
  for (auto scene : this->scenes)
  {
    this->UpdateScene(scene);
  }

  ++tick;
}

//////////////////////////////////////////////////
void SceneBuilder::ClearScene(ScenePtr _scene)
{
  _scene->DestroyLights();
  _scene->DestroyVisuals();
}

//////////////////////////////////////////////////
void SceneBuilder::BuildScene(ScenePtr _scene)
{
  _scene->SetBackgroundColor(0.2, 0.2, 0.2);
}

//////////////////////////////////////////////////
void SceneBuilder::ResetCamera(CameraPtr)
{
  // do nothing by default
}

//////////////////////////////////////////////////
void SceneBuilder::UpdateScene(ScenePtr)
{
  // do nothing by default
}

//////////////////////////////////////////////////
void SceneBuilder::RegisterMaterials()
{
  for (auto scene : this->scenes)
  {
    this->RegisterMaterials(scene);
  }
}

//////////////////////////////////////////////////
void SceneBuilder::RegisterMaterials(ScenePtr _scene)
{
  if (!_scene->MaterialRegistered("Red"))
  {
    MaterialPtr mat = _scene->CreateMaterial("Red");
    mat->SetAmbient(0.3, 0.0, 0.0);
    mat->SetDiffuse(0.8, 0.0, 0.0);
    mat->SetSpecular(0.8, 0.8, 0.8);
    mat->SetShininess(50);
    mat->SetReflectivity(0);
  }

  if (!_scene->MaterialRegistered("White"))
  {
    MaterialPtr mat = _scene->CreateMaterial("White");
    mat->SetAmbient(0.8, 0.8, 0.8);
    mat->SetDiffuse(0.6, 0.6, 0.6);
    mat->SetSpecular(0.8, 0.8, 0.8);
    mat->SetShininess(50);
    mat->SetReflectivity(0);
  }


  if (!_scene->MaterialRegistered("Green"))
  {
    MaterialPtr mat = _scene->CreateMaterial("Green");
    mat->SetAmbient(0.0, 0.3, 0.0);
    mat->SetDiffuse(0.0, 0.8, 0.0);
    mat->SetSpecular(0.8, 0.8, 0.8);
    mat->SetShininess(50);
    mat->SetReflectivity(0);
  }

  if (!_scene->MaterialRegistered("Blue"))
  {
    MaterialPtr mat = _scene->CreateMaterial("Blue");
    mat->SetAmbient(0.0, 0.0, 0.3);
    mat->SetDiffuse(0.0, 0.0, 0.8);
    mat->SetSpecular(0.8, 0.8, 0.8);
    mat->SetShininess(50);
    mat->SetReflectivity(0);
  }

  if (!_scene->MaterialRegistered("Yellow"))
  {
    MaterialPtr mat = _scene->CreateMaterial("Yellow");
    mat->SetAmbient(0.3, 0.3, 0.0);
    mat->SetDiffuse(0.8, 0.8, 0.0);
    mat->SetSpecular(0.8, 0.8, 0.8);
    mat->SetShininess(50);
    mat->SetReflectivity(0);
  }

  std::vector<std::string> baseNames;
  baseNames.push_back("Blue");
  baseNames.push_back("Green");
  baseNames.push_back("Red");
  baseNames.push_back("White");
  baseNames.push_back("Yellow");

  for (auto baseName : baseNames)
  {
    std::string parentName = baseName;
    std::string childName = "Texture" + baseName;

    if (!_scene->MaterialRegistered(childName))
    {
      MaterialPtr mat = _scene->Material(parentName)->Clone(childName);
      mat->SetTexture(RESOURCE_PATH + "/tiles.jpg");
    }
  }

  for (auto baseName : baseNames)
  {
    std::string parentName = "Texture" + baseName;
    std::string childName = "Normal" + baseName;

    if (!_scene->MaterialRegistered(childName))
    {
      MaterialPtr mat = _scene->Material(parentName)->Clone(childName);
      mat->SetNormalMap(
          RESOURCE_PATH + "/brick_normal.jpg");
    }
  }

  for (auto baseName : baseNames)
  {
    std::string parentName = "Texture" + baseName;
    std::string childName = "Reflect" + baseName;

    if (!_scene->MaterialRegistered(childName))
    {
      MaterialPtr mat = _scene->Material(parentName)->Clone(childName);
      mat->SetReflectivity(0.25);
    }
  }

  for (auto baseName : baseNames)
  {
    std::string parentName = "Normal" + baseName;
    std::string childName = "NormalReflect" + baseName;

    if (!_scene->MaterialRegistered(childName))
    {
      MaterialPtr mat = _scene->Material(parentName)->Clone(childName);
      mat->SetReflectivity(0.25);
    }
  }

  for (auto baseName : baseNames)
  {
    std::string parentName = baseName;
    std::string childName = "Trans" + baseName;

    if (!_scene->MaterialRegistered(childName))
    {
      MaterialPtr mat = _scene->Material(parentName)->Clone(childName);
      mat->SetTransparency(0.75);
    }
  }
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////

const std::string SimpleSceneBuilder::LIGHT("Light");

const std::string SimpleSceneBuilder::SPHERE("Sphere");

const std::string SimpleSceneBuilder::PLANE("Plane");

//////////////////////////////////////////////////
SimpleSceneBuilder::SimpleSceneBuilder()
{
}

//////////////////////////////////////////////////
SimpleSceneBuilder::~SimpleSceneBuilder()
{
}

//////////////////////////////////////////////////
//! [build scene SimpleSceneBuilder]
void SimpleSceneBuilder::BuildScene(ScenePtr _scene)
{
  SceneBuilder::BuildScene(_scene);
  VisualPtr root = _scene->RootVisual();
  _scene->SetAmbientLight(0.1, 0.1, 0.1);

  DirectionalLightPtr light = _scene->CreateDirectionalLight(LIGHT);
  light->SetDirection(0.3, 0.5, -1.0);
  light->SetDiffuseColor(0.8, 0.8, 0.8);
  light->SetSpecularColor(0.8, 0.8, 0.8);
  root->AddChild(light);

  VisualPtr sphere = _scene->CreateVisual(SPHERE);
  sphere->AddGeometry(_scene->CreateSphere());
  sphere->SetLocalPosition(3.0, 0.0, 0.5);
  sphere->SetLocalRotation(0.0, 0.0, 0.0);
  sphere->SetMaterial("Red");
  root->AddChild(sphere);

  VisualPtr plane = _scene->CreateVisual(PLANE);
  plane->AddGeometry(_scene->CreatePlane());
  plane->SetLocalPosition(3.0, 0.0, 0.0);
  plane->SetLocalRotation(0.0, 0.0, 0.0);
  plane->SetLocalScale(5.0, 10.5, 1.0);
  plane->SetMaterial("White");
  root->AddChild(plane);
}
//! [build scene SimpleSceneBuilder]
//////////////////////////////////////////////////
void SimpleSceneBuilder::ResetCamera(CameraPtr _camera)
{
  _camera->SetLocalPosition(0.5, 0.0, 1.0);
  _camera->SetLocalRotation(0.0, 0.1, 0.0);
}

//////////////////////////////////////////////////
void SimpleSceneBuilder::UpdateScene(ScenePtr _scene)
{
  SceneBuilder::UpdateScene(_scene);
  double scale = cos(this->tick * 0.05);

  VisualPtr sphere = _scene->VisualByName(SPHERE);
  math::Pose3d pose = sphere->LocalPose();
  pose.Pos().Z(0.6 + scale * 0.1);
  sphere->SetLocalPose(pose);
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////

const std::string AllShapesSceneBuilder::CONE("Cone");

const std::string AllShapesSceneBuilder::CYLINDER("Cylinder");

const std::string AllShapesSceneBuilder::BOX("Box");

//////////////////////////////////////////////////
AllShapesSceneBuilder::AllShapesSceneBuilder()
{
}

//////////////////////////////////////////////////
AllShapesSceneBuilder::~AllShapesSceneBuilder()
{
}

//////////////////////////////////////////////////
//! [build scene AllShapesSceneBuilder]
void AllShapesSceneBuilder::BuildScene(ScenePtr _scene)
{
  SimpleSceneBuilder::BuildScene(_scene);
  VisualPtr root = _scene->RootVisual();

  VisualPtr cylinder = _scene->CreateVisual(CYLINDER);
  cylinder->AddGeometry(_scene->CreateCylinder());
  cylinder->SetLocalPosition(3.3, 1.0, 0.5);
  cylinder->SetLocalRotation(0.0, -0.5, 0.0);
  cylinder->SetLocalScale(0.5, 0.5, 1.5);
  cylinder->SetMaterial("Green");
  root->AddChild(cylinder);

  VisualPtr cone = _scene->CreateVisual(CONE);
  cone->AddGeometry(_scene->CreateCone());
  cone->SetLocalPosition(2.6, -0.8, 1.2);
  cone->SetLocalRotation(-0.5, -0.75, 0.0);
  cone->SetLocalScale(0.8, 0.8, 1.0);
  cone->SetMaterial("Blue");
  root->AddChild(cone);

  VisualPtr box = _scene->CreateVisual(BOX);
  box->AddGeometry(_scene->CreateBox());
  box->SetLocalPosition(2.2, -0.8, 0.2);
  box->SetLocalRotation(IGN_PI / 4, 0.0, IGN_PI / 4);
  box->SetLocalScale(0.5, 0.5, 0.5);
  box->SetMaterial("Yellow");
  root->AddChild(box);
}
//! [build scene AllShapesSceneBuilder]

//////////////////////////////////////////////////
//////////////////////////////////////////////////
TextureSceneBuilder::TextureSceneBuilder()
{
}

//////////////////////////////////////////////////
TextureSceneBuilder::~TextureSceneBuilder()
{
}

//////////////////////////////////////////////////
//! [build scene TextureSceneBuilder]
void TextureSceneBuilder::BuildScene(ScenePtr _scene)
{
  AllShapesSceneBuilder::BuildScene(_scene);

  VisualPtr box = _scene->VisualByName(BOX);
  box->SetMaterial("TextureYellow");

  VisualPtr cone = _scene->VisualByName(CONE);
  cone->SetMaterial("TextureBlue");

  VisualPtr cylinder = _scene->VisualByName(CYLINDER);
  cylinder->SetMaterial("TextureGreen");

  VisualPtr plane = _scene->VisualByName(PLANE);
  plane->SetMaterial("TextureWhite");

  VisualPtr sphere = _scene->VisualByName(SPHERE);
  sphere->SetMaterial("TextureRed");
}
//! [build scene TextureSceneBuilder]

//////////////////////////////////////////////////
//////////////////////////////////////////////////
NormalMapSceneBuilder::NormalMapSceneBuilder()
{
}

//////////////////////////////////////////////////
NormalMapSceneBuilder::~NormalMapSceneBuilder()
{
}

//////////////////////////////////////////////////
//! [build scene NormalMapSceneBuilder]
void NormalMapSceneBuilder::BuildScene(ScenePtr _scene)
{
  TextureSceneBuilder::BuildScene(_scene);

  VisualPtr box = _scene->VisualByName(BOX);
  box->SetMaterial("NormalYellow");

  VisualPtr cone = _scene->VisualByName(CONE);
  cone->SetMaterial("NormalBlue");

  VisualPtr cylinder = _scene->VisualByName(CYLINDER);
  cylinder->SetMaterial("NormalGreen");

  VisualPtr plane = _scene->VisualByName(PLANE);
  plane->SetMaterial("NormalWhite");

  VisualPtr sphere = _scene->VisualByName(SPHERE);
  sphere->SetMaterial("NormalRed");
}
//! [build scene NormalMapSceneBuilder]

//////////////////////////////////////////////////
//////////////////////////////////////////////////
ReflectionSceneBuilder::ReflectionSceneBuilder()
{
}

//////////////////////////////////////////////////
ReflectionSceneBuilder::~ReflectionSceneBuilder()
{
}

//////////////////////////////////////////////////
void ReflectionSceneBuilder::BuildScene(ScenePtr _scene)
{
  TextureSceneBuilder::BuildScene(_scene);

  VisualPtr box = _scene->VisualByName(BOX);
  box->SetMaterial("ReflectYellow");

  VisualPtr cone = _scene->VisualByName(CONE);
  cone->SetMaterial("ReflectBlue");

  VisualPtr cylinder = _scene->VisualByName(CYLINDER);
  cylinder->SetMaterial("ReflectGreen");

  VisualPtr plane = _scene->VisualByName(PLANE);
  plane->SetMaterial("ReflectWhite");

  VisualPtr sphere = _scene->VisualByName(SPHERE);
  sphere->SetMaterial("ReflectRed");
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
NormalReflectionSceneBuilder::NormalReflectionSceneBuilder()
{
}

//////////////////////////////////////////////////
NormalReflectionSceneBuilder::~NormalReflectionSceneBuilder()
{
}

//////////////////////////////////////////////////
void NormalReflectionSceneBuilder::BuildScene(ScenePtr _scene)
{
  NormalMapSceneBuilder::BuildScene(_scene);

  VisualPtr box = _scene->VisualByName(BOX);
  box->SetMaterial("NormalReflectYellow");

  VisualPtr cone = _scene->VisualByName(CONE);
  cone->SetMaterial("NormalReflectBlue");

  VisualPtr cylinder = _scene->VisualByName(CYLINDER);
  cylinder->SetMaterial("NormalReflectGreen");

  VisualPtr plane = _scene->VisualByName(PLANE);
  plane->SetMaterial("NormalReflectWhite");

  VisualPtr sphere = _scene->VisualByName(SPHERE);
  sphere->SetMaterial("NormalReflectRed");
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
TransparencySceneBuilder::TransparencySceneBuilder()
{
}

//////////////////////////////////////////////////
TransparencySceneBuilder::~TransparencySceneBuilder()
{
}

//////////////////////////////////////////////////
void TransparencySceneBuilder::BuildScene(ScenePtr _scene)
{
  TextureSceneBuilder::BuildScene(_scene);
  VisualPtr root = _scene->RootVisual();

  VisualPtr box = _scene->VisualByName(BOX);
  box->SetMaterial("TransYellow");

  VisualPtr cone = _scene->VisualByName(CONE);
  cone->SetMaterial("TransBlue");

  VisualPtr cylinder = _scene->VisualByName(CYLINDER);
  cylinder->SetMaterial("TransGreen");

  VisualPtr sphere = _scene->VisualByName(SPHERE);
  sphere->SetMaterial("TransRed");

  VisualPtr background = _scene->CreateVisual();
  background->AddGeometry(_scene->CreatePlane());
  background->SetLocalPosition(4, 0, 4);
  background->SetLocalRotation(0, -IGN_PI / 2, 0);
  background->SetLocalScale(10, 10, 1);
  background->SetMaterial("TextureWhite");
  root->AddChild(background);
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
ShadowSceneBuilder::ShadowSceneBuilder(unsigned int _n, double _dist) :
  n(_n),
  dist(_dist)
{
}

//////////////////////////////////////////////////
ShadowSceneBuilder::~ShadowSceneBuilder()
{
}

//////////////////////////////////////////////////
void ShadowSceneBuilder::BuildScene(ScenePtr _scene)
{
  SimpleSceneBuilder::BuildScene(_scene);
  _scene->DestroyLightByName(LIGHT);

  PointLightPtr light = _scene->CreatePointLight();
  light->SetLocalPosition(5.5, -2.0, 4.75);
  light->SetDiffuseColor(1.0, 1.0, 1.0);
  light->SetSpecularColor(1.0, 1.0, 1.0);
  this->AddLight(light, _scene);
}

//////////////////////////////////////////////////
void ShadowSceneBuilder::AddLight(PointLightPtr _light, ScenePtr _scene)
{
  VisualPtr root = _scene->RootVisual();
  unsigned int total = this->n * this->n * this->n;
  double step = (this->n == 1) ? 0 : this->dist / (this->n - 1);
  double factor = 1.0 / total;

  math::Vector3d position = _light->LocalPosition();
  double offset = (this->n == 1) ? 0 : this->dist / 2;
  double x0 = position.X() - offset;
  double y0 = position.Y() - offset;
  double z0 = position.Z() - offset;

  math::Color diffuse;
  diffuse = _light->DiffuseColor();
  diffuse = diffuse * factor;

  math::Color specular;
  specular = _light->SpecularColor();
  specular = specular * factor;

  for (unsigned int i = 0; i < this->n; ++i)
  {
    for (unsigned int j = 0; j < this->n; ++j)
    {
      for (unsigned int k = 0; k < this->n; ++k)
      {
        math::Vector3d subPosition;
        subPosition.X(i * step + x0);
        subPosition.Y(j * step + y0);
        subPosition.Z(k * step + z0);

        PointLightPtr sublight = _scene->CreatePointLight();
        sublight->SetLocalPosition(subPosition);
        sublight->SetDiffuseColor(diffuse);
        sublight->SetSpecularColor(specular);
        root->AddChild(sublight);
      }
    }
  }
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////

const std::string BoxSceneBuilder::LIGHT("Light");

const std::string BoxSceneBuilder::BOX("Box");

//////////////////////////////////////////////////
BoxSceneBuilder::BoxSceneBuilder()
{
}

//////////////////////////////////////////////////
BoxSceneBuilder::~BoxSceneBuilder()
{
}

//////////////////////////////////////////////////
void BoxSceneBuilder::BuildScene(ScenePtr _scene)
{
  SceneBuilder::BuildScene(_scene);
  VisualPtr root = _scene->RootVisual();
  _scene->SetAmbientLight(0.1, 0.1, 0.1);

  DirectionalLightPtr light = _scene->CreateDirectionalLight(LIGHT);
  light->SetDirection(0.3, 0.5, -1.0);
  light->SetDiffuseColor(0.8, 0.8, 0.8);
  light->SetSpecularColor(0.8, 0.8, 0.8);
  root->AddChild(light);

  VisualPtr box = _scene->CreateVisual(BOX);
  box->AddGeometry(_scene->CreateBox());
  box->SetLocalPosition(3.0, 0.0, 0.5);
  box->SetLocalRotation(0.0, 0.0, IGN_PI / 4);
  box->SetMaterial("Red");
  root->AddChild(box);
}

//////////////////////////////////////////////////
void BoxSceneBuilder::ResetCamera(CameraPtr _camera)
{
  _camera->SetLocalPosition(0.5, 0.0, 1.0);
  _camera->SetLocalRotation(0.0, 0.1, 0.0);
}

//////////////////////////////////////////////////
void BoxSceneBuilder::UpdateScene(ScenePtr _scene)
{
  SceneBuilder::UpdateScene(_scene);
  double scale = cos(this->tick * 0.05);

  VisualPtr box = _scene->VisualByName(BOX);
  math::Pose3d pose = box->LocalPose();
  pose.Pos().Z(0.6 + scale * 0.1);
  box->SetLocalPose(pose);
}
