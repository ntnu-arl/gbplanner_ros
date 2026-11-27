/*
 * Copyright (C) 2021 Rhys Mainwaring
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

// The functions BuildScene and createCamera are copied from the simple_demo
// example.

#include "IgnitionRenderer.hh"

#include <QOpenGLContext>
#include <QQuickWindow>

#include <ignition/common/Console.hh>

#include <iostream>

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
void BuildScene(ignition::rendering::ScenePtr _scene)
{
  // initialize _scene
  _scene->SetAmbientLight(0.3, 0.3, 0.3);
  VisualPtr root = _scene->RootVisual();

  // create directional light
  DirectionalLightPtr light0 = _scene->CreateDirectionalLight();
  light0->SetDirection(-0.5, 0.5, -1);
  light0->SetDiffuseColor(0.5, 0.5, 0.5);
  light0->SetSpecularColor(0.5, 0.5, 0.5);
  root->AddChild(light0);

  // create point light
  PointLightPtr light2 = _scene->CreatePointLight();
  light2->SetDiffuseColor(0.5, 0.5, 0.5);
  light2->SetSpecularColor(0.5, 0.5, 0.5);
  light2->SetLocalPosition(3, 5, 5);
  root->AddChild(light2);

  // create green material
  MaterialPtr green = _scene->CreateMaterial();
  green->SetAmbient(0.0, 0.5, 0.0);
  green->SetDiffuse(0.0, 0.7, 0.0);
  green->SetSpecular(0.5, 0.5, 0.5);
  green->SetShininess(50);
  green->SetReflectivity(0);

  // create center visual
  VisualPtr center = _scene->CreateVisual();
  center->AddGeometry(_scene->CreateSphere());
  center->SetLocalPosition(3, 0, 0);
  center->SetLocalScale(0.1, 0.1, 0.1);
  center->SetMaterial(green);
  root->AddChild(center);

  // create red material
  MaterialPtr red = _scene->CreateMaterial();
  red->SetAmbient(0.5, 0.0, 0.0);
  red->SetDiffuse(1.0, 0.0, 0.0);
  red->SetSpecular(0.5, 0.5, 0.5);
  red->SetShininess(50);
  red->SetReflectivity(0);
  red->SetRenderOrder(3);

  // create sphere visual
  VisualPtr sphere = _scene->CreateVisual();
  sphere->AddGeometry(_scene->CreateSphere());
  sphere->SetOrigin(0.0, -0.5, 0.0);
  sphere->SetLocalPosition(3, 0, 0);
  sphere->SetLocalRotation(0, 0, 0);
  sphere->SetLocalScale(1, 1, 1);
  sphere->SetMaterial(red);
  root->AddChild(sphere);

  // create blue material
  MaterialPtr blue = _scene->CreateMaterial();
  blue->SetAmbient(0.0, 0.0, 0.3);
  blue->SetDiffuse(0.0, 0.0, 0.8);
  blue->SetSpecular(0.5, 0.5, 0.5);
  blue->SetShininess(50);
  blue->SetReflectivity(0);

  // create box visual
  VisualPtr box = _scene->CreateVisual();
  box->AddGeometry(_scene->CreateBox());
  box->SetOrigin(0.0, 0.5, 0.0);
  box->SetLocalPosition(3, 0, 0);
  box->SetLocalRotation(IGN_PI / 4, 0, IGN_PI / 3);
  box->SetLocalScale(1, 2.5, 1);
  box->SetMaterial(blue);
  root->AddChild(box);

  // create ellipsoid visual
  VisualPtr ellipsoidVisual = _scene->CreateVisual();
  auto ellipsoid = _scene->CreateSphere();
  ellipsoidVisual->SetLocalScale(1.2, 0.7, 0.5);
  ellipsoidVisual->AddGeometry(ellipsoid);
  ellipsoidVisual->SetLocalPosition(3, -1, 0);
  ellipsoidVisual->SetMaterial(green);
  root->AddChild(ellipsoidVisual);

  // create white material
  MaterialPtr white = _scene->CreateMaterial();
  white->SetAmbient(0.5, 0.5, 0.5);
  white->SetDiffuse(0.8, 0.8, 0.8);
  white->SetReceiveShadows(true);
  white->SetReflectivity(0);
  white->SetRenderOrder(0);

  VisualPtr capsuleVisual = _scene->CreateVisual();
  CapsulePtr capsule = _scene->CreateCapsule();
  capsule->SetLength(0.2);
  capsule->SetRadius(0.2);
  capsuleVisual->AddGeometry(capsule);
  capsuleVisual->SetOrigin(0.0, 0.0, 0.0);
  capsuleVisual->SetLocalPosition(4, 2, 0);
  capsuleVisual->SetLocalScale(1, 1, 1);
  capsuleVisual->SetMaterial(red);
  root->AddChild(capsuleVisual);

  // create plane visual
  VisualPtr plane = _scene->CreateVisual();
  plane->AddGeometry(_scene->CreatePlane());
  plane->SetLocalScale(5, 8, 1);
  plane->SetLocalPosition(3, 0, -0.5);
  plane->SetMaterial(white);
  root->AddChild(plane);

  // create plane visual
  VisualPtr plane2 = _scene->CreateVisual();
  plane2->AddGeometry(_scene->CreatePlane());
  plane2->SetLocalScale(5, 8, 1);
  plane2->SetLocalPosition(4, 0.5, -0.5);
  plane2->Scale(0.1, 0.1, 1);
  plane2->SetMaterial(red);
  root->AddChild(plane2);

  // create axis visual
  VisualPtr axis = _scene->CreateAxisVisual();
  axis->SetLocalPosition(4.0, 0.5, -0.4);
  root->AddChild(axis);

  // create camera
  CameraPtr camera = _scene->CreateCamera("camera");
  camera->SetLocalPosition(0.0, 0.0, 0.0);
  camera->SetLocalRotation(0.0, 0.0, 0.0);
  camera->SetImageWidth(800);
  camera->SetImageHeight(600);
  camera->SetAntiAliasing(2);
  camera->SetAspectRatio(800.0/600.0);
  camera->SetHFOV(IGN_PI / 2);
  root->AddChild(camera);

  // track target
  camera->SetTrackTarget(box);
}

//////////////////////////////////////////////////
ignition::rendering::CameraPtr CreateCamera(const std::string &_engineName)
{
  // create and populate scene
  std::map<std::string, std::string> params;

  // ensure that the QML application and Ignition / Ogre2 share an OpenGL
  // context
  params["useCurrentGLContext"] = "1";
  RenderEngine *engine = rendering::engine(_engineName, params);
  if (!engine)
  {
    std::cout << "Engine '" << _engineName
                << "' is not supported" << std::endl;
    return CameraPtr();
  }
  ScenePtr scene = engine->CreateScene("scene");
  BuildScene(scene);

  // return camera sensor
  SensorPtr sensor = scene->SensorByName("camera");
  return std::dynamic_pointer_cast<Camera>(sensor);
}

//////////////////////////////////////////////////
IgnitionRenderer::~IgnitionRenderer()
{
}

//////////////////////////////////////////////////
IgnitionRenderer::IgnitionRenderer()
{
}

//////////////////////////////////////////////////
void IgnitionRenderer::Initialise()
{
  // no-op - all initialised on the main thread
}

//////////////////////////////////////////////////
void IgnitionRenderer::InitialiseOnMainThread()
{
  if (!this->initialised)
  {
    this->InitEngine();
    this->initialised = true;
  }
}

//////////////////////////////////////////////////
void IgnitionRenderer::Render()
{
  // pre-render may regenerate textureId if the size changes
  this->camera->PreRender();
  this->textureId = this->camera->RenderTextureGLId();

  // render to texture
  this->camera->Update();

  // Move camera
  this->UpdateCamera();
}

//////////////////////////////////////////////////
bool IgnitionRenderer::Initialised() const
{
  return this->initialised;
}

//////////////////////////////////////////////////
unsigned int IgnitionRenderer::TextureId() const
{
  return this->textureId;
}

//////////////////////////////////////////////////
QSize IgnitionRenderer::TextureSize() const
{
  return this->textureSize;
}

//////////////////////////////////////////////////
void IgnitionRenderer::InitEngine()
{
  std::string engineName("ogre2");

  common::Console::SetVerbosity(4);

  try
  {
    this->camera = CreateCamera(engineName);
  }
  catch (...)
  {
    std::cerr << "Error starting up: " << engineName << std::endl;
  }

  if (!this->camera)
  {
    ignerr << "No cameras found. Scene will not be rendered" << std::endl;
    return;
  }

  // quick check on sizing...
  ignmsg << "imageW: " << this->camera->ImageWidth() << "\n";
  ignmsg << "imageH: " << this->camera->ImageHeight() << "\n";

  // pre-render will force texture creation and may update texture id
  this->camera->PreRender();
  this->textureId = this->camera->RenderTextureGLId();
}

//////////////////////////////////////////////////
void IgnitionRenderer::UpdateCamera()
{
  double angle = this->cameraOffset / 2 * M_PI;
  double x = sin(angle) * 3.0 + 3.0;
  double y = cos(angle) * 3.0;
  this->camera->SetLocalPosition(x, y, 0.0);

  this->cameraOffset += 0.0005;
}
