/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#if defined(__APPLE__)
  #include <OpenGL/gl.h>
  #include <GLUT/glut.h>
#elif not defined(_WIN32)
  #include <GL/glew.h>
  #include <GL/gl.h>
  #include <GL/glut.h>
#endif

#include <iostream>
#include <vector>

#include <ignition/common/Console.hh>
#include <ignition/common/MeshManager.hh>
#include <ignition/rendering.hh>
#include <ignition/math.hh>

#include "example_config.hh"
#include "GlutWindow.hh"

using namespace ignition;
using namespace rendering;

const std::string RESOURCE_PATH =
    common::joinPaths(std::string(PROJECT_BINARY_PATH), "media");

uint n_ducks = 0;
uint n_boxes = 0;
uint n_spheres = 0;

//////////////////////////////////////////////////
VisualPtr createDuck(ScenePtr _scene,
  const math::Vector3d &_position, MaterialPtr _material,
  const math::Quaterniond &_rotation = math::Quaterniond(1.5708, 0, 2.0))
{
  n_ducks++;

  // create a mesh
  VisualPtr mesh = _scene->CreateVisual(
    "duck" + std::to_string(n_ducks));
  mesh->SetLocalPosition(_position);
  mesh->SetLocalRotation(_rotation);
  MeshDescriptor descriptor;
  descriptor.meshName = common::joinPaths(RESOURCE_PATH, "duck.dae");
  common::MeshManager *meshManager = common::MeshManager::Instance();
  descriptor.mesh = meshManager->Load(descriptor.meshName);
  MeshPtr meshGeom = _scene->CreateMesh(descriptor);
  mesh->AddGeometry(meshGeom);
  mesh->SetMaterial(_material);
  mesh->SetUserData("label", 5);

  return mesh;
}

//////////////////////////////////////////////////
VisualPtr createBox(ScenePtr _scene,
  const math::Vector3d &_position, MaterialPtr _material,
  const std::string &_name = "",
  const math::Quaterniond &_rotation = math::Quaterniond(0,0,0))
{
  n_boxes++;

  std::string name;
  if (_name == "")
    name = "box" + std::to_string(n_boxes);
  else
    name = _name;

  // create a box
  VisualPtr box = _scene->CreateVisual(name);
  box->SetLocalPosition(_position);
  box->SetLocalRotation(_rotation);
  GeometryPtr boxGeom = _scene->CreateBox();
  box->AddGeometry(boxGeom);
  box->SetMaterial(_material);
  box->SetUserData("label", 2);

  return box;
}

//////////////////////////////////////////////////
VisualPtr createSphere(ScenePtr _scene,
  math::Vector3d _position, MaterialPtr _material,
  math::Quaterniond _rotation = math::Quaterniond(0,0,0))
{
  n_spheres++;

  // create a sphere
  VisualPtr sphere = _scene->CreateVisual(
    "sphere" + std::to_string(n_spheres));
  sphere->SetLocalPosition(_position);
  GeometryPtr sphereGeom = _scene->CreateSphere();
  sphere->AddGeometry(sphereGeom);
  sphere->SetMaterial(_material);
  sphere->SetUserData("label", 3);

  return sphere;
}

//////////////////////////////////////////////////
void buildScene(ScenePtr _scene, BoundingBoxType _type)
{
  // initialize _scene
  _scene->SetAmbientLight(0.3, 0.3, 0.3);
  _scene->SetBackgroundColor(0.3, 0.3, 0.3);
  VisualPtr root = _scene->RootVisual();

  ////////////////////// Lights ///////////////////////
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

  ////////////////////// Material //////////////////////
  // sky blue material
  MaterialPtr skyBlue = _scene->CreateMaterial();
  skyBlue->SetAmbient(0.0, 0.5, 0.5);
  skyBlue->SetDiffuse(0.0, 1.0, 1.0);
  skyBlue->SetShininess(50);
  skyBlue->SetReflectivity(0);
  // blue material
  MaterialPtr blue = _scene->CreateMaterial();
  blue->SetAmbient(0.0, 0.0, 0.5);
  blue->SetDiffuse(0.0, 0.0, 1.0);
  blue->SetSpecular(0.5, 0.5, 0.5);
  blue->SetShininess(50);
  blue->SetReflectivity(0);
  // create green material
  MaterialPtr green = _scene->CreateMaterial();
  green->SetAmbient(0.0, 0.5, 0.0);
  green->SetDiffuse(0.0, 1.0, 0.0);
  green->SetSpecular(0.5, 0.5, 0.5);
  green->SetShininess(50);
  green->SetReflectivity(0);

  ////////////////////// Visuals ///////////////////////
  // create plane visual
  VisualPtr plane = _scene->CreateVisual("plane");
  plane->AddGeometry(_scene->CreatePlane());
  plane->SetLocalScale(5, 8, 1);
  plane->SetLocalPosition(3, 0, -0.5);
  root->AddChild(plane);

  // create a mesh
  auto duck = createDuck(_scene, math::Vector3d(5, 0, 0), skyBlue);
  root->AddChild(duck);

  // create a sphere1
  auto sphere1 = createSphere(_scene, math::Vector3d(3, -1.5, 0), green);
  root->AddChild(sphere1);

  // create a sphere2
  auto sphere2 = createSphere(_scene, math::Vector3d(5, 4, 2), green);
  root->AddChild(sphere2);

  // create boxes
  auto box1 = createBox(_scene, math::Vector3d(3, 2, 0), blue);
  root->AddChild(box1);

  auto box2 = createBox(_scene, math::Vector3d(2, -1, 1), blue);
  box2->SetLocalScale(1.2);
  root->AddChild(box2);

  // create camera
  CameraPtr camera = _scene->CreateCamera("camera");
  camera->SetLocalPosition(0.0, 0.0, 0.0);
  camera->SetLocalRotation(0.0, 0.0, 0.0);
  camera->SetImageWidth(800);
  camera->SetImageHeight(600);
  camera->SetImageFormat(PixelFormat::PF_R8G8B8);
  camera->SetAspectRatio(1.333);
  camera->SetHFOV(IGN_PI / 2);
  camera->SetNearClipPlane(0.001);
  camera->SetFarClipPlane(1000);
  root->AddChild(camera);

  BoundingBoxCameraPtr boundingboxCamera =
    _scene->CreateBoundingBoxCamera("boundingbox_camera");
  boundingboxCamera->SetLocalPose(camera->LocalPose());
  boundingboxCamera->SetTrackTarget(camera);
  boundingboxCamera->SetFollowTarget(camera);
  boundingboxCamera->SetImageWidth(camera->ImageWidth());
  boundingboxCamera->SetImageHeight(camera->ImageHeight());
  boundingboxCamera->SetImageFormat(camera->ImageFormat());
  boundingboxCamera->SetAspectRatio(camera->AspectRatio());
  boundingboxCamera->SetHFOV(camera->HFOV());
  boundingboxCamera->SetNearClipPlane(camera->NearClipPlane());
  boundingboxCamera->SetFarClipPlane(camera->FarClipPlane());
  boundingboxCamera->SetBoundingBoxType(_type);
  root->AddChild(boundingboxCamera);
}

//////////////////////////////////////////////////
std::vector<CameraPtr> createCameras(const std::string &_engineName,
    BoundingBoxType _type)
{
  // create and populate scene
  RenderEngine *engine = rendering::engine(_engineName);
  if (!engine)
  {
    ignwarn << "Engine '" << _engineName
              << "' is not supported" << std::endl;
    return std::vector<CameraPtr>();
  }
  ScenePtr scene = engine->CreateScene("scene");
  buildScene(scene, _type);

  // return camera sensor
  SensorPtr sensor = scene->SensorByName("camera");
  SensorPtr boundingboxSensor = scene->SensorByName("boundingbox_camera");

  std::vector<CameraPtr> cameras;
  cameras.push_back(std::dynamic_pointer_cast<Camera>(sensor));
  cameras.push_back(std::dynamic_pointer_cast<Camera>(boundingboxSensor));

  return cameras;
}

//////////////////////////////////////////////////
int main(int _argc, char** _argv)
{
  glutInit(&_argc, _argv);

  // default to 3D boxes unless user specifies otherwise
  BoundingBoxType bboxType = BoundingBoxType::BBT_BOX3D;
  if (_argc == 2)
  {
    const std::string type3d = "3D";
    const std::string type2dVisible = "2D_visible";
    const std::string type2dFull = "2D_full";
    if (type2dVisible.compare(_argv[1]) == 0)
    {
      bboxType = BoundingBoxType::BBT_VISIBLEBOX2D;
    }
    else if (type2dFull.compare(_argv[1]) == 0)
    {
      bboxType = BoundingBoxType::BBT_FULLBOX2D;
    }
    else if (type3d.compare(_argv[1]) != 0)
    {
      ignerr << "Invalid bounding box type given. Valid options are: "
             << type3d << ", " << type2dVisible << ", or "
             << type2dFull << "\n";
      ignerr << "you entered " << _argv[1] << "\n";
      return -1;
    }
  }

  common::Console::SetVerbosity(4);
  std::vector<std::string> engineNames;
  std::vector<CameraPtr> cameras;

  engineNames.push_back("ogre2");

  for (const auto &engineName : engineNames)
  {
    try
    {
      cameras = createCameras(engineName, bboxType);
    }
    catch (...)
    {
      std::cerr << "Error starting up: " << engineName << std::endl;
      return -1;
    }
  }
  run(cameras);
  return 0;
}
