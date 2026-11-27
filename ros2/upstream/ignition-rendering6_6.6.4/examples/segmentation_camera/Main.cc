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
#include <string>
#include <vector>

#include <ignition/common/Console.hh>
#include <ignition/common/MeshManager.hh>
#include <ignition/rendering.hh>

#include "example_config.hh"
#include "GlutWindow.hh"

using namespace ignition;
using namespace rendering;

const std::string RESOURCE_PATH =
    common::joinPaths(std::string(PROJECT_BINARY_PATH), "media");

void buildScene(ScenePtr _scene)
{
  // initialize _scene
  _scene->SetAmbientLight(0.3, 0.3, 0.3);
  _scene->SetBackgroundColor(0.3, 0.3, 0.3);
  VisualPtr root = _scene->RootVisual();

  ////////////////////// Visuals ///////////////////////
  // create plane visual
  VisualPtr plane = _scene->CreateVisual("plane");
  plane->AddGeometry(_scene->CreatePlane());
  plane->SetLocalScale(5, 8, 1);
  plane->SetLocalPosition(3, 0, -0.5);
  root->AddChild(plane);

  // create a mesh
  VisualPtr mesh = _scene->CreateVisual();
  mesh->SetLocalPosition(3, 0, 0);
  mesh->SetLocalRotation(1.5708, 0, 2.0);
  MeshDescriptor descriptor;
  descriptor.meshName = common::joinPaths(RESOURCE_PATH, "duck.dae");
  common::MeshManager *meshManager = common::MeshManager::Instance();
  descriptor.mesh = meshManager->Load(descriptor.meshName);
  MeshPtr meshGeom = _scene->CreateMesh(descriptor);
  mesh->AddGeometry(meshGeom);
  mesh->SetUserData("label", 5);
  root->AddChild(mesh);

  // create a box
  VisualPtr box = _scene->CreateVisual("box");
  box->SetLocalPosition(3, 1.5, 0);
  box->SetLocalRotation(0, 0, 0.7);
  GeometryPtr boxGeom = _scene->CreateBox();
  box->AddGeometry(boxGeom);
  box->SetUserData("label", 2);
  root->AddChild(box);

  // create a box
  VisualPtr box2 = _scene->CreateVisual("box2");
  box2->SetLocalPosition(2, -1, 1);
  box2->SetLocalRotation(0, 0.3, 0.7);
  GeometryPtr boxGeom2 = _scene->CreateBox();
  box2->AddGeometry(boxGeom2);
  box2->SetUserData("label", 2);
  root->AddChild(box2);

  // create a sphere
  VisualPtr sphere = _scene->CreateVisual("sphere");
  sphere->SetLocalPosition(3, -1.5, 0);
  GeometryPtr sphereGeom = _scene->CreateSphere();
  sphere->AddGeometry(sphereGeom);
  sphere->SetUserData("label", 3);
  root->AddChild(sphere);

  // create a sphere2
  VisualPtr sphere2 = _scene->CreateVisual("sphere2");
  sphere2->SetLocalPosition(5, 4, 2);
  GeometryPtr sphereGeom2 = _scene->CreateSphere();
  sphere2->AddGeometry(sphereGeom2);
  sphere2->SetUserData("label", 3);
  root->AddChild(sphere2);

  // create camera
  SegmentationCameraPtr camera = _scene->CreateSegmentationCamera("camera");
  camera->SetLocalPosition(0.0, 0.0, 0.5);
  camera->SetLocalRotation(0.0, 0.0, 0.0);
  camera->SetImageWidth(800);
  camera->SetImageHeight(600);
  camera->SetImageFormat(PixelFormat::PF_R8G8B8);
  camera->SetAspectRatio(1.333);
  camera->SetHFOV(IGN_PI / 2);
  camera->EnableColoredMap(true);
  camera->SetSegmentationType(SegmentationType::ST_SEMANTIC);
  root->AddChild(camera);
}

//////////////////////////////////////////////////
CameraPtr createCamera(const std::string &_engineName,
    const std::map<std::string, std::string>& _params)
{
  // create and populate scene
  RenderEngine *engine = rendering::engine(_engineName, _params);
  if (!engine)
  {
    ignwarn << "Engine '" << _engineName
              << "' is not supported" << std::endl;
    return CameraPtr();
  }
  ScenePtr scene = engine->CreateScene("scene");
  buildScene(scene);

  // return camera sensor
  SensorPtr sensor = scene->SensorByName("camera");
  return std::dynamic_pointer_cast<Camera>(sensor);
}

//////////////////////////////////////////////////
int main(int _argc, char** _argv)
{
  glutInit(&_argc, _argv);

  // Expose engine name to command line because we can't instantiate both
  // ogre and ogre2 at the same time
  std::string engineName("ogre2");
  if (_argc > 1)
  {
    engineName = _argv[1];
  }

  GraphicsAPI graphicsApi = GraphicsAPI::OPENGL;
  if (_argc > 2)
  {
    graphicsApi = GraphicsAPIUtils::Set(std::string(_argv[2]));
  }

  common::Console::SetVerbosity(4);
  try
  {
    std::map<std::string, std::string> params;
    if (engineName.compare("ogre2") == 0
        && graphicsApi == GraphicsAPI::METAL)
    {
      params["metal"] = "1";
    }

    CameraPtr camera = createCamera(engineName, params);
    if (camera)
    {
      run(camera);
    }
  }
  catch (...)
  {
    std::cerr << "Error starting up: " << engineName << std::endl;
  }

  return 0;
}
