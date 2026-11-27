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

#include <gz/common/Console.hh>
#include <gz/common/Skeleton.hh>
#include <gz/common/SkeletonAnimation.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/Mesh.hh>
#include <gz/rendering.hh>

#include "example_config.hh"
#include "GlutWindow.hh"

using namespace gz;
using namespace rendering;

const std::string RESOURCE_PATH =
    common::joinPaths(std::string(PROJECT_BINARY_PATH), "media");

//////////////////////////////////////////////////
void buildScene(ScenePtr _scene, std::vector<VisualPtr> &_visuals,
    common::SkeletonPtr &_skel)
{
  // initialize _scene
  _scene->SetAmbientLight(0.3, 0.3, 0.3);
  _scene->SetBackgroundColor(0.3, 0.3, 0.3);
  VisualPtr root = _scene->RootVisual();

  // create directional light
  DirectionalLightPtr light0 = _scene->CreateDirectionalLight();
  light0->SetDirection(0.5, 0.5, -1);
  light0->SetDiffuseColor(0.8, 0.8, 0.8);
  light0->SetSpecularColor(0.5, 0.5, 0.5);
  root->AddChild(light0);

  // create a visual for the actor, attach mesh and get skeleton
  // Skeleton will be animated by GlutWindow

  //! [load mesh]
  ignmsg << "Creating mesh with animations..." << std::endl;
  MeshDescriptor descriptor;
  descriptor.meshName = common::joinPaths(RESOURCE_PATH, "walk.dae");
  common::MeshManager *meshManager = common::MeshManager::Instance();
  descriptor.mesh = meshManager->Load(descriptor.meshName);
  //! [load mesh]

  // add bvh animation
  //! [add animation]
  std::string bvhFile = common::joinPaths(RESOURCE_PATH, "cmu-13_26.bvh");
  double scale = 0.055;
  _skel = descriptor.mesh->MeshSkeleton();
  _skel->AddBvhAnimation(bvhFile, scale);
  if (_skel->AnimationCount() == 0)
  {
    ignerr << "Failed to load animation." << std::endl;
    return;
  }
  ignmsg << "Loaded animations: " << std::endl;
  for (unsigned int i = 0; i < _skel->AnimationCount(); ++i)
    ignmsg << "  * " << _skel->Animation(i)->Name() << std::endl;
  //! [add animation]

  unsigned int size = 25;
  double halfSize = size * 0.5;
  unsigned int count = 0;
  ignmsg << "Creating " << size*size << " meshes with skeleton animation"
         << std::endl;
  for (unsigned int i = 0; i < size; ++i)
  {
    double x = i + 3;
    for (unsigned int j = 0; j < size; ++j)
    {
      double y = halfSize - j;
      //! [create a visual for the actor]
      std::string actorName = "actor" + std::to_string(count++);
      VisualPtr actorVisual = _scene->CreateVisual(actorName);

      actorVisual->SetLocalPosition(x, y, 0);
      actorVisual->SetLocalRotation(0, 0, 3.14);
      //! [create a visual for the actor]

      //! [create mesh]
      auto mesh = _scene->CreateMesh(descriptor);
      if (!mesh)
      {
        std::cerr << "Failed to load mesh with animation." << std::endl;
        return;
      }
      //! [create mesh]
      //! [attach mesh]
      actorVisual->AddGeometry(mesh);
      root->AddChild(actorVisual);
      //! [attach mesh]

      _visuals.push_back(actorVisual);
    }
  }

  // create gray material
  MaterialPtr gray = _scene->CreateMaterial();
  gray->SetAmbient(0.7, 0.7, 0.7);
  gray->SetDiffuse(0.7, 0.7, 0.7);
  gray->SetSpecular(0.7, 0.7, 0.7);

  // create grid visual
  GridPtr gridGeom = _scene->CreateGrid();
  if (gridGeom)
  {
    VisualPtr grid = _scene->CreateVisual();
    gridGeom->SetCellCount(20);
    gridGeom->SetCellLength(1);
    gridGeom->SetVerticalCellCount(0);
    grid->AddGeometry(gridGeom);
    grid->SetLocalPosition(3, 0, 0.0);
    grid->SetMaterial(gray);
    root->AddChild(grid);
  }

  // create camera
  CameraPtr camera = _scene->CreateCamera("camera");
  camera->SetLocalPosition(0.0, 0.0, 2.0);
  camera->SetLocalRotation(0.0, 0.5, 0.0);
  camera->SetImageWidth(800);
  camera->SetImageHeight(600);
  camera->SetAntiAliasing(2);
  camera->SetAspectRatio(1.333);
  camera->SetHFOV(IGN_PI / 2);
  root->AddChild(camera);
}

//////////////////////////////////////////////////
CameraPtr createCamera(const std::string &_engineName,
    const std::map<std::string, std::string>& _params,
    std::vector<VisualPtr> &_visuals, common::SkeletonPtr &_skel)
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
  buildScene(scene, _visuals, _skel);

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
  std::string ogreEngineName("ogre");
  if (_argc > 1)
  {
    ogreEngineName = _argv[1];
  }

  GraphicsAPI graphicsApi = GraphicsAPI::OPENGL;
  if (_argc > 2)
  {
    graphicsApi = GraphicsAPIUtils::Set(std::string(_argv[2]));
  }

  common::Console::SetVerbosity(4);
  std::vector<std::string> engineNames;
  std::vector<CameraPtr> cameras;

  engineNames.push_back(ogreEngineName);

  std::vector<VisualPtr> visuals;
  ic::SkeletonPtr skel = nullptr;

  for (auto engineName : engineNames)
  {
    std::cout << "Starting engine [" << engineName << "]" << std::endl;
    try
    {
      std::map<std::string, std::string> params;
      if (engineName.compare("ogre2") == 0
          && graphicsApi == GraphicsAPI::METAL)
      {
        params["metal"] = "1";
      }

      CameraPtr camera = createCamera(engineName, params, visuals, skel);
      if (camera)
      {
        cameras.push_back(camera);
      }
    }
    catch (...)
    {
      std::cerr << "Error starting up: " << engineName << std::endl;
    }
  }
  run(cameras, visuals, skel);
  return 0;
}
