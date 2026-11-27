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

#if defined(__APPLE__)
  #include <OpenGL/gl.h>
  #include <GLUT/glut.h>
#elif not defined(_WIN32)
  #include <GL/glew.h>
  #include <GL/gl.h>
  #include <GL/glut.h>
#endif

#include <iostream>
#include <gazebo/common/Console.hh>
#include <gazebo/transport/TransportIface.hh>

#include <gz/rendering/Camera.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>

#include "CameraWindow.hh"
#include "SceneManager.hh"

using namespace gz;
using namespace rendering;

void Connect()
{
  gazebo::common::Console::SetQuiet(false);
  gazebo::transport::init();
  gazebo::transport::run();

  SceneManager *manager = SceneManager::Instance();
  manager->Load();
  manager->Init();
}

ScenePtr CreateScene(const std::string &_engine)
{
  RenderEngine *engine = rendering::engine(_engine);
  if (!engine)
  {
    std::cout << "Engine '" << _engine
              << "' is not supported" << std::endl;
    return ScenePtr();
  }

  ScenePtr scene = engine->CreateScene("scene");
  SceneManager::Instance()->AddScene(scene);
  return scene;
}

CameraPtr CreateCamera(const std::string &_engine)
{
  ScenePtr scene = CreateScene(_engine);
  if (!scene)
    return CameraPtr();
  VisualPtr root = scene->RootVisual();

  CameraPtr camera = scene->CreateCamera("camera");
  camera->SetLocalPosition(5.0, -5.0, 2.0);
  camera->SetLocalRotation(0.0, 0.27, 2.36);
  camera->SetImageWidth(640);
  camera->SetImageHeight(480);
  camera->SetAntiAliasing(2);
  camera->SetAspectRatio(1.333);
  camera->SetHFOV(IGN_PI / 2);
  root->AddChild(camera);

  return camera;
}

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

  Connect();
  std::vector<CameraPtr> cameras;
  std::vector<std::string> engineNames;

  engineNames.push_back(ogreEngineName);
  engineNames.push_back("optix");

  for (auto engineName : engineNames)
  {
    CameraPtr camera = CreateCamera(engineName);
    if (camera)
      cameras.push_back(camera);
  }

  GlutRun(cameras);
  return 0;
}
