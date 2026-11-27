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
#include <gz/common/Console.hh>
#include "ManualSceneDemo.hh"
#include "SceneBuilder.hh"
#include "DemoWindow.hh"

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
ManualSceneDemo::ManualSceneDemo() :
  builderIndex(0),
  cameraIndex(0)
{
}

//////////////////////////////////////////////////
ManualSceneDemo::~ManualSceneDemo()
{
}

//////////////////////////////////////////////////
unsigned int ManualSceneDemo::SceneCount() const
{
  return this->builders.size();
}

//////////////////////////////////////////////////
void ManualSceneDemo::AddScene(SceneBuilderPtr _builder)
{
  this->builders.push_back(_builder);
}

//////////////////////////////////////////////////
void ManualSceneDemo::PrevScene()
{
  this->SelectScene(this->builderIndex - 1);
}

//////////////////////////////////////////////////
void ManualSceneDemo::NextScene()
{
  this->SelectScene(this->builderIndex + 1);
}

//////////////////////////////////////////////////
void ManualSceneDemo::SelectScene(int _index)
{
  int count = this->SceneCount();
  _index += (_index < 0) ? count : 0;
  this->builderIndex = _index % count;
  this->ChangeScene();
}

//////////////////////////////////////////////////
SceneBuilderPtr ManualSceneDemo::CurrentScene()
{
  return this->builders[this->builderIndex];
}

//////////////////////////////////////////////////
unsigned int ManualSceneDemo::CameraCount() const
{
  return this->cameras.size();
}

//////////////////////////////////////////////////
void ManualSceneDemo::AddCamera(const std::string &_engineName,
    const std::map<std::string, std::string>& _params)
{
  RenderEngine *engine = rendering::engine(_engineName, _params);
  if (!engine)
  {
    std::cout << "Engine '" << _engineName
              << "' is not supported" << std::endl;
    return;
  }

  ScenePtr scene = engine->CreateScene("Scene");
  VisualPtr root = scene->RootVisual();

  CameraPtr camera = scene->CreateCamera("Camera");
  camera->SetImageWidth(640);
  camera->SetImageHeight(480);
  camera->SetAntiAliasing(2);
  camera->SetAspectRatio(1.333);
  camera->SetHFOV(IGN_PI / 2);
  root->AddChild(camera);

  this->scenes.push_back(scene);
  this->cameras.push_back(camera);
}

//////////////////////////////////////////////////
void ManualSceneDemo::PrevCamera()
{
  this->SelectCamera(this->cameraIndex - 1);
}

//////////////////////////////////////////////////
void ManualSceneDemo::NextCamera()
{
  this->SelectCamera(this->cameraIndex + 1);
}

//////////////////////////////////////////////////
void ManualSceneDemo::SelectCamera(int _index)
{
  int count = this->CameraCount();
  _index += (_index < 0) ? count : 0;
  this->cameraIndex = _index % count;
}

//////////////////////////////////////////////////
CameraPtr ManualSceneDemo::Camera(int _index)
{
  unsigned int i = _index % this->cameras.size();
  return this->cameras[i];
}

//////////////////////////////////////////////////
CameraPtr ManualSceneDemo::CurrentCamera()
{
  return this->cameras[this->cameraIndex];
}

//////////////////////////////////////////////////
void ManualSceneDemo::Run()
{
  for (auto builder : this->builders)
  {
    builder->SetScenes(this->scenes);
    builder->SetCameras(this->cameras);
  }

  this->SelectScene(0);
  this->SelectCamera(0);
  run(shared_from_this());
}

//////////////////////////////////////////////////
void ManualSceneDemo::Update()
{
  this->CurrentScene()->UpdateScenes();
}

//////////////////////////////////////////////////
void ManualSceneDemo::ChangeScene()
{
  SceneBuilderPtr builder = this->CurrentScene();
  builder->ResetCameras();
  builder->BuildScenes();
}

//////////////////////////////////////////////////
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

  std::map<std::string, std::string> params;
  if (ogreEngineName.compare("ogre2") == 0
      && graphicsApi == GraphicsAPI::METAL)
  {
    params["metal"] = "1";
  }

  common::Console::SetVerbosity(4);
//! [add scenes]
  ManualSceneDemoPtr sceneDemo(new ManualSceneDemo);
  sceneDemo->AddScene(SceneBuilderPtr(new SceneBuilder));
  sceneDemo->AddScene(SceneBuilderPtr(new SimpleSceneBuilder));
  sceneDemo->AddScene(SceneBuilderPtr(new AllShapesSceneBuilder));
  sceneDemo->AddScene(SceneBuilderPtr(new TextureSceneBuilder));
  sceneDemo->AddScene(SceneBuilderPtr(new NormalMapSceneBuilder));
  sceneDemo->AddScene(SceneBuilderPtr(new ReflectionSceneBuilder));
  sceneDemo->AddScene(SceneBuilderPtr(new NormalReflectionSceneBuilder));
  // sceneDemo->AddScene(SceneBuilderPtr(new TransparencySceneBuilder));
  sceneDemo->AddScene(SceneBuilderPtr(new ShadowSceneBuilder(1)));
  sceneDemo->AddScene(SceneBuilderPtr(new ShadowSceneBuilder(2)));
  sceneDemo->AddScene(SceneBuilderPtr(new ShadowSceneBuilder(3)));
  sceneDemo->AddScene(SceneBuilderPtr(new ShadowSceneBuilder(4)));
  sceneDemo->AddScene(SceneBuilderPtr(new ShadowSceneBuilder(5)));
//! [add scenes]
  sceneDemo->AddCamera(ogreEngineName, params);
  sceneDemo->AddCamera("optix");
  sceneDemo->Run();
  return 0;
}

