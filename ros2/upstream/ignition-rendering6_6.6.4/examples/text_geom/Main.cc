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
#include <gz/rendering.hh>

#include "GlutWindow.hh"

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
void buildScene(ScenePtr _scene)
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


  MaterialPtr grey = _scene->CreateMaterial("Grey");
  grey->SetAmbient(0.7, 0.7, 0.7);
  grey->SetDiffuse(0.7, 0.7, 0.7);
  grey->SetSpecular(0.8, 0.8, 0.8);

  // create plane visual
  VisualPtr plane = _scene->CreateVisual();
  plane->AddGeometry(_scene->CreatePlane());
  plane->SetLocalScale(5, 8, 1);
  plane->SetLocalPosition(3, 0, -0.5);
  plane->SetMaterial(grey);
  root->AddChild(plane);

  // create green material
  MaterialPtr green = _scene->CreateMaterial();
  green->SetDiffuse(0.0, 0.7, 0.0);

  //! [create text geometry]
  TextPtr textGeom = _scene->CreateText();
  textGeom->SetFontName("Liberation Sans");
  textGeom->SetTextString("Hello World!");
  textGeom->SetShowOnTop(true);
  textGeom->SetTextAlignment(TextHorizontalAlign::LEFT,
                             TextVerticalAlign::BOTTOM);

  // create text visual
  VisualPtr text = _scene->CreateVisual();
  text->AddGeometry(textGeom);
  text->SetLocalPosition(3, 2, 0);
  text->SetLocalRotation(0, 0, 0);
  // Only diffuse component will be used!
  text->SetMaterial(green);
  root->AddChild(text);
  //! [create text geometry]

  // create camera
  CameraPtr camera = _scene->CreateCamera("camera");
  camera->SetLocalPosition(0.0, 0.0, 0.0);
  camera->SetLocalRotation(0.0, 0.0, 0.0);
  camera->SetImageWidth(800);
  camera->SetImageHeight(600);
  camera->SetAntiAliasing(2);
  camera->SetAspectRatio(1.333);
  camera->SetHFOV(IGN_PI / 2);
  root->AddChild(camera);
}

//////////////////////////////////////////////////
CameraPtr createCamera(const std::string &_engineName)
{
  // create and populate scene
  RenderEngine *engine = rendering::engine(_engineName);
  if (!engine)
  {
    std::cout << "Engine '" << _engineName
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

  common::Console::SetVerbosity(4);
  std::vector<std::string> engineNames;
  std::vector<CameraPtr> cameras;

  engineNames.push_back("ogre");
  engineNames.push_back("optix");
  for (auto engineName : engineNames)
  {
    try
    {
      CameraPtr camera = createCamera(engineName);
      if (camera)
        cameras.push_back(camera);
      else
        std::cerr << "Null camera!" << std::endl;
    }
    catch (...)
    {
      // std::cout << ex.what() << std::endl;
      std::cerr << "Error starting up: " << engineName << std::endl;
    }
    run(cameras);
  }
  return 0;
}
