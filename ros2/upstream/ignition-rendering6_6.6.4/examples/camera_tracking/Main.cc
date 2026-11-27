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

  // create grid visual
  VisualPtr grid = _scene->CreateVisual();
  GridPtr gridGeom = _scene->CreateGrid();
  gridGeom->SetCellCount(20);
  gridGeom->SetCellLength(1);
  gridGeom->SetVerticalCellCount(0);
  grid->AddGeometry(gridGeom);
  grid->SetLocalPosition(3, 0, 0.0);
  root->AddChild(grid);

  // create camera
  CameraPtr camera = _scene->CreateCamera("camera");
  camera->SetLocalPosition(0.0, 0.0, 3.0);
  camera->SetLocalRotation(0.0, 0.0, 0.0);
  camera->SetImageWidth(800);
  camera->SetImageHeight(600);
  camera->SetAntiAliasing(2);
  camera->SetAspectRatio(1.333);
  camera->SetHFOV(IGN_PI / 2);
  root->AddChild(camera);
}


//////////////////////////////////////////////////
NodePtr createMainNode(ScenePtr _scene)
{
  // create green material
  MaterialPtr green = _scene->CreateMaterial();
  green->SetAmbient(0.0, 0.5, 0.0);
  green->SetDiffuse(0.0, 0.7, 0.0);
  green->SetSpecular(0.5, 0.5, 0.5);
  green->SetShininess(50);
  green->SetReflectivity(0);

  // create box visual
  VisualPtr box = _scene->CreateVisual();
  box->AddGeometry(_scene->CreateBox());
  box->SetLocalPosition(3, 0, 0);
  box->SetMaterial(green);

  VisualPtr root = _scene->RootVisual();
  root->AddChild(box);

  return std::dynamic_pointer_cast<Node>(box);
}

//////////////////////////////////////////////////
CameraPtr createCamera(const std::string &_engineName,
    const std::map<std::string, std::string>& _params)
{
  // create and populate scene
  RenderEngine *engine = rendering::engine(_engineName, _params);
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
  std::vector<NodePtr> nodes;

  engineNames.push_back(ogreEngineName);
  engineNames.push_back("optix");

  for (auto engineName : engineNames)
  {
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
        cameras.push_back(camera);
        NodePtr node = createMainNode(camera->Scene());
        if (node)
          nodes.push_back(node);
        camera->SetTrackTarget(node, math::Vector3d(0.5, 0, 0));
      }
    }
    catch (...)
    {
      // std::cout << ex.what() << std::endl;
      std::cerr << "Error starting up: " << engineName << std::endl;
    }
  }
  run(cameras, nodes);
  return 0;
}
