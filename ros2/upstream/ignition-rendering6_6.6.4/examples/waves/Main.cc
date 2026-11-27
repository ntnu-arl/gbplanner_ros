/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#else
  #include <GL/glew.h>
  #include <GL/glut.h>
  #if _WIN32
    #define NOMINMAX
    #include <windows.h>
    #include <GL/glu.h>
  #else
    #include <GL/gl.h>
  #endif
#endif

#include <iostream>
#include <vector>

#include <ignition/common/Console.hh>
#include <ignition/rendering.hh>

#include "example_config.hh"

#include "GlutWindow.hh"

using namespace ignition;
using namespace rendering;

const std::string vertexShaderGLSL330File = "GerstnerWaves_vs_330.glsl";
const std::string fragmentShaderGLSL330File = "GerstnerWaves_fs_330.glsl";

const std::string RESOURCE_PATH =
    ignition::common::joinPaths(std::string(PROJECT_BINARY_PATH), "media");

//////////////////////////////////////////////////
void buildScene(ScenePtr _scene, const std::string &_engineName)
{
  // initialize _scene
  _scene->SetAmbientLight(0.8, 0.8, 0.8);
  VisualPtr root = _scene->RootVisual();

  // enable sky
  _scene->SetSkyEnabled(true);

  // create directional light
  DirectionalLightPtr light0 = _scene->CreateDirectionalLight();
  light0->SetDirection(-0.5, 0.5, -1);
  light0->SetDiffuseColor(0.5, 0.5, 0.5);
  light0->SetSpecularColor(0.5, 0.5, 0.5);
  root->AddChild(light0);

  std::string vertexShaderFile;
  std::string fragmentShaderFile;

  vertexShaderFile = vertexShaderGLSL330File;
  fragmentShaderFile = fragmentShaderGLSL330File;

  // create shader materials
  // path to look for vertex and fragment shader parameters
  std::string vertexShaderPath = ignition::common::joinPaths(
      RESOURCE_PATH, vertexShaderFile);

  std::string fragmentShaderPath = ignition::common::joinPaths(
      RESOURCE_PATH, fragmentShaderFile);

  // create shader material
  ignition::rendering::MaterialPtr shader = _scene->CreateMaterial();
  shader->SetVertexShader(vertexShaderPath);
  shader->SetFragmentShader(fragmentShaderPath);

   // create waves visual
   VisualPtr waves = _scene->CreateVisual("waves");
   MeshDescriptor descriptor;
   descriptor.meshName = common::joinPaths(RESOURCE_PATH, "mesh.dae");
   common::MeshManager *meshManager = common::MeshManager::Instance();
   descriptor.mesh = meshManager->Load(descriptor.meshName);
   MeshPtr meshGeom = _scene->CreateMesh(descriptor);
   waves->AddGeometry(meshGeom);
   waves->SetLocalPosition(3, 0, 0);
   waves->SetLocalScale(1, 1, 1);
   waves->SetMaterial(shader);
   root->AddChild(waves);

  // create camera
  CameraPtr camera = _scene->CreateCamera("camera");
  camera->SetLocalPosition(0, 0.0, 3.5);
  camera->SetLocalRotation(0.0, 0.0, 0.0);
  camera->SetImageWidth(800);
  camera->SetImageHeight(600);
  camera->SetAntiAliasing(4);
  camera->SetAspectRatio(1.333);
  camera->SetHFOV(IGN_PI / 2);
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
    std::cout << "Engine '" << _engineName
              << "' is not supported" << std::endl;
    return CameraPtr();
  }
  ScenePtr scene = engine->CreateScene("scene");
  buildScene(scene, _engineName);

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

  for (auto engineName : engineNames)
  {
    try
    {
      std::map<std::string, std::string> params;
      if (engineName.compare("ogre2") == 0)
      {
         if (graphicsApi == GraphicsAPI::METAL)
         {
          // \todo(anyone) uncomment once metal shaders are available
          // params["metal"] = "1";
          ignerr << "Metal shaders are not implemented yet. Using GLSL" << std::endl;
         }
      }
      else
      {
        // todo(anyone) Passing textures to custom shaders is currently
        // only available in ogre2
        engineName = "ogre2";
        ignerr << "Only ogre2 engine is supported. Switching to use ogre2."
               << std::endl;
      }

      CameraPtr camera = createCamera(engineName, params);
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

  run(cameras);
  return 0;
}
