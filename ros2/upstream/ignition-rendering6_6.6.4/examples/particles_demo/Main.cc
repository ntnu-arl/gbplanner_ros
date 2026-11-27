/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include "example_config.hh"
#include "GlutWindow.hh"

using namespace ignition;
using namespace rendering;

const std::string RESOURCE_PATH =
    common::joinPaths(std::string(PROJECT_BINARY_PATH), "media");

//////////////////////////////////////////////////
void buildScene(ScenePtr _scene)
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

  //! [create a mesh]
  VisualPtr mesh = _scene->CreateVisual();
  mesh->SetLocalPosition(3, 0, 0);
  mesh->SetLocalRotation(1.5708, 0, 2.0);
  MeshDescriptor descriptor;
  descriptor.meshName = common::joinPaths(RESOURCE_PATH, "duck.dae");
  common::MeshManager *meshManager = common::MeshManager::Instance();
  descriptor.mesh = meshManager->Load(descriptor.meshName);
  MeshPtr meshGeom = _scene->CreateMesh(descriptor);
  mesh->AddGeometry(meshGeom);
  root->AddChild(mesh);
  //! [create a mesh]

  // create gray material
  MaterialPtr gray = _scene->CreateMaterial();
  gray->SetAmbient(1, 1, 1);
  gray->SetDiffuse(1, 1, 1);
  gray->SetSpecular(1, 1, 1);

  //! [create grid visual]
  VisualPtr grid = _scene->CreateVisual();
  GridPtr gridGeom = _scene->CreateGrid();
  gridGeom->SetCellCount(20);
  gridGeom->SetCellLength(1);
  gridGeom->SetVerticalCellCount(0);
  grid->AddGeometry(gridGeom);
  grid->SetLocalPosition(3, 0, 0.0);
  grid->SetMaterial(gray);
  root->AddChild(grid);
  //! [create grid visual]

  //! [create camera]
  CameraPtr camera = _scene->CreateCamera("camera");
  camera->SetLocalPosition(-5.0, -5.0, 3);
  camera->SetLocalRotation(0.0, 0.0, 1.0);
  camera->SetImageWidth(800);
  camera->SetImageHeight(600);
  camera->SetAntiAliasing(2);
  camera->SetAspectRatio(1.333);
  camera->SetHFOV(IGN_PI / 2);
  root->AddChild(camera);
  //! [create camera]

  // create particle material
  MaterialPtr particleMaterial = _scene->CreateMaterial();
  particleMaterial->SetDiffuse(0.7, 0.7, 0.7);
  particleMaterial->SetTexture(RESOURCE_PATH + "/smoke.png");
  particleMaterial->SetAlphaFromTexture(true);

  //! [create particle emitter]
  ParticleEmitterPtr emitter = _scene->CreateParticleEmitter();
  emitter->SetType(EM_POINT);
  emitter->SetLocalPose({2, 1.10, 1.25, 1.5708, 0, 2.3});
  emitter->SetRate(10);
  emitter->SetParticleSize({1, 1, 1});
  emitter->SetLifetime(2);
  emitter->SetVelocityRange(10, 20);
  emitter->SetMaterial(particleMaterial);
  emitter->SetColorRangeImage(RESOURCE_PATH + "/smokecolors.png");
  emitter->SetScaleRate(10);
  emitter->SetEmitting(true);
  root->AddChild(emitter);
  //! [create particle emitter]

  // area emitter
  ParticleEmitterPtr areaEmitter = _scene->CreateParticleEmitter();
  areaEmitter->SetType(EM_BOX);
  areaEmitter->SetEmitterSize({3.0, 3.0, 3.0});
  areaEmitter->SetLocalPose({3, 0, 0, 0, -1.5707, 0});
  areaEmitter->SetRate(10);
  areaEmitter->SetParticleSize({0.01, 0.01, 0.01});
  areaEmitter->SetLifetime(1);
  areaEmitter->SetVelocityRange(0.5, 1);
  areaEmitter->SetMaterial(particleMaterial);
  areaEmitter->SetColorRangeImage(RESOURCE_PATH + "/smokecolors.png");
  areaEmitter->SetScaleRate(1);
  areaEmitter->SetEmitting(true);
  root->AddChild(areaEmitter);
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
  std::string ogreEngineName("ogre2");
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
      if (engineName.compare("ogre2") == 0
          && graphicsApi == GraphicsAPI::METAL)
      {
        params["metal"] = "1";
      }

      CameraPtr camera = createCamera(engineName, params);
      if (camera)
      {
        cameras.push_back(camera);
      }
    }
    catch (...)
    {
      // std::cout << ex.what() << std::endl;
      std::cerr << "Error starting up: " << engineName << std::endl;
    }
  }
  run(cameras);
  return 0;
}
