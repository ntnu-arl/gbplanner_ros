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
#include <ignition/common/HeightmapData.hh>
#include <ignition/common/ImageHeightmap.hh>
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

  // create point light
  PointLightPtr light1 = _scene->CreatePointLight();
  light1->SetDiffuseColor(0.8, 0.8, 0.8);
  light1->SetSpecularColor(0.2, 0.2, 0.2);
  light1->SetLocalPosition(30, -3, 6);
  light1->SetAttenuationConstant(0.1);
  light1->SetAttenuationLinear(0.001);
  light1->SetAttenuationQuadratic(0.0001);
  light1->SetAttenuationRange(80);
  light1->SetCastShadows(true);
  root->AddChild(light1);

//! [create a heightmap]
  auto data = std::make_shared<common::ImageHeightmap>();
  data->Load(common::joinPaths(RESOURCE_PATH, "heightmap_bowl.png"));

  HeightmapDescriptor desc;
  desc.SetName("example_bowl");
  desc.SetData(data);
  desc.SetSize({17, 17, 10});
  desc.SetSampling(2u);
  desc.SetUseTerrainPaging(false);

  HeightmapTexture textureA;
  textureA.SetSize(1.0);
  textureA.SetDiffuse("../media/dirt_diffusespecular.png");
  textureA.SetNormal("../media/flat_normal.png");
  desc.AddTexture(textureA);

  HeightmapBlend blendA;
  blendA.SetMinHeight(2.0);
  blendA.SetFadeDistance(5.0);
  desc.AddBlend(blendA);

  HeightmapTexture textureB;
  textureB.SetSize(1.0);
  textureB.SetDiffuse("../media/grass_diffusespecular.png");
  textureB.SetNormal("../media/flat_normal.png");
  desc.AddTexture(textureB);

  HeightmapBlend blendB;
  blendB.SetMinHeight(4.0);
  blendB.SetFadeDistance(5.0);
  desc.AddBlend(blendB);

  HeightmapTexture textureC;
  textureC.SetSize(1.0);
  textureC.SetDiffuse("../media/fungus_diffusespecular.png");
  textureC.SetNormal("../media/flat_normal.png");
  desc.AddTexture(textureC);

  auto heightmapGeom = _scene->CreateHeightmap(desc);

  auto vis = _scene->CreateVisual();
  vis->AddGeometry(heightmapGeom);
  root->AddChild(vis);
//! [create a heightmap]

//! [create another heightmap]
  auto data2 = std::make_shared<common::ImageHeightmap>();
  data2->Load(common::joinPaths(RESOURCE_PATH, "city_terrain.jpg"));

  HeightmapDescriptor desc2;
  desc2.SetName("example_city");
  desc2.SetData(data2);
  desc2.SetSize({26, 26, 20});
  desc2.SetSampling(2u);
  desc2.SetUseTerrainPaging(true);

  HeightmapTexture textureA2;
  textureA2.SetSize(1.0);
  textureA2.SetDiffuse("../media/fungus_diffusespecular.png");
  textureA2.SetNormal("../media/flat_normal.png");
  desc2.AddTexture(textureA2);

  HeightmapBlend blendA2;
  blendA2.SetMinHeight(2.0);
  blendA2.SetFadeDistance(5.0);
  desc2.AddBlend(blendA2);

  HeightmapTexture textureB2;
  textureB2.SetSize(1.0);
  textureB2.SetDiffuse("../media/grass_diffusespecular.png");
  textureB2.SetNormal("../media/flat_normal.png");
  desc2.AddTexture(textureB2);

  HeightmapBlend blendB2;
  blendB2.SetMinHeight(8.0);
  blendB2.SetFadeDistance(5.0);
  desc2.AddBlend(blendB2);

  HeightmapTexture textureC2;
  textureC2.SetSize(1.0);
  textureC2.SetDiffuse("../media/dirt_diffusespecular.png");
  textureC2.SetNormal("../media/flat_normal.png");
  desc2.AddTexture(textureC2);
  desc2.SetPosition({30, 10, 0});
  auto heightmapGeom2 = _scene->CreateHeightmap(desc2);

  auto vis2 = _scene->CreateVisual();
  vis2->AddGeometry(heightmapGeom2);
  root->AddChild(vis2);
//! [create another heightmap]

  // create gray material
  MaterialPtr gray = _scene->CreateMaterial();
  gray->SetAmbient(0.7, 0.7, 0.7);
  gray->SetDiffuse(0.7, 0.7, 0.7);
  gray->SetSpecular(0.7, 0.7, 0.7);

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
  camera->SetLocalPosition(1.441, 25.787, 17.801);
  camera->SetLocalRotation(0.0, 0.588, -1.125);
  camera->SetImageWidth(1600);
  camera->SetImageHeight(900);
  camera->SetAntiAliasing(2);
  camera->SetAspectRatio(1.333);
  camera->SetHFOV(IGN_PI / 2);
  root->AddChild(camera);
//! [create camera]
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

  common::Console::SetVerbosity(4);
  std::vector<std::string> engineNames;
  std::vector<CameraPtr> cameras;

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
