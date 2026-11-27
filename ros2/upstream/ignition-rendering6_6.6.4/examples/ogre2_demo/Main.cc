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
#include <gz/common/MeshManager.hh>
#include <gz/rendering.hh>

#include "example_config.hh"
#include "GlutWindow.hh"

using namespace gz;
using namespace rendering;


const std::string RESOURCE_PATH =
    common::joinPaths(std::string(PROJECT_BINARY_PATH), "media");


//////////////////////////////////////////////////
void buildScene(ScenePtr _scene)
{
  // initialize _scene
  _scene->SetAmbientLight(0.2, 0.2, 0.2);
  _scene->SetBackgroundColor(0.2, 0.2, 0.2);
  VisualPtr root = _scene->RootVisual();

  // enable sky
  _scene->SetSkyEnabled(true);

  // create PBR material
  MaterialPtr matPBR = _scene->CreateMaterial();
  std::string textureMap = common::joinPaths(RESOURCE_PATH, "pump_albedo.png");
  std::string normalMap = common::joinPaths(RESOURCE_PATH, "pump_normal.png");
  std::string roughnessMap =
      common::joinPaths(RESOURCE_PATH, "pump_roughness.png");
  std::string metalnessMap =
      common::joinPaths(RESOURCE_PATH, "pump_metallic.png");
  std::string environmentMap =
      common::joinPaths(RESOURCE_PATH, "fort_point.dds");
  matPBR->SetTexture(textureMap);
  matPBR->SetNormalMap(normalMap);
  matPBR->SetRoughnessMap(roughnessMap);
  matPBR->SetMetalnessMap(metalnessMap);
  matPBR->SetMetalness(0.7);
  matPBR->SetRoughness(0.3);
  matPBR->SetEnvironmentMap(environmentMap);

  // create mesh for PBR
  VisualPtr meshPBR = _scene->CreateVisual("pump");
  meshPBR->SetLocalPosition(2, 0.0, -0.3);
  meshPBR->SetLocalRotation(0, 0, 0);
  MeshDescriptor descriptorPBR;
  descriptorPBR.meshName = common::joinPaths(RESOURCE_PATH, "pump.dae");
  common::MeshManager *meshManager = common::MeshManager::Instance();
  descriptorPBR.mesh = meshManager->Load(descriptorPBR.meshName);
  MeshPtr meshPBRGeom = _scene->CreateMesh(descriptorPBR);
  meshPBRGeom->SetMaterial(matPBR);
  meshPBR->AddGeometry(meshPBRGeom);
  root->AddChild(meshPBR);

  // create green material
  MaterialPtr green = _scene->CreateMaterial();
  green->SetDiffuse(0.0, 0.8, 0.0);
  green->SetSpecular(0.2, 0.4, 0.2);
  green->SetMetalness(0.5);
  green->SetRoughness(0.8);

  // create cylinder visual
  VisualPtr cylinder = _scene->CreateVisual("cylinder");
  cylinder->AddGeometry(_scene->CreateCylinder());
  cylinder->SetLocalPosition(3, -0.5, 0.3);
  cylinder->SetLocalScale(0.7, 0.7, 0.7);
  cylinder->SetMaterial(green);
  root->AddChild(cylinder);

  // create duck material
  MaterialPtr duckMat = _scene->CreateMaterial();
  duckMat->SetTexture(common::joinPaths(RESOURCE_PATH, "duck.png"));
  duckMat->SetRoughness(0.3f);
  duckMat->SetMetalness(0.3f);

  // create a mesh
  VisualPtr mesh = _scene->CreateVisual("duck");
  mesh->SetLocalPosition(4, 0, 0);
  mesh->SetLocalRotation(1.5708, 0, 2.0);
  MeshDescriptor descriptor;
  descriptor.meshName = common::joinPaths(RESOURCE_PATH, "duck.dae");
  descriptor.mesh = meshManager->Load(descriptor.meshName);
  MeshPtr meshGeom = _scene->CreateMesh(descriptor);
  meshGeom->SetMaterial(duckMat);
  mesh->AddGeometry(meshGeom);
  root->AddChild(mesh);

  // create red material
  MaterialPtr red = _scene->CreateMaterial();
  red->SetDiffuse(0.8, 0.0, 0.0);
  red->SetSpecular(0.5, 0.2, 0.2);
  red->SetRoughness(0.2);
  red->SetMetalness(1.0);

  // create sphere visual
  VisualPtr sphere = _scene->CreateVisual("sphere");
  sphere->AddGeometry(_scene->CreateSphere());
  sphere->SetLocalPosition(2.5, 0, 0.3);
  sphere->SetLocalRotation(0, 0, 0);
  sphere->SetLocalScale(0.5, 0.5, 0.5);
  sphere->SetMaterial(red);
  root->AddChild(sphere);

  //! [create envmap]
  // create mirror material
  MaterialPtr mirrorMat = _scene->CreateMaterial();
  mirrorMat->SetDiffuse(1.0, 1.0, 1.0);
  mirrorMat->SetRoughness(0.1);
  mirrorMat->SetMetalness(0.9);
  std::string skyEnvironmentMap =
      common::joinPaths(RESOURCE_PATH, "skybox_lowres.dds");
  mirrorMat->SetEnvironmentMap(skyEnvironmentMap);
  //! [create envmap]

  // create box visual
  VisualPtr box = _scene->CreateVisual("box");
  box->AddGeometry(_scene->CreateBox());
  box->SetLocalPosition(3.0, 0.5, 0.3);
  box->SetLocalRotation(0, 0, 0);
  box->SetLocalScale(0.5, 0.5, 0.5);
  box->SetMaterial(mirrorMat);
  root->AddChild(box);

  // create backpack material
  MaterialPtr backpackMat = _scene->CreateMaterial();
  backpackMat->SetTexture(common::joinPaths(RESOURCE_PATH, "backpack.png"));
  backpackMat->SetRoughness(0.8f);
  backpackMat->SetMetalness(0.0f);

  // create a backpack
  // backpack mesh is mirrored and requires texture addressing mode to be
  // set to 'wrap', which is the default in ign-rendering-ogre2
  VisualPtr backpack = _scene->CreateVisual("backpack");
  backpack->SetLocalPosition(2.5, -1, 0);
  backpack->SetLocalRotation(0, 0, -1.57);
  MeshDescriptor backpackDesc;
  backpackDesc.meshName = common::joinPaths(RESOURCE_PATH, "backpack.dae");
  backpackDesc.mesh = meshManager->Load(backpackDesc.meshName);
  MeshPtr backpackMeshGeom = _scene->CreateMesh(backpackDesc);
  backpackMeshGeom->SetMaterial(backpackMat);
  backpack->AddGeometry(backpackMeshGeom);
  root->AddChild(backpack);

  // create white material
  MaterialPtr white = _scene->CreateMaterial();
  white->SetDiffuse(1.0, 1.0, 1.0);
  white->SetSpecular(1.0, 1.0, 1.0);

  // create plane visual
  VisualPtr plane = _scene->CreateVisual("plane");
  plane->AddGeometry(_scene->CreatePlane());
  plane->SetLocalScale(20, 20, 1);
  plane->SetLocalPosition(0, 0, -0.5);
  plane->SetMaterial(white);
  root->AddChild(plane);

  // create directional light
  DirectionalLightPtr light0 = _scene->CreateDirectionalLight();
  light0->SetDirection(0.5, 0.5, -1);
  light0->SetDiffuseColor(0.8, 0.7, 0.6);
  light0->SetSpecularColor(0.3, 0.3, 0.3);
  light0->SetCastShadows(true);
  root->AddChild(light0);

  // create spot light
  SpotLightPtr light1 = _scene->CreateSpotLight();
  light1->SetDiffuseColor(0.8, 0.8, 0.3);
  light1->SetSpecularColor(0.2, 0.2, 0.2);
  light1->SetLocalPosition(0, 3, 3);
  light1->SetDirection(1, -1, -1);
  light1->SetAttenuationConstant(0.1);
  light1->SetAttenuationLinear(0.001);
  light1->SetAttenuationQuadratic(0.0001);
  light1->SetFalloff(0.8);
  light1->SetCastShadows(true);
  root->AddChild(light1);

  // create point light
  PointLightPtr light2 = _scene->CreatePointLight();
  light2->SetDiffuseColor(0.2, 0.4, 0.8);
  light2->SetSpecularColor(0.2, 0.2, 0.2);
  light2->SetLocalPosition(3, 0, 2);
  light2->SetAttenuationConstant(0.1);
  light2->SetAttenuationLinear(0.001);
  light2->SetAttenuationQuadratic(0.0001);
  light2->SetCastShadows(true);
  root->AddChild(light2);

  // create spot light that does not cast shadows
  SpotLightPtr light3 = _scene->CreateSpotLight();
  light3->SetDiffuseColor(0.3, 0.3, 0.3);
  light3->SetSpecularColor(0.2, 0.2, 0.2);
  light3->SetLocalPosition(0, -3, 3);
  light3->SetDirection(1, 1, -1);
  light3->SetAttenuationConstant(0.1);
  light3->SetAttenuationLinear(0.001);
  light3->SetAttenuationQuadratic(0.0001);
  light3->SetFalloff(0.8);
  light3->SetCastShadows(false);
  root->AddChild(light3);

  // spot light test
  int n = 3;
  for (int i = 0; i < n; ++i)
  {
    for (int j = 0; j < n; ++j)
    {
      // create box visual
      std::stringstream name;
      name << "spotlight_test_" << i << j;
      VisualPtr boxVis = _scene->CreateVisual(name.str());
      boxVis->AddGeometry(_scene->CreateBox());
      double x = -n + i*n -5;
      double y = -n + j*n;
      boxVis->SetLocalPosition(x, y, 0.0);
      boxVis->SetLocalRotation(0, 0, 0);
      boxVis->SetLocalScale(0.5, 0.5, 0.5);
      boxVis->SetMaterial(green);
      root->AddChild(boxVis);

      name << "_light";
      SpotLightPtr spotLight = _scene->CreateSpotLight(name.str());
      spotLight->SetDiffuseColor(1.0, 1.0, 1.0);
      spotLight->SetSpecularColor(0.2, 0.2, 0.2);
      spotLight->SetLocalPosition(x, y, 2.0);
      spotLight->SetDirection(0, 0, -1);
      spotLight->SetCastShadows(true);
      root->AddChild(spotLight);
    }
  }

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
  // get render pass system
  CameraPtr camera = std::dynamic_pointer_cast<Camera>(sensor);
  RenderPassSystemPtr rpSystem = engine->RenderPassSystem();
  if (rpSystem)
  {
    // add gaussian noise pass
    RenderPassPtr pass = rpSystem->Create<GaussianNoisePass>();
    GaussianNoisePassPtr noisePass =
        std::dynamic_pointer_cast<GaussianNoisePass>(pass);
    noisePass->SetMean(0.1);
    noisePass->SetStdDev(0.08);
    noisePass->SetEnabled(false);
    camera->AddRenderPass(noisePass);
  }

  return camera;
}

//////////////////////////////////////////////////
int main(int _argc, char** _argv)
{
  glutInit(&_argc, _argv);

  common::Console::SetVerbosity(4);
  std::vector<std::string> engineNames;
  std::vector<CameraPtr> cameras;

  GraphicsAPI graphicsApi = GraphicsAPI::OPENGL;
  if (_argc > 2)
  {
    graphicsApi = GraphicsAPIUtils::Set(std::string(_argv[2]));
  }

  engineNames.push_back("ogre2");
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
