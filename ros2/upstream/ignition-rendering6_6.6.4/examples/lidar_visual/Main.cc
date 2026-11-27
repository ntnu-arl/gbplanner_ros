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
#include <ignition/rendering.hh>

#include "example_config.hh"
#include "GlutWindow.hh"

using namespace ignition;
using namespace rendering;

const std::string RESOURCE_PATH =
    common::joinPaths(std::string(PROJECT_BINARY_PATH), "media");

// paramters for the LidarVisual and GpuRays API are initialised here
const double hMinAngle = -2.26889;
const double hMaxAngle = 2.26889;
const double vMinAngle = 0;
const double vMaxAngle = 0;
const double minRange = 0.08;
const double maxRange = 10.0;
const int hRayCount = 640;
const int vRayCount = 1;
std::vector<double> pts;

ignition::math::Pose3d testPose(ignition::math::Vector3d(0, 0, 0.5),
    ignition::math::Quaterniond::Identity);

//////////////////////////////////////////////////
void OnNewGpuRaysFrame(float *_scanDest, const float *_scan,
                  unsigned int _width, unsigned int _height,
                  unsigned int _channels,
                  const std::string &/*_format*/)
{
  float f;
  int size =  _width * _height * _channels;
  memcpy(_scanDest, _scan, size * sizeof(f));
}

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

  // create gray material
  MaterialPtr gray = _scene->CreateMaterial();
  gray->SetAmbient(0.7, 0.7, 0.7);
  gray->SetDiffuse(0.7, 0.7, 0.7);
  gray->SetSpecular(0.7, 0.7, 0.7);

  // create red material
  MaterialPtr red = _scene->CreateMaterial();
  red->SetAmbient(1.0, 0.2, 0.1);
  red->SetDiffuse(1.0, 0.2, 0.1);
  red->SetSpecular(1.0, 0.2, 0.1);

  // create green material
  MaterialPtr green = _scene->CreateMaterial();
  green->SetAmbient(0.1, 1, 0.1);
  green->SetDiffuse(0.1, 1, 0.1);
  green->SetSpecular(0.1, 1, 0.1);

  // create yellow material
  MaterialPtr yellow = _scene->CreateMaterial();
  yellow->SetAmbient(1, 1, 0.01);
  yellow->SetDiffuse(1, 1, 0.01);
  yellow->SetSpecular(1, 1, 0.01);

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

  ignition::math::Pose3d box01Pose(ignition::math::Vector3d(6, 0, 0.5),
                                   ignition::math::Quaterniond::Identity);
  VisualPtr visualBox1 = _scene->CreateVisual("UnitBox1");
  visualBox1->AddGeometry(_scene->CreateBox());
  visualBox1->SetWorldPosition(box01Pose.Pos());
  visualBox1->SetWorldRotation(box01Pose.Rot());
  visualBox1->SetMaterial(red);
  root->AddChild(visualBox1);

  ignition::math::Pose3d box02Pose(ignition::math::Vector3d(6, 6, 0.5),
                                   ignition::math::Quaterniond::Identity);
  VisualPtr visualBox2 = _scene->CreateVisual("UnitBox2");
  visualBox2->AddGeometry(_scene->CreateBox());
  visualBox2->SetWorldPosition(box02Pose.Pos());
  visualBox2->SetWorldRotation(box02Pose.Rot());
  visualBox2->SetMaterial(green);
  root->AddChild(visualBox2);

  ignition::math::Pose3d sphere01Pose(ignition::math::Vector3d(1, -3, 0.5),
                                   ignition::math::Quaterniond::Identity);
  VisualPtr visualSphere1 = _scene->CreateVisual("UnitSphere1");
  visualSphere1->AddGeometry(_scene->CreateSphere());
  visualSphere1->SetWorldPosition(sphere01Pose.Pos());
  visualSphere1->SetWorldRotation(sphere01Pose.Rot());
  visualSphere1->SetMaterial(yellow);
  root->AddChild(visualSphere1);

  // create camera
  CameraPtr camera = _scene->CreateCamera("camera");
  camera->SetLocalPosition(0.0, 0.0, 2.0);
  camera->SetLocalRotation(0.0, 0.5, 0.0);
  camera->SetImageWidth(1200);
  camera->SetImageHeight(900);
  camera->SetAntiAliasing(2);
  camera->SetAspectRatio(1.333);
  camera->SetHFOV(IGN_PI / 2);
  root->AddChild(camera);
}

//////////////////////////////////////////////////
GpuRaysPtr createGpuRaySensor(ScenePtr _scene)
{
  // set parameters for GPU lidar sensor and visualisations
  // parameters are based on a sample 2D planar laser sensor
  // add GPU lidar sensor and set parameters
  GpuRaysPtr gpuRays = _scene->CreateGpuRays("gpu_rays");
  gpuRays->SetWorldPosition(testPose.Pos());
  gpuRays->SetWorldRotation(testPose.Rot());
  gpuRays->SetNearClipPlane(minRange);
  gpuRays->SetFarClipPlane(maxRange);
  gpuRays->SetAngleMin(hMinAngle);
  gpuRays->SetAngleMax(hMaxAngle);
  gpuRays->SetRayCount(hRayCount);
  gpuRays->SetVerticalAngleMin(vMinAngle);
  gpuRays->SetVerticalAngleMax(vMaxAngle);
  gpuRays->SetVerticalRayCount(vRayCount);

  VisualPtr root = _scene->RootVisual();
  root->AddChild(gpuRays);

  unsigned int channels = gpuRays->Channels();
  float *scan = new float[hRayCount * vRayCount * channels];

  common::ConnectionPtr c =
    gpuRays->ConnectNewGpuRaysFrame(
        std::bind(&::OnNewGpuRaysFrame, scan,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));

  // update the sensor data
  gpuRays->Update();

  pts.clear();
  for (int j = 0; j < vRayCount; j++)
  {
    for (int i = 0; i < gpuRays->RayCount(); ++i)
    {
      pts.push_back(scan[j*channels*gpuRays->RayCount() + i * channels]);
    }
  }
  return gpuRays;
}

//////////////////////////////////////////////////
LidarVisualPtr createLidar(ScenePtr _scene)
{
  // create lidar visual
  LidarVisualPtr lidar = _scene->CreateLidarVisual();
  lidar->SetMinHorizontalAngle(hMinAngle);
  lidar->SetHorizontalRayCount(hRayCount);
  lidar->SetMaxHorizontalAngle(hMaxAngle);
  lidar->SetVerticalRayCount(vRayCount);
  lidar->SetMinVerticalAngle(vMinAngle);
  lidar->SetMaxVerticalAngle(vMaxAngle);
  lidar->SetMaxRange(maxRange);
  lidar->SetMinRange(minRange);
  lidar->SetSize(5.0);

  // the types can be set as follows:-
  // LVT_POINTS -> Lidar Points at the range value
  // LVT_RAY_LINES -> Lines along the lidar sensor to the obstacle
  // LVT_TRIANGLE_STRIPS -> Coloured triangle strips denoting hitting and
  // non-hitting parts of the scan
  lidar->SetType(LidarVisualType::LVT_TRIANGLE_STRIPS);
  lidar->SetPoints(pts);

  VisualPtr root = _scene->RootVisual();
  root->AddChild(lidar);

  // set this value to false if only the rays that are hitting another obstacle
  // are to be displayed.
  lidar->SetDisplayNonHitting(true);

  lidar->SetWorldPosition(testPose.Pos());
  lidar->SetWorldRotation(testPose.Rot());

  // update lidar visual
  lidar->Update();

  return lidar;
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
  std::vector<LidarVisualPtr> nodes;
  std::vector<GpuRaysPtr> sensors;

  engineNames.push_back(ogreEngineName);

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

      CameraPtr camera = createCamera(engineName, params);
      if (camera)
      {
        cameras.push_back(camera);
        sensors.push_back(createGpuRaySensor(camera->Scene()));
        nodes.push_back(createLidar(camera->Scene()));
      }
    }
    catch (...)
    {
      std::cerr << "Error starting up: " << engineName << std::endl;
    }
  }

  run(cameras, nodes, nodes[0]->Points());
  return 0;
}
