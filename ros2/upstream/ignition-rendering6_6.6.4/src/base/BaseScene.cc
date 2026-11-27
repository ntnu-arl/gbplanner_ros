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

#include <sstream>

#include <gz/math/Helpers.hh>

#include <gz/common/Console.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>

#include "gz/common/Time.hh"

#include "gz/rendering/ArrowVisual.hh"
#include "gz/rendering/AxisVisual.hh"
#include "gz/rendering/BoundingBoxCamera.hh"
#include "gz/rendering/COMVisual.hh"
#include "gz/rendering/InertiaVisual.hh"
#include "gz/rendering/JointVisual.hh"
#include "gz/rendering/LidarVisual.hh"
#include "gz/rendering/LightVisual.hh"
#include "gz/rendering/Camera.hh"
#include "gz/rendering/Capsule.hh"
#include "gz/rendering/DepthCamera.hh"
#include "gz/rendering/GizmoVisual.hh"
#include "gz/rendering/GpuRays.hh"
#include "gz/rendering/Grid.hh"
#include "gz/rendering/ParticleEmitter.hh"
#include "gz/rendering/RayQuery.hh"
#include "gz/rendering/RenderTarget.hh"
#include "gz/rendering/Text.hh"
#include "gz/rendering/ThermalCamera.hh"
#include "gz/rendering/SegmentationCamera.hh"
#include "gz/rendering/Visual.hh"
#include "gz/rendering/base/BaseStorage.hh"
#include "gz/rendering/base/BaseScene.hh"

using namespace gz;
using namespace rendering;

// Prevent deprecation warnings for simTime
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
//////////////////////////////////////////////////
BaseScene::BaseScene(unsigned int _id, const std::string &_name) :
  id(_id),
  name(_name),
  loaded(false),
  initialized(false),
  nextObjectId(math::MAX_UI16),
  nodes(nullptr)
{
}

//////////////////////////////////////////////////
BaseScene::~BaseScene()
{
}
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

//////////////////////////////////////////////////
void BaseScene::Load()
{
  if (!this->loaded)
  {
    this->loaded = this->LoadImpl();
  }
}

//////////////////////////////////////////////////
void BaseScene::Init()
{
  if (!this->loaded)
  {
    ignerr << "Scene must be loaded first" << std::endl;
    return;
  }

  if (!this->initialized)
  {
    this->initialized = this->InitImpl();
    this->CreateNodeStore();
    this->CreateMaterials();
  }
}

//////////////////////////////////////////////////
void BaseScene::Fini()
{
  this->Destroy();
}

//////////////////////////////////////////////////
bool BaseScene::IsLoaded() const
{
  return this->loaded;
}

//////////////////////////////////////////////////
bool BaseScene::IsInitialized() const
{
  return this->initialized;
}

//////////////////////////////////////////////////
unsigned int BaseScene::Id() const
{
  return this->id;
}

//////////////////////////////////////////////////
std::string BaseScene::Name() const
{
  return this->name;
}

//////////////////////////////////////////////////
std::chrono::steady_clock::duration BaseScene::Time() const
{
  return this->time;
}

//////////////////////////////////////////////////
void BaseScene::SetTime(const std::chrono::steady_clock::duration &_time)
{
#if IGNITION_RENDERING_MAJOR_VERSION <= 6
  // TODO(anyone): Remove me in ign-rendering7
  if (std::chrono::duration_cast<std::chrono::nanoseconds>(_time).count() != -1)
#endif
  {
    this->time = _time;
  }
}

//////////////////////////////////////////////////
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
common::Time BaseScene::SimTime() const
{
  return this->simTime;
}

////////////////////////////////////////////////////
void BaseScene::SetSimTime(const common::Time &_time)
{
  this->simTime = _time;
}
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

//////////////////////////////////////////////////
VisualPtr BaseScene::VisualAt(const CameraPtr &_camera,
                              const math::Vector2i &_mousePos)
{
  VisualPtr visual;
  RayQueryPtr rayQuery = this->CreateRayQuery();
  if (!rayQuery)
    return visual;

  double nx =
      2.0 * _mousePos.X() / static_cast<double>(_camera->ImageWidth()) - 1.0;
  double ny =
      1.0 - 2.0 * _mousePos.Y() / static_cast<double>(_camera->ImageHeight());

  math::Vector2d mousePos(nx, ny);

  if (rayQuery)
  {
    rayQuery->SetFromCamera(_camera, mousePos);
    RayQueryResult result = rayQuery->ClosestPoint();

    if (result)
    {
      visual = this->Visuals()->GetById(result.objectId);
    }
  }
  return visual;
}

//////////////////////////////////////////////////
void BaseScene::SetAmbientLight(double _r, double _g, double _b, double _a)
{
  this->SetAmbientLight(math::Color(_r, _g, _b, _a));
}

//////////////////////////////////////////////////
math::Color BaseScene::BackgroundColor() const
{
  return this->backgroundColor;
}

//////////////////////////////////////////////////
void BaseScene::SetBackgroundColor(double _r, double _g, double _b, double _a)
{
  this->SetBackgroundColor(math::Color(_r, _g, _b, _a));
}

//////////////////////////////////////////////////
void BaseScene::SetBackgroundColor(const math::Color &_color)
{
  this->backgroundColor = _color;
}

//////////////////////////////////////////////////
bool BaseScene::IsGradientBackgroundColor() const
{
  return this->isGradientBackgroundColor;
}

//////////////////////////////////////////////////
std::array<math::Color, 4> BaseScene::GradientBackgroundColor() const
{
  return this->gradientBackgroundColor;
}

//////////////////////////////////////////////////
void BaseScene::SetGradientBackgroundColor(
  const std::array<math::Color, 4> &_colors)
{
  this->gradientBackgroundColor = _colors;
  this->isGradientBackgroundColor = true;
}

//////////////////////////////////////////////////
void BaseScene::RemoveGradientBackgroundColor()
{
  this->gradientBackgroundColor = {math::Color::Black, math::Color::Black,
      math::Color::Black, math::Color::Black};
  this->isGradientBackgroundColor = false;
}

//////////////////////////////////////////////////
MaterialPtr BaseScene::BackgroundMaterial() const
{
  return this->backgroundMaterial;
}

//////////////////////////////////////////////////
void BaseScene::SetBackgroundMaterial(MaterialPtr _material)
{
  this->backgroundMaterial = _material;
}

//////////////////////////////////////////////////
unsigned int BaseScene::NodeCount() const
{
  return this->nodes->Size();
}

//////////////////////////////////////////////////
bool BaseScene::HasNode(ConstNodePtr _node) const
{
  return this->nodes->Contains(_node);
}

//////////////////////////////////////////////////
bool BaseScene::HasNodeId(unsigned int _id) const
{
  return this->nodes->ContainsId(_id);
}

//////////////////////////////////////////////////
bool BaseScene::HasNodeName(const std::string &_name) const
{
  return this->nodes->ContainsName(_name);
}

//////////////////////////////////////////////////
NodePtr BaseScene::NodeById(unsigned int _id) const
{
  return this->nodes->GetById(_id);
}

//////////////////////////////////////////////////
NodePtr BaseScene::NodeByName(const std::string &_name) const
{
  return this->nodes->GetByName(_name);
}

//////////////////////////////////////////////////
NodePtr BaseScene::NodeByIndex(unsigned int _index) const
{
  return this->nodes->GetByIndex(_index);
}

//////////////////////////////////////////////////
void BaseScene::DestroyNode(NodePtr _node, bool _recursive)
{
  if (!_node)
    return;

  if (_recursive)
  {
    std::set<unsigned int> nodeIds;
    this->DestroyNodeRecursive(_node, nodeIds);
  }
  else
    this->nodes->Destroy(_node);
}

//////////////////////////////////////////////////
void BaseScene::DestroyNodeRecursive(NodePtr _node,
    std::set<unsigned int> &_nodeIds)
{
  // check if we have visited this node before
  if (_nodeIds.find(_node->Id()) != _nodeIds.end())
  {
    ignwarn << "Detected loop in scene tree while recursively destroying nodes."
            << " Breaking loop." << std::endl;
    _node->RemoveParent();
    return;
  }
  _nodeIds.insert(_node->Id());

  // destroy child nodes first
  while (_node->ChildCount() > 0u)
  {
    this->DestroyNodeRecursive(_node->ChildByIndex(0u), _nodeIds);
  }

  // destroy node
  this->nodes->Destroy(_node);
}

//////////////////////////////////////////////////
void BaseScene::DestroyNodeById(unsigned int _id)
{
  this->nodes->DestroyById(_id);
}

//////////////////////////////////////////////////
void BaseScene::DestroyNodeByName(const std::string &_name)
{
  this->nodes->DestroyByName(_name);
}

//////////////////////////////////////////////////
void BaseScene::DestroyNodeByIndex(unsigned int _index)
{
  this->nodes->DestroyByIndex(_index);
}

//////////////////////////////////////////////////
void BaseScene::DestroyNodes()
{
  this->nodes->DestroyAll();
}

//////////////////////////////////////////////////
unsigned int BaseScene::LightCount() const
{
  return this->Lights()->Size();
}

//////////////////////////////////////////////////
bool BaseScene::HasLight(ConstLightPtr _light) const
{
  return this->Lights()->Contains(_light);
}

//////////////////////////////////////////////////
bool BaseScene::HasLightId(unsigned int _id) const
{
  return this->Lights()->ContainsId(_id);
}

//////////////////////////////////////////////////
bool BaseScene::HasLightName(const std::string &_name) const
{
  return this->Lights()->ContainsName(_name);
}

//////////////////////////////////////////////////
LightPtr BaseScene::LightById(unsigned int _id) const
{
  return this->Lights()->GetById(_id);
}

//////////////////////////////////////////////////
LightPtr BaseScene::LightByName(const std::string &_name) const
{
  return this->Lights()->GetByName(_name);
}

//////////////////////////////////////////////////
LightPtr BaseScene::LightByIndex(unsigned int _index) const
{
  return this->Lights()->GetByIndex(_index);
}

//////////////////////////////////////////////////
void BaseScene::DestroyLight(LightPtr _light, bool _recursive)
{
  if (_recursive)
  {
    this->DestroyNode(_light, _recursive);
  }
  else
  {
    this->Lights()->Destroy(_light);
  }
}

//////////////////////////////////////////////////
void BaseScene::DestroyLightById(unsigned int _id)
{
  this->Lights()->DestroyById(_id);
}

//////////////////////////////////////////////////
void BaseScene::DestroyLightByName(const std::string &_name)
{
  this->Lights()->DestroyByName(_name);
}

//////////////////////////////////////////////////
void BaseScene::DestroyLightByIndex(unsigned int _index)
{
  this->Lights()->DestroyByIndex(_index);
}

//////////////////////////////////////////////////
void BaseScene::DestroyLights()
{
  this->Lights()->DestroyAll();
}

//////////////////////////////////////////////////
unsigned int BaseScene::SensorCount() const
{
  return this->Sensors()->Size();
}

//////////////////////////////////////////////////
bool BaseScene::HasSensor(ConstSensorPtr _sensor) const
{
  return this->Sensors()->Contains(_sensor);
}

//////////////////////////////////////////////////
bool BaseScene::HasSensorId(unsigned int _id) const
{
  return this->Sensors()->ContainsId(_id);
}

//////////////////////////////////////////////////
bool BaseScene::HasSensorName(const std::string &_name) const
{
  return this->Sensors()->ContainsName(_name);
}

//////////////////////////////////////////////////
SensorPtr BaseScene::SensorById(unsigned int _id) const
{
  return this->Sensors()->GetById(_id);
}

//////////////////////////////////////////////////
SensorPtr BaseScene::SensorByName(const std::string &_name) const
{
  return this->Sensors()->GetByName(_name);
}

//////////////////////////////////////////////////
SensorPtr BaseScene::SensorByIndex(unsigned int _index) const
{
  return this->Sensors()->GetByIndex(_index);
}

//////////////////////////////////////////////////
void BaseScene::DestroySensor(SensorPtr _sensor, bool _recursive)
{
  if (_recursive)
  {
    this->DestroyNode(_sensor, _recursive);
  }
  else
  {
    this->Sensors()->Destroy(_sensor);
  }
}

//////////////////////////////////////////////////
void BaseScene::DestroySensorById(unsigned int _id)
{
  this->Sensors()->DestroyById(_id);
}

//////////////////////////////////////////////////
void BaseScene::DestroySensorByName(const std::string &_name)
{
  this->Sensors()->DestroyByName(_name);
}

//////////////////////////////////////////////////
void BaseScene::DestroySensorByIndex(unsigned int _index)
{
  this->Sensors()->DestroyByIndex(_index);
}

//////////////////////////////////////////////////
void BaseScene::DestroySensors()
{
  this->Sensors()->DestroyAll();
}

//////////////////////////////////////////////////
unsigned int BaseScene::VisualCount() const
{
  return this->Visuals()->Size();
}

//////////////////////////////////////////////////
bool BaseScene::HasVisual(ConstVisualPtr _visual) const
{
  return this->Visuals()->Contains(_visual);
}

//////////////////////////////////////////////////
bool BaseScene::HasVisualId(unsigned int _id) const
{
  return this->Visuals()->ContainsId(_id);
}

//////////////////////////////////////////////////
bool BaseScene::HasVisualName(const std::string &_name) const
{
  return this->Visuals()->ContainsName(_name);
}

//////////////////////////////////////////////////
VisualPtr BaseScene::VisualById(unsigned int _id) const
{
  return this->Visuals()->GetById(_id);
}

//////////////////////////////////////////////////
VisualPtr BaseScene::VisualByName(const std::string &_name) const
{
  return this->Visuals()->GetByName(_name);
}

//////////////////////////////////////////////////
VisualPtr BaseScene::VisualByIndex(unsigned int _index) const
{
  return this->Visuals()->GetByIndex(_index);
}

//////////////////////////////////////////////////
void BaseScene::DestroyVisual(VisualPtr _visual, bool _recursive)
{
  if (_recursive)
  {
    this->DestroyNode(_visual, _recursive);
  }
  else
  {
    this->Visuals()->Destroy(_visual);
  }
}

//////////////////////////////////////////////////
void BaseScene::DestroyVisualById(unsigned int _id)
{
  this->Visuals()->DestroyById(_id);
}

//////////////////////////////////////////////////
void BaseScene::DestroyVisualByName(const std::string &_name)
{
  this->Visuals()->DestroyByName(_name);
}

//////////////////////////////////////////////////
void BaseScene::DestroyVisualByIndex(unsigned int _index)
{
  this->Visuals()->DestroyByIndex(_index);
}

//////////////////////////////////////////////////
void BaseScene::DestroyVisuals()
{
  this->Visuals()->DestroyAll();
}

//////////////////////////////////////////////////
bool BaseScene::MaterialRegistered(const std::string &_name) const
{
  return this->Materials()->ContainsKey(_name);
}

//////////////////////////////////////////////////
MaterialPtr BaseScene::Material(const std::string &_name) const
{
  return this->Materials()->Get(_name);
}

//////////////////////////////////////////////////
void BaseScene::RegisterMaterial(const std::string &_name,
    MaterialPtr _material)
{
  if (_material)
    this->Materials()->Put(_name, _material);
}

//////////////////////////////////////////////////
void BaseScene::UnregisterMaterial(const std::string &_name)
{
  this->Materials()->Remove(_name);
}

//////////////////////////////////////////////////
void BaseScene::UnregisterMaterials()
{
  this->Materials()->RemoveAll();
}

//////////////////////////////////////////////////
void BaseScene::DestroyMaterial(MaterialPtr _material)
{
  if (!_material)
    return;

  std::string matName = _material->Name();
  _material->Destroy();
  this->UnregisterMaterial(matName);
}

//////////////////////////////////////////////////
void BaseScene::DestroyMaterials()
{
  for (unsigned int i = 0; i < this->Materials()->Size(); ++i)
  {
    auto m = this->Materials()->GetByIndex(i);
    m->Destroy();
  }
  this->UnregisterMaterials();
}

//////////////////////////////////////////////////
DirectionalLightPtr BaseScene::CreateDirectionalLight()
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateDirectionalLight(objId);
}

//////////////////////////////////////////////////
DirectionalLightPtr BaseScene::CreateDirectionalLight(unsigned int _id)
{
  std::string objName = this->CreateObjectName(_id, "DirectionalLight");
  return this->CreateDirectionalLight(_id, objName);
}

//////////////////////////////////////////////////
DirectionalLightPtr BaseScene::CreateDirectionalLight(const std::string &_name)
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateDirectionalLight(objId, _name);
}

//////////////////////////////////////////////////
DirectionalLightPtr BaseScene::CreateDirectionalLight(unsigned int _id,
    const std::string &_name)
{
  DirectionalLightPtr light = this->CreateDirectionalLightImpl(_id, _name);
  bool result = this->RegisterLight(light);
  return (result) ? light : nullptr;
}

//////////////////////////////////////////////////
PointLightPtr BaseScene::CreatePointLight()
{
  unsigned int objId = this->CreateObjectId();
  return this->CreatePointLight(objId);
}
//////////////////////////////////////////////////
PointLightPtr BaseScene::CreatePointLight(unsigned int _id)
{
  std::string objName = this->CreateObjectName(_id, "PointLight");
  return this->CreatePointLight(_id, objName);
}
//////////////////////////////////////////////////
PointLightPtr BaseScene::CreatePointLight(const std::string &_name)
{
  unsigned int objId = this->CreateObjectId();
  return this->CreatePointLight(objId, _name);
}
//////////////////////////////////////////////////
PointLightPtr BaseScene::CreatePointLight(unsigned int _id,
    const std::string &_name)
{
  PointLightPtr light = this->CreatePointLightImpl(_id, _name);
  bool result = this->RegisterLight(light);
  return (result) ? light : nullptr;
}

//////////////////////////////////////////////////
SpotLightPtr BaseScene::CreateSpotLight()
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateSpotLight(objId);
}
//////////////////////////////////////////////////
SpotLightPtr BaseScene::CreateSpotLight(unsigned int _id)
{
  std::string objName = this->CreateObjectName(_id, "SpotLight");
  return this->CreateSpotLight(_id, objName);
}
//////////////////////////////////////////////////
SpotLightPtr BaseScene::CreateSpotLight(const std::string &_name)
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateSpotLight(objId, _name);
}
//////////////////////////////////////////////////
SpotLightPtr BaseScene::CreateSpotLight(unsigned int _id,
    const std::string &_name)
{
  SpotLightPtr light = this->CreateSpotLightImpl(_id, _name);
  bool result = this->RegisterLight(light);
  return (result) ? light : nullptr;
}

//////////////////////////////////////////////////
CameraPtr BaseScene::CreateCamera()
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateCamera(objId);
}
//////////////////////////////////////////////////
CameraPtr BaseScene::CreateCamera(unsigned int _id)
{
  std::string objName = this->CreateObjectName(_id, "Camera");
  return this->CreateCamera(_id, objName);
}
//////////////////////////////////////////////////
CameraPtr BaseScene::CreateCamera(const std::string &_name)
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateCamera(objId, _name);
}
//////////////////////////////////////////////////
CameraPtr BaseScene::CreateCamera(unsigned int _id, const std::string &_name)
{
  CameraPtr camera = this->CreateCameraImpl(_id, _name);
  bool result = this->RegisterSensor(camera);
  return (result) ? camera : nullptr;
}

//////////////////////////////////////////////////
DepthCameraPtr BaseScene::CreateDepthCamera()
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateDepthCamera(objId);
}
//////////////////////////////////////////////////
DepthCameraPtr BaseScene::CreateDepthCamera(const unsigned int _id)
{
  std::string objName = this->CreateObjectName(_id, "DepthCamera");
  return this->CreateDepthCamera(_id, objName);
}
//////////////////////////////////////////////////
DepthCameraPtr BaseScene::CreateDepthCamera(const std::string &_name)
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateDepthCamera(objId, _name);
}
//////////////////////////////////////////////////
DepthCameraPtr BaseScene::CreateDepthCamera(const unsigned int _id,
    const std::string &_name)
{
  DepthCameraPtr camera = this->CreateDepthCameraImpl(_id, _name);
  bool result = this->RegisterSensor(camera);
  return (result) ? camera : nullptr;
}

//////////////////////////////////////////////////
ThermalCameraPtr BaseScene::CreateThermalCamera()
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateThermalCamera(objId);
}
//////////////////////////////////////////////////
ThermalCameraPtr BaseScene::CreateThermalCamera(const unsigned int _id)
{
  std::string objName = this->CreateObjectName(_id, "ThermalCamera");
  return this->CreateThermalCamera(_id, objName);
}
//////////////////////////////////////////////////
ThermalCameraPtr BaseScene::CreateThermalCamera(const std::string &_name)
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateThermalCamera(objId, _name);
}
//////////////////////////////////////////////////
ThermalCameraPtr BaseScene::CreateThermalCamera(const unsigned int _id,
    const std::string &_name)
{
  ThermalCameraPtr camera = this->CreateThermalCameraImpl(_id, _name);
  bool result = this->RegisterSensor(camera);
  return (result) ? camera : nullptr;
}

//////////////////////////////////////////////////
BoundingBoxCameraPtr BaseScene::CreateBoundingBoxCamera()
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateBoundingBoxCamera(objId);
}
//////////////////////////////////////////////////
BoundingBoxCameraPtr BaseScene::CreateBoundingBoxCamera(
  const unsigned int _id)
{
  std::string objName = this->CreateObjectName(_id, "BoundingBoxCamera");
  return this->CreateBoundingBoxCamera(_id, objName);
}
//////////////////////////////////////////////////
BoundingBoxCameraPtr BaseScene::CreateBoundingBoxCamera(
  const std::string &_name)
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateBoundingBoxCamera(objId, _name);
}
//////////////////////////////////////////////////
BoundingBoxCameraPtr BaseScene::CreateBoundingBoxCamera(
  const unsigned int _id,
    const std::string &_name)
{
  BoundingBoxCameraPtr camera = this->CreateBoundingBoxCameraImpl(_id, _name);
  bool result = this->RegisterSensor(camera);
  return (result) ? camera : nullptr;
}

//////////////////////////////////////////////////
SegmentationCameraPtr BaseScene::CreateSegmentationCamera()
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateSegmentationCamera(objId);
}
//////////////////////////////////////////////////
SegmentationCameraPtr BaseScene::CreateSegmentationCamera(
  const unsigned int _id)
{
  std::string objName = this->CreateObjectName(_id, "SegmentationCamera");
  return this->CreateSegmentationCamera(_id, objName);
}
//////////////////////////////////////////////////
SegmentationCameraPtr BaseScene::CreateSegmentationCamera(
  const std::string &_name)
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateSegmentationCamera(objId, _name);
}
//////////////////////////////////////////////////
SegmentationCameraPtr BaseScene::CreateSegmentationCamera(
  const unsigned int _id,
    const std::string &_name)
{
  SegmentationCameraPtr camera = this->CreateSegmentationCameraImpl(_id, _name);
  bool result = this->RegisterSensor(camera);
  return (result) ? camera : nullptr;
}

//////////////////////////////////////////////////
GpuRaysPtr BaseScene::CreateGpuRays()
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateGpuRays(objId);
}
//////////////////////////////////////////////////
GpuRaysPtr BaseScene::CreateGpuRays(const unsigned int _id)
{
  std::string objName = this->CreateObjectName(_id, "GpuRays");
  return this->CreateGpuRays(_id, objName);
}
//////////////////////////////////////////////////
GpuRaysPtr BaseScene::CreateGpuRays(const std::string &_name)
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateGpuRays(objId, _name);
}
//////////////////////////////////////////////////
GpuRaysPtr BaseScene::CreateGpuRays(const unsigned int _id,
    const std::string &_name)
{
  GpuRaysPtr gpuRays = this->CreateGpuRaysImpl(_id, _name);
  bool result = this->RegisterSensor(gpuRays);
  return (result) ? gpuRays : nullptr;
}

//////////////////////////////////////////////////
VisualPtr BaseScene::CreateVisual()
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateVisual(objId);
}
//////////////////////////////////////////////////
VisualPtr BaseScene::CreateVisual(unsigned int _id)
{
  std::string objName = this->CreateObjectName(_id, "Visual");
  return this->CreateVisual(_id, objName);
}
//////////////////////////////////////////////////
VisualPtr BaseScene::CreateVisual(const std::string &_name)
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateVisual(objId, _name);
}
//////////////////////////////////////////////////
VisualPtr BaseScene::CreateVisual(unsigned int _id, const std::string &_name)
{
  VisualPtr visual = this->CreateVisualImpl(_id, _name);
  bool result = this->RegisterVisual(visual);
  return (result) ? visual : nullptr;
}

//////////////////////////////////////////////////
ArrowVisualPtr BaseScene::CreateArrowVisual()
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateArrowVisual(objId);
}
//////////////////////////////////////////////////
ArrowVisualPtr BaseScene::CreateArrowVisual(unsigned int _id)
{
  std::string objName = this->CreateObjectName(_id, "ArrowVisual");
  return this->CreateArrowVisual(_id, objName);
}
//////////////////////////////////////////////////
ArrowVisualPtr BaseScene::CreateArrowVisual(const std::string &_name)
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateArrowVisual(objId, _name);
}
//////////////////////////////////////////////////
ArrowVisualPtr BaseScene::CreateArrowVisual(unsigned int _id,
    const std::string &_name)
{
  ArrowVisualPtr visual = this->CreateArrowVisualImpl(_id, _name);
  bool result = this->RegisterVisual(visual);
  return (result) ? visual : nullptr;
}

//////////////////////////////////////////////////
AxisVisualPtr BaseScene::CreateAxisVisual()
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateAxisVisual(objId);
}
//////////////////////////////////////////////////
AxisVisualPtr BaseScene::CreateAxisVisual(unsigned int _id)
{
  std::string objName = this->CreateObjectName(_id, "AxisVisual");
  return this->CreateAxisVisual(_id, objName);
}
//////////////////////////////////////////////////
AxisVisualPtr BaseScene::CreateAxisVisual(const std::string &_name)
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateAxisVisual(objId, _name);
}
//////////////////////////////////////////////////
AxisVisualPtr BaseScene::CreateAxisVisual(unsigned int _id,
    const std::string &_name)
{
  AxisVisualPtr visual = this->CreateAxisVisualImpl(_id, _name);
  bool result = this->RegisterVisual(visual);
  return (result) ? visual : nullptr;
}

//////////////////////////////////////////////////
COMVisualPtr BaseScene::CreateCOMVisual()
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateCOMVisual(objId);
}

//////////////////////////////////////////////////
COMVisualPtr BaseScene::CreateCOMVisual(unsigned int _id)
{
  std::string objName = this->CreateObjectName(_id, "COMVisual");
  return this->CreateCOMVisual(_id, objName);
}

//////////////////////////////////////////////////
COMVisualPtr BaseScene::CreateCOMVisual(const std::string &_name)
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateCOMVisual(objId, _name);
}

//////////////////////////////////////////////////
COMVisualPtr BaseScene::CreateCOMVisual(unsigned int _id,
    const std::string &_name)
{
  COMVisualPtr visual = this->CreateCOMVisualImpl(_id, _name);
  bool result = this->RegisterVisual(visual);
  return (result) ? visual : nullptr;
}

InertiaVisualPtr BaseScene::CreateInertiaVisual()
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateInertiaVisual(objId);
}

//////////////////////////////////////////////////
InertiaVisualPtr BaseScene::CreateInertiaVisual(unsigned int _id)
{
  std::string objName = this->CreateObjectName(_id, "InertiaVisual");
  return this->CreateInertiaVisual(_id, objName);
}

//////////////////////////////////////////////////
InertiaVisualPtr BaseScene::CreateInertiaVisual(const std::string &_name)
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateInertiaVisual(objId, _name);
}

//////////////////////////////////////////////////
InertiaVisualPtr BaseScene::CreateInertiaVisual(unsigned int _id,
    const std::string &_name)
{
  InertiaVisualPtr visual = this->CreateInertiaVisualImpl(_id, _name);
  bool result = this->RegisterVisual(visual);
  return (result) ? visual : nullptr;
}

//////////////////////////////////////////////////
JointVisualPtr BaseScene::CreateJointVisual()
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateJointVisual(objId);
}

//////////////////////////////////////////////////
JointVisualPtr BaseScene::CreateJointVisual(unsigned int _id)
{
  std::string objName = this->CreateObjectName(_id, "JointVisual");
  return this->CreateJointVisual(_id, objName);
}

//////////////////////////////////////////////////
JointVisualPtr BaseScene::CreateJointVisual(const std::string &_name)
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateJointVisual(objId, _name);
}

//////////////////////////////////////////////////
JointVisualPtr BaseScene::CreateJointVisual(unsigned int _id,
    const std::string &_name)
{
  JointVisualPtr visual = this->CreateJointVisualImpl(_id, _name);
  bool result = this->RegisterVisual(visual);
  return (result) ? visual : nullptr;
}

//////////////////////////////////////////////////
LightVisualPtr BaseScene::CreateLightVisual()
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateLightVisual(objId);
}

//////////////////////////////////////////////////
LightVisualPtr BaseScene::CreateLightVisual(unsigned int _id)
{
  std::string objName = this->CreateObjectName(_id, "LightVisual");
  return this->CreateLightVisual(_id, objName);
}

//////////////////////////////////////////////////
LightVisualPtr BaseScene::CreateLightVisual(const std::string &_name)
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateLightVisual(objId, _name);
}

//////////////////////////////////////////////////
LightVisualPtr BaseScene::CreateLightVisual(unsigned int _id,
    const std::string &_name)
{
  LightVisualPtr visual = this->CreateLightVisualImpl(_id, _name);
  bool result = this->RegisterVisual(visual);
  return (result) ? visual : nullptr;
}

//////////////////////////////////////////////////
GizmoVisualPtr BaseScene::CreateGizmoVisual()
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateGizmoVisual(objId);
}

//////////////////////////////////////////////////
GizmoVisualPtr BaseScene::CreateGizmoVisual(unsigned int _id)
{
  std::string objName = this->CreateObjectName(_id, "GizmoVisual");
  return this->CreateGizmoVisual(_id, objName);
}

//////////////////////////////////////////////////
GizmoVisualPtr BaseScene::CreateGizmoVisual(const std::string &_name)
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateGizmoVisual(objId, _name);
}

//////////////////////////////////////////////////
GizmoVisualPtr BaseScene::CreateGizmoVisual(unsigned int _id,
    const std::string &_name)
{
  GizmoVisualPtr visual = this->CreateGizmoVisualImpl(_id, _name);
  bool result = this->RegisterVisual(visual);
  return (result) ? visual : nullptr;
}

//////////////////////////////////////////////////
GeometryPtr BaseScene::CreateBox()
{
  unsigned int objId = this->CreateObjectId();
  std::string objName = this->CreateObjectName(objId, "Box");
  return this->CreateBoxImpl(objId, objName);
}

//////////////////////////////////////////////////
GeometryPtr BaseScene::CreateCone()
{
  unsigned int objId = this->CreateObjectId();
  std::string objName = this->CreateObjectName(objId, "Cone");
  return this->CreateConeImpl(objId, objName);
}

//////////////////////////////////////////////////
GeometryPtr BaseScene::CreateCylinder()
{
  unsigned int objId = this->CreateObjectId();
  std::string objName = this->CreateObjectName(objId, "Cylinder");
  return this->CreateCylinderImpl(objId, objName);
}

//////////////////////////////////////////////////
GeometryPtr BaseScene::CreatePlane()
{
  unsigned int objId = this->CreateObjectId();
  std::string objName = this->CreateObjectName(objId, "Plane");
  return this->CreatePlaneImpl(objId, objName);
}

//////////////////////////////////////////////////
CapsulePtr BaseScene::CreateCapsule()
{
  unsigned int objId = this->CreateObjectId();
  std::string objName = this->CreateObjectName(objId, "Capsule");
  return this->CreateCapsuleImpl(objId, objName);
}

//////////////////////////////////////////////////
GeometryPtr BaseScene::CreateSphere()
{
  unsigned int objId = this->CreateObjectId();
  std::string objName = this->CreateObjectName(objId, "Sphere");
  return this->CreateSphereImpl(objId, objName);
}

//////////////////////////////////////////////////
MeshPtr BaseScene::CreateMesh(const std::string &_meshName)
{
  MeshDescriptor descriptor(_meshName);
  return this->CreateMesh(descriptor);
}

//////////////////////////////////////////////////
MeshPtr BaseScene::CreateMesh(const common::Mesh *_mesh)
{
  MeshDescriptor descriptor(_mesh);
  return this->CreateMesh(descriptor);
}

//////////////////////////////////////////////////
MeshPtr BaseScene::CreateMesh(const MeshDescriptor &_desc)
{
  std::string meshName = (_desc.mesh) ?
      _desc.mesh->Name() : _desc.meshName;

  unsigned int objId = this->CreateObjectId();
  std::string objName = this->CreateObjectName(objId, "Mesh-" + meshName);
  return this->CreateMeshImpl(objId, objName, _desc);
}

//////////////////////////////////////////////////
HeightmapPtr BaseScene::CreateHeightmap(const HeightmapDescriptor &_desc)
{
  unsigned int objId = this->CreateObjectId();
  std::string objName = this->CreateObjectName(objId, "Heightmap");
  return this->CreateHeightmapImpl(objId, objName, _desc);
}

//////////////////////////////////////////////////
GridPtr BaseScene::CreateGrid()
{
  unsigned int objId = this->CreateObjectId();
  std::string objName = this->CreateObjectName(objId, "Grid");
  return this->CreateGridImpl(objId, objName);
}

//////////////////////////////////////////////////
MarkerPtr BaseScene::CreateMarker()
{
  unsigned int objId = this->CreateObjectId();
  std::string objName = this->CreateObjectName(objId, "Marker");
  return this->CreateMarkerImpl(objId, objName);
}

//////////////////////////////////////////////////
LidarVisualPtr BaseScene::CreateLidarVisual()
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateLidarVisual(objId);
}

//////////////////////////////////////////////////
LidarVisualPtr BaseScene::CreateLidarVisual(unsigned int _id)
{
  const std::string objName = this->CreateObjectName(_id, "LidarVisual");
  return this->CreateLidarVisual(_id, objName);
}

//////////////////////////////////////////////////
LidarVisualPtr BaseScene::CreateLidarVisual(const std::string &_name)
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateLidarVisual(objId, _name);
}

//////////////////////////////////////////////////
LidarVisualPtr BaseScene::CreateLidarVisual(unsigned int _id,
                                            const std::string &_name)
{
  LidarVisualPtr lidar = this->CreateLidarVisualImpl(_id, _name);
  bool result = this->RegisterVisual(lidar);
  return (result) ? lidar : nullptr;
}

//////////////////////////////////////////////////
WireBoxPtr BaseScene::CreateWireBox()
{
  unsigned int objId = this->CreateObjectId();
  std::string objName = this->CreateObjectName(objId, "Wirebox");
  return this->CreateWireBoxImpl(objId, objName);
}

//////////////////////////////////////////////////
TextPtr BaseScene::CreateText()
{
  unsigned int objId = this->CreateObjectId();
  std::string objName = this->CreateObjectName(objId, "Text");
  return this->CreateTextImpl(objId, objName);
}

//////////////////////////////////////////////////
TextPtr BaseScene::CreateTextImpl(const unsigned int /*_id*/,
    const std::string &/*_name*/)
{
  return TextPtr();
}

//////////////////////////////////////////////////
MaterialPtr BaseScene::CreateMaterial(const std::string &_name)
{
  unsigned int objId = this->CreateObjectId();

  std::string objName = _name.empty() ?
      this->CreateObjectName(objId, "Material") : _name;

  MaterialPtr material = this->CreateMaterialImpl(objId, objName);
  this->RegisterMaterial(objName, material);

  return material;
}

//////////////////////////////////////////////////
MaterialPtr BaseScene::CreateMaterial(const common::Material &_material)
{
  MaterialPtr material;
  unsigned int objId = this->CreateObjectId();
  std::string objName = _material.Name().empty() ?
      this->CreateObjectName(objId, "Material") : _material.Name();

  material = this->CreateMaterialImpl(objId, objName);
  material->CopyFrom(_material);
  this->RegisterMaterial(objName, material);

  return material;
}

//////////////////////////////////////////////////
RenderTexturePtr BaseScene::CreateRenderTexture()
{
  unsigned int objId = this->CreateObjectId();
  std::string objName = this->CreateObjectName(objId, "RenderTexture");
  return this->CreateRenderTextureImpl(objId, objName);
}

//////////////////////////////////////////////////
RenderWindowPtr BaseScene::CreateRenderWindow()
{
  unsigned int objId = this->CreateObjectId();
  std::string objName = this->CreateObjectName(objId, "RenderWindow");
  return this->CreateRenderWindowImpl(objId, objName);
}

//////////////////////////////////////////////////
RayQueryPtr BaseScene::CreateRayQuery()
{
  unsigned int objId = this->CreateObjectId();
  std::string objName = this->CreateObjectName(objId, "RayQuery");
  return this->CreateRayQueryImpl(objId, objName);
}

//////////////////////////////////////////////////
ParticleEmitterPtr BaseScene::CreateParticleEmitter()
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateParticleEmitter(objId);
}

//////////////////////////////////////////////////
ParticleEmitterPtr BaseScene::CreateParticleEmitter(unsigned int _id)
{
  std::string objName = this->CreateObjectName(_id, "ParticleEmitter");
  return this->CreateParticleEmitter(_id, objName);
}

//////////////////////////////////////////////////
ParticleEmitterPtr BaseScene::CreateParticleEmitter(const std::string &_name)
{
  unsigned int objId = this->CreateObjectId();
  return this->CreateParticleEmitter(objId, _name);
}

//////////////////////////////////////////////////
ParticleEmitterPtr BaseScene::CreateParticleEmitter(unsigned int _id,
    const std::string &_name)
{
  ParticleEmitterPtr visual = this->CreateParticleEmitterImpl(_id, _name);
  bool result = this->RegisterVisual(visual);
  return (result) ? visual : nullptr;
}

//////////////////////////////////////////////////
void BaseScene::SetSkyEnabled(bool _enabled)  // NOLINT(readability/casting)
{
  // no op, let derived class implement this.
  if (_enabled)
  {
    ignerr << "Sky not supported by: "
           << this->Engine()->Name() << std::endl;
  }
}

//////////////////////////////////////////////////
bool BaseScene::SkyEnabled() const
{
  return false;
}


//////////////////////////////////////////////////
void BaseScene::PreRender()
{
  this->RootVisual()->PreRender();
}

//////////////////////////////////////////////////
void BaseScene::PostRender()
{
}

//////////////////////////////////////////////////
void BaseScene::SetCameraPassCountPerGpuFlush(uint8_t /*_numPass*/)
{
}

//////////////////////////////////////////////////
uint8_t BaseScene::CameraPassCountPerGpuFlush() const
{
  return 0u;
}

//////////////////////////////////////////////////
bool BaseScene::LegacyAutoGpuFlush() const
{
  return true;
}

//////////////////////////////////////////////////
void BaseScene::Clear()
{
  this->DestroyNodes();
  auto root = this->RootVisual();
  if (root)
  {
    root->RemoveChildren();
    this->DestroyNode(root);
  }
  this->DestroyMaterials();
  this->nextObjectId = math::MAX_UI16;
}

//////////////////////////////////////////////////
void BaseScene::Destroy()
{
  // TODO(anyone): destroy context
  this->Clear();
  this->loaded = false;
  this->initialized = false;
}

//////////////////////////////////////////////////
unsigned int BaseScene::CreateObjectId()
{
  return this->nextObjectId--;
}

//////////////////////////////////////////////////
std::string BaseScene::CreateObjectName(unsigned int _id,
    const std::string &_prefix)
{
  std::stringstream ss;
  ss << this->name << "::" << _prefix;
  ss << "(" << std::to_string(_id) << ")";
  return ss.str();
}

//////////////////////////////////////////////////
bool BaseScene::RegisterLight(LightPtr _light)
{
  return (_light) ? this->Lights()->Add(_light) : false;
}

//////////////////////////////////////////////////
bool BaseScene::RegisterSensor(SensorPtr _sensor)
{
  return (_sensor) ? this->Sensors()->Add(_sensor) : false;
}

//////////////////////////////////////////////////
bool BaseScene::RegisterVisual(VisualPtr _visual)
{
  return (_visual) ? this->Visuals()->Add(_visual) : false;
}

//////////////////////////////////////////////////
void BaseScene::CreateNodeStore()
{
  NodeCompositeStorePtr compStore(new BaseNodeCompositeStore);

  // get specific stores
  LightStorePtr lights = this->Lights();
  SensorStorePtr sensors = this->Sensors();
  VisualStorePtr visuals = this->Visuals();

  // convert to node stores
  NodeStorePtr lightNodes(new BaseStoreWrapper<Node, Light>(lights));
  NodeStorePtr sensorNodes(new BaseStoreWrapper<Node, Sensor>(sensors));
  NodeStorePtr visualNodes(new BaseStoreWrapper<Node, Visual>(visuals));

  // add to composite store
  compStore->AddStore(lightNodes);
  compStore->AddStore(sensorNodes);
  compStore->AddStore(visualNodes);

  this->nodes = compStore;
}

//////////////////////////////////////////////////
void BaseScene::CreateMaterials()
{
  MaterialPtr material;

  material = this->CreateMaterial("Default/TransRed");
  material->SetAmbient(1.0, 0.0, 0.0);
  material->SetDiffuse(1.0, 0.0, 0.0);
  material->SetEmissive(1.0, 0.0, 0.0);
  material->SetTransparency(0.5);
  material->SetCastShadows(false);
  material->SetReceiveShadows(false);
  material->SetLightingEnabled(false);

  material = this->CreateMaterial("Default/TransGreen");
  material->SetAmbient(0.0, 1.0, 0.0);
  material->SetDiffuse(0.0, 1.0, 0.0);
  material->SetEmissive(0.0, 1.0, 0.0);
  material->SetTransparency(0.5);
  material->SetCastShadows(false);
  material->SetReceiveShadows(false);
  material->SetLightingEnabled(false);

  material = this->CreateMaterial("Default/TransBlue");
  material->SetAmbient(0.0, 0.0, 1.0);
  material->SetDiffuse(0.0, 0.0, 1.0);
  material->SetEmissive(0.0, 0.0, 1.0);
  material->SetTransparency(0.5);
  material->SetCastShadows(false);
  material->SetReceiveShadows(false);
  material->SetLightingEnabled(false);

  material = this->CreateMaterial("Default/TransYellow");
  material->SetAmbient(1.0, 1.0, 0.0);
  material->SetDiffuse(1.0, 1.0, 0.0);
  material->SetEmissive(1.0, 1.0, 0.0);
  material->SetTransparency(0.5);
  material->SetCastShadows(false);
  material->SetReceiveShadows(false);
  material->SetLightingEnabled(false);

  material = this->CreateMaterial("Default/TransPurple");
  material->SetAmbient(1.0, 0.0, 1.0);
  material->SetDiffuse(1.0, 0.0, 1.0);
  material->SetEmissive(1.0, 0.0, 1.0);
  material->SetTransparency(0.5);
  material->SetCastShadows(false);
  material->SetReceiveShadows(false);
  material->SetLightingEnabled(false);
  material->SetDepthWriteEnabled(false);

  material = this->CreateMaterial("Default/White");
  material->SetAmbient(1.0, 1.0, 1.0);
  material->SetDiffuse(1.0, 1.0, 1.0);
  material->SetEmissive(1.0, 1.0, 1.0);
  material->SetTransparency(0);
  material->SetCastShadows(true);
  material->SetReceiveShadows(true);
  material->SetLightingEnabled(true);

  const char *env = std::getenv("IGN_RENDERING_RESOURCE_PATH");
  std::string resourcePath = (env) ? std::string(env) :
      IGN_RENDERING_RESOURCE_PATH;

  // path to look for CoM material texture
  std::string com_material_texture_path = common::joinPaths(
      resourcePath, "media", "materials", "textures", "com.png");
  material = this->CreateMaterial("Default/CoM");
  material->SetTexture(com_material_texture_path);
  material->SetTransparency(0);
  material->SetCastShadows(false);
  material->SetReceiveShadows(true);
  material->SetLightingEnabled(true);
}
