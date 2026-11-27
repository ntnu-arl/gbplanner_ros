/* * Copyright (C) 2015 Open Source Robotics Foundation *
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

#include <gz/common/Console.hh>

#include "gz/rendering/optix/OptixArrowVisual.hh"
#include "gz/rendering/optix/OptixAxisVisual.hh"
#include "gz/rendering/optix/OptixBox.hh"
#include "gz/rendering/optix/OptixCamera.hh"
#include "gz/rendering/optix/OptixCone.hh"
#include "gz/rendering/optix/OptixCylinder.hh"
#include "gz/rendering/optix/OptixGeometry.hh"
#include "gz/rendering/optix/OptixGrid.hh"
#include "gz/rendering/optix/OptixLightManager.hh"
#include "gz/rendering/optix/OptixMeshFactory.hh"
#include "gz/rendering/optix/OptixRayTypes.hh"
#include "gz/rendering/optix/OptixRenderEngine.hh"
#include "gz/rendering/optix/OptixRenderTarget.hh"
#include "gz/rendering/optix/OptixScene.hh"
#include "gz/rendering/optix/OptixSphere.hh"
#include "gz/rendering/optix/OptixStorage.hh"
#include "gz/rendering/optix/OptixVisual.hh"

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
OptixScene::OptixScene(unsigned int _id, const std::string &_name) :
  BaseScene(_id, _name),
  rootVisual(nullptr),
  meshFactory(nullptr),
  optixContext(nullptr),
  optixMissProgram(nullptr),
  optixBoxGeometry(nullptr),
  optixConeGeometry(nullptr),
  optixCylinderGeometry(nullptr),
  optixSphereGeometry(nullptr)
{
  // TODO: move defaults to BaseScene
  this->ambientLight = math::Color::Black;
  this->backgroundColor = math::Color::Black;
  this->gradientBackgroundColor = {math::Color::Black, math::Color::Black,
      math::Color::Black, math::Color::Black};
}

//////////////////////////////////////////////////
OptixScene::~OptixScene()
{
}

//////////////////////////////////////////////////
void OptixScene::Fini()
{
}

//////////////////////////////////////////////////
RenderEngine *OptixScene::Engine() const
{
  return OptixRenderEngine::Instance();
}

//////////////////////////////////////////////////
VisualPtr OptixScene::RootVisual() const
{
  return this->rootVisual;
}

//////////////////////////////////////////////////
math::Color OptixScene::AmbientLight() const
{
  return this->ambientLight;
}

//////////////////////////////////////////////////
void OptixScene::SetAmbientLight(const math::Color &_color)
{
  // TODO: clean up
  this->ambientLight = _color;

  this->optixContext["ambientLightColor"]->setFloat(
      _color.R(), _color.G(), _color.B());
}

//////////////////////////////////////////////////
void OptixScene::SetBackgroundColor(const math::Color &_color)
{
  this->backgroundColor = _color;
  this->optixMissProgram["color"]->setFloat(
      _color.R(), _color.G(), _color.B());
}

//////////////////////////////////////////////////
void OptixScene::SetGradientBackgroundColor(
    const std::array<math::Color, 4> &_colors)
{
  this->gradientBackgroundColor = _colors;
  this->isGradientBackgroundColor = true;

  ignwarn << "SetGradientBackgroundColor not implemented for Optix"
          << std::endl;
}

//////////////////////////////////////////////////
void OptixScene::RemoveGradientBackgroundColor()
{
  this->isGradientBackgroundColor = false;
  ignwarn << "RemoveGradientBackgroundColor not implemented for Optix"
          << std::endl;
}

//////////////////////////////////////////////////
void OptixScene::PreRender()
{
  this->lightManager->Clear();
  BaseScene::PreRender();
  this->lightManager->PreRender();
}

//////////////////////////////////////////////////
void OptixScene::Clear()
{
}

//////////////////////////////////////////////////
void OptixScene::Destroy()
{
}

//////////////////////////////////////////////////
OptixLightManagerPtr OptixScene::LightManager() const
{
  return this->lightManager;
}

//////////////////////////////////////////////////
optix::Context OptixScene::OptixContext() const
{
  return this->optixContext;
}

//////////////////////////////////////////////////
optix::Program OptixScene::CreateOptixProgram(const std::string &_fileBase,
    const std::string &_function)
{
  std::string fileName = OptixRenderEngine::Instance()->PtxFile(_fileBase);
  return this->optixContext->createProgramFromPTXFile(fileName, _function);
}

//////////////////////////////////////////////////
bool OptixScene::LoadImpl()
{
  return true;
}

//////////////////////////////////////////////////
bool OptixScene::InitImpl()
{
  this->CreateContext();
  this->CreateRootVisual();
  this->CreateLightManager();
  this->CreateMeshFactory();
  this->CreateStores();
  return true;
}

//////////////////////////////////////////////////
LightStorePtr OptixScene::Lights() const
{
  return this->lights;
}

//////////////////////////////////////////////////
SensorStorePtr OptixScene::Sensors() const
{
  return this->sensors;
}

//////////////////////////////////////////////////
VisualStorePtr OptixScene::Visuals() const
{
  return this->visuals;
}

//////////////////////////////////////////////////
MaterialMapPtr OptixScene::Materials() const
{
  return this->materials;
}

//////////////////////////////////////////////////
DirectionalLightPtr OptixScene::CreateDirectionalLightImpl(unsigned int _id,
    const std::string &_name)
{
  OptixDirectionalLightPtr light(new OptixDirectionalLight);
  bool result = this->InitObject(light, _id, _name);
  return (result) ? light : nullptr;
}

//////////////////////////////////////////////////
PointLightPtr OptixScene::CreatePointLightImpl(unsigned int _id,
    const std::string &_name)
{
  OptixPointLightPtr light(new OptixPointLight);
  bool result = this->InitObject(light, _id, _name);
  return (result) ? light : nullptr;
}

//////////////////////////////////////////////////
SpotLightPtr OptixScene::CreateSpotLightImpl(unsigned int _id,
    const std::string &_name)
{
  OptixSpotLightPtr light(new OptixSpotLight);
  bool result = this->InitObject(light, _id, _name);
  return (result) ? light : nullptr;
}

//////////////////////////////////////////////////
CameraPtr OptixScene::CreateCameraImpl(unsigned int _id,
    const std::string &_name)
{
  OptixCameraPtr camera(new OptixCamera);
  camera->traceId = this->NextEntryId();
  camera->clearId = this->NextEntryId();
  bool result = this->InitObject(camera, _id, _name);
  return (result) ? camera : nullptr;
}

//////////////////////////////////////////////////
DepthCameraPtr OptixScene::CreateDepthCameraImpl(unsigned int /*_id*/,
    const std::string &/*_name*/)
{
  // TODO(anyone)
  return DepthCameraPtr();
}


//////////////////////////////////////////////////
VisualPtr OptixScene::CreateVisualImpl(unsigned int _id,
    const std::string &_name)
{
  OptixVisualPtr visual(new OptixVisual);
  bool result = this->InitObject(visual, _id, _name);
  return (result) ? visual : nullptr;
}

//////////////////////////////////////////////////
ArrowVisualPtr OptixScene::CreateArrowVisualImpl(unsigned int _id,
    const std::string &_name)
{
  OptixArrowVisualPtr visual(new OptixArrowVisual);
  bool result = this->InitObject(visual, _id, _name);
  return (result) ? visual : nullptr;
}

//////////////////////////////////////////////////
AxisVisualPtr OptixScene::CreateAxisVisualImpl(unsigned int _id,
    const std::string &_name)
{
  OptixAxisVisualPtr visual(new OptixAxisVisual);
  bool result = this->InitObject(visual, _id, _name);
  return (result) ? visual : nullptr;
}

//////////////////////////////////////////////////
GeometryPtr OptixScene::CreateBoxImpl(unsigned int _id,
    const std::string &_name)
{
  if (!this->optixBoxGeometry)
  {
    this->optixBoxGeometry =
        OptixBox::CreateOptixGeometry(this->SharedThis());
  }

  OptixBoxPtr box(new OptixBox);
  box->optixGeometry = this->optixBoxGeometry;
  bool result = this->InitObject(box, _id, _name);
  box->SetMaterial(this->CreateMaterial());
  return (result) ? box : nullptr;
}

//////////////////////////////////////////////////
GeometryPtr OptixScene::CreateConeImpl(unsigned int _id,
    const std::string &_name)
{
  if (!this->optixConeGeometry)
  {
    this->optixConeGeometry =
        OptixCone::CreateOptixGeometry(this->SharedThis());
  }

  OptixConePtr cone(new OptixCone);
  cone->optixGeometry = this->optixConeGeometry;
  bool result = this->InitObject(cone, _id, _name);
  cone->SetMaterial(this->CreateMaterial());
  return (result) ? cone : nullptr;
}

//////////////////////////////////////////////////
GeometryPtr OptixScene::CreateCylinderImpl(unsigned int _id,
    const std::string &_name)
{
  if (!this->optixCylinderGeometry)
  {
    this->optixCylinderGeometry =
        OptixCylinder::CreateOptixGeometry(this->SharedThis());
  }

  OptixCylinderPtr cylinder(new OptixCylinder);
  cylinder->optixGeometry = this->optixCylinderGeometry;
  bool result = this->InitObject(cylinder, _id, _name);
  cylinder->SetMaterial(this->CreateMaterial());
  return (result) ? cylinder : nullptr;
}

//////////////////////////////////////////////////
GeometryPtr OptixScene::CreatePlaneImpl(unsigned int _id,
    const std::string &_name)
{
  return this->CreateMeshImpl(_id, _name, "unit_plane");
}

//////////////////////////////////////////////////
GeometryPtr OptixScene::CreateSphereImpl(unsigned int _id,
    const std::string &_name)
{
  if (!this->optixSphereGeometry)
  {
    this->optixSphereGeometry =
        OptixSphere::CreateOptixGeometry(this->SharedThis());
  }

  OptixSpherePtr sphere(new OptixSphere);
  sphere->optixGeometry = this->optixSphereGeometry;
  bool result = this->InitObject(sphere, _id, _name);
  sphere->SetMaterial(this->CreateMaterial());
  return (result) ? sphere : nullptr;
}

//////////////////////////////////////////////////
MeshPtr OptixScene::CreateMeshImpl(unsigned int _id, const std::string &_name,
    const std::string &_meshName)
{
  MeshDescriptor descriptor(_meshName);
  return this->CreateMeshImpl(_id, _name, descriptor);
}

//////////////////////////////////////////////////
MeshPtr OptixScene::CreateMeshImpl(unsigned int _id, const std::string &_name,
    const MeshDescriptor &_desc)
{
  OptixMeshPtr mesh = this->meshFactory->Create(_desc);
  bool result = this->InitObject(mesh, _id, _name);
  return (result) ? mesh : nullptr;
}

//////////////////////////////////////////////////
GridPtr OptixScene::CreateGridImpl(
    unsigned int _id, const std::string &_name)
{
  // TODO implement optix grid! Use box as stub for now.
  if (!this->optixBoxGeometry)
  {
    this->optixBoxGeometry =
        OptixBox::CreateOptixGeometry(this->SharedThis());
  }

  OptixGridPtr grid(new OptixGrid);
  grid->optixGeometry = this->optixBoxGeometry;
  bool result = this->InitObject(grid, _id, _name);
  grid->SetMaterial(this->CreateMaterial());
  return (result) ? grid: nullptr;
}

//////////////////////////////////////////////////
MaterialPtr OptixScene::CreateMaterialImpl(unsigned int _id,
    const std::string &_name)
{
  OptixMaterialPtr material(new OptixMaterial);
  bool result = this->InitObject(material, _id, _name);
  return (result) ? material : nullptr;
}

//////////////////////////////////////////////////
RenderTexturePtr OptixScene::CreateRenderTextureImpl(
    unsigned int _id, const std::string &_name)
{
  OptixRenderTexturePtr renderTexture(new OptixRenderTexture);
  bool result = this->InitObject(renderTexture, _id, _name);
  return (result) ? renderTexture : nullptr;
}

//////////////////////////////////////////////////
RenderWindowPtr OptixScene::CreateRenderWindowImpl(
    unsigned int _id, const std::string &_name)
{
  OptixRenderWindowPtr renderWindow(new OptixRenderWindow);
  bool result = this->InitObject(renderWindow, _id, _name);
  return (result) ? renderWindow: nullptr;
}

//////////////////////////////////////////////////
RayQueryPtr OptixScene::CreateRayQueryImpl(
    unsigned int /*_id*/, const std::string &/*_name*/)
{
  /// TODO
  return nullptr;
}

//////////////////////////////////////////////////
bool OptixScene::InitObject(OptixObjectPtr _object, unsigned int _id,
    const std::string &_name)
{
  // assign needed varibles
  _object->id = _id;
  _object->name = _name;
  _object->scene = this->SharedThis();

  // initialize object
  _object->Load();
  _object->Init();

  return true;
}

//////////////////////////////////////////////////
unsigned int OptixScene::NextEntryId()
{
  unsigned int entryId = this->optixContext->getEntryPointCount();
  this->optixContext->setEntryPointCount(entryId + 1);
  return entryId;
}

//////////////////////////////////////////////////
void OptixScene::CreateContext()
{
  this->optixContext = optix::Context::create();

  // TODO: set dynamically
  // this->optixContext->setStackSize(65536);
  // TODO: set dynamically
  // this->optixContext->setStackSize(45536);
  // TODO: set dynamically
  this->optixContext->setStackSize(10000);
  this->optixContext->setEntryPointCount(0);
  this->optixContext->setRayTypeCount(RT_COUNT);

  // TODO: setup programatically
  this->optixContext["sceneEpsilon"]->setFloat(1E-4);
  this->optixContext["maxReflectionDepth"]->setInt(3);
  this->optixContext["maxRefractionDepth"]->setInt(3);
  this->optixContext["importanceCutoff"]->setFloat(0.01);

  // TODO: remove after testing
  this->optixContext->setPrintEnabled(true);
  this->optixContext->setPrintBufferSize(4096);

  // TODO: clean up code
  this->optixMissProgram = this->CreateOptixProgram("OptixMissProgram", "Miss");
  this->optixMissProgram["color"]->setFloat(0, 0, 0);
  this->optixContext->setMissProgram(RT_RADIANCE, this->optixMissProgram);
}

//////////////////////////////////////////////////
void OptixScene::CreateRootVisual()
{
  // create unregistered visual
  this->rootVisual = OptixVisualPtr(new OptixVisual);
  unsigned int rootId = this->CreateObjectId();
  std::string rootName = this->CreateObjectName(rootId, "ROOT");

  // check if root visual created successfully
  if (!this->InitObject(this->rootVisual, rootId, rootName))
  {
    ignerr << "Unable to create root visual" << std::endl;
    this->rootVisual = nullptr;
  }

  // create transform-less optix root group
  this->optixRootGroup = this->optixContext->createGroup();
  this->optixRootAccel = this->optixContext->createAcceleration(
      "NoAccel", "NoAccel");
  this->optixRootGroup->setAcceleration(this->optixRootAccel);

  // attach root visual to actual root group
  this->optixRootGroup->addChild(this->rootVisual->OptixGroup());
  optix::Group rootGroup = this->rootVisual->OptixGroup();
  optixContext["rootGroup"]->set(this->optixRootGroup);
}

//////////////////////////////////////////////////
void OptixScene::CreateLightManager()
{
  OptixScenePtr sharedThis = this->SharedThis();
  this->lightManager = OptixLightManagerPtr(new OptixLightManager(sharedThis));
}

//////////////////////////////////////////////////
void OptixScene::CreateMeshFactory()
{
  OptixScenePtr sharedThis = this->SharedThis();
  this->meshFactory = OptixMeshFactoryPtr(new OptixMeshFactory(sharedThis));
}

//////////////////////////////////////////////////
void OptixScene::CreateStores()
{
  this->lights = OptixLightStorePtr(new OptixLightStore);
  this->sensors = OptixSensorStorePtr(new OptixSensorStore);
  this->visuals = OptixVisualStorePtr(new OptixVisualStore);
  this->materials = OptixMaterialMapPtr(new OptixMaterialMap);
}

//////////////////////////////////////////////////
OptixScenePtr OptixScene::SharedThis()
{
  ScenePtr sharedBase = this->shared_from_this();
  return std::dynamic_pointer_cast<OptixScene>(sharedBase);
}
