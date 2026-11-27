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
#include "LuxCoreEngineScene.hh"

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
void LuxCoreEngineScene::LogHandler(const char * /*msg*/)
{
}

//////////////////////////////////////////////////
LuxCoreEngineScene::LuxCoreEngineScene(unsigned int _id,
                                       const std::string &_name)
    : BaseScene(_id, _name)
{
}

//////////////////////////////////////////////////
LuxCoreEngineScene::~LuxCoreEngineScene()
{
}

//////////////////////////////////////////////////
void LuxCoreEngineScene::Fini()
{
}

//////////////////////////////////////////////////
RenderEngine *LuxCoreEngineScene::Engine() const
{
  // \todo(anyone) Implement this function
  return nullptr;
}

//////////////////////////////////////////////////
VisualPtr LuxCoreEngineScene::RootVisual() const
{
  // \todo(anyone) Implement this function
  return nullptr;
}

//////////////////////////////////////////////////
math::Color LuxCoreEngineScene::AmbientLight() const
{
  // \todo(anyone) Implement this function
  return math::Color::Black;
}

//////////////////////////////////////////////////
void LuxCoreEngineScene::SetAmbientLight(const math::Color &/*_color*/)
{
  // \todo(anyone) Implement this function
}

//////////////////////////////////////////////////
DirectionalLightPtr
LuxCoreEngineScene::CreateDirectionalLightImpl(unsigned int _id,
                                               const std::string &_name)
{
  LuxCoreEngineDirectionalLightPtr light(
      new LuxCoreEngineDirectionalLight("sun"));

  sceneLux->Parse(luxrays::Property() << luxrays::Property(
                      "scene.lights." + _name + ".type")("sun"));

  bool result = this->InitObject(light, _id, _name);
  return (result) ? light : nullptr;
}

//////////////////////////////////////////////////
PointLightPtr
LuxCoreEngineScene::CreatePointLightImpl(unsigned int _id,
                                         const std::string &_name)
{
  LuxCoreEnginePointLightPtr light(new LuxCoreEnginePointLight("point"));

  sceneLux->Parse(luxrays::Property() << luxrays::Property(
                      "scene.lights." + _name + ".type")("point"));

  bool result = this->InitObject(light, _id, _name);
  return (result) ? light : nullptr;
}

//////////////////////////////////////////////////
SpotLightPtr LuxCoreEngineScene::CreateSpotLightImpl(unsigned int /*_id*/,
    const std::string &/*_name*/)
{
  // \todo(anyone) Implement this function
  return nullptr;
}

CameraPtr LuxCoreEngineScene::CreateCameraImpl(unsigned int _id,
                                               const std::string &_name)
{
  LuxCoreEngineCameraPtr camera(new LuxCoreEngineCamera);

  sceneLux->Parse(luxrays::Property()
                  << luxrays::Property("scene.camera.type")("perspective"));

  bool result = this->InitObject(camera, _id, _name);
  return (result) ? camera : nullptr;
}

//////////////////////////////////////////////////
DepthCameraPtr
LuxCoreEngineScene::CreateDepthCameraImpl(unsigned int /*_id*/,
                                          const std::string &/*_name*/)
{
  // \todo(anyone) Implement this function
  return nullptr;
}

//////////////////////////////////////////////////
VisualPtr LuxCoreEngineScene::CreateVisualImpl(unsigned int _id,
                                               const std::string &_name)
{
  LuxCoreEngineVisualPtr visual(new LuxCoreEngineVisual);
  bool result = this->InitObject(visual, _id, _name);
  return (result) ? visual : nullptr;
}

//////////////////////////////////////////////////
ArrowVisualPtr
LuxCoreEngineScene::CreateArrowVisualImpl(unsigned int /*_id*/,
                                          const std::string &/*_name*/)
{
  // \todo(anyone) Implement this function
  return nullptr;
}

//////////////////////////////////////////////////
AxisVisualPtr
LuxCoreEngineScene::CreateAxisVisualImpl(unsigned int /*_id*/,
                                         const std::string &/*_name*/)
{
  // \todo(anyone) Implement this function
  return nullptr;
}

//////////////////////////////////////////////////
GeometryPtr LuxCoreEngineScene::CreateBoxImpl(unsigned int _id,
                                              const std::string &_name)
{
  return this->CreateMeshImpl(_id, _name, "unit_box");
}

//////////////////////////////////////////////////
GeometryPtr LuxCoreEngineScene::CreateConeImpl(unsigned int /*_id*/,
                                               const std::string &/*_name*/)
{
  // \todo(anyone) Implement this function
  return nullptr;
}

//////////////////////////////////////////////////
GeometryPtr LuxCoreEngineScene::CreateCylinderImpl(unsigned int /*_id*/,
                                                   const std::string &/*_name*/)
{
  // \todo(anyone) Implement this function
  return nullptr;
}

//////////////////////////////////////////////////
GeometryPtr LuxCoreEngineScene::CreatePlaneImpl(unsigned int _id,
                                                const std::string &_name)
{
  return this->CreateMeshImpl(_id, _name, "unit_plane");
}

//////////////////////////////////////////////////
GeometryPtr LuxCoreEngineScene::CreateSphereImpl(unsigned int /*_id*/,
                                                 const std::string &/*_name*/)
{
  // \todo(anyone) Implement this function
  return nullptr;
}

//////////////////////////////////////////////////
MeshPtr LuxCoreEngineScene::CreateMeshImpl(unsigned int _id,
                                           const std::string &_name,
                                           const std::string &_meshName)
{
  MeshDescriptor descriptor(_meshName);
  return this->CreateMeshImpl(_id, _name, descriptor);
}

//////////////////////////////////////////////////
MeshPtr LuxCoreEngineScene::CreateMeshImpl(unsigned int _id,
                                           const std::string &_name,
                                           const MeshDescriptor &_desc)
{
  LuxCoreEngineMeshPtr mesh = this->meshFactory->Create(_desc, _name);
  if (nullptr == mesh)
    return nullptr;

  if (mesh->SubMeshes()->Size() > 0)
  {
    for (unsigned int i = 0; i < mesh->SubMeshes()->Size(); i++)
    {
      sceneLux->Parse(
          luxrays::Property(
              "scene.objects." + mesh->SubMeshes()->GetByIndex(i)->Name() +
              ".shape")(mesh->SubMeshes()->GetByIndex(i)->Name() + "-submesh")
          << luxrays::Property("scene.objects." +
                               mesh->SubMeshes()->GetByIndex(i)->Name() +
                               ".material")("Default/White"));
    }
  }
  else
  {
    sceneLux->Parse(
        luxrays::Property("scene.objects." + _name + ".shape")(_name + "-mesh")
        << luxrays::Property("scene.objects." + _name +
                             ".material")("Default/White"));
  }

  bool result = this->InitObject(mesh, _id, mesh->Name());
  return (result) ? mesh : nullptr;
}

//////////////////////////////////////////////////
CapsulePtr LuxCoreEngineScene::CreateCapsuleImpl(unsigned int /*_id*/,
                                                 const std::string &/*_name*/)
{
  // \todo(anyone) Implement this function
  return nullptr;
}

//////////////////////////////////////////////////
GridPtr LuxCoreEngineScene::CreateGridImpl(unsigned int /*_id*/,
                                           const std::string &/*_name*/)
{
  // \todo(anyone) Implement this function
  return nullptr;
}

//////////////////////////////////////////////////
MarkerPtr LuxCoreEngineScene::CreateMarkerImpl(unsigned int /*_id*/,
                                               const std::string &/*_name*/)
{
  // \todo(anyone) Implement this function
  return nullptr;
}

//////////////////////////////////////////////////
LidarVisualPtr
LuxCoreEngineScene::CreateLidarVisualImpl(unsigned int /*_id*/,
                                          const std::string &/*_name*/)
{
  // \todo(anyone) Implement this function
  return nullptr;
}

//////////////////////////////////////////////////
HeightmapPtr
LuxCoreEngineScene::CreateHeightmapImpl(unsigned int /*_id*/,
                                        const std::string &/*_name*/,
                                        const HeightmapDescriptor &/*_desc*/)
{
  // \todo(anyone) Implement this function
  return nullptr;
}

//////////////////////////////////////////////////
WireBoxPtr LuxCoreEngineScene::CreateWireBoxImpl(unsigned int /*_id*/,
                                                 const std::string &/*_name*/)
{
  // \todo(anyone) Implement this function
  return nullptr;
}

//////////////////////////////////////////////////
MaterialPtr LuxCoreEngineScene::CreateMaterialImpl(unsigned int _id,
                                                   const std::string &_name)
{
  LuxCoreEngineMaterialPtr material(new LuxCoreEngineMaterial);

  sceneLux->Parse(luxrays::Property() << luxrays::Property(
                      "scene.materials." + Name() + ".id")(_id));

  bool result = this->InitObject(material, _id, _name);
  return (result) ? material : nullptr;
}

//////////////////////////////////////////////////
RenderTexturePtr
LuxCoreEngineScene::CreateRenderTextureImpl(unsigned int /*_id*/,
                                            const std::string &/*_name*/)
{
  // \todo(anyone) Implement this function
  return nullptr;
}

//////////////////////////////////////////////////
RenderWindowPtr
LuxCoreEngineScene::CreateRenderWindowImpl(unsigned int /*_id*/,
                                           const std::string &/*_name*/)
{
  // \todo(anyone) Implement this function
  return nullptr;
}

//////////////////////////////////////////////////
RayQueryPtr LuxCoreEngineScene::CreateRayQueryImpl(unsigned int /*_id*/,
                                                   const std::string &/*_name*/)
{
  // \todo(anyone) Implement this function
  return nullptr;
}

//////////////////////////////////////////////////
COMVisualPtr LuxCoreEngineScene::CreateCOMVisualImpl(unsigned int /*_id*/,
    const std::string &/*_name*/)
{
  // \todo(anyone) Implement this function
  return nullptr;
}

//////////////////////////////////////////////////
InertiaVisualPtr
LuxCoreEngineScene::CreateInertiaVisualImpl(unsigned int /*_id*/,
                                            const std::string &/*_name*/)
{
  // \todo(anyone) Implement this function
  return nullptr;
}

//////////////////////////////////////////////////
JointVisualPtr
LuxCoreEngineScene::CreateJointVisualImpl(unsigned int /*_id*/,
                                          const std::string &/*_name*/)
{
  // \todo(anyone) Implement this function
  return nullptr;
}

//////////////////////////////////////////////////
LightStorePtr LuxCoreEngineScene::Lights() const
{
  return this->lights;
}

//////////////////////////////////////////////////
SensorStorePtr LuxCoreEngineScene::Sensors() const
{
  return this->sensors;
}

//////////////////////////////////////////////////
VisualStorePtr LuxCoreEngineScene::Visuals() const
{
  return this->visuals;
}

//////////////////////////////////////////////////
MaterialMapPtr LuxCoreEngineScene::Materials() const
{
  return this->materials;
}

//////////////////////////////////////////////////
bool LuxCoreEngineScene::LoadImpl()
{
  return true;
}

//////////////////////////////////////////////////
bool LuxCoreEngineScene::InitImpl()
{
  luxcore::Init(LogHandler);

  sceneLux = luxcore::Scene::Create();

  this->CreateStores();
  this->CreateMeshFactory();
  return true;
}

//////////////////////////////////////////////////
ignition::rendering::LightVisualPtr
LuxCoreEngineScene::CreateLightVisualImpl(unsigned int /*_id*/,
                                          const std::string &/*_name*/)
{
  // \todo(anyone) Implement this function
  return nullptr;
}

//////////////////////////////////////////////////
void LuxCoreEngineScene::CreateMeshFactory()
{
  LuxCoreEngineScenePtr sharedThis = this->SharedThis();
  this->meshFactory =
      LuxCoreEngineMeshFactoryPtr(new LuxCoreEngineMeshFactory(sharedThis));
}

//////////////////////////////////////////////////
void LuxCoreEngineScene::CreateStores()
{
  this->lights = LuxCoreEngineLightStorePtr(new LuxCoreEngineLightStore);
  this->sensors = LuxCoreEngineSensorStorePtr(new LuxCoreEngineSensorStore);
  this->visuals = LuxCoreEngineVisualStorePtr(new LuxCoreEngineVisualStore);
  this->materials = LuxCoreEngineMaterialMapPtr(new LuxCoreEngineMaterialMap);
}

//////////////////////////////////////////////////
bool LuxCoreEngineScene::InitObject(LuxCoreEngineObjectPtr _object,
                                    unsigned int _id,
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
LuxCoreEngineScenePtr LuxCoreEngineScene::SharedThis()
{
  ScenePtr sharedBase = this->shared_from_this();
  return std::dynamic_pointer_cast<LuxCoreEngineScene>(sharedBase);
}

//////////////////////////////////////////////////
luxcore::Scene *LuxCoreEngineScene::SceneLux()
{
  return this->sceneLux;
}

