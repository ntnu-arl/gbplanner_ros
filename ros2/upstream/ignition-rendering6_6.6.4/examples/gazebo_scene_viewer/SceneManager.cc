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

#include <gazebo/common/Events.hh>

#include <gz/common/MeshManager.hh>
#include <gz/common/Console.hh>
#include <gz/math/Helpers.hh>

#include <gz/rendering/Camera.hh>
#include <gz/rendering/Capsule.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>

#include "SceneManager.hh"
#include "SceneManagerPrivate.hh"

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
SceneManager::SceneManager() :
  pimpl(new SceneManagerPrivate)
{
}

//////////////////////////////////////////////////
SceneManager::~SceneManager()
{
  this->Fini();
  delete this->pimpl;
}

//////////////////////////////////////////////////
void SceneManager::Load()
{
  this->pimpl->Load();
}

//////////////////////////////////////////////////
void SceneManager::Init()
{
  this->pimpl->Init();
}

//////////////////////////////////////////////////
void SceneManager::Fini()
{
  this->pimpl->Fini();
}

//////////////////////////////////////////////////
unsigned int SceneManager::SceneCount() const
{
  return this->pimpl->SceneCount();
}

//////////////////////////////////////////////////
bool SceneManager::HasScene(unsigned int _id) const
{
  return this->pimpl->HasScene(_id);
}

//////////////////////////////////////////////////
bool SceneManager::HasScene(const std::string &_name) const
{
  return this->pimpl->HasScene(_name);
}

//////////////////////////////////////////////////
bool SceneManager::HasScene(ConstScenePtr _scene) const
{
  return this->pimpl->HasScene(_scene);
}

//////////////////////////////////////////////////
ScenePtr SceneManager::Scene(unsigned int _id) const
{
  return this->pimpl->Scene(_id);
}

//////////////////////////////////////////////////
ScenePtr SceneManager::Scene(const std::string &_name) const
{
  return this->pimpl->Scene(_name);
}

//////////////////////////////////////////////////
ScenePtr SceneManager::SceneAt(unsigned int _index) const
{
  return this->pimpl->SceneAt(_index);
}

//////////////////////////////////////////////////
void SceneManager::AddScene(ScenePtr _scene)
{
  this->pimpl->AddScene(_scene);
}

//////////////////////////////////////////////////
ScenePtr SceneManager::RemoveScene(unsigned int _id)
{
  return this->pimpl->RemoveScene(_id);
}

//////////////////////////////////////////////////
ScenePtr SceneManager::RemoveScene(const std::string &_name)
{
  return this->pimpl->RemoveScene(_name);
}

//////////////////////////////////////////////////
ScenePtr SceneManager::RemoveScene(ScenePtr _scene)
{
  return this->pimpl->RemoveScene(_scene);
}

//////////////////////////////////////////////////
ScenePtr SceneManager::RemoveSceneAt(unsigned int _index)
{
  return this->pimpl->RemoveSceneAt(_index);
}

//////////////////////////////////////////////////
void SceneManager::RemoveScenes()
{
  this->pimpl->RemoveScenes();
}

//////////////////////////////////////////////////
void SceneManager::UpdateScenes()
{
  this->pimpl->UpdateScenes();
}

//////////////////////////////////////////////////
SceneManagerPrivate::SceneManagerPrivate() :
  currentSceneManager(new CurrentSceneManager),
  newSceneManager(new NewSceneManager),
  sceneRequestId(-1),
  promotionNeeded(false)
{
}

//////////////////////////////////////////////////
SceneManagerPrivate::~SceneManagerPrivate()
{
    newSceneManager.reset();
    currentSceneManager.reset();
}

//////////////////////////////////////////////////
void SceneManagerPrivate::Load()
{
}

//////////////////////////////////////////////////
void SceneManagerPrivate::Init()
{
//! [init scene manager]
  // listen for pre-render events
  this->preRenderConn = gazebo::event::Events::ConnectPreRender(
        std::bind(&SceneManagerPrivate::UpdateScenes, this));

  // setup transport communication node
  this->transportNode = gazebo::transport::NodePtr(
      new gazebo::transport::Node());
  this->transportNode->Init();

  // create publisher for sending scene request
  this->requestPub =
      this->transportNode->Advertise<gazebo::msgs::Request>("~/request");

  // listen for deletion requests
  this->requestSub = this->transportNode->Subscribe("~/request",
      &SceneManagerPrivate::OnRequest, this);

  // listen for scene & deletion requests responses
  this->responseSub = this->transportNode->Subscribe("~/response",
      &SceneManagerPrivate::OnResponse, this);

  // listen for to light updates
  this->lightSub = this->transportNode->Subscribe("~/light",
      &SceneManagerPrivate::OnLightUpdate, this);

  // TODO(anyone): handle non-local model info

  // listen for to model updates
  this->modelSub = this->transportNode->Subscribe("~/model/info",
      &SceneManagerPrivate::OnModelUpdate, this);

  // listen for to joint updates
  this->jointSub = this->transportNode->Subscribe("~/joint",
      &SceneManagerPrivate::OnJointUpdate, this);

  // listen for to visual updates
  this->visualSub = this->transportNode->Subscribe("~/visual",
      &SceneManagerPrivate::OnVisualUpdate, this);

  // listen for to sensor updates
  this->sensorSub = this->transportNode->Subscribe("~/sensor",
      &SceneManagerPrivate::OnSensorUpdate, this);

  // TODO(anyone): handle non-local pose info

  // listen for to pose updates
  this->poseSub = this->transportNode->Subscribe("~/pose/local/info",
      &SceneManagerPrivate::OnPoseUpdate, this);
//! [init scene manager]
}

//////////////////////////////////////////////////
void SceneManagerPrivate::Fini()
{
  // block all message receival during cleanup
  std::lock_guard<std::mutex> generalLock(this->generalMutex);
  std::lock_guard<std::mutex> poseLock(this->poseMutex);

  this->transportNode->Fini();
}

//////////////////////////////////////////////////
unsigned int SceneManagerPrivate::SceneCount() const
{
  unsigned int count = 0;
  count += this->currentSceneManager->SceneCount();
  count += this->newSceneManager->SceneCount();
  return count;
}

//////////////////////////////////////////////////
bool SceneManagerPrivate::HasScene(unsigned int _id) const
{
  return this->currentSceneManager->HasScene(_id) ||
         this->newSceneManager->HasScene(_id);
}

//////////////////////////////////////////////////
bool SceneManagerPrivate::HasScene(const std::string &_name) const
{
  return this->currentSceneManager->HasScene(_name) ||
         this->newSceneManager->HasScene(_name);
}

//////////////////////////////////////////////////
bool SceneManagerPrivate::HasScene(ConstScenePtr _scene) const
{
  return this->currentSceneManager->HasScene(_scene) ||
         this->newSceneManager->HasScene(_scene);
}

//////////////////////////////////////////////////
ScenePtr SceneManagerPrivate::Scene(unsigned int _id) const
{
  ScenePtr scene = this->currentSceneManager->Scene(_id);
  return (scene) ? scene : this->newSceneManager->Scene(_id);
}

//////////////////////////////////////////////////
ScenePtr SceneManagerPrivate::Scene(const std::string &_name) const
{
  ScenePtr scene = this->currentSceneManager->Scene(_name);
  return (scene) ? scene : this->newSceneManager->Scene(_name);
}

//////////////////////////////////////////////////
ScenePtr SceneManagerPrivate::SceneAt(unsigned int _index) const
{
  ScenePtr scene = this->currentSceneManager->SceneAt(_index);
  return (scene) ? scene : this->newSceneManager->SceneAt(_index);
}

//////////////////////////////////////////////////
void SceneManagerPrivate::AddScene(ScenePtr _scene)
{
  // block all message receival during update
  std::lock_guard<std::mutex> generalLock(this->generalMutex);
  std::lock_guard<std::mutex> poseLock(this->poseMutex);

  this->newSceneManager->AddScene(_scene);

  // check if new request needed
  if (this->sceneRequestId < 0)
  {
    this->SendSceneRequest();
  }
}

//////////////////////////////////////////////////
ScenePtr SceneManagerPrivate::RemoveScene(unsigned int _id)
{
  // block all message receival during update
  std::lock_guard<std::mutex> generalLock(this->generalMutex);
  std::lock_guard<std::mutex> poseLock(this->poseMutex);

  ScenePtr scene = this->currentSceneManager->RemoveScene(_id);
  return (scene) ? scene : this->newSceneManager->RemoveScene(_id);
}

//////////////////////////////////////////////////
ScenePtr SceneManagerPrivate::RemoveScene(const std::string &_name)
{
  // block all message receival during update
  std::lock_guard<std::mutex> generalLock(this->generalMutex);
  std::lock_guard<std::mutex> poseLock(this->poseMutex);

  ScenePtr scene = this->currentSceneManager->RemoveScene(_name);
  return (scene) ? scene : this->newSceneManager->RemoveScene(_name);
}

//////////////////////////////////////////////////
ScenePtr SceneManagerPrivate::RemoveScene(ScenePtr _scene)
{
  // block all message receival during update
  std::lock_guard<std::mutex> generalLock(this->generalMutex);
  std::lock_guard<std::mutex> poseLock(this->poseMutex);

  ScenePtr scene = this->currentSceneManager->RemoveScene(_scene);
  return (scene) ? scene : this->newSceneManager->RemoveScene(_scene);
}

//////////////////////////////////////////////////
ScenePtr SceneManagerPrivate::RemoveSceneAt(unsigned int _index)
{
  // block all message receival during update
  std::lock_guard<std::mutex> generalLock(this->generalMutex);
  std::lock_guard<std::mutex> poseLock(this->poseMutex);

  ScenePtr scene = this->currentSceneManager->RemoveSceneAt(_index);
  return (scene) ? scene : this->newSceneManager->RemoveSceneAt(_index);
}

//////////////////////////////////////////////////
void SceneManagerPrivate::RemoveScenes()
{
  // block all message receival during update
  std::lock_guard<std::mutex> generalLock(this->generalMutex);
  std::lock_guard<std::mutex> poseLock(this->poseMutex);

  this->currentSceneManager->RemoveScenes();
  this->newSceneManager->RemoveScenes();
}

//////////////////////////////////////////////////
void SceneManagerPrivate::UpdateScenes()
{
  // block all message receival during update
  std::lock_guard<std::mutex> generalLock(this->generalMutex);
  std::lock_guard<std::mutex> poseLock(this->poseMutex);

  this->currentSceneManager->UpdateScenes();

  // check if scene reponse received
  if (this->promotionNeeded)
  {
    this->newSceneManager->UpdateScenes();
    this->PromoteNewScenes();
    this->promotionNeeded = false;
  }
}

//////////////////////////////////////////////////
void SceneManagerPrivate::SendSceneRequest()
{
  gazebo::msgs::Request *request = gazebo::msgs::CreateRequest("scene_info");
  this->sceneRequestId = request->id();
  this->requestPub->Publish(*request);
}

//////////////////////////////////////////////////
void SceneManagerPrivate::OnRequest(::ConstRequestPtr &_requestMsg)
{
  // check if deletion request
  if (_requestMsg->request() == "entity_delete")
  {
    // record details & wait for response
    unsigned int requestId = _requestMsg->id();
    std::string name = _requestMsg->data();
    this->requestedRemovals[requestId] = name;
  }
}

//////////////////////////////////////////////////
void SceneManagerPrivate::OnResponse(::ConstResponsePtr &_responseMsg)
{
  // check if response to our scene request
  if (_responseMsg->id() == this->sceneRequestId)
  {
    this->OnSceneResponse(_responseMsg);
  }

  // else check if response to a delete request
  else if (_responseMsg->request() == "entity_delete")
  {
    this->OnRemovalResponse(_responseMsg);
  }
}

//////////////////////////////////////////////////
void SceneManagerPrivate::OnSceneResponse(::ConstResponsePtr &_responseMsg)
{
  // block all message receival during update
  std::lock_guard<std::mutex> generalLock(this->generalMutex);
  std::lock_guard<std::mutex> poseLock(this->poseMutex);

  // pass scene response to new scene manager
  const std::string &data = _responseMsg->serialized_data();
  this->newSceneManager->SetSceneData(data);

  // update state
  this->promotionNeeded = true;
  this->sceneRequestId = -1;
}

//////////////////////////////////////////////////
void SceneManagerPrivate::OnRemovalResponse(::ConstResponsePtr &_responseMsg)
{
  // TODO(anyone): check if message sent after scene response

  unsigned int requestId = _responseMsg->id();

  // check if delete was successful
  if (_responseMsg->response() == "success")
  {
    std::string name = this->requestedRemovals[requestId];
    this->OnRemovalUpdate(name);
  }

  // delete request regardless
  this->requestedRemovals.erase(requestId);
}

//////////////////////////////////////////////////
void SceneManagerPrivate::OnSceneUpdate(::ConstScenePtr)
{
  // block all message receival during update
  std::lock_guard<std::mutex> generalLock(this->generalMutex);
  std::lock_guard<std::mutex> poseLock(this->poseMutex);

  this->DemoteCurrentScenes();
  this->sceneRequestId = -1;
  this->SendSceneRequest();
}

//////////////////////////////////////////////////
void SceneManagerPrivate::OnLightUpdate(::ConstLightPtr &_lightMsg)
{
  // wait for update unlock before adding message
  std::lock_guard<std::mutex> lock(this->generalMutex);

  if (this->currentSceneManager == nullptr || this->newSceneManager == nullptr)
  {
    return;
  }

  this->currentSceneManager->OnLightUpdate(_lightMsg);
  this->newSceneManager->OnLightUpdate(_lightMsg);
}

//////////////////////////////////////////////////
void SceneManagerPrivate::OnModelUpdate(::ConstModelPtr &_modelMsg)
{
  // wait for update unlock before adding message
  std::lock_guard<std::mutex> lock(this->generalMutex);

  if (this->currentSceneManager == nullptr || this->newSceneManager == nullptr)
  {
    return;
  }

  this->currentSceneManager->OnModelUpdate(_modelMsg);
  this->newSceneManager->OnModelUpdate(_modelMsg);
}

//////////////////////////////////////////////////
void SceneManagerPrivate::OnJointUpdate(::ConstJointPtr &_jointMsg)
{
  // wait for update unlock before adding message
  std::lock_guard<std::mutex> lock(this->generalMutex);

  if (this->currentSceneManager == nullptr || this->newSceneManager == nullptr)
  {
    return;
  }

  this->currentSceneManager->OnJointUpdate(_jointMsg);
  this->newSceneManager->OnJointUpdate(_jointMsg);
}

//////////////////////////////////////////////////
void SceneManagerPrivate::OnVisualUpdate(::ConstVisualPtr &_visualMsg)
{
  // wait for update unlock before adding message
  std::lock_guard<std::mutex> lock(this->generalMutex);

  if (this->currentSceneManager == nullptr || this->newSceneManager == nullptr)
  {
    return;
  }

  this->currentSceneManager->OnVisualUpdate(_visualMsg);
  this->newSceneManager->OnVisualUpdate(_visualMsg);
}

//////////////////////////////////////////////////
void SceneManagerPrivate::OnSensorUpdate(::ConstSensorPtr &_sensorMsg)
{
  // wait for update unlock before adding message
  std::lock_guard<std::mutex> lock(this->generalMutex);

  if (this->currentSceneManager == nullptr || this->newSceneManager == nullptr)
  {
    return;
  }

  this->currentSceneManager->OnSensorUpdate(_sensorMsg);
  this->newSceneManager->OnSensorUpdate(_sensorMsg);
}

//////////////////////////////////////////////////
void SceneManagerPrivate::OnPoseUpdate(::ConstPosesStampedPtr &_posesMsg)
{
  // wait for update unlock before adding message
  std::lock_guard<std::mutex> lock(this->poseMutex);

  if (this->currentSceneManager == nullptr || this->newSceneManager == nullptr)
  {
    return;
  }

  this->currentSceneManager->OnPoseUpdate(_posesMsg);
  this->newSceneManager->OnPoseUpdate(_posesMsg);
}

//////////////////////////////////////////////////
void SceneManagerPrivate::OnRemovalUpdate(const std::string &_name)
{
  // wait for update unlock before adding message
  std::lock_guard<std::mutex> lock(this->poseMutex);

  if (this->currentSceneManager == nullptr || this->newSceneManager == nullptr)
  {
    return;
  }

  this->currentSceneManager->OnRemovalUpdate(_name);
  this->newSceneManager->OnRemovalUpdate(_name);
}

//////////////////////////////////////////////////
void SceneManagerPrivate::PromoteNewScenes()
{
  unsigned int newSceneCount = this->newSceneManager->SceneCount();

  // move each new scene
  for (unsigned int i = 0; i < newSceneCount; ++i)
  {
    ScenePtr scene = this->newSceneManager->SceneAt(i);
    this->currentSceneManager->AddScene(scene);
  }

  // clear new scenes
  this->newSceneManager->Clear();
}

//////////////////////////////////////////////////
void SceneManagerPrivate::DemoteCurrentScenes()
{
  // promote new first to clear messages
  // & to maintain scene index order
  this->PromoteNewScenes();

  // get updated current scene size
  unsigned int currSceneCount = this->currentSceneManager->SceneCount();

  // move each current scene
  for (unsigned int i = 0; i < currSceneCount; ++i)
  {
    ScenePtr scene = this->currentSceneManager->SceneAt(i);
    this->newSceneManager->AddScene(scene);
    scene->Clear();
  }

  // clear current scenes
  this->currentSceneManager->Clear();
}

//////////////////////////////////////////////////
SubSceneManager::SubSceneManager() :
  activeScene(nullptr)
{
  this->CreateGeometryFunctionMap();
}

//////////////////////////////////////////////////
SubSceneManager::~SubSceneManager()
{
}

//////////////////////////////////////////////////
unsigned int SubSceneManager::SceneCount() const
{
  // TODO(anyone): encapsulate

  return this->scenes.size();
}

//////////////////////////////////////////////////
bool SubSceneManager::HasScene(unsigned int _id) const
{
  // TODO(anyone): encapsulate

  for (auto scene : this->scenes)
  {
    if (scene->Id() == _id)
    {
      return true;
    }
  }

  return false;
}

//////////////////////////////////////////////////
bool SubSceneManager::HasScene(const std::string &_name) const
{
  // TODO(anyone): encapsulate

  for (auto scene : this->scenes)
  {
    if (scene->Name() == _name)
    {
      return true;
    }
  }

  return false;
}

//////////////////////////////////////////////////
bool SubSceneManager::HasScene(ConstScenePtr _scene) const
{
  // TODO(anyone): encapsulate

  for (auto scene : this->scenes)
  {
    if (scene == _scene)
    {
      return true;
    }
  }

  return false;
}

//////////////////////////////////////////////////
ScenePtr SubSceneManager::Scene(unsigned int _id) const
{
  // TODO(anyone): encapsulate

  for (auto scene : this->scenes)
  {
    if (scene->Id() == _id)
    {
      return scene;
    }
  }

  return nullptr;
}

//////////////////////////////////////////////////
ScenePtr SubSceneManager::Scene(const std::string &_name) const
{
  // TODO(anyone): encapsulate

  for (auto scene : this->scenes)
  {
    if (scene->Name() == _name)
    {
      return scene;
    }
  }

  return nullptr;
}

//////////////////////////////////////////////////
ScenePtr SubSceneManager::SceneAt(unsigned int _index) const
{
  // TODO(anyone): encapsulate

  if (_index >= this->SceneCount())
  {
    ignerr << "Invalid scene index: " << _index << std::endl;
    return nullptr;
  }

  auto iter = this->scenes.begin();
  std::advance(iter, _index);
  return *iter;
}

//////////////////////////////////////////////////
void SubSceneManager::AddScene(ScenePtr _scene)
{
  // TODO(anyone): encapsulate

  if (!_scene)
  {
    ignerr << "Cannot add null scene pointer" << std::endl;
    return;
  }

  if (this->HasScene(_scene))
  {
    ignerr << "Scene has already been added" << std::endl;
    return;
  }

  this->scenes.push_back(_scene);
}

//////////////////////////////////////////////////
ScenePtr SubSceneManager::RemoveScene(unsigned int _id)
{
  // TODO(anyone): encapsulate

  for (auto scene : this->scenes)
  {
    if (scene->Id() == _id)
    {
      return scene;
    }
  }

  return nullptr;
}

//////////////////////////////////////////////////
ScenePtr SubSceneManager::RemoveScene(const std::string &_name)
{
  // TODO(anyone): encapsulate

  for (auto scene : this->scenes)
  {
    if (scene->Name() == _name)
    {
      return scene;
    }
  }

  return nullptr;
}

//////////////////////////////////////////////////
ScenePtr SubSceneManager::RemoveScene(ScenePtr _scene)
{
  // TODO(anyone): encapsulate

  for (auto scene : this->scenes)
  {
    if (scene == _scene)
    {
      return scene;
    }
  }

  return nullptr;
}

//////////////////////////////////////////////////
ScenePtr SubSceneManager::RemoveSceneAt(unsigned int _index)
{
  // TODO(anyone): encapsulate

  if (_index >= this->SceneCount())
  {
    ignerr << "Invalid scene index: " << _index << std::endl;
    return nullptr;
  }

  auto iter = this->scenes.begin();
  std::advance(iter, _index);
  ScenePtr scene = *iter;
  this->scenes.erase(iter);
  return scene;
}

//////////////////////////////////////////////////
void SubSceneManager::RemoveScenes()
{
  this->Clear();
}

//////////////////////////////////////////////////
void SubSceneManager::UpdateScenes()
{
  // update each scene in list
  for (auto scene : this->scenes)
  {
    this->activeScene = scene;
    this->ProcessMessages();
  }

  ClearMessages();
  this->activeScene = nullptr;
}

//////////////////////////////////////////////////
void SubSceneManager::Clear()
{
  this->scenes.clear();
  this->ClearMessages();
  this->activeScene = nullptr;
}

//////////////////////////////////////////////////
void SubSceneManager::OnLightUpdate(::ConstLightPtr &_lightMsg)
{
  // check if message needed
  if (!this->scenes.empty())
  {
    this->lightMsgs.push_back(*_lightMsg);
  }
}

//////////////////////////////////////////////////
void SubSceneManager::OnModelUpdate(::ConstModelPtr &_modelMsg)
{
  // check if message needed
  if (!this->scenes.empty())
  {
    this->modelMsgs.push_back(*_modelMsg);
  }
}

//////////////////////////////////////////////////
void SubSceneManager::OnJointUpdate(::ConstJointPtr &_jointMsg)
{
  // check if message needed
  if (!this->scenes.empty())
  {
    this->jointMsgs.push_back(*_jointMsg);
  }
}

//////////////////////////////////////////////////
void SubSceneManager::OnVisualUpdate(::ConstVisualPtr &_visualMsg)
{
  // check if message needed
  if (!this->scenes.empty())
  {
    this->visualMsgs.push_back(*_visualMsg);
  }
}

//////////////////////////////////////////////////
void SubSceneManager::OnSensorUpdate(::ConstSensorPtr &_sensorMsg)
{
  // check if message needed
  if (!this->scenes.empty())
  {
    this->sensorMsgs.push_back(*_sensorMsg);
  }
}

//////////////////////////////////////////////////
void SubSceneManager::OnRemovalUpdate(const std::string &_name)
{
  // check if message needed
  if (!this->scenes.empty())
  {
    this->approvedRemovals.push_back(_name);
  }
}

//////////////////////////////////////////////////
void SubSceneManager::ProcessMessages()
{
  // process each queued message
//! [process message]
  this->ProcessLights();
  this->ProcessModels();
  this->ProcessJoints();
  this->ProcessVisuals();
  this->ProcessSensors();
  this->ProcessPoses();
  this->ProcessRemovals();
//! [process message]

  // flush changes to scene
  this->activeScene->SetTime(this->timePosesReceived);
  this->activeScene->PreRender();
}

//////////////////////////////////////////////////
void SubSceneManager::ClearMessages()
{
  this->lightMsgs.clear();
  this->modelMsgs.clear();
  this->jointMsgs.clear();
  this->visualMsgs.clear();
  this->sensorMsgs.clear();
  this->approvedRemovals.clear();
}

//////////////////////////////////////////////////
void SubSceneManager::ProcessLight(const gazebo::msgs::Light &_lightMsg)
{
  // TODO(anyone): get parent when protobuf message is updated
  this->ProcessLight(_lightMsg, this->activeScene->RootVisual());
}

//////////////////////////////////////////////////
void SubSceneManager::ProcessLight(const gazebo::msgs::Light &_lightMsg,
    VisualPtr _parent)
{
  // check if type specified
  if (_lightMsg.has_type())
  {
    gazebo::msgs::Light::LightType type = _lightMsg.type();

    // determine light type
    switch (_lightMsg.type())
    {
      case gazebo::msgs::Light::POINT:
        this->ProcessPointLight(_lightMsg, _parent);
        return;

      case gazebo::msgs::Light::SPOT:
        this->ProcessSpotLight(_lightMsg, _parent);
        return;

      case gazebo::msgs::Light::DIRECTIONAL:
        this->ProcessDirectionalLight(_lightMsg, _parent);
        return;

      default:
        ignerr << "Invalid light type: " << type << std::endl;
        return;
    }
  }

  // update existing light
  std::string name = _lightMsg.name();
  LightPtr light = this->activeScene->LightByName(name);
  if (light) this->ProcessLightImpl(_lightMsg, light);
}

//////////////////////////////////////////////////
void SubSceneManager::ProcessDirectionalLight(
    const gazebo::msgs::Light &_lightMsg, VisualPtr _parent)
{
  DirectionalLightPtr light = this->DirectionalLight(_lightMsg, _parent);
  if (light) this->ProcessDirectionalLightImpl(_lightMsg, light);
}

//////////////////////////////////////////////////
void SubSceneManager::ProcessDirectionalLightImpl(
    const gazebo::msgs::Light &_lightMsg, DirectionalLightPtr _light)
{
  // set direction if available
  if (_lightMsg.has_direction())
  {
    const gazebo::msgs::Vector3d &dirMsg = _lightMsg.direction();
    _light->SetDirection(SubSceneManager::Convert(dirMsg));
  }

  // process general light information
  this->ProcessLightImpl(_lightMsg, _light);
}

//////////////////////////////////////////////////
DirectionalLightPtr SubSceneManager::DirectionalLight(
    const gazebo::msgs::Light &_lightMsg, VisualPtr _parent)
{
  // find existing light with name
  std::string name = _lightMsg.name();
  LightPtr light = this->activeScene->LightByName(name);

  DirectionalLightPtr dirLight =
      std::dynamic_pointer_cast<gz::rendering::DirectionalLight>(light);

  // check if not found
  if (!dirLight)
  {
    dirLight = this->CreateDirectionalLight(_lightMsg);
    _parent->AddChild(dirLight);
  }

  return dirLight;
}

//////////////////////////////////////////////////
DirectionalLightPtr SubSceneManager::CreateDirectionalLight(
    const gazebo::msgs::Light &_lightMsg)
{
  std::string name = _lightMsg.name();
  return this->activeScene->CreateDirectionalLight(name);
}

//////////////////////////////////////////////////
void SubSceneManager::ProcessPointLight(const gazebo::msgs::Light &_lightMsg,
    VisualPtr _parent)
{
  PointLightPtr light = this->PointLight(_lightMsg, _parent);
  if (light) this->ProcessPointLightImpl(_lightMsg, light);
}

//////////////////////////////////////////////////
void SubSceneManager::ProcessPointLightImpl(
    const gazebo::msgs::Light &_lightMsg, PointLightPtr _light)
{
  // process general light information
  this->ProcessLightImpl(_lightMsg, _light);
}

//////////////////////////////////////////////////
PointLightPtr SubSceneManager::PointLight(
    const gazebo::msgs::Light &_lightMsg, VisualPtr _parent)
{
  // find existing light with name
  std::string name = _lightMsg.name();
  LightPtr light = this->activeScene->LightByName(name);
  PointLightPtr pointLight =
      std::dynamic_pointer_cast<gz::rendering::PointLight>(light);

  // check if not found
  if (!pointLight)
  {
    pointLight = this->CreatePointLight(_lightMsg);
    _parent->AddChild(pointLight);
  }

  return pointLight;
}

//////////////////////////////////////////////////
PointLightPtr SubSceneManager::CreatePointLight(
    const gazebo::msgs::Light &_lightMsg)
{
  std::string name = _lightMsg.name();
  return this->activeScene->CreatePointLight(name);
}

//////////////////////////////////////////////////
void SubSceneManager::ProcessSpotLight(const gazebo::msgs::Light &_lightMsg,
    VisualPtr _parent)
{
  SpotLightPtr light = this->SpotLight(_lightMsg, _parent);
  if (light) this->ProcessSpotLightImpl(_lightMsg, light);
}

//////////////////////////////////////////////////
void SubSceneManager::ProcessSpotLightImpl(
    const gazebo::msgs::Light &_lightMsg, SpotLightPtr _light)
{
  // set direction if available
  if (_lightMsg.has_direction())
  {
    const gazebo::msgs::Vector3d &dirMsg = _lightMsg.direction();
    _light->SetDirection(SubSceneManager::Convert(dirMsg));
  }

  // set inner-angle if available
  if (_lightMsg.has_spot_inner_angle())
  {
    double radians = _lightMsg.spot_inner_angle();
    _light->SetInnerAngle(math::Angle(radians));
  }

  // set outer-angle if available
  if (_lightMsg.has_spot_outer_angle())
  {
    double radians = _lightMsg.spot_outer_angle();
    _light->SetOuterAngle(math::Angle(radians));
  }

  // set falloff if available
  if (_lightMsg.has_spot_falloff())
  {
    double falloff = _lightMsg.spot_falloff();
    _light->SetFalloff(falloff);
  }

  // process general light information
  this->ProcessLightImpl(_lightMsg, _light);
}

//////////////////////////////////////////////////
SpotLightPtr SubSceneManager::SpotLight(const gazebo::msgs::Light &_lightMsg,
    VisualPtr _parent)
{
  // find existing light with name
  std::string name = _lightMsg.name();
  LightPtr light = this->activeScene->LightByName(name);
  SpotLightPtr spotLight =
      std::dynamic_pointer_cast<gz::rendering::SpotLight>(light);

  // check if not found
  if (!spotLight)
  {
    spotLight = this->CreateSpotLight(_lightMsg);
    _parent->AddChild(spotLight);
  }

  return spotLight;
}

//////////////////////////////////////////////////
SpotLightPtr SubSceneManager::CreateSpotLight(
    const gazebo::msgs::Light &_lightMsg)
{
  std::string name = _lightMsg.name();
  return this->activeScene->CreateSpotLight(name);
}

//////////////////////////////////////////////////
void SubSceneManager::ProcessLightImpl(const gazebo::msgs::Light &_lightMsg,
    LightPtr _light)
{
  // set pose if available
  if (_lightMsg.has_pose())
  {
    this->SetPose(_light, _lightMsg.pose());
  }

  // set diffuse if available
  if (_lightMsg.has_diffuse())
  {
    const gazebo::msgs::Color &colorMsg = _lightMsg.diffuse();
    _light->SetDiffuseColor(SubSceneManager::Convert(colorMsg));
  }

  // set specular if available
  if (_lightMsg.has_specular())
  {
    const gazebo::msgs::Color &colorMsg = _lightMsg.specular();
    _light->SetSpecularColor(SubSceneManager::Convert(colorMsg));
  }

  // set attenuation constant if available
  if (_lightMsg.has_attenuation_constant())
  {
    double attenConst = _lightMsg.attenuation_constant();
    _light->SetAttenuationConstant(attenConst);
  }

  // set attenuation linear if available
  if (_lightMsg.has_attenuation_linear())
  {
    double attenLinear = _lightMsg.attenuation_linear();
    _light->SetAttenuationLinear(attenLinear);
  }

  // set attenuation quadratic if available
  if (_lightMsg.has_attenuation_quadratic())
  {
    double attenQuad = _lightMsg.attenuation_quadratic();
    _light->SetAttenuationQuadratic(attenQuad);
  }

  // set attenuation range if available
  if (_lightMsg.has_range())
  {
    double attenRange = _lightMsg.range();
    _light->SetAttenuationRange(attenRange);
  }

  // set cast-shadows if available
  if (_lightMsg.has_cast_shadows())
  {
    bool castShadows = _lightMsg.cast_shadows();
    _light->SetCastShadows(castShadows);
  }
}

//////////////////////////////////////////////////
void SubSceneManager::ProcessSensor(const gazebo::msgs::Sensor &_sensorMsg)
{
  VisualPtr parent = this->Parent(_sensorMsg.parent());
  this->ProcessSensor(_sensorMsg, parent);
}

//////////////////////////////////////////////////
void SubSceneManager::ProcessSensor(const gazebo::msgs::Sensor &_sensorMsg,
    VisualPtr _parent)
{
  // TODO(anyone): process all types

  if (_sensorMsg.has_camera())
  {
    this->ProcessCamera(_sensorMsg, _parent);
  }
}

//////////////////////////////////////////////////
void SubSceneManager::ProcessCamera(const gazebo::msgs::Sensor &_sensorMsg,
    VisualPtr _parent)
{
  CameraPtr camera = this->Camera(_sensorMsg, _parent);

  // TODO(anyone): update camera
}

//////////////////////////////////////////////////
CameraPtr SubSceneManager::Camera(const gazebo::msgs::Sensor &_sensorMsg,
    VisualPtr _parent)
{
  // find existing camera with name
  std::string name = _sensorMsg.name();
  SensorPtr sensor = this->activeScene->SensorByName(name);
  CameraPtr camera = std::dynamic_pointer_cast<gz::rendering::Camera>(sensor);

  // check if not found
  if (!camera)
  {
    camera = this->CreateCamera(_sensorMsg);
    _parent->AddChild(camera);
  }

  return camera;
}

//////////////////////////////////////////////////
CameraPtr SubSceneManager::CreateCamera(const gazebo::msgs::Sensor &_sensorMsg)
{
  bool hasId = _sensorMsg.has_id();
  unsigned int id = _sensorMsg.id();
  std::string name = _sensorMsg.name();

  return (hasId) ?
      this->activeScene->CreateCamera(id, name) :
      this->activeScene->CreateCamera(name);
}

//////////////////////////////////////////////////
void SubSceneManager::ProcessModel(const gazebo::msgs::Model &_modelMsg)
{
  VisualPtr parent = this->activeScene->RootVisual();
  this->ProcessModel(_modelMsg, parent);
}

//////////////////////////////////////////////////
void SubSceneManager::ProcessModel(const gazebo::msgs::Model &_modelMsg,
    VisualPtr _parent)
{
  VisualPtr model = this->Model(_modelMsg, _parent);

  // set pose if available
  if (_modelMsg.has_pose())
  {
    this->SetPose(model, _modelMsg.pose());
  }

  // set scale if available
  if (_modelMsg.has_scale())
  {
    this->SetScale(model, _modelMsg.scale());
  }

  // process each sensor in joint
  for (int i = 0; i < _modelMsg.joint_size(); ++i)
  {
    const gazebo::msgs::Joint &joint = _modelMsg.joint(i);
    this->ProcessJoint(joint, model);
  }

  // process each sensor in link
  for (int i = 0; i < _modelMsg.link_size(); ++i)
  {
    const gazebo::msgs::Link &link = _modelMsg.link(i);
    this->ProcessLink(link, model);
  }

  // process each sensor in visual
  // always skip first empty visual
  for (int i = 1; i < _modelMsg.visual_size(); ++i)
  {
    const gazebo::msgs::Visual &visual = _modelMsg.visual(i);
    this->ProcessVisual(visual, model);
  }
}

//////////////////////////////////////////////////
VisualPtr SubSceneManager::Model(const gazebo::msgs::Model &_modelMsg,
    VisualPtr _parent)
{
  bool hasId = _modelMsg.has_id();
  unsigned int id = _modelMsg.id();
  std::string name = _modelMsg.name();
  return this->Visual(hasId, id, name, _parent);
}

//////////////////////////////////////////////////
void SubSceneManager::ProcessJoint(const gazebo::msgs::Joint &_jointMsg)
{
  VisualPtr parent = this->Parent(_jointMsg.parent());
  this->ProcessJoint(_jointMsg, parent);
}

//////////////////////////////////////////////////
void SubSceneManager::ProcessJoint(const gazebo::msgs::Joint &_jointMsg,
    VisualPtr _parent)
{
  VisualPtr joint = this->Joint(_jointMsg, _parent);

  // set pose if available
  if (_jointMsg.has_pose())
  {
    this->SetPose(joint, _jointMsg.pose());
  }

  // process each sensor in joint
  for (int i = 0; i < _jointMsg.sensor_size(); ++i)
  {
    const gazebo::msgs::Sensor &sensor = _jointMsg.sensor(i);
    this->ProcessSensor(sensor, joint);
  }
}

//////////////////////////////////////////////////
VisualPtr SubSceneManager::Joint(const gazebo::msgs::Joint &_jointMsg,
    VisualPtr _parent)
{
  bool hasId = _jointMsg.has_id();
  unsigned int id = _jointMsg.id();
  std::string name = _jointMsg.name();
  return this->Visual(hasId, id, name, _parent);
}

//////////////////////////////////////////////////
void SubSceneManager::ProcessVisual(const gazebo::msgs::Visual &_visualMsg)
{
  VisualPtr parent = this->Parent(_visualMsg.parent_name());
  this->ProcessVisual(_visualMsg, parent);
}

//////////////////////////////////////////////////
void SubSceneManager::ProcessVisual(const gazebo::msgs::Visual &_visualMsg,
    VisualPtr _parent)
{
  VisualPtr visual = this->Visual(_visualMsg, _parent);

  // TODO(anyone): handle cast shadows
  // TODO(anyone): handle transparency
  // TODO(anyone): handle scale & geom size

  // set pose if available
  if (_visualMsg.has_pose())
  {
    this->SetPose(visual, _visualMsg.pose());
  }

  // set scale if available
  if (_visualMsg.has_scale())
  {
    this->SetScale(visual, _visualMsg.scale());
  }

  // set geometry if available
  if (_visualMsg.has_geometry())
  {
    this->ProcessGeometry(_visualMsg.geometry(), visual);
  }

  // set material if available
  if (_visualMsg.has_material())
  {
    const gazebo::msgs::Material matMsg = _visualMsg.material();
    MaterialPtr material = this->CreateMaterial(matMsg);
    visual->SetMaterial(material);
  }
}

//////////////////////////////////////////////////
VisualPtr SubSceneManager::Visual(const gazebo::msgs::Visual &_visualMsg,
    VisualPtr _parent)
{
  bool hasId = _visualMsg.has_id();
  unsigned int id = _visualMsg.id();
  std::string name = _visualMsg.name();
  return this->Visual(hasId, id, name, _parent);
}

//////////////////////////////////////////////////
void SubSceneManager::ProcessLink(const gazebo::msgs::Link &_linkMsg,
    VisualPtr _parent)
{
  VisualPtr link = this->Link(_linkMsg, _parent);

  // set pose if available
  if (_linkMsg.has_pose())
  {
    this->SetPose(link, _linkMsg.pose());
  }

  // process each visual in link
  // always skip first empty visual
  for (int i = 1; i < _linkMsg.visual_size(); ++i)
  {
    const gazebo::msgs::Visual &visual = _linkMsg.visual(i);
    this->ProcessVisual(visual, link);
  }

  // process each sensor in link
  for (int i = 0; i < _linkMsg.sensor_size(); ++i)
  {
    const gazebo::msgs::Sensor &sensor = _linkMsg.sensor(i);
    this->ProcessSensor(sensor, link);
  }
}

//////////////////////////////////////////////////
VisualPtr SubSceneManager::Link(const gazebo::msgs::Link &_linkMsg,
    VisualPtr _parent)
{
  bool hasId = _linkMsg.has_id();
  unsigned int id = _linkMsg.id();
  std::string name = _linkMsg.name();
  return this->Visual(hasId, id, name, _parent);
}

//////////////////////////////////////////////////
VisualPtr SubSceneManager::Visual(bool _hasId, unsigned int _id,
    const std::string &_name, VisualPtr _parent)
{
  // find existing visual with name
  VisualPtr visual = this->activeScene->VisualByName(_name);

  // check if not found
  if (!visual)
  {
    visual = this->CreateVisual(_hasId, _id, _name);
    _parent->AddChild(visual);
  }

  return visual;
}

//////////////////////////////////////////////////
VisualPtr SubSceneManager::CreateVisual(bool _hasId, unsigned int _id,
    const std::string &_name)
{
  return (_hasId) ?
      this->activeScene->CreateVisual(_id, _name) :
      this->activeScene->CreateVisual(_name);
}

//////////////////////////////////////////////////
void SubSceneManager::ProcessGeometry(
    const gazebo::msgs::Geometry &_geometryMsg, VisualPtr _parent)
{
  GeomType geomType = _geometryMsg.type();
  GeomFunc geomFunc = this->geomFunctions[geomType];
  _parent->RemoveGeometries();

  // check if invalid type
  if (!geomFunc)
  {
    ignerr << "Unsupported geometry type: " << geomType << std::endl;
    ignwarn << "Using empty geometry instead" << std::endl;
    geomFunc = this->geomFunctions[gazebo::msgs::Geometry::EMPTY];
  }

  (this->*geomFunc)(_geometryMsg, _parent);
}

//////////////////////////////////////////////////
void SubSceneManager::ProcessBox(const gazebo::msgs::Geometry &_geometryMsg,
    VisualPtr _parent)
{
  GeometryPtr box = this->activeScene->CreateBox();
  const gazebo::msgs::BoxGeom &boxMsg = _geometryMsg.box();
  const gazebo::msgs::Vector3d sizeMsg = boxMsg.size();
  _parent->SetLocalScale(SubSceneManager::Convert(sizeMsg));
  _parent->AddGeometry(box);
}

//////////////////////////////////////////////////
void SubSceneManager::ProcessCone(
    const gazebo::msgs::Geometry &/*_geometryMsg*/, VisualPtr _parent)
{
  // TODO(anyone): needs protobuf msg
  GeometryPtr cone = this->activeScene->CreateCone();
  // const gazebo::msgs::ConeGeom &coneMsg = _geometryMsg.cone();
  // double x = coneMsg.radius();
  // double y = coneMsg.radius();
  // double z = coneMsg.length();
  // _parent->SetLocalScale(x, y, z);
  _parent->AddGeometry(cone);
}

//////////////////////////////////////////////////
void SubSceneManager::ProcessCapsule(
    const gazebo::msgs::Geometry & _geometryMsg, VisualPtr _parent)
{
  // \todo(anyone) needs gazebo capsule msg
  CapsulePtr capsule = this->activeScene->CreateCapsule();
  // Const gazebo::msgs::CapsuleGeom &capsuleMsg = _geometryMsg.capsule();
  // Double x = 2 * capsuleMsg.radius();
  // Double y = 2 * capsuleMsg.radius();
  // Double z = capsuleMsg.length();
  // _parent->SetLocalScale(x, y, z);
  _parent->AddGeometry(std::dynamic_pointer_cast<Geometry>(capsule));
}

//////////////////////////////////////////////////
//! [process cylinder]
void SubSceneManager::ProcessCylinder(
    const gazebo::msgs::Geometry &_geometryMsg, VisualPtr _parent)
{
  GeometryPtr cylinder = this->activeScene->CreateCylinder();
  const gazebo::msgs::CylinderGeom &cylinderMsg = _geometryMsg.cylinder();
  double x = 2 * cylinderMsg.radius();
  double y = 2 * cylinderMsg.radius();
  double z = cylinderMsg.length();
  _parent->SetLocalScale(x, y, z);
  _parent->AddGeometry(cylinder);
}
//! [process cylinder]

//////////////////////////////////////////////////
void SubSceneManager::ProcessEmpty(const gazebo::msgs::Geometry&, VisualPtr)
{
  // do nothing
}

//////////////////////////////////////////////////
void SubSceneManager::ProcessMesh(const gazebo::msgs::Geometry &_geometryMsg,
    VisualPtr _parent)
{
  const gazebo::msgs::MeshGeom &meshMsg = _geometryMsg.mesh();

  // if the model contains model:// try to find the meshes in ~/.gazebo
  std::list<std::string> modelPaths;
  char *homePath = getenv("HOME");
  std::string home;
  if (!homePath)
    home = "/tmp/gazebo";
  else
    home = homePath;

  modelPaths.push_back(home + "/.gazebo/models");

  std::string meshName = meshMsg.filename();
  int index = meshName.find("://");
  std::string prefix = meshName.substr(0, index);
  std::string suffix = meshName.substr(index + 3, meshName.size() - index - 3);

  if (prefix == "model")
  {
     fs::path path;
     for (std::list<std::string>::iterator iter = modelPaths.begin();
          iter != modelPaths.end(); ++iter)
     {
       path = fs::path(*iter) / suffix;
       if (fs::exists(path))
       {
         meshName = path.string();
         break;
       }
     }
  }

  // initialize mesh parameters
  MeshDescriptor descriptor;
  descriptor.meshName = meshName;

  // assign sub-mesh if available
  if (meshMsg.has_submesh())
  {
    descriptor.subMeshName = meshMsg.submesh();
  }

  // assign sub-mesh if available
  if (meshMsg.has_center_submesh())
  {
    descriptor.centerSubMesh = meshMsg.center_submesh();
  }

  // actually create mesh geometry
  common::MeshManager *meshManager = common::MeshManager::Instance();
  descriptor.mesh = meshManager->Load(descriptor.meshName);
  MeshPtr mesh = this->activeScene->CreateMesh(descriptor);

  // set scale if available
  if (meshMsg.has_scale())
  {
    const gazebo::msgs::Vector3d scaleMsg = meshMsg.scale();
    _parent->SetLocalScale(SubSceneManager::Convert(scaleMsg));
  }

  // attach geometry to parent
  _parent->AddGeometry(mesh);
}

//////////////////////////////////////////////////
void SubSceneManager::ProcessPlane(const gazebo::msgs::Geometry &_geometryMsg,
    VisualPtr _parent)
{
  // TODO(anyone): handle plane normal
  GeometryPtr plane = this->activeScene->CreatePlane();
  const gazebo::msgs::PlaneGeom &planeMsg = _geometryMsg.plane();
  const gazebo::msgs::Vector2d planeSize = planeMsg.size();
  _parent->SetLocalScale(planeSize.x(), planeSize.y(), 1);
  _parent->AddGeometry(plane);
}

//////////////////////////////////////////////////
void SubSceneManager::ProcessSphere(const gazebo::msgs::Geometry &_geometryMsg,
    VisualPtr _parent)
{
  GeometryPtr sphere = this->activeScene->CreateSphere();
  const gazebo::msgs::SphereGeom &sphereMsg = _geometryMsg.sphere();
  _parent->SetLocalScale(2 * sphereMsg.radius());
  _parent->AddGeometry(sphere);
}

//////////////////////////////////////////////////
MaterialPtr SubSceneManager::CreateMaterial(
    const gazebo::msgs::Material &_materialMsg)
{
  MaterialPtr material = this->activeScene->CreateMaterial();

  // TODO(anyone): remove after testing
  material->SetShininess(50);
  material->SetReflectivity(0.25);

  // set ambient if available
  if (_materialMsg.has_ambient())
  {
    gazebo::msgs::Color msg = _materialMsg.ambient();
    math::Color ambient(msg.r(), msg.g(), msg.b(), msg.a());
    material->SetAmbient(ambient);
  }

  // set diffuse if available
  if (_materialMsg.has_diffuse())
  {
    gazebo::msgs::Color msg = _materialMsg.diffuse();
    math::Color diffuse(msg.r(), msg.g(), msg.b(), msg.a());
    material->SetDiffuse(diffuse);
  }

  // set specular if available
  if (_materialMsg.has_specular())
  {
    gazebo::msgs::Color msg = _materialMsg.specular();
    math::Color specular(msg.r(), msg.g(), msg.b(), msg.a());
    material->SetSpecular(specular);
  }

  // set emissive if available
  if (_materialMsg.has_emissive())
  {
    gazebo::msgs::Color msg = _materialMsg.emissive();
    math::Color emissive(msg.r(), msg.g(), msg.b(), msg.a());
    material->SetEmissive(emissive);
  }

  // set lighting if available
  if (_materialMsg.has_lighting())
  {
    bool lighting = _materialMsg.lighting();
    material->SetLightingEnabled(lighting);
  }

  // set normal-map if available
  if (_materialMsg.has_normal_map())
  {
    const std::string &normal_map = _materialMsg.normal_map();
    material->SetNormalMap(normal_map);
  }

  // set shader-type if available
  if (_materialMsg.has_shader_type())
  {
    gazebo::msgs::Material::ShaderType shader_type =
        _materialMsg.shader_type();
    ShaderType type = SubSceneManager::Convert(shader_type);
    material->SetShaderType(type);
  }

  // TODO(anyone): handle scripts

  return material;
}

//////////////////////////////////////////////////
void SubSceneManager::ProcessPose(const gazebo::msgs::Pose &_poseMsg)
{
  std::string name = _poseMsg.name();
  NodePtr node = this->activeScene->NodeByName(name);
  if (node) this->SetPose(node, _poseMsg);
}

//////////////////////////////////////////////////
void SubSceneManager::SetPose(NodePtr _node,
    const gazebo::msgs::Pose &_poseMsg)
{
  math::Pose3d pose = SubSceneManager::Convert(_poseMsg);
  _node->SetLocalPose(pose);
}

//////////////////////////////////////////////////
void SubSceneManager::SetScale(VisualPtr _visual,
    const gazebo::msgs::Vector3d &_scaleMsg)
{
  math::Vector3d scale = SubSceneManager::Convert(_scaleMsg);
  _visual->SetLocalScale(scale);
}

//////////////////////////////////////////////////
VisualPtr SubSceneManager::Parent(const std::string &_name)
{
  // assign default parent node
  VisualPtr parent = this->activeScene->RootVisual();

  // check if name given
  if (!_name.empty())
  {
    // get node with name
    parent = this->activeScene->VisualByName(_name);

    // node not found
    if (!parent)
    {
      if (_name != "default")
      {
        ignerr  << "invalid parent name: " << _name << std::endl;
        ignwarn << "using scene root node" << std::endl;
      }

      parent = this->activeScene->RootVisual();
    }
  }

  return parent;
}

//////////////////////////////////////////////////
void SubSceneManager::ProcessRemoval(const std::string &_name)
{
  this->activeScene->DestroyNodeByName(_name);
}

//////////////////////////////////////////////////
math::Color SubSceneManager::Convert(const gazebo::msgs::Color &_colorMsg)
{
  math::Color color;
  color.R() = _colorMsg.r();
  color.G() = _colorMsg.g();
  color.B() = _colorMsg.b();
  color.A() = _colorMsg.a();
  return color;
}

//////////////////////////////////////////////////
math::Pose3d SubSceneManager::Convert(const gazebo::msgs::Pose &_poseMsg)
{
  math::Pose3d pose;

  const gazebo::msgs::Vector3d &posMsg = _poseMsg.position();
  pose.Pos() = SubSceneManager::Convert(posMsg);

  const gazebo::msgs::Quaternion &rotMsg = _poseMsg.orientation();
  pose.Rot() = SubSceneManager::Convert(rotMsg);

  return pose;
}

//////////////////////////////////////////////////
math::Vector3d SubSceneManager::Convert(const gazebo::msgs::Vector3d &_vecMsg)
{
  math::Vector3d vec;
  vec.X(_vecMsg.x());
  vec.Y(_vecMsg.y());
  vec.Z(_vecMsg.z());
  return vec;
}

//////////////////////////////////////////////////
math::Quaterniond SubSceneManager::Convert(
    const gazebo::msgs::Quaternion &_quatMsg)
{
  math::Quaterniond quat;
  quat.W(_quatMsg.w());
  quat.X(_quatMsg.x());
  quat.Y(_quatMsg.y());
  quat.Z(_quatMsg.z());
  return quat;
}

//////////////////////////////////////////////////
ShaderType SubSceneManager::Convert(gazebo::msgs::Material::ShaderType _type)
{
  switch (_type)
  {
    case gazebo::msgs::Material::VERTEX:
      return ST_VERTEX;

    case gazebo::msgs::Material::PIXEL:
      return ST_PIXEL;

    case gazebo::msgs::Material::NORMAL_MAP_OBJECT_SPACE:
      return ST_NORM_OBJ;

    case gazebo::msgs::Material::NORMAL_MAP_TANGENT_SPACE:
      return ST_NORM_TAN;

    default:
      return ST_UNKNOWN;
  }
}

//////////////////////////////////////////////////
void SubSceneManager::CreateGeometryFunctionMap()
{
  this->geomFunctions[gazebo::msgs::Geometry::BOX] =
      &SubSceneManager::ProcessBox;

  // todo(anyone): enable when cone protobuf msg is created
  // this->geomFunctions[gazebo::msgs::Geometry::CONE] =
  //     &SubSceneManager::ProcessCone;

  // todo(anyone): enable when capsule protobuf msg is created
  // this->geomFunctions[gazebo::msgs::Geometry::CAPSULE] =
  //     &SubSceneManager::ProcessSphere;

  this->geomFunctions[gazebo::msgs::Geometry::CYLINDER] =
      &SubSceneManager::ProcessCylinder;

  this->geomFunctions[gazebo::msgs::Geometry::EMPTY] =
      &SubSceneManager::ProcessEmpty;

  this->geomFunctions[gazebo::msgs::Geometry::MESH] =
      &SubSceneManager::ProcessMesh;

  this->geomFunctions[gazebo::msgs::Geometry::PLANE] =
      &SubSceneManager::ProcessPlane;

  this->geomFunctions[gazebo::msgs::Geometry::SPHERE] =
      &SubSceneManager::ProcessSphere;
}

//////////////////////////////////////////////////
CurrentSceneManager::CurrentSceneManager()
{
}

//////////////////////////////////////////////////
CurrentSceneManager::~CurrentSceneManager()
{
}

//////////////////////////////////////////////////
//! [on update pose]
void CurrentSceneManager::OnPoseUpdate(::ConstPosesStampedPtr &_posesMsg)
{
  // record pose timestamp
  this->timePosesReceived = std::chrono::seconds(_posesMsg->time().sec()) +
      std::chrono::nanoseconds(_posesMsg->time().nsec());

  // process each pose in message
  for (int i = 0; i < _posesMsg->pose_size(); ++i)
  {
    // replace into pose map
    gazebo::msgs::Pose pose = _posesMsg->pose(i);
    std::string name = pose.name();
    this->poseMsgs[name] = pose;
  }
}
//! [on update pose]

//////////////////////////////////////////////////
void CurrentSceneManager::ClearMessages()
{
  SubSceneManager::ClearMessages();
  this->poseMsgs.clear();
}

//////////////////////////////////////////////////
void CurrentSceneManager::ProcessLights()
{
  // process each light in list
  for (auto &lightMsg : this->lightMsgs)
  {
    this->ProcessLight(lightMsg);
  }
}

//////////////////////////////////////////////////
void CurrentSceneManager::ProcessModels()
{
  // process each model in list
  for (auto &modelMsg : this->modelMsgs)
  {
    this->ProcessModel(modelMsg);
  }
}

//////////////////////////////////////////////////
void CurrentSceneManager::ProcessJoints()
{
  // process each joint in list
  for (auto &jointMsg : this->jointMsgs)
  {
    this->ProcessJoint(jointMsg);
  }
}

//////////////////////////////////////////////////
void CurrentSceneManager::ProcessVisuals()
{
  // process each visual in list
  for (auto &visualMsg : this->visualMsgs)
  {
    this->ProcessVisual(visualMsg);
  }
}

//////////////////////////////////////////////////
void CurrentSceneManager::ProcessSensors()
{
  // process each sensor in list
  for (auto &sensorMsg : this->sensorMsgs)
  {
    this->ProcessSensor(sensorMsg);
  }
}

//////////////////////////////////////////////////
void CurrentSceneManager::ProcessPoses()
{
  // process each pose in list
  for (auto poseMsgPair : this->poseMsgs)
  {
    this->ProcessPose(poseMsgPair.second);
  }
}

//////////////////////////////////////////////////
void CurrentSceneManager::ProcessRemovals()
{
  // process each removal in list
  for (auto &removal : this->approvedRemovals)
  {
    this->ProcessRemoval(removal);
  }
}

//////////////////////////////////////////////////
NewSceneManager::NewSceneManager() :
  sceneReceived(false)
{
}

//////////////////////////////////////////////////
NewSceneManager::~NewSceneManager()
{
}

//////////////////////////////////////////////////
void NewSceneManager::SetSceneData(const std::string &_data)
{
  this->sceneMsg.ParseFromString(_data);
  this->sceneReceived = true;
}

//////////////////////////////////////////////////
void NewSceneManager::OnPoseUpdate(::ConstPosesStampedPtr &_posesMsg)
{
  this->posesMsgs.push_back(*_posesMsg);
}

//////////////////////////////////////////////////
void NewSceneManager::ProcessMessages()
{
  this->ProcessScene();
  SubSceneManager::ProcessMessages();
}

//////////////////////////////////////////////////
void NewSceneManager::ClearMessages()
{
  this->posesMsgs.clear();
  this->sceneReceived = false;
  SubSceneManager::ClearMessages();
}

//////////////////////////////////////////////////
void NewSceneManager::ProcessScene()
{
  // TODO(anyone): process environment info

  // delete all previous nodes
  // this->activeScene->Clear();

  // process ambient if available
  if (this->sceneMsg.has_ambient())
  {
    gazebo::msgs::Color colorMsg = this->sceneMsg.ambient();
    math::Color color(colorMsg.r(), colorMsg.g(), colorMsg.b());
    this->activeScene->SetAmbientLight(color);
  }

  // process background if available
  if (this->sceneMsg.has_background())
  {
    gazebo::msgs::Color colorMsg = this->sceneMsg.background();
    math::Color color(colorMsg.r(), colorMsg.g(), colorMsg.b());
    this->activeScene->SetBackgroundColor(color);
  }

  // process each scene light
  for (int i = 0; i < this->sceneMsg.light_size(); ++i)
  {
    gazebo::msgs::Light lightMsg = this->sceneMsg.light(i);
    this->ProcessLight(lightMsg, this->activeScene->RootVisual());
  }

  // process each scene model
  for (int i = 0; i < this->sceneMsg.model_size(); ++i)
  {
    gazebo::msgs::Model modelMsg = this->sceneMsg.model(i);
    this->ProcessModel(modelMsg, this->activeScene->RootVisual());
  }
}

//////////////////////////////////////////////////
void NewSceneManager::ProcessLights()
{
  // process each light in list
  for (auto &lightMsg : this->lightMsgs)
  {
    // TODO(anyone): check if message sent after scene response
    this->ProcessLight(lightMsg);
  }
}

//////////////////////////////////////////////////
void NewSceneManager::ProcessModels()
{
  // process each model in list
  for (auto &modelMsg : this->modelMsgs)
  {
    // TODO(anyone): check if message sent after scene response
    this->ProcessModel(modelMsg);
  }
}

//////////////////////////////////////////////////
void NewSceneManager::ProcessJoints()
{
  // process each joint in list
  for (auto &jointMsg : this->jointMsgs)
  {
    // TODO(anyone): check if message sent after scene response
    this->ProcessJoint(jointMsg);
  }
}

//////////////////////////////////////////////////
void NewSceneManager::ProcessVisuals()
{
  // process each visual in list
  for (auto &visualMsg : this->visualMsgs)
  {
    // TODO(anyone): check if message sent after scene response
    this->ProcessVisual(visualMsg);
  }
}

//////////////////////////////////////////////////
void NewSceneManager::ProcessSensors()
{
  // process each sensor in list
  for (auto &sensorMsg : this->sensorMsgs)
  {
    // TODO(anyone): check if message sent after scene response
    this->ProcessSensor(sensorMsg);
  }
}

//////////////////////////////////////////////////
void NewSceneManager::ProcessPoses()
{
  // process each poses in list
  for (auto &posesMsg : this->posesMsgs)
  {
    // TODO(anyone): check if message sent after scene response
    this->ProcessPoses(posesMsg);
  }
}

//////////////////////////////////////////////////
void NewSceneManager::ProcessPoses(const gazebo::msgs::PosesStamped &_posesMsg)
{
  // record pose timestamp
  this->timePosesReceived = std::chrono::seconds(_posesMsg.time().sec()) +
      std::chrono::nanoseconds(_posesMsg.time().nsec());

  // process each pose in list
  for (int i = 0; i < _posesMsg.pose_size(); ++i)
  {
    gazebo::msgs::Pose poseMsg = _posesMsg.pose(i);
    this->ProcessPose(poseMsg);
  }
}

//////////////////////////////////////////////////
void NewSceneManager::ProcessRemovals()
{
  // process each removal in list
  for (auto &removal : this->approvedRemovals)
  {
    // TODO(anyone): check if message sent after scene response
    this->ProcessRemoval(removal);
  }
}
