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

#include <gz/common/Console.hh>

#include "gz/rendering/RenderPassSystem.hh"
#include "gz/rendering/base/BaseRenderEngine.hh"

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
BaseRenderEngine::BaseRenderEngine()
{
  this->renderPassSystem.reset(new rendering::RenderPassSystem());
}

//////////////////////////////////////////////////
BaseRenderEngine::~BaseRenderEngine()
{
}

//////////////////////////////////////////////////
bool BaseRenderEngine::Load(const std::map<std::string, std::string> &_params)
{
  if (this->loaded)
  {
    ignwarn << "Render-engine has already been loaded" << std::endl;
    return true;
  }

  this->loaded = this->LoadImpl(_params);
  return this->loaded;
}

//////////////////////////////////////////////////
bool BaseRenderEngine::Init()
{
  if (!this->loaded)
  {
    ignerr << "Render-engine must be loaded first" << std::endl;
    return false;
  }

  if (this->initialized)
  {
    ignwarn << "Render-engine has already been initialized" << std::endl;
    return true;
  }

  this->initialized = this->InitImpl();
  return this->initialized;
}

//////////////////////////////////////////////////
bool BaseRenderEngine::Fini()
{
  this->Destroy();
  return true;
}

//////////////////////////////////////////////////
bool BaseRenderEngine::IsLoaded() const
{
  return this->loaded;
}

//////////////////////////////////////////////////
bool BaseRenderEngine::IsInitialized() const
{
  return this->initialized;
}

//////////////////////////////////////////////////
bool BaseRenderEngine::IsEnabled() const
{
  return this->initialized;
}

//////////////////////////////////////////////////
unsigned int BaseRenderEngine::SceneCount() const
{
  auto scenes = this->Scenes();
  if (scenes)
    return scenes->Size();
  return 0u;
}

//////////////////////////////////////////////////
bool BaseRenderEngine::HasScene(ConstScenePtr _scene) const
{
  auto scenes = this->Scenes();
  if (scenes)
    return scenes->Contains(_scene);
  return false;
}

//////////////////////////////////////////////////
bool BaseRenderEngine::HasSceneId(unsigned int _id) const
{
  auto scenes = this->Scenes();
  if (scenes)
    return scenes->ContainsId(_id);
  return false;
}

//////////////////////////////////////////////////
bool BaseRenderEngine::HasSceneName(const std::string &_name) const
{
  auto scenes = this->Scenes();
  if (scenes)
    return scenes->ContainsName(_name);
  return false;
}

//////////////////////////////////////////////////
ScenePtr BaseRenderEngine::SceneById(unsigned int _id) const
{
  auto scenes = this->Scenes();
  if (scenes)
    return scenes->GetById(_id);
  return ScenePtr();
}

//////////////////////////////////////////////////
ScenePtr BaseRenderEngine::SceneByName(const std::string &_name) const
{
  auto scenes = this->Scenes();
  if (scenes)
    return scenes->GetByName(_name);
  return ScenePtr();
}

//////////////////////////////////////////////////
ScenePtr BaseRenderEngine::SceneByIndex(unsigned int _index) const
{
  auto scenes = this->Scenes();
  if (scenes)
    return scenes->GetByIndex(_index);
  return ScenePtr();
}

//////////////////////////////////////////////////
void BaseRenderEngine::DestroyScene(ScenePtr _scene)
{
  auto scenes = this->Scenes();
  if (!scenes)
    return;
  scenes->Destroy(_scene);
}

//////////////////////////////////////////////////
void BaseRenderEngine::DestroySceneById(unsigned int _id)
{
  auto scenes = this->Scenes();
  if (!scenes)
    return;
  scenes->DestroyById(_id);
}

//////////////////////////////////////////////////
void BaseRenderEngine::DestroySceneByName(const std::string &_name)
{
  auto scenes = this->Scenes();
  if (!scenes)
    return;
  scenes->DestroyByName(_name);
}

//////////////////////////////////////////////////
void BaseRenderEngine::DestroySceneByIndex(unsigned int _index)
{
  auto scenes = this->Scenes();
  if (!scenes)
    return;
  scenes->DestroyByIndex(_index);
}

//////////////////////////////////////////////////
void BaseRenderEngine::DestroyScenes()
{
  auto scenes = this->Scenes();
  if (!scenes)
    return;
  scenes->DestroyAll();
}

//////////////////////////////////////////////////
ScenePtr BaseRenderEngine::CreateScene(const std::string &_name)
{
  unsigned int sceneId = this->NextSceneId();
  return this->CreateScene(sceneId, _name);
}

//////////////////////////////////////////////////
ScenePtr BaseRenderEngine::CreateScene(unsigned int _id,
    const std::string &_name)
{
  if (!this->IsInitialized())
  {
    ignerr << "Render-engine has not been initialized" << std::endl;
    return nullptr;
  }

  if (this->HasSceneId(_id))
  {
    ignerr << "Scene already exists with id: " << _id << std::endl;
    return nullptr;
  }

  if (this->HasSceneName(_name))
  {
    ignerr << "Scene already exists with name: " << _name << std::endl;
    return nullptr;
  }

  ScenePtr scene = this->CreateSceneImpl(_id, _name);
  this->PrepareScene(scene);
  return scene;
}

//////////////////////////////////////////////////
void BaseRenderEngine::Destroy()
{
  this->DestroyScenes();
  this->loaded = false;
  this->initialized = false;
}

//////////////////////////////////////////////////
void BaseRenderEngine::AddResourcePath(const std::string &_path)
{
  this->resourcePaths.push_back(_path);
}

//////////////////////////////////////////////////
void BaseRenderEngine::SetHeadless(bool _headless)
{
  this->isHeadless = _headless;
}

//////////////////////////////////////////////////
bool BaseRenderEngine::Headless() const
{
  return this->isHeadless;
}

//////////////////////////////////////////////////
void BaseRenderEngine::PrepareScene(ScenePtr _scene)
{
  if (_scene)
  {
    _scene->Load();
    _scene->Init();
  }
}

//////////////////////////////////////////////////
unsigned int BaseRenderEngine::NextSceneId()
{
  return this->nextSceneId--;
}

//////////////////////////////////////////////////
RenderPassSystemPtr BaseRenderEngine::RenderPassSystem() const
{
  if (!this->renderPassSystem)
  {
    ignerr << "Render pass not supported by the requested render engine"
        << std::endl;
    return RenderPassSystemPtr();
  }
  return this->renderPassSystem;
}
