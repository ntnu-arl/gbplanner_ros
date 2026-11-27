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

#include "gz/common/Console.hh"

#include "gz/rendering/RenderEngine.hh"
#include "gz/rendering/RenderingIface.hh"
#include "gz/rendering/RenderEngineManager.hh"
#include "gz/rendering/Scene.hh"

namespace ignition
{
namespace rendering
{
inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
//
/*//////////////////////////////////////////////////
bool load()
{
  SceneManager::Instance()->Load();
  return true;
}

//////////////////////////////////////////////////
bool init()
{
  SceneManager::Instance()->Init();
  return true;
}

//////////////////////////////////////////////////
bool fini()
{
  SceneManager::Instance()->Fini();
  return true;
}*/

//////////////////////////////////////////////////
unsigned int engineCount()
{
  return RenderEngineManager::Instance()->EngineCount();
}

//////////////////////////////////////////////////
bool hasEngine(const std::string &_name)
{
  return RenderEngineManager::Instance()->HasEngine(_name);
}

//////////////////////////////////////////////////
bool isEngineLoaded(const std::string &_name)
{
  return RenderEngineManager::Instance()->IsEngineLoaded(_name);
}

//////////////////////////////////////////////////
std::vector<std::string> loadedEngines()
{
  return RenderEngineManager::Instance()->LoadedEngines();
}

//////////////////////////////////////////////////
RenderEngine *engine(const std::string &_name,
    const std::map<std::string, std::string> &_params,
    const std::string &_path)
{
  return RenderEngineManager::Instance()->Engine(_name, _params, _path);
}

//////////////////////////////////////////////////
RenderEngine *engine(const unsigned int _index,
    const std::map<std::string, std::string> &_params,
    const std::string &_path)
{
  return RenderEngineManager::Instance()->EngineAt(_index, _params, _path);
}

//////////////////////////////////////////////////
bool unloadEngine(const std::string &_name)
{
  return RenderEngineManager::Instance()->UnloadEngine(_name);
}

//////////////////////////////////////////////////
void registerEngine(const std::string &_name,
    RenderEngine *_engine)
{
  RenderEngineManager::Instance()->RegisterEngine(_name, _engine);
}

//////////////////////////////////////////////////
void unregisterEngine(const std::string &_name)
{
  RenderEngineManager::Instance()->UnregisterEngine(_name);
}

//////////////////////////////////////////////////
void unregisterEngine(RenderEngine *_engine)
{
  RenderEngineManager::Instance()->UnregisterEngine(_engine);
}

//////////////////////////////////////////////////
void unregisterEngine(const unsigned int _index)
{
  RenderEngineManager::Instance()->UnregisterEngineAt(_index);
}

//////////////////////////////////////////////////
void setPluginPaths(const std::list<std::string> &_paths)
{
  RenderEngineManager::Instance()->SetPluginPaths(_paths);
}

//////////////////////////////////////////////////
ScenePtr sceneFromFirstRenderEngine()
{
  auto loadedEngNames = gz::rendering::loadedEngines();
  if (loadedEngNames.empty())
  {
    igndbg << "No rendering engine is loaded yet" << std::endl;
    return nullptr;
  }

  auto engineName = loadedEngNames[0];
  if (loadedEngNames.size() > 1)
  {
    ignwarn << "More than one engine is available. "
      << "Using engine [" << engineName << "]" << std::endl;
  }

  auto engine = gz::rendering::engine(engineName);
  if (!engine)
  {
    ignerr << "Internal error: failed to load engine [" << engineName
      << "]." << std::endl;
    return nullptr;
  }

  if (engine->SceneCount() == 0)
  {
    igndbg << "No scene has been created yet" << std::endl;
    return nullptr;
  }

  auto scene = engine->SceneByIndex(0);
  if (nullptr == scene)
  {
    ignerr << "Internal error: scene is null." << std::endl;
    return nullptr;
  }

  if (engine->SceneCount() > 1)
  {
    ignwarn << "More than one scene is available. "
      << "Using scene [" << scene->Name() << "]" << std::endl;
  }

  if (!scene->IsInitialized() || nullptr == scene->RootVisual())
  {
    igndbg << "Scene is not initialized yet" << std::endl;
    return nullptr;
  }

  return scene;
}
}
}
}
