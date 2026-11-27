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

#include <map>
#include <mutex>

#include <gz/common/Console.hh>
#include <gz/common/SystemPaths.hh>

#include <gz/plugin/Loader.hh>

#include "gz/rendering/config.hh"
#include "gz/rendering/RenderEngine.hh"
#include "gz/rendering/RenderEngineManager.hh"
#include "gz/rendering/RenderEnginePlugin.hh"

/// \brief Holds information about an engine
struct EngineInfo
{
  /// \brief The name of the engine's shared library, default engines can also
  /// be specified with their regular name (ogre, optix, etc.).
  std::string name;

  /// \brief The pointer to the render engine.
  gz::rendering::RenderEngine *engine;
};

/// \brief Private implementation of the RenderEngineManager class.
class gz::rendering::RenderEngineManagerPrivate
{
  /// \brief EngineMap that maps engine name to an engine pointer.
  typedef std::map<std::string, RenderEngine *> EngineMap;

  /// \brief EngineMap iterator.
  typedef EngineMap::iterator EngineIter;

  /// \brief Get a pointer to the render engine from an EngineMap iterator.
  /// \param[in] _iter EngineMap iterator
  /// \param[in] _path Another search path for rendering engine plugin.
  public: RenderEngine *Engine(EngineInfo _info,
      const std::map<std::string, std::string> &_params,
      const std::string &_path);

  /// \brief Unload the given render engine from an EngineMap iterator.
  /// The engine will remain registered and can be loaded again later.
  /// \param[in] _iter EngineMap iterator
  /// \return True if the engine is unloaded
  public: bool UnloadEngine(EngineIter _iter);

  /// \brief Register default engines supplied by ign-rendering
  public: void RegisterDefaultEngines();

  /// \brief Unregister an engine using an EngineMap iterator.
  /// Once an engine is unregistered, it can no longer be loaded. Use
  /// RenderEngineManagerPrivate::UnloadEngine to unload the engine without
  /// unregistering it if you wish to load the engine again
  /// \param[in] _iter EngineMap iterator
  public: void UnregisterEngine(EngineIter _iter);

  /// \brief Load a render engine plugin.
  /// \param[in] _filename Filename of plugin shared library
  /// \param[in] _path Another search path for rendering engine plugin.
  /// \return True if the plugin is loaded successfully
  public: bool LoadEnginePlugin(const std::string &_filename,
              const std::string &_path);

  /// \brief Unload a render engine plugin.
  /// \param[in] _engineName Name of engine associated with this plugin
  /// \return True if the plugin is unloaded successfully
  public: bool UnloadEnginePlugin(const std::string &_engineName);

  /// \brief Engines that have been registered
  public: EngineMap engines;

  /// \brief A map of default engine library names to their plugin names.
  public: std::map<std::string, std::string> defaultEngines;

  /// \brief A map of loaded engine plugins to its plugin name.
  public: std::map<std::string, std::string> enginePlugins;

  /// \brief Plugin loader for managing render engine plugin libraries.
  public: gz::plugin::Loader pluginLoader;

  /// \brief Environment variable which holds paths to look for plugins
  public: std::string pluginPathEnv = "IGN_RENDERING_PLUGIN_PATH";

  /// \brief List which holds paths to look for engine plugins.
  public: std::list<std::string> pluginPaths;

  /// \brief Mutex to protect the engines map.
  public: std::recursive_mutex enginesMutex;
};

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
// RenderEngineManager
//////////////////////////////////////////////////
RenderEngineManager::RenderEngineManager() :
  dataPtr(new RenderEngineManagerPrivate)
{
  this->dataPtr->RegisterDefaultEngines();
}

//////////////////////////////////////////////////
RenderEngineManager::~RenderEngineManager()
{
}

//////////////////////////////////////////////////
unsigned int RenderEngineManager::EngineCount() const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->enginesMutex);
  return this->dataPtr->engines.size();
}

//////////////////////////////////////////////////
bool RenderEngineManager::HasEngine(const std::string &_name) const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->enginesMutex);
  auto iter = this->dataPtr->engines.find(_name);

  if (iter == this->dataPtr->engines.end())
  {
    // Check if the provided name is a name of a default engine, if so,
    // translate the name to the shared library name
    auto defaultIt = this->dataPtr->defaultEngines.find(_name);
    if (defaultIt != this->dataPtr->defaultEngines.end())
      iter = this->dataPtr->engines.find(defaultIt->second);
  }

  return iter != this->dataPtr->engines.end();
}

//////////////////////////////////////////////////
bool RenderEngineManager::IsEngineLoaded(const std::string &_name) const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->enginesMutex);
  auto iter = this->dataPtr->engines.find(_name);

  if (iter == this->dataPtr->engines.end())
  {
    // Check if the provided name is a name of a default engine, if so,
    // translate the name to the shared library name
    auto defaultIt = this->dataPtr->defaultEngines.find(_name);
    if (defaultIt != this->dataPtr->defaultEngines.end())
    {
      iter = this->dataPtr->engines.find(defaultIt->second);
      if (iter == this->dataPtr->engines.end())
        return false;
    }
    else
      return false;
  }

  return nullptr != iter->second;
}

//////////////////////////////////////////////////
std::vector<std::string> RenderEngineManager::LoadedEngines() const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->enginesMutex);
  std::vector<std::string> engines;
  for (auto [name, engine] :  // NOLINT(whitespace/braces)
      this->dataPtr->engines)
  {
    std::string n = name;
    if (nullptr != engine)
    {
      // ign-rendering3 changed loaded engine names to the actual lib name.
      // For backward compatibility, return engine name if it is one of the
      // default engines
      for (const auto &it : this->dataPtr->defaultEngines)
      {
        if (it.second == name)
        {
          n = it.first;
          break;
        }
      }
      engines.push_back(n);
    }
  }
  return engines;
}

//////////////////////////////////////////////////
RenderEngine *RenderEngineManager::Engine(const std::string &_name,
    const std::map<std::string, std::string> &_params,
    const std::string &_path)
{
  EngineInfo info{_name, nullptr};
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->enginesMutex);
  // check in the list of available engines
  auto iter = this->dataPtr->engines.find(_name);
  if (iter != this->dataPtr->engines.end())
  {
    info.name = iter->first;
    info.engine = iter->second;
  }

  return this->dataPtr->Engine(info, _params, _path);
}

//////////////////////////////////////////////////
RenderEngine *RenderEngineManager::EngineAt(unsigned int _index,
    const std::map<std::string, std::string> &_params,
    const std::string &_path)
{
  if (_index >= this->EngineCount())
  {
    ignerr << "Invalid render-engine index: " << _index << std::endl;
    return nullptr;
  }

  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->enginesMutex);
  auto iter = this->dataPtr->engines.begin();
  std::advance(iter, _index);
  return this->dataPtr->Engine({iter->first, iter->second}, _params, _path);
}

//////////////////////////////////////////////////
bool RenderEngineManager::UnloadEngine(const std::string &_name)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->enginesMutex);
  // check in the list of available engines
  auto iter = this->dataPtr->engines.find(_name);

  if (iter == this->dataPtr->engines.end())
  {
    // Check if the provided name is a name of a default engine, if so,
    // translate the name to the shared library name
    auto defaultIt = this->dataPtr->defaultEngines.find(_name);
    if (defaultIt != this->dataPtr->defaultEngines.end())
      iter = this->dataPtr->engines.find(defaultIt->second);

    if (iter == this->dataPtr->engines.end())
    {
      ignerr << "No render-engine registered with name: " << _name << std::endl;
      return false;
    }
  }

  return this->dataPtr->UnloadEngine(iter);
}

//////////////////////////////////////////////////
bool RenderEngineManager::UnloadEngineAt(unsigned int _index)
{
  if (_index >= this->EngineCount())
  {
    ignerr << "Invalid render-engine index: " << _index << std::endl;
    return false;
  }

  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->enginesMutex);
  auto iter = this->dataPtr->engines.begin();
  std::advance(iter, _index);
  return this->dataPtr->UnloadEngine(iter);
}

//////////////////////////////////////////////////
void RenderEngineManager::RegisterEngine(const std::string &_name,
    RenderEngine *_engine)
{
  if (!_engine)
  {
    ignerr << "Render-engine cannot be null" << std::endl;
    return;
  }

  if (this->HasEngine(_name))
  {
    ignerr << "Render-engine already registered with name: "
          << _name << std::endl;

    return;
  }

  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->enginesMutex);
  this->dataPtr->engines[_name] = _engine;
}

//////////////////////////////////////////////////
void RenderEngineManager::UnregisterEngine(const std::string &_name)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->enginesMutex);
  auto iter = this->dataPtr->engines.find(_name);

  if (iter != this->dataPtr->engines.end())
  {
    this->dataPtr->UnregisterEngine(iter);
  }
}

//////////////////////////////////////////////////
void RenderEngineManager::UnregisterEngine(RenderEngine *_engine)
{
  if (!_engine)
    return;

  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->enginesMutex);
  auto begin = this->dataPtr->engines.begin();
  auto end = this->dataPtr->engines.end();

  for (auto iter = begin; iter != end; ++iter)
  {
    if (iter->second == _engine)
    {
      this->dataPtr->UnregisterEngine(iter);
      return;
    }
  }
}

//////////////////////////////////////////////////
void RenderEngineManager::UnregisterEngineAt(unsigned int _index)
{
  if (_index >= this->EngineCount())
  {
    ignerr << "Invalid render-engine index: " << _index << std::endl;
    return;
  }

  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->enginesMutex);
  auto iter = this->dataPtr->engines.begin();
  std::advance(iter, _index);
  this->dataPtr->UnregisterEngine(iter);
}

//////////////////////////////////////////////////
void RenderEngineManager::SetPluginPaths(const std::list<std::string> &_paths)
{
  this->dataPtr->pluginPaths = _paths;
}

//////////////////////////////////////////////////
// RenderEngineManagerPrivate
//////////////////////////////////////////////////
RenderEngine *RenderEngineManagerPrivate::Engine(EngineInfo _info,
    const std::map<std::string, std::string> &_params,
    const std::string &_path)
{
  RenderEngine *engine = _info.engine;

  if (!engine)
  {
    std::string libName = _info.name;

    // Check if the provided name is a name of a default engine, if so,
    // translate the name to the shared library name
    auto defaultIt = this->defaultEngines.find(_info.name);
    if (defaultIt != this->defaultEngines.end())
      libName = defaultIt->second;

    // Load the engine plugin
    if (this->LoadEnginePlugin(libName, _path))
    {
      std::lock_guard<std::recursive_mutex> lock(this->enginesMutex);
      auto engineIt = this->engines.find(libName);
      if (engineIt != this->engines.end())
        engine = engineIt->second;
    }
  }

  if (!engine)
    return nullptr;

  if (!engine->IsInitialized())
  {
    engine->Load(_params);
    engine->Init();
  }

  return engine;
}

//////////////////////////////////////////////////
bool RenderEngineManagerPrivate::UnloadEngine(EngineIter _iter)
{
  RenderEngine *engine = _iter->second;

  if (!engine)
    return false;

  engine->Destroy();

  return this->UnloadEnginePlugin(_iter->first);
}

//////////////////////////////////////////////////
void RenderEngineManagerPrivate::RegisterDefaultEngines()
{
  // TODO(anyone): Find a cleaner way to get the default engine library name

  // cppcheck-suppress unreadVariable
  std::string libName = "ignition-rendering-";

  // cppcheck-suppress unusedVariable
  std::string engineName;

  std::lock_guard<std::recursive_mutex> lock(this->enginesMutex);
#if HAVE_OGRE
  engineName = "ogre";
  this->defaultEngines[engineName] = libName + engineName;
  if (this->engines.find(libName + engineName) == this->engines.end())
    this->engines[libName + engineName] = nullptr;
#endif
#if HAVE_OGRE2
  engineName = "ogre2";
  this->defaultEngines[engineName] = libName + engineName;
  if (this->engines.find(libName + engineName) == this->engines.end())
    this->engines[libName + engineName] = nullptr;
#endif
#if HAVE_OPTIX
  engineName = "optix";
  this->defaultEngines[engineName] = libName + engineName;
  if (this->engines.find(libName + engineName) == this->engines.end())
    this->engines[libName + engineName] = nullptr;
#endif
}

//////////////////////////////////////////////////
bool RenderEngineManagerPrivate::LoadEnginePlugin(
    const std::string &_filename, const std::string &_path)
{
  ignmsg << "Loading plugin [" << _filename << "]" << std::endl;

  ignition::common::SystemPaths systemPaths;
  systemPaths.SetPluginPathEnv(this->pluginPathEnv);

  // Add default install folder.
  systemPaths.AddPluginPaths(std::string(IGN_RENDERING_PLUGIN_PATH));
  systemPaths.AddPluginPaths({IGNITION_RENDERING_ENGINE_INSTALL_DIR});

  // Add any preset plugin paths.
  for (const auto &path : this->pluginPaths)
    systemPaths.AddPluginPaths(path);

  // Add extra search path.
  systemPaths.AddPluginPaths(_path);

  auto pathToLib = systemPaths.FindSharedLibrary(_filename);
  if (pathToLib.empty())
  {
    ignerr << "Failed to load plugin [" << _filename <<
              "] : couldn't find shared library." << std::endl;
    return false;
  }

  // Load plugin
  auto pluginNames = this->pluginLoader.LoadLib(pathToLib);
  if (pluginNames.empty())
  {
    ignerr << "Failed to load plugin [" << _filename <<
              "] : couldn't load library on path [" << pathToLib <<
              "]." << std::endl;
    return false;
  }

  auto engineNames = pluginLoader.PluginsImplementing<
      rendering::RenderEnginePlugin>();

  if (engineNames.empty())
  {
    std::stringstream error;
    error << "Found no render engine plugins in ["
          << _filename << "], available interfaces are:"
          << std::endl;
    for (auto pluginName : pluginNames)
    {
      error << "- " << pluginName << std::endl;
    }
    ignerr << error.str();
    return false;
  }

  auto engineName = *engineNames.begin();
  if (engineNames.size() > 1)
  {
    std::stringstream warn;
    warn << "Found multiple render engine plugins in ["
          << _filename << "]:"
          << std::endl;
    for (auto pluginName : engineNames)
    {
      warn << "- " << pluginName << std::endl;
    }
    warn << "Loading [" << engineName << "]." << std::endl;
    ignwarn << warn.str();
  }

  auto plugin = pluginLoader.Instantiate(engineName);
  if (!plugin)
  {
    ignerr << "Failed to instantiate plugin [" << engineName << "]"
           << std::endl;
    return false;
  }

  auto renderPlugin =
      plugin->QueryInterface<rendering::RenderEnginePlugin>();

  if (!renderPlugin)
  {
    ignerr << "Failed to query interface from [" << engineName << "]"
           << std::endl;
    return false;
  }

  // This triggers the engine to be instantiated
  {
    std::lock_guard<std::recursive_mutex> lock(this->enginesMutex);
    this->engines[_filename] = renderPlugin->Engine();
  }

  // store engine plugin data so plugin can be unloaded later
  this->enginePlugins[_filename] = engineName;

  return true;
}

//////////////////////////////////////////////////
bool RenderEngineManagerPrivate::UnloadEnginePlugin(
    const std::string &_engineName)
{
  auto it = this->enginePlugins.find(_engineName);
  if (it == this->enginePlugins.end())
  {
    ignmsg << "Skip unloading engine plugin. [" << _engineName << "] "
           << "not loaded from plugin." << std::endl;
    return false;
  }

  std::string pluginName = it->second;
  this->enginePlugins.erase(it);

#ifndef _WIN32
  // Unloading the plugin on windows causes tests to crash on exit
  // see issue #45
  if (!this->pluginLoader.ForgetLibraryOfPlugin(pluginName))
  {
    ignerr << "Failed to unload plugin: " << pluginName << std::endl;
  }
#endif

  std::lock_guard<std::recursive_mutex> lock(this->enginesMutex);
  auto engineIt = this->engines.find(_engineName);
  if (engineIt == this->engines.end())
    return false;

  // set to null - this means engine is still registered but not loaded
  this->engines[_engineName] = nullptr;

  return true;
}

//////////////////////////////////////////////////
void RenderEngineManagerPrivate::UnregisterEngine(EngineIter _iter)
{
  _iter->second->Destroy();

  std::lock_guard<std::recursive_mutex> lock(this->enginesMutex);
  this->engines.erase(_iter);
}
