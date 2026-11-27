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
#ifndef GZ_RENDERING_RENDERINGIFACE_HH_
#define GZ_RENDERING_RENDERINGIFACE_HH_

#include <list>
#include <map>
#include <string>
#include <vector>

#include "gz/rendering/config.hh"
#include "gz/rendering/Export.hh"
#include "gz/rendering/RenderTypes.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    class RenderEngine;

    /// \brief Load shared render-engine resources
    /// \return True if successful
    IGNITION_RENDERING_VISIBLE
    bool load();

    /// \brief Initialized shared render-engine features
    /// \return True if successful
    IGNITION_RENDERING_VISIBLE
    bool init();

    /// \brief Destroy all render-engines and related resources
    /// \return True if successful
    IGNITION_RENDERING_VISIBLE
    bool fini();

    /// \brief Get the number of available render-engines.
    /// \return The number of available render-engines
    IGNITION_RENDERING_VISIBLE
    unsigned int engineCount();

    /// \brief Determine if a render-engine is registered under the given name
    /// \param[in] _name Name of the desired render-engine
    /// \return True if a render-engine is registered under the given name
    IGNITION_RENDERING_VISIBLE
    bool hasEngine(const std::string &_name);

    /// \brief Determine if a render-engine is already loaded.
    /// \param[in] _name Name of the desired render-engine
    /// \return True if a render-engine is loaded under the given name
    IGNITION_RENDERING_VISIBLE
    bool isEngineLoaded(const std::string &_name);

    /// \brief Get the names of all engines currently loaded.
    /// \return All the engines currently loaded.
    IGNITION_RENDERING_VISIBLE
    std::vector<std::string> loadedEngines();

    /// \brief Get the render-engine registered under the given name. If no
    /// render-engine is registered under the given name, NULL will be
    /// returned.
    /// If the engine is registered, but not loaded, this function will load it.
    /// \param[in] _name Name of the desired render-engine
    /// \param[in] _params Parameters to be passed to the render engine.
    /// \param[in] _path Another search path for rendering engine plugin.
    /// \return The specified render-engine
    IGNITION_RENDERING_VISIBLE
    RenderEngine *engine(const std::string &_name,
        const std::map<std::string, std::string> &_params = {},
        const std::string &_path = "");

    /// \brief Get the render-engine registered at the given index. If no
    /// render-engine is registered at the given index, NULL will be returned.
    /// If the engine is registered, but not loaded, this function will load it.
    /// \param[in] _index Index of the desired render-engine
    /// \param[in] _params Parameters to be passed to the render engine.
    /// \param[in] _path Another search path for rendering engine plugin.
    /// \return The specified render-engine
    IGNITION_RENDERING_VISIBLE
    RenderEngine *engine(const unsigned int _index,
        const std::map<std::string, std::string> &_params = {},
        const std::string &_path = "");

    /// \brief Unload the render-engine registered under the given name.
    /// \param[in] _name Name of the desired render-engine
    /// \return True if the engine is unloaded
    IGNITION_RENDERING_VISIBLE
    bool unloadEngine(const std::string &_name);

    /// \brief Register a new render-engine under the given name. If the given
    /// name is already in use, the render-engine will not be registered.
    /// \param[in] _name Name the render-engine will be registered under
    /// \param[in] _engine Render-engine to be registered
    IGNITION_RENDERING_VISIBLE
    void registerEngine(const std::string &_name, RenderEngine *_engine);

    /// \brief Unregister a render-engine registered under the given name.
    /// If the no render-engine is registered under the given name no work
    /// will be done.
    /// \param[in] _name Name of the render-engine to unregister
    IGNITION_RENDERING_VISIBLE
    void unregisterEngine(const std::string &_name);

    /// \brief Unregister the given render-engine. If the given render-engine
    /// is not currently registered, no work will be done.
    /// \param[in] _engine Render-engine to unregister
    IGNITION_RENDERING_VISIBLE
    void unregisterEngine(RenderEngine *_engine);

    /// \brief Unregister a render-engine at the given index. If the no
    /// render-engine is registered at the given index, no work will be done.
    /// \param[in] _index Index of the render-engine to unregister
    IGNITION_RENDERING_VISIBLE
    void unregisterEngine(const unsigned int _index);

    /// \brief Set the plugin paths from which render engines can be loaded.
    /// \param[in] _paths The list of the plugin paths
    IGNITION_RENDERING_VISIBLE
    void setPluginPaths(const std::list<std::string> &_paths);

    /// \brief Most applications will only have one rendering engine loaded
    /// at a time, and only one scene within that. This helper function gets
    /// the first scene that can be found in the first loaded rendering engine.
    ///
    /// It's not recommended to call this function when there's more than one
    /// engine or scene.
    ///
    /// \return Pointer to a scene that was found, null if no scene is loaded.
    IGNITION_RENDERING_VISIBLE
    ScenePtr sceneFromFirstRenderEngine();
    }
  }
}
#endif
