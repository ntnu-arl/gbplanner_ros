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
#ifndef GZ_RENDERING_RENDERENGINEMANAGER_HH_
#define GZ_RENDERING_RENDERENGINEMANAGER_HH_

#include <list>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <gz/common/SingletonT.hh>
#include <gz/common/SuppressWarning.hh>
#include "gz/rendering/config.hh"
#include "gz/rendering/Export.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    // forward declarations.
    class RenderEngine;
    class RenderEngineManagerPrivate;

    /// \class RenderEngineManager RenderEngineManager.hh
    /// ignition/rendering/RenderEngineManager.hh
    /// \brief Collection of render-engines. This provides access to all the
    /// render-engines available at runtime. RenderEngine objects should not
    /// be access directly, but instead via the RenderEngineManager to maintain
    /// a flexible render-engine agnostic design.
    class IGNITION_RENDERING_VISIBLE RenderEngineManager :
      public virtual common::SingletonT<RenderEngineManager>
    {
      /// \brief Constructor
      public: RenderEngineManager();

      /// \brief Deconstructor
      public: ~RenderEngineManager();

      /// \brief Get the number of available render-engines
      /// \return the number of available render-engines
      public: unsigned int EngineCount() const;

      /// \brief Determine if a render-engine with the given name is avaiable.
      /// It also checks the list of default engines supplied by ign-rendering.
      /// \param[in] _name Name of the desired render-engine
      /// \return True if the specified render-engine is available
      public: bool HasEngine(const std::string &_name) const;

      /// \brief Determine if a render-engine with the given name is already
      /// loaded.
      /// \param[in] _name Name of the desired render-engine
      /// \return True if the specified render-engine is loaded.
      public: bool IsEngineLoaded(const std::string &_name) const;

      /// \brief Get the list of all engines already loaded.
      /// \return Names of all loaded engines.
      public: std::vector<std::string> LoadedEngines() const;

      /// \brief Get the render-engine with the given name. If the no
      /// render-engine is registered under the given name, NULL will be
      /// returned.
      /// \param[in] _name Name of the desired render-engine
      /// \param[in] _params Parameters to be passed to the render engine.
      /// \param[in] _path Another search path for rendering engine plugin.
      /// \return The specified render-engine
      public: RenderEngine *Engine(const std::string &_name,
                  const std::map<std::string, std::string> &_params = {},
                  const std::string &_path = "");

      /// \brief Get the render-engine at the given index. If no
      /// render-engine is exists at the given index, NULL will be returned.
      /// \param[in] _index Index of the desired render-engine
      /// \param[in] _params Parameters to be passed to the render engine.
      /// \param[in] _path Another search path for rendering engine plugin.
      /// \return The specified render-engine
      public: RenderEngine *EngineAt(unsigned int _index,
                  const std::map<std::string, std::string> &_params = {},
                  const std::string &_path = "");

      /// \brief Unload the render-engine with the given name. If the no
      /// render-engine is registered under the given name, false will be
      /// returned.
      /// \param[in] _name Name of the desired render-engine
      /// \return  True if the engine is unloaded
      public: bool UnloadEngine(const std::string &_name);

      /// \brief Unload the render-engine at the given index. If the no
      /// render-engine is registered under the given name, false will be
      /// returned.
      /// \param[in] _index Index of the desired render-engine
      /// \return  True if the engine is unloaded
      public: bool UnloadEngineAt(unsigned int _index);

      /// \brief Register a new render-engine under the given name. If the
      /// given name is already in use, the render-engine will not be
      /// registered.
      /// \param[in] _name Name the render-engine will be registered under
      /// \param[in] _engine Render-engine to be registered
      public: void RegisterEngine(const std::string &_name,
                  RenderEngine *_engine);

      /// \brief Unregister a render-engine registered under the given name.
      /// If no render-engine is registered under the given name no work
      /// will be done.
      /// \param[in] _name Name of the render-engine to unregister
      public: void UnregisterEngine(const std::string &_name);

      /// \brief Unregister the given render-engine. If the given render-engine
      /// is not currently registered, no work will be done.
      /// \param[in] _engine Render-engine to unregister
      public: void UnregisterEngine(RenderEngine *_engine);

      /// \brief Unregister a render-engine at the given index. If the no
      /// render-engine is registered at the given index, no work will be done.
      /// \param[in] _index Index of the render-engine to unregister
      public: void UnregisterEngineAt(unsigned int _index);

      /// \brief Set the plugin paths from which render engines can be loaded.
      /// \param[in] _paths The list of the plugin paths
      public: void SetPluginPaths(const std::list<std::string> &_paths);

      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      /// \brief private implementation details
      private: std::unique_ptr<RenderEngineManagerPrivate> dataPtr;
      IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING

      /// \brief required SingletonT friendship
      private: friend class gz::common::SingletonT<RenderEngineManager>;
    };
    }
  }
}
#endif
