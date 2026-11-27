/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#ifndef GZ_RENDERING_RENDERENGINEPLUGIN_HH_
#define GZ_RENDERING_RENDERENGINEPLUGIN_HH_

#include <memory>
#include <string>

#include <gz/common/SuppressWarning.hh>

#include "gz/rendering/config.hh"
#include "gz/rendering/Export.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    // Forward declarations
    class RenderEngine;
    class RenderEnginePluginPrivate;

    /// \brief Base plugin class for render engines
    class IGNITION_RENDERING_VISIBLE RenderEnginePlugin
    {
      /// \brief Constructor
      public: RenderEnginePlugin();

      /// \brief Destructor
      public: virtual ~RenderEnginePlugin();

      /// \brief Get the name of render engine
      /// \return Name of render engine
      public: virtual std::string Name() const = 0;

      /// \brief Get a pointer to the render engine
      /// \return Render engine instance
      public: virtual RenderEngine *Engine() const = 0;

      /// \brief Pointer to private data class
      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      public: std::unique_ptr<RenderEnginePluginPrivate> dataPtr;
      IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING
    };
    }
  }
}
#endif
