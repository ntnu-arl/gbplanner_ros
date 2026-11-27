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
#ifndef GZ_RENDERING_OPTIX_OPTIXRENDERENGINE_HH_
#define GZ_RENDERING_OPTIX_OPTIXRENDERENGINE_HH_

#include <map>
#include <string>
#include <gz/common/SingletonT.hh>

#include "gz/rendering/RenderEnginePlugin.hh"
#include "gz/rendering/base/BaseRenderEngine.hh"
#include "gz/rendering/optix/OptixRenderTypes.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \brief Plugin for loading optix render engine
    class IGNITION_RENDERING_OPTIX_VISIBLE OptixRenderEnginePlugin :
      public RenderEnginePlugin
    {
      /// \brief Constructor
      public: OptixRenderEnginePlugin();

      /// \brief Destructor
      public: ~OptixRenderEnginePlugin() = default;

      /// \brief Get the name of the render engine loaded by this plugin.
      /// \return Name of render engine
      public: std::string Name() const;

      /// \brief Get a pointer to the render engine loaded by this plugin.
      /// \return Render engine instance
      public: RenderEngine *Engine() const;
    };

    class IGNITION_RENDERING_OPTIX_VISIBLE OptixRenderEngine :
      public virtual BaseRenderEngine,
      public common::SingletonT<OptixRenderEngine>
    {
      /// \brief Constructor
      private: OptixRenderEngine();

      public: virtual ~OptixRenderEngine();

      public: virtual bool Fini();

      public: virtual std::string Name() const;

      public: std::string PtxFile(const std::string& _fileBase) const;

      protected: virtual ScenePtr CreateSceneImpl(unsigned int _id,
                  const std::string &_name);

      protected: virtual SceneStorePtr Scenes() const;

      // Documentation Inherited
      protected: virtual bool LoadImpl(
          const std::map<std::string, std::string> &_params) override;

      protected: virtual bool InitImpl();

      private: OptixSceneStorePtr scenes;

      private: static const std::string PTX_PREFIX;

      private: static const std::string PTX_SUFFIX;

      private: friend class SingletonT<OptixRenderEngine>;
    };
    }
  }
}
#endif
