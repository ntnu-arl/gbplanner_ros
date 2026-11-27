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
#ifndef GZ_RENDERING_OPTIX_OPTIXLIGHTMANAGER_HH_
#define GZ_RENDERING_OPTIX_OPTIXLIGHTMANAGER_HH_

#include <string>
#include <vector>
#include "gz/rendering/config.hh"
#include "gz/rendering/optix/OptixLightTypes.hh"
#include "gz/rendering/optix/OptixRenderTypes.hh"
#include "gz/rendering/optix/OptixIncludes.hh"
#include "gz/rendering/optix/Export.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    class IGNITION_RENDERING_OPTIX_VISIBLE OptixLightManager
    {
      public: explicit OptixLightManager(OptixScenePtr _scene);

      public: virtual ~OptixLightManager();

      public: virtual void AddDirectionalLight(OptixDirectionalLightPtr _light);

      public: virtual void AddPointLight(OptixPointLightPtr _light);

      public: virtual void AddSpotLight(OptixSpotLightPtr _light);

      public: virtual void PreRender();

      public: virtual void Clear();

      protected: virtual void WriteDirectionalBuffer();

      protected: virtual void WritePointBuffer();

      protected: virtual void WriteSpotBuffer();

      protected: template <class T>
                 void WriteBuffer(optix::Buffer _buffer,
                     const std::vector<T> &_data);

      private: void CreateBuffers();

      private: template <class T>
               optix::Buffer CreateBuffer(const std::string &_name);

      protected: OptixScenePtr scene;

      protected: std::vector<OptixDirectionalLightData> directionalData;

      protected: std::vector<OptixPointLightData> pointData;

      protected: std::vector<OptixSpotLightData> spotData;

      protected: optix::Buffer directionalBuffer;

      protected: optix::Buffer pointBuffer;

      protected: optix::Buffer spotBuffer;
    };
    }
  }
}
#endif
