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
#ifndef GZ_RENDERING_OPTIX_OPTIXRENDERTARGET_HH_
#define GZ_RENDERING_OPTIX_OPTIXRENDERTARGET_HH_

#include "gz/rendering/base/BaseRenderTarget.hh"
#include "gz/rendering/optix/OptixIncludes.hh"
#include "gz/rendering/optix/OptixRenderTypes.hh"
#include "gz/rendering/optix/OptixObject.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    class IGNITION_RENDERING_OPTIX_VISIBLE OptixRenderTarget :
      public virtual BaseRenderTarget<OptixObject>
    {
      protected: OptixRenderTarget();

      public: virtual ~OptixRenderTarget();

      public: virtual void Copy(Image &_image) const;

      public: virtual optix::Buffer OptixBuffer() const = 0;

      protected: unsigned int MemorySize() const;

      protected: float *hostData;
    };

    class IGNITION_RENDERING_OPTIX_VISIBLE OptixRenderTexture :
      public virtual BaseRenderTexture<OptixRenderTarget>
    {
      protected: OptixRenderTexture();

      public: virtual ~OptixRenderTexture();

      public: virtual void Destroy();

      public: virtual optix::Buffer OptixBuffer() const;

      protected: virtual void RebuildImpl();

      protected: optix::Buffer optixBuffer;

      protected: virtual void Init();

      private: friend class OptixScene;
    };

    /// \brief Render window mocked using a render texture.
    class IGNITION_RENDERING_OPTIX_VISIBLE OptixRenderWindow :
      public virtual BaseRenderWindow<OptixRenderTexture>
    {
      /// \brief Constructor
      protected: OptixRenderWindow();

      /// \brief Destructor
      public: virtual ~OptixRenderWindow();

      /// \brief Render windows should only be created by the scene class
      private: friend class OptixScene;
    };
    }
  }
}
#endif
