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
#ifndef GZ_RENDERING_OPTIX_OPTIXCAMERA_HH_
#define GZ_RENDERING_OPTIX_OPTIXCAMERA_HH_

#include <string>
#include "gz/rendering/base/BaseCamera.hh"
#include "gz/rendering/optix/OptixRenderTypes.hh"
#include "gz/rendering/optix/OptixSensor.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    class IGNITION_RENDERING_OPTIX_VISIBLE OptixCamera :
      public BaseCamera<OptixSensor>
    {
      protected: OptixCamera();

      public: virtual ~OptixCamera();

      public: virtual void SetHFOV(const math::Angle &_hfov);

      public: virtual void SetAspectRatio(const double _ratio);

      public: virtual void SetAntiAliasing(const unsigned int _aa);

      public: virtual void PreRender();

      public: virtual void Render();

      protected: virtual RenderTargetPtr RenderTarget() const;

      protected: virtual void WriteCameraToDevice();

      protected: virtual void WriteCameraToDeviceImpl();

      protected: virtual void WritePoseToDeviceImpl();

      protected: virtual void Init();

      protected: virtual void CreateRenderTexture();

      // Documentation inherited.
      protected: virtual RenderWindowPtr CreateRenderWindow();

      protected: virtual void CreateRenderProgram();

      protected: virtual void CreateClearProgram();

      protected: virtual void CreateErrorProgram();

      protected: optix::Program optixRenderProgram;

      protected: optix::Program optixClearProgram;

      protected: optix::Program optixErrorProgram;

      protected: OptixRenderTexturePtr renderTexture;

      protected: bool cameraDirty;

      protected: unsigned int traceId;

      protected: unsigned int clearId;

      private: static const std::string PTX_BASE_NAME;

      private: static const std::string PTX_RENDER_FUNCTION;

      private: static const std::string PTX_CLEAR_FUNCTION;

      private: friend class OptixScene;
    };
    }
  }
}
#endif
