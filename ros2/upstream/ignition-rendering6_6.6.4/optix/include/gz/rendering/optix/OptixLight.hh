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
#ifndef GZ_RENDERING_OPTIX_OPTIXLIGHT_HH_
#define GZ_RENDERING_OPTIX_OPTIXLIGHT_HH_

#include "gz/rendering/base/BaseLight.hh"
#include "gz/rendering/optix/OptixNode.hh"
#include "gz/rendering/optix/OptixIncludes.hh"
#include "gz/rendering/optix/OptixLightTypes.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    class IGNITION_RENDERING_OPTIX_VISIBLE OptixLight :
      public BaseLight<OptixNode>
    {
      protected: OptixLight();

      public: virtual ~OptixLight();

      public: virtual gz::math::Color DiffuseColor() const;

      public: virtual void SetDiffuseColor(
          const gz::math::Color &_color);

      public: virtual gz::math::Color SpecularColor() const;

      public: virtual void SetSpecularColor(
          const gz::math::Color &_color);

      public: virtual double AttenuationConstant() const;

      public: virtual void SetAttenuationConstant(double _value);

      public: virtual double AttenuationLinear() const;

      public: virtual void SetAttenuationLinear(double _value);

      public: virtual double AttenuationQuadratic() const;

      public: virtual void SetAttenuationQuadratic(double _value);

      public: virtual double AttenuationRange() const;

      public: virtual void SetAttenuationRange(double _range);

      public: virtual bool CastShadows() const;

      public: virtual void SetCastShadows(bool _castShadows);

      protected: virtual void WritePoseToDeviceImpl();

      protected: virtual OptixCommonLightData &CommonData() = 0;

      protected: virtual const OptixCommonLightData &CommonData() const = 0;

      protected: virtual void Init();
    };

    class IGNITION_RENDERING_OPTIX_VISIBLE OptixDirectionalLight :
      public BaseDirectionalLight<OptixLight>
    {
      protected: OptixDirectionalLight();

      public: virtual ~OptixDirectionalLight();

      public: virtual gz::math::Vector3d Direction() const;

      public: virtual void SetDirection(const gz::math::Vector3d &_dir);

      public: virtual OptixDirectionalLightData Data() const;

      public: virtual void PreRender();

      protected: virtual OptixCommonLightData &CommonData();

      protected: virtual const OptixCommonLightData &CommonData() const;

      protected: OptixDirectionalLightData data;

      private: OptixDirectionalLightPtr SharedThis();

      private: friend class OptixScene;
    };

    class IGNITION_RENDERING_OPTIX_VISIBLE OptixPointLight :
      public BasePointLight<OptixLight>
    {
      protected: OptixPointLight();

      public: virtual ~OptixPointLight();

      public: virtual OptixPointLightData Data() const;

      public: virtual void PreRender();

      protected: virtual OptixCommonLightData &CommonData();

      protected: virtual const OptixCommonLightData &CommonData() const;

      protected: OptixPointLightData data;

      private: OptixPointLightPtr SharedThis();

      private: friend class OptixScene;
    };

    class IGNITION_RENDERING_OPTIX_VISIBLE OptixSpotLight :
      public BaseSpotLight<OptixLight>
    {
      protected: OptixSpotLight();

      public: virtual ~OptixSpotLight();

      public: virtual gz::math::Vector3d Direction() const;

      public: virtual void SetDirection(const gz::math::Vector3d &_dir);

      public: virtual gz::math::Angle InnerAngle() const;

      public: virtual void SetInnerAngle(const gz::math::Angle &_angle);

      public: virtual gz::math::Angle OuterAngle() const;

      public: virtual void SetOuterAngle(const gz::math::Angle &_angle);

      public: virtual double Falloff() const;

      public: virtual void SetFalloff(double _falloff);

      public: virtual OptixSpotLightData Data() const;

      public: virtual void PreRender();

      protected: virtual OptixCommonLightData &CommonData();

      protected: virtual const OptixCommonLightData &CommonData() const;

      protected: OptixSpotLightData data;

      private: OptixSpotLightPtr SharedThis();

      private: friend class OptixScene;
    };
    }
  }
}
#endif
