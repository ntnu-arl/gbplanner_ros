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
#ifndef GZ_RENDERING_LIGHT_HH_
#define GZ_RENDERING_LIGHT_HH_

#include "gz/math/Color.hh"
#include "gz/rendering/config.hh"
#include "gz/rendering/Node.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \class Light Light.hh ignition/rendering/Light.hh
    /// \brief Represents a light source in the scene graph
    class IGNITION_RENDERING_VISIBLE Light :
      public virtual Node
    {
      /// \brief Deconstructor
      public: virtual ~Light() { }

      /// \brief Get the diffuse color
      /// \return The diffuse color
      public: virtual math::Color DiffuseColor() const = 0;

      /// \brief Set the diffuse color
      /// \param[in] _r Red value
      /// \param[in] _g Green value
      /// \param[in] _b Blue value
      /// \param[in] _a Alpha value
      public: virtual void SetDiffuseColor(double _r, double _g, double _b,
                  double _a = 1.0) = 0;

      /// \brief Set the diffuse color
      /// \param[in] _color New diffuse color
      public: virtual void SetDiffuseColor(const math::Color &_color) = 0;

      /// \brief Get the specular color
      /// \return The specular color
      public: virtual math::Color SpecularColor() const = 0;

      /// \brief Set the specular color
      /// \param[in] _r Red value
      /// \param[in] _g Green value
      /// \param[in] _b Blue value
      /// \param[in] _a Alpha value
      public: virtual void SetSpecularColor(double _r, double _g, double _b,
                  double _a = 1.0) = 0;

      /// \brief Set the specular color
      /// \param[in] _color New specular color
      public: virtual void SetSpecularColor(const math::Color &_color) = 0;

      /// \brief Get the attenuation constant value
      /// \return The attenuation constant value
      public: virtual double AttenuationConstant() const = 0;

      /// \brief Set the attenuation constant value
      /// \param[in] _value New attenuation constant value
      public: virtual void SetAttenuationConstant(double _value) = 0;

      /// \brief Get the attenuation linear value
      /// \return The attenuation linear value
      public: virtual double AttenuationLinear() const = 0;

      /// \brief Set the attenuation linear value
      /// \param[in] _value New attenuation linear value
      public: virtual void SetAttenuationLinear(double _value) = 0;

      /// \brief Get the attenuation quadratic value
      /// \return The attenuation quadratic value
      public: virtual double AttenuationQuadratic() const = 0;

      /// \brief Set the attenuation quadratic value
      /// \param[in] _value New attenuation quadratic value
      public: virtual void SetAttenuationQuadratic(double _value) = 0;

      /// \brief Get the attenuation range
      /// \return The attenuation range
      public: virtual double AttenuationRange() const = 0;

      /// \brief Set the attenuation range
      /// \param[in] _range New attenuation range
      public: virtual void SetAttenuationRange(double _range) = 0;

      /// \brief Determine if this light cast shadows
      /// \return True if this light cast shadows
      public: virtual bool CastShadows() const = 0;

      /// \brief Specify if this light should cast shadows
      /// \param[in] _castShadows True if this light cast shadows
      public: virtual void SetCastShadows(bool _castShadows) = 0;

      /// \brief Get the light intensity
      /// \return The light intensity
      public: virtual double Intensity() const = 0;

      /// \brief Set the light intensity
      /// \param[in] _intensity New light intensity
      public: virtual void SetIntensity(double _intensity) = 0;
    };

    /// \class DirectionalLight Light.hh ignition/rendering/Light.hh
    /// \brief Represents a infinite directional light
    class IGNITION_RENDERING_VISIBLE DirectionalLight :
      public virtual Light
    {
      /// \brief Destructor
      public: virtual ~DirectionalLight() { }

      /// \brief Get the direction of the light
      /// \return The direction of the light
      public: virtual math::Vector3d Direction() const = 0;

      /// \brief Set the direction of the light
      /// \param[in] _x X-component of direction vector
      /// \param[in] _y Y-component of direction vector
      /// \param[in] _z Z-component of direction vector
      public: virtual void SetDirection(double _x, double _y, double _z) = 0;

      /// \brief Set the direction of the light
      /// \param[in] _dir New direction vector
      public: virtual void SetDirection(const math::Vector3d &_dir) = 0;
    };

    /// \class PointLight Light.hh ignition/rendering/Light.hh
    /// \brief Represents a point light
    class IGNITION_RENDERING_VISIBLE PointLight :
      public virtual Light
    {
      /// \brief Deconstructor
      public: virtual ~PointLight() { }
    };

    /// \class SpotLight Light.hh ignition/rendering/Light.hh
    /// \brief Represents a spotlight
    class IGNITION_RENDERING_VISIBLE SpotLight :
      public virtual Light
    {
      /// \brief Deconstructor
      public: virtual ~SpotLight() { }

      /// \brief Get direction of the light
      /// \return The direction of the light
      public: virtual math::Vector3d Direction() const = 0;

      /// \brief Set the direction of the light
      /// \param[in] _x X-component of direction vector
      /// \param[in] _y Y-component of direction vector
      /// \param[in] _z Z-component of direction vector
      public: virtual void SetDirection(double _x, double _y, double _z) = 0;

      /// \brief Set the direction of the light
      /// \param[in] _dir New direction vector
      public: virtual void SetDirection(const math::Vector3d &_dir) = 0;

      /// \brief Get the inner angle of the spotlight
      /// \return The inner angle of the spotlight
      public: virtual math::Angle InnerAngle() const = 0;

      /// \brief Set the inner angle of the spotlight
      /// \param[in] _radians New inner angle of the spotlight in radians
      public: virtual void SetInnerAngle(double _radians) = 0;

      /// \brief Set the inner angle of the spotlight
      /// \param[in] _angle New inner angle of the spotlight
      public: virtual void SetInnerAngle(const math::Angle &_angle) = 0;

      /// \brief Get the outer angle of the spotlight
      /// \return The outer angle of the spotlight
      public: virtual math::Angle OuterAngle() const = 0;

      /// \brief Set the outer angle of the spotlight
      /// \param[in] _radians New outer angle of the spotlight in radians
      public: virtual void SetOuterAngle(double _radians) = 0;

      /// \brief Set the outer angle of the spotlight
      /// \param[in] _angle New outer angle of the spotlight
      public: virtual void SetOuterAngle(const math::Angle &_angle) = 0;

      /// \brief Get the falloff of the spotlight
      /// \return The falloff of the spotlight
      public: virtual double Falloff() const = 0;

      /// \brief Set the falloff of the spotlight
      /// \param[in] _falloff New falloff of the spotlight
      public: virtual void SetFalloff(double _falloff) = 0;
    };
    }
  }
}
#endif
