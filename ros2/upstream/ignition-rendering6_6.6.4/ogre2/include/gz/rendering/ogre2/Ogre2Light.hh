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
#ifndef GZ_RENDERING_OGRE2_OGRE2LIGHT_HH_
#define GZ_RENDERING_OGRE2_OGRE2LIGHT_HH_

#include <memory>

#include "gz/rendering/base/BaseLight.hh"
#include "gz/rendering/ogre2/Ogre2Node.hh"

#ifdef _MSC_VER
  #pragma warning(push, 0)
#endif
#include <OgreLight.h>
#ifdef _MSC_VER
  #pragma warning(pop)
#endif

namespace Ogre
{
  class Light;
}

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    // forward declaration
    class Ogre2LightPrivate;

    /// \brief Ogre 2.x implementation of the light class
    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2Light :
      public BaseLight<Ogre2Node>
    {
      /// \brief Constructor
      protected: Ogre2Light();

      /// \brief Destructor
      public: virtual ~Ogre2Light();

      // Documentation Inherited
      public: virtual math::Color DiffuseColor() const override;

      // Documentation Inherited
      public: virtual void SetDiffuseColor(const math::Color &_color) override;

      // Documentation Inherited
      public: virtual math::Color SpecularColor() const override;

      // Documentation Inherited
      public: virtual void SetSpecularColor(const math::Color &_color) override;

      // Documentation Inherited
      public: virtual double AttenuationConstant() const override;

      // Documentation Inherited
      public: virtual void SetAttenuationConstant(double _value) override;

      // Documentation Inherited
      public: virtual double AttenuationLinear() const override;

      // Documentation Inherited
      public: virtual void SetAttenuationLinear(double _value) override;

      // Documentation Inherited
      public: virtual double AttenuationQuadratic() const override;

      // Documentation Inherited
      public: virtual void SetAttenuationQuadratic(double _value) override;

      // Documentation Inherited
      public: virtual double AttenuationRange() const override;

      // Documentation Inherited
      public: virtual void SetAttenuationRange(double _range) override;

      // Documentation Inherited
      public: virtual bool CastShadows() const override;

      // Documentation Inherited
      public: virtual void SetCastShadows(bool _castShadows) override;

      // Documentation Inherited
      public: virtual double Intensity() const override;

      // Documentation Inherited
      public: virtual void SetIntensity(double _intensity) override;

      /// \brief Get a pointer to ogre light
      public: virtual Ogre::Light *Light() const;

      /// \brief Destroy the light
      public: virtual void Destroy() override;

      /// \brief Initialize the light
      protected: virtual void Init() override;

      /// \brief Create the light
      private: void CreateLight();

      /// \brief Update the attenuation based on the values specified.
      private: void UpdateAttenuation();

      /// \brief Attenuation constant value
      protected: double attenConstant = 1.0;

      /// \brief Attenuation linear factor
      protected: double attenLinear = 0.0;

      /// \brief Attenuation quadratic factor
      protected: double attenQuadratic = 0.0;

      /// \brief Attenuation range
      protected: double attenRange = 100.0;

      /// \brief Pointer to ogre light
      protected: Ogre::Light *ogreLight = nullptr;

      /// \brief Light type
      protected: Ogre::Light::LightTypes ogreLightType;

      /// \brief Pointer to private data class
      private: std::unique_ptr<Ogre2LightPrivate> dataPtr;
    };

    /// \brief Ogre 2.x implementation of the directional light class
    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2DirectionalLight :
      public BaseDirectionalLight<Ogre2Light>
    {
      /// \brief Constructor
      protected: Ogre2DirectionalLight();

      /// \brief Destructor
      public: virtual ~Ogre2DirectionalLight();

      // Documentation Inherited
      public: virtual math::Vector3d Direction() const override;

      // Documentation Inherited
      public: virtual void SetDirection(const math::Vector3d &_dir) override;

    /// \brief Ogre 2.x implementation of the directional light class
      private: friend class Ogre2Scene;
    };

    /// \brief Ogre 2.x implementation of the point light class
    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2PointLight :
      public BasePointLight<Ogre2Light>
    {
      /// \brief Constructor
      protected: Ogre2PointLight();

      /// \brief Destructor
      public: virtual ~Ogre2PointLight();

      /// \brief Only an ogre scene can create an ogre point light
      private: friend class Ogre2Scene;
    };

    /// \brief Ogre 2.x implementation of the spot light class
    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2SpotLight :
      public BaseSpotLight<Ogre2Light>
    {
      /// \brief Constructor
      protected: Ogre2SpotLight();

      /// \brief Destructor
      public: virtual ~Ogre2SpotLight();

      // Documentation inherited.
      public: virtual math::Vector3d Direction() const override;

      // Documentation Inherited
      public: virtual void SetDirection(const math::Vector3d &_dir) override;

      // Documentation Inherited
      public: virtual math::Angle InnerAngle() const override;

      // Documentation Inherited
      public: virtual void SetInnerAngle(const math::Angle &_angle) override;

      // Documentation Inherited
      public: virtual math::Angle OuterAngle() const override;

      // Documentation Inherited
      public: virtual void SetOuterAngle(const math::Angle &_angle) override;

      // Documentation Inherited
      public: virtual double Falloff() const override;

      // Documentation Inherited
      public: virtual void SetFalloff(double _falloff) override;

      /// \brief Only an ogre scene can create an ogre spot light
      private: friend class Ogre2Scene;
    };
    }
  }
}
#endif
