/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#ifndef IGNITION_RENDERING_BASE_BASELIGHTVISUAL_HH_
#define IGNITION_RENDERING_BASE_BASELIGHTVISUAL_HH_

#include <vector>

#include "ignition/rendering/base/BaseObject.hh"
#include "ignition/rendering/base/BaseRenderTypes.hh"
#include "ignition/rendering/LightVisual.hh"
#include "ignition/rendering/Scene.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \brief Base implementation of a light visual
    template <class T>
    class BaseLightVisual :
      public virtual LightVisual,
      public virtual T
    {
      /// \brief Constructor
      protected: BaseLightVisual();

      /// \brief Destructor
      public: virtual ~BaseLightVisual();

      // Documentation inherited.
      protected: virtual void Init() override;

      // Documentation inherited.
      protected: virtual void PreRender() override;

      // Documentation inherited
      public: virtual void SetType(LightVisualType _type) override;

      // Documentation inherited
      public: virtual void SetInnerAngle(double _innerAngle) override;

      // Documentation inherited
      public: virtual double InnerAngle() override;

      // Documentation inherited
      public: virtual void SetOuterAngle(double _innerAngle) override;

      // Documentation inherited
      public: virtual double OuterAngle() override;

      /// \brief Draw the light visual using dynamic renderables
      public: std::vector<ignition::math::Vector3d> CreateVisualLines();

      // Documentation inherited
      public: virtual LightVisualType Type() override;

      /// \brief Type of light visual
      protected: LightVisualType type = LightVisualType::LVT_EMPTY;

      /// \brief Inner angle for spot lights
      protected: double innerAngle = 0;

      /// \brief Outer angle for spot lights
      protected: double outerAngle = 0;

      /// \brief Flag to indicate light properties have changed
      protected: bool dirtyLightVisual = false;
    };

    //////////////////////////////////////////////////
    template <class T>
    BaseLightVisual<T>::BaseLightVisual()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    BaseLightVisual<T>::~BaseLightVisual()
    {
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseLightVisual<T>::PreRender()
    {
      T::PreRender();
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseLightVisual<T>::Init()
    {
      T::Init();
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseLightVisual<T>::SetType(LightVisualType _type)
    {
      this->type = _type;
      this->dirtyLightVisual = true;
    }

    //////////////////////////////////////////////////
    template <class T>
    LightVisualType BaseLightVisual<T>::Type()
    {
      return this->type;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseLightVisual<T>::SetInnerAngle(double _innerAngle)
    {
      this->innerAngle = _innerAngle;
      this->dirtyLightVisual = true;
    }

    //////////////////////////////////////////////////
    template <class T>
    double BaseLightVisual<T>::InnerAngle()
    {
      return this->innerAngle;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseLightVisual<T>::SetOuterAngle(double _outerAngle)
    {
      this->outerAngle = _outerAngle;
      this->dirtyLightVisual = true;
    }

    //////////////////////////////////////////////////
    template <class T>
    double BaseLightVisual<T>::OuterAngle()
    {
      return this->outerAngle;
    }

    template <class T>
    std::vector<ignition::math::Vector3d>
    BaseLightVisual<T>::CreateVisualLines()
    {
      std::vector<ignition::math::Vector3d> positions;

      if (this->type == LightVisualType::LVT_DIRECTIONAL)
      {
        float s = 0.5;
        positions.emplace_back(ignition::math::Vector3d(-s, -s, 0));
        positions.emplace_back(ignition::math::Vector3d(-s, s, 0));

        positions.emplace_back(ignition::math::Vector3d(-s, s, 0));
        positions.emplace_back(ignition::math::Vector3d(s, s, 0));

        positions.emplace_back(ignition::math::Vector3d(s, s, 0));
        positions.emplace_back(ignition::math::Vector3d(s, -s, 0));

        positions.emplace_back(ignition::math::Vector3d(s, -s, 0));
        positions.emplace_back(ignition::math::Vector3d(-s, -s, 0));

        positions.emplace_back(ignition::math::Vector3d(0, 0, 0));
        positions.emplace_back(ignition::math::Vector3d(0, 0, -s));
      }
      else if (this->type == LightVisualType::LVT_POINT)
      {
        float s = 0.1f;
        positions.emplace_back(ignition::math::Vector3d(-s, -s, 0));
        positions.emplace_back(ignition::math::Vector3d(-s, s, 0));

        positions.emplace_back(ignition::math::Vector3d(-s, s, 0));
        positions.emplace_back(ignition::math::Vector3d(s, s, 0));

        positions.emplace_back(ignition::math::Vector3d(s, s, 0));
        positions.emplace_back(ignition::math::Vector3d(s, -s, 0));

        positions.emplace_back(ignition::math::Vector3d(s, -s, 0));
        positions.emplace_back(ignition::math::Vector3d(-s, -s, 0));

        positions.emplace_back(ignition::math::Vector3d(-s, -s, 0));
        positions.emplace_back(ignition::math::Vector3d(0, 0, s));

        positions.emplace_back(ignition::math::Vector3d(-s, s, 0));
        positions.emplace_back(ignition::math::Vector3d(0, 0, s));

        positions.emplace_back(ignition::math::Vector3d(s, s, 0));
        positions.emplace_back(ignition::math::Vector3d(0, 0, s));

        positions.emplace_back(ignition::math::Vector3d(s, -s, 0));
        positions.emplace_back(ignition::math::Vector3d(0, 0, s));

        positions.emplace_back(ignition::math::Vector3d(-s, -s, 0));
        positions.emplace_back(ignition::math::Vector3d(0, 0, -s));

        positions.emplace_back(ignition::math::Vector3d(-s, s, 0));
        positions.emplace_back(ignition::math::Vector3d(0, 0, -s));

        positions.emplace_back(ignition::math::Vector3d(s, s, 0));
        positions.emplace_back(ignition::math::Vector3d(0, 0, -s));

        positions.emplace_back(ignition::math::Vector3d(s, -s, 0));
        positions.emplace_back(ignition::math::Vector3d(0, 0, -s));
      }
      else if (this->type == LightVisualType::LVT_SPOT)
      {
        double angles[2];
        double range = 0.2;
        angles[0] = range * tan(outerAngle / 2.0);
        angles[1] = range * tan(innerAngle / 2.0);

        unsigned int i = 0;
        positions.emplace_back(ignition::math::Vector3d(0, 0, 0));
        positions.emplace_back(
            ignition::math::Vector3d(angles[i], angles[i], -range));

        for (i = 0; i < 2; i++)
        {
          positions.emplace_back(ignition::math::Vector3d(0, 0, 0));
          positions.emplace_back(ignition::math::Vector3d(
                angles[i], angles[i], -range));

          positions.emplace_back(ignition::math::Vector3d(0, 0, 0));
          positions.emplace_back(ignition::math::Vector3d(
                -angles[i], -angles[i], -range));

          positions.emplace_back(ignition::math::Vector3d(0, 0, 0));
          positions.emplace_back(ignition::math::Vector3d(
                angles[i], -angles[i], -range));

          positions.emplace_back(ignition::math::Vector3d(0, 0, 0));
          positions.emplace_back(ignition::math::Vector3d(
                -angles[i], angles[i], -range));

          positions.emplace_back(ignition::math::Vector3d(
                angles[i], angles[i], -range));
          positions.emplace_back(ignition::math::Vector3d(
                -angles[i], angles[i], -range));

          positions.emplace_back(ignition::math::Vector3d(
                -angles[i], angles[i], -range));
          positions.emplace_back(ignition::math::Vector3d(
                -angles[i], -angles[i], -range));

          positions.emplace_back(ignition::math::Vector3d(
                -angles[i], -angles[i], -range));
          positions.emplace_back(ignition::math::Vector3d(
                angles[i], -angles[i], -range));

          positions.emplace_back(ignition::math::Vector3d(
                angles[i], -angles[i], -range));
          positions.emplace_back(ignition::math::Vector3d(
                angles[i], angles[i], -range));
        }
      }
      return positions;
    }
    }
  }
}
#endif
