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
#ifndef IGNITION_RENDERING_LIGHTVISUAL_HH_
#define IGNITION_RENDERING_LIGHTVISUAL_HH_

#include "ignition/rendering/config.hh"
#include "ignition/rendering/Object.hh"
#include "ignition/rendering/RenderTypes.hh"
#include "ignition/rendering/Visual.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {

      /// \brief Enum for LightVisual types
      enum IGNITION_RENDERING_VISIBLE LightVisualType
      {
        /// \brief No type
        LVT_EMPTY                = 0,

        /// \brief Point light
        LVT_POINT               = 1,

        /// \brief Directional light
        LVT_DIRECTIONAL         = 2,

        /// \brief Spot light
        LVT_SPOT                = 3
      };

    /// \class LightVisual LightVisual.hh ignition/rendering/LightVisual.hh
    /// \brief Represents a light visual
    class IGNITION_RENDERING_VISIBLE LightVisual :
      public virtual Visual
    {
      /// \brief Descructor
      public: virtual ~LightVisual() {}

      /// \brief set type of the light
      /// \param[in] _type type of the light
      public: virtual void SetType(LightVisualType _type) = 0;

      /// \brief Get light visual type
      /// \return The light visual type
      public: virtual LightVisualType Type() = 0;

      /// \brief Set inner angle for spot lights
      /// \param[in] _innerAngle Inner angle
      public: virtual void SetInnerAngle(double _innerAngle) = 0;

      /// \brief Get inner angle
      /// \return The light inner angle
      public: virtual double InnerAngle() = 0;

      /// \brief Set outer angle for spot lights
      /// \param[in] _outerAngle Outer angle
      public: virtual void SetOuterAngle(double _outerAngle) = 0;

      /// \brief Get outer angle
      /// \return The light outer angle
      public: virtual double OuterAngle() = 0;
    };
    }
  }
}
#endif
