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
#ifndef GZ_RENDERING_ARROWVISUAL_HH_
#define GZ_RENDERING_ARROWVISUAL_HH_

#include "gz/rendering/config.hh"
#include "gz/rendering/CompositeVisual.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \class ArrowVisual ArrowVisual.hh ignition/rendering/ArrowVisual.hh
    /// \brief Represents a arrow composite visual
    class IGNITION_RENDERING_VISIBLE ArrowVisual :
      public virtual CompositeVisual
    {
      /// \brief Deconstructor
      public: virtual ~ArrowVisual() { }

      /// \brief Get arrow-head visual
      /// \return The arrow-head visual
      public: virtual VisualPtr Head() const = 0;

      /// \brief Get arrow-shaft visual
      /// \return The arrow-shaft visual
      public: virtual VisualPtr Shaft() const = 0;

      /// \brief Get arrow-rotation visual
      /// \return The arrow-rotation visual
      public: virtual VisualPtr Rotation() const = 0;

      /// \brief set true to show the arrow head, false otherwise
      /// \param[in] _b true to show the arrow head, false otherwise
      public: virtual void ShowArrowHead(bool _b) = 0;

      /// \brief set true to show the arrow shaft, false otherwise
      /// \param[in] _b true to show the arrow shaft, false otherwise
      public: virtual void ShowArrowShaft(bool _b) = 0;

      /// \brief Set true to show the rotation of the arrow, false otherwise
      /// \param[in] _b True to show the arrow rotation.
      public: virtual void ShowArrowRotation(bool _b) = 0;
    };
    }
  }
}

#endif
