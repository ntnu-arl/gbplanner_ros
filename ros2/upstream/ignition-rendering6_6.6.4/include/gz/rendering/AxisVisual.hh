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
#ifndef GZ_RENDERING_AXISVISUAL_HH_
#define GZ_RENDERING_AXISVISUAL_HH_

#include "gz/rendering/config.hh"
#include "gz/rendering/CompositeVisual.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \class AxisVisual AxisVisual.hh ignition/rendering/AxisVisual.hh
    /// \brief Represents a axis composite visual
    class IGNITION_RENDERING_VISIBLE AxisVisual :
      public virtual CompositeVisual
    {
      /// \brief Deconstructor
      public: virtual ~AxisVisual() { }

      /// \brief set true to show the axis heads, false otherwise
      /// \param[in] _b true to show the axis heads, false otherwise
      public: virtual void ShowAxisHead(bool _b) = 0;

      /// \brief set true to show the specified axis head, false otherwise
      /// \param[in] _axis Axis index. 0: x, 1: y, 2: z
      /// \param[in] _b true to show the specified axis head, false otherwise
      public: virtual void ShowAxisHead(unsigned int _axis, bool _b) = 0;
    };
    }
  }
}
#endif
