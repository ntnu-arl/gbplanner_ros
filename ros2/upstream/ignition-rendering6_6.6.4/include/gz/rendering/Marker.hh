/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#ifndef GZ_RENDERING_MARKER_HH_
#define GZ_RENDERING_MARKER_HH_

#include <gz/common/Time.hh>
#include <gz/math/Color.hh>
#include <gz/math/Vector3.hh>
#include "gz/rendering/config.hh"
#include "gz/rendering/Geometry.hh"
#include "gz/rendering/Object.hh"
#include "gz/rendering/RenderTypes.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \brief Enum for marker types
    enum IGNITION_RENDERING_VISIBLE MarkerType
    {
      /// \brief No type
      MT_NONE           = 0,

      /// \brief Box geometry
      MT_BOX            = 1,

      /// \brief Cylinder geometry
      MT_CYLINDER       = 2,

      /// \brief Line strip primitive
      MT_LINE_STRIP     = 3,

      /// \brief Line list primitive
      MT_LINE_LIST      = 4,

      /// \brief Points primitive
      MT_POINTS         = 5,

      /// \brief Sphere geometry
      MT_SPHERE         = 6,

      /// \brief Text geometry
      MT_TEXT           = 7,

      /// \brief Triangle fan primitive
      MT_TRIANGLE_FAN   = 8,

      /// \brief Triangle list primitive
      MT_TRIANGLE_LIST  = 9,

      /// \brief Triangle strip primitive
      MT_TRIANGLE_STRIP = 10,

      /// \brief Capsule geometry
      MT_CAPSULE        = 11,
    };

    /// \class Marker Marker.hh ignition/rendering/Marker
    /// \brief A marker geometry class. The marker's visual appearance is based
    /// on the marker type specified.
    class IGNITION_RENDERING_VISIBLE Marker :
      public virtual Geometry
    {
      protected: Marker();

      /// \brief Destructor
      public: virtual ~Marker();

      /// \brief Set the lifetime of this Marker
      /// \param[in] _lifetime The time at which the marker will be removed
      public: virtual void SetLifetime(
                  const std::chrono::steady_clock::duration &_lifetime) = 0;

      /// \brief Get the lifetime of this Marker
      /// \return The time at which the marker will be removed
      public: virtual std::chrono::steady_clock::duration Lifetime() const = 0;

      /// \brief Set the layer of this Marker
      /// \param[in] _layer Layer at which the marker will reside
      public: virtual void SetLayer(int32_t _layer) = 0;

      /// \brief Get the layer of this Marker
      /// \return The layer of the marker
      public: virtual int32_t Layer() const = 0;

      /// \brief Set the render type of this Marker
      /// \param[in] _markerType The desired render type
      public: virtual void SetType(
                  const gz::rendering::MarkerType _markerType) = 0;

      /// \brief Get the render type of this Marker
      /// \return The render type of the marker
      public: virtual gz::rendering::MarkerType Type() const = 0;

      /// \brief Set size of the marker. Only affects MT_POINTS.
      /// e.g. size of rasterized points in pixels
      /// \param[in] _size Size of the marker
      public: virtual void SetSize(double _size) = 0;

      /// \brief Get the size of the marker.
      /// \return The size of the marker
      /// \sa SetSize
      public: virtual double Size() const = 0;

      /// \brief Clear the points of the marker, if applicable
      public: virtual void ClearPoints() = 0;

      /// \brief Add a point with its respective color to the marker
      /// \param[in] _x X coordinate
      /// \param[in] _y Y coordinate
      /// \param[in] _z Z coordinate
      /// \param[in] _color The color the point is set to
      public: virtual void AddPoint(double _x,
                  double _y, double _z,
                  const gz::math::Color &_color) = 0;

      /// \brief Add a point with its respective color to the marker
      /// \param[in] _pt A vector containing the position of the point
      /// \param[in] _color The color the point is set to
      public: virtual void AddPoint(const gz::math::Vector3d &_pt,
                  const gz::math::Color &_color) = 0;

      /// \brief Set an existing point's vector
      /// \param[in] _index The index of the point
      /// \param[in] _value The new positional vector of the point
      public: virtual void SetPoint(unsigned int _index,
                  const gz::math::Vector3d &_value) = 0;
    };
    }
  }
}

#endif
