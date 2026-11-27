/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#ifndef IGNITION_RENDERING_LIDARVISUAL_HH_
#define IGNITION_RENDERING_LIDARVISUAL_HH_

#include <vector>
#include <ignition/common/Time.hh>
#include <ignition/math/Color.hh>
#include <ignition/math/Vector3.hh>
#include "ignition/rendering/config.hh"
#include "ignition/rendering/Visual.hh"
#include "ignition/rendering/Object.hh"
#include "ignition/rendering/RenderTypes.hh"
#include "ignition/rendering/Marker.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \brief Enum for LidarVisual types
    enum IGNITION_RENDERING_VISIBLE LidarVisualType
    {
      /// \brief No type
      LVT_NONE           = 0,

      /// \brief Ray line visual
      LVT_RAY_LINES     = 1,

      /// \brief Points visual
      LVT_POINTS         = 2,

      /// \brief Triangle strips visual
      LVT_TRIANGLE_STRIPS = 3
    };

    /// \class LidarVisual LidarVisual.hh ignition/rendering/LidarVisual
    /// \brief A LidarVisual geometry class. The visual appearance is based
    /// on the type specified.
    class IGNITION_RENDERING_VISIBLE LidarVisual :
      public virtual Visual
    {
      protected: LidarVisual();

      /// \brief Destructor
      public: virtual ~LidarVisual();

      /// \brief Clear the points of the lidar visual
      public: virtual void ClearPoints() = 0;

      /// \brief Update the Visual
      public: virtual void Update() = 0;

      /// \brief Set lidar points to be visualised
      /// \param[in] _points Vector of points representing distance of the ray
      public: virtual void SetPoints(const std::vector<double> &_points) = 0;

      /// \brief Set lidar points to be visualised
      /// \param[in] _points Vector of points representing distance of the ray
      /// \param[in] _colors Vector of colors for the rendered points
      public: virtual void SetPoints(const std::vector<double> &_points,
                        const std::vector<ignition::math::Color> &_colors) = 0;

      /// \brief Set minimum vertical angle
      /// \param[in] _minVerticalAngle Minimum vertical angle
      public: virtual void SetMinVerticalAngle(
                  double _minVerticalAngle) = 0;

      /// \brief Get minimum vertical angle
      /// \return The minimum vertical angle value of the lidar visual
      public: virtual double MinVerticalAngle() const = 0;

      /// \brief Set maximum vertical angle
      /// \param[in] _maxVerticalAngle Maximum vertical angle value
      public: virtual void SetMaxVerticalAngle(
                  double _maxVerticalAngle) = 0;

      /// \brief Get minimum vertical angle
      /// \return The maximum vertical angle value of the lidar visual
      public: virtual double MaxVerticalAngle() const = 0;

      /// \brief Set minimum horizontal angle
      /// \param[in] _minHorizontalAngle Minimum horizontal angle value
      public: virtual void SetMinHorizontalAngle(
                  double _minHorizontalAngle) = 0;

      /// \brief Get minimum horizontal angle
      /// \return The minimum horizontal angle value of the lidar visual
      public: virtual double MinHorizontalAngle() const = 0;

      /// \brief Set maximum horizontal angle
      /// \param[in] _maxHorizontalAngle Maximum horizontal angle value
      public: virtual void SetMaxHorizontalAngle(
                  double _maxHorizontalAngle) = 0;

      /// \brief Get maximum horizontal angle
      /// \return The maximum horizontal angle value of the lidar visual
      public: virtual double MaxHorizontalAngle() const = 0;

      /// \brief Set number of vertical rays
      /// \param[in] _verticalRayCount Number of vertical rays
      public: virtual void SetVerticalRayCount(
                  unsigned int _verticalRayCount) = 0;

      /// \brief Get number of vertical rays
      /// \return The number of vertical rays
      public: virtual unsigned int VerticalRayCount() const = 0;

      /// \brief Set number of horizontal rays
      /// \return _horizontalRayCount Number of vertical rays
      public: virtual void SetHorizontalRayCount(
                  unsigned int _horizontalRayCount) = 0;

      /// \brief Get number of horizontal rays
      /// \return The number of horizontal rays
      public: virtual unsigned int HorizontalRayCount() const = 0;

      /// \brief Set minimum range of lidar rays
      /// \param[in] _minRange The minimum range for lidar rays
      public: virtual void SetMinRange(double _minRange) = 0;

      /// \brief Get minimum range of lidar rays
      /// \return The minimum range for lidar rays
      public: virtual double MinRange() const = 0;

      /// \brief Set maximum range of lidar rays
      /// \param[in] _maxRange The maximum range for lidar rays
      public: virtual void SetMaxRange(double _maxRange) = 0;

      /// \brief Get maximum range of lidar rays
      /// \return The maximum range for lidar rays
      public: virtual double MaxRange() const = 0;

      /// \brief Set offset of visual
      /// \param[in] _offset The offset of the lidar rays from visual origin
      public: virtual void SetOffset(
                  const ignition::math::Pose3d _offset) = 0;

      /// \brief Get offset of visual
      /// \return The offset of the lidar rays from visual origin
      public: virtual ignition::math::Pose3d  Offset() const = 0;

      /// \brief Get number of points in laser data
      /// \return The number of points in the laser data
      public: virtual unsigned int PointCount() const = 0;

      /// \brief Get the points in laser data
      /// \return The points in the laser data
      public: virtual std::vector<double> Points() const = 0;

      /// \brief Set type for lidar visual
      /// \param[in] _type The type of visualisation for lidar data
      public: virtual void SetType(const LidarVisualType _type) = 0;

      /// \brief Get the type for lidar visual
      /// \return The type for lidar visual
      public: virtual LidarVisualType Type() const = 0;

      /// \brief Set size of the lidar visualization,
      /// e.g. size of rasterized lidar points in pixels
      /// \param[in] _size Size of the lidar visualization.
      public: virtual void SetSize(double _size) = 0;

      /// \brief Get size of the lidar visualization
      /// \return Size of the lidar visualization.
      /// \sa SetSize
      public: virtual double Size() const = 0;

      /// \brief Set if non-hitting rays will be displayed
      /// (this does not work for TRIANGLE_STRIPS visual)
      /// \param[in] _display Boolean value to display non hitting visuals
      public: virtual void SetDisplayNonHitting(bool _display) = 0;

      /// \brief Get if non-hitting rays will be displayed
      /// \return Boolean value if non-hitting rays will be displayed
      public: virtual bool DisplayNonHitting() const = 0;
    };
    }
  }
}
#endif
