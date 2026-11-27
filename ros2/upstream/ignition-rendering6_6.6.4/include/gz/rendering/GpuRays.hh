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
#ifndef GZ_RENDERING_GPURAYS_HH_
#define GZ_RENDERING_GPURAYS_HH_

#include <string>

#include <gz/common/Event.hh>

#include "gz/rendering/Image.hh"
#include "gz/rendering/Sensor.hh"
#include "gz/rendering/Scene.hh"
#include "gz/rendering/Camera.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \class GpuRays GpuRays.hh ignition/rendering/GpuRays.hh
    /// \brief Generate depth ray data.
    class IGNITION_RENDERING_VISIBLE GpuRays :
      public virtual Camera
    {
      /// \brief Callback function for new frame render event listeners
      public: typedef std::function<void(const void*, unsigned int,
          unsigned int, unsigned int, const std::string&)> NewFrameListener;

      /// \brief Destructor
      public: virtual ~GpuRays() { }

      /// \brief All things needed to get back z buffer for gpu rays data.
      /// \return Array of gpu rays data.
      public: virtual const float *Data() const = 0;

      /// \brief Copy to the specified memory direction the gpu rays data.
      public: virtual void Copy(float *_data) = 0;

      /// \brief Configure behaviour for data values outside of camera range
      /// \param[in] _clamp True to clamp data to camera clip distances,
      // false to leave data values as +/-inf when out of camera range
      public: virtual void SetClamp(const bool _clamp) = 0;

      /// \brief Get behaviour for data values outside of camera range
      /// \return True if data values are clampped to camera clip distances,
      // false if data values outside of camera range are returned as +/-inf
      public: virtual bool Clamp() const = 0;

      /// \brief Connect to a gpu rays frame signal
      /// \param[in] _subscriber Callback that is called when a new image is
      /// generated. The callback function parameters are:
      ///   _frame:   Image frame is an array of floats. Size is equal
      ///             to width * height * channels
      ///             Each gpu rays reading occupies 3 floats
      ///             Index 0: depth value
      ///             Index 1: retro value
      ///             Index 2: 0. Not used
      ///   _width:   Width of image, i.e. number of data in the horizonal scan
      ///   _height:  Height o image, i.e. number of scans in vertical direction
      ///   _channels: Number of channels, i.e. 3 floats per gpu rays reading
      ///   _format:  Pixel format of the image frame.
      /// \return A pointer to the connection. This must be kept in scope.
      public: virtual common::ConnectionPtr ConnectNewGpuRaysFrame(
                  std::function<void(const float *_frame, unsigned int _width,
                  unsigned int _height, unsigned int _depth,
                  const std::string &)> _subscriber) = 0;

      /// \brief Set sensor horizontal or vertical
      /// \param[in] _horizontal True if horizontal, false if not
      public: virtual void SetIsHorizontal(const bool _horizontal) = 0;

      /// \brief Gets if sensor is horizontal
      /// \return True if horizontal, false if not
      public: virtual bool IsHorizontal() const = 0;

      /// \brief Get the vertical field-of-view.
      /// \return The vertical field of view of the gpu rays.
      public: virtual math::Angle VFOV() const = 0;

      /// \brief Get the ray count ratio (equivalent to aspect ratio)
      /// \return The ray count ratio (equivalent to aspect ratio)
      public: virtual double RayCountRatio() const = 0;

      /// \brief Get the ray count ratio (equivalent to aspect ratio)
      /// \return The ray count ratio (equivalent to aspect ratio)
      public: virtual double RangeCountRatio() const = 0;

      /// \brief Sets the ray count ratio (equivalent to aspect ratio)
      /// \param[in] _rayCountRatio ray count ratio (equivalent to aspect ratio)
      public: virtual void SetRayCountRatio(const double _rayCountRatio) = 0;

      /// \brief Get minimal horizontal angle value
      // \return minimal horizontal angle value
      public: virtual gz::math::Angle AngleMin() const = 0;

      /// \brief Set minimal horizontal angle value
      public: virtual void SetAngleMin(double _angle) = 0;

      /// \brief Get maximal horizontal angle value
      // \return maximal horizontal angle value
      public: virtual gz::math::Angle AngleMax() const = 0;

      /// \brief Set maximal horizontal angle value
      public: virtual void SetAngleMax(double _angle) = 0;

      /// \brief Get horizontal quantity of rays
      // \return horizontal quantity of rays
      public: virtual int RayCount() const = 0;

      /// \brief Set horizontal quantity of rays
      public: virtual void SetRayCount(int _samples) = 0;

      /// \brief Get hoizontal range count, i.e. ray count * horz resolution
      // \return horizontal range count
      public: virtual int RangeCount() const = 0;

      /// \brief Get vertical quantity of rays
      // \return vertical quantity of rays
      public: virtual int VerticalRayCount() const = 0;

      /// \brief Set vertical quantity of rays
      public: virtual void SetVerticalRayCount(int _samples) = 0;

      /// \brief Get vertical range count, i.e. ray count * vert resolution
      // \return Vertical range count
      public: virtual int VerticalRangeCount() const = 0;

      /// \brief Get minimal vertical angle value
      // \return minimal vertical angle value
      public: virtual gz::math::Angle VerticalAngleMin() const = 0;

      /// \brief Set minimal vertical angle value
      public: virtual void SetVerticalAngleMin(const double _angle) = 0;

      /// \brief Get maximal vertical angle value
      // \return maximal vertical angle value
      public: virtual gz::math::Angle VerticalAngleMax() const = 0;

      /// \brief Set maximal vertical angle value
      // \return minimal vertical angle value
      public: virtual void SetVerticalAngleMax(const double _angle) = 0;

      /// \brief Get the number of channels used to store the ray data.
      /// \return Channel count.
      public: virtual unsigned int Channels() const = 0;

      /// \brief Set the horizontal resolution. This number is multiplied by
      /// RayCount to calculate RangeCount, which is the the number range data
      /// points.
      /// \sa RayCount()
      /// \param[in] _resolution The new horizontal resolution. The
      /// absolute value of this parameter is used to prevent a
      /// negative resolution value.
      public: virtual void SetHorizontalResolution(double _resolution) = 0;

      /// \brief Get the horizontal resolution. This number is multiplied by
      /// RayCount to calculate RangeCount, which is the the number range data
      /// points.
      /// \sa RayCount()
      /// \return The horizontal resolution
      public: virtual double HorizontalResolution() const = 0;

      /// \brief Set the vertical resolution. This number is multiplied by
      /// VerticalRayCount to calculate VerticalRangeCount, which is the the
      /// number vertical range data points.
      /// \param[in] _resolution The new vertical resolution. The
      /// absolute value of this parameter is used to prevent a
      /// negative resolution value.
      /// \sa VerticalRayCount()
      public: virtual void SetVerticalResolution(double _resolution) = 0;

      /// \brief Get the vertical resolution. This number is multiplied by
      /// VerticalRayCount to calculate VerticalRangeCount, which is the the
      /// number vertical range data points.
      /// \return The vertical resolution.
      /// \sa VerticalRayCount()
      public: virtual double VerticalResolution() const = 0;
    };
  }
  }
}
#endif
