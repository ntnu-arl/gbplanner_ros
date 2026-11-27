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
#ifndef GZ_RENDERING_THERMALCAMERA_HH_
#define GZ_RENDERING_THERMALCAMERA_HH_

#include <string>
#include "gz/rendering/Camera.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    /* \class ThermalCamera ThermalCamera.hh \
      * ignition/rendering/ThermalCamera.hh
     */
    /// \brief Thermal camera that produces temperature data.
    /// An object's temperature can be set through the Visual class
    /// using SetUserData with the key "temperature". Ambient temperature
    /// is returned for object that do not have this property set. Note that
    /// temperature variations for the environment and objects can be set using
    /// the Set*Range functions in this class. For simplicity, the variations
    /// are modeled as a function of depth (which is not how a real thermal
    /// sensor works). Ideally we support heatmaps for visuals in order to
    /// produce more realistic temperature output.
    class IGNITION_RENDERING_VISIBLE ThermalCamera :
      public virtual Camera
    {
      /// \brief Destructor
      public: virtual ~ThermalCamera() { }

      /// \brief Set the ambient temperature of the environment
      /// \param[in] _ambient Ambient temperature in kelvin
      /// \sa AmbientTemperature
      public: virtual void SetAmbientTemperature(float _ambient) = 0;

      /// \brief Get the ambient temperature of the environment
      /// \return Ambient temperature in kelvin
      /// \sa SetAmbientTemperature
      public: virtual float AmbientTemperature() const = 0;

      /// \brief Set the range of ambient temperature
      /// \param[in] _range The ambient temperature ranges from
      /// (ambient - range/2) to (ambient + range/2).
      /// \sa AmbientTemperatureRange
      public: virtual void SetAmbientTemperatureRange(float _range) = 0;

      /// \brief Get the range of ambient temperature
      /// \return The ambient temperature range. Ambient temperature ranges
      /// from (ambient - range/2) to (ambient - range/2)
      /// \sa SetAmbientTemperatureRange
      public: virtual float AmbientTemperatureRange() const = 0;

      /// \brief  Set the minimum temperature the sensor can detect
      /// \param[in] _min Min temperature in kelvin
      /// \sa MinTemperature
      public: virtual void SetMinTemperature(float _min) = 0;

      /// \brief Get the minimum temperature the sensor can detect
      /// \return Min temperature in kelvin
      /// \sa SetMinTemperature
      public: virtual float MinTemperature() const = 0;

      /// \brief  Set the maximum temperature the sensor can detect
      /// \param[in] _max Max temperature in kelvin
      /// \sa MaxTemperature
      public: virtual void SetMaxTemperature(float _max) = 0;

      /// \brief Get the maximum temperature the sensor can detect
      /// \return Max temperature in kelvin
      /// \sa SetMaxTemperature
      public: virtual float MaxTemperature() const = 0;

      /// \brief Set the temperature linear resolution. The thermal image data
      /// returned will be temperature in kelvin / resolution.
      /// Typical values are 0.01 (10mK), 0.1 (100mK), or 0.04 to simulate
      /// 14 bit format.
      /// \param[in] _resolution Temperature linear resolution
      /// \sa LinearResolution
      public: virtual void SetLinearResolution(float _resolution) = 0;

      /// \brief Get the temperature linear resolution
      /// \return Temperature linear resolution
      /// \sa SetLinearResolution
      public: virtual float LinearResolution() const = 0;

      /// \brief Get the range of temperature variation for all heat sources.
      /// \return Temperature variation range
      /// \sa SetHeatSourceTemperatureRange
      public: virtual float HeatSourceTemperatureRange() const = 0;

      /// \brief Set the range of temperature variation for all heat sources.
      /// If set, the temperature reading of heat sources will vary within
      /// a temperature of +/- range/2 instead of returning a fixed uniform
      /// temperature for the entire heat source
      /// \param[in] _range Temperature variation range
      /// \sa HeatSourceTemperatureRange
      public: virtual void SetHeatSourceTemperatureRange(float _range) = 0;

      /// \brief Connect to the new thermal image event
      /// \param[in] _subscriber Subscriber callback function. The callback
      /// function arguments are: <thermal data, width, height, depth, format>
      /// \return Pointer to the new Connection. This must be kept in scope
      public: virtual gz::common::ConnectionPtr ConnectNewThermalFrame(
          std::function<void(const uint16_t *, unsigned int, unsigned int,
          unsigned int, const std::string &)>  _subscriber) = 0;
    };
  }
  }
}
#endif
