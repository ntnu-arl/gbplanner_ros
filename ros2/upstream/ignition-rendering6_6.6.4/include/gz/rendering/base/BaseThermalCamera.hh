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
#ifndef GZ_RENDERING_BASE_BASETHERMALCAMERA_HH_
#define GZ_RENDERING_BASE_BASETHERMALCAMERA_HH_

#include <string>

#include "gz/rendering/base/BaseCamera.hh"
#include "gz/rendering/ThermalCamera.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \brief Base implementation of the ThermalCamera class
    template <class T>
    class BaseThermalCamera :
      public virtual ThermalCamera,
      public virtual BaseCamera<T>,
      public virtual T
    {
      /// \brief Constructor
      protected: BaseThermalCamera();

      /// \brief Destructor
      public: virtual ~BaseThermalCamera();

      // Documentation inherited.
      public: virtual void SetAmbientTemperature(float _ambient) override;

      // Documentation inherited.
      public: virtual float AmbientTemperature() const override;

      // Documentation inherited.
      public: virtual void SetAmbientTemperatureRange(float _range) override;

      // Documentation inherited.
      public: virtual float AmbientTemperatureRange() const override;

      // Documentation inherited.
      public: virtual void SetMinTemperature(float _min) override;

      // Documentation inherited.
      public: virtual float MinTemperature() const override;

      // Documentation inherited.
      public: virtual void SetMaxTemperature(float _max) override;

      // Documentation inherited.
      public: virtual float MaxTemperature() const override;

      // Documentation inherited.
      public: virtual void SetLinearResolution(float _max) override;

      // Documentation inherited.
      public: virtual float LinearResolution() const override;

      // Documentation inherited.
      public: virtual float HeatSourceTemperatureRange() const override;

      // Documentation inherited.
      public: virtual void SetHeatSourceTemperatureRange(float _range) override;

      // Documentation inherted.
      public: virtual gz::common::ConnectionPtr ConnectNewThermalFrame(
          std::function<void(const uint16_t *, unsigned int, unsigned int,
          unsigned int, const std::string &)>  _subscriber) override;

      /// \brief Ambient temperature of the environment
      protected: float ambient = 0.0f;

      /// \brief Ambient temperature range
      protected: float ambientRange = 0.0f;

      /// \brief Minimum temperature
      protected: float minTemp = -gz::math::INF_F;

      /// \brief Maximum temperature
      protected: float maxTemp = gz::math::INF_F;

      /// \brief Linear resolution. Defaults to 10mK.
      protected: float resolution = 0.01f;

      /// \brief Range of heat source temperature variation
      protected: float heatSourceTempRange = 0.0f;
    };

    //////////////////////////////////////////////////
    template <class T>
    BaseThermalCamera<T>::BaseThermalCamera()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    BaseThermalCamera<T>::~BaseThermalCamera()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseThermalCamera<T>::SetAmbientTemperature(float _ambient)
    {
      this->ambient = _ambient;
    }

    //////////////////////////////////////////////////
    template <class T>
    float BaseThermalCamera<T>::AmbientTemperature() const
    {
      return this->ambient;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseThermalCamera<T>::SetAmbientTemperatureRange(float _range)
    {
      this->ambientRange = _range;
    }

    //////////////////////////////////////////////////
    template <class T>
    float BaseThermalCamera<T>::AmbientTemperatureRange() const
    {
      return this->ambientRange;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseThermalCamera<T>::SetMinTemperature(float _min)
    {
      this->minTemp = _min;
    }

    //////////////////////////////////////////////////
    template <class T>
    float BaseThermalCamera<T>::MinTemperature() const
    {
      return this->minTemp;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseThermalCamera<T>::SetMaxTemperature(float _max)
    {
      this->maxTemp = _max;
    }

    //////////////////////////////////////////////////
    template <class T>
    float BaseThermalCamera<T>::MaxTemperature() const
    {
      return this->maxTemp;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseThermalCamera<T>::SetLinearResolution(float _resolution)
    {
      this->resolution =  _resolution;
    }

    //////////////////////////////////////////////////
    template <class T>
    float BaseThermalCamera<T>::LinearResolution() const
    {
      return this->resolution;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseThermalCamera<T>::SetHeatSourceTemperatureRange(float _range)
    {
      this->heatSourceTempRange =  _range;
    }

    //////////////////////////////////////////////////
    template <class T>
    float BaseThermalCamera<T>::HeatSourceTemperatureRange() const
    {
      return this->heatSourceTempRange;
    }

    //////////////////////////////////////////////////
    template <class T>
    common::ConnectionPtr BaseThermalCamera<T>::ConnectNewThermalFrame(
          std::function<void(const uint16_t *, unsigned int, unsigned int,
          unsigned int, const std::string &)>)
    {
      return nullptr;
    }
  }
  }
}
#endif
