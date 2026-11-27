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
#ifndef GZ_RENDERING_BASE_BASEGPURAYS_HH_
#define GZ_RENDERING_BASE_BASEGPURAYS_HH_

#include <string>

#include <gz/common/Event.hh>
#include <gz/common/Console.hh>

#include "gz/rendering/GpuRays.hh"
#include "gz/rendering/Image.hh"
#include "gz/rendering/RenderEngine.hh"
#include "gz/rendering/Scene.hh"
#include "gz/rendering/base/BaseRenderTarget.hh"
#include "gz/rendering/base/BaseCamera.hh"
#include "gz/rendering/Visual.hh"
#include "gz/rendering/RenderTypes.hh"


namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    template <class T>
    class BaseGpuRays :
      public virtual GpuRays,
      public virtual BaseCamera<T>,
      public virtual T
    {
      /// \brief Constructor
      protected: BaseGpuRays();

      /// \brief Destructor
      public: virtual ~BaseGpuRays();

      // Documentation inherited.
      public: virtual const float *Data() const override;

      // Documentation inherited.
      public: virtual void Copy(float *_data) override;

      // Documentation inherited.
      public: virtual void SetClamp(bool _enable) override;

      // Documentation inherited.
      public: virtual bool Clamp() const override;

      // Documentation inherited.
      public: virtual common::ConnectionPtr ConnectNewGpuRaysFrame(
                  std::function<void(const float *_frame, unsigned int _width,
                  unsigned int _height, unsigned int _depth,
                  const std::string &_format)> _subscriber) override;

      /// \brief Pointer to the render target
      public: virtual RenderTargetPtr RenderTarget() const override = 0;

      // Documentation inherited.
      public: virtual void SetIsHorizontal(const bool _horizontal) override;

      // Documentation inherited.
      public: virtual bool IsHorizontal() const override;

      /// \brief Set the vertical fov
      /// \param[in] _vfov vertical fov
      public: virtual void SetVFOV(const math::Angle &_vfov);

      // Documentation inherited.
      public: virtual math::Angle VFOV() const override;

      // Documentation inherited.
      public: virtual double RayCountRatio() const override;

      // Documentation inherited.
      public: virtual double RangeCountRatio() const override;

      // Documentation inherited.
      public: virtual void SetRayCountRatio(
                  const double _rayCountRatio) override;

      // Documentation inherited.
      public: virtual gz::math::Angle AngleMin() const override;

      // Documentation inherited.
      public: virtual void SetAngleMin(double _angle) override;

      // Documentation inherited.
      public: virtual gz::math::Angle AngleMax() const override;

      // Documentation inherited.
      public: virtual void SetAngleMax(double _angle) override;

      // Documentation inherited.
      public: virtual void SetVerticalRayCount(int _samples) override;

      // Documentation inherited.
      public: virtual void SetRayCount(int _samples) override;

      // Documentation inherited.
      public: virtual int RayCount() const override;

      // Documentation inherited.
      public: virtual int RangeCount() const override;

      // Documentation inherited.
      public: virtual int VerticalRayCount() const override;

      // Documentation inherited.
      public: virtual int VerticalRangeCount() const override;

      // Documentation inherited.
      public: virtual gz::math::Angle VerticalAngleMin() const override;

      // Documentation inherited.
      public: virtual void SetVerticalAngleMin(const double _angle) override;

      // Documentation inherited.
      public: virtual gz::math::Angle VerticalAngleMax() const override;

      // Documentation inherited.
      public: virtual void SetVerticalAngleMax(const double _angle) override;

      // Documentation inherited.
      public: virtual unsigned int Channels() const override;

      // Documentation inherited.
      public: virtual void SetHorizontalResolution(double _resolution) override;

      // Documentation inherited.
      public: virtual double HorizontalResolution() const override;

      // Documentation inherited.
      public: virtual void SetVerticalResolution(double resolution) override;

      // Documentation inherited.
      public: virtual double VerticalResolution() const override;

      /// \brief maximum value used for data outside sensor range
      public: float dataMaxVal = gz::math::INF_D;

      /// \brief minimum value used for data outside sensor range
      public: float dataMinVal = -gz::math::INF_D;

      /// \brief True if data values are clamped to camera clip distances,
      // false if data outside of camera range is +/- inf
      public: bool clamping = false;

      /// \brief Ray count ratio.
      protected: double rayCountRatio = 0;

      /// \brief Range count ratio.
      protected: double rangeCountRatio = 0;

      /// \brief Vertical field-of-view.
      protected: math::Angle vfov;

      /// \brief True if the sensor is horizontal only.
      protected: bool isHorizontal = true;

      /// \brief Horizontal minimal angle
      protected: double minAngle = 0;

      /// \brief Horizontal maximal angle
      protected: double maxAngle = 0;

      /// \brief Vertical minimal angle
      protected: double vMinAngle = 0;

      /// \brief Vertical maximal angle
      protected: double vMaxAngle = 0;

      /// \brief Quantity of horizontal rays
      protected: int hSamples = 0;

      /// \brief Quantity of verical rays
      protected: int vSamples = 0;

      /// \brief Resolution of horizontal rays
      protected: double hResolution = 1;

      /// \brief Resolution of vertical rays
      protected: double vResolution = 1;

      /// \brief Number of channels used to store the data
      protected: unsigned int channels = 1u;

      private: friend class OgreScene;
    };

    //////////////////////////////////////////////////
    template <class T>
    BaseGpuRays<T>::BaseGpuRays()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    BaseGpuRays<T>::~BaseGpuRays()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    const float *BaseGpuRays<T>::Data() const
    {
      return nullptr;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseGpuRays<T>::Copy(float *_dataDest)
    {
      // Unused
      (void)_dataDest;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseGpuRays<T>::SetClamp(bool _enable)
    {
      this->clamping = _enable;

      if (this->clamping)
      {
        this->dataMinVal = this->NearClipPlane();
        this->dataMaxVal = this->FarClipPlane();
      }
      else
      {
        this->dataMinVal = -gz::math::INF_D;
        this->dataMaxVal = gz::math::INF_D;
      }
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseGpuRays<T>::Clamp() const

    {
      return this->clamping;
    }

    //////////////////////////////////////////////////
    template <class T>
    gz::common::ConnectionPtr BaseGpuRays<T>::ConnectNewGpuRaysFrame(
          std::function<void(const float *, unsigned int, unsigned int,
          unsigned int, const std::string &)>)
    {
      return nullptr;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseGpuRays<T>::SetIsHorizontal(const bool _horizontal)
    {
      this->isHorizontal = _horizontal;
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseGpuRays<T>::IsHorizontal() const
    {
      return this->isHorizontal;
    }

    //////////////////////////////////////////////////
    template <class T>
    double BaseGpuRays<T>::RayCountRatio() const
    {
      return this->rayCountRatio;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseGpuRays<T>::SetRayCountRatio(const double _rayCountRatio)
    {
      this->rayCountRatio = _rayCountRatio;
    }

    //////////////////////////////////////////////////
    template <class T>
    double BaseGpuRays<T>::RangeCountRatio() const
    {
      return this->rangeCountRatio;
    }

    //////////////////////////////////////////////////
    template <class T>
    math::Angle BaseGpuRays<T>::VFOV() const
    {
      return this->vfov;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseGpuRays<T>::SetVFOV(const math::Angle &_vfov)
    {
      this->vfov = _vfov;
    }

    template <class T>
    //////////////////////////////////////////////////
    gz::math::Angle BaseGpuRays<T>::AngleMin() const
    {
      return this->minAngle;
    }

    template <class T>
    //////////////////////////////////////////////////
    void BaseGpuRays<T>::SetAngleMin(double _angle)
    {
      this->minAngle = _angle;
    }

    template <class T>
    //////////////////////////////////////////////////
    gz::math::Angle BaseGpuRays<T>::AngleMax() const
    {
      return this->maxAngle;
    }

    template <class T>
    //////////////////////////////////////////////////
    void BaseGpuRays<T>::SetAngleMax(double _angle)
    {
      this->maxAngle = _angle;
    }

    template <class T>
    //////////////////////////////////////////////////
    int BaseGpuRays<T>::RayCount() const
    {
      return this->hSamples;
    }

    template <class T>
    //////////////////////////////////////////////////
    void BaseGpuRays<T>::SetRayCount(int _samples)
    {
      this->hSamples = _samples;
    }

    template <class T>
    //////////////////////////////////////////////////
    int BaseGpuRays<T>::RangeCount() const
    {
      return static_cast<int>(this->RayCount() * this->hResolution);
    }

    template <class T>
    //////////////////////////////////////////////////
    int BaseGpuRays<T>::VerticalRayCount() const
    {
      return this->vSamples;
    }

    template <class T>
    //////////////////////////////////////////////////
    void BaseGpuRays<T>::SetVerticalRayCount(int _samples)
    {
      this->vSamples = _samples;
    }

    template <class T>
    //////////////////////////////////////////////////
    int BaseGpuRays<T>::VerticalRangeCount() const
    {
      return static_cast<int>(this->VerticalRayCount() * this->vResolution);
    }

    template <class T>
    //////////////////////////////////////////////////
    gz::math::Angle BaseGpuRays<T>::VerticalAngleMin() const
    {
      return this->vMinAngle;
    }

    template <class T>
    //////////////////////////////////////////////////
    void BaseGpuRays<T>::SetVerticalAngleMin(const double _angle)
    {
        this->vMinAngle = _angle;
    }

    template <class T>
    //////////////////////////////////////////////////
    gz::math::Angle BaseGpuRays<T>::VerticalAngleMax() const
    {
      return this->vMaxAngle;
    }

    template <class T>
    //////////////////////////////////////////////////
    void BaseGpuRays<T>::SetVerticalAngleMax(const double _angle)
    {
        this->vMaxAngle = _angle;
    }

    template <class T>
    //////////////////////////////////////////////////
    unsigned int BaseGpuRays<T>::Channels() const
    {
      return this->channels;
    }

    template <class T>
    //////////////////////////////////////////////////
    void BaseGpuRays<T>::SetHorizontalResolution(double _resolution)
    {
      this->hResolution = std::abs(_resolution);
    }

    template <class T>
    //////////////////////////////////////////////////
    double BaseGpuRays<T>::HorizontalResolution() const
    {
      return this->hResolution;
    }

    template <class T>
    //////////////////////////////////////////////////
    void BaseGpuRays<T>::SetVerticalResolution(double _resolution)
    {
      this->vResolution = std::abs(_resolution);
    }

    template <class T>
    //////////////////////////////////////////////////
    double BaseGpuRays<T>::VerticalResolution() const
    {
      return this->vResolution;
    }
    }
  }
}
#endif
