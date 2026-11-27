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
#ifndef GZ_RENDERING_BASE_BASEDEPTHCAMERA_HH_
#define GZ_RENDERING_BASE_BASEDEPTHCAMERA_HH_

#include <string>

#include <gz/common/Event.hh>

#include "gz/rendering/base/BaseCamera.hh"
#include "gz/rendering/DepthCamera.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    template <class T>
    class BaseDepthCamera :
      public virtual DepthCamera,
      public virtual BaseCamera<T>,
      public virtual T
    {
      protected: BaseDepthCamera();

      public: virtual ~BaseDepthCamera();

      public: virtual void CreateDepthTexture();

      public: virtual const float *DepthData() const;

      public: virtual gz::common::ConnectionPtr ConnectNewDepthFrame(
          std::function<void(const float *, unsigned int, unsigned int,
          unsigned int, const std::string &)>  _subscriber);

      public: virtual gz::common::ConnectionPtr ConnectNewRGBPointCloud(
          std::function<void(const float *, unsigned int, unsigned int,
          unsigned int, const std::string &)>  _subscriber);
    };

    //////////////////////////////////////////////////
    template <class T>
    BaseDepthCamera<T>::BaseDepthCamera()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    BaseDepthCamera<T>::~BaseDepthCamera()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseDepthCamera<T>::CreateDepthTexture()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    const float *BaseDepthCamera<T>::DepthData() const
    {
      return nullptr;
    }

    //////////////////////////////////////////////////
    template <class T>
    gz::common::ConnectionPtr BaseDepthCamera<T>::ConnectNewDepthFrame(
          std::function<void(const float *, unsigned int, unsigned int,
          unsigned int, const std::string &)>)
    {
      return nullptr;
    }

    //////////////////////////////////////////////////
    template <class T>
    gz::common::ConnectionPtr BaseDepthCamera<T>::ConnectNewRGBPointCloud(
          std::function<void(const float *, unsigned int, unsigned int,
          unsigned int, const std::string &)>)
    {
      return nullptr;
    }
  }
  }
}
#endif
