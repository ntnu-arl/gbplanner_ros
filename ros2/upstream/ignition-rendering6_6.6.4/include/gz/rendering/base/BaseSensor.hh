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
#ifndef GZ_RENDERING_BASE_BASESENSOR_HH_
#define GZ_RENDERING_BASE_BASESENSOR_HH_

#include "gz/rendering/Sensor.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    template <class T>
    class BaseSensor :
      public virtual Sensor,
      public virtual T
    {
      protected: BaseSensor();

      public: virtual ~BaseSensor();

      // Documentation inherited.
      public: virtual void SetVisibilityMask(uint32_t _mask) override;

      // Documentation inherited.
      public: virtual uint32_t VisibilityMask() const override;

      /// \brief Camera's visibility mask
      protected: uint32_t visibilityMask = IGN_VISIBILITY_ALL;
    };

    //////////////////////////////////////////////////
    template <class T>
    BaseSensor<T>::BaseSensor()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    BaseSensor<T>::~BaseSensor()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseSensor<T>::SetVisibilityMask(uint32_t _mask)
    {
      this->visibilityMask = _mask;
    }

    //////////////////////////////////////////////////
    template <class T>
    uint32_t BaseSensor<T>::VisibilityMask() const
    {
      return this->visibilityMask;
    }
    }
  }
}
#endif
