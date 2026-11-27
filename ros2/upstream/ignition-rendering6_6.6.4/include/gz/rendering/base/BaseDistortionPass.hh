/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#ifndef IGNITION_RENDERING_BASEDISTORTIONPASS_HH_
#define IGNITION_RENDERING_BASEDISTORTIONPASS_HH_

#include "ignition/rendering/DistortionPass.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /* \class BaseDistortionPass BaseDistortionPass.hh \
     * ignition/rendering/base/BaseDistortionPass.hh
     */
    /// \brief Base distortion render pass
    template <class T>
    class BaseDistortionPass :
      public virtual DistortionPass,
      public virtual T
    {
      /// \brief Constructor
      protected: BaseDistortionPass();

      /// \brief Destructor
      public: virtual ~BaseDistortionPass();

      // Documentation inherited.
      public: double K1() const;

      // Documentation inherited.
      public: double K2() const;

      // Documentation inherited.
      public: double K3() const;

      // Documentation inherited.
      public: double P1() const;

      // Documentation inherited.
      public: double P2() const;

      // Documentation inherited.
      public: math::Vector2d Center() const;

      // Documentation inherited.
      public: void SetK1(double _k1);

      // Documentation inherited.
      public: void SetK2(double _k2);

      // Documentation inherited.
      public: void SetK3(double _k3);

      // Documentation inherited.
      public: void SetP1(double _p1);

      // Documentation inherited.
      public: void SetP2(double _p2);

      // Documentation inherited.
      public: void SetCenter(const math::Vector2d &_center);

      /// \brief Radial distortion coefficient k1.
      public: double k1 = 0;

      /// \brief Radial distortion coefficient k2.
      public: double k2 = 0;

      /// \brief Radial distortion coefficient k3.
      public: double k3 = 0;

      /// \brief Tangential distortion coefficient p1.
      public: double p1 = 0;

      /// \brief Tangential distortion coefficient p2.
      public: double p2 = 0;

      /// \brief Lens center used for distortion
      public: math::Vector2d lensCenter = {0.5, 0.5};
    };

    //////////////////////////////////////////////////
    // BaseDistortionPass
    //////////////////////////////////////////////////
    template <class T>
    BaseDistortionPass<T>::BaseDistortionPass()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    BaseDistortionPass<T>::~BaseDistortionPass()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    double BaseDistortionPass<T>::K1() const
    {
      return this->k1;
    }

    //////////////////////////////////////////////////
    template <class T>
    double BaseDistortionPass<T>::K2() const
    {
      return this->k2;
    }

    //////////////////////////////////////////////////
    template <class T>
    double BaseDistortionPass<T>::K3() const
    {
      return this->k3;
    }

    //////////////////////////////////////////////////
    template <class T>
    double BaseDistortionPass<T>::P1() const
    {
      return this->p1;
    }

    //////////////////////////////////////////////////
    template <class T>
    double BaseDistortionPass<T>::P2() const
    {
      return this->p2;
    }

    //////////////////////////////////////////////////
    template <class T>
    math::Vector2d BaseDistortionPass<T>::Center() const
    {
      return this->lensCenter;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseDistortionPass<T>::SetK1(double _k1)
    {
      this->k1 = _k1;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseDistortionPass<T>::SetK2(double _k2)
    {
      this->k2 = _k2;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseDistortionPass<T>::SetK3(double _k3)
    {
      this->k3 = _k3;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseDistortionPass<T>::SetP1(double _p1)
    {
      this->p1 = _p1;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseDistortionPass<T>::SetP2(double _p2)
    {
      this->p2 = _p2;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseDistortionPass<T>::SetCenter(const math::Vector2d &_center)
    {
      this->lensCenter = _center;
    }
    }
  }
}
#endif
