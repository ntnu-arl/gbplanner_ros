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
#ifndef GZ_RENDERING_BASEMARKER_HH_
#define GZ_RENDERING_BASEMARKER_HH_

#include <gz/common/SuppressWarning.hh>

#include "gz/rendering/Marker.hh"
#include "gz/rendering/base/BaseObject.hh"
#include "gz/rendering/base/BaseRenderTypes.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    /// \brief Base implementation of a Marker geometry
    template <class T>
    class BaseMarker :
      public virtual Marker,
      public virtual T
    {
      /// \brief Constructor.
      protected: BaseMarker();

      /// \brief Destructor.
      public: virtual ~BaseMarker();

      /// \brief PreRender function
      public: virtual void PreRender() override;

      /// \brief Destroy function
      public: virtual void Destroy() override;

      // Documentation inherited
      public: virtual void SetLifetime(const
                  std::chrono::steady_clock::duration &_lifetime) override;

      // Documentation inherited
      public: virtual std::chrono::steady_clock::duration Lifetime()
                  const override;

      // Documentation inherited
      public: virtual void SetType(const MarkerType _markerType) override;

      // Documentation inherited
      public: virtual MarkerType Type() const override;

      // Documentation inherited
      public: virtual void SetSize(double _size) override;

      // Documentation inherited
      public: virtual double Size() const override;

      // Documentation inherited
      public: virtual void SetLayer(int32_t _layer) override;

      // Documentation inherited
      public: virtual int32_t Layer() const override;

      // Documentation inherited
      public: virtual void ClearPoints() override;

      // Documentation inherited
      public: virtual void AddPoint(double _x,
                  double _y, double _z,
                  const gz::math::Color &_color) override;

      // Documentation inherited
      public: virtual void AddPoint(const gz::math::Vector3d &_pt,
                  const gz::math::Color &_color) override;

      // Documentation inherited
      public: virtual void SetPoint(unsigned int _index,
                  const gz::math::Vector3d &_value) override;

      /// \brief Life time of a marker
      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      protected: std::chrono::steady_clock::duration lifetime =
          std::chrono::steady_clock::duration::zero();
      IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING

      /// \brief Layer at which the marker will reside
      protected: int32_t layer = 0;

      /// \brief Flag to indicate if marker needs to be updated
      protected: bool markerDirty = false;

      /// \brief Marker type
      protected: MarkerType markerType =
          gz::rendering::MarkerType::MT_NONE;

      /// \brief Marker size
      protected: double size = 1.0;
    };

    /////////////////////////////////////////////////
    // BaseMarker
    /////////////////////////////////////////////////
    template <class T>
    BaseMarker<T>::BaseMarker()
    {
    }

    /////////////////////////////////////////////////
    template <class T>
    BaseMarker<T>::~BaseMarker()
    {
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseMarker<T>::SetLifetime(
        const std::chrono::steady_clock::duration &_lifetime)
    {
      this->lifetime = _lifetime;
      this->markerDirty = true;
    }

    /////////////////////////////////////////////////
    template <class T>
    std::chrono::steady_clock::duration BaseMarker<T>::Lifetime() const
    {
      return this->lifetime;
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseMarker<T>::SetLayer(int32_t _layer)
    {
      this->layer = _layer;
      this->markerDirty = true;
    }

    /////////////////////////////////////////////////
    template <class T>
    int32_t BaseMarker<T>::Layer() const
    {
      return this->layer;
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseMarker<T>::SetType(const MarkerType _markerType)
    {
      this->markerType = _markerType;
      this->markerDirty = true;
    }

    /////////////////////////////////////////////////
    template <class T>
    MarkerType BaseMarker<T>::Type() const
    {
      return this->markerType;
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseMarker<T>::SetSize(double _size)
    {
      this->size = _size;
      this->markerDirty = true;
    }

    /////////////////////////////////////////////////
    template <class T>
    double BaseMarker<T>::Size() const
    {
      return this->size;
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseMarker<T>::PreRender()
    {
      T::PreRender();
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseMarker<T>::Destroy()
    {
      T::Destroy();
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseMarker<T>::ClearPoints()
    {
        // no op
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseMarker<T>::AddPoint(const gz::math::Vector3d &,
                                 const gz::math::Color &)
    {
        // no op
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseMarker<T>::AddPoint(double _x, double _y, double _z,
                  const gz::math::Color &_color)
    {
      this->AddPoint(gz::math::Vector3d(_x, _y, _z), _color);
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseMarker<T>::SetPoint(unsigned int,
                  const gz::math::Vector3d &)
    {
      // no op
    }
    }
  }
}
#endif
