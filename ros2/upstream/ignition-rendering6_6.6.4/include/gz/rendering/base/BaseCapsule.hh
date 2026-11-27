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

#ifndef IGNITION_RENDERING_BASECAPSULE_HH_
#define IGNITION_RENDERING_BASECAPSULE_HH_

#include <ignition/common/Console.hh>

#include "ignition/rendering/Capsule.hh"
#include "ignition/rendering/Scene.hh"
#include "ignition/rendering/base/BaseObject.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    /// \brief Base implementation of a Capsule Geometry
    template <class T>
    class BaseCapsule :
      public virtual Capsule,
      public virtual T
    {
      // Documentation inherited
      protected: BaseCapsule();

      // Documentation inherited
      public: virtual ~BaseCapsule();

      // Documentation inherited
      public: virtual void SetRadius(double _radius) override;

      // Documentation inherited
      public: virtual void SetLength(double _length) override;

      // Documentation inherited
      public: virtual double Radius() const override;

      // Documentation inherited
      public: virtual double Length() const override;

      // Documentation inherited
      public: virtual GeometryPtr Clone() const override;

      /// \brief Radius of the capsule
      protected: double radius = 0.5;

      /// \brief Length of the capsule
      protected: double length = 0.5;

      /// \brief Flag to indicate capsule properties have changed
      protected: bool capsuleDirty = false;
    };

    /////////////////////////////////////////////////
    // BaseCapsule
    /////////////////////////////////////////////////
    template <class T>
    BaseCapsule<T>::BaseCapsule()
    {
    }

    /////////////////////////////////////////////////
    template <class T>
    BaseCapsule<T>::~BaseCapsule()
    {
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseCapsule<T>::SetRadius(double _radius)
    {
      this->radius = _radius;
      this->capsuleDirty = true;
    }

    /////////////////////////////////////////////////
    template <class T>
    double BaseCapsule<T>::Radius() const
    {
      return this->radius;
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseCapsule<T>::SetLength(double _length)
    {
      this->length = _length;
      this->capsuleDirty = true;
    }

    /////////////////////////////////////////////////
    template <class T>
    double BaseCapsule<T>::Length() const
    {
      return this->length;
    }

    /////////////////////////////////////////////////
    template <class T>
    GeometryPtr BaseCapsule<T>::Clone() const
    {
      if (!this->Scene())
      {
        ignerr << "Cloning a Capsule failed because the capsule to be "
          << "cloned does not belong to a scene.\n";
        return nullptr;
      }

      auto result = this->Scene()->CreateCapsule();
      if (result)
      {
        result->SetRadius(this->Radius());
        result->SetLength(this->Length());

        if (this->Material())
          result->SetMaterial(this->Material());
      }

      return result;
    }
    }
  }
}
#endif
