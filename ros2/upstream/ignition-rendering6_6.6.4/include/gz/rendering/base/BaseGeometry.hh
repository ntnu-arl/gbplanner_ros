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
#ifndef GZ_RENDERING_BASE_BASEGEOMETRY_HH_
#define GZ_RENDERING_BASE_BASEGEOMETRY_HH_

#include <string>

#include <ignition/common/Console.hh>

#include "gz/rendering/Geometry.hh"
#include "gz/rendering/Scene.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    template <class T>
    class BaseGeometry :
      public virtual Geometry,
      public virtual T
    {
      protected: BaseGeometry();

      public: virtual ~BaseGeometry();

      public: virtual VisualPtr Parent() const override = 0;

      public: virtual void RemoveParent() override;

      // Documentation inherited
      public: virtual void SetMaterial(const std::string &_material,
                  bool _unique = true) override;

      // Documentation inherited
      public: virtual void SetMaterial(MaterialPtr _material,
                  bool _unique = true) override = 0;

      // Documentation inherited
      public: virtual GeometryPtr Clone() const override;

      // Documentation inherited
      public: virtual void Destroy() override;
    };

    //////////////////////////////////////////////////
    template <class T>
    BaseGeometry<T>::BaseGeometry()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    BaseGeometry<T>::~BaseGeometry()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseGeometry<T>::RemoveParent()
    {
      VisualPtr parent = this->Parent();

      if (parent)
      {
        auto baseShared = this->shared_from_this();

        auto thisShared =
          std::dynamic_pointer_cast<BaseGeometry<T>>(baseShared);

        parent->RemoveGeometry(thisShared);
      }
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseGeometry<T>::SetMaterial(const std::string &_name, bool _unique)
    {
      MaterialPtr material = this->Scene()->Material(_name);
      if (material) this->SetMaterial(material, _unique);
    }

    //////////////////////////////////////////////////
    template <class T>
    GeometryPtr BaseGeometry<T>::Clone() const
    {
      ignwarn << "Clone functionality for Geometry does not exist yet.\n";
      return nullptr;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseGeometry<T>::Destroy()
    {
      T::Destroy();
      this->RemoveParent();
    }
    }
  }
}
#endif
