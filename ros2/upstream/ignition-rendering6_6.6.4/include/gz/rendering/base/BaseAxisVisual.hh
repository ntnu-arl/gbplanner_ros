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
#ifndef GZ_RENDERING_BASE_BASEAXISVISUAL_HH_
#define GZ_RENDERING_BASE_BASEAXISVISUAL_HH_

#include "gz/rendering/AxisVisual.hh"
#include "gz/rendering/ArrowVisual.hh"
#include "gz/rendering/Scene.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    template <class T>
    class BaseAxisVisual :
      public virtual AxisVisual,
      public virtual T
    {
      protected: BaseAxisVisual();

      public: virtual ~BaseAxisVisual();

      public: virtual void Init() override;

      // Documentation inherited.
      protected: virtual void Destroy() override;

      // Documentation inherited.
      public: virtual void SetLocalScale(
          const math::Vector3d &_scale) override;

      // Documentation inherited.
      public: virtual math::Vector3d LocalScale() const override;

      // Documentation inherited.
      public: void ShowAxisHead(bool _b) override;

      // Documentation inherited.
      public: void ShowAxisHead(unsigned int _axis, bool _b) override;

      // Documentation inherited.
      public: virtual void SetVisible(bool _visible) override;
    };

    //////////////////////////////////////////////////
    template <class T>
    BaseAxisVisual<T>::BaseAxisVisual()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    BaseAxisVisual<T>::~BaseAxisVisual()
    {
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseAxisVisual<T>::Destroy()
    {
      for (unsigned int i = 0; i < this->ChildCount(); ++i)
      {
        auto arrow = std::dynamic_pointer_cast<rendering::ArrowVisual>(
              this->ChildByIndex(i));
        if (arrow)
        {
          arrow->Destroy();
        }
      }
      T::Destroy();
    }

    //////////////////////////////////////////////////
    template <class T>
    math::Vector3d BaseAxisVisual<T>::LocalScale() const
    {
      if (this->ChildCount() > 0) {
        return this->ChildByIndex(0)->LocalScale();
      }
      return math::Vector3d::Zero;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseAxisVisual<T>::SetLocalScale(const math::Vector3d &_scale)
    {
      for (unsigned int i = 0; i < this->ChildCount(); ++i)
        this->ChildByIndex(i)->SetLocalScale(_scale.X(),
                                             _scale.Y(),
                                             _scale.Z());
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseAxisVisual<T>::ShowAxisHead(bool _b)
    {
      for (unsigned int i = 0; i < this->ChildCount(); ++i)
      {
        auto arrow = std::dynamic_pointer_cast<rendering::ArrowVisual>(
              this->ChildByIndex(i));
        if (arrow)
        {
          arrow->ShowArrowHead(_b);
        }
      }
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseAxisVisual<T>::ShowAxisHead(unsigned int _axis, bool _b)
    {
      auto arrow = std::dynamic_pointer_cast<rendering::ArrowVisual>(
            this->ChildByIndex(2u - _axis));
      if (arrow)
      {
        arrow->ShowArrowHead(_b);
      }
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseAxisVisual<T>::Init()
    {
      T::Init();

      ArrowVisualPtr xArrow = this->Scene()->CreateArrowVisual();
      xArrow->SetLocalPosition(0, 0, 0);
      xArrow->SetLocalRotation(0, IGN_PI / 2, 0);
      xArrow->SetMaterial("Default/TransRed");
      this->AddChild(xArrow);

      ArrowVisualPtr yArrow = this->Scene()->CreateArrowVisual();
      yArrow->SetLocalPosition(0, 0, 0);
      yArrow->SetLocalRotation(-IGN_PI / 2, 0, 0);
      yArrow->SetMaterial("Default/TransGreen");
      this->AddChild(yArrow);

      ArrowVisualPtr zArrow = this->Scene()->CreateArrowVisual();
      zArrow->SetLocalPosition(0, 0, 0);
      zArrow->SetLocalRotation(0, 0, 0);
      zArrow->SetMaterial("Default/TransBlue");
      this->AddChild(zArrow);
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseAxisVisual<T>::SetVisible(bool _visible)
    {
      T::SetVisible(_visible);

      for (unsigned int i = 0; i < this->ChildCount(); ++i)
      {
        auto arrow = std::dynamic_pointer_cast<rendering::ArrowVisual>(
              this->ChildByIndex(i));
        if (arrow != nullptr)
          arrow->SetVisible(_visible);
      }
    }
    }
  }
}
#endif
