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
#ifndef GZ_RENDERING_BASE_BASEARROWVISUAL_HH_
#define GZ_RENDERING_BASE_BASEARROWVISUAL_HH_

#include <string>

#include <gz/common/MeshManager.hh>

#include "gz/rendering/ArrowVisual.hh"
#include "gz/rendering/Scene.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    template <class T>
    class BaseArrowVisual :
      public virtual ArrowVisual,
      public virtual T
    {
      /// \brief Constructor
      protected: BaseArrowVisual();

      /// \brief Destructor
      public: virtual ~BaseArrowVisual();

      // Documentation inherited.
      protected: virtual void Destroy() override;

      // Documentation inherited.
      public: virtual VisualPtr Head() const override;

      // Documentation inherited.
      public: virtual VisualPtr Shaft() const override;

      // Documentation inherited.
      public: virtual VisualPtr Rotation() const override;

      // Documentation inherited
      public: virtual void ShowArrowHead(bool _b) override;

      // Documentation inherited
      public: virtual void ShowArrowShaft(bool _b) override;

      // Documentation inherited
      public: virtual void ShowArrowRotation(bool _b) override;

      // Documentation inherited
      public: virtual void SetVisible(bool _visible) override;

      // Documentation inherited.
      protected: virtual void Init() override;

      /// \brief Flag to indicate whether arrow rotation is visible
      protected: bool rotationVisible = false;
    };

    //////////////////////////////////////////////////
    template <class T>
    BaseArrowVisual<T>::BaseArrowVisual()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    BaseArrowVisual<T>::~BaseArrowVisual()
    {
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseArrowVisual<T>::Destroy()
    {
      while (this->ChildCount() > 0u)
      {
        auto visual = std::dynamic_pointer_cast<Visual>(this->ChildByIndex(0));
        if (visual)
        {
          visual->Destroy();
        }
      }
      T::Destroy();
    }

    //////////////////////////////////////////////////
    template <class T>
    VisualPtr BaseArrowVisual<T>::Head() const
    {
      return std::dynamic_pointer_cast<Visual>(this->ChildByIndex(2));
    }

    //////////////////////////////////////////////////
    template <class T>
    VisualPtr BaseArrowVisual<T>::Shaft() const
    {
      return std::dynamic_pointer_cast<Visual>(this->ChildByIndex(1));
    }

    //////////////////////////////////////////////////
    template <class T>
    VisualPtr BaseArrowVisual<T>::Rotation() const
    {
      return std::dynamic_pointer_cast<Visual>(this->ChildByIndex(0));
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseArrowVisual<T>::ShowArrowHead(bool _b)
    {
      NodePtr child = this->ChildByIndex(2);
      VisualPtr visual = std::dynamic_pointer_cast<Visual>(child);
      if (visual)
      {
        visual->SetVisible(_b);
      }
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseArrowVisual<T>::ShowArrowShaft(bool _b)
    {
      NodePtr child = this->ChildByIndex(1);
      VisualPtr visual = std::dynamic_pointer_cast<Visual>(child);
      if (visual)
      {
        visual->SetVisible(_b);
      }
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseArrowVisual<T>::ShowArrowRotation(bool _b)
    {
      NodePtr child = this->ChildByIndex(0);
      VisualPtr visual = std::dynamic_pointer_cast<Visual>(child);
      if (visual)
      {
        visual->SetVisible(_b);
        this->rotationVisible = _b;
      }
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseArrowVisual<T>::SetVisible(bool _visible)
    {
      T::SetVisible(_visible);

      NodePtr child = this->ChildByIndex(0);
      VisualPtr visual = std::dynamic_pointer_cast<Visual>(child);
      if (visual)
      {
        // Force rotation visual visibility to false
        // if the arrow visual is not visible.
        // Else, rotation visual's visibility overrides
        // its parent's visibility.
        visual->SetVisible(this->rotationVisible && _visible);
      }
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseArrowVisual<T>::Init()
    {
      T::Init();

      VisualPtr cone = this->Scene()->CreateVisual();
      cone->AddGeometry(this->Scene()->CreateCone());
      cone->SetOrigin(0, 0, -0.5);
      cone->SetLocalPosition(0, 0, 0);
      cone->SetLocalScale(0.1, 0.1, 0.25);
      this->AddChild(cone);

      VisualPtr cylinder = this->Scene()->CreateVisual();
      cylinder->AddGeometry(this->Scene()->CreateCylinder());
      cylinder->SetOrigin(0, 0, 0.5);
      cylinder->SetLocalPosition(0, 0, 0);
      cylinder->SetLocalScale(0.05, 0.05, 0.5);
      this->AddChild(cylinder);

      common::MeshManager *meshMgr = common::MeshManager::Instance();
      std::string rotMeshName = "arrow_rotation";
      if (!meshMgr->HasMesh(rotMeshName))
        meshMgr->CreateTube(rotMeshName, 0.070f, 0.075f, 0.01f, 1, 32);

      VisualPtr rotationVis = this->Scene()->CreateVisual();
      rotationVis->AddGeometry(this->Scene()->CreateMesh(rotMeshName));
      rotationVis->SetOrigin(0, 0, -0.125);
      rotationVis->SetLocalPosition(0, 0, 0);
      rotationVis->SetVisible(this->rotationVisible);
      this->AddChild(rotationVis);

      this->SetOrigin(0, 0, -0.5);
    }
    }
  }
}
#endif
