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
#ifndef IGNITION_RENDERING_BASE_BASEJOINTVISUAL_HH_
#define IGNITION_RENDERING_BASE_BASEJOINTVISUAL_HH_

#include <algorithm>
#include <string>

#include "ignition/common/Console.hh"

#include "ignition/rendering/ArrowVisual.hh"
#include "ignition/rendering/AxisVisual.hh"
#include "ignition/rendering/JointVisual.hh"
#include "ignition/rendering/Scene.hh"
#include "ignition/rendering/base/BaseObject.hh"
#include "ignition/rendering/base/BaseRenderTypes.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \brief Base implementation of a joint visual
    template <class T>
    class BaseJointVisual :
      public virtual JointVisual,
      public virtual T
    {
      /// \brief Constructor
      protected: BaseJointVisual();

      /// \brief Destructor
      public: virtual ~BaseJointVisual();

      // Documentation inherited.
      protected: virtual void Init() override;

      // Documentation inherited.
      protected: virtual void PreRender() override;

      // Documentation inherited.
      protected: virtual void Destroy() override;

      // Documentation inherited.
      public: virtual void SetAxis(const ignition::math::Vector3d &_axis,
          bool _useParentFrame) override;

      // Documentation inherited.
      public: virtual ignition::math::Vector3d Axis() const override;

      // Documentation inherited.
      public: virtual void SetParentAxis(
          const ignition::math::Vector3d &_axis,
          const std::string &_parentName,
          bool _useParentFrame) override;

      // Documentation inherited.
      public: virtual ignition::math::Vector3d ParentAxis() const override;

      // Documentation inherited.
      public: virtual bool UpdateAxis(const ignition::math::Vector3d &_axis,
          bool _useParentFrame) override;

      // Documentation inherited.
      public: virtual bool UpdateParentAxis(
          const ignition::math::Vector3d &_axis,
          bool _useParentFrame) override;

      // Documentation inherited.
      public: virtual void SetType(const JointVisualType _type) override;

      // Documentation inherited.
      public: virtual JointVisualType Type() const override;

      // Documentation inherited.
      public: virtual JointVisualPtr ParentAxisVisual() const override;

      // Documentation inherited.
      public: virtual ArrowVisualPtr ArrowVisual() const override;

      // Documentation inherited.
      public: virtual void SetVisible(bool _visible) override;

      /// \brief Implementation for updating an axis' arrow visual.
      /// \param[in] _arrowVisual Arrow visual to be updated.
      /// \param[in] _axis Axis vector.
      /// \param[in] _useParentFrame True if the axis vector is
      /// expressed in the joint parent frame.
      protected: void UpdateAxisImpl(ArrowVisualPtr _arrowVisual,
          const ignition::math::Vector3d &_axis,
          bool _useParentFrame);

      /// \brief Helper function to create axis visual.
      protected: void CreateAxis();

      /// \brief Helper function to create parent axis visual.
      protected: void CreateParentAxis();

      /// \brief Scale the joint visual according to the joint's child.
      protected: void ScaleToChild();

      /// \brief Type of joint visualization.
      protected: JointVisualType jointVisualType =
          JointVisualType::JVT_NONE;

      /// \brief The joint's XYZ frame visual.
      protected: AxisVisualPtr axisVisual = nullptr;

      /// \brief The visual representing the one joint axis. There can be only
      /// one axis visual per joint visual, so joints with two axes have a 2nd
      /// JointVisual with its own arrowVisual.
      protected: ArrowVisualPtr arrowVisual = nullptr;

      /// \brief Second joint visual for hinge2 and universal joints. It is a
      /// simplified visual without an XYZ frame.
      protected: JointVisualPtr parentAxisVis = nullptr;

      /// \brief Scale based on the size of the joint's child.
      protected: ignition::math::Vector3d scaleToChild =
          ignition::math::Vector3d::One;

      /// \brief Flag to indicate joint visual type has changed.
      protected: bool dirtyJointType = false;

      /// \brief Flag to indicate axis data has changed.
      protected: bool dirtyAxis = false;

      /// \brief Flag to indicate parent axis data has changed.
      protected: bool dirtyParentAxis = false;

      /// \brief Joint visual axis vector.
      protected: ignition::math::Vector3d axis =
          ignition::math::Vector3d::Zero;

      /// \brief Flag to indicate whether axis vector is
      /// expressed in joint parent frame.
      protected: bool useParentFrame = false;

      /// \brief Flag to update the axis visual.
      protected: bool updateAxis = false;

      /// \brief Parent axis vector.
      protected: ignition::math::Vector3d parentAxis =
          ignition::math::Vector3d::Zero;

      /// \brief Joint parent name.
      protected: std::string jointParentName = "";

      /// \brief Flag to indicate whether parent axis vector is
      /// expressed in joint parent frame.
      protected: bool parentAxisUseParentFrame = false;

      /// \brief Flag to update the parent axis visual.
      protected: bool updateParentAxis = false;
    };

    //////////////////////////////////////////////////
    template <class T>
    BaseJointVisual<T>::BaseJointVisual()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    BaseJointVisual<T>::~BaseJointVisual()
    {
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseJointVisual<T>::PreRender()
    {
      T::PreRender();

      if (this->ParentAxisVisual())
      {
        this->ParentAxisVisual()->PreRender();
      }

      if (this->dirtyJointType)
      {
        this->UpdateAxis(this->axis, this->useParentFrame);
        this->UpdateParentAxis(this->parentAxis,
            this->parentAxisUseParentFrame);

        this->dirtyJointType = false;
      }

      if (this->dirtyAxis)
      {
        this->CreateAxis();
        this->dirtyAxis = false;
      }

      if (this->dirtyParentAxis)
      {
        this->CreateParentAxis();
        this->dirtyParentAxis = false;
      }

      if (this->updateAxis)
      {
        this->updateAxis =
            !this->UpdateAxis(this->axis, this->useParentFrame);
      }

      if (this->updateParentAxis)
      {
        this->updateParentAxis =
            !this->UpdateParentAxis(this->parentAxis,
                this->parentAxisUseParentFrame);
      }
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseJointVisual<T>::Init()
    {
      T::Init();

      this->axisVisual = this->Scene()->CreateAxisVisual();
      this->AddChild(this->axisVisual);
      this->SetInheritScale(false);
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseJointVisual<T>::Destroy()
    {
      if (this->arrowVisual != nullptr)
      {
        this->arrowVisual->Destroy();
        this->arrowVisual.reset();
      }

      if (this->axisVisual != nullptr)
      {
        this->axisVisual->Destroy();
        this->axisVisual.reset();
      }

      if (this->parentAxisVis != nullptr)
      {
        this->parentAxisVis->Destroy();
        this->parentAxisVis.reset();
      }

      this->dirtyJointType = false;
      this->dirtyAxis = false;
      this->dirtyParentAxis = false;

      T::Destroy();
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseJointVisual<T>::SetAxis(
          const ignition::math::Vector3d &_axis,
          bool _useParentFrame)
    {
      this->axis = _axis;
      this->useParentFrame = _useParentFrame;
      this->dirtyAxis = true;
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseJointVisual<T>::CreateAxis()
    {
      if (this->arrowVisual)
      {
        this->arrowVisual->Destroy();
        this->arrowVisual.reset();
      }

      this->arrowVisual = this->Scene()->CreateArrowVisual();
      this->arrowVisual->SetMaterial("Default/TransYellow");
      this->arrowVisual->SetLocalPosition(0, 0, 0);
      this->arrowVisual->SetLocalRotation(0, 0, 0);
      this->AddChild(this->arrowVisual);

      this->updateAxis = true;
      this->ScaleToChild();
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseJointVisual<T>::SetParentAxis(
          const ignition::math::Vector3d &_axis,
          const std::string &_parentName,
          bool _useParentFrame)
    {
      if (this->Type() != JointVisualType::JVT_REVOLUTE2 &&
          this->Type() != JointVisualType::JVT_UNIVERSAL)
      {
        ignlog << "Joint visual is not of type Revolute2 or "
               << " Universal "
               << " so the parent axis will not be shown\n";
        return;
      }

      this->parentAxis = _axis;
      this->parentAxisUseParentFrame = _useParentFrame;
      this->jointParentName = _parentName;
      this->dirtyParentAxis = true;
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseJointVisual<T>::CreateParentAxis()
    {
      auto jointParentVis = this->Scene()->NodeByName(this->jointParentName);
      if (jointParentVis == nullptr)
      {
        ignlog << "Joint parent with name " << this->jointParentName
               << " does not exist"
               << " so the parent axis will not be shown\n";
        return;
      }

      if (this->parentAxisVis)
      {
        this->parentAxisVis->Destroy();
        this->parentAxisVis.reset();
      }

      this->parentAxisVis = this->Scene()->CreateJointVisual();
      jointParentVis->AddChild(this->parentAxisVis);
      this->parentAxisVis->SetType(this->Type());
      this->parentAxisVis->SetAxis(this->parentAxis,
          this->parentAxisUseParentFrame);

      this->updateParentAxis = true;
      this->ScaleToChild();
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseJointVisual<T>::UpdateAxis(const ignition::math::Vector3d &_axis,
          bool _useParentFrame)
    {
      if (this->ArrowVisual() && this->HasParent())
      {
        this->UpdateAxisImpl(this->ArrowVisual(), _axis, _useParentFrame);
        return true;
      }

      return false;
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseJointVisual<T>::UpdateParentAxis(
          const ignition::math::Vector3d &_axis,
          bool _useParentFrame)
    {
      if (this->ParentAxisVisual() &&
          this->ParentAxisVisual()->ArrowVisual() &&
          this->ParentAxisVisual()->HasParent())
      {
        this->UpdateAxisImpl(this->ParentAxisVisual()->ArrowVisual(),
            _axis, _useParentFrame);
        return true;
      }

      return false;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseJointVisual<T>::UpdateAxisImpl(ArrowVisualPtr _arrowVisual,
          const ignition::math::Vector3d &_axis,
          bool _useParentFrame)
    {
      // Get rotation to axis vector
      ignition::math::Vector3d axisDir = _axis;
      ignition::math::Vector3d u = axisDir.Normalize();
      ignition::math::Vector3d v = ignition::math::Vector3d::UnitZ;
      double cosTheta = v.Dot(u);
      double angle = acos(cosTheta);
      ignition::math::Quaterniond quat;
      // check the parallel case
      if (ignition::math::equal(angle, IGN_PI))
        quat.Axis(u.Perpendicular(), angle);
      else
        quat.Axis((v.Cross(u)).Normalize(), angle);
      _arrowVisual->SetLocalRotation(quat);

      if (_useParentFrame)
      {
        ignition::math::Pose3d parentInitPose =
            this->Parent()->InitialLocalPose();

        // get rotation of joint visual in model frame
        ignition::math::Quaterniond quatFromModel =
            (parentInitPose * this->LocalPose()).Rot();

        // rotate arrow visual so that the axis vector applies to the model
        // frame.
        _arrowVisual->SetLocalRotation(quatFromModel.Inverse() *
            _arrowVisual->LocalRotation());
      }

      _arrowVisual->ShowArrowRotation(
          this->Type() == JointVisualType::JVT_REVOLUTE ||
          this->Type() == JointVisualType::JVT_REVOLUTE2 ||
          this->Type() == JointVisualType::JVT_UNIVERSAL ||
          this->Type() == JointVisualType::JVT_GEARBOX);

      if (this->axisVisual)
        _arrowVisual->SetVisible(true);
      else
        return;

      // Don't change the visibility of joint child axis
      if (this->ArrowVisual() != _arrowVisual)
        return;

       // Hide existing arrow head if it overlaps with the axis
      auto axisWorldRotation = _arrowVisual->WorldPose().Rot();
      auto jointWorldRotation = this->WorldPose().Rot();

      this->axisVisual->ShowAxisHead(true);
      _arrowVisual->ShowArrowShaft(true);

      auto axisWorld = axisWorldRotation * ignition::math::Vector3d::UnitZ;
      if (axisWorld == jointWorldRotation * ignition::math::Vector3d::UnitX)
      {
        this->axisVisual->ShowAxisHead(0, false);
        _arrowVisual->ShowArrowShaft(false);
      }
      else if (axisWorld ==
          jointWorldRotation * ignition::math::Vector3d::UnitY)
      {
        this->axisVisual->ShowAxisHead(1, false);
        _arrowVisual->ShowArrowShaft(false);
      }
      else if (axisWorld ==
          jointWorldRotation * ignition::math::Vector3d::UnitZ)
      {
        this->axisVisual->ShowAxisHead(2, false);
        _arrowVisual->ShowArrowShaft(false);
      }
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseJointVisual<T>::ScaleToChild()
    {
      if (!this->HasParent())
        return;

      // Joint visual is attached to the child's visual
      VisualPtr parentVisual =
          std::dynamic_pointer_cast<Visual>(this->Parent());

      if (parentVisual)
      {
        double childSize =
            std::max(0.1, parentVisual->BoundingBox().Size().Length());
        this->scaleToChild = ignition::math::Vector3d(childSize * 0.2,
            childSize * 0.2, childSize * 0.2);
        this->SetLocalScale(this->scaleToChild);
        if (this->ParentAxisVisual())
          this->ParentAxisVisual()->SetLocalScale(this->scaleToChild);
      }
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseJointVisual<T>::SetType(const JointVisualType _type)
    {
      this->jointVisualType = _type;
      this->dirtyJointType = true;
    }

    //////////////////////////////////////////////////
    template <class T>
    ignition::math::Vector3d BaseJointVisual<T>::Axis() const
    {
      return this->axis;
    }

    //////////////////////////////////////////////////
    template <class T>
    ignition::math::Vector3d BaseJointVisual<T>::ParentAxis() const
    {
      return this->parentAxis;
    }

    //////////////////////////////////////////////////
    template <class T>
    JointVisualType BaseJointVisual<T>::Type() const
    {
      return this->jointVisualType;
    }

    //////////////////////////////////////////////////
    template <class T>
    JointVisualPtr BaseJointVisual<T>::ParentAxisVisual() const
    {
      return this->parentAxisVis;
    }

    //////////////////////////////////////////////////
    template <class T>
    ArrowVisualPtr BaseJointVisual<T>::ArrowVisual() const
    {
      return this->arrowVisual;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseJointVisual<T>::SetVisible(bool _visible)
    {
      T::SetVisible(_visible);

      if (this->ArrowVisual())
        this->ArrowVisual()->SetVisible(_visible);

      if (this->Type() == JointVisualType::JVT_REVOLUTE2 ||
          this->Type() == JointVisualType::JVT_UNIVERSAL)
      {
        if (this->ParentAxisVisual())
          this->ParentAxisVisual()->SetVisible(_visible);
      }

      if (this->axisVisual)
        this->axisVisual->SetVisible(_visible);
    }
    }
  }
}
#endif
