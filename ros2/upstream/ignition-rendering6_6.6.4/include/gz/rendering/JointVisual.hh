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
#ifndef IGNITION_RENDERING_JOINTVISUAL_HH_
#define IGNITION_RENDERING_JOINTVISUAL_HH_

#include <string>

#include <ignition/math/Vector3.hh>

#include "ignition/rendering/config.hh"
#include "ignition/rendering/Object.hh"
#include "ignition/rendering/RenderTypes.hh"
#include "ignition/rendering/Visual.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \brief Enum for JointVisual types
    enum IGNITION_RENDERING_VISIBLE JointVisualType
    {
      /// \brief No type
      JVT_NONE           = 0,

      /// \brief Revolute joint type
      JVT_REVOLUTE       = 1,

      /// \brief Revolute2 joint type
      JVT_REVOLUTE2      = 2,

      /// \brief Prismatic joint type
      JVT_PRISMATIC      = 3,

      /// \brief Universal joint type
      JVT_UNIVERSAL      = 4,

      /// \brief Ball joint type
      JVT_BALL           = 5,

      /// \brief Screw joint type
      JVT_SCREW          = 6,

      /// \brief Gearbox joint type
      JVT_GEARBOX        = 7,

      /// \brief Fixed joint type
      JVT_FIXED          = 8
    };

    /// \class JointVisual JointVisual.hh
    /// ignition/rendering/JointVisual.hh
    /// \brief Represents a joint visual
    class IGNITION_RENDERING_VISIBLE JointVisual :
      public virtual Visual
    {
      /// \brief Destructor
      public: virtual ~JointVisual() {}

      /// \brief Create an axis and attach it to the joint visual.
      /// \param[in] _axis Axis vector.
      /// \param[in] _useParentFrame True if axis vector is expressed in
      /// parent frame.
      public: virtual void SetAxis(const ignition::math::Vector3d &_axis,
          bool _useParentFrame = false) = 0;

      /// \brief Get axis vector.
      /// \return The axis vector.
      public: virtual ignition::math::Vector3d Axis() const = 0;

      /// \brief Create a parent axis for hinge2 and universal joint types
      /// and attach it to the joint visual.
      /// \param[in] _axis Axis vector.
      /// \param[in] _parentName Joint parent name.
      /// \param[in] _useParentFrame True if axis vector is expressed in
      /// parent frame.
      public: virtual void SetParentAxis(
          const ignition::math::Vector3d &_axis,
          const std::string &_parentName,
          bool _useParentFrame = false) = 0;

      /// \brief Get parent axis vector.
      /// \return The parent axis vector.
      public: virtual ignition::math::Vector3d ParentAxis() const = 0;

      /// \brief Update an axis' arrow visual.
      /// \param[in] _axis Axis vector.
      /// \param[in] _useParentFrame True if axis vector is expressed in
      /// parent frame.
      /// \return True if axis was updated else false.
      public: virtual bool UpdateAxis(const ignition::math::Vector3d &_axis,
          bool _useParentFrame = false) = 0;

      /// \brief Update the parent axis' arrow visual if it exists.
      /// \param[in] _axis Axis vector.
      /// \param[in] _useParentFrame True if axis vector is expressed in
      /// parent frame.
      /// \return True if parent axis was updated else false.
      public: virtual bool UpdateParentAxis(
          const ignition::math::Vector3d &_axis,
          bool _useParentFrame = false) = 0;

      /// \brief Set type for joint visual.
      /// \param[in] _type The type of visualisation for joint.
      public: virtual void SetType(const JointVisualType _type) = 0;

      /// \brief Get joint visual type.
      /// \return The joint visual type.
      public: virtual JointVisualType Type() const = 0;

      /// \brief Get the JointVisual which is attached to the parent.
      /// \return Parent axis visual.
      public: virtual JointVisualPtr ParentAxisVisual() const = 0;

      /// \brief Get the arrow visual which represents the axis attached to the
      /// child.
      /// \return Arrow visual.
      public: virtual ArrowVisualPtr ArrowVisual() const = 0;
    };
    }
  }
}
#endif
