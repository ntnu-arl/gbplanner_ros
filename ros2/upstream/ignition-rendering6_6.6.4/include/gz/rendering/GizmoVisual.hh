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
#ifndef GZ_RENDERING_GIZMOVISUAL_HH_
#define GZ_RENDERING_GIZMOVISUAL_HH_

#include "gz/rendering/config.hh"
#include "gz/rendering/CompositeVisual.hh"
#include "gz/rendering/RenderTypes.hh"
#include "gz/rendering/Export.hh"
#include "gz/rendering/TransformType.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \class GizmoVisual GizmoVisual.hh ignition/rendering/GizmoVisual.hh
    /// \brief A gizmo that contains rotate, translate, and scale visuals
    class IGNITION_RENDERING_VISIBLE GizmoVisual :
      public virtual CompositeVisual
    {
      /// \brief Destructor
      public: virtual ~GizmoVisual() { }

      /// \brief Set the transform mode of the gizmo. This controls the visual
      /// appearance of the gizmo. Only the visuals in the specified mode will
      /// be visible, e.g. if set to TM_TRANSLATION mode, then only the
      /// translation axis visuals will be visible.
      /// \param[in] _mode Transform mode to set the gizmo to
      /// \sa GizmoMode
      public: virtual void SetTransformMode(TransformMode _mode) = 0;

      /// \brief Get the current transform mode.
      /// \return Transform mode
      /// \sa SetTransformMode
      public: virtual TransformMode Mode() const = 0;

      /// \brief Set the active axis in the gizmo visual
      /// \param[in] _axis Axis to be set active
      public: virtual void SetActiveAxis(const math::Vector3d &_axis) = 0;

      /// \brief Get the current active axis.
      /// \return Active axis
      /// \sa SetActiveAxis
      public: virtual math::Vector3d ActiveAxis() const = 0;

      /// \brief Get the Transform axis associated with the given id.
      /// \param[in] _id  Id of the visual associated with a transform axis
      /// \return Transform axis with the given id
      public: virtual TransformAxis AxisById(unsigned int _id) const = 0;

      /// \brief Get the child visual by axis
      /// \return Visual representing the specified transform axis or transform
      /// mode
      /// \sa TransformAxis, TransformMode
      public: virtual VisualPtr ChildByAxis(unsigned int _axis) const = 0;
    };
    }
  }
}
#endif
