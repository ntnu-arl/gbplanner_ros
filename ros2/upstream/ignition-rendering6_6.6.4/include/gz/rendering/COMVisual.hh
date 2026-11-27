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
#ifndef IGNITION_RENDERING_COMVISUAL_HH_
#define IGNITION_RENDERING_COMVISUAL_HH_

#include <string>
#include <ignition/math/Inertial.hh>
#include "ignition/rendering/config.hh"
#include "ignition/rendering/Object.hh"
#include "ignition/rendering/RenderTypes.hh"
#include "ignition/rendering/Visual.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {

    /// \class COMVisual COMVisual.hh
    /// ignition/rendering/COMVisual.hh
    /// \brief Represents a center of mass visual
    class IGNITION_RENDERING_VISIBLE COMVisual :
      public virtual Visual
    {
      /// \brief Destructor
      public: virtual ~COMVisual() {}

      /// \brief Set the inertial component of the visual
      /// \param[in] _inertial Inertial component of the visual
      public: virtual void SetInertial(
                  const ignition::math::Inertiald &_inertial) = 0;

      /// \brief Set the mass of the parent
      /// \param[in] _mass Parent mass
      public: virtual void SetMass(double _mass) = 0;

      /// \brief Get the mass of the parent
      /// \return Parent mass
      public: virtual double Mass() const = 0;

      /// \brief Get the inertia pose
      /// \return Inertia pose in parent frame.
      public: virtual ignition::math::Pose3d InertiaPose() const = 0;

      /// \brief Get the sphere visual
      /// \return Pointer to the sphere visual
      public: virtual VisualPtr SphereVisual() const = 0;
    };
    }
  }
}
#endif
