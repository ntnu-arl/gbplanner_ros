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
#ifndef IGNITION_RENDERING_DISTORTIONPASS_HH_
#define IGNITION_RENDERING_DISTORTIONPASS_HH_

#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include "ignition/rendering/config.hh"
#include "ignition/rendering/Export.hh"
#include "ignition/rendering/RenderPass.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /* \class DistortionPass DistortionPass.hh \
     * ignition/rendering/DistortionPass.hh
     */
    /// \brief A render pass that applies distortion to the render target
    class IGNITION_RENDERING_VISIBLE DistortionPass
      : public virtual RenderPass
    {
      /// \brief Constructor
      public: DistortionPass();

      /// \brief Destructor
      public: virtual ~DistortionPass();

      /// \brief Get the radial distortion coefficient k1.
      /// \return Distortion coefficient k1.
      public: virtual double K1() const = 0;

      /// \brief Get the radial distortion coefficient k2.
      /// \return Distortion coefficient k2.
      public: virtual double K2() const = 0;

      /// \brief Get the radial distortion coefficient k3.
      /// \return Distortion coefficient k3.
      public: virtual double K3() const = 0;

      /// \brief Get the tangential distortion coefficient p1.
      /// \return Distortion coefficient p1.
      public: virtual double P1() const = 0;

      /// \brief Get the tangential distortion coefficient p2.
      /// \return Distortion coefficient p2.
      public: virtual double P2() const = 0;

      /// \brief Get the distortion center.
      /// \return Distortion center.
      public: virtual math::Vector2d Center() const = 0;

      /// \brief Set the radial distortion coefficient k1.
      /// \param[in] _k1 Distortion coefficient k1.
      public: virtual void SetK1(double _k1) = 0;

      /// \brief Set the radial distortion coefficient k2.
      /// \param[in] _k2 Distortion coefficient k1.
      public: virtual void SetK2(double _k2) = 0;

      /// \brief Set the radial distortion coefficient k3.
      /// \param[in] _k3 Distortion coefficient k1.
      public: virtual void SetK3(double _k3) = 0;

      /// \brief Set the tangential distortion coefficient p1.
      /// \param[in] _p1 Distortion coefficient k1.
      public: virtual void SetP1(double _p1) = 0;

      /// \brief Set the tangential distortion coefficient p2.
      /// \param[in] _p2 Distortion coefficient k1.
      public: virtual void SetP2(double _p2) = 0;

      /// \brief Set the distortion center.
      /// \param[in] _center Distortion center.
      public: virtual void SetCenter(const math::Vector2d &_center) = 0;
    };
    }
  }
}
#endif
