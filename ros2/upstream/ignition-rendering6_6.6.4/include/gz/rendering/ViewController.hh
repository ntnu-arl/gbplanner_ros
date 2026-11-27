/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#ifndef GZ_RENDERING_VIEWCONTROLLER_HH_
#define GZ_RENDERING_VIEWCONTROLLER_HH_

#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>

#include "gz/rendering/config.hh"
#include "gz/rendering/Camera.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \class ViewController ViewController.hh
    ///   ignition/rendering/ViewController.hh
    /// \brief A camera view controller
    class IGNITION_RENDERING_VISIBLE ViewController
    {
      /// \brief Deconstructor
      public: virtual ~ViewController() { }

      /// \brief Set the camera that will be controlled by this view controller.
      /// \param[in] _camera Camera to control
      public: virtual void SetCamera(const CameraPtr &_camera) = 0;

      /// \brief Set target point for pan, zoom, oribit
      /// \param[in] _target Target point in world coordinates
      public: virtual void SetTarget(const math::Vector3d &_target) = 0;

      /// \brief Set zoom amount
      /// \param[in] _value Camera zoon value, e.g. mouse scroll delta
      public: virtual void Zoom(const double _value) = 0;

      /// \brief Set camera pan (translational movement) around target point.
      /// \param[in] _value Pan amount in image plane, e.g. mouse drag delta.
      public: virtual void Pan(const math::Vector2d &_value) = 0;

      /// \brief Set camera orbit (rotational movement) around target point.
      /// \param[in] _value robit amount in image plane, e.g. mouse drag delta
      public: virtual void Orbit(const math::Vector2d &_value) = 0;
    };
    }
  }
}
#endif
