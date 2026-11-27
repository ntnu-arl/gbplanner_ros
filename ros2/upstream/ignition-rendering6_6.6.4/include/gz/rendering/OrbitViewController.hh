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
#ifndef GZ_RENDERING_ORBITVIEWCONTROLLER_HH_
#define GZ_RENDERING_ORBITVIEWCONTROLLER_HH_

#include <memory>

#include <gz/common/SuppressWarning.hh>

#include <gz/math/Vector3.hh>

#include "gz/rendering/config.hh"
#include "gz/rendering/Camera.hh"
#include "gz/rendering/ViewController.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    // forward declarations
    class OrbitViewControllerPrivate;

    /// \class OrbitViewController OrbitViewController.hh
    /// ignition/rendering/OrbitViewController.hh
    /// \brief A camera view controller
    class IGNITION_RENDERING_VISIBLE OrbitViewController
        : public virtual ViewController
    {
      /// \brief Deconstructor
      public: OrbitViewController();

      /// \brief Deconstructor
      public: explicit OrbitViewController(const CameraPtr &_camera);

      /// \brief Deconstructor
      public: virtual ~OrbitViewController();

      /// \brief Set the camera that will be controlled by this view controller.
      /// \param[in] _camera Camera to control
      public: virtual void SetCamera(const CameraPtr &_camera);

      /// \brief Get the camera that is controlled by this view controller.
      /// \return Camera being controlled
      public: virtual CameraPtr Camera() const;

      /// \brief Set target point for pan, zoom, oribit
      /// \param[in] _target Target point in world coordinates
      public: virtual void SetTarget(const math::Vector3d &_target);

      /// \brief Get target point for pan, zoom, oribit
      /// \return target point in world coordinates
      public: virtual math::Vector3d &Target() const;

      /// \brief Set zoom amount
      /// \param[in] _value Camera zoon value, e.g. mouse scroll delta
      public: virtual void Zoom(const double _value);

      /// \brief Set camera pan (translational movement) around target point.
      /// \param[in] _value Pan amount in image plane, e.g. mouse drag delta.
      public: virtual void Pan(const math::Vector2d &_value);

      /// \brief Set camera orbit (rotational movement) around target point.
      /// \param[in] _value robit amount in image plane, e.g. mouse drag delta
      public: virtual void Orbit(const math::Vector2d &_value);

      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      /// \brief Private data pointer
      public: std::unique_ptr<OrbitViewControllerPrivate> dataPtr;
      IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING
    };
    }
  }
}
#endif
