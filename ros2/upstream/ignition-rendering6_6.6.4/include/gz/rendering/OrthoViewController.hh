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
#ifndef IGNITION_RENDERING_ORTHOVIEWCONTROLLER_HH_
#define IGNITION_RENDERING_ORTHOVIEWCONTROLLER_HH_

#include <memory>
#include <string>
#include <ignition/math/Vector2.hh>

#include "ignition/rendering/ViewController.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    // Forward declare private data pointer.
    class OrthoViewControllerPrivate;

    /// \class OrthoViewController OrthoViewController.hh
    /// \brief Orthographic view controller
    class IGNITION_RENDERING_VISIBLE OrthoViewController
        : public virtual ViewController
    {
      /// \brief Constructor
      public: OrthoViewController();

      /// \brief Constructor.
      /// \param[in] _camera Pointer to the camera to control.
      public: explicit OrthoViewController(const CameraPtr &_camera);

      /// \brief Destructor.
      public: virtual ~OrthoViewController();

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
      public: virtual const math::Vector3d &Target() const;

      /// \brief Set zoom amount
      /// \param[in] _value Camera zoon value, e.g. mouse scroll delta
      public: virtual void Zoom(const double _value);

      /// \brief Set camera pan (translational movement) around target point.
      /// \param[in] _value Pan amount in image plane, e.g. mouse drag delta.
      public: virtual void Pan(const math::Vector2d &_value);

      /// \brief Set camera orbit (rotational movement) around target point.
      /// \param[in] _value robit amount in image plane, e.g. mouse drag delta
      public: virtual void Orbit(const math::Vector2d &_value);

      /// \brief Resize and update the camera projection matrix
      /// \param[in] _width Image width
      /// \param[in] _height Image height
      private: virtual void Resize(const unsigned int _width,
                                  const unsigned int _height);

      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      /// \brief Private data pointer
      public: std::unique_ptr<OrthoViewControllerPrivate> dataPtr;
      IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING
    };
    }
  }
}
#endif
