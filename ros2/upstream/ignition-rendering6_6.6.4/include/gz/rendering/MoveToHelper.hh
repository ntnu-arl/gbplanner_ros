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
#ifndef GZ_RENDERING_MOVETOHELPER_HH_
#define GZ_RENDERING_MOVETOHELPER_HH_

#include <memory>

#include <gz/common/Animation.hh>
#include <gz/common/KeyFrame.hh>

#include <gz/math/Box.hh>
#include <gz/math/Pose3.hh>

#include "gz/rendering/Camera.hh"

namespace ignition
{
  namespace rendering
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
      // forward declaration
      class MoveToHelperPrivate;

      /// \brief Helper class for animating a user camera to move to a target
      /// entity
      class IGNITION_RENDERING_VISIBLE MoveToHelper
      {
        public: MoveToHelper();

        public: ~MoveToHelper();

        /// \brief Move the camera to look at the specified target
        /// param[in] _camera Camera to be moved
        /// param[in] _target Target to look at
        /// param[in] _duration Duration of the move to animation, in seconds.
        /// param[in] _onAnimationComplete Callback function when animation is
        /// complete
        public: void MoveTo(const rendering::CameraPtr &_camera,
            const rendering::NodePtr &_target, double _duration,
            std::function<void()> _onAnimationComplete);

        /// \brief Move the camera to the specified pose.
        /// param[in] _camera Camera to be moved
        /// param[in] _target Pose to move to
        /// param[in] _duration Duration of the move to animation, in seconds.
        /// param[in] _onAnimationComplete Callback function when animation is
        /// complete
        public: void MoveTo(const rendering::CameraPtr &_camera,
            const math::Pose3d &_target, double _duration,
            std::function<void()> _onAnimationComplete);

        /// \brief Move the camera to look at the specified target
        /// param[in] _camera Camera to be moved
        /// param[in] _direction The pose to assume relative to the
        /// entit(y/ies), (0, 0, 0) indicates to return the camera back to the
        /// home pose originally loaded in from the sdf.
        /// param[in] _duration Duration of the move to animation, in seconds.
        /// param[in] _onAnimationComplete Callback function when animation is
        /// complete
        public: void LookDirection(const rendering::CameraPtr &_camera,
            const math::Vector3d &_direction, const math::Vector3d &_lookAt,
            double _duration, std::function<void()> _onAnimationComplete);

        /// \brief Add time to the animation.
        /// \param[in] _time Time to add in seconds
        public: void AddTime(double _time);

        /// \brief Get whether the move to helper is idle, i.e. no animation
        /// is being executed.
        /// \return True if idle, false otherwise
        public: bool Idle() const;

        /// \brief Set the initial camera pose
        /// param[in] _pose The init pose of the camera
        public: void SetInitCameraPose(const math::Pose3d &_pose);

        IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
        private: std::unique_ptr<MoveToHelperPrivate> dataPtr;
        IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING
      };
    }
  }
}
#endif
