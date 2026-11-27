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
#ifndef GZ_RENDERING_TRANSFORMCONTROLLER_HH_
#define GZ_RENDERING_TRANSFORMCONTROLLER_HH_

#include <memory>

#include <gz/common/SuppressWarning.hh>

#include <gz/math/Quaternion.hh>
#include <gz/math/Plane.hh>
#include <gz/math/Vector3.hh>

#include "gz/rendering/config.hh"
#include "gz/rendering/Camera.hh"
#include "gz/rendering/GizmoVisual.hh"
#include "gz/rendering/TransformType.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    // forward declarations
    class TransformControllerPrivate;

    /* \class TransformController TransformController.hh \
     * ignition/rendering/TransformController.hh
     */
    /// \brief An transform tool for translating, rotating, and scaling objects.
    class IGNITION_RENDERING_VISIBLE TransformController
    {
      /// \brief Constructor
      public: TransformController();

      /// \brief Destructor
      public: virtual ~TransformController();

      /// \brief Update the transform controller. Its visual is updated based
      /// on the current transform mode, space, and axis.
      public: virtual void Update();

      /// \brief Set the camera.
      /// \param[in] _camera Camera
      public: virtual void SetCamera(const CameraPtr &_camera);

      /// \brief get the camera that is controlled by this view controller.
      /// \return camera being controlled
      public: virtual CameraPtr Camera() const;

      /// \brief Get the attached node
      /// \return The node which is being transformed, nullptr if there is no
      /// node attached.
      public: virtual NodePtr Node() const;

      /// \brief Attach the transform controller to the specified node
      /// \param[in] _node Node that will be transformed
      public: virtual void Attach(NodePtr _node);

      /// \brief Detach the transform controller from the currently attached
      /// node
      public: virtual void Detach();

      /// \brief Set the transform mode
      /// \param[in] _mode Transform mode to set to
      public: virtual void SetTransformMode(TransformMode _mode);

      /// \brief Get the current transform mode
      /// \return Transform mode
      public: virtual TransformMode Mode() const;

      /// \brief Set the transform space
      /// \param[in] _space Transform space to set to
      public: virtual void SetTransformSpace(TransformSpace _space);

      /// \brief Get the transform space
      /// \return Transform space
      public: virtual TransformSpace Space() const;

      /// \brief Set the active transform axis. This highlights the axis visual
      /// \param[in] _axis Transform axis vector
      public: virtual void SetActiveAxis(const math::Vector3d &_axis);

      /// \brief Get the current highlighted active axis of transform
      /// \return Active axis of transform
      public: virtual math::Vector3d ActiveAxis() const;

      /// \brief Get the axis of transform represented by the given node id
      /// \param[in] _id Id of the node to check
      /// \return Transform axis
      public: virtual math::Vector3d AxisById(unsigned int _id) const;

      /// \brief Translate the attached node
      /// \param[in] _translation Translation in the current transform space
      /// \param[in] _snap True to snap the final position to fixed increments
      /// \sa SetTransformSpace
      public: virtual void Translate(const math::Vector3d &_translation,
          bool _snap = false);

      /// \brief Rotate the attached node
      /// \param[in] _rotation Rotation in the current transform space
      /// \param[in] _snap True to snap rotation angle to fixed increments
      /// \sa SetTransformSpace
      public: virtual void Rotate(const math::Quaterniond &_rotation,
          bool _snap = false);

      /// \brief Scale the attached node.
      /// \param[in] _scale Scale in the current transform space
      /// \param[in] _snap True to snap scale to fixed increments
      /// \sa SetTransformSpace
      public: virtual void Scale(const math::Vector3d &_scale,
          bool _snap = false);

      /// \brief Snap a point at intervals of a fixed distance. Currently used
      /// to give a snapping behavior when transforming objects
      /// \param[in] _point Input point.
      /// \param[in] _interval Fixed distance interval at which the point
      /// is snapped.
      /// \param[in] _sensitivity Sensitivity of point snapping, in terms of a
      /// percentage of the interval.
      /// \return Snapped 3D point.
      public: static gz::math::Vector3d SnapPoint(
          const gz::math::Vector3d &_point, const double _interval = 1.0,
          const double _sensitivity = 0.4);

      /// \brief Start the transform process. To be used with helper functions
      /// that computions transforms from 2d movements.
      /// \sa Stop
      public: virtual void Start();

      /// \brief Stop the transform process. To be used with helper functions
      /// that computions transforms from 2d movements.
      /// \sa Stop
      public: virtual void Stop();

      /// \brief Check if the transform process is active
      /// \return True if node is being transformed
      public: virtual bool Active() const;

      /// \brief Helper function to compute 3d translation from 2d translation
      /// movement. Useful for converting 2d mouse drag motion to displacement
      /// in world frame. Note that the camera must be set and Start() must be
      /// called before using this function.
      /// \param[in] _axis Axis of 3d translation. The resulting 3d vector
      /// is constrained to this axis.
      /// \param[in] _start Starting position of the 2d translation movement
      /// \param[in] _end End position of the 2d translation movement
      /// \return 3d translation vector in the current transform space
      public: math::Vector3d TranslationFrom2d(const math::Vector3d &_axis,
          const math::Vector2d &_start, const math::Vector2d &_end);

      /// \brief Helper function to compute 3d rotation from 2d translation
      /// movement. Useful for converting 2d mouse drag motion to rotation angle
      /// in world frame. Note that the camera must be set and Start() must be
      /// called before using this function.
      /// \param[in] _axis Axis of 3d rotation. The resulting rotation is
      /// is constrained to this axis.
      /// \param[in] _start Starting position of the 2d translation movement
      /// \param[in] _end End position of the 2d translation movement
      /// \return Rotation in the current transform space
      public: math::Quaterniond RotationFrom2d(const math::Vector3d &_axis,
          const math::Vector2d &_start, const math::Vector2d &_end);

      /// \brief Helper function to compute 3d scale from 2d translation
      /// movement. Useful for converting 2d mouse drag motion to scale vector
      /// in world frame. Note that the camera must be set and Start() must be
      /// called before using this function.
      /// \param[in] _axis Axis of scale. The resulting 3d vector
      /// is constrained to this axis.
      /// \param[in] _start Starting position of the 2d translation movement
      /// \param[in] _end End position of the 2d translation movement
      /// \return 3d scale vector in the current transform space
      public: math::Vector3d ScaleFrom2d(const math::Vector3d &_axis,
          const math::Vector2d &_start, const math::Vector2d &_end);

      /// \brief Convert a TransformAxis to a vector type
      /// \param[in] _axis Transform axis
      /// \return 3d axis vector
      public: static math::Vector3d ToAxis(TransformAxis _axis);

      /// \brief Helper function to create a plane for transfomation.
      /// The plane is chosen based on the current transform mode and the
      /// transform axis specified.
      /// \param[in] _axis Transformation axis used to identify the plane for
      /// projection.
      /// \param[in] _pt2d 2d point to help fine tune the plane created.
      /// \return Plane created for transform
      protected: math::Planed CreatePlaneForTransform(
          const math::Vector3d &_axis, const math::Vector2d &_pt2d);

      /// \brief Helper function to compute ray plane intersection
      /// \param[in] _ray Input ray
      /// \param[in] _plane Plane to check for intersection
      /// \param[out] _result Intersection point
      /// \return True if intersection point exist, false if ray is parallel
      /// to plane
      protected: bool RayPlaneIntersection(RayQueryPtr _ray,
          const math::Planed &_plane, math::Vector3d &_result);

      /// \brief Private data pointer
      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      public: std::unique_ptr<TransformControllerPrivate> dataPtr;
      IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING
    };
    }
  }
}
#endif
