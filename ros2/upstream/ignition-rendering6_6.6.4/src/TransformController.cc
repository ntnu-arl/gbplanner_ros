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

#include <gz/common/Console.hh>
#include <gz/math/Plane.hh>

#include "gz/rendering/Camera.hh"
#include "gz/rendering/GizmoVisual.hh"
#include "gz/rendering/RayQuery.hh"
#include "gz/rendering/Scene.hh"
#include "gz/rendering/TransformController.hh"

/// \brief Private data class for TransformController
class gz::rendering::TransformControllerPrivate
{
  /// \brief Gizmo visual that provides translation, rotation, and scale
  /// tandles for transformation
  public: GizmoVisualPtr gizmoVisual;

  /// \brief Node to be transformed
  public: NodePtr node;

  /// \brief Camera the transform controller is associated with
  public: CameraPtr camera;

  /// \brief Current transform mode, i.e. translation, rotation, scale
  public: TransformMode mode = TransformMode::TM_NONE;

  /// \brief Space in which transformations are applied
  public: TransformSpace space = TransformSpace::TS_LOCAL;

  /// \brief Ray used for checking intersection with planes for computing
  /// 3d world coordinates from 2d
  public: RayQueryPtr ray;

  /// \brief Current active axis of transformation
  public: gz::math::Vector3d axis;

  /// \brief Flag to indicate that transfomation is in progress.
  public: bool active = false;

  /// \brief Initial pose of the node being transformed.
  /// This is set in Start();
  public: gz::math::Pose3d nodeStartPose;

  /// \brief Initial scale of the node being transformed.
  /// This is set in Start();
  public: gz::math::Vector3d nodeStartScale;
};

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
TransformController::TransformController()
    : dataPtr(new TransformControllerPrivate)
{
}

//////////////////////////////////////////////////
TransformController::~TransformController()
{
}

//////////////////////////////////////////////////
void TransformController::SetCamera(const CameraPtr &_camera)
{
  if (this->dataPtr->camera == _camera)
    return;
  this->dataPtr->camera = _camera;

  if (!this->dataPtr->ray)
    this->dataPtr->ray = this->dataPtr->camera->Scene()->CreateRayQuery();
}

//////////////////////////////////////////////////
CameraPtr TransformController::Camera() const
{
  return this->dataPtr->camera;
}

//////////////////////////////////////////////////
void TransformController::Update()
{
  if (!this->dataPtr->node)
  {
    if (this->dataPtr->gizmoVisual)
      this->dataPtr->gizmoVisual->SetTransformMode(TransformMode::TM_NONE);
    return;
  }

  if (!this->dataPtr->gizmoVisual)
  {
    this->dataPtr->gizmoVisual =
        this->dataPtr->node->Scene()->CreateGizmoVisual();
    this->dataPtr->node->Scene()->RootVisual()->AddChild(
        this->dataPtr->gizmoVisual);
  }

  // update gizmo
  this->dataPtr->gizmoVisual->SetTransformMode(this->dataPtr->mode);
  this->dataPtr->gizmoVisual->SetActiveAxis(this->dataPtr->axis);

  if (!this->dataPtr->camera)
    return;

  // update scale
  math::Vector3d nodePos = this->dataPtr->node->WorldPose().Pos();
  double scale = nodePos.Distance(this->dataPtr->camera->WorldPose().Pos())
      / 2.0;
  this->dataPtr->gizmoVisual->SetLocalScale(scale, scale, scale);

  // update gizmo visual position
  math::Vector3d pos = this->dataPtr->node->WorldPosition();
  // update gizmo visual rotation so that they are always facing the
  // eye position
  math::Quaterniond rot;
  if (this->dataPtr->mode == TransformMode::TM_ROTATION)
  {
    math::Vector3d eye = this->dataPtr->camera->WorldPosition() - pos;
    eye = eye.Normalize();
    math::Quaterniond nodeRot;
    if (this->dataPtr->space == TransformSpace::TS_LOCAL)
    {
      nodeRot = this->dataPtr->node->WorldRotation();
      eye = nodeRot.RotateVectorReverse(eye);
    }
    VisualPtr xVis = this->dataPtr->gizmoVisual->ChildByAxis(
        TransformAxis::TA_ROTATION_X);
    math::Vector3d xRot(atan2(-eye.Y(), eye.Z()), 0, 0);
    math::Vector3d xRotOffset(0, -IGN_PI * 0.5, 0);
    xVis->SetWorldRotation(nodeRot *
        math::Quaterniond(xRot) * math::Quaterniond(xRotOffset));

    VisualPtr yVis = this->dataPtr->gizmoVisual->ChildByAxis(
        TransformAxis::TA_ROTATION_Y);
    math::Vector3d yRot(0, atan2(eye.X(), eye.Z()), 0);
    math::Vector3d yRotOffset(IGN_PI * 0.5, -IGN_PI * 0.5, 0);
    yVis->SetWorldRotation(nodeRot *
        math::Quaterniond(yRot) * math::Quaterniond(yRotOffset));

    VisualPtr zVis = this->dataPtr->gizmoVisual->ChildByAxis(
        TransformAxis::TA_ROTATION_Z);
    math::Vector3d zRot(0, 0, atan2(eye.Y(), eye.X()));
    zVis->SetWorldRotation(nodeRot * math::Quaterniond(zRot));

    VisualPtr circleVis = this->dataPtr->gizmoVisual->ChildByAxis(
        TransformAxis::TA_ROTATION_Z << 1);
    math::Matrix4d lookAt;
    lookAt = lookAt.LookAt(this->dataPtr->camera->WorldPosition(), pos);
    math::Vector3d circleRotOffset(0, IGN_PI * 0.5, 0);
    circleVis->SetWorldRotation(
        lookAt.Rotation() * math::Quaterniond(circleRotOffset));
  }
  else
  {
    // scaling is limited to local space
    if (this->dataPtr->space == TransformSpace::TS_LOCAL ||
        this->dataPtr->mode == TransformMode::TM_SCALE)
      rot = this->dataPtr->node->WorldRotation();
  }

  this->dataPtr->gizmoVisual->SetWorldPose(
      math::Pose3d(pos, rot));
}

//////////////////////////////////////////////////
NodePtr TransformController::Node() const
{
  return this->dataPtr->node;
}

//////////////////////////////////////////////////
void TransformController::Attach(NodePtr _node)
{
  if (this->dataPtr->node == _node)
    return;

  this->dataPtr->node = _node;
}

//////////////////////////////////////////////////
void TransformController::Detach()
{
  this->dataPtr->node.reset();
}

//////////////////////////////////////////////////
void TransformController::SetTransformMode(TransformMode _mode)
{
  if (this->dataPtr->mode == _mode)
    return;

  this->dataPtr->mode = _mode;
}

//////////////////////////////////////////////////
TransformMode TransformController::Mode() const
{
  return this->dataPtr->mode;
}

//////////////////////////////////////////////////
math::Vector3d TransformController::ActiveAxis() const
{
  return this->dataPtr->axis;
}

//////////////////////////////////////////////////
void TransformController::SetActiveAxis(const math::Vector3d &_axis)
{
  this->dataPtr->axis = _axis;
}

//////////////////////////////////////////////////
void TransformController::Start()
{
  if (!this->dataPtr->node)
    return;
  this->dataPtr->nodeStartPose = this->dataPtr->node->WorldPose();
  this->dataPtr->nodeStartScale = this->dataPtr->node->LocalScale();
  this->dataPtr->active = true;
}

//////////////////////////////////////////////////
void TransformController::Stop()
{
  this->dataPtr->active = false;
  this->dataPtr->axis = math::Vector3d::Zero;
}

//////////////////////////////////////////////////
bool TransformController::Active() const
{
  return this->dataPtr->active;
}

//////////////////////////////////////////////////
math::Vector3d TransformController::AxisById(unsigned int _id) const
{
  if (!this->dataPtr->gizmoVisual)
    return math::Vector3d::Zero;

  TransformAxis ta = this->dataPtr->gizmoVisual->AxisById(_id);
  return TransformController::ToAxis(ta);
}

//////////////////////////////////////////////////
void TransformController::Translate(
    const math::Vector3d &_translation, bool _snap)
{
  if (!this->dataPtr->node)
  {
    ignerr << "No node attached for transformation" << std::endl;
    return;
  }

  bool started = this->dataPtr->active;
  if (!started)
    this->Start();

  math::Vector3d translation = _translation;
  if (this->dataPtr->space == TransformSpace::TS_LOCAL)
    translation = this->dataPtr->nodeStartPose.Rot().RotateVector(translation);

  math::Vector3d pos = this->dataPtr->nodeStartPose.Pos() + translation;
  if (_snap)
    pos = TransformController::SnapPoint(pos);

  this->dataPtr->node->SetWorldPosition(pos);

  if (!started)
    this->Stop();
}

//////////////////////////////////////////////////
void TransformController::SetTransformSpace(TransformSpace _space)
{
  this->dataPtr->space = _space;
}

//////////////////////////////////////////////////
TransformSpace TransformController::Space() const
{
  return this->dataPtr->space;
}

//////////////////////////////////////////////////
math::Vector3d TransformController::TranslationFrom2d(
    const math::Vector3d &_axis,
    const math::Vector2d &_start, const math::Vector2d &_end)
{
  if (!this->dataPtr->node)
  {
    ignerr << "No node attached" << std::endl;
    return math::Vector3d::Zero;
  }

  if (!this->dataPtr->camera)
  {
    ignerr << "Camera not set" << std::endl;
    return math::Vector3d::Zero;
  }

  math::Planed plane = this->CreatePlaneForTransform(_axis, _start);

  // Compute two points on the plane. The first point is the current
  // pos, the second is the previous pos
  math::Vector3d translation;
  math::Vector3d start;
  this->dataPtr->ray->SetFromCamera(this->dataPtr->camera, _start);
  if (!this->RayPlaneIntersection(this->dataPtr->ray, plane, start))
    return translation;
  math::Vector3d end;
  this->dataPtr->ray->SetFromCamera(this->dataPtr->camera, _end);
  if (!this->RayPlaneIntersection(this->dataPtr->ray, plane, end))
    return translation;

  end = end - this->dataPtr->nodeStartPose.Pos();
  start = start - this->dataPtr->nodeStartPose.Pos();

  translation = end - start;
  if (this->dataPtr->space == TransformSpace::TS_LOCAL)
  {
    translation =
        this->dataPtr->nodeStartPose.Rot().RotateVectorReverse(translation);
  }

  translation *= _axis;

  return translation;
}

//////////////////////////////////////////////////
math::Quaterniond TransformController::RotationFrom2d(
    const math::Vector3d &_axis,
    const math::Vector2d &_start, const math::Vector2d &_end)
{
  /// get start and end pos in world frame from 2d point
  math::Planed plane = this->CreatePlaneForTransform(_axis, _start);
  math::Quaterniond rot;
  math::Vector3d start;
  this->dataPtr->ray->SetFromCamera(this->dataPtr->camera, _start);
  if (!this->RayPlaneIntersection(this->dataPtr->ray, plane, start))
    return rot;
  math::Vector3d end;
  this->dataPtr->ray->SetFromCamera(this->dataPtr->camera, _end);
  if (!this->RayPlaneIntersection(this->dataPtr->ray, plane, end))
    return rot;

  // get vectors from node pos to both points
  start = start - this->dataPtr->nodeStartPose.Pos();
  end = end - this->dataPtr->nodeStartPose.Pos();
  start = start.Normalize();
  end = end.Normalize();
  // compute angle between two vectors
  double signTest = start.Cross(end).Dot(plane.Normal());
  double angle = atan2((start.Cross(end)).Length(), start.Dot(end));

  if (signTest < 0 )
    angle *= -1;

  rot = math::Quaterniond(this->dataPtr->axis, angle);

  return rot;
}

//////////////////////////////////////////////////
math::Vector3d TransformController::ScaleFrom2d(
    const math::Vector3d &_axis,
    const math::Vector2d &_start, const math::Vector2d &_end)
{
  if (!this->dataPtr->node)
  {
    ignerr << "No node attached" << std::endl;
    return math::Vector3d::Zero;
  }

  if (!this->dataPtr->camera)
  {
    ignerr << "Camera not set" << std::endl;
    return math::Vector3d::Zero;
  }

  math::Planed plane = this->CreatePlaneForTransform(_axis, _start);

  // Compute two points on the plane. The first point is the current
  // pos, the second is the previous pos
  math::Vector3d scale = math::Vector3d::One;
  math::Vector3d start;
  this->dataPtr->ray->SetFromCamera(this->dataPtr->camera, _start);
  if (!this->RayPlaneIntersection(this->dataPtr->ray, plane, start))
    return scale;
  math::Vector3d end;
  this->dataPtr->ray->SetFromCamera(this->dataPtr->camera, _end);
  if (!this->RayPlaneIntersection(this->dataPtr->ray, plane, end))
    return scale;

  end = end - this->dataPtr->nodeStartPose.Pos();
  start = start - this->dataPtr->nodeStartPose.Pos();

  double s = end.Length() / start.Length();
  scale = math::Vector3d(s, s, s);

  // limit scale to axis of transform
  if (_axis.X() <= 0)
    scale.X(1);
  if (_axis.Y() <= 0)
    scale.Y(1);
  if (_axis.Z() <= 0)
    scale.Z(1);

  return scale;
}

//////////////////////////////////////////////////
math::Planed TransformController::CreatePlaneForTransform(
    const math::Vector3d &_axis, const math::Vector2d &_pt2d)
{
  math::Pose3d pose = this->dataPtr->nodeStartPose;
  math::Vector3d planeNorm(0, 0, 0);

  // get plane normal to create plane for ray-plane intersection
  if (this->dataPtr->mode == TransformMode::TM_ROTATION)
  {
    if (this->dataPtr->space == TransformSpace::TS_LOCAL)
    {
      if (_axis.X() > 0)
        planeNorm = this->dataPtr->nodeStartPose.Rot().XAxis();
      else if (_axis.Y() > 0)
        planeNorm = this->dataPtr->nodeStartPose.Rot().YAxis();
      else if (_axis.Z() > 0)
        planeNorm = this->dataPtr->nodeStartPose.Rot().ZAxis();
    }
    else
      planeNorm = _axis;
  }
  else
  {
    math::Vector3d projNorm(0, 0, 0);
    math::Vector3d planeNormOther(0, 0, 0);

    if (_axis.X() > 0 && _axis.Y() > 0)
    {
      planeNorm.Z(1);
      projNorm.Z(1);
    }
    else if (_axis.Z() > 0)
    {
      planeNorm.Y(1);
      projNorm.X(1);
      planeNormOther.X(1);
    }
    else if (_axis.X() > 0)
    {
      planeNorm.Z(1);
      projNorm.Y(1);
      planeNormOther.Y(1);
    }
    else if (_axis.Y() > 0)
    {
      planeNorm.Z(1);
      projNorm.X(1);
      planeNormOther.X(1);
    }

    if (this->dataPtr->space == TransformSpace::TS_LOCAL)
    {
      planeNorm = pose.Rot().RotateVector(planeNorm);
      projNorm = pose.Rot().RotateVector(projNorm);
      planeNormOther = pose.Rot().RotateVector(planeNormOther);
    }

    // compute ray from camera. Get its direction vector
    this->dataPtr->ray->SetFromCamera(this->dataPtr->camera, _pt2d);
    math::Vector3d dir = this->dataPtr->ray->Direction();

    // Fine tune ray casting: compute angle of ray to tangent plane
    // Use the plane that is less parallel for better results.
    double angle = dir.Dot(planeNorm);
    double angleOther = dir.Dot(planeNormOther);
    if (fabs(angleOther) > fabs(angle))
    {
      projNorm = planeNorm;
      planeNorm = planeNormOther;
    }
  }

  // Compute the distance from the camera to plane
  double d = pose.Pos().Dot(planeNorm);
  return math::Planed(planeNorm, d);
}

//////////////////////////////////////////////////
bool TransformController::RayPlaneIntersection(RayQueryPtr _ray,
  const math::Planed &_plane, math::Vector3d &_result)
{
  // Cast ray from the camera into the world
  math::Vector3d origin = _ray->Origin();
  math::Vector3d dir = _ray->Direction();

  // get distance of ray from plane
  double dist = _plane.Distance(origin, dir);
  if (math::equal(dist, 0.0))
    return false;

  // compute ray-point intersection point
  //  3d pos in world frame
  _result = origin + dir * dist;

  return true;
}

//////////////////////////////////////////////////
void TransformController::Rotate(const math::Quaterniond &_rotation, bool _snap)
{
  if (!this->dataPtr->node)
  {
    ignerr << "No node attached for transformation" << std::endl;
    return;
  }

  bool started = this->dataPtr->active;
  if (!started)
    this->Start();

  math::Quaterniond rotation = _rotation;
  if (this->dataPtr->space == TransformSpace::TS_LOCAL)
    rotation = this->dataPtr->nodeStartPose.Rot() * rotation;
  else
    rotation = rotation * this->dataPtr->nodeStartPose.Rot();

  if (_snap)
  {
    math::Vector3d axis;
    double angle;
    rotation.ToAxis(axis, angle);
    angle = rint(angle / (IGN_PI * 0.25)) * (IGN_PI * 0.25);
    rotation.Axis(axis, angle);
  }

  this->dataPtr->node->SetWorldRotation(rotation);

  if (!started)
    this->Stop();
}

//////////////////////////////////////////////////
void TransformController::Scale(const math::Vector3d &_scale, bool _snap)
{
  if (!this->dataPtr->node)
  {
    ignerr << "No node attached for transformation" << std::endl;
    return;
  }

  bool started = this->dataPtr->active;
  if (!started)
    this->Start();

  math::Vector3d scale = _scale;
  scale = this->dataPtr->nodeStartScale * scale.Abs();

  if (_snap)
  {
    scale = this->SnapPoint(scale);
    // prevent setting zero scale
    scale.X(std::max(1e-4, scale.X()));
    scale.Y(std::max(1e-4, scale.Y()));
    scale.Z(std::max(1e-4, scale.Z()));
  }

  this->dataPtr->node->SetLocalScale(scale);

  if (!started)
    this->Stop();
}

/////////////////////////////////////////////////
math::Vector3d TransformController::SnapPoint(
    const math::Vector3d &_point,
    double _interval, double _sensitivity)
{
  if (_interval < 0)
  {
    ignerr << "Interval distance must be greater than or equal to 0"
        << std::endl;
    return math::Vector3d::Zero;
  }

  if (_sensitivity < 0 || _sensitivity > 1.0)
  {
    ignerr << "Sensitivity must be between 0 and 1" << std::endl;
    return math::Vector3d::Zero;
  }

  math::Vector3d point = _point;
  double snap = _interval * _sensitivity;

  double remainder = fmod(point.X(), _interval);
  int sign = remainder >= 0 ? 1 : -1;
  if (fabs(remainder) < snap)
      point.X() -= remainder;
  else if (fabs(remainder) > (_interval - snap))
      point.X() = point.X() - remainder + _interval * sign;

  remainder = fmod(point.Y(), _interval);
  sign = remainder >= 0 ? 1 : -1;
  if (fabs(remainder) < snap)
      point.Y() -= remainder;
  else if (fabs(remainder) > (_interval - snap))
      point.Y() = point.Y() - remainder + _interval * sign;

  remainder = fmod(point.Z(), _interval);
  sign = remainder >= 0 ? 1 : -1;
  if (fabs(remainder) < snap)
      point.Z() -= remainder;
  else if (fabs(remainder) > (_interval - snap))
      point.Z() = point.Z() - remainder + _interval * sign;

  return point;
}

//////////////////////////////////////////////////
math::Vector3d TransformController::ToAxis(TransformAxis _axis)
{
  math::Vector3d axis;
  if (_axis & TA_TRANSLATION_X || _axis & TA_ROTATION_X ||
      _axis & TA_SCALE_X)
    axis.X(1);
  if (_axis & TA_TRANSLATION_Y || _axis & TA_ROTATION_Y ||
      _axis & TA_SCALE_Y)
    axis.Y(1);
  if (_axis & TA_TRANSLATION_Z || _axis & TA_ROTATION_Z ||
      _axis & TA_SCALE_Z)
    axis.Z(1);
  return axis;
}
