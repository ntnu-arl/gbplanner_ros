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

#include "gz/rendering/MoveToHelper.hh"

#include <memory>

#include <gz/common/Animation.hh>
#include <gz/common/KeyFrame.hh>
#include <gz/math/Pose3.hh>

#include "gz/rendering/Camera.hh"

class gz::rendering::MoveToHelperPrivate
{
  /// \brief Pose animation object
  public: std::unique_ptr<common::PoseAnimation> poseAnim;

  /// \brief Pointer to the camera being moved
  public: rendering::CameraPtr camera;

  /// \brief Callback function when animation is complete.
  public: std::function<void()> onAnimationComplete;

  /// \brief Initial pose of the camera used for view angles
  public: math::Pose3d initCameraPose;
};

using namespace gz;
using namespace rendering;

////////////////////////////////////////////////
MoveToHelper::MoveToHelper() :
  dataPtr(new MoveToHelperPrivate)
{
}

////////////////////////////////////////////////
MoveToHelper::~MoveToHelper() = default;

////////////////////////////////////////////////
void MoveToHelper::MoveTo(const CameraPtr &_camera,
    const math::Pose3d &_target,
    double _duration, std::function<void()> _onAnimationComplete)
{
  this->dataPtr->camera = _camera;
  this->dataPtr->poseAnim = std::make_unique<common::PoseAnimation>(
      "move_to", _duration, false);
  this->dataPtr->onAnimationComplete = std::move(_onAnimationComplete);

  math::Pose3d start = _camera->WorldPose();

  common::PoseKeyFrame *key = this->dataPtr->poseAnim->CreateKeyFrame(0);
  key->Translation(start.Pos());
  key->Rotation(start.Rot());

  key = this->dataPtr->poseAnim->CreateKeyFrame(_duration);
  if (_target.Pos().IsFinite())
    key->Translation(_target.Pos());
  else
    key->Translation(start.Pos());

  if (_target.Rot().IsFinite())
    key->Rotation(_target.Rot());
  else
    key->Rotation(start.Rot());
}

////////////////////////////////////////////////
void MoveToHelper::MoveTo(const CameraPtr &_camera,
    const NodePtr &_target,
    double _duration, std::function<void()> _onAnimationComplete)
{
  this->dataPtr->camera = _camera;
  this->dataPtr->poseAnim = std::make_unique<common::PoseAnimation>(
      "move_to", _duration, false);
  this->dataPtr->onAnimationComplete = std::move(_onAnimationComplete);

  math::Pose3d start = _camera->WorldPose();

  // todo(anyone) implement bounding box function in rendering to get
  // target size and center.
  // Assume fixed size and target world position is its center
  math::Box targetBBox(1.0, 1.0, 1.0);
  math::Vector3d targetCenter = _target->WorldPosition();
  math::Vector3d dir = targetCenter - start.Pos();
  dir.Correct();
  dir.Normalize();

  // distance to move
  double maxSize = targetBBox.Size().Max();
  double dist = start.Pos().Distance(targetCenter) - maxSize;

  // Scale to fit in view
  double hfov = this->dataPtr->camera->HFOV().Radian();
  double offset = maxSize*0.5 / std::tan(hfov/2.0);

  // End position and rotation
  math::Vector3d endPos = start.Pos() + dir*(dist - offset);
  math::Quaterniond endRot =
      math::Matrix4d::LookAt(endPos, targetCenter).Rotation();
  math::Pose3d end(endPos, endRot);

  common::PoseKeyFrame *key = this->dataPtr->poseAnim->CreateKeyFrame(0);
  key->Translation(start.Pos());
  key->Rotation(start.Rot());

  key = this->dataPtr->poseAnim->CreateKeyFrame(_duration);
  key->Translation(end.Pos());
  key->Rotation(end.Rot());
}

////////////////////////////////////////////////
void MoveToHelper::LookDirection(const CameraPtr &_camera,
    const math::Vector3d &_direction, const math::Vector3d &_lookAt,
    double _duration, std::function<void()> _onAnimationComplete)
{
  this->dataPtr->camera = _camera;
  this->dataPtr->poseAnim = std::make_unique<common::PoseAnimation>(
      "view_angle", _duration, false);
  this->dataPtr->onAnimationComplete = std::move(_onAnimationComplete);

  math::Pose3d start = _camera->WorldPose();

  // Look at world origin unless there are visuals selected
  // Keep current distance to look at target
  math::Vector3d camPos = _camera->WorldPose().Pos();
  double distance = std::fabs((camPos - _lookAt).Length());

  // Calculate camera position
  math::Vector3d endPos = _lookAt - _direction * distance;

  // Calculate camera orientation
  math::Quaterniond endRot =
    math::Matrix4d::LookAt(endPos, _lookAt).Rotation();

  // Move camera to that pose
  common::PoseKeyFrame *key = this->dataPtr->poseAnim->CreateKeyFrame(0);
  key->Translation(start.Pos());
  key->Rotation(start.Rot());

  // Move camera back to initial pose
  if (_direction == math::Vector3d::Zero)
  {
    endPos = this->dataPtr->initCameraPose.Pos();
    endRot = this->dataPtr->initCameraPose.Rot();
  }

  key = this->dataPtr->poseAnim->CreateKeyFrame(_duration);
  key->Translation(endPos);
  key->Rotation(endRot);
}

////////////////////////////////////////////////
void MoveToHelper::AddTime(double _time)
{
  if (!this->dataPtr->camera || !this->dataPtr->poseAnim)
    return;

  common::PoseKeyFrame kf(0);

  this->dataPtr->poseAnim->AddTime(_time);
  this->dataPtr->poseAnim->InterpolatedKeyFrame(kf);

  math::Pose3d offset(kf.Translation(), kf.Rotation());

  this->dataPtr->camera->SetWorldPose(offset);

  if (this->dataPtr->poseAnim->Length() <= this->dataPtr->poseAnim->Time())
  {
    if (this->dataPtr->onAnimationComplete)
    {
      this->dataPtr->onAnimationComplete();
    }
    this->dataPtr->camera.reset();
    this->dataPtr->poseAnim.reset();
    this->dataPtr->onAnimationComplete = nullptr;
  }
}

////////////////////////////////////////////////
bool MoveToHelper::Idle() const
{
  return this->dataPtr->poseAnim == nullptr;
}

////////////////////////////////////////////////
void MoveToHelper::SetInitCameraPose(const math::Pose3d &_pose)
{
  this->dataPtr->initCameraPose = _pose;
}
