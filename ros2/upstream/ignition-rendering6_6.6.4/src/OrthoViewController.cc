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

#include <ignition/common/Console.hh>

#include "ignition/rendering/OrthoViewController.hh"
#include "ignition/rendering/Scene.hh"
#include "ignition/rendering/Visual.hh"

/// \internal
/// \brief OrthoViewController private data.
class ignition::rendering::OrthoViewControllerPrivate
{
  /// \brief Build a custom scaled orthographic projection matrix.
  /// \param[in] _left Left position
  /// \param[in] _right Right position
  /// \param[in] _bottom Bottom position
  /// \param[in] _top Top position
  /// \param[in] _near Near clip distance
  /// \param[in] _far Far clip distance
  /// \return Custom orthographic projection matrix
  public: math::Matrix4d BuildScaledOrthoMatrix(
              float _left, float _right,
              float _bottom, float _top,
              float _near, float _far) const;

  /// \brief Pointer to camera
  public: CameraPtr camera;

  /// \brief Scale used for zooming within the orthographic view
  public: double scale = 1.0;

  /// \brief Target point for camera movements
  public: math::Vector3d target;
};

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
OrthoViewController::OrthoViewController()
    : dataPtr(new OrthoViewControllerPrivate)
{
  this->dataPtr->scale = 100;
}

//////////////////////////////////////////////////
OrthoViewController::OrthoViewController(const CameraPtr &_camera)
    : dataPtr(new OrthoViewControllerPrivate)
{
  this->dataPtr->scale = 100;
  this->SetCamera(_camera);
}

//////////////////////////////////////////////////
OrthoViewController::~OrthoViewController()
{
}

//////////////////////////////////////////////////
void OrthoViewController::SetCamera(const CameraPtr &_camera)
{
  this->dataPtr->camera = _camera;
  if (!this->dataPtr->camera)
    return;

  // reset scale and update projection
  if (this->dataPtr->camera->ProjectionType() != CPT_ORTHOGRAPHIC)
  {
    this->dataPtr->scale = 100;
  }
  this->dataPtr->camera->SetProjectionType(CPT_ORTHOGRAPHIC);
  unsigned int width = this->dataPtr->camera->ImageWidth();
  unsigned int height = this->dataPtr->camera->ImageHeight();
  this->Resize(width, height);
}

//////////////////////////////////////////////////
CameraPtr OrthoViewController::Camera() const
{
  return this->dataPtr->camera;
}

//////////////////////////////////////////////////
void OrthoViewController::SetTarget(const math::Vector3d &_target)
{
  this->dataPtr->target = _target;
}

//////////////////////////////////////////////////
const math::Vector3d &OrthoViewController::Target() const
{
  return this->dataPtr->target;
}

//////////////////////////////////////////////////
void OrthoViewController::Zoom(double _value)
{
  // Zoom to mouse cursor position
  // Three step process:
  // Translate mouse point to center of screen
  // Zoom by changing the orthographic window size
  // Translate back to mouse cursor position

  ignition::math::Vector3d translation;
  int width = this->dataPtr->camera->ImageWidth();
  int height = this->dataPtr->camera->ImageHeight();

  double orthoWidth = width / this->dataPtr->scale;
  double orthoHeight = height / this->dataPtr->scale;

  auto screenPos = this->dataPtr->camera->Project(
    this->dataPtr->target);;

  translation.Set(0.0,
      ((width/2.0 - screenPos.X()) / static_cast<float>(width))
      * orthoWidth,
      ((height/2.0 - screenPos.Y()) / static_cast<float>(height))
      * orthoHeight);

  this->dataPtr->camera->SetWorldPosition(
      this->dataPtr->camera->WorldPosition() +
      this->dataPtr->camera->WorldRotation() * translation);

  // zoom by changing scale
  // for simplicity, zoom in if a positive value is given otherwise
  // zoom out
  // todo(anyone) improving zooming based on distance to target
  double factor = 1.0;
  if (_value > 0)
    factor = 0.9;
  else
    factor = 1.1;

  this->dataPtr->scale /= factor;

  // build custom projection matrix from custom near and far clipping planes,
  // had to set a negative near clippping plane to workaround a camera
  // culling issue in orthographic view.
  auto proj = this->dataPtr->BuildScaledOrthoMatrix(
      -width / this->dataPtr->scale / 2.0,
       width / this->dataPtr->scale / 2.0,
      -height / this->dataPtr->scale / 2.0,
       height / this->dataPtr->scale / 2.0,
      -500, this->dataPtr->camera->FarClipPlane());

  this->dataPtr->camera->SetProjectionMatrix(proj);

  double newOrthoWidth = width / this->dataPtr->scale;
  double newOrthoHeight = height / this->dataPtr->scale;

  translation.Set(0.0,
      ((screenPos.X() - width/2.0) / static_cast<double>(width))
      * newOrthoWidth,
      ((screenPos.Y() - height/2.0) / static_cast<double>(height))
      * newOrthoHeight);

  this->dataPtr->camera->SetWorldPosition(
      this->dataPtr->camera->WorldPosition() +
      this->dataPtr->camera->WorldRotation() * translation);
}

//////////////////////////////////////////////////
void OrthoViewController::Resize(unsigned int _width, unsigned int _height)
{
  math::Matrix4d proj = this->dataPtr->BuildScaledOrthoMatrix(
       _width / this->dataPtr->scale / -2.0,
       _width / this->dataPtr->scale / 2.0,
       _height / this->dataPtr->scale / -2.0,
       _height / this->dataPtr->scale / 2.0,
      -500, this->dataPtr->camera->FarClipPlane());

  this->dataPtr->camera->SetProjectionMatrix(proj);
}

//////////////////////////////////////////////////
void OrthoViewController::Pan(const math::Vector2d &_value)
{
  if (!this->dataPtr->camera)
  {
    ignerr << "Camera is NULL" << std::endl;
    return;
  }

  double viewportWidth = this->dataPtr->camera->ImageWidth();
  double viewportHeight = this->dataPtr->camera->ImageHeight();

  ignition::math::Vector3d translation;

  double factor = 1.0;

  double orthoWidth = viewportWidth/this->dataPtr->scale;
  double orthoHeight = viewportHeight/this->dataPtr->scale;

  // Translate in the "y" "z" plane.
  translation.Set(0.0,
      (_value.X() / static_cast<double>(viewportWidth)) *
      orthoWidth * factor,
      (_value.Y() / static_cast<double>(viewportHeight)) *
      orthoHeight * factor);

  // Translate in the local coordinate frame
  this->dataPtr->camera->SetWorldPosition(
      this->dataPtr->camera->WorldPosition() +
      this->dataPtr->camera->WorldRotation() * translation);
}

//////////////////////////////////////////////////
void OrthoViewController::Orbit(const math::Vector2d &_value)
{
  if (!this->dataPtr->camera)
  {
    ignerr << "Camera is NULL" << std::endl;
    return;
  }

  double dy = 2 * IGN_PI * _value.X() / this->dataPtr->camera->ImageWidth();
  double dp = 2 * IGN_PI * _value.Y() / this->dataPtr->camera->ImageHeight();

  // translate to make target the origin for rotation
  this->dataPtr->camera->SetWorldPosition(
      this->dataPtr->camera->WorldPosition() - this->dataPtr->target);

  // rotate around world axis at target point
  math::Quaterniond yawQuat;
  yawQuat.Axis(math::Vector3d::UnitZ, -dy);
  this->dataPtr->camera->SetWorldRotation(
      yawQuat * this->dataPtr->camera->WorldRotation());
  this->dataPtr->camera->SetWorldPosition(
      yawQuat * this->dataPtr->camera->WorldPosition());

  math::Quaterniond localPitchQuat;
  localPitchQuat.Axis(
      this->dataPtr->camera->WorldRotation()*math::Vector3d::UnitY, dp);
  this->dataPtr->camera->SetWorldRotation(
      localPitchQuat * this->dataPtr->camera->WorldRotation());
  this->dataPtr->camera->SetWorldPosition(
      localPitchQuat * this->dataPtr->camera->WorldPosition());

  // translate camera back
  this->dataPtr->camera->SetWorldPosition(
      this->dataPtr->camera->WorldPosition() + this->dataPtr->target);
}

//////////////////////////////////////////////////
math::Matrix4d OrthoViewControllerPrivate::BuildScaledOrthoMatrix(
    float _left, float _right,
    float _bottom, float _top,
    float _near, float _far) const
{
  float invw = 1.0f / (_right - _left);
  float invh = 1.0f / (_top - _bottom);
  float invd = 1.0f / (_far - _near);

  math::Matrix4d proj;
  proj(0, 0) = 2.0f * invw;
  proj(0, 3) = -(_right + _left) * invw;
  proj(1, 1) = 2.0f * invh;
  proj(1, 3) = -(_top + _bottom) * invh;
  proj(2, 2) = -2.0f * invd;
  proj(2, 3) = -(_far + _near) * invd;
  proj(3, 3) = 1.0f;

  return proj;
}
