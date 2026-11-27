/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include "gz/rendering/ogre2/Ogre2Camera.hh"
#include "gz/rendering/ogre2/Ogre2Conversions.hh"
#include "gz/rendering/ogre2/Ogre2RenderTarget.hh"
#include "gz/rendering/ogre2/Ogre2Scene.hh"
#include "gz/rendering/ogre2/Ogre2SelectionBuffer.hh"
#include "gz/rendering/Utils.hh"

#ifdef _MSC_VER
  #pragma warning(push, 0)
#endif
#include <OgreCamera.h>
#include <OgreItem.h>
#include <OgreSceneManager.h>
#ifdef _MSC_VER
  #pragma warning(pop)
#endif

/// \brief Private data for the Ogre2Camera class
class gz::rendering::Ogre2CameraPrivate
{
};

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
Ogre2Camera::Ogre2Camera()
  : dataPtr(std::make_unique<Ogre2CameraPrivate>())
{
}

//////////////////////////////////////////////////
Ogre2Camera::~Ogre2Camera()
{
  this->Destroy();
}

//////////////////////////////////////////////////
void Ogre2Camera::Destroy()
{
  if (!this->ogreCamera || !this->Scene()->IsInitialized())
    return;

  this->DestroyRenderTexture();

  Ogre::SceneManager *ogreSceneManager;
  ogreSceneManager = this->scene->OgreSceneManager();
  if (ogreSceneManager == nullptr)
  {
    ignerr << "Scene manager cannot be obtained" << std::endl;
  }
  if (ogreSceneManager->findCameraNoThrow(this->name) != nullptr)
  {
    ogreSceneManager->destroyCamera(this->ogreCamera);
    this->ogreCamera = nullptr;
  }
}

//////////////////////////////////////////////////
math::Angle Ogre2Camera::HFOV() const
{
  double vfov = this->ogreCamera->getFOVy().valueRadians();
  double hFOV = 2.0 * atan(tan(vfov / 2.0) * this->AspectRatio());
  return math::Angle(hFOV);
}

//////////////////////////////////////////////////
void Ogre2Camera::SetHFOV(const math::Angle &_angle)
{
  BaseCamera::SetHFOV(_angle);
  double angle = _angle.Radian();
  double vfov = 2.0 * atan(tan(angle / 2.0) / this->AspectRatio());
  this->ogreCamera->setFOVy(Ogre::Radian(vfov));
}

//////////////////////////////////////////////////
double Ogre2Camera::AspectRatio() const
{
  return this->ogreCamera->getAspectRatio();
}

//////////////////////////////////////////////////
void Ogre2Camera::SetAspectRatio(const double _ratio)
{
  BaseCamera::SetAspectRatio(_ratio);
  return this->ogreCamera->setAspectRatio(_ratio);
}

//////////////////////////////////////////////////
unsigned int Ogre2Camera::AntiAliasing() const
{
  return this->renderTexture->AntiAliasing();
}

//////////////////////////////////////////////////
void Ogre2Camera::SetAntiAliasing(const unsigned int _aa)
{
  BaseCamera::SetAntiAliasing(_aa);
  this->renderTexture->SetAntiAliasing(_aa);
}

//////////////////////////////////////////////////
math::Color Ogre2Camera::BackgroundColor() const
{
  return this->renderTexture->BackgroundColor();
}

//////////////////////////////////////////////////
void Ogre2Camera::SetBackgroundColor(const math::Color &_color)
{
  this->renderTexture->SetBackgroundColor(_color);
}

//////////////////////////////////////////////////
MaterialPtr Ogre2Camera::BackgroundMaterial() const
{
  return this->renderTexture->BackgroundMaterial();
}

//////////////////////////////////////////////////
void Ogre2Camera::SetBackgroundMaterial(MaterialPtr _material)
{
  this->renderTexture->SetBackgroundMaterial(_material);
}

//////////////////////////////////////////////////
void Ogre2Camera::SetMaterial(const MaterialPtr &_material)
{
  this->renderTexture->SetMaterial(_material);
}

//////////////////////////////////////////////////
void Ogre2Camera::Render()
{
  this->renderTexture->Render();
}

//////////////////////////////////////////////////
RenderTargetPtr Ogre2Camera::RenderTarget() const
{
  return this->renderTexture;
}

//////////////////////////////////////////////////
void Ogre2Camera::Init()
{
  BaseCamera::Init();
  this->CreateCamera();
  this->CreateRenderTexture();
  this->Reset();
}

//////////////////////////////////////////////////
void Ogre2Camera::CreateCamera()
{
  // create ogre camera object
  Ogre::SceneManager *ogreSceneManager;
  ogreSceneManager = this->scene->OgreSceneManager();
  this->ogreCamera = ogreSceneManager->createCamera(this->name);
  // by default, ogre2 cameras are attached to root scene node
  this->ogreCamera->detachFromParent();
  this->ogreNode->attachObject(this->ogreCamera);

  // rotate to Gazebo coordinate system
  this->ogreCamera->yaw(Ogre::Degree(-90.0));
  this->ogreCamera->roll(Ogre::Degree(-90.0));
  this->ogreCamera->setFixedYawAxis(false);

  // TODO(anyone): provide api access
  this->ogreCamera->setAutoAspectRatio(true);
  this->ogreCamera->setProjectionType(Ogre::PT_PERSPECTIVE);
  this->ogreCamera->setCustomProjectionMatrix(false);
}

//////////////////////////////////////////////////
void Ogre2Camera::CreateRenderTexture()
{
  this->DestroyRenderTexture();
  RenderTexturePtr base = this->scene->CreateRenderTexture();
  this->renderTexture = std::dynamic_pointer_cast<Ogre2RenderTexture>(base);
  this->renderTexture->SetCamera(this->ogreCamera);
  this->renderTexture->SetFormat(PF_R8G8B8);
  this->renderTexture->SetWidth(this->ImageWidth());
  this->renderTexture->SetHeight(this->ImageHeight());
  this->renderTexture->SetBackgroundColor(this->scene->BackgroundColor());
  this->renderTexture->SetVisibilityMask(this->visibilityMask);
}


//////////////////////////////////////////////////
void Ogre2Camera::DestroyRenderTexture()
{
  if (this->renderTexture)
  {
    dynamic_cast<Ogre2RenderTarget *>(this->renderTexture.get())->Destroy();
    this->renderTexture.reset();
  }
}
//////////////////////////////////////////////////
unsigned int Ogre2Camera::RenderTextureGLId() const
{
  if (!this->renderTexture)
    return 0u;

  Ogre2RenderTexturePtr rt =
      std::dynamic_pointer_cast<Ogre2RenderTexture>(this->renderTexture);

  if (!rt)
    return 0u;

  return rt->GLId();
}

//////////////////////////////////////////////////
void Ogre2Camera::SetShadowsDirty()
{
  this->SetShadowsNodeDefDirty();
}

//////////////////////////////////////////////////
void Ogre2Camera::SetShadowsNodeDefDirty()
{
  if (!this->renderTexture)
    return;

  Ogre2RenderTexturePtr rt =
      std::dynamic_pointer_cast<Ogre2RenderTexture>(this->renderTexture);

  if (rt)
    rt->SetShadowsNodeDefDirty();
}

//////////////////////////////////////////////////
void Ogre2Camera::SetSelectionBuffer()
{
  this->selectionBuffer = new Ogre2SelectionBuffer(this->name, this->scene,
    this->ImageWidth(), this->ImageHeight());
}

//////////////////////////////////////////////////
Ogre2SelectionBuffer *Ogre2Camera::SelectionBuffer() const
{
  return this->selectionBuffer;
}

//////////////////////////////////////////////////
VisualPtr Ogre2Camera::VisualAt(const math::Vector2i &_mousePos)
{
  VisualPtr result;

  if (!this->selectionBuffer)
  {
    this->SetSelectionBuffer();

    if (!this->selectionBuffer)
    {
      return result;
    }
  }
  else
  {
    this->selectionBuffer->SetDimensions(
      this->ImageWidth(), this->ImageHeight());
  }

  float ratio = screenScalingFactor();
  math::Vector2i mousePos(
      static_cast<int>(std::rint(ratio * _mousePos.X())),
      static_cast<int>(std::rint(ratio * _mousePos.Y())));
  Ogre::Item *ogreItem = this->selectionBuffer->OnSelectionClick(
      mousePos.X(), mousePos.Y());

  if (ogreItem)
  {
    if (!ogreItem->getUserObjectBindings().getUserAny().isEmpty() &&
        ogreItem->getUserObjectBindings().getUserAny().getType() ==
        typeid(unsigned int))
    {
      try
      {
        result = this->scene->VisualById(Ogre::any_cast<unsigned int>(
              ogreItem->getUserObjectBindings().getUserAny()));
      }
      catch(Ogre::Exception &e)
      {
        ignerr << "Ogre Error:" << e.getFullDescription() << "\n";
      }
    }
  }

  return result;
}

//////////////////////////////////////////////////
RenderWindowPtr Ogre2Camera::CreateRenderWindow()
{
  RenderWindowPtr base = this->scene->CreateRenderWindow();
  Ogre2RenderWindowPtr renderWindow =
      std::dynamic_pointer_cast<Ogre2RenderWindow>(base);
  renderWindow->SetWidth(this->ImageWidth());
  renderWindow->SetHeight(this->ImageHeight());
  renderWindow->SetDevicePixelRatio(1);
  renderWindow->SetCamera(this->ogreCamera);
  renderWindow->SetBackgroundColor(this->scene->BackgroundColor());

  this->renderTexture = renderWindow;
  return base;
}

//////////////////////////////////////////////////
math::Matrix4d Ogre2Camera::ProjectionMatrix() const
{
  return Ogre2Conversions::Convert(this->ogreCamera->getProjectionMatrix());
}

//////////////////////////////////////////////////
math::Matrix4d Ogre2Camera::ViewMatrix() const
{
  return Ogre2Conversions::Convert(this->ogreCamera->getViewMatrix(true));
}

//////////////////////////////////////////////////
void Ogre2Camera::SetProjectionMatrix(const math::Matrix4d &_matrix)
{
  BaseCamera::SetProjectionMatrix(_matrix);
  this->ogreCamera->setCustomProjectionMatrix(true,
      Ogre2Conversions::Convert(this->projectionMatrix));
}

//////////////////////////////////////////////////
void Ogre2Camera::SetProjectionType(CameraProjectionType _type)
{
  BaseCamera::SetProjectionType(_type);
  switch (this->projectionType)
  {
    default:
    case CPT_PERSPECTIVE:
      this->ogreCamera->setProjectionType(Ogre::PT_PERSPECTIVE);
      break;
    case CPT_ORTHOGRAPHIC:
      this->ogreCamera->setProjectionType(Ogre::PT_ORTHOGRAPHIC);
      break;
  }
  // reset projection matrix when projection type changes
  this->ogreCamera->setCustomProjectionMatrix(false);
}

//////////////////////////////////////////////////
void Ogre2Camera::SetNearClipPlane(const double _near)
{
  BaseCamera::SetNearClipPlane(_near);
  this->ogreCamera->setNearClipDistance(_near);
}

//////////////////////////////////////////////////
void Ogre2Camera::SetFarClipPlane(const double _far)
{
  BaseCamera::SetFarClipPlane(_far);
  this->ogreCamera->setFarClipDistance(_far);
}

//////////////////////////////////////////////////
Ogre::Camera *Ogre2Camera::OgreCamera() const
{
  return ogreCamera;
}

//////////////////////////////////////////////////
void Ogre2Camera::SetVisibilityMask(uint32_t _mask)
{
  if (_mask & ~Ogre::VisibilityFlags::RESERVED_VISIBILITY_FLAGS)
  {
    ignwarn << "Ogre2Camera::SetVisibilityMask: Mask bits " << std::hex
            << ~Ogre::VisibilityFlags::RESERVED_VISIBILITY_FLAGS << std::dec
            << " are set but will be ignored as they conflict with the "
            << "reserved bits used internally by the ogre2 backend."
            << std::endl;
  }
  BaseSensor::SetVisibilityMask(_mask);
  if (this->renderTexture)
    this->renderTexture->SetVisibilityMask(_mask);
}
