/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "gz/rendering/ogre/OgreCamera.hh"
#include "gz/rendering/ogre/OgreConversions.hh"
#include "gz/rendering/ogre/OgreIncludes.hh"
#include "gz/rendering/ogre/OgreMaterial.hh"
#include "gz/rendering/ogre/OgreRenderTarget.hh"
#include "gz/rendering/ogre/OgreScene.hh"
#include "gz/rendering/ogre/OgreSelectionBuffer.hh"
#include "gz/rendering/Utils.hh"

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
OgreCamera::OgreCamera()
{
}

//////////////////////////////////////////////////
OgreCamera::~OgreCamera()
{
  this->Destroy();
}

//////////////////////////////////////////////////
Ogre::Camera *OgreCamera::Camera() const
{
  return this->ogreCamera;
}

//////////////////////////////////////////////////
void OgreCamera::Destroy()
{
  if (!this->ogreCamera)
    return;

  this->DestroyRenderTexture();

  Ogre::SceneManager *ogreSceneManager;
  ogreSceneManager = this->scene->OgreSceneManager();
  if (ogreSceneManager == nullptr)
  {
    ignerr << "Scene manager cannot be obtained" << std::endl;
  }
  else
  {
    if (ogreSceneManager->hasCamera(this->name))
    {
      ogreSceneManager->destroyCamera(this->name);
      this->ogreCamera = nullptr;
    }
  }
}

//////////////////////////////////////////////////
math::Angle OgreCamera::HFOV() const
{
  double vfov = this->ogreCamera->getFOVy().valueRadians();
  double hFOV = 2.0 * atan(tan(vfov / 2.0) * this->AspectRatio());
  return math::Angle(hFOV);
}

//////////////////////////////////////////////////
void OgreCamera::SetHFOV(const math::Angle &_angle)
{
  BaseCamera::SetHFOV(_angle);
  double angle = _angle.Radian();
  double vfov = 2.0 * atan(tan(angle / 2.0) / this->AspectRatio());
  this->ogreCamera->setFOVy(Ogre::Radian(vfov));
}

//////////////////////////////////////////////////
double OgreCamera::AspectRatio() const
{
  return this->ogreCamera->getAspectRatio();
}

//////////////////////////////////////////////////
void OgreCamera::SetAspectRatio(const double _ratio)
{
  BaseCamera::SetAspectRatio(_ratio);
  return this->ogreCamera->setAspectRatio(_ratio);
}

//////////////////////////////////////////////////
unsigned int OgreCamera::AntiAliasing() const
{
  return this->renderTexture->AntiAliasing();
}

//////////////////////////////////////////////////
void OgreCamera::SetAntiAliasing(const unsigned int _aa)
{
  BaseCamera::SetAntiAliasing(_aa);
  this->renderTexture->SetAntiAliasing(_aa);
}

//////////////////////////////////////////////////
math::Color OgreCamera::BackgroundColor() const
{
  return this->renderTexture->BackgroundColor();
}

/////////////////////////////////////////////////
void OgreCamera::SetBackgroundColor(const math::Color &_color)
{
  this->renderTexture->SetBackgroundColor(_color);
}

//////////////////////////////////////////////////
void OgreCamera::SetMaterial(const MaterialPtr &_material)
{
  this->renderTexture->SetMaterial(_material);
}

//////////////////////////////////////////////////
void OgreCamera::Render()
{
  this->renderTexture->Render();
}

//////////////////////////////////////////////////
RenderTargetPtr OgreCamera::RenderTarget() const
{
  return this->renderTexture;
}

//////////////////////////////////////////////////
void OgreCamera::Init()
{
  BaseCamera::Init();
  this->CreateCamera();
  this->CreateRenderTexture();
  this->Reset();
}

//////////////////////////////////////////////////
void OgreCamera::CreateCamera()
{
  // create ogre camera object
  Ogre::SceneManager *ogreSceneManager;
  ogreSceneManager = this->scene->OgreSceneManager();
  if (ogreSceneManager == nullptr)
  {
    ignerr << "Scene manager cannot be obtained" << std::endl;
  }

  this->ogreCamera = ogreSceneManager->createCamera(this->name);
  if (ogreCamera == nullptr)
  {
    ignerr << "Ogre camera cannot be created" << std::endl;
  }

  this->ogreNode->attachObject(this->ogreCamera);

  // rotate to Gazebo coordinate system
  this->ogreCamera->yaw(Ogre::Degree(-90.0));
  this->ogreCamera->roll(Ogre::Degree(-90.0));
  this->ogreCamera->setFixedYawAxis(false);

  // TODO(anyone): provide api access
  this->ogreCamera->setAutoAspectRatio(true);
  this->ogreCamera->setRenderingDistance(0);
  this->ogreCamera->setPolygonMode(Ogre::PM_SOLID);
  this->ogreCamera->setProjectionType(Ogre::PT_PERSPECTIVE);
  this->ogreCamera->setCustomProjectionMatrix(false);
}

//////////////////////////////////////////////////
void OgreCamera::CreateRenderTexture()
{
  this->DestroyRenderTexture();
  RenderTexturePtr base = this->scene->CreateRenderTexture();
  this->renderTexture = std::dynamic_pointer_cast<OgreRenderTexture>(base);
  this->renderTexture->SetCamera(this->ogreCamera);
  this->renderTexture->SetFormat(PF_R8G8B8);
  this->renderTexture->SetWidth(this->ImageWidth());
  this->renderTexture->SetHeight(this->ImageHeight());
  this->renderTexture->SetBackgroundColor(this->scene->BackgroundColor());
}

//////////////////////////////////////////////////
void OgreCamera::DestroyRenderTexture()
{
  if (this->renderTexture)
  {
    dynamic_cast<OgreRenderTarget *>(this->renderTexture.get())->Destroy();
    this->renderTexture.reset();
  }
}

//////////////////////////////////////////////////
unsigned int OgreCamera::RenderTextureGLId() const
{
  if (!this->renderTexture)
    return 0u;

  OgreRenderTexturePtr rt =
      std::dynamic_pointer_cast<OgreRenderTexture>(this->renderTexture);

  if (!rt)
    return 0u;

  return rt->GLId();
}

//////////////////////////////////////////////////
void OgreCamera::SetSelectionBuffer()
{
  this->selectionBuffer = new OgreSelectionBuffer(this->name,
      this->scene->OgreSceneManager());
}

//////////////////////////////////////////////////
VisualPtr OgreCamera::VisualAt(const math::Vector2i
    &_mousePos)
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

  float ratio = screenScalingFactor();
  math::Vector2i mousePos(
      static_cast<int>(std::rint(ratio * _mousePos.X())),
      static_cast<int>(std::rint(ratio * _mousePos.Y())));

  Ogre::Entity *entity = this->selectionBuffer->OnSelectionClick(
      mousePos.X(), mousePos.Y());

  if (entity)
  {
    if (!entity->getUserObjectBindings().getUserAny().isEmpty() &&
        entity->getUserObjectBindings().getUserAny().getType() ==
        typeid(unsigned int))
    {
      try
      {
        result = this->scene->VisualById(Ogre::any_cast<unsigned int>(
              entity->getUserObjectBindings().getUserAny()));
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
RenderWindowPtr OgreCamera::CreateRenderWindow()
{
  RenderWindowPtr base = this->scene->CreateRenderWindow();
  OgreRenderWindowPtr renderWindow =
      std::dynamic_pointer_cast<OgreRenderWindow>(base);
  renderWindow->SetWidth(this->ImageWidth());
  renderWindow->SetHeight(this->ImageHeight());
  renderWindow->SetDevicePixelRatio(1);
  renderWindow->SetCamera(this->ogreCamera);
  renderWindow->SetBackgroundColor(this->scene->BackgroundColor());

  this->renderTexture = renderWindow;
  return base;
}

//////////////////////////////////////////////////
math::Matrix4d OgreCamera::ProjectionMatrix() const
{
  return OgreConversions::Convert(this->ogreCamera->getProjectionMatrix());
}

//////////////////////////////////////////////////
math::Matrix4d OgreCamera::ViewMatrix() const
{
  return OgreConversions::Convert(this->ogreCamera->getViewMatrix(true));
}

//////////////////////////////////////////////////
void OgreCamera::SetProjectionMatrix(const math::Matrix4d &_matrix)
{
  BaseCamera::SetProjectionMatrix(_matrix);
  this->ogreCamera->setCustomProjectionMatrix(true,
      OgreConversions::Convert(this->projectionMatrix));
}

//////////////////////////////////////////////////
void OgreCamera::SetProjectionType(CameraProjectionType _type)
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
void OgreCamera::SetNearClipPlane(const double _near)
{
  // this->nearClip = _near;
  BaseCamera::SetNearClipPlane(_near);
  this->ogreCamera->setNearClipDistance(_near);
}

//////////////////////////////////////////////////
void OgreCamera::SetFarClipPlane(const double _far)
{
  BaseCamera::SetFarClipPlane(_far);
  this->ogreCamera->setFarClipDistance(_far);
}

//////////////////////////////////////////////////
double OgreCamera::NearClip() const
{
  if (this->ogreCamera)
    return this->ogreCamera->getNearClipDistance();
  else
    return 0;
}

//////////////////////////////////////////////////
double OgreCamera::FarClip() const
{
  if (this->ogreCamera)
    return this->ogreCamera->getFarClipDistance();
  else
    return 0;
}

//////////////////////////////////////////////////
void OgreCamera::SetVisibilityMask(uint32_t _mask)
{
  BaseCamera::SetVisibilityMask(_mask);
  this->renderTexture->SetVisibilityMask(_mask);
}
