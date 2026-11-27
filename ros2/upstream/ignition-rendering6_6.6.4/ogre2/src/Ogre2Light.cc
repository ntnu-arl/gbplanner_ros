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

#include <gz/common/Console.hh>

#include "gz/rendering/ogre2/Ogre2Light.hh"

#include "gz/rendering/ogre2/Ogre2Conversions.hh"
#include "gz/rendering/ogre2/Ogre2Scene.hh"

#ifdef _MSC_VER
  #pragma warning(push, 0)
#endif
#include <OgreSceneManager.h>
#ifdef _MSC_VER
  #pragma warning(pop)
#endif

/// \brief Private data for the Ogre2Light class
class gz::rendering::Ogre2LightPrivate
{
};

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
// Ogre2Light
//////////////////////////////////////////////////
Ogre2Light::Ogre2Light() :
  ogreLightType(Ogre::Light::LT_POINT),
  dataPtr(std::make_unique<Ogre2LightPrivate>())
{
}

//////////////////////////////////////////////////
Ogre2Light::~Ogre2Light()
{
}

//////////////////////////////////////////////////
math::Color Ogre2Light::DiffuseColor() const
{
  Ogre::ColourValue color = this->ogreLight->getDiffuseColour();
  return Ogre2Conversions::Convert(color);
}

//////////////////////////////////////////////////
void Ogre2Light::SetDiffuseColor(const math::Color &_color)
{
  this->ogreLight->setDiffuseColour(_color.R(), _color.G(), _color.B());
}

//////////////////////////////////////////////////
math::Color Ogre2Light::SpecularColor() const
{
  Ogre::ColourValue color = this->ogreLight->getSpecularColour();
  return Ogre2Conversions::Convert(color);
}

//////////////////////////////////////////////////
void Ogre2Light::SetSpecularColor(const math::Color &_color)
{
  this->ogreLight->setSpecularColour(_color.R(), _color.G(), _color.B());
}

//////////////////////////////////////////////////
double Ogre2Light::AttenuationConstant() const
{
  return this->ogreLight->getAttenuationConstant();
}

//////////////////////////////////////////////////
void Ogre2Light::SetAttenuationConstant(double _value)
{
  this->attenConstant = _value;
  this->UpdateAttenuation();
}

//////////////////////////////////////////////////
double Ogre2Light::AttenuationLinear() const
{
  return this->ogreLight->getAttenuationLinear();
}

//////////////////////////////////////////////////
void Ogre2Light::SetAttenuationLinear(double _value)
{
  this->attenLinear = _value;
  this->UpdateAttenuation();
}

//////////////////////////////////////////////////
double Ogre2Light::AttenuationQuadratic() const
{
  return this->ogreLight->getAttenuationQuadric();
}

//////////////////////////////////////////////////
void Ogre2Light::SetAttenuationQuadratic(double _value)
{
  this->attenQuadratic = _value;
  this->UpdateAttenuation();
}

//////////////////////////////////////////////////
double Ogre2Light::AttenuationRange() const
{
  return this->ogreLight->getAttenuationRange();
}

//////////////////////////////////////////////////
void Ogre2Light::SetAttenuationRange(double _range)
{
  this->attenRange = _range;
  this->UpdateAttenuation();
}

//////////////////////////////////////////////////
double Ogre2Light::Intensity() const
{
  return this->ogreLight->getPowerScale() / IGN_PI;
}

//////////////////////////////////////////////////
void Ogre2Light::SetIntensity(double _intensity)
{
  this->ogreLight->setPowerScale(_intensity * IGN_PI);
}

//////////////////////////////////////////////////
bool Ogre2Light::CastShadows() const
{
  return this->ogreLight->getCastShadows();
}

//////////////////////////////////////////////////
void Ogre2Light::SetCastShadows(bool _castShadows)
{
  this->ogreLight->setCastShadows(_castShadows);
  this->scene->SetShadowsDirty(true);
}

//////////////////////////////////////////////////
Ogre::Light *Ogre2Light::Light() const
{
  return this->ogreLight;
}

//////////////////////////////////////////////////
void Ogre2Light::Destroy()
{
  BaseLight::Destroy();
  Ogre::SceneManager *ogreSceneManager = this->scene->OgreSceneManager();
  ogreSceneManager->destroySceneNode(this->ogreLight->getParentSceneNode());
  ogreSceneManager->destroyLight(this->ogreLight);
}

//////////////////////////////////////////////////
void Ogre2Light::Init()
{
  Ogre2Node::Init();
  this->CreateLight();
  this->Reset();
}

//////////////////////////////////////////////////
void Ogre2Light::CreateLight()
{
  Ogre::SceneManager *sceneManager;
  sceneManager = this->scene->OgreSceneManager();
  this->ogreLight = sceneManager->createLight();
  this->ogreLight->setType(this->ogreLightType);
  // create an intermediate scene node to hold light object otherwise
  // functions that update the light pose will affect the light direction
  this->ogreNode->createChildSceneNode()->attachObject(this->ogreLight);
  this->ogreLight->setCastShadows(true);
  this->ogreLight->setPowerScale(Ogre::Math::PI);
  this->UpdateAttenuation();
}

//////////////////////////////////////////////////
void Ogre2Light::UpdateAttenuation()
{
  this->ogreLight->setAttenuation(this->attenRange, this->attenConstant,
      this->attenLinear, this->attenQuadratic);
}

//////////////////////////////////////////////////
// Ogre2DirectionalLight
//////////////////////////////////////////////////
Ogre2DirectionalLight::Ogre2DirectionalLight()
{
  this->ogreLightType = Ogre::Light::LT_DIRECTIONAL;
}

//////////////////////////////////////////////////
Ogre2DirectionalLight::~Ogre2DirectionalLight()
{
}

//////////////////////////////////////////////////
math::Vector3d Ogre2DirectionalLight::Direction() const
{
  return Ogre2Conversions::Convert(this->ogreLight->getDirection());
}

//////////////////////////////////////////////////
void Ogre2DirectionalLight::SetDirection(const math::Vector3d &_dir)
{
  this->ogreLight->setDirection(Ogre2Conversions::Convert(_dir));
}

//////////////////////////////////////////////////
// OgrePointLight
//////////////////////////////////////////////////
Ogre2PointLight::Ogre2PointLight()
{
  this->ogreLightType = Ogre::Light::LT_POINT;
}

//////////////////////////////////////////////////
Ogre2PointLight::~Ogre2PointLight()
{
}

//////////////////////////////////////////////////
// OgreSpotLight
//////////////////////////////////////////////////
Ogre2SpotLight::Ogre2SpotLight()
{
  this->ogreLightType = Ogre::Light::LT_SPOTLIGHT;
}

//////////////////////////////////////////////////
Ogre2SpotLight::~Ogre2SpotLight()
{
}

//////////////////////////////////////////////////
math::Vector3d Ogre2SpotLight::Direction() const
{
  return Ogre2Conversions::Convert(this->ogreLight->getDirection());
}

//////////////////////////////////////////////////
void Ogre2SpotLight::SetDirection(const math::Vector3d &_dir)
{
  this->ogreLight->setDirection(Ogre2Conversions::Convert(_dir));
}

//////////////////////////////////////////////////
math::Angle Ogre2SpotLight::InnerAngle() const
{
  return Ogre2Conversions::Convert(this->ogreLight->getSpotlightInnerAngle());
}

//////////////////////////////////////////////////
void Ogre2SpotLight::SetInnerAngle(const math::Angle &_angle)
{
  this->ogreLight->setSpotlightInnerAngle(Ogre2Conversions::Convert(_angle));
}

//////////////////////////////////////////////////
math::Angle Ogre2SpotLight::OuterAngle() const
{
  return Ogre2Conversions::Convert(this->ogreLight->getSpotlightOuterAngle());
}

//////////////////////////////////////////////////
void Ogre2SpotLight::SetOuterAngle(const math::Angle &_angle)
{
  this->ogreLight->setSpotlightOuterAngle(Ogre2Conversions::Convert(_angle));
}

//////////////////////////////////////////////////
double Ogre2SpotLight::Falloff() const
{
  return this->ogreLight->getSpotlightFalloff();
}

//////////////////////////////////////////////////
void Ogre2SpotLight::SetFalloff(double _falloff)
{
  this->ogreLight->setSpotlightFalloff(_falloff);
}

//////////////////////////////////////////////////
