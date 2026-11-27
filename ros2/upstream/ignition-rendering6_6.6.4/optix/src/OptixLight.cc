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
#include "gz/rendering/optix/OptixLight.hh"

#include "gz/rendering/optix/OptixConversions.hh"
#include "gz/rendering/optix/OptixLightManager.hh"
#include "gz/rendering/optix/OptixIncludes.hh"
#include "gz/rendering/optix/OptixScene.hh"

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
/// OptixLight
OptixLight::OptixLight()
{
}

//////////////////////////////////////////////////
OptixLight::~OptixLight()
{
}

//////////////////////////////////////////////////
math::Color OptixLight::DiffuseColor() const
{
  float4 optixColor = this->CommonData().color.diffuse;;
  return OptixConversions::ConvertColor(optixColor);
}

//////////////////////////////////////////////////
void OptixLight::SetDiffuseColor(const math::Color &_color)
{
  float4 optixColor = OptixConversions::ConvertColor(_color);
  this->CommonData().color.diffuse = optixColor;
}

//////////////////////////////////////////////////
math::Color OptixLight::SpecularColor() const
{
  float4 optixColor = this->CommonData().color.specular;;
  return OptixConversions::ConvertColor(optixColor);
}

//////////////////////////////////////////////////
void OptixLight::SetSpecularColor(const math::Color &_color)
{
  float4 optixColor = OptixConversions::ConvertColor(_color);
  this->CommonData().color.specular = optixColor;
}

//////////////////////////////////////////////////
double OptixLight::AttenuationConstant() const
{
  return this->CommonData().atten.constant;
}

//////////////////////////////////////////////////
void OptixLight::SetAttenuationConstant(double _value)
{
  this->CommonData().atten.constant = _value;
}

//////////////////////////////////////////////////
double OptixLight::AttenuationLinear() const
{
  return this->CommonData().atten.linear;
}

//////////////////////////////////////////////////
void OptixLight::SetAttenuationLinear(double _value)
{
  this->CommonData().atten.linear = _value;
}

//////////////////////////////////////////////////
double OptixLight::AttenuationQuadratic() const
{
  return this->CommonData().atten.quadratic;
}

//////////////////////////////////////////////////
void OptixLight::SetAttenuationQuadratic(double _value)
{
  this->CommonData().atten.quadratic = _value;
}

//////////////////////////////////////////////////
double OptixLight::AttenuationRange() const
{
  return this->CommonData().atten.range;
}

//////////////////////////////////////////////////
void OptixLight::SetAttenuationRange(double _range)
{
  this->CommonData().atten.range = _range;
}

//////////////////////////////////////////////////
bool OptixLight::CastShadows() const
{
  return this->CommonData().castShadows;
}

//////////////////////////////////////////////////
void OptixLight::SetCastShadows(bool _castShadows)
{
  this->CommonData().castShadows = _castShadows;
}

//////////////////////////////////////////////////
void OptixLight::WritePoseToDeviceImpl()
{
  BaseLight::WritePoseToDeviceImpl();

  // TODO: handle rotation
  math::Pose3d worldPose = this->WorldPose();
  OptixCommonLightData &_data = this->CommonData();
  _data.position.x = worldPose.Pos().X();
  _data.position.y = worldPose.Pos().Y();
  _data.position.z = worldPose.Pos().Z();
}

//////////////////////////////////////////////////
void OptixLight::Init()
{
  BaseLight::Init();
  this->Reset();
}

//////////////////////////////////////////////////
/// OptixDirectionalLight
OptixDirectionalLight::OptixDirectionalLight()
{
}

//////////////////////////////////////////////////
OptixDirectionalLight::~OptixDirectionalLight()
{
}

//////////////////////////////////////////////////
math::Vector3d OptixDirectionalLight::Direction() const
{
  return OptixConversions::ConvertVector(this->data.direction);
}

//////////////////////////////////////////////////
void OptixDirectionalLight::SetDirection(const math::Vector3d &_dir)
{
  this->data.direction = OptixConversions::ConvertVector(_dir);
}

//////////////////////////////////////////////////
OptixDirectionalLightData OptixDirectionalLight::Data() const
{
  return this->data;
}

//////////////////////////////////////////////////
void OptixDirectionalLight::PreRender()
{
  BaseDirectionalLight::PreRender();
  OptixLightManagerPtr lightManager = this->scene->LightManager();
  lightManager->AddDirectionalLight(this->SharedThis());
}

//////////////////////////////////////////////////
OptixCommonLightData &OptixDirectionalLight::CommonData()
{
  return this->data.common;
}

//////////////////////////////////////////////////
const OptixCommonLightData &OptixDirectionalLight::CommonData() const
{
  return this->data.common;
}

//////////////////////////////////////////////////
OptixDirectionalLightPtr OptixDirectionalLight::SharedThis()
{
  ObjectPtr sharedBase = this->shared_from_this();
  return std::dynamic_pointer_cast<OptixDirectionalLight>(sharedBase);
}

//////////////////////////////////////////////////
/// OptixPointLight
OptixPointLight::OptixPointLight()
{
}

//////////////////////////////////////////////////
OptixPointLight::~OptixPointLight()
{
}

//////////////////////////////////////////////////
OptixPointLightData OptixPointLight::Data() const
{
  return this->data;
}

//////////////////////////////////////////////////
void OptixPointLight::PreRender()
{
  BasePointLight::PreRender();
  OptixLightManagerPtr lightManager = this->scene->LightManager();
  lightManager->AddPointLight(this->SharedThis());
}

//////////////////////////////////////////////////
OptixCommonLightData &OptixPointLight::CommonData()
{
  return this->data.common;
}

//////////////////////////////////////////////////
const OptixCommonLightData &OptixPointLight::CommonData() const
{
  return this->data.common;
}

//////////////////////////////////////////////////
OptixPointLightPtr OptixPointLight::SharedThis()
{
  ObjectPtr sharedBase = this->shared_from_this();
  return std::dynamic_pointer_cast<OptixPointLight>(sharedBase);
}

//////////////////////////////////////////////////
// OptixSpotLight
//////////////////////////////////////////////////
OptixSpotLight::OptixSpotLight()
{
}

//////////////////////////////////////////////////
OptixSpotLight::~OptixSpotLight()
{
}

//////////////////////////////////////////////////
math::Vector3d OptixSpotLight::Direction() const
{
  return OptixConversions::ConvertVector(this->data.direction);
}

//////////////////////////////////////////////////
void OptixSpotLight::SetDirection(const math::Vector3d &_dir)
{
  this->data.direction = OptixConversions::ConvertVector(_dir);
}

//////////////////////////////////////////////////
math::Angle OptixSpotLight::InnerAngle() const
{
  return math::Angle(this->data.spot.outerAngle);
}

//////////////////////////////////////////////////
void OptixSpotLight::SetInnerAngle(const math::Angle &_angle)
{
  this->data.spot.outerAngle = _angle.Radian();
}

//////////////////////////////////////////////////
math::Angle OptixSpotLight::OuterAngle() const
{
  return math::Angle(this->data.spot.outerAngle);
}

//////////////////////////////////////////////////
void OptixSpotLight::SetOuterAngle(const math::Angle &_angle)
{
  this->data.spot.outerAngle = _angle.Radian();
}

//////////////////////////////////////////////////
double OptixSpotLight::Falloff() const
{
  return this->data.spot.falloff;
}

//////////////////////////////////////////////////
void OptixSpotLight::SetFalloff(double _falloff)
{
  this->data.spot.falloff = _falloff;
}

//////////////////////////////////////////////////
OptixSpotLightData OptixSpotLight::Data() const
{
  return this->data;
}

//////////////////////////////////////////////////
void OptixSpotLight::PreRender()
{
  BaseSpotLight::PreRender();
  OptixLightManagerPtr lightManager = this->scene->LightManager();
  lightManager->AddSpotLight(this->SharedThis());
}

//////////////////////////////////////////////////
OptixCommonLightData &OptixSpotLight::CommonData()
{
  return this->data.common;
}

//////////////////////////////////////////////////
const OptixCommonLightData &OptixSpotLight::CommonData() const
{
  return this->data.common;
}

//////////////////////////////////////////////////
OptixSpotLightPtr OptixSpotLight::SharedThis()
{
  ObjectPtr sharedBase = this->shared_from_this();
  return std::dynamic_pointer_cast<OptixSpotLight>(sharedBase);
}
