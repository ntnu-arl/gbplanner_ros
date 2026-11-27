/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#include "LuxCoreEngineLight.hh"
#include "LuxCoreEngineScene.hh"

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
math::Color LuxCoreEngineLight::DiffuseColor() const
{
  // todo(anyone) implement this function
  return math::Color::White;
}

//////////////////////////////////////////////////
void LuxCoreEngineLight::SetDiffuseColor(double _r, double _g, double _b,
                                         double /*_a*/)
{
  this->gainR = _r;
  this->gainG = _g;
  this->gainB = _b;

  this->UpdateLuxSDL();
}

//////////////////////////////////////////////////
void LuxCoreEngineLight::SetDiffuseColor(const math::Color &/*_color*/)
{
  // todo(anyone) Implement this function
}

//////////////////////////////////////////////////
math::Color LuxCoreEngineLight::SpecularColor() const
{
  // todo(anyone) Implement this function
  return math::Color::White;
}

//////////////////////////////////////////////////
void LuxCoreEngineLight::SetSpecularColor(const math::Color &/*_color*/)
{
  // todo(anyone) Implement this function
}

//////////////////////////////////////////////////
double LuxCoreEngineLight::AttenuationConstant() const
{
  // todo(anyone) Implement this function
  return 0.0;
}

//////////////////////////////////////////////////
void LuxCoreEngineLight::SetAttenuationConstant(double /*_value*/)
{
  // todo(anyone) Implement this function
}

//////////////////////////////////////////////////
double LuxCoreEngineLight::AttenuationLinear() const
{
  // todo(anyone) Implement this function
  return 0.0;
}

//////////////////////////////////////////////////
void LuxCoreEngineLight::SetAttenuationLinear(double /*_value*/)
{
  // todo(anyone) Implement this function
}

//////////////////////////////////////////////////
double LuxCoreEngineLight::AttenuationQuadratic() const
{
  // todo(anyone) Implement this function
  return 0.0;
}

//////////////////////////////////////////////////
void LuxCoreEngineLight::SetAttenuationQuadratic(double /*_value*/)
{
  // todo(anyone) Implement this function
}

//////////////////////////////////////////////////
double LuxCoreEngineLight::AttenuationRange() const
{
  // todo(anyone) Implement this function
  return 0.0;
}

//////////////////////////////////////////////////
void LuxCoreEngineLight::SetAttenuationRange(double /*_range*/)
{
  // todo(anyone) Implement this function
}

//////////////////////////////////////////////////
bool LuxCoreEngineLight::CastShadows() const
{
  // todo(anyone) Implement this function
  return false;
}

//////////////////////////////////////////////////
void LuxCoreEngineLight::SetCastShadows(bool /*_castShadows*/)
{
  // todo(anyone) Implement this function
}

//////////////////////////////////////////////////
double LuxCoreEngineLight::Intensity() const
{
  // todo(anyone) Implement this function
  return 0.0;
}

//////////////////////////////////////////////////
void LuxCoreEngineLight::SetIntensity(double /*_intensity*/)
{
  // todo(anyone) Implement this function
}

//////////////////////////////////////////////////
LuxCoreEngineDirectionalLight::LuxCoreEngineDirectionalLight(
    std::string lightType)
{
  this->lightType = lightType;
}

//////////////////////////////////////////////////
math::Vector3d LuxCoreEngineDirectionalLight::Direction() const
{
  // todo(anyone) Implement this function
  return math::Vector3d::Zero;
}

//////////////////////////////////////////////////
void LuxCoreEngineDirectionalLight::SetDirection(double _x, double _y,
                                                 double _z)
{
  this->directionX = _x;
  this->directionY = _y;
  this->directionZ = -_z;

  this->UpdateLuxSDL();
}

//////////////////////////////////////////////////
void LuxCoreEngineDirectionalLight::SetDirection(const math::Vector3d &/*_dir*/)
{
  // todo(anyone) Implement this function
}

//////////////////////////////////////////////////
void LuxCoreEngineDirectionalLight::UpdateLuxSDL()
{
  scene->SceneLux()->Parse(
      luxrays::Property("scene.lights." + Name() + ".type")(this->lightType)
      << luxrays::Property("scene.lights." + Name() +
                           ".gain")(this->gainR, this->gainG, this->gainB)
      << luxrays::Property("scene.lights." + Name() + ".dir")(
             this->directionX, this->directionY, this->directionZ));
}

//////////////////////////////////////////////////
LuxCoreEnginePointLight::LuxCoreEnginePointLight(std::string lightType)
{
  this->lightType = lightType;
}

//////////////////////////////////////////////////
void LuxCoreEnginePointLight::SetLocalPosition(double _x, double _y,
                                               double _z)
{
  this->localPositionX = _x;
  this->localPositionY = _y;
  this->localPositionZ = _z;

  this->UpdateLuxSDL();
}

//////////////////////////////////////////////////
void LuxCoreEnginePointLight::UpdateLuxSDL()
{
  scene->SceneLux()->Parse(
      luxrays::Property("scene.lights." + Name() + ".type")(this->lightType)
      << luxrays::Property("scene.lights." + Name() +
                           ".gain")(this->gainR, this->gainG, this->gainB)
      << luxrays::Property("scene.lights." + Name() + ".position")(
             this->localPositionX, this->localPositionY, this->localPositionZ)
      << luxrays::Property("scene.lights." + Name() + ".power")(20)
      << luxrays::Property("scene.lights." + Name() + ".efficency")(20));
}
