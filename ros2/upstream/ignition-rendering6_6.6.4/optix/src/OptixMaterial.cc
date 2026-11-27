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
#include "gz/rendering/optix/OptixMaterial.hh"
#include "gz/rendering/optix/OptixConversions.hh"
#include "gz/rendering/optix/OptixRayTypes.hh"
#include "gz/rendering/optix/OptixRenderEngine.hh"
#include "gz/rendering/optix/OptixScene.hh"
#include "gz/rendering/optix/OptixTextureFactory.hh"

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////

const std::string OptixMaterial::PTX_FILE_BASE("OptixMaterial");

const std::string OptixMaterial::PTX_CLOSEST_HIT_FUNC("ClosestHit");

const std::string OptixMaterial::PTX_ANY_HIT_FUNC("AnyHit");

//////////////////////////////////////////////////
OptixMaterial::OptixMaterial() :
  optixMaterial(nullptr),
  optixTexture(nullptr),
  optixNormalMap(nullptr),
  optixEmptyTexture(nullptr)
{
}

//////////////////////////////////////////////////
OptixMaterial::~OptixMaterial()
{
}

//////////////////////////////////////////////////
bool OptixMaterial::LightingEnabled() const
{
  return this->lightingEnabled;
}

//////////////////////////////////////////////////
void OptixMaterial::SetLightingEnabled(const bool _enabled)
{
  this->lightingEnabled = _enabled;
  this->colorDirty = true;
}

//////////////////////////////////////////////////
math::Color OptixMaterial::Ambient() const
{
  return this->ambient;
}

//////////////////////////////////////////////////
void OptixMaterial::SetAmbient(const math::Color &_color)
{
  this->ambient = _color;
  this->colorDirty = true;
}

//////////////////////////////////////////////////
math::Color OptixMaterial::Diffuse() const
{
  return this->diffuse;
}

//////////////////////////////////////////////////
void OptixMaterial::SetDiffuse(const math::Color &_color)
{
  this->diffuse = _color;
  this->colorDirty = true;
}

//////////////////////////////////////////////////
math::Color OptixMaterial::Specular() const
{
  return this->specular;
}

//////////////////////////////////////////////////
void OptixMaterial::SetSpecular(const math::Color &_color)
{
  this->specular = _color;
  this->colorDirty = true;
}

//////////////////////////////////////////////////
math::Color OptixMaterial::Emissive() const
{
  return this->emissive;
}

//////////////////////////////////////////////////
void OptixMaterial::SetEmissive(const math::Color &_color)
{
  this->emissive = _color;
  this->colorDirty = true;
}

//////////////////////////////////////////////////
double OptixMaterial::Shininess() const
{
  return this->shininess;
}

//////////////////////////////////////////////////
void OptixMaterial::SetShininess(const double _shininess)
{
  this->shininess = _shininess;
  this->colorDirty = true;
}

//////////////////////////////////////////////////
double OptixMaterial::Transparency() const
{
  return this->transparency;
}

//////////////////////////////////////////////////
void OptixMaterial::SetTransparency(const double _transparency)
{
  this->transparency = std::min(std::max(_transparency, 0.0), 1.0);
  this->colorDirty = true;
}

//////////////////////////////////////////////////
double OptixMaterial::Reflectivity() const
{
  return this->reflectivity;
}

//////////////////////////////////////////////////
void OptixMaterial::SetReflectivity(const double _reflectivity)
{
  this->reflectivity = std::min(std::max(_reflectivity, 0.0), 1.0);
  this->colorDirty = true;
}

//////////////////////////////////////////////////
bool OptixMaterial::CastShadows() const
{
  return this->castShadows;
}

//////////////////////////////////////////////////
void OptixMaterial::SetCastShadows(const bool _castShadows)
{
  this->castShadows = _castShadows;
  this->colorDirty = true;
}

//////////////////////////////////////////////////
bool OptixMaterial::ReceiveShadows() const
{
  return this->receiveShadows;
}

//////////////////////////////////////////////////
void OptixMaterial::SetReceiveShadows(const bool _receiveShadows)
{
  this->receiveShadows = _receiveShadows;
  this->colorDirty = true;
}

//////////////////////////////////////////////////
bool OptixMaterial::ReflectionEnabled() const
{
  return this->reflectionEnabled;
}

//////////////////////////////////////////////////
void OptixMaterial::SetReflectionEnabled(const bool _reflectionEnabled)
{
  this->reflectionEnabled = _reflectionEnabled;
  this->colorDirty = true;
}

//////////////////////////////////////////////////
bool OptixMaterial::HasTexture() const
{
  return !this->textureName.empty();
}

//////////////////////////////////////////////////
std::string OptixMaterial::Texture() const
{
  return this->textureName;
}

//////////////////////////////////////////////////
void OptixMaterial::SetTexture(const std::string &_name)
{
  if (_name.empty())
  {
    this->ClearTexture();
  }
  else if (_name != this->textureName)
  {
    this->textureName = _name;
    this->textureDirty = true;
  }
}

//////////////////////////////////////////////////
void OptixMaterial::ClearTexture()
{
  if (this->HasTexture())
  {
    this->textureName = "";
    this->textureDirty = true;
  }
}

//////////////////////////////////////////////////
bool OptixMaterial::HasNormalMap() const
{
  return !this->normalMapName.empty();
}

//////////////////////////////////////////////////
std::string OptixMaterial::NormalMap() const
{
  return this->normalMapName;
}

//////////////////////////////////////////////////
void OptixMaterial::SetNormalMap(const std::string &_name)
{
  if (_name.empty())
  {
    this->ClearNormalMap();
  }
  else if (_name != this->normalMapName)
  {
    this->normalMapName = _name;
    this->normalMapDirty = true;
  }
}

//////////////////////////////////////////////////
void OptixMaterial::ClearNormalMap()
{
  if (this->HasNormalMap())
  {
    this->normalMapName = "";
    this->normalMapDirty = true;
  }
}

//////////////////////////////////////////////////
void OptixMaterial::PreRender()
{
  this->WriteColorToDevice();
  this->WriteTextureToDevice();
  this->WriteNormalMapToDevice();
}

//////////////////////////////////////////////////
void OptixMaterial::Destroy()
{
  if (this->optixTexture)
  {
    this->optixTexture->destroy();
    this->optixTexture = 0;
  }

  if (this->optixNormalMap)
  {
    this->optixNormalMap->destroy();
    this->optixNormalMap = 0;
  }

  if (this->optixEmptyTexture)
  {
    this->optixEmptyTexture->destroy();
    this->optixEmptyTexture = 0;
  }

  if (this->optixMaterial)
  {
    this->optixMaterial->destroy();
    this->optixMaterial = 0;
  }

  BaseMaterial::Destroy();
}

//////////////////////////////////////////////////
optix::Material OptixMaterial::Material() const
{
  return this->optixMaterial;
}

//////////////////////////////////////////////////
void OptixMaterial::WriteColorToDevice()
{
  if (this->colorDirty)
  {
    this->WriteColorToDeviceImpl();
    this->colorDirty = false;
  }
}

//////////////////////////////////////////////////
void OptixMaterial::WriteTextureToDevice()
{
  if (this->textureDirty)
  {
    this->WriteTextureToDeviceImpl();
    this->textureDirty = false;
  }
}

//////////////////////////////////////////////////
void OptixMaterial::WriteNormalMapToDevice()
{
  if (this->normalMapDirty)
  {
    this->WriteNormalMapToDeviceImpl();
    this->normalMapDirty = false;
  }
}

//////////////////////////////////////////////////
void OptixMaterial::WriteColorToDeviceImpl()
{
  // TODO: clean up
  this->optixMaterial["diffuse"]->setFloat(this->diffuse.R(),
      this->diffuse.G(), this->diffuse.B());
  this->optixMaterial["ambient"]->setFloat(this->ambient.R(),
      this->ambient.G(), this->ambient.B());
  this->optixMaterial["specular"]->setFloat(this->specular.R(),
      this->specular.G(), this->specular.B());
  this->optixMaterial["emissive"]->setFloat(this->emissive.R(),
      this->emissive.G(), this->emissive.B());
  this->optixMaterial["reflectivity"]->setFloat(this->reflectivity);
  this->optixMaterial["transparency"]->setFloat(this->transparency);

  // TODO: employ bool bitmask
  this->optixMaterial["lightingEnabled"]->setUint(this->lightingEnabled);
  this->optixMaterial["castShadows"]->setUint(this->castShadows);
  this->optixMaterial["receiveShadows"]->setUint(this->receiveShadows);
}

//////////////////////////////////////////////////
void OptixMaterial::WriteTextureToDeviceImpl()
{
  if (this->optixTexture)
  {
    this->optixTexture->destroy();
    this->optixTexture = 0;
  }

  if (this->textureName.empty())
  {
    this->optixMaterial["texSampler"]->setTextureSampler(
        this->optixEmptyTexture);
  }
  else
  {
    OptixTextureFactory texFactory(this->scene);
    this->optixTexture = texFactory.Create(this->textureName);
    this->optixMaterial["texSampler"]->setTextureSampler(this->optixTexture);
  }
}

//////////////////////////////////////////////////
void OptixMaterial::WriteNormalMapToDeviceImpl()
{
  if (this->optixNormalMap)
  {
    this->optixNormalMap->destroy();
    this->optixNormalMap = 0;
  }

  if (this->normalMapName.empty())
  {
    this->optixMaterial["normSampler"]->setTextureSampler(
        this->optixEmptyTexture);
  }
  else
  {
    OptixTextureFactory texFactory(this->scene);
    this->optixNormalMap = texFactory.Create(this->normalMapName);
    this->optixMaterial["normSampler"]->setTextureSampler(this->optixNormalMap);
  }
}

//////////////////////////////////////////////////
void OptixMaterial::Init()
{
  BaseMaterial::Init();
  optix::Context optixContext = this->scene->OptixContext();

  optix::Program closestHitProgram =
      this->scene->CreateOptixProgram(PTX_FILE_BASE, PTX_CLOSEST_HIT_FUNC);

  optix::Program anyHitProgram =
      this->scene->CreateOptixProgram(PTX_FILE_BASE, PTX_ANY_HIT_FUNC);

  this->optixMaterial = optixContext->createMaterial();
  optixMaterial->setClosestHitProgram(RT_RADIANCE, closestHitProgram);
  optixMaterial->setAnyHitProgram(RT_SHADOW, anyHitProgram);

  OptixTextureFactory texFactory(this->scene);
  this->optixEmptyTexture = texFactory.Create();
  this->optixMaterial["texSampler"]->setTextureSampler(this->optixEmptyTexture);

  this->Reset();
}
