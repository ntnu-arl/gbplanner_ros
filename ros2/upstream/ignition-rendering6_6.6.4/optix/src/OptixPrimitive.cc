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

#include <gz/common/Console.hh>

#include "gz/rendering/optix/OptixPrimitive.hh"
#include "gz/rendering/optix/OptixMaterial.hh"
#include "gz/rendering/optix/OptixScene.hh"

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////

const std::string OptixPrimitive::PTX_INTERSECT_FUNC("Intersect");

const std::string OptixPrimitive::PTX_BOUNDS_FUNC("Bounds");

//////////////////////////////////////////////////
OptixPrimitive::OptixPrimitive()
{
}

//////////////////////////////////////////////////
OptixPrimitive::~OptixPrimitive()
{
}

//////////////////////////////////////////////////
MaterialPtr OptixPrimitive::Material() const
{
  return this->material;
}

//////////////////////////////////////////////////
void OptixPrimitive::SetMaterial(MaterialPtr _material, bool unique)
{
  // TODO: clean up

  if (!_material)
  {
    ignerr << "Cannot assign null material" << std::endl;
    return;
  }

  _material = (unique) ? _material->Clone() : _material;

  OptixMaterialPtr derived =
      std::dynamic_pointer_cast<OptixMaterial>(_material);

  if (!derived)
  {
    ignerr << "Cannot assign material created by another render-engine"
           << std::endl;

    return;
  }

  this->SetMaterialImpl(derived);
}

//////////////////////////////////////////////////
optix::GeometryGroup OptixPrimitive::OptixGeometryGroup() const
{
  return this->optixGeomGroup;
}

//////////////////////////////////////////////////
void OptixPrimitive::PreRender()
{
  OptixGeometry::PreRender();
  this->material->PreRender();
}

//////////////////////////////////////////////////
optix::Acceleration OptixPrimitive::OptixAccel() const
{
  return this->optixAccel;
}

//////////////////////////////////////////////////
optix::GeometryInstance OptixPrimitive::OptixGeometryInstance() const
{
  return this->optixGeomInstance;
}

//////////////////////////////////////////////////
optix::Geometry OptixPrimitive::OptixGeometry() const
{
  return this->optixGeometry;
}

//////////////////////////////////////////////////
void OptixPrimitive::SetMaterialImpl(OptixMaterialPtr _material)
{
  this->optixGeomInstance->setMaterialCount(0);
  this->optixGeomInstance->addMaterial(_material->Material());
  this->material = _material;
}

//////////////////////////////////////////////////
void OptixPrimitive::Init()
{
  OptixGeometry::Init();
  optix::Context optixContext = this->scene->OptixContext();
  this->optixGeomInstance = optixContext->createGeometryInstance();
  this->optixGeomInstance->setGeometry(this->optixGeometry);
  this->optixAccel = optixContext->createAcceleration("Bvh", "Bvh");
  this->optixGeomGroup = optixContext->createGeometryGroup();
  this->optixGeomGroup->addChild(this->optixGeomInstance);
  this->optixGeomGroup->setAcceleration(this->optixAccel);
}

//////////////////////////////////////////////////
optix::Geometry OptixPrimitive::CreateOptixGeometry(OptixScenePtr _scene,
    const std::string &_ptxFile)
{
  // create geometry programs
  optix::Program interProgram, boundsProgram;
  interProgram = _scene->CreateOptixProgram(_ptxFile, PTX_INTERSECT_FUNC);
  boundsProgram = _scene->CreateOptixProgram(_ptxFile, PTX_BOUNDS_FUNC);

  // create actual geometry
  optix::Context optixContext = _scene->OptixContext();
  optix::Geometry geometry = optixContext->createGeometry();
  geometry->setIntersectionProgram(interProgram);
  geometry->setBoundingBoxProgram(boundsProgram);
  geometry->setPrimitiveCount(1);

  return geometry;
}
