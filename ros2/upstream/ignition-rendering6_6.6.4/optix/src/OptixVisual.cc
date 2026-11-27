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

#include "gz/rendering/optix/OptixVisual.hh"
#include "gz/rendering/optix/OptixConversions.hh"
#include "gz/rendering/optix/OptixStorage.hh"

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
OptixVisual::OptixVisual()
{
  // TODO: move defaults to BaseVisual
}

//////////////////////////////////////////////////
OptixVisual::~OptixVisual()
{
}

//////////////////////////////////////////////////
optix::Group OptixVisual::OptixGroup() const
{
  return this->optixGroup;
}

//////////////////////////////////////////////////
optix::Acceleration OptixVisual::OptixAccel() const
{
  return this->optixAccel;
}

//////////////////////////////////////////////////
void OptixVisual::PreRender()
{
  BaseVisual::PreRender();

  // TODO: optimize this funtionality
  math::Vector3d worldScale = this->WorldScale();

  for (unsigned int i = 0; i < this->GeometryCount(); ++i)
  {
    OptixGeometryPtr geometry = this->geometries->DerivedByIndex(i);
    geometry->SetScale(worldScale);
  }
}

//////////////////////////////////////////////////
GeometryStorePtr OptixVisual::Geometries() const
{
  return this->geometries;
}

//////////////////////////////////////////////////
bool OptixVisual::AttachGeometry(GeometryPtr _geometry)
{
  OptixGeometryPtr derived =
      std::dynamic_pointer_cast<OptixGeometry>(_geometry);

  if (!derived)
  {
    ignerr << "Cannot attach geometry created by another render-engine"
          << std::endl;

    return false;
  }

  derived->SetParent(this->SharedThis());
  optix::GeometryGroup childGeomGroup = derived->OptixGeometryGroup();
  this->optixGroup->addChild(childGeomGroup);
  this->optixAccel->markDirty();
  return true;
}

//////////////////////////////////////////////////
bool OptixVisual::DetachGeometry(GeometryPtr _geometry)
{
  OptixGeometryPtr derived =
      std::dynamic_pointer_cast<OptixGeometry>(_geometry);

  if (!derived)
  {
    ignerr << "Cannot detach geometry created by another render-engine"
          << std::endl;

    return false;
  }

  this->optixGroup->removeChild(derived->OptixGeometryGroup());
  this->optixAccel->markDirty();
  return true;
}

//////////////////////////////////////////////////
void OptixVisual::Init()
{
  BaseVisual::Init();
  this->CreateStorage();
}

//////////////////////////////////////////////////
void OptixVisual::CreateStorage()
{
  this->geometries = OptixGeometryStorePtr(new OptixGeometryStore);
}

//////////////////////////////////////////////////
OptixVisualPtr OptixVisual::SharedThis()
{
  return std::dynamic_pointer_cast<OptixVisual>(shared_from_this());
}
