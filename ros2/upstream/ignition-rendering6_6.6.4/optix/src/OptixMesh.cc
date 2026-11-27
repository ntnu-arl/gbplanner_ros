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

#include "gz/rendering/optix/OptixMesh.hh"
#include "gz/rendering/optix/OptixIncludes.hh"
#include "gz/rendering/optix/OptixMaterial.hh"
#include "gz/rendering/optix/OptixStorage.hh"

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
// OptixMesh
//////////////////////////////////////////////////
OptixMesh::OptixMesh()
{
}

//////////////////////////////////////////////////
OptixMesh::~OptixMesh()
{
}

//////////////////////////////////////////////////
optix::GeometryGroup OptixMesh::OptixGeometryGroup() const
{
  return this->optixGeomGroup;
}

//////////////////////////////////////////////////
optix::Acceleration OptixMesh::OptixAccel() const
{
  return this->optixAccel;
}

//////////////////////////////////////////////////
SubMeshStorePtr OptixMesh::SubMeshes() const
{
  return this->subMeshes;
}

//////////////////////////////////////////////////
// OptixSubMesh
//////////////////////////////////////////////////
OptixSubMesh::OptixSubMesh()
{
}

//////////////////////////////////////////////////
OptixSubMesh::~OptixSubMesh()
{
}

//////////////////////////////////////////////////
optix::GeometryInstance OptixSubMesh::OptixGeometryInstance() const
{
  return this->optixGeomInstance;
}

//////////////////////////////////////////////////
void OptixSubMesh::SetMaterialImpl(MaterialPtr _material)
{
  OptixMaterialPtr derived =
      std::dynamic_pointer_cast<OptixMaterial>(_material);

  if (!derived)
  {
    ignerr << "Cannot assign material created by another render-engine"
        << std::endl;

    return;
  }

  optix::Material optixMaterial = derived->Material();
  this->optixGeomInstance->setMaterialCount(0);
  this->optixGeomInstance->addMaterial(optixMaterial);
}
