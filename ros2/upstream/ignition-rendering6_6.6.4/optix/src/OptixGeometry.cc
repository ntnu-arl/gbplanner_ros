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

#include "gz/rendering/optix/OptixGeometry.hh"

#include "gz/rendering/optix/OptixScene.hh"
#include "gz/rendering/optix/OptixVisual.hh"

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
OptixGeometry::OptixGeometry()
{
}

//////////////////////////////////////////////////
OptixGeometry::~OptixGeometry()
{
}

//////////////////////////////////////////////////
bool OptixGeometry::HasParent() const
{
  return this->parent != nullptr;
}

//////////////////////////////////////////////////
VisualPtr OptixGeometry::Parent() const
{
  return this->parent;
}

//////////////////////////////////////////////////
void OptixGeometry::SetParent(OptixVisualPtr _parent)
{
  this->parent = _parent;
}

//////////////////////////////////////////////////
void OptixGeometry::SetScale(math::Vector3d _scale)
{
  optix::GeometryGroup optixGeomGroup = this->OptixGeometryGroup();
  unsigned int count = optixGeomGroup->getChildCount();

  for (unsigned int i = 0; i < count; ++i)
  {
    optix::GeometryInstance optixGeomInstance = optixGeomGroup->getChild(i);
    optixGeomInstance["scale"]->setFloat(_scale.X(), _scale.Y(), _scale.Z());
  }
}
