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
#include "LuxCoreEngineVisual.hh"

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
void LuxCoreEngineVisual::SetLocalPosition(double _x, double _y, double _z)
{
  math::Vector3 rotation = this->pose.Rot().Euler();
  this->pose.Set(_x, _y, _z, rotation[0], rotation[1], rotation[2]);

  for (auto iter = this->geometries->Begin(); iter != this->geometries->End();
       ++iter)
  {
    iter->second->SetLocalPosition(_x, _y, _z);
  }
}

//////////////////////////////////////////////////
void LuxCoreEngineVisual::SetLocalScale(double _x, double _y, double _z)
{
  for (auto iter = this->geometries->Begin(); iter != this->geometries->End();
       ++iter)
  {
    iter->second->SetLocalScale(_x, _y, _z);
  }
}

//////////////////////////////////////////////////
void LuxCoreEngineVisual::SetLocalRotation(double _r, double _p, double _y)
{
  for (auto iter = this->geometries->Begin(); iter != this->geometries->End();
       ++iter)
  {
    iter->second->SetLocalRotation(_r, _p, _y);
  }
}

//////////////////////////////////////////////////
void LuxCoreEngineVisual::SetMaterial(MaterialPtr _material, bool _unique)
{
  for (auto iter = this->geometries->Begin(); iter != this->geometries->End();
       ++iter)
  {
    iter->second->SetMaterial(_material, _unique);
  }
}

//////////////////////////////////////////////////
GeometryStorePtr LuxCoreEngineVisual::Geometries() const
{
  return this->geometries;
}

//////////////////////////////////////////////////
bool LuxCoreEngineVisual::AttachGeometry(GeometryPtr _geometry)
{
  return true;
}

//////////////////////////////////////////////////
bool LuxCoreEngineVisual::DetachGeometry(GeometryPtr _geometry)
{
  return true;
}

//////////////////////////////////////////////////
void LuxCoreEngineVisual::Init()
{
  BaseVisual::Init();
  this->CreateStorage();
}

//////////////////////////////////////////////////
void LuxCoreEngineVisual::CreateStorage()
{
  this->geometries =
      LuxCoreEngineGeometryStorePtr(new LuxCoreEngineGeometryStore);
}

//////////////////////////////////////////////////
LuxCoreEngineVisualPtr LuxCoreEngineVisual::SharedThis()
{
  return std::dynamic_pointer_cast<LuxCoreEngineVisual>(shared_from_this());
}
