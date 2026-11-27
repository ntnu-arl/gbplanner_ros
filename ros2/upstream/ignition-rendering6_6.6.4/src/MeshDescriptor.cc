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
#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>

#include "gz/rendering/MeshDescriptor.hh"

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
MeshDescriptor::MeshDescriptor()
{
}

//////////////////////////////////////////////////
MeshDescriptor::MeshDescriptor(const std::string &_meshName) :
  meshName(_meshName)
{
}

//////////////////////////////////////////////////
MeshDescriptor::MeshDescriptor(const common::Mesh *_mesh) :
  mesh(_mesh)
{
}

//////////////////////////////////////////////////
void MeshDescriptor::Load()
{
  if (this->mesh)
  {
    this->meshName = this->mesh->Name();
  }
  else if (!this->meshName.empty())
  {
    this->mesh = common::MeshManager::Instance()->MeshByName(this->meshName);
    if (!this->mesh)
    {
      ignerr << "Mesh manager can't find mesh named [" << this->meshName << "]"
             << std::endl;
    }
  }
  else
  {
    ignerr << "Missing mesh or mesh name" << std::endl;
  }
}
