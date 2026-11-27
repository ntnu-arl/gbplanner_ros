/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "gz/rendering/RenderPassSystem.hh"

using namespace gz;
using namespace rendering;

/// \brief Private implementation of the RenderPassSystem class
class gz::rendering::RenderPassSystemPrivate
{
};

std::map<std::string, RenderPassFactory *> RenderPassSystem::renderPassMap;

//////////////////////////////////////////////////
// RenderPassSystem
//////////////////////////////////////////////////
RenderPassSystem::RenderPassSystem() :
  dataPtr(new RenderPassSystemPrivate)
{
}

//////////////////////////////////////////////////
RenderPassSystem::~RenderPassSystem()
{
}

//////////////////////////////////////////////////
RenderPassPtr RenderPassSystem::CreateImpl(const std::string &_type)
{
  RenderPassPtr pass;
  auto it = renderPassMap.find(_type);
  if (it != renderPassMap.end())
  {
    pass.reset(it->second->New());
  }
  else
  {
    ignwarn << "RenderPass of typeid '" << _type << "' is not registered"
            << std::endl;
  }
  return pass;
}

//////////////////////////////////////////////////
void RenderPassSystem::Register(const std::string &_name,
    RenderPassFactory *_factory)
{
  renderPassMap[_name] = _factory;
}
