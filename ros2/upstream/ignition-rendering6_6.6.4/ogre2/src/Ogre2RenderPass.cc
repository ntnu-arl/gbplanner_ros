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
#include "gz/rendering/ogre2/Ogre2RenderPass.hh"

/// \brief Private data for the Ogre2RenderPass class
class ignition::rendering::Ogre2RenderPassPrivate
{
};

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
Ogre2RenderPass::Ogre2RenderPass()
  : dataPtr(std::make_unique<Ogre2RenderPassPrivate>())
{
}

//////////////////////////////////////////////////
Ogre2RenderPass::~Ogre2RenderPass()
{
}

//////////////////////////////////////////////////
void Ogre2RenderPass::Destroy()
{
}

//////////////////////////////////////////////////
void Ogre2RenderPass::CreateRenderPass()
{
  // To be overriden by derived render pass classes
}

//////////////////////////////////////////////////
std::string Ogre2RenderPass::OgreCompositorNodeDefinitionName() const
{
  return this->ogreCompositorNodeDefName;
}
