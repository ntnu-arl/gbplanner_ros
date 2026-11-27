/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include "gz/rendering/ogre/OgreScene.hh"
#include "gz/rendering/ogre/OgreRenderTargetMaterial.hh"

using namespace gz::rendering;


//////////////////////////////////////////////////
OgreRenderTargetMaterial::OgreRenderTargetMaterial(
    OgreScenePtr _scene, Ogre::RenderTarget *_renderTarget,
    Ogre::Material *_material):
  scene(_scene), renderTarget(_renderTarget), material(_material)
{
  // Pick a name that's unlikely to collide with a real material scheme
  this->schemeName = "__ignition__rendering__OgreRenderTargetMaterial";
  this->renderTarget->getViewport(0)->setMaterialScheme(this->schemeName);
  this->renderTarget->addListener(this);
}

//////////////////////////////////////////////////
OgreRenderTargetMaterial::~OgreRenderTargetMaterial()
{
  if (this->scene->IsInitialized())
    this->renderTarget->removeListener(this);
}

//////////////////////////////////////////////////
void OgreRenderTargetMaterial::preRenderTargetUpdate(
    const Ogre::RenderTargetEvent & /*_evt*/)
{
  // this->scene->addRenderQueueListener(this);
  Ogre::MaterialManager::getSingleton().addListener(this);
}

//////////////////////////////////////////////////
void OgreRenderTargetMaterial::postRenderTargetUpdate(
    const Ogre::RenderTargetEvent & /*_evt*/)
{
  Ogre::MaterialManager::getSingleton().removeListener(this);
}

//////////////////////////////////////////////////
/// \brief Ogre callback that assigns material to new renderables
Ogre::Technique *OgreRenderTargetMaterial::handleSchemeNotFound(
    uint16_t /*_schemeIndex*/, const Ogre::String &_schemeName,
    Ogre::Material * /*_originalMaterial*/, uint16_t /*_lodIndex*/,
    const Ogre::Renderable * /*_rend*/)
{
  if (_schemeName == this->schemeName)
  {
    // not using getBestTechnique() because it leads to infinite recursion here
    return this->material->getSupportedTechnique(0);
  }
  return nullptr;
}
