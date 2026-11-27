/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include "gz/rendering/ogre2/Ogre2RenderTargetMaterial.hh"

#ifdef _MSC_VER
  #pragma warning(push, 0)
#endif
#include <OgreSceneManager.h>
#include <OgreViewport.h>
#ifdef _MSC_VER
  #pragma warning(pop)
#endif

using namespace gz::rendering;


//////////////////////////////////////////////////
Ogre2RenderTargetMaterial::Ogre2RenderTargetMaterial(
    Ogre::SceneManager *_scene, Ogre::Camera *_renderCamera,
    Ogre::Material *_material):
  scene(_scene), renderCamera(_renderCamera), material(_material)
{
  // Pick a name that's unlikely to collide with a real material scheme
  this->schemeName = "__ignition__rendering__Ogre2RenderTargetMaterial";
  Ogre::Viewport* renderViewport =
    _scene->getCurrentViewport0();
  renderViewport->setMaterialScheme(this->schemeName);
  this->renderCamera->addListener(this);
}

//////////////////////////////////////////////////
Ogre2RenderTargetMaterial::~Ogre2RenderTargetMaterial()
{
  this->renderCamera->removeListener(this);
}

//////////////////////////////////////////////////
void Ogre2RenderTargetMaterial::cameraPreRenderScene(
    Ogre::Camera * /*_evt*/)
{
  Ogre::MaterialManager::getSingleton().addListener(this);
}

//////////////////////////////////////////////////
void Ogre2RenderTargetMaterial::cameraPostRenderScene(
    Ogre::Camera * /*_evt*/)
{
  Ogre::MaterialManager::getSingleton().removeListener(this);
}

//////////////////////////////////////////////////
/// \brief Ogre callback that assigns material to new renderables
Ogre::Technique *Ogre2RenderTargetMaterial::handleSchemeNotFound(
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

