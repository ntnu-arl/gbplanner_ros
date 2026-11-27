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

#include "gz/common/Console.hh"
#include "gz/rendering/ogre/OgreIncludes.hh"
#include "gz/rendering/ogre/OgreMaterialSwitcher.hh"
#include "gz/rendering/RenderTypes.hh"

using namespace gz;
using namespace rendering;


/////////////////////////////////////////////////
OgreMaterialSwitcher::OgreMaterialSwitcher()
  : lastTechnique(nullptr)
{
  this->currentColor = math::Color(0.0f, 0.0f, 0.1f);
}

/////////////////////////////////////////////////
OgreMaterialSwitcher::~OgreMaterialSwitcher()
{
}

/////////////////////////////////////////////////
Ogre::Technique *OgreMaterialSwitcher::handleSchemeNotFound(
    uint16_t /*_schemeIndex*/, const Ogre::String &_schemeName,
    Ogre::Material *_originalMaterial, uint16_t /*_lodIndex*/,
    const Ogre::Renderable *_rend)
{
  // selection buffer: check scheme name against the one specified in
  // OgreSelectionBuffer::CreateRTTBuffer. Only proceed if this is a callback
  // from the selection camera.
  if (_schemeName != "selection")
    return nullptr;

  if (!_rend || typeid(*_rend) != typeid(Ogre::SubEntity))
    return nullptr;

  const Ogre::SubEntity *subEntity =
    static_cast<const Ogre::SubEntity *>(_rend);

  if (!(subEntity->getParent()->getVisibilityFlags() &
      IGN_VISIBILITY_SELECTABLE))
  {
    const_cast<Ogre::SubEntity *>(subEntity)->setCustomParameter(1,
        Ogre::Vector4(0, 0, 0, 0));
    return nullptr;
  }

  if (this->lastEntity == subEntity->getParent()->getName())
  {
    const_cast<Ogre::SubEntity *>(subEntity)->setCustomParameter(1,
        Ogre::Vector4(this->currentColor.R(), this->currentColor.G(),
                      this->currentColor.B(), 1.0));
  }
  else
  {
    // load the selection buffer material
    if (this->plainTechnique == nullptr)
    {
      // plain opaque material
      Ogre::ResourcePtr res =
        Ogre::MaterialManager::getSingleton().load("ign-rendering/plain_color",
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

      // OGRE 1.9 changes the shared pointer definition
      #if OGRE_VERSION_LT_1_11_0
      Ogre::MaterialPtr plainMaterial = res.staticCast<Ogre::Material>();
      #else
      Ogre::MaterialPtr plainMaterial =
        std::static_pointer_cast<Ogre::Material>(res);
      #endif

      this->plainTechnique = plainMaterial->getTechnique(0);
      Ogre::Pass *plainPass = this->plainTechnique->getPass(0);
      plainPass->setDepthCheckEnabled(true);
      plainPass->setDepthWriteEnabled(true);

      // overlay material
      Ogre::MaterialPtr overlayMaterial =
          plainMaterial->clone("plain_color_overlay");
      this->overlayTechnique =
          overlayMaterial->getTechnique(0);
      if (!this->overlayTechnique || !this->overlayTechnique->getPass(0))
      {
        ignerr << "Problem creating the selection buffer overlay material"
            << std::endl;
        return nullptr;
      }
      Ogre::Pass *overlayPass = this->overlayTechnique->getPass(0);
      overlayPass->setDepthCheckEnabled(false);
      overlayPass->setDepthWriteEnabled(false);
    }

    // Make sure we keep the same depth properties so that
    // certain overlay objects can be picked by the mouse.
    Ogre::Technique *newTechnique = this->plainTechnique;

    Ogre::Technique *originalTechnique = _originalMaterial->getTechnique(0);
    if (originalTechnique)
    {
      Ogre::Pass *originalPass = originalTechnique->getPass(0);
      if (originalPass)
      {
        // check if it's an overlay material by assuming the
        // depth check and depth write properties are off.
        bool depthCheck = originalPass->getDepthCheckEnabled();
        bool depthWrite = originalPass->getDepthWriteEnabled();
        if (!depthCheck && !depthWrite)
          newTechnique = this->overlayTechnique;
      }
    }

    this->lastTechnique = newTechnique;

    this->NextColor();

    const_cast<Ogre::SubEntity *>(subEntity)->setCustomParameter(1,
        Ogre::Vector4(this->currentColor.R(), this->currentColor.G(),
          this->currentColor.B(), 1.0));

    this->lastEntity = subEntity->getParent()->getName();
    this->colorDict[this->currentColor.AsRGBA()] = this->lastEntity;
  }

  return this->lastTechnique;
}

/////////////////////////////////////////////////
void OgreMaterialSwitcher::preRenderTargetUpdate(
    const Ogre::RenderTargetEvent &/*_evt*/)
{
  Ogre::MaterialManager::getSingleton().addListener(this);
}

/////////////////////////////////////////////////
void OgreMaterialSwitcher::postRenderTargetUpdate(
    const Ogre::RenderTargetEvent &/*_evt*/)
{
  Ogre::MaterialManager::getSingleton().removeListener(this);
}

/////////////////////////////////////////////////
std::string OgreMaterialSwitcher::EntityName(
    const math::Color &_color) const
{
  auto iter = this->colorDict.find(_color.AsRGBA());

  if (iter != this->colorDict.end())
    return (*iter).second;
  else
    return std::string();
}

/////////////////////////////////////////////////
void OgreMaterialSwitcher::NextColor()
{
  auto color = this->currentColor.AsARGB();
  color++;
  this->currentColor.SetFromARGB(color);
}

/////////////////////////////////////////////////
void OgreMaterialSwitcher::Reset()
{
  this->currentColor = math::Color(0.0, 0.0, 0.0);
  this->lastTechnique = nullptr;
  this->lastEntity.clear();
  this->colorDict.clear();
}
