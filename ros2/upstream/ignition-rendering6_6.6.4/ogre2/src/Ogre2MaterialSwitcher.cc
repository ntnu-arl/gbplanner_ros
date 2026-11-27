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
#include "gz/rendering/ogre2/Ogre2MaterialSwitcher.hh"
#include "gz/rendering/ogre2/Ogre2Scene.hh"
#include "gz/rendering/RenderTypes.hh"

#ifdef _MSC_VER
  #pragma warning(push, 0)
#endif
#include <OgreItem.h>
#include <OgreMaterialManager.h>
#include <OgrePass.h>
#include <OgreSceneManager.h>
#include <OgreTechnique.h>
#ifdef _MSC_VER
  #pragma warning(pop)
#endif

using namespace gz;
using namespace rendering;


/// \brief A map of ogre sub item pointer to their original low level material
/// \todo(anyone) Added here for ABI compatibity. Move to private class
/// in ign-rendering7
std::map<Ogre2MaterialSwitcher *,
         std::map<Ogre::SubItem *, Ogre::MaterialPtr>> materialMap;

/////////////////////////////////////////////////
Ogre2MaterialSwitcher::Ogre2MaterialSwitcher(Ogre2ScenePtr _scene)
{
  this->currentColor = math::Color(0.0f, 0.0f, 0.1f);
  this->scene = _scene;

  // plain opaque material
  Ogre::ResourcePtr res =
    Ogre::MaterialManager::getSingleton().load("ign-rendering/plain_color",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  this->plainMaterial = res.staticCast<Ogre::Material>();
  this->plainMaterial->load();

  // plain overlay material
  this->plainOverlayMaterial =
      this->plainMaterial->clone("plain_color_overlay");
  if (!this->plainOverlayMaterial->getTechnique(0) ||
      !this->plainOverlayMaterial->getTechnique(0)->getPass(0))
  {
    ignerr << "Problem creating selection buffer overlay material"
        << std::endl;
    return;
  }
  Ogre::Pass *overlayPass =
      this->plainOverlayMaterial->getTechnique(0)->getPass(0);
  Ogre::HlmsMacroblock macroblock(*overlayPass->getMacroblock());
  macroblock.mDepthCheck = false;
  macroblock.mDepthWrite = false;
  overlayPass->setMacroblock(macroblock);
}

/////////////////////////////////////////////////
Ogre2MaterialSwitcher::~Ogre2MaterialSwitcher()
{
}

////////////////////////////////////////////////
void Ogre2MaterialSwitcher::cameraPreRenderScene(
    Ogre::Camera * /*_evt*/)
{
  // swap item to use v1 shader material
  // Note: keep an eye out for performance impact on switching materials
  // on the fly. We are not doing this often so should be ok.
  auto itor = this->scene->OgreSceneManager()->getMovableObjectIterator(
      Ogre::ItemFactory::FACTORY_TYPE_NAME);
  while (itor.hasMoreElements())
  {
    this->NextColor();

    Ogre::MovableObject *object = itor.peekNext();
    Ogre::Item *item = static_cast<Ogre::Item *>(object);

    this->colorDict[this->currentColor.AsRGBA()] = item->getName();

    for (unsigned int i = 0; i < item->getNumSubItems(); ++i)
    {
      Ogre::SubItem *subItem = item->getSubItem(i);
      subItem->setCustomParameter(1,
          Ogre::Vector4(this->currentColor.R(), this->currentColor.G(),
                        this->currentColor.B(), 1.0));

      // case when item is using low level materials
      // e.g. shaders
      if (!subItem->getMaterial().isNull())
      {
        materialMap[this][subItem] = subItem->getMaterial();
        auto technique = subItem->getMaterial()->getTechnique(0);

        if (technique && !technique->isDepthWriteEnabled() &&
            !technique->isDepthCheckEnabled())
        {
          subItem->setMaterial(this->plainOverlayMaterial);
        }
        else
        {
          subItem->setMaterial(this->plainMaterial);
        }
      }
      // regular Pbs Hlms datablock
      else
      {
        Ogre::HlmsDatablock *datablock = subItem->getDatablock();
        this->datablockMap[subItem] = datablock;
        // check if it's an overlay material by assuming the
        // depth check and depth write properties are off.
        if (!datablock->getMacroblock()->mDepthWrite &&
            !datablock->getMacroblock()->mDepthCheck)
          subItem->setMaterial(this->plainOverlayMaterial);
        else
          subItem->setMaterial(this->plainMaterial);
      }
    }
    itor.moveNext();
  }
}

/////////////////////////////////////////////////
void Ogre2MaterialSwitcher::cameraPostRenderScene(
    Ogre::Camera * /*_evt*/)
{
  // restore item to use hlms material
  auto itor = this->scene->OgreSceneManager()->getMovableObjectIterator(
      Ogre::ItemFactory::FACTORY_TYPE_NAME);
  while (itor.hasMoreElements())
  {
    Ogre::MovableObject *object = itor.peekNext();
    Ogre::Item *item = static_cast<Ogre::Item *>(object);
    for (unsigned int i = 0; i < item->getNumSubItems(); ++i)
    {
      Ogre::SubItem *subItem = item->getSubItem(i);
      auto it = this->datablockMap.find(subItem);
      if (it != this->datablockMap.end())
        subItem->setDatablock(it->second);
      else
      {
        auto mIt = materialMap[this].find(subItem);
        if (mIt != materialMap[this].end())
          subItem->setMaterial(mIt->second);
      }
    }
    itor.moveNext();
  }
  this->datablockMap.clear();
  materialMap[this].clear();
}

/////////////////////////////////////////////////
std::string Ogre2MaterialSwitcher::EntityName(
    const math::Color &_color) const
{
  auto iter = this->colorDict.find(_color.AsRGBA());

  if (iter != this->colorDict.end())
    return (*iter).second;
  else
    return std::string();
}

/////////////////////////////////////////////////
void Ogre2MaterialSwitcher::NextColor()
{
  auto color = this->currentColor.AsARGB();
  color++;
  this->currentColor.SetFromARGB(color);
}

/////////////////////////////////////////////////
void Ogre2MaterialSwitcher::Reset()
{
  this->currentColor = math::Color(
      0.0, 0.0, 0.0);
  this->colorDict.clear();
}
