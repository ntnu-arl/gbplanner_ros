/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#include "Ogre2BoundingBoxMaterialSwitcher.hh"

#include "ignition/rendering/ogre2/Ogre2Scene.hh"
#include "ignition/rendering/ogre2/Ogre2Visual.hh"

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

using namespace ignition;
using namespace rendering;

/////////////////////////////////////////////////
Ogre2BoundingBoxMaterialSwitcher::Ogre2BoundingBoxMaterialSwitcher(
    Ogre2ScenePtr _scene)
{
  this->scene = _scene;

  // plain material to switch item's material
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
    ignerr << "Problem creating bounding box camera overlay material"
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
Ogre2BoundingBoxMaterialSwitcher::~Ogre2BoundingBoxMaterialSwitcher()
{
}

////////////////////////////////////////////////
VisualPtr Ogre2BoundingBoxMaterialSwitcher::TopLevelModelVisual(
    VisualPtr _visual) const
{
  if (!_visual)
    return _visual;
  VisualPtr p = _visual;
  while (p->Parent() && p->Parent() != _visual->Scene()->RootVisual())
    p = std::dynamic_pointer_cast<Visual>(p->Parent());
  return p;
}

////////////////////////////////////////////////
void Ogre2BoundingBoxMaterialSwitcher::cameraPreRenderScene(
    Ogre::Camera * /*_cam*/)
{
  this->datablockMap.clear();
  auto itor = this->scene->OgreSceneManager()->getMovableObjectIterator(
      Ogre::ItemFactory::FACTORY_TYPE_NAME);

  while (itor.hasMoreElements())
  {
    Ogre::MovableObject *object = itor.peekNext();
    Ogre::Item *item = static_cast<Ogre::Item *>(object);

    // get visual from ogre item
    Ogre::Any userAny = item->getUserObjectBindings().getUserAny();

    if (!userAny.isEmpty() && userAny.getType() == typeid(unsigned int))
    {
      VisualPtr visual;
      try
      {
        visual = this->scene->VisualById(Ogre::any_cast<unsigned int>(userAny));
      }
      catch(Ogre::Exception &e)
      {
        ignerr << "Ogre Error:" << e.getFullDescription() << "\n";
      }
      Ogre2VisualPtr ogreVisual = std::dynamic_pointer_cast<Ogre2Visual>(
        visual);

      // get class user data
      Variant labelAny = ogreVisual->UserData(this->labelKey);

      int label = this->backgroundLabel;
      try
      {
        label = std::get<int>(labelAny);
      }
      catch(std::bad_variant_access &e)
      {
        // items with no class are considered background
        label = this->backgroundLabel;
      }

      // for full bbox, each pixel contains 1 channel for label
      // and 2 channels stores ogreId
      uint32_t ogreId = item->getId();

      float labelColor = label / 255.0;
      float ogreId1 = (ogreId / 256) / 255.0;
      float ogreId2 = (ogreId % 256) / 255.0;

      // Material color
      auto customParameter = Ogre::Vector4(ogreId2, ogreId1, labelColor, 1.0);

      // Multi-links models handeling
      auto itemName = visual->Name();
      std::string parentName = this->TopLevelModelVisual(visual)->Name();

      this->ogreIdName[ogreId] = parentName;

      // Switch material for all sub items
      for (unsigned int i = 0; i < item->getNumSubItems(); i++)
      {
        // save subitems material
        Ogre::SubItem *subItem = item->getSubItem(i);
        Ogre::HlmsDatablock *datablock = subItem->getDatablock();
        this->datablockMap[subItem] = datablock;

        subItem->setCustomParameter(1, customParameter);

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
void Ogre2BoundingBoxMaterialSwitcher::cameraPostRenderScene(
    Ogre::Camera * /*_cam*/)
{
  // restore the original material
  for (auto it : this->datablockMap)
  {
    Ogre::SubItem *subItem = it.first;
    subItem->setDatablock(it.second);
  }
}
