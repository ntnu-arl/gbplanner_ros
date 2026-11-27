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

#include "Ogre2SegmentationMaterialSwitcher.hh"

#include <algorithm>
#include <utility>
#include <vector>

#include <ignition/common/Console.hh>

#include "gz/rendering/ogre2/Ogre2Heightmap.hh"
#include "gz/rendering/ogre2/Ogre2Scene.hh"
#include "gz/rendering/ogre2/Ogre2Visual.hh"
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

using namespace ignition;
using namespace rendering;

/////////////////////////////////////////////////
Ogre2SegmentationMaterialSwitcher::Ogre2SegmentationMaterialSwitcher(
  Ogre2ScenePtr _scene, SegmentationCamera *_camera)
{
  this->scene = _scene;
  this->segmentationCamera = _camera;

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
Ogre2SegmentationMaterialSwitcher::~Ogre2SegmentationMaterialSwitcher()
{
  this->segmentationCamera = nullptr;
}

/////////////////////////////////////////////////
bool Ogre2SegmentationMaterialSwitcher::IsTakenColor(const math::Color &_color)
{
  // Get the int value of the 24 bit color
  // Multiply by 255 as color values are normalized
  int64_t colorId = static_cast<int64_t>((_color.R() * 255) * 256 * 256 +
    (_color.G() * 255) * 256 + (_color.B() * 255));

  if (this->takenColors.count(colorId))
  {
    return true;
  }
  else
  {
    this->takenColors.insert(colorId);
    return false;
  }
}

/////////////////////////////////////////////////
math::Color Ogre2SegmentationMaterialSwitcher::LabelToColor(int64_t _label,
  bool _isMultiLink)
{
  if (_label == this->segmentationCamera->BackgroundLabel())
    return this->segmentationCamera->BackgroundColor();

  // use label as seed to generate the same color for the label
  this->generator.seed(_label);
  std::uniform_int_distribution<int> distribution(0, 255);

  // random color
  int r = distribution(this->generator);
  int g = distribution(this->generator);
  int b = distribution(this->generator);

  math::Color color(
    static_cast<float>(r),
    static_cast<float>(g),
    static_cast<float>(b));

  // if the label is colored before return the color
  // don't check for taken colors in that case, all items
  // with the same label will have the same color
  if (this->segmentationCamera->Type() == SegmentationType::ST_SEMANTIC &&
    this->coloredLabel.count(_label))
    return color;

  if (_isMultiLink)
    return color;

  // loop recursivly till finding a unique color
  if (this->IsTakenColor(color))
      return this->LabelToColor(_label);

  this->coloredLabel.insert(_label);

  // We don't multiply by 255 here as (r,g,b) are in [0-255] range
  int64_t colorId = r * 256 * 256 + g * 256 + b;
  this->colorToLabel[colorId] = _label;

  return color;
}

////////////////////////////////////////////////
VisualPtr Ogre2SegmentationMaterialSwitcher::TopLevelModelVisual(
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
void Ogre2SegmentationMaterialSwitcher::cameraPreRenderScene(
    Ogre::Camera * /*_cam*/)
{
  this->colorToLabel.clear();
  auto itor = this->scene->OgreSceneManager()->getMovableObjectIterator(
      Ogre::ItemFactory::FACTORY_TYPE_NAME);

  // Used for multi-link models, where each model has many ogre items but
  // belongs to the same object, and all of them has the same parent name
  std::string prevParentName = "";

  // Store the ogre objects in a vector
  std::vector<Ogre::MovableObject *> ogreObjects;
  while (itor.hasMoreElements())
  {
    Ogre::MovableObject *object = itor.peekNext();
    ogreObjects.push_back(object);
    itor.moveNext();
  }

  // Sort the ogre objects by name
  // The algorithm of handeling multi-link models depends on a sorted objects
  // by name, so all links that belongs to the same object come in order
  std::sort(ogreObjects.begin(), ogreObjects.end(),
    [] (Ogre::MovableObject * object1, Ogre::MovableObject * object2) {
      return object1->getName() > object2->getName();
  });

  for (auto object : ogreObjects)
  {
    Ogre::Item *item = static_cast<Ogre::Item *>(object);

    // get visual from ogre item
    Ogre::Any userAny = item->getUserObjectBindings().getUserAny();

    if (!userAny.isEmpty() && userAny.getType() == typeid(unsigned int))
    {
      // get visual id for the ogre item
      auto visualId = Ogre::any_cast<unsigned int>(userAny);

      VisualPtr visual;
      try
      {
        visual = this->scene->VisualById(visualId);
      }
      catch(Ogre::Exception &e)
      {
        ignerr << "Ogre Error:" << e.getFullDescription() << "\n";
      }
      Ogre2VisualPtr ogreVisual = std::dynamic_pointer_cast<Ogre2Visual>(
        visual);

      // get class user data
      Variant labelAny = ogreVisual->UserData("label");
      int label;
      try
      {
        label = std::get<int>(labelAny);
      }
      catch(std::bad_variant_access &e)
      {
        // items with no class are considered background
        label = this->segmentationCamera->BackgroundLabel();
      }

      // sub item custom parameter to set the pixel color material
      Ogre::Vector4 customParameter;

      // Material Switching
      if (this->segmentationCamera->Type() == SegmentationType::ST_SEMANTIC)
      {
        if (this->segmentationCamera->IsColoredMap())
        {
          // semantic material (each pixel has item's color)
          math::Color color = this->LabelToColor(label);
          customParameter = Ogre::Vector4(
            color.R(), color.G(), color.B(), 1.0);
        }
        else
        {
          // labels ids material (each pixel has item's label)
          float labelColor = label / 255.0;
          customParameter = Ogre::Vector4(
            labelColor, labelColor, labelColor, 1.0);
        }
      }
      else if (this->segmentationCamera->Type() ==
          SegmentationType::ST_PANOPTIC)
      {
        auto itemName = visual->Name();
        std::string parentName = this->TopLevelModelVisual(visual)->Name();

        auto it = this->instancesCount.find(label);
        if (it == this->instancesCount.end())
          it = this->instancesCount.insert(std::make_pair(label, 0)).first;

        // Multi link model has many links with the same first name and should
        // have the same pixels color
        bool isMultiLink = false;
        if (parentName == prevParentName)
        {
          isMultiLink = true;
        }
        else
        {
          it->second++;
          prevParentName = parentName;
        }

        int instanceCount = it->second;

        if (this->segmentationCamera->IsColoredMap())
        {
          // convert 24 bit number to int64
          int compositeId = label * 256 * 256 + instanceCount;

          math::Color color;
          if (label == this->segmentationCamera->BackgroundLabel())
            color = this->LabelToColor(label, isMultiLink);
          else
            color = this->LabelToColor(compositeId, isMultiLink);

          customParameter = Ogre::Vector4(
            color.R(), color.G(), color.B(), 1.0);
        }
        else
        {
          // 256 => 8 bits .. 255 => color percentage
          float labelColor = label / 255.0;
          float instanceColor1 = (instanceCount / 256) / 255.0;
          float instanceColor2 = (instanceCount % 256) / 255.0;

          customParameter = Ogre::Vector4(
            instanceColor2, instanceColor1, labelColor, 1.0);
        }
      }

      for (unsigned int i = 0; i < item->getNumSubItems(); ++i)
      {
        Ogre::SubItem *subItem = item->getSubItem(i);
        subItem->setCustomParameter(1, customParameter);

        // save subitems material
        // case when item is using low level materials
        // e.g. shaders
        if (!subItem->getMaterial().isNull())
        {
          this->segmentationMaterialMap[subItem] = subItem->getMaterial();
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
    }
  }

  // reset the count & colors tracking
  this->instancesCount.clear();
  this->takenColors.clear();
  this->coloredLabel.clear();

  // disable heightmaps in segmentation camera sensor
  // until we support changing its material based on input label
  // TODO(anyone) add support for heightmaps with the segmentation camera
  // https://github.com/ignitionrobotics/ign-rendering/issues/444
  auto heightmaps = this->scene->Heightmaps();
  for (auto h : heightmaps)
  {
    auto heightmap = h.lock();
    if (heightmap)
      heightmap->Parent()->SetVisible(false);
  }
}

////////////////////////////////////////////////
void Ogre2SegmentationMaterialSwitcher::cameraPostRenderScene(
    Ogre::Camera * /*_cam*/)
{
  // restore item to use pbs hlms material
  for (const auto &[subItem, dataBlock] : this->datablockMap)
    subItem->setDatablock(dataBlock);

  for (const auto &[subItem, material] : this->segmentationMaterialMap)
    subItem->setMaterial(material);

  this->datablockMap.clear();
  this->segmentationMaterialMap.clear();

  // re-enable heightmaps
  auto heightmaps = this->scene->Heightmaps();
  for (auto h : heightmaps)
  {
    auto heightmap = h.lock();
    if (heightmap)
      heightmap->Parent()->SetVisible(true);
  }
}

////////////////////////////////////////////////
const std::unordered_map<int64_t, int64_t> &
Ogre2SegmentationMaterialSwitcher::ColorToLabel() const
{
  return this->colorToLabel;
}
