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

#include "ignition/rendering/ogre/OgreLightVisual.hh"
#include "ignition/rendering/ogre/OgreDynamicLines.hh"

#include "ignition/rendering/ogre/OgreScene.hh"

class ignition::rendering::OgreLightVisualPrivate
{
  /// \brief Grid materal
  public: OgreMaterialPtr material = nullptr;

  std::shared_ptr<OgreDynamicLines> line = nullptr;
};

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
OgreLightVisual::OgreLightVisual()
  : dataPtr(new OgreLightVisualPrivate)
{
}

//////////////////////////////////////////////////
OgreLightVisual::~OgreLightVisual()
{
}

//////////////////////////////////////////////////
void OgreLightVisual::PreRender()
{
  if (this->dirtyLightVisual)
  {
    this->CreateVisual();
    this->dirtyLightVisual = false;
  }
}

//////////////////////////////////////////////////
void OgreLightVisual::Init()
{
  BaseLightVisual::Init();
  this->CreateVisual();
}

//////////////////////////////////////////////////
Ogre::MovableObject *OgreLightVisual::OgreObject() const
{
  std::shared_ptr<Ogre::MovableObject> mv =
    std::dynamic_pointer_cast<Ogre::MovableObject>(this->dataPtr->line);
  return mv.get();
}

//////////////////////////////////////////////////
void OgreLightVisual::CreateVisual()
{
  if (!this->dataPtr->line)
  {
    this->dataPtr->line = std::shared_ptr<OgreDynamicLines>(
      new OgreDynamicLines(MT_LINE_LIST));
    this->ogreNode->attachObject(this->OgreObject());
    MaterialPtr lightVisualMaterial =
      this->Scene()->Material("Default/TransGreen");
    this->SetMaterial(lightVisualMaterial, false);
  }
  std::vector<ignition::math::Vector3d> positions = this->CreateVisualLines();

  for (const auto &p : positions)
  {
    this->dataPtr->line->AddPoint(p.X(), p.Y(), p.Z());
  }
  this->dataPtr->line->Update();
}

//////////////////////////////////////////////////
void OgreLightVisual::SetMaterial(MaterialPtr _material, bool _unique)
{
  _material = (_unique) ? _material->Clone() : _material;

  OgreMaterialPtr derived =
      std::dynamic_pointer_cast<OgreMaterial>(_material);

  if (!derived)
  {
    ignerr << "Cannot assign material created by another render-engine"
        << std::endl;

    return;
  }

  this->SetMaterialImpl(derived);
}

//////////////////////////////////////////////////
void OgreLightVisual::SetMaterialImpl(OgreMaterialPtr _material)
{
  std::string materialName = _material->Name();
  Ogre::MaterialPtr ogreMaterial = _material->Material();

#if (OGRE_VERSION <= ((1 << 16) | (10 << 8) | 7))
  this->dataPtr->line->setMaterial(materialName);
#else
  this->dataPtr->line->setMaterial(ogreMaterial);
#endif
  this->dataPtr->material = _material;

  this->dataPtr->material->SetReceiveShadows(false);
  this->dataPtr->material->SetLightingEnabled(false);
}

//////////////////////////////////////////////////
MaterialPtr OgreLightVisual::Material() const
{
  return this->dataPtr->material;
}
