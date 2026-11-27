/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#include <ignition/common/Console.hh>

#include "ignition/rendering/ogre/OgreWireBox.hh"
#include "ignition/rendering/ogre/OgreMaterial.hh"
#include "ignition/rendering/ogre/OgreScene.hh"
#include "ignition/rendering/ogre/OgreDynamicLines.hh"

class ignition::rendering::OgreWireBoxPrivate
{
  /// \brief WireBox materal
  public: OgreMaterialPtr material;

  /// \brief Ogre manual object used to render the wire box.
  public: Ogre::ManualObject *manualObject = nullptr;
};

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
OgreWireBox::OgreWireBox()
    : dataPtr(new OgreWireBoxPrivate)
{
}

//////////////////////////////////////////////////
OgreWireBox::~OgreWireBox()
{
  if (!this->Scene()->IsInitialized())
    return;

  if (this->dataPtr->manualObject)
  {
    this->scene->OgreSceneManager()->destroyMovableObject(
        this->dataPtr->manualObject);
    this->dataPtr->manualObject = nullptr;
  }
}

//////////////////////////////////////////////////
void OgreWireBox::PreRender()
{
  if (this->wireBoxDirty)
  {
    this->Create();
    this->wireBoxDirty = false;
  }
}

//////////////////////////////////////////////////
Ogre::MovableObject *OgreWireBox::OgreObject() const
{
  return this->dataPtr->manualObject;
}

//////////////////////////////////////////////////
void OgreWireBox::Init()
{
  this->Create();
}

//////////////////////////////////////////////////
void OgreWireBox::Create()
{
  if (!this->dataPtr->manualObject)
  {
    this->dataPtr->manualObject =
      this->scene->OgreSceneManager()->createManualObject(this->name);
  }

  this->dataPtr->manualObject->clear();
  this->dataPtr->manualObject->setCastShadows(false);

  this->dataPtr->manualObject->estimateVertexCount(12);
  std::string materialName = this->dataPtr->material ?
      this->dataPtr->material->Name() : "Default/White";
  this->dataPtr->manualObject->begin(materialName,
      Ogre::RenderOperation::OT_LINE_LIST);

  ignition::math::Vector3d max = this->box.Max();
  ignition::math::Vector3d min = this->box.Min();

  // line 0
  this->dataPtr->manualObject->position(min.X(), min.Y(), min.Z());
  this->dataPtr->manualObject->position(max.X(), min.Y(), min.Z());

  // line 1
  this->dataPtr->manualObject->position(min.X(), min.Y(), min.Z());
  this->dataPtr->manualObject->position(min.X(), min.Y(), max.Z());

  // line 2
  this->dataPtr->manualObject->position(min.X(), min.Y(), min.Z());
  this->dataPtr->manualObject->position(min.X(), max.Y(), min.Z());

  // line 3
  this->dataPtr->manualObject->position(min.X(), max.Y(), min.Z());
  this->dataPtr->manualObject->position(min.X(), max.Y(), max.Z());

  // line 4
  this->dataPtr->manualObject->position(min.X(), max.Y(), min.Z());
  this->dataPtr->manualObject->position(max.X(), max.Y(), min.Z());

  // line 5
  this->dataPtr->manualObject->position(max.X(), min.Y(), min.Z());
  this->dataPtr->manualObject->position(max.X(), min.Y(), max.Z());

  // line 6
  this->dataPtr->manualObject->position(max.X(), min.Y(), min.Z());
  this->dataPtr->manualObject->position(max.X(), max.Y(), min.Z());

  // line 7
  this->dataPtr->manualObject->position(min.X(), max.Y(), max.Z());
  this->dataPtr->manualObject->position(max.X(), max.Y(), max.Z());

  // line 8
  this->dataPtr->manualObject->position(min.X(), max.Y(), max.Z());
  this->dataPtr->manualObject->position(min.X(), min.Y(), max.Z());

  // line 9
  this->dataPtr->manualObject->position(max.X(), max.Y(), min.Z());
  this->dataPtr->manualObject->position(max.X(), max.Y(), max.Z());

  // line 10
  this->dataPtr->manualObject->position(max.X(), min.Y(), max.Z());
  this->dataPtr->manualObject->position(max.X(), max.Y(), max.Z());

  // line 11
  this->dataPtr->manualObject->position(min.X(), min.Y(), max.Z());
  this->dataPtr->manualObject->position(max.X(), min.Y(), max.Z());

  this->dataPtr->manualObject->end();
}

//////////////////////////////////////////////////
void OgreWireBox::SetMaterial(MaterialPtr _material, bool _unique)
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
void OgreWireBox::SetMaterialImpl(OgreMaterialPtr _material)
{
  std::string materialName = _material->Name();
  Ogre::MaterialPtr ogreMaterial = _material->Material();
  this->dataPtr->manualObject->setMaterialName(0, materialName);
  this->dataPtr->material = _material;

  this->dataPtr->material->SetReceiveShadows(false);
  this->dataPtr->material->SetLightingEnabled(false);
}

//////////////////////////////////////////////////
MaterialPtr OgreWireBox::Material() const
{
  return this->dataPtr->material;
}
