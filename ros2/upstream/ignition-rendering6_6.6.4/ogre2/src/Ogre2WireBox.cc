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

#include "ignition/rendering/ogre2/Ogre2WireBox.hh"
#include "ignition/rendering/ogre2/Ogre2Material.hh"
#include "ignition/rendering/ogre2/Ogre2Scene.hh"
#include "ignition/rendering/ogre2/Ogre2DynamicRenderable.hh"

using namespace ignition;
using namespace rendering;

class ignition::rendering::Ogre2WireBoxPrivate
{
  /// \brief Wirebox material
  public: Ogre2MaterialPtr material;

  /// \brief Ogre renderable used to render the wirebox.
  public: std::shared_ptr<Ogre2DynamicRenderable> wireBox = nullptr;
};

//////////////////////////////////////////////////
Ogre2WireBox::Ogre2WireBox()
    : dataPtr(new Ogre2WireBoxPrivate)
{
}

//////////////////////////////////////////////////
Ogre2WireBox::~Ogre2WireBox()
{
}

//////////////////////////////////////////////////
void Ogre2WireBox::PreRender()
{
  if (this->wireBoxDirty)
  {
    this->Create();
    this->wireBoxDirty = false;
  }
}

//////////////////////////////////////////////////
Ogre::MovableObject *Ogre2WireBox::OgreObject() const
{
  return this->dataPtr->wireBox->OgreObject();
}

//////////////////////////////////////////////////
void Ogre2WireBox::Init()
{
  this->Create();
}

//////////////////////////////////////////////////
void Ogre2WireBox::Create()
{
  if (!this->dataPtr->wireBox)
  {
    this->dataPtr->wireBox.reset(new Ogre2DynamicRenderable(this->scene));
  }

  // Clear any previous data from the wirebox
  this->dataPtr->wireBox->Clear();
  this->dataPtr->wireBox->Update();

  this->dataPtr->wireBox->SetOperationType(MarkerType::MT_LINE_LIST);

  ignition::math::Vector3d max = this->box.Max();
  ignition::math::Vector3d min = this->box.Min();

  // line 0
  this->dataPtr->wireBox->AddPoint({min.X(), min.Y(), min.Z()});
  this->dataPtr->wireBox->AddPoint({max.X(), min.Y(), min.Z()});

  // line 1
  this->dataPtr->wireBox->AddPoint({min.X(), min.Y(), min.Z()});
  this->dataPtr->wireBox->AddPoint({min.X(), min.Y(), max.Z()});

  // line 2
  this->dataPtr->wireBox->AddPoint({min.X(), min.Y(), min.Z()});
  this->dataPtr->wireBox->AddPoint({min.X(), max.Y(), min.Z()});

  // line 3
  this->dataPtr->wireBox->AddPoint({min.X(), max.Y(), min.Z()});
  this->dataPtr->wireBox->AddPoint({min.X(), max.Y(), max.Z()});

  // line 4
  this->dataPtr->wireBox->AddPoint({min.X(), max.Y(), min.Z()});
  this->dataPtr->wireBox->AddPoint({max.X(), max.Y(), min.Z()});

  // line 5
  this->dataPtr->wireBox->AddPoint({max.X(), min.Y(), min.Z()});
  this->dataPtr->wireBox->AddPoint({max.X(), min.Y(), max.Z()});

  // line 6
  this->dataPtr->wireBox->AddPoint({max.X(), min.Y(), min.Z()});
  this->dataPtr->wireBox->AddPoint({max.X(), max.Y(), min.Z()});

  // line 7
  this->dataPtr->wireBox->AddPoint({min.X(), max.Y(), max.Z()});
  this->dataPtr->wireBox->AddPoint({max.X(), max.Y(), max.Z()});

  // line 8
  this->dataPtr->wireBox->AddPoint({min.X(), max.Y(), max.Z()});
  this->dataPtr->wireBox->AddPoint({min.X(), min.Y(), max.Z()});

  // line 9
  this->dataPtr->wireBox->AddPoint({max.X(), max.Y(), min.Z()});
  this->dataPtr->wireBox->AddPoint({max.X(), max.Y(), max.Z()});

  // line 10
  this->dataPtr->wireBox->AddPoint({max.X(), min.Y(), max.Z()});
  this->dataPtr->wireBox->AddPoint({max.X(), max.Y(), max.Z()});

  // line 11
  this->dataPtr->wireBox->AddPoint({min.X(), min.Y(), max.Z()});
  this->dataPtr->wireBox->AddPoint({max.X(), min.Y(), max.Z()});

  this->dataPtr->wireBox->Update();
}

//////////////////////////////////////////////////
void Ogre2WireBox::SetMaterial(MaterialPtr _material, bool _unique)
{
  _material = (_unique) ? _material->Clone() : _material;

  Ogre2MaterialPtr derived =
      std::dynamic_pointer_cast<Ogre2Material>(_material);

  if (!derived)
  {
    ignerr << "Cannot assign material created by another render-engine"
        << std::endl;

    return;
  }

  // Set material for the underlying dynamic renderable
  this->dataPtr->wireBox->SetMaterial(_material, false);
  this->SetMaterialImpl(derived);
}

//////////////////////////////////////////////////
void Ogre2WireBox::SetMaterialImpl(Ogre2MaterialPtr _material)
{
  Ogre::MaterialPtr ogreMaterial = _material->Material();
  this->dataPtr->material = _material;

  this->dataPtr->material->SetReceiveShadows(false);
  this->dataPtr->material->SetCastShadows(false);
  this->dataPtr->material->SetLightingEnabled(false);
}

//////////////////////////////////////////////////
MaterialPtr Ogre2WireBox::Material() const
{
  return this->dataPtr->material;
}
