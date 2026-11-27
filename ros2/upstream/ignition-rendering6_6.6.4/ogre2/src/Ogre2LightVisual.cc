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
#include "ignition/rendering/ogre2/Ogre2LightVisual.hh"
#include "ignition/rendering/ogre2/Ogre2Material.hh"
#include "ignition/rendering/ogre2/Ogre2DynamicRenderable.hh"

#ifdef _MSC_VER
  #pragma warning(push, 0)
#endif
#include <OgreSceneNode.h>
#ifdef _MSC_VER
  #pragma warning(pop)
#endif

using namespace ignition;
using namespace rendering;

class ignition::rendering::Ogre2LightVisualPrivate
{
  /// \brief light visual materal
  public: Ogre2MaterialPtr material = nullptr;

  /// \brief Ogre renderable used to render the light visual.
  public: std::shared_ptr<Ogre2DynamicRenderable> lightVisual = nullptr;
};

//////////////////////////////////////////////////
Ogre2LightVisual::Ogre2LightVisual()
  : dataPtr(new Ogre2LightVisualPrivate)
{
}

//////////////////////////////////////////////////
Ogre2LightVisual::~Ogre2LightVisual()
{
}

//////////////////////////////////////////////////
void Ogre2LightVisual::PreRender()
{
  if (this->dirtyLightVisual)
  {
    this->CreateVisual();
    this->dirtyLightVisual = false;
  }
}

//////////////////////////////////////////////////
void Ogre2LightVisual::Init()
{
  BaseLightVisual::Init();
  this->CreateVisual();
}

//////////////////////////////////////////////////
Ogre::MovableObject *Ogre2LightVisual::OgreObject() const
{
  return this->dataPtr->lightVisual->OgreObject();
}

//////////////////////////////////////////////////
void Ogre2LightVisual::CreateVisual()
{
  if (!this->dataPtr->lightVisual)
  {
    this->dataPtr->lightVisual.reset(
      new Ogre2DynamicRenderable(this->Scene()));
    this->ogreNode->attachObject(this->dataPtr->lightVisual->OgreObject());
  }

  // Clear any previous data from the grid and update
  this->dataPtr->lightVisual->Clear();
  this->dataPtr->lightVisual->Update();

  this->dataPtr->lightVisual->SetOperationType(MarkerType::MT_LINE_LIST);
  if (this->dataPtr->material == nullptr)
  {
    MaterialPtr defaultMat = this->Scene()->Material("Default/TransGreen");
    this->SetMaterial(defaultMat, false);
  }

  std::vector<ignition::math::Vector3d> positions = this->CreateVisualLines();

  for (const auto &p : positions)
  {
    this->dataPtr->lightVisual->AddPoint(p.X(), p.Y(), p.Z());
  }

  this->dataPtr->lightVisual->Update();
}

//////////////////////////////////////////////////
void Ogre2LightVisual::SetMaterial(MaterialPtr _material, bool _unique)
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
  this->dataPtr->lightVisual->SetMaterial(_material, false);
  this->SetMaterialImpl(derived);
}

//////////////////////////////////////////////////
void Ogre2LightVisual::SetMaterialImpl(Ogre2MaterialPtr _material)
{
  Ogre::MaterialPtr ogreMaterial = _material->Material();
  this->dataPtr->material = _material;
}

//////////////////////////////////////////////////
MaterialPtr Ogre2LightVisual::Material() const
{
  return this->dataPtr->material;
}
