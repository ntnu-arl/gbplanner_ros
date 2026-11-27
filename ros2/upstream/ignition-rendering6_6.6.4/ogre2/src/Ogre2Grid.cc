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

#include <gz/common/Console.hh>

#include "gz/rendering/ogre2/Ogre2Grid.hh"
#include "gz/rendering/ogre2/Ogre2Material.hh"
#include "gz/rendering/ogre2/Ogre2Scene.hh"
#include "gz/rendering/ogre2/Ogre2DynamicRenderable.hh"

using namespace gz;
using namespace rendering;

class gz::rendering::Ogre2GridPrivate
{
  /// \brief Grid materal
  public: Ogre2MaterialPtr material;

  /// \brief Ogre renderable used to render the grid.
  public: std::shared_ptr<Ogre2DynamicRenderable> grid = nullptr;
};

//////////////////////////////////////////////////
Ogre2Grid::Ogre2Grid()
    : dataPtr(new Ogre2GridPrivate)
{
}

//////////////////////////////////////////////////
Ogre2Grid::~Ogre2Grid()
{
  BaseGrid::Destroy();
}

//////////////////////////////////////////////////
void Ogre2Grid::PreRender()
{
  BaseGrid::PreRender();

  if (this->gridDirty)
  {
    this->Create();
    this->gridDirty = false;
  }
}

//////////////////////////////////////////////////
Ogre::MovableObject *Ogre2Grid::OgreObject() const
{
  return this->dataPtr->grid->OgreObject();
}

//////////////////////////////////////////////////
void Ogre2Grid::Init()
{
  this->Create();
}

//////////////////////////////////////////////////
void Ogre2Grid::Create()
{
  if (!this->dataPtr->grid)
  {
    this->dataPtr->grid.reset(new Ogre2DynamicRenderable(this->scene));
  }

  // Clear any previous data from the grid and update
  this->dataPtr->grid->Clear();
  this->dataPtr->grid->Update();

  this->dataPtr->grid->SetOperationType(MarkerType::MT_LINE_LIST);
  double baseExtent = (this->cellLength *
     static_cast<double>(this->cellCount - this->cellCount % 2))/2;
  for (unsigned int h = 0; h <= this->verticalCellCount; ++h)
  {
    double hReal = this->heightOffset +
        (this->verticalCellCount / 2 - static_cast<double>(h))
        * this->cellLength;

    // If there are odd vertical cells, shift cell planes up
    if (this->verticalCellCount % 2)
      hReal += this->cellLength;

    for (unsigned int i = 0; i <= this->cellCount; i++)
    {
      double extent = baseExtent;

      // If there is an odd cell count, extend a row and column along
      // the positive x and y axes
      if (this->cellCount % 2)
        extent += this->cellLength;

      double inc = extent - (i * this->cellLength);
      math::Vector3d p1{inc, -baseExtent, hReal};
      math::Vector3d p2{inc, extent , hReal};
      math::Vector3d p3{-baseExtent, inc, hReal};
      math::Vector3d p4{extent, inc, hReal};

      this->dataPtr->grid->AddPoint(p1);
      this->dataPtr->grid->AddPoint(p2);
      this->dataPtr->grid->AddPoint(p3);
      this->dataPtr->grid->AddPoint(p4);
    }
  }
  if (this->verticalCellCount > 0)
  {
    for (unsigned int x = 0; x <= this->cellCount; ++x)
    {
      for (unsigned int y = 0; y <= this->cellCount; ++y)
      {
        double xReal = baseExtent - x * this->cellLength;
        double yReal = baseExtent - y * this->cellLength;

        double zTop = (this->verticalCellCount / 2.0f) * this->cellLength;
        double zBottom = -zTop;

        // If odd vertical cell count, add cell length offset to adjust
        // z min and max
        if (this->verticalCellCount % 2)
        {
          zTop += this->cellLength / 2.0f;
          zBottom += this->cellLength / 2.0f;
        }

        // If odd horizontal cell count, shift vertical lines
        // towards positive x, y axes
        if (this->cellCount % 2)
        {
          xReal += this->cellLength;
          yReal += this->cellLength;
        }

        this->dataPtr->grid->AddPoint(xReal, yReal, zBottom);
        this->dataPtr->grid->AddPoint(xReal, yReal, zTop);
      }
    }
  }
  this->dataPtr->grid->Update();
}

//////////////////////////////////////////////////
void Ogre2Grid::SetMaterial(MaterialPtr _material, bool _unique)
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
  this->dataPtr->grid->SetMaterial(_material, false);
  this->SetMaterialImpl(derived);
}

//////////////////////////////////////////////////
void Ogre2Grid::SetMaterialImpl(Ogre2MaterialPtr _material)
{
  Ogre::MaterialPtr ogreMaterial = _material->Material();
  this->dataPtr->material = _material;

  this->dataPtr->material->SetReceiveShadows(false);
  this->dataPtr->material->SetCastShadows(false);
  this->dataPtr->material->SetLightingEnabled(false);
}

//////////////////////////////////////////////////
MaterialPtr Ogre2Grid::Material() const
{
  return this->dataPtr->material;
}
