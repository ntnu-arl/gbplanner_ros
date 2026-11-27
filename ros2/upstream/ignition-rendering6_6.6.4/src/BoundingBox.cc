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


#include "ignition/rendering/BoundingBox.hh"

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
class ignition::rendering::BoundingBoxPrivate
{
  /// \brief Center of the bounding box
  public: math::Vector3d center;

  /// \brief Size of the bounding box
  public: math::Vector3d size;

  /// \brief Orientation of the bounding box
  public: math::Quaterniond orientation;

  /// \brief Label of the bounding box
  public: uint32_t label;

  /// \brief 3D vertices of the bounding box
  public: std::vector<math::Vector3d> vertices3d;

  /// \brief Helper method for computing the bounding box's 3D vertices.
  /// This should be called whenever the box's center, size, or orientation
  /// changes.
  public: void ComputeVertices();
};


//////////////////////////////////////////////////
void BoundingBoxPrivate::ComputeVertices()
{
  this->vertices3d.clear();

  /*
    The numbers listed here are the corresponding indices in the vector
    that is returned by this method
      1 -------- 0
     /|         /|
    2 -------- 3 .
    | |        | |
    . 5 -------- 4
    |/         |/
    6 -------- 7
  */

  const auto w = this->size.X();
  const auto h = this->size.Y();
  const auto l = this->size.Z();

  // 8 vertices | box corners
  this->vertices3d.push_back(math::Vector3d(w/2, h/2, l/2));
  this->vertices3d.push_back(math::Vector3d(-w/2, h/2, l/2));
  this->vertices3d.push_back(math::Vector3d(-w/2, h/2, -l/2));
  this->vertices3d.push_back(math::Vector3d(w/2, h/2, -l/2));
  this->vertices3d.push_back(math::Vector3d(w/2, -h/2, l/2));
  this->vertices3d.push_back(math::Vector3d(-w/2, -h/2, l/2));
  this->vertices3d.push_back(math::Vector3d(-w/2, -h/2, -l/2));
  this->vertices3d.push_back(math::Vector3d(w/2, -h/2, -l/2));

  // Transform
  for (auto &vertex : this->vertices3d)
  {
    vertex = this->orientation * vertex + this->center;
  }
}

//////////////////////////////////////////////////
BoundingBox::BoundingBox() :
    dataPtr(std::make_unique<BoundingBoxPrivate>())
{
}

/////////////////////////////////////////////////
BoundingBox::~BoundingBox()
{
}

//////////////////////////////////////////////////
BoundingBox::BoundingBox(const BoundingBox &_box)
  : dataPtr(std::make_unique<BoundingBoxPrivate>(*_box.dataPtr))
{
}

//////////////////////////////////////////////////
BoundingBox::BoundingBox(BoundingBox &&_box) noexcept
  : dataPtr(std::exchange(_box.dataPtr, nullptr))
{
}

/////////////////////////////////////////////////
BoundingBox &BoundingBox::operator=(
    const BoundingBox &_box)
{
  return *this = BoundingBox(_box);
}

/////////////////////////////////////////////////
BoundingBox &BoundingBox::operator=(BoundingBox &&_box)
{
  std::swap(this->dataPtr, _box.dataPtr);
  return *this;
}

/////////////////////////////////////////////////
const math::Vector3d &BoundingBox::Center() const
{
  return this->dataPtr->center;
}

/////////////////////////////////////////////////
void BoundingBox::SetCenter(const math::Vector3d &_center)
{
  this->dataPtr->center = _center;
  this->dataPtr->ComputeVertices();
}

/////////////////////////////////////////////////
const math::Vector3d &BoundingBox::Size() const
{
  return this->dataPtr->size;
}

/////////////////////////////////////////////////
void BoundingBox::SetSize(const math::Vector3d &_size)
{
  this->dataPtr->size = _size;
  this->dataPtr->ComputeVertices();
}

/////////////////////////////////////////////////
const math::Quaterniond &BoundingBox::Orientation() const
{
  return this->dataPtr->orientation;
}

/////////////////////////////////////////////////
void BoundingBox::SetOrientation(const math::Quaterniond &_orientation)
{
  this->dataPtr->orientation = _orientation;
  this->dataPtr->ComputeVertices();
}

/////////////////////////////////////////////////
const std::vector<math::Vector3d> &BoundingBox::Vertices3D() const
{
  return this->dataPtr->vertices3d;
}

/////////////////////////////////////////////////
uint32_t BoundingBox::Label() const
{
  return this->dataPtr->label;
}

/////////////////////////////////////////////////
void BoundingBox::SetLabel(uint32_t _label)
{
  this->dataPtr->label = _label;
}
