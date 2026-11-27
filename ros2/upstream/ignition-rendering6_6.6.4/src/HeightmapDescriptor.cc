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


#include "ignition/rendering/HeightmapDescriptor.hh"

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
class ignition::rendering::HeightmapTexturePrivate
{
  /// \brief Texture size.
  public: double size{1.0};

  /// \brief Path to diffuse texture file.
  public: std::string diffuse;

  /// \brief Path to normal map file.
  public: std::string normal;
};

//////////////////////////////////////////////////
class ignition::rendering::HeightmapBlendPrivate
{
  /// \brief Minimum height to blend from.
  public: double minHeight{0.0};

  /// \brief Distance to blend.
  public: double fadeDistance{0.0};
};

//////////////////////////////////////////////////
class ignition::rendering::HeightmapDescriptorPrivate
{
  /// \brief Name used for caching
  public: std::string name;

  /// \brief Contains heightfield data.
  public: std::shared_ptr<common::HeightmapData> data{nullptr};

  /// \brief Heightmap XYZ size in meters.
  public: math::Vector3d size{1.0, 1.0, 1.0};

  /// \brief Heightmap XYZ origin in meters.
  public: math::Vector3d position{0.0, 0.0, 0.0};

  /// \brief Flag that enables/disables the terrain paging
  public: bool useTerrainPaging{false};

  /// \brief Number of samples per heightmap datum.
  public: unsigned int sampling{1u};

  /// \brief Textures in this heightmap, in height order.
  public: std::vector<HeightmapTexture> textures;

  /// \brief Blends in this heightmap, in height order. There should be one
  /// less than textures.
  public: std::vector<HeightmapBlend> blends;
};

//////////////////////////////////////////////////
HeightmapTexture::HeightmapTexture() :
    dataPtr(std::make_unique<HeightmapTexturePrivate>())
{
}

/////////////////////////////////////////////////
HeightmapTexture::~HeightmapTexture()
{
}

//////////////////////////////////////////////////
HeightmapTexture::HeightmapTexture(const HeightmapTexture &_texture)
  : dataPtr(new HeightmapTexturePrivate(*_texture.dataPtr))
{
}

//////////////////////////////////////////////////
HeightmapTexture::HeightmapTexture(HeightmapTexture &&_texture) noexcept
  : dataPtr(std::exchange(_texture.dataPtr, nullptr))
{
}

/////////////////////////////////////////////////
HeightmapTexture &HeightmapTexture::operator=(
    const HeightmapTexture &_texture)
{
  return *this = HeightmapTexture(_texture);
}

/////////////////////////////////////////////////
HeightmapTexture &HeightmapTexture::operator=(HeightmapTexture &&_texture)
{
  std::swap(this->dataPtr, _texture.dataPtr);
  return *this;
}

//////////////////////////////////////////////////
double HeightmapTexture::Size() const
{
  return this->dataPtr->size;
}

//////////////////////////////////////////////////
void HeightmapTexture::SetSize(double _size)
{
  this->dataPtr->size = _size;
}

//////////////////////////////////////////////////
std::string HeightmapTexture::Diffuse() const
{
  return this->dataPtr->diffuse;
}

//////////////////////////////////////////////////
void HeightmapTexture::SetDiffuse(const std::string &_diffuse)
{
  this->dataPtr->diffuse = _diffuse;
}

//////////////////////////////////////////////////
std::string HeightmapTexture::Normal() const
{
  return this->dataPtr->normal;
}

//////////////////////////////////////////////////
void HeightmapTexture::SetNormal(const std::string &_normal)
{
  this->dataPtr->normal = _normal;
}

//////////////////////////////////////////////////
HeightmapBlend::HeightmapBlend() :
    dataPtr(std::make_unique<HeightmapBlendPrivate>())
{
}


/////////////////////////////////////////////////
HeightmapBlend::~HeightmapBlend()
{
}

//////////////////////////////////////////////////
HeightmapBlend::HeightmapBlend(const HeightmapBlend &_blend)
  : dataPtr(new HeightmapBlendPrivate(*_blend.dataPtr))
{
}

//////////////////////////////////////////////////
HeightmapBlend::HeightmapBlend(HeightmapBlend &&_blend) noexcept
  : dataPtr(std::exchange(_blend.dataPtr, nullptr))
{
}

/////////////////////////////////////////////////
HeightmapBlend &HeightmapBlend::operator=(
    const HeightmapBlend &_blend)
{
  return *this = HeightmapBlend(_blend);
}

/////////////////////////////////////////////////
HeightmapBlend &HeightmapBlend::operator=(HeightmapBlend &&_blend)
{
  std::swap(this->dataPtr, _blend.dataPtr);
  return *this;
}
//////////////////////////////////////////////////
double HeightmapBlend::MinHeight() const
{
  return this->dataPtr->minHeight;
}

//////////////////////////////////////////////////
void HeightmapBlend::SetMinHeight(double _minHeight)
{
  this->dataPtr->minHeight = _minHeight;
}

//////////////////////////////////////////////////
double HeightmapBlend::FadeDistance() const
{
  return this->dataPtr->fadeDistance;
}

//////////////////////////////////////////////////
void HeightmapBlend::SetFadeDistance(double _fadeDistance)
{
  this->dataPtr->fadeDistance = _fadeDistance;
}

//////////////////////////////////////////////////
HeightmapDescriptor::HeightmapDescriptor() :
    dataPtr(std::make_unique<HeightmapDescriptorPrivate>())
{
}

/////////////////////////////////////////////////
HeightmapDescriptor::~HeightmapDescriptor() = default;

//////////////////////////////////////////////////
HeightmapDescriptor::HeightmapDescriptor(const HeightmapDescriptor &_heightmap)
  : dataPtr(new HeightmapDescriptorPrivate(*_heightmap.dataPtr))
{
}

//////////////////////////////////////////////////
HeightmapDescriptor::HeightmapDescriptor(HeightmapDescriptor &&_heightmap)
  noexcept
  : dataPtr(std::exchange(_heightmap.dataPtr, nullptr))
{
}

/////////////////////////////////////////////////
HeightmapDescriptor &HeightmapDescriptor::operator=(
    const HeightmapDescriptor &_heightmap)
{
  return *this = HeightmapDescriptor(_heightmap);
}

/////////////////////////////////////////////////
HeightmapDescriptor &HeightmapDescriptor::operator=(
    HeightmapDescriptor &&_heightmap)
{
  std::swap(this->dataPtr, _heightmap.dataPtr);
  return *this;
}

//////////////////////////////////////////////////
const std::string &HeightmapDescriptor::Name() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
void HeightmapDescriptor::SetName(const std::string &_name)
{
  this->dataPtr->name = _name;
}

//////////////////////////////////////////////////
std::shared_ptr<common::HeightmapData> HeightmapDescriptor::Data() const
{
  return this->dataPtr->data;
}

//////////////////////////////////////////////////
void HeightmapDescriptor::SetData(
    const std::shared_ptr<common::HeightmapData> &_data)
{
  this->dataPtr->data = _data;
}

//////////////////////////////////////////////////
math::Vector3d HeightmapDescriptor::Size() const
{
  return this->dataPtr->size;
}

//////////////////////////////////////////////////
void HeightmapDescriptor::SetSize(const math::Vector3d &_size)
{
  this->dataPtr->size = _size;
}

//////////////////////////////////////////////////
math::Vector3d HeightmapDescriptor::Position() const
{
  return this->dataPtr->position;
}

//////////////////////////////////////////////////
void HeightmapDescriptor::SetPosition(const math::Vector3d &_position)
{
  this->dataPtr->position = _position;
}

//////////////////////////////////////////////////
bool HeightmapDescriptor::UseTerrainPaging() const
{
  return this->dataPtr->useTerrainPaging;
}

//////////////////////////////////////////////////
void HeightmapDescriptor::SetUseTerrainPaging(bool _useTerrainPaging)
{
  this->dataPtr->useTerrainPaging = _useTerrainPaging;
}

//////////////////////////////////////////////////
unsigned int HeightmapDescriptor::Sampling() const
{
  return this->dataPtr->sampling;
}

//////////////////////////////////////////////////
void HeightmapDescriptor::SetSampling(unsigned int _sampling)
{
  this->dataPtr->sampling = _sampling;
}

/////////////////////////////////////////////////
uint64_t HeightmapDescriptor::TextureCount() const
{
  return this->dataPtr->textures.size();
}

/////////////////////////////////////////////////
const HeightmapTexture *HeightmapDescriptor::TextureByIndex(uint64_t _index)
    const
{
  if (_index < this->dataPtr->textures.size())
    return &this->dataPtr->textures[_index];
  return nullptr;
}

/////////////////////////////////////////////////
void HeightmapDescriptor::AddTexture(const HeightmapTexture &_texture)
{
  this->dataPtr->textures.push_back(_texture);
}

/////////////////////////////////////////////////
uint64_t HeightmapDescriptor::BlendCount() const
{
  return this->dataPtr->blends.size();
}

/////////////////////////////////////////////////
const HeightmapBlend *HeightmapDescriptor::BlendByIndex(uint64_t _index) const
{
  if (_index < this->dataPtr->blends.size())
    return &this->dataPtr->blends[_index];
  return nullptr;
}

/////////////////////////////////////////////////
void HeightmapDescriptor::AddBlend(const HeightmapBlend &_blend)
{
  this->dataPtr->blends.push_back(_blend);
}
