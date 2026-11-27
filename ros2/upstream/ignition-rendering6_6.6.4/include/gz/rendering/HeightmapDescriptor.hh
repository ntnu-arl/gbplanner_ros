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
#ifndef IGNITION_RENDERING_HEIGHTMAPDESCRIPTOR_HH_
#define IGNITION_RENDERING_HEIGHTMAPDESCRIPTOR_HH_

#include <memory>
#include <string>
#include <ignition/common/HeightmapData.hh>
#include <ignition/common/SuppressWarning.hh>

#include "ignition/rendering/config.hh"
#include "ignition/rendering/Export.hh"

namespace ignition
{
namespace rendering
{
inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
  class HeightmapDescriptorPrivate;
  class HeightmapTexturePrivate;
  class HeightmapBlendPrivate;

  /// \brief Texture to be used on heightmaps.
  class IGNITION_RENDERING_VISIBLE HeightmapTexture
  {
    /// \brief Constructor
    public: HeightmapTexture();

    /// \brief Copy constructor
    /// \param[in] _texture HeightmapTexture to copy.
    public: HeightmapTexture(const HeightmapTexture &_texture);

    /// \brief Move constructor
    /// \param[in] _texture HeightmapTexture to move.
    public: HeightmapTexture(HeightmapTexture &&_texture) noexcept;

    /// \brief Destructor
    public: virtual ~HeightmapTexture();

    /// \brief Move assignment operator.
    /// \param[in] _texture Heightmap texture to move.
    /// \return Reference to this.
    public: HeightmapTexture &operator=(HeightmapTexture &&_texture);

    /// \brief Copy Assignment operator.
    /// \param[in] _texture The heightmap texture to set values from.
    /// \return *this
    public: HeightmapTexture &operator=(const HeightmapTexture &_texture);

    /// \brief Get the heightmap texture's size.
    /// \return The size of the heightmap texture in meters.
    public: double Size() const;

    /// \brief Set the size of the texture in meters.
    /// \param[in] _size The size of the texture in meters.
    public: void SetSize(double _size);

    /// \brief Get the heightmap texture's diffuse map.
    /// \return The diffuse map of the heightmap texture.
    public: std::string Diffuse() const;

    /// \brief Set the filename of the diffuse map.
    /// \param[in] _diffuse The diffuse map of the heightmap texture.
    public: void SetDiffuse(const std::string &_diffuse);

    /// \brief Get the heightmap texture's normal map.
    /// \return The normal map of the heightmap texture.
    public: std::string Normal() const;

    /// \brief Set the filename of the normal map.
    /// \param[in] _normal The normal map of the heightmap texture.
    public: void SetNormal(const std::string &_normal);

    /// \brief Private data pointer.
    IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
    private: std::unique_ptr<HeightmapTexturePrivate> dataPtr;
    IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING
  };

  /// \brief Blend information to be used between textures on heightmaps.
  class IGNITION_RENDERING_VISIBLE HeightmapBlend
  {
    /// \brief Constructor
    public: HeightmapBlend();

    /// \brief Copy constructor
    /// \param[in] _blend HeightmapBlend to copy.
    public: HeightmapBlend(const HeightmapBlend &_blend);

    /// \brief Move constructor
    /// \param[in] _blend HeightmapBlend to move.
    public: HeightmapBlend(HeightmapBlend &&_blend) noexcept;

    /// \brief Destructor
    public: virtual ~HeightmapBlend();

    /// \brief Move assignment operator.
    /// \param[in] _blend Heightmap blend to move.
    /// \return Reference to this.
    public: HeightmapBlend &operator=(HeightmapBlend &&_blend);

    /// \brief Copy Assignment operator.
    /// \param[in] _blend The heightmap blend to set values from.
    /// \return *this
    public: HeightmapBlend &operator=(const HeightmapBlend &_blend);

    /// \brief Get the heightmap blend's minimum height.
    /// \return The minimum height of the blend layer.
    public: double MinHeight() const;

    /// \brief Set the minimum height of the blend in meters.
    /// \param[in] _minHeight The minimum height of the blend in meters.
    public: void SetMinHeight(double _minHeight);

    /// \brief Get the heightmap blend's fade distance.
    /// \return The fade distance of the heightmap blend in meters.
    public: double FadeDistance() const;

    /// \brief Set the distance over which the blend occurs.
    /// \param[in] _fadeDistance The distance in meters.
    public: void SetFadeDistance(double _fadeDistance);

    /// \brief Private data pointer.
    IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
    private: std::unique_ptr<HeightmapBlendPrivate> dataPtr;
    IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING
  };

  /// \class HeightmapDescriptor HeightmapDescriptor.hh
  /// ignition/rendering/HeightmapDescriptor.hh
  /// \brief Describes how a Heightmap should be loaded
  class IGNITION_RENDERING_VISIBLE HeightmapDescriptor
  {
    /// \brief Constructor
    public: HeightmapDescriptor();

    /// \brief Copy constructor
    /// \param[in] _desc HeightmapDescriptor to copy.
    public: HeightmapDescriptor(const HeightmapDescriptor &_desc);

    /// \brief Move constructor
    /// \param[in] _desc HeightmapDescriptor to move.
    public: HeightmapDescriptor(HeightmapDescriptor &&_desc) noexcept;

    /// \brief Destructor
    public: virtual ~HeightmapDescriptor();

    /// \brief Move assignment operator.
    /// \param[in] _desc HeightmapDescriptor to move.
    /// \return Reference to this.
    public: HeightmapDescriptor &operator=(HeightmapDescriptor &&_desc);

    /// \brief Copy Assignment operator.
    /// \param[in] _desc The heightmap to set values from.
    /// \return *this
    public: HeightmapDescriptor &operator=(const HeightmapDescriptor &_desc);

    /// \brief Get the heightmap's name used for caching.
    /// This is different from its unique `Heightmap::Name()`.
    /// \return Heightmap's given name. Defaults to `Heightmap::Name()`.
    public: const std::string &Name() const;

    /// \brief Set the heightmap's name.
    /// \param[in] _name Heightmap's name.
    public: void SetName(const std::string &_name);

    /// \brief Get the heightfield data.
    /// \return Heightmap data.
    public: std::shared_ptr<common::HeightmapData> Data() const;

    /// \brief Set the heightfield data.
    /// \param[in] _data New data.
    public: void SetData(const std::shared_ptr<common::HeightmapData> &_data);

    /// \brief Get the heightmap's scaling factor.
    /// \return The heightmap's size.
    public: ignition::math::Vector3d Size() const;

    /// \brief Set the heightmap's scaling factor. Defaults to 1x1x1.
    /// \return The heightmap's size factor.
    public: void SetSize(const ignition::math::Vector3d &_size);

    /// \brief Get the heightmap's position offset.
    /// \return The heightmap's position offset.
    public: ignition::math::Vector3d Position() const;

    /// \brief Set the heightmap's position offset.
    /// \return The heightmap's position offset.
    public: void SetPosition(const ignition::math::Vector3d &_position);

    /// \brief Get whether the heightmap uses terrain paging.
    /// \return True if the heightmap uses terrain paging.
    public: bool UseTerrainPaging() const;

    /// \brief Set whether the heightmap uses terrain paging. Defaults to false.
    /// \param[in] _use True to use.
    public: void SetUseTerrainPaging(bool _use);

    /// \brief Get the heightmap's sampling per datum.
    /// \return The heightmap's sampling.
    public: unsigned int Sampling() const;

    /// \brief Set the heightmap's sampling. Defaults to 1.
    /// \param[in] _sampling The heightmap's sampling per datum.
    public: void SetSampling(unsigned int _sampling);

    /// \brief Get the number of heightmap textures.
    /// \return Number of heightmap textures contained in this Heightmap object.
    public: uint64_t TextureCount() const;

    /// \brief Get a heightmap texture based on an index.
    /// \param[in] _index Index of the heightmap texture. The index should be in
    /// the range [0..TextureCount()).
    /// \return Pointer to the heightmap texture. Nullptr if the index does not
    /// exist.
    /// \sa uint64_t TextureCount() const
    public: const HeightmapTexture *TextureByIndex(uint64_t _index) const;

    /// \brief Add a heightmap texture.
    /// \param[in] _texture Texture to add.
    public: void AddTexture(const HeightmapTexture &_texture);

    /// \brief Get the number of heightmap blends.
    /// \return Number of heightmap blends contained in this Heightmap object.
    public: uint64_t BlendCount() const;

    /// \brief Get a heightmap blend based on an index.
    /// \param[in] _index Index of the heightmap blend. The index should be in
    /// the range [0..BlendCount()).
    /// \return Pointer to the heightmap blend. Nullptr if the index does not
    /// exist.
    /// \sa uint64_t BlendCount() const
    public: const HeightmapBlend *BlendByIndex(uint64_t _index) const;

    /// \brief Add a heightmap blend.
    /// \param[in] _blend Blend to add.
    public: void AddBlend(const HeightmapBlend &_blend);

    /// \internal
    /// \brief Private data
    IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
    private: std::unique_ptr<HeightmapDescriptorPrivate> dataPtr;
    IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING
  };
}
}
}
#endif
