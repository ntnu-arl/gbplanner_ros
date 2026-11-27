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

#ifndef IGNITION_RENDERING_OGRE_OGREHEIGHTMAP_HH_
#define IGNITION_RENDERING_OGRE_OGREHEIGHTMAP_HH_

#include <memory>
#include <string>
#include <vector>

#include "ignition/rendering/base/BaseHeightmap.hh"
#include "ignition/rendering/ogre/OgreGeometry.hh"
#include "ignition/rendering/ogre/OgreIncludes.hh"

// Ignoring warning: "non dll-interface class
// 'ignition::rendering::v5::Heightmap' used as base for dll-interface class"
// because `Heightmap` and `BaseHeightmap` are header-only
#ifdef _MSC_VER
 #pragma warning(push)
 #pragma warning(disable:4275)
#endif

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    // Forward declaration
    class OgreHeightmapPrivate;

    /// \brief Ogre implementation of a heightmap geometry.
    class IGNITION_RENDERING_OGRE_VISIBLE OgreHeightmap
      : public BaseHeightmap<OgreGeometry>
    {
      /// \brief Constructor
      protected: explicit OgreHeightmap(const HeightmapDescriptor &_desc);

      /// \brief Destructor
      public: virtual ~OgreHeightmap();

      // Documentation inherited.
      public: virtual void Init() override;

      // Documentation inherited.
      public: virtual void PreRender() override;

      /// \brief Returns NULL, heightmaps don't have movable objects.
      /// \return Null pointer.
      public: virtual Ogre::MovableObject *OgreObject() const override;

      /// \brief Returns NULL, heightmap materials don't inherit from
      /// MaterialPtr.
      /// \return Null pointer.
      public: virtual MaterialPtr Material() const override;

      /// \brief Has no effect for heightmaps. The material is set through a
      /// HeightmapDescriptor.
      /// \param[in] _material Not used.
      /// \param[in] _unique Not used.
      public: virtual void SetMaterial(MaterialPtr _material, bool _unique)
          override;

      /// \brief Configure the terrain default values.
      /// \todo Move to private?
      private: void ConfigureTerrainDefaults();

      /// \brief Checks if the terrain was previously loaded by comparing its
      /// hash against the one stored in the terrain directory
      /// \param[in] _terrainDirPath Path to the directory containing the
      /// terrain files and hash.
      /// \return True if the terrain requires to regenerate the terrain files.
      private: bool PrepareTerrain(const std::string &_terrainDirPath);

      /// \brief Update the hash of a terrain file. The hash will be written in
      /// a file called gzterrain.SHA1 . This method will be used when the
      /// paging is enabled and the terrain is loaded for the first time or if
      /// the heightmap's image has been modified.
      /// \param[in] _hash New hash value
      /// \param[in] _terrainDir Directory where the terrain hash and the
      /// terrain pages are stored. Ex: $TMP/gazebo-paging/heigthmap_bowl
      private: void UpdateTerrainHash(const std::string &_hash,
          const std::string &_terrainDir);

      /// \brief Split a terrain into subterrains
      /// \param[in] _heightmap Source vector of floats with the heights.
      /// \param[in] _n Number of subterrains.
      /// \param[out] _v Destination vector with the subterrains.
      private: void SplitHeights(const std::vector<float> &_heightmap,
                  int _n, std::vector<std::vector<float>> &_v);

      /// \brief Define a section of the terrain.
      /// \param[in] _x X coordinate of the terrain.
      /// \param[in] _y Y coordinate of the terrain.
      private: void DefineTerrain(int _x, int _y);

      /// \brief Create terrain material generator. There are two types:
      /// custom material generator that support user material scripts,
      /// and a default material generator that uses our own glsl shader
      /// and supports PSSM shadows.
      private: void CreateMaterial();

      /// \brief Initialize all the blend material maps.
      /// \param[in] _terrain The terrain to initialize the blend maps.
      private: bool InitBlendMaps(Ogre::Terrain *_terrain);

      /// \brief Internal function used to setup shadows for the terrain.
      /// \param[in] _enabled True to enable shadows.
      private: void SetupShadows(bool _enabled);

      /// \brief Heightmap should only be created by scene.
      private: friend class OgreScene;

      /// \brief Private data class
      private: std::unique_ptr<OgreHeightmapPrivate> dataPtr;
    };
    }
  }
}
#ifdef _MSC_VER
 #pragma warning(pop)
#endif

#endif
