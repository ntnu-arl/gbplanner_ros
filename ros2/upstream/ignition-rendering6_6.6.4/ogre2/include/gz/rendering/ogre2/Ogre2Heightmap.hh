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

#ifndef IGNITION_RENDERING_OGRE2_OGRE2HEIGHTMAP_HH_
#define IGNITION_RENDERING_OGRE2_OGRE2HEIGHTMAP_HH_

#include <memory>

#include "ignition/rendering/base/BaseHeightmap.hh"
#include "ignition/rendering/ogre2/Ogre2Geometry.hh"

// Ignoring warning: "non dll-interface class
// 'ignition::rendering::v5::Heightmap' used as base for dll-interface class"
// because `Heightmap` and `BaseHeightmap` are header-only
#ifdef _MSC_VER
 #pragma warning(push)
 #pragma warning(disable:4275)
#endif

namespace Ogre
{
  class Camera;
  class Terra;
}

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    // Forward declaration
    class Ogre2HeightmapPrivate;

    /// \brief Ogre implementation of a heightmap geometry.
    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2Heightmap
      : public BaseHeightmap<Ogre2Geometry>
    {
      /// \brief Constructor
      /// \param[in] _desc Parameters describing how a
      /// heightmap should be loaded
      public: explicit Ogre2Heightmap(const HeightmapDescriptor &_desc);

      /// \brief Destructor
      public: virtual ~Ogre2Heightmap() override;

      // Documentation inherited.
      public: virtual void Init() override;

      // Documentation inherited.
      public: virtual void PreRender() override;

      /// \brief Returns the Terra pointer as it is a movable object that
      /// must be attached to a regular SceneNode
      /// \remarks This behavior is different from ogre1
      /// \return Terra pointer
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

      /// \internal
      /// \brief Retrieves the internal Terra pointer
      /// \return internal Terra pointer
      public: Ogre::Terra* Terra();

      /// \internal
      /// \brief Must be called before rendering with the camera
      /// that will perform rendering.
      ///
      /// May update shadows if light direction changed
      /// \param[in] _activeCamera Camera about to be used for rendering
      public: void UpdateForRender(Ogre::Camera *_activeCamera);

      // Documentation inherited.
      // \todo(iche033) rename this to Destroy and
      // make this function public and virtual
      private: void DestroyImpl();

      /// \brief Heightmap should only be created by scene.
      private: friend class OgreScene;

      /// \brief Private data class
      private: std::unique_ptr<Ogre2HeightmapPrivate> dataPtr;
    };
    }
  }
}
#ifdef _MSC_VER
 #pragma warning(pop)
#endif

#endif
