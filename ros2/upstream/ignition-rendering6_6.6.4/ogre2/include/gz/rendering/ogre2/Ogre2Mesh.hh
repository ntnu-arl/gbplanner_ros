/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#ifndef GZ_RENDERING_OGRE2_OGRE2MESH_HH_
#define GZ_RENDERING_OGRE2_OGRE2MESH_HH_

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include "gz/rendering/base/BaseMesh.hh"
#include "gz/rendering/ogre2/Ogre2Geometry.hh"
#include "gz/rendering/ogre2/Ogre2Object.hh"
#include "gz/rendering/ogre2/Ogre2RenderTypes.hh"

namespace Ogre
{
  class Item;
  class SubItem;
}

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    // forward declaration
    class Ogre2MeshPrivate;
    class Ogre2SubMeshPrivate;

    /// \brief Ogre2.x implementation of the mesh class
    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2Mesh :
      public BaseMesh<Ogre2Geometry>
    {
      /// \brief Constructor
      protected: Ogre2Mesh();

      /// \brief Destructor
      public: virtual ~Ogre2Mesh();

      // Documentation inherited
      public: virtual void Destroy() override;

      // Documentation inherited.
      public: virtual bool HasSkeleton() const override;

      // Documentation inherited.
      public: virtual std::map<std::string, math::Matrix4d>
                          SkeletonLocalTransforms() const override;

      // Documentation inherited.
      public: virtual void SetSkeletonLocalTransforms(
            const std::map<std::string, math::Matrix4d> &_tfs) override;

      // Documentation inherited.
      public: virtual std::unordered_map<std::string, float>
                          SkeletonWeights() const override;

      // Documentation inherited.
      public: virtual void SetSkeletonWeights(
            const std::unordered_map<std::string, float> &_weights) override;

      // Documentation inherited.
      public: virtual void SetSkeletonAnimationEnabled(const std::string &_name,
            bool _enabled, bool _loop = true, float _weight = 1.0) override;

      // Documentation inherited.
      public: virtual bool SkeletonAnimationEnabled(const std::string &_name)
            const override;

      // Documentation inherited.
      public: virtual void UpdateSkeletonAnimation(
            std::chrono::steady_clock::duration _time) override;

      // Documentation inherited
      public: virtual Ogre::MovableObject *OgreObject() const override;

      /// \brief Get a list of submeshes in this mesh
      protected: virtual SubMeshStorePtr SubMeshes() const override;

      /// \brief Store containing all the submeshes
      protected: Ogre2SubMeshStorePtr subMeshes;

      /// \brief Pointer to the ogre item object
      protected: Ogre::Item *ogreItem = nullptr;

      /// \brief Make scene our friend so it can create an ogre2 mesh
      private: friend class Ogre2Scene;

      /// \brief Make mesh factory our friend so it can create an ogre2 mesh
      private: friend class Ogre2MeshFactory;

      /// \brief Pointer to private data
      private: std::unique_ptr<Ogre2MeshPrivate> dataPtr;
    };

    /// \brief Ogre2.x implementation of the submesh class
    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2SubMesh :
      public BaseSubMesh<Ogre2Object>
    {
      /// \brief Constructor
      protected: Ogre2SubMesh();

      /// \brief Destructor
      public: virtual ~Ogre2SubMesh();

      // Documentation inherited
      public: virtual void Destroy() override;

      /// \brief Set the name of the mesh stored in Ogre2
      /// \param[in] _name Name of the mesh
      public: void SetMeshName(const std::string &_name);

      /// \brief Get internal ogre subitem created from this submesh
      public: virtual Ogre::SubItem *Ogre2SubItem() const;

      /// \brief Helper function for setting the material to use
      /// \param[in] _material Material to be assigned to the submesh
      protected: virtual void SetMaterialImpl(MaterialPtr _material) override;

      /// \brief Initialize the submesh
      protected: virtual void Init() override;

      /// \brief Ogre subitem representing the submesh
      protected: Ogre::SubItem *ogreSubItem = nullptr;

      /// \brief Make scene our friend so it can create an ogre2 mesh
      private: friend class Ogre2Scene;

      /// \brief Make submesh factory our friend so it can create an
      /// ogre2 submesh
      private: friend class Ogre2SubMeshStoreFactory;

      /// \brief Pointer to private data
      private: std::unique_ptr<Ogre2SubMeshPrivate> dataPtr;
    };
    }
  }
}
#endif
