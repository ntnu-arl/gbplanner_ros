/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef GZ_RENDERING_OGRE_OGREMESH_HH_
#define GZ_RENDERING_OGRE_OGREMESH_HH_

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include "gz/rendering/base/BaseMesh.hh"
#include "gz/rendering/ogre/OgreGeometry.hh"
#include "gz/rendering/ogre/OgreObject.hh"
#include "gz/rendering/ogre/OgreRenderTypes.hh"

namespace Ogre
{
  class Entity;
  class SubEntity;
}

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    // forward declarations
    class OgreMeshPrivate;

    class IGNITION_RENDERING_OGRE_VISIBLE OgreMesh :
      public BaseMesh<OgreGeometry>
    {
      typedef std::vector<std::string> NameList;

      protected: OgreMesh();

      public: virtual ~OgreMesh();

      // Documentation inherited.
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
      public: virtual std::unordered_map<std::string, float> SkeletonWeights()
            const override;

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

      public: virtual Ogre::MovableObject *OgreObject() const override;

      protected: virtual SubMeshStorePtr SubMeshes() const override;

      protected: OgreSubMeshStorePtr subMeshes;

      protected: Ogre::Entity *ogreEntity = nullptr;

      private: friend class OgreScene;

      private: friend class OgreMeshFactory;

      /// \brief Pointer to private data
      private: std::unique_ptr<OgreMeshPrivate> dataPtr;
    };

    class IGNITION_RENDERING_OGRE_VISIBLE OgreSubMesh :
      public BaseSubMesh<OgreObject>
    {
      protected: OgreSubMesh();

      public: virtual ~OgreSubMesh();

      public: virtual Ogre::SubEntity *OgreSubEntity() const;

      public: virtual void Destroy() override;

      // Documentation inherited
      protected: virtual void SetMaterialImpl(MaterialPtr _material) override;

      protected: virtual void Init() override;

      protected: Ogre::SubEntity *ogreSubEntity;

      private: friend class OgreScene;

      private: friend class OgreSubMeshStoreFactory;
    };
    }
  }
}
#endif
