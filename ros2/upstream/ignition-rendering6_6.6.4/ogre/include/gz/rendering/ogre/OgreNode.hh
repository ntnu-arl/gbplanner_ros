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
#ifndef GZ_RENDERING_OGRE_OGRENODE_HH_
#define GZ_RENDERING_OGRE_OGRENODE_HH_

#include <gz/common/SuppressWarning.hh>

#include "gz/rendering/base/BaseNode.hh"
#include "gz/rendering/ogre/OgreRenderTypes.hh"
#include "gz/rendering/ogre/OgreObject.hh"

namespace Ogre
{
  class SceneNode;
}

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    class IGNITION_RENDERING_OGRE_VISIBLE OgreNode :
      public BaseNode<OgreObject>
    {
      protected: OgreNode();

      public: virtual ~OgreNode();

      public: virtual bool HasParent() const override;

      public: virtual NodePtr Parent() const override;

      public: virtual Ogre::SceneNode *Node() const;

      public: virtual void Destroy() override;

      // Documentation inherited.
      public: virtual math::Vector3d LocalScale() const override;

      // Documentation inherited.
      public: virtual bool InheritScale() const override;

      // Documentation inherited.
      public: virtual void SetInheritScale(bool _inherit) override;

      // Documentation inherited.
      protected: virtual void SetLocalScaleImpl(
                     const math::Vector3d &_scale) override;

      protected: virtual NodeStorePtr Children() const override;

      protected: virtual bool AttachChild(NodePtr _child) override;

      protected: virtual bool DetachChild(NodePtr _child) override;

      protected: virtual math::Pose3d RawLocalPose() const override;

      protected: virtual void SetRawLocalPose(const math::Pose3d &_Pose3d)
          override;

      protected: virtual math::Vector3d RawLocalPosition() const;

      protected: virtual void SetRawLocalPosition(
                     const math::Vector3d &_position);

      protected: virtual math::Quaterniond RawLocalRotation() const;

      protected: virtual void SetRawLocalRotation(
                     const math::Quaterniond &_rotation);

      protected: virtual void SetParent(OgreNodePtr _parent);

      protected: virtual void Load() override;

      protected: virtual void Init() override;

      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      protected: OgreNodePtr parent;
      IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING

      protected: Ogre::SceneNode *ogreNode = nullptr;

      protected: OgreNodeStorePtr children;

      private: OgreNodePtr SharedThis();

      // TODO(anyone): remove the need for a visual friend class
      private: friend class OgreVisual;
    };
    }
  }
}
#endif
