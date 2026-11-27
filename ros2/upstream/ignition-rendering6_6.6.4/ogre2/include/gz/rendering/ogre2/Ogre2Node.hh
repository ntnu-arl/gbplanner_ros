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
#ifndef GZ_RENDERING_OGRE2_OGRE2NODE_HH_
#define GZ_RENDERING_OGRE2_OGRE2NODE_HH_

#include "gz/rendering/base/BaseNode.hh"
#include "gz/rendering/ogre2/Ogre2RenderTypes.hh"
#include "gz/rendering/ogre2/Ogre2Object.hh"

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
    /// \brief Ogre2.x implementation of the Node class
    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2Node :
      public BaseNode<Ogre2Object>
    {
      /// \brief Constructor
      protected: Ogre2Node();

      /// \brief Destructor
      public: virtual ~Ogre2Node();

      // Documentation inherited.
      public: virtual bool HasParent() const override;

      // Documentation inherited.
      public: virtual NodePtr Parent() const override;

      /// \brief Get a pointer to the underlying scene node
      /// \return Ogre scene node pointer
      public: virtual Ogre::SceneNode *Node() const;

      // Documentation inherited.
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

      // Documentation inherited.
      protected: virtual NodeStorePtr Children() const override;

      // Documentation inherited.
      protected: virtual bool AttachChild(NodePtr _child) override;

      // Documentation inherited.
      protected: virtual bool DetachChild(NodePtr _child) override;

      // Documentation inherited.
      protected: virtual math::Pose3d RawLocalPose() const override;

      // Documentation inherited.
      protected: virtual void SetRawLocalPose(const math::Pose3d &_Pose3d)
                     override;

      /// \brief Get the raw local position of the node
      /// \return Local position
      protected: virtual math::Vector3d RawLocalPosition() const;

      /// \brief Set the raw local position of the node
      /// \param[in] _position Local position to set the node to
      protected: virtual void SetRawLocalPosition(
                     const math::Vector3d &_position);

      /// \brief Get the raw local rotation of the node
      /// \return Local rotation
      protected: virtual math::Quaterniond RawLocalRotation() const;

      /// \brief Set the raw local rotation of the node
      /// \param[in] _rotation Local rotation to set the node to
      protected: virtual void SetRawLocalRotation(
                     const math::Quaterniond &_rotation);

      /// \brief Set the parent node
      /// \param[in] _parent The parent ogre node
      protected: virtual void SetParent(Ogre2NodePtr _parent);

      // Documentation inherited.
      protected: virtual void Load() override;

      // Documentation inherited.
      protected: virtual void Init() override;

      /// \brief get a shared pointer to this
      private: Ogre2NodePtr SharedThis();

      /// \brief Pointer to the parent ogre node
      protected: Ogre2NodePtr parent;

      /// \brief The underlying ogre scene node
      protected: Ogre::SceneNode *ogreNode = nullptr;

      /// \brief A list of child nodes
      protected: Ogre2NodeStorePtr children;

      // TODO(anyone): remove the need for a visual friend class
      private: friend class Ogre2Visual;
    };
    }
  }
}
#endif
