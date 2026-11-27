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
#ifndef GZ_RENDERING_BASE_BASENODE_HH_
#define GZ_RENDERING_BASE_BASENODE_HH_

#include <map>
#include <string>

#include "gz/rendering/Node.hh"
#include "gz/rendering/Storage.hh"
#include "gz/rendering/base/BaseStorage.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    template <class T>
    class BaseNode :
      public virtual Node,
      public virtual T
    {
      protected: BaseNode();

      public: virtual ~BaseNode();

      public: virtual NodePtr Parent() const override = 0;

      // Documentation inherited
      public: virtual void RemoveParent() override;

      public: virtual math::Vector3d LocalPosition() const override;

      public: virtual math::Pose3d LocalPose() const override;

      // Documentation inherited
      public: virtual math::Pose3d InitialLocalPose() const override;

      public: virtual void SetLocalPose(const math::Pose3d &_pose) override;

      public: virtual void SetLocalPosition(double _x, double _y, double _z)
          override;

      public: virtual void SetLocalPosition(const math::Vector3d &_position)
          override;

      public: virtual math::Quaterniond LocalRotation() const override;

      public: virtual void SetLocalRotation(double _r, double _p, double _y)
          override;

      public: virtual void SetLocalRotation(double _w, double _x, double _y,
                  double _z) override;

      public: virtual void SetLocalRotation(const math::Quaterniond &_rotation)
          override;

      public: virtual math::Pose3d WorldPose() const override;

      public: virtual void SetWorldPose(const math::Pose3d &_pose) override;

      public: virtual math::Vector3d WorldPosition() const override;

      public: virtual void SetWorldPosition(double _x, double _y, double _z)
         override;

      public: virtual void SetWorldPosition(const math::Vector3d &_position)
          override;

      public: virtual math::Quaterniond WorldRotation() const override;

      public: virtual void SetWorldRotation(double _r, double _p, double _y)
          override;

      public: virtual void SetWorldRotation(double _w, double _x, double _y,
                  double _z) override;

      public: virtual void SetWorldRotation(const math::Quaterniond &_rotation)
          override;

      public: virtual math::Pose3d WorldToLocal(const math::Pose3d &_pose)
          const override;

      public: virtual math::Vector3d Origin() const override;

      public: virtual void SetOrigin(double _x, double _y, double _z) override;

      public: virtual void SetOrigin(const math::Vector3d &_origin) override;

      // Documentation inherited
      public: virtual math::Vector3d LocalScale() const override = 0;

      // Documentation inherited
      public: virtual void SetLocalScale(double _scale) override;

      // Documentation inherited
      public: virtual void SetLocalScale(double _x, double _y, double _z)
                      override;

      // Documentation inherited
      public: virtual void SetLocalScale(const math::Vector3d &_scale) override;

      // Documentation inherited
      public: virtual math::Vector3d WorldScale() const override;

      // Documentation inherited
      public: virtual void SetWorldScale(double _scale) override;

      // Documentation inherited
      public: virtual void SetWorldScale(double _x, double _y, double _z)
                      override;

      // Documentation inherited
      public: virtual void SetWorldScale(const math::Vector3d &_scale) override;

      // Documentation inherited
      public: virtual void Scale(double _scale) override;

      // Documentation inherited
      public: virtual void Scale(double _x, double _y, double _z) override;

      // Documentation inherited
      public: virtual void Scale(const math::Vector3d &_scale) override;

      // Documentation inherited
      public: virtual bool InheritScale() const override = 0;

      public: virtual void Destroy() override;

      public: virtual unsigned int ChildCount() const override;

      public: virtual bool HasChild(ConstNodePtr _child) const override;

      public: virtual bool HasChildId(unsigned int _id) const override;

      public: virtual bool HasChildName(const std::string &_name) const
          override;

      public: virtual NodePtr ChildById(unsigned int _id) const override;

      public: virtual NodePtr ChildByName(const std::string &_name) const
          override;

      public: virtual NodePtr ChildByIndex(unsigned int _index) const override;

      public: virtual void AddChild(NodePtr _child) override;

      public: virtual NodePtr RemoveChild(NodePtr _child) override;

      public: virtual NodePtr RemoveChildById(unsigned int _id) override;

      public: virtual NodePtr RemoveChildByName(const std::string &_name)
          override;

      public: virtual NodePtr RemoveChildByIndex(unsigned int _index) override;

      public: virtual void RemoveChildren() override;

      public: virtual void PreRender() override;

      // Documentation inherited
      public: virtual void SetUserData(const std::string &_key, Variant _value)
        override;

      // Documentation inherited
      public: virtual Variant UserData(const std::string &_key) const override;

      // Documentation inherited
      public: virtual bool HasUserData(const std::string &_key) const override;

      protected: virtual void PreRenderChildren();

      protected: virtual math::Pose3d RawLocalPose() const = 0;

      protected: virtual void SetRawLocalPose(const math::Pose3d &_pose) = 0;

      protected: virtual NodeStorePtr Children() const = 0;

      protected: virtual bool AttachChild(NodePtr _child) = 0;

      protected: virtual bool DetachChild(NodePtr _child) = 0;

      /// \brief Implementation of the SetLocalScale function
      /// \param[in] _scale Scale to set the visual to
      protected: virtual void SetLocalScaleImpl(
                     const math::Vector3d &_scale) = 0;

      protected: math::Vector3d origin;

      /// \brief Flag to indicate whether initial local pose
      /// is set for this node.
      protected: bool initialLocalPoseSet = false;

      /// \brief Initial local pose for this node.
      protected: ignition::math::Pose3d initialLocalPose =
          ignition::math::Pose3d::Zero;

      /// \brief A map of custom key value data
      protected: std::map<std::string, Variant> userData;
    };

    //////////////////////////////////////////////////
    template <class T>
    BaseNode<T>::BaseNode()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    BaseNode<T>::~BaseNode()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseNode<T>::RemoveParent()
    {
      NodePtr parent = this->Parent();

      if (parent)
      {
        auto baseShared = this->shared_from_this();
        auto thisShared = std::dynamic_pointer_cast<BaseNode<T>>(baseShared);
        parent->RemoveChild(thisShared);
      }
    }


    //////////////////////////////////////////////////
    template <class T>
    void BaseNode<T>::AddChild(NodePtr _child)
    {
      if (_child->Id() == this->Id())
      {
        ignerr << "Cannot add self as a child node" << std::endl;
        return;
      }

      if (this->AttachChild(_child))
      {
        this->Children()->Add(_child);
      }
    }

    //////////////////////////////////////////////////
    template <class T>
    NodePtr BaseNode<T>::RemoveChild(NodePtr _child)
    {
      NodePtr child = this->Children()->Remove(_child);
      if (child) this->DetachChild(child);
      return child;
    }

    //////////////////////////////////////////////////
    template <class T>
    NodePtr BaseNode<T>::RemoveChildById(unsigned int _id)
    {
      NodePtr child = this->Children()->RemoveById(_id);
      if (child) this->DetachChild(child);
      return child;
    }

    //////////////////////////////////////////////////
    template <class T>
    NodePtr BaseNode<T>::RemoveChildByName(const std::string &_name)
    {
      NodePtr child = this->Children()->RemoveByName(_name);
      if (child) this->DetachChild(child);
      return child;
    }

    //////////////////////////////////////////////////
    template <class T>
    NodePtr BaseNode<T>::RemoveChildByIndex(unsigned int _index)
    {
      NodePtr child = this->Children()->RemoveByIndex(_index);
      if (child) this->DetachChild(child);
      return child;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseNode<T>::RemoveChildren()
    {
      for (unsigned int i = this->ChildCount(); i > 0; --i)
      {
        this->RemoveChildByIndex(i - 1);
      }
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseNode<T>::PreRender()
    {
      this->PreRenderChildren();
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseNode<T>::PreRenderChildren()
    {
      unsigned int count = this->ChildCount();

      for (unsigned int i = 0; i < count; ++i)
      {
        this->ChildByIndex(i)->PreRender();
      }
    }

    //////////////////////////////////////////////////
    template <class T>
    math::Pose3d BaseNode<T>::LocalPose() const
    {
      math::Pose3d pose = this->RawLocalPose();
      pose.Pos() += pose.Rot() * this->origin;
      return pose;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseNode<T>::SetLocalPose(const math::Pose3d &_pose)
    {
      if (!_pose.IsFinite())
      {
        ignerr << "Unable to set non-finite pose [" << _pose
               << "] to node [" << this->Name() << "]" << std::endl;
        return;
      }

      math::Pose3d pose = _pose;
      pose.Pos() = pose.Pos() - pose.Rot() * this->origin;

      if (!initialLocalPoseSet)
      {
        this->initialLocalPose = pose;
        this->initialLocalPoseSet = true;
      }

      this->SetRawLocalPose(pose);
    }

    //////////////////////////////////////////////////
    template <class T>
    math::Pose3d BaseNode<T>::InitialLocalPose() const
    {
      return this->initialLocalPose;
    }

    //////////////////////////////////////////////////
    template <class T>
    math::Vector3d BaseNode<T>::LocalPosition() const
    {
      return this->LocalPose().Pos();
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseNode<T>::SetLocalPosition(double _x, double _y, double _z)
    {
      this->SetLocalPosition(math::Vector3d(_x, _y, _z));
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseNode<T>::SetLocalPosition(const math::Vector3d &_position)
    {
      if (!_position.IsFinite())
      {
        ignerr << "Unable to set non-finite position [" << _position
               << "] to node [" << this->Name() << "]" << std::endl;
        return;
      }

      math::Pose3d pose = this->LocalPose();
      pose.Pos() = _position;
      this->SetLocalPose(pose);
    }

    //////////////////////////////////////////////////
    template <class T>
    math::Quaterniond BaseNode<T>::LocalRotation() const
    {
      return this->LocalPose().Rot();
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseNode<T>::SetLocalRotation(double _r, double _p, double _y)
    {
      this->SetLocalRotation(math::Quaterniond(_r, _p, _y));
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseNode<T>::SetLocalRotation(double _w, double _x, double _y,
        double _z)
    {
      this->SetLocalRotation(math::Quaterniond(_w, _x, _y, _z));
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseNode<T>::SetLocalRotation(const math::Quaterniond &_rotation)
    {
      if (!_rotation.IsFinite())
      {
        ignerr << "Unable to set non-finite rotation [" << _rotation
               << "] to node [" << this->Name() << "]" << std::endl;
        return;
      }

      math::Pose3d pose = this->LocalPose();
      pose.Rot() = _rotation;
      this->SetLocalPose(pose);
    }

    //////////////////////////////////////////////////
    template <class T>
    math::Pose3d BaseNode<T>::WorldPose() const
    {
      NodePtr parent = this->Parent();
      math::Pose3d pose = this->LocalPose();

      if (!parent)
      {
        return pose;
      }

      return parent->WorldPose() * pose;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseNode<T>::SetWorldPose(const math::Pose3d &_pose)
    {
      math::Pose3d pose = this->WorldToLocal(_pose);
      this->SetLocalPose(pose);
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseNode<T>::SetWorldPosition(double _x, double _y, double _z)
    {
      this->SetWorldPosition(math::Vector3d(_x, _y, _z));
    }

    //////////////////////////////////////////////////
    template <class T>
    math::Vector3d BaseNode<T>::WorldPosition() const
    {
      return this->WorldPose().Pos();
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseNode<T>::SetWorldPosition(const math::Vector3d &_position)
    {
      math::Pose3d pose = this->WorldPose();
      pose.Pos() = _position;
      this->SetWorldPose(pose);
    }

    //////////////////////////////////////////////////
    template <class T>
    math::Quaterniond BaseNode<T>::WorldRotation() const
    {
      return this->WorldPose().Rot();
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseNode<T>::SetWorldRotation(double _r, double _p, double _y)
    {
      this->SetWorldRotation(math::Quaterniond(_r, _p, _y));
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseNode<T>::SetWorldRotation(double _w, double _x, double _y,
        double _z)
    {
      this->SetWorldRotation(math::Quaterniond(_w, _x, _y, _z));
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseNode<T>::SetWorldRotation(const math::Quaterniond &_rotation)
    {
      math::Pose3d pose = this->WorldPose();
      pose.Rot() = _rotation;
      this->SetWorldPose(pose);
    }

    //////////////////////////////////////////////////
    template <class T>
    math::Pose3d BaseNode<T>::WorldToLocal(const math::Pose3d &_pose) const
    {
      NodePtr parent = this->Parent();

      if (!parent)
      {
        return _pose;
      }

      return _pose - parent->WorldPose();
    }

    //////////////////////////////////////////////////
    template <class T>
    math::Vector3d BaseNode<T>::Origin() const
    {
      return this->origin;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseNode<T>::SetOrigin(double _x, double _y, double _z)
    {
      this->SetOrigin(math::Vector3d(_x, _y, _z));
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseNode<T>::SetOrigin(const math::Vector3d &_origin)
    {
      if (!_origin.IsFinite())
      {
        ignerr << "Unable to set non-finite origin [" << _origin
               << "] to node [" << this->Name() << "]" << std::endl;
        return;
      }
      this->origin = _origin;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseNode<T>::SetLocalScale(double _scale)
    {
      this->SetLocalScale(math::Vector3d(_scale, _scale, _scale));
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseNode<T>::SetLocalScale(double _x, double _y, double _z)
    {
      this->SetLocalScale(math::Vector3d(_x, _y, _z));
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseNode<T>::SetLocalScale(const math::Vector3d &_scale)
    {
      math::Pose3d rawPose = this->LocalPose();
      this->SetLocalScaleImpl(_scale);
      this->SetLocalPose(rawPose);
    }

    //////////////////////////////////////////////////
    template <class T>
    math::Vector3d BaseNode<T>::WorldScale() const
    {
      math::Vector3d scale = this->LocalScale();

      if (!this->InheritScale() || !this->HasParent())
      {
        return scale;
      }

      return scale * this->Parent()->WorldScale();
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseNode<T>::SetWorldScale(double _scale)
    {
      this->SetWorldScale(math::Vector3d(_scale, _scale, _scale));
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseNode<T>::SetWorldScale(double _x, double _y, double _z)
    {
      this->SetWorldScale(math::Vector3d(_x, _y, _z));
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseNode<T>::SetWorldScale(const math::Vector3d &_scale)
    {
      math::Vector3d toScale = math::Vector3d::One;
      if (this->InheritScale() && this->HasParent())
        toScale = this->Parent()->WorldScale();

      this->SetLocalScale(_scale / toScale);
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseNode<T>::Scale(double _scale)
    {
      this->Scale(math::Vector3d(_scale, _scale, _scale));
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseNode<T>::Scale(double _x, double _y, double _z)
    {
      this->Scale(math::Vector3d(_x, _y, _z));
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseNode<T>::Scale(const math::Vector3d &_scale)
    {
      this->SetLocalScale(_scale * this->LocalScale());
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseNode<T>::Destroy()
    {
      T::Destroy();
      this->RemoveParent();
    }

    //////////////////////////////////////////////////
    template <class T>
    unsigned int BaseNode<T>::ChildCount() const
    {
      return this->Children()->Size();
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseNode<T>::HasChild(ConstNodePtr _child) const
    {
      return this->Children()->Contains(_child);
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseNode<T>::HasChildId(unsigned int _id) const
    {
      return this->Children()->ContainsId(_id);
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseNode<T>::HasChildName(const std::string &_name) const
    {
      return this->Children()->ContainsName(_name);
    }

    //////////////////////////////////////////////////
    template <class T>
    NodePtr BaseNode<T>::ChildById(unsigned int _id) const
    {
      return this->Children()->GetById(_id);
    }

    //////////////////////////////////////////////////
    template <class T>
    NodePtr BaseNode<T>::ChildByName(const std::string &_name) const
    {
      return this->Children()->GetByName(_name);
    }

    //////////////////////////////////////////////////
    template <class T>
    NodePtr BaseNode<T>::ChildByIndex(unsigned int _index) const
    {
      return this->Children()->GetByIndex(_index);
    }
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseNode<T>::SetUserData(const std::string &_key, Variant _value)
    {
     this->userData[_key] = _value;
    }

    //////////////////////////////////////////////////
    template <class T>
    Variant BaseNode<T>::UserData(const std::string &_key) const
    {
     Variant value;
     auto it = this->userData.find(_key);
     if (it != this->userData.end())
       value = it->second;
     return value;
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseNode<T>::HasUserData(const std::string &_key) const
    {
      return this->userData.find(_key) != this->userData.end();
    }
  }
}
#endif
