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
#ifndef GZ_RENDERING_NODE_HH_
#define GZ_RENDERING_NODE_HH_

#include <map>
#include <string>
#include <variant>

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>

#include "gz/rendering/config.hh"
#include "gz/rendering/RenderTypes.hh"
#include "gz/rendering/Object.hh"
#include "gz/rendering/Export.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \brief Alias for a variant that can hold various types of data.
    /// The first type of the variant is std::monostate in order to prevent
    /// default-constructed variants from holding a type (a default-constructed
    /// variant is returned when a user calls Node::UserData with a key that
    /// doesn't exist for the node. In this case, since the key doesn't
    /// exist, the variant that is returned shouldn't hold any types - an
    /// "empty variant" should be returned for keys that don't exist)
    using Variant =
      std::variant<std::monostate, int, float, double, std::string, bool,
        unsigned int>;

    /// \class Node Node.hh ignition/rendering/Node.hh
    /// \brief Represents a single posable node in the scene graph
    class IGNITION_RENDERING_VISIBLE Node :
      public virtual Object
    {
      /// \brief Deconstructor
      public: virtual ~Node() { }

      /// \brief Determine if this Node is attached to another Node.
      /// \return True if this Node has a parent Node
      public: virtual bool HasParent() const = 0;

      /// \brief Get the parent Node
      /// \return the parent Node
      public: virtual NodePtr Parent() const = 0;

      /// \brief Detach this Node from its parent. If this
      /// Node does not have a parent, no work will be done.
      public: virtual void RemoveParent() = 0;

      /// \brief Get the local pose
      /// \return The local pose
      public: virtual math::Pose3d LocalPose() const = 0;

      /// \brief Get the initial local pose
      /// \return The initial local pose
      public: virtual math::Pose3d InitialLocalPose() const = 0;

      /// \brief Set the local pose
      /// \param[in] _pose New local pose
      public: virtual void SetLocalPose(const math::Pose3d &_pose) = 0;

      /// \brief Get the local position
      /// \return The local position
      public: virtual math::Vector3d LocalPosition() const = 0;

      /// \brief Set the local position
      /// \param[in] _x X-coordinate
      /// \param[in] _y Y-coordinate
      /// \param[in] _z Z-coordinate
      public: virtual void SetLocalPosition(double _x, double _y,
                  double _z) = 0;

      /// \brief Set the local position
      /// \param[in] _position New local position
      public: virtual void SetLocalPosition(
                  const math::Vector3d &_position) = 0;

      /// \brief Get the local rotation
      /// \return The local rotation
      public: virtual math::Quaterniond LocalRotation() const = 0;

      /// \brief Set the local rotation
      /// \param[in] _r roll
      /// \param[in] _p pitch
      /// \param[in] _y yaw
      public: virtual void SetLocalRotation(double _r, double _p,
                  double _y) = 0;

      /// \brief Set the local rotation
      /// \param[in] _w W-coordinate
      /// \param[in] _x X-coordinate
      /// \param[in] _y Y-coordinate
      /// \param[in] _z Z-coordinate
      public: virtual void SetLocalRotation(double _w, double _x, double _y,
                  double _z) = 0;

      /// \brief Set the local rotation
      /// \param[in] _rotation New local rotation
      public: virtual void SetLocalRotation(
                  const math::Quaterniond &_rotation) = 0;

      /// \brief Get the world pose
      /// \return The world pose
      public: virtual math::Pose3d WorldPose() const = 0;

      /// \brief Set the world pose
      /// \param[in] _pose New world pose
      public: virtual void SetWorldPose(const math::Pose3d &_pose) = 0;

      /// \brief Get the world position
      /// \return The world position
      public: virtual math::Vector3d WorldPosition() const = 0;

      /// \brief Set the world position
      /// \param[in] _x X-coordinate
      /// \param[in] _y Y-coordinate
      /// \param[in] _z Z-coordinate
      public: virtual void SetWorldPosition(double _x, double _y,
                  double _z) = 0;

      /// \brief Set the world position
      /// \param[in] _position New world position
      public: virtual void SetWorldPosition(
                  const math::Vector3d &_position) = 0;

      /// \brief Get the world rotation
      /// \return The world rotation
      public: virtual math::Quaterniond WorldRotation() const = 0;

      /// \brief Set the world rotation
      /// \param[in] _r roll
      /// \param[in] _p pitch
      /// \param[in] _y yaw
      public: virtual void SetWorldRotation(double _r, double _p,
                  double _y) = 0;

      /// \brief Set the world rotation
      /// \param[in] _w W-coordinate
      /// \param[in] _x X-coordinate
      /// \param[in] _y Y-coordinate
      /// \param[in] _z Z-coordinate
      public: virtual void SetWorldRotation(double _w, double _x, double _y,
                  double _z) = 0;

      /// \brief Set the world rotation
      /// \param[in] _rotation New world rotation
      public: virtual void SetWorldRotation(
                  const math::Quaterniond &_rotation) = 0;

      /// \brief Convert given world pose to local pose
      /// \param[in] _pose World pose to be converted
      public: virtual math::Pose3d WorldToLocal(
                  const math::Pose3d &_pose) const = 0;

      /// \brief Get position of origin
      /// \return The position of the origin
      public: virtual math::Vector3d Origin() const = 0;

      /// \brief Set position of origin. The position should be relative to the
      /// original origin of the geometry.
      /// \param[in] _x X-coordinate
      /// \param[in] _y Y-coordinate
      /// \param[in] _z Z-coordinate
      public: virtual void SetOrigin(double _x, double _y, double _z) = 0;

      /// \brief Set position of origin. The position should be relative to the
      /// original origin of the geometry.
      /// \param[in] _origin New origin position
      public: virtual void SetOrigin(const math::Vector3d &_origin) = 0;

      /// \brief Get the local scale
      /// \return The local scale
      public: virtual math::Vector3d LocalScale() const = 0;

      /// \brief Set the local scale. The given scale will be assigned to the
      /// x, y, and z coordinates.
      /// \param[in] _scale New local scale
      public: virtual void SetLocalScale(double _scale) = 0;

      /// \brief Set the local scale
      /// \param[in] _x New x-coordinate scale
      /// \param[in] _y New y-coordinate scale
      /// \param[in] _z New z-coordinate scale
      public: virtual void SetLocalScale(double _x, double _y, double _z) = 0;

      /// \brief Set the local scale
      /// \param[in] _scale New local scale
      public: virtual void SetLocalScale(const math::Vector3d &_scale) = 0;

      /// \brief Get the world scale
      /// \return The world scale
      public: virtual math::Vector3d WorldScale() const = 0;

      /// \brief Set the world scale. The given scale will be assigned to the
      /// x, y, and z coordinates.
      /// \param[in] _scale New world scale
      public: virtual void SetWorldScale(double _scale) = 0;

      /// \brief Set the world scale
      /// \param[in] _x New x-coordinate scale
      /// \param[in] _y New y-coordinate scale
      /// \param[in] _z New z-coordinate scale
      public: virtual void SetWorldScale(double _x, double _y, double _z) = 0;

      /// \brief Set the world scale
      /// \param[in] _scale New world scale
      public: virtual void SetWorldScale(const math::Vector3d &_scale) = 0;

      /// \brief Scale the current scale by the given scalar. The given scalar
      /// will be assigned to the x, y, and z coordinates.
      /// \param[in] _scale Scalar to alter the current scale
      public: virtual void Scale(double _scale) = 0;

      /// \brief Scale the current scale by the given scalars
      /// \param[in] _x Scalar to alter the current x-coordinate scale
      /// \param[in] _y Scalar to alter the current y-coordinate scale
      /// \param[in] _z Scalar to alter the current z-coordinate scale
      public: virtual void Scale(double _x, double _y, double _z) = 0;

      /// \brief Scale the current scale by the given scalars
      /// \param[in] _scale Scalars to alter the current scale
      public: virtual void Scale(const math::Vector3d &_scale) = 0;

      /// \brief Determine if this visual inherits scale from this parent
      /// \return True if this visual inherits scale from this parent
      public: virtual bool InheritScale() const = 0;

      /// \brief Specify if this visual inherits scale from its parent
      /// \param[in] _inherit True if this visual inherits scale from its parent
      public: virtual void SetInheritScale(bool _inherit) = 0;

      /// \brief Get number of child nodes
      /// \return The number of child nodes
      public: virtual unsigned int ChildCount() const = 0;

      /// \brief Determine if given node is an attached child
      /// \return True if given node is an attached child
      public: virtual bool HasChild(ConstNodePtr _child) const = 0;

      /// \brief Determine if node with given ID is an attached child
      /// \param[in] _id ID of the node in question
      /// \return True if node with given ID is an attached child
      public: virtual bool HasChildId(unsigned int _id) const = 0;

      /// \brief Determine if node with given name is an attached child
      /// \param[in] _name Name of the node in question
      /// \return True if node with given name is an attached child
      public: virtual bool HasChildName(const std::string &_name) const = 0;

      /// \brief Get node with given ID. If no child exists with given ID, NULL
      /// will be returned.
      /// \param[in] _id ID of the desired node
      /// \return The specified node
      public: virtual NodePtr ChildById(unsigned int _id) const = 0;

      /// \brief Get node with given name. If no child exists with given name,
      /// NULL will be returned.
      /// \param[in] _name Name of the desired node
      /// \return The specified node
      public: virtual NodePtr ChildByName(
                  const std::string &_name) const = 0;

      /// \brief Get node at given index. If no child exists at given index,
      /// NULL will be returned.
      /// \param[in] _index Index of the desired node
      /// \return The specified node
      public: virtual NodePtr ChildByIndex(unsigned int _index) const = 0;

      /// \brief Add the given node to this node. If the given node is
      /// already a child, no work will be done.
      /// \param[in] _child Child node to be added
      public: virtual void AddChild(NodePtr _child) = 0;

      /// \brief Remove (detach) the given node from this node. If the given
      /// node is not a child of this node, no work will be done.
      /// \param[in] _child Child node to be removed
      /// \return The removed child node
      public: virtual NodePtr RemoveChild(NodePtr _child) = 0;

      /// \brief Remove (detach) the node with the given ID from this node.
      /// If the specified node is not a child of this node, no work will be
      /// done.
      /// \param[in] _id ID of the child node to be removed
      /// \return The removed child node
      public: virtual NodePtr RemoveChildById(unsigned int _id) = 0;

      /// \brief Remove (detach) the node with the given name from this node.
      /// If the specified node is not a child of this node, no work will be
      /// done.
      /// \param[in] _name Name of the child node to be removed
      /// \return The removed child node
      public: virtual NodePtr RemoveChildByName(const std::string &_name) = 0;

      /// \brief Remove (detach) the node at the given index from this node.
      /// If the specified node is not a child of this node, no work will be
      /// done.
      /// \param[in] _index Index of the child node to be removed
      /// \return The removed child node
      public: virtual NodePtr RemoveChildByIndex(unsigned int _index) = 0;

      /// \brief Remove all child nodes from this node
      /// This detaches all the child nodes but does not destroy them
      public: virtual void RemoveChildren() = 0;

      /// \brief Store any custom data associated with this node
      /// \param[in] _key Unique key
      /// \param[in] _value Value in any type
      public: virtual void SetUserData(
        const std::string &_key, Variant _value) = 0;

      /// \brief Get custom data stored in this node
      /// \param[in] _key Unique key
      /// \return Value in any type. If _key does not exist for the node, an
      /// empty variant is returned (i.e., no data).
      public: virtual Variant UserData(const std::string &_key) const = 0;

      /// \brief Check if node has custom data
      /// \param[in] _key Unique key
      /// \return True if node has custom data with the specified key
      public: virtual bool HasUserData(const std::string &_key) const = 0;
    };
    }
  }
}
#endif
