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

#include <gz/common/Console.hh>

#include "gz/rendering/ogre2/Ogre2Camera.hh"
#include "gz/rendering/ogre2/Ogre2Node.hh"
#include "gz/rendering/ogre2/Ogre2Conversions.hh"
#include "gz/rendering/ogre2/Ogre2Scene.hh"
#include "gz/rendering/ogre2/Ogre2Storage.hh"

#ifdef _MSC_VER
  #pragma warning(push, 0)
#endif
#include <OgreSceneManager.h>
#ifdef _MSC_VER
  #pragma warning(pop)
#endif

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
Ogre2Node::Ogre2Node()
{
}

//////////////////////////////////////////////////
Ogre2Node::~Ogre2Node()
{
}

//////////////////////////////////////////////////
bool Ogre2Node::HasParent() const
{
  return this->parent != nullptr;
}

//////////////////////////////////////////////////
NodePtr Ogre2Node::Parent() const
{
  return this->parent;
}

//////////////////////////////////////////////////
Ogre::SceneNode *Ogre2Node::Node() const
{
  return this->ogreNode;
}

//////////////////////////////////////////////////
void Ogre2Node::Destroy()
{
  if (!ogreNode)
    return;

  BaseNode::Destroy();

  if (nullptr != this->scene)
  {
    Ogre::SceneManager *ogreSceneManager = this->scene->OgreSceneManager();
    if (nullptr != ogreSceneManager)
      ogreSceneManager->destroySceneNode(this->ogreNode);
  }
  this->ogreNode = nullptr;
}

//////////////////////////////////////////////////
math::Pose3d Ogre2Node::RawLocalPose() const
{
  math::Pose3d Pose3d;
  Pose3d.Pos() = this->RawLocalPosition();
  Pose3d.Rot() = this->RawLocalRotation();
  return Pose3d;
}

//////////////////////////////////////////////////
void Ogre2Node::SetRawLocalPose(const math::Pose3d &_Pose3d)
{
  this->SetRawLocalPosition(_Pose3d.Pos());
  this->SetRawLocalRotation(_Pose3d.Rot());
}

//////////////////////////////////////////////////
math::Vector3d Ogre2Node::RawLocalPosition() const
{
  if (nullptr == this->ogreNode)
    return math::Vector3d();

  return Ogre2Conversions::Convert(this->ogreNode->getPosition());
}

//////////////////////////////////////////////////
void Ogre2Node::SetRawLocalPosition(const math::Vector3d &_position)
{
  if (nullptr == this->ogreNode)
    return;

  // ogre crashes in the compositor shadow pass with an
  // Ogre::AxisAlignedBox::setExtents assertion error when the camera scene node
  // position has large values. Added a workaround that places a max limit on
  // the length of the position vector.
  if (dynamic_cast<Ogre2Camera *>(this) && _position.Length() > 1e9)
  {
    ignerr << "Unable to set camera node position to a distance larger than "
           << "1e9 from origin" << std::endl;
    return;
  }
  this->ogreNode->setPosition(Ogre2Conversions::Convert(_position));
}

//////////////////////////////////////////////////
math::Quaterniond Ogre2Node::RawLocalRotation() const
{
  if (nullptr == this->ogreNode)
    return math::Quaterniond();

  return Ogre2Conversions::Convert(this->ogreNode->getOrientation());
}

//////////////////////////////////////////////////
void Ogre2Node::SetRawLocalRotation(const math::Quaterniond &_rotation)
{
  if (nullptr == this->ogreNode)
    return;

  this->ogreNode->setOrientation(Ogre2Conversions::Convert(_rotation));
}

//////////////////////////////////////////////////
void Ogre2Node::SetParent(Ogre2NodePtr _parent)
{
  this->parent = _parent;
}

//////////////////////////////////////////////////
void Ogre2Node::Load()
{
}

//////////////////////////////////////////////////
void Ogre2Node::Init()
{
  if (nullptr == this->scene)
  {
    ignerr << "Failed to initialize node: scene is NULL" << std::endl;
    return;
  }

  auto sceneManager = this->scene->OgreSceneManager();
  if (nullptr == sceneManager)
  {
    ignerr << "Failed to initialize node: scene manager is NULL" << std::endl;
    return;
  }

  this->ogreNode = sceneManager->createSceneNode();
  if (nullptr == this->ogreNode)
  {
    ignerr << "Failed to create Ogre node" << std::endl;
    return;
  }
  this->ogreNode->setInheritScale(true);
  this->children = Ogre2NodeStorePtr(new Ogre2NodeStore);
}

//////////////////////////////////////////////////
NodeStorePtr Ogre2Node::Children() const
{
  return this->children;
}

//////////////////////////////////////////////////
bool Ogre2Node::AttachChild(NodePtr _child)
{
  if (nullptr == this->ogreNode)
    return false;

  Ogre2NodePtr derived = std::dynamic_pointer_cast<Ogre2Node>(_child);

  if (!derived)
  {
    ignerr << "Cannot attach node created by another render-engine"
        << std::endl;
    return false;
  }

  // Check for loop. Ogre throws exception if child node to be added
  // is a direct ancestor of this node
  auto p = this->ogreNode->getParent();
  while (p != nullptr)
  {
    if (p == derived->Node())
    {
      ignerr << "Node cycle detected. Not adding Node: " << _child->Name()
             << std::endl;
      return false;
    }
    p = p->getParent();
  }

  derived->SetParent(this->SharedThis());
  this->ogreNode->addChild(derived->Node());
  return true;
}

//////////////////////////////////////////////////
bool Ogre2Node::DetachChild(NodePtr _child)
{
  if (nullptr == this->ogreNode)
    return false;

  Ogre2NodePtr derived = std::dynamic_pointer_cast<Ogre2Node>(_child);

  if (!derived)
  {
    ignerr << "Cannot detach node created by another render-engine"
        << std::endl;
    return false;
  }

  this->ogreNode->removeChild(derived->Node());

  return true;
}

//////////////////////////////////////////////////
Ogre2NodePtr Ogre2Node::SharedThis()
{
  ObjectPtr object = shared_from_this();
  return std::dynamic_pointer_cast<Ogre2Node>(object);
}

//////////////////////////////////////////////////
math::Vector3d Ogre2Node::LocalScale() const
{
  if (nullptr == this->ogreNode)
    return math::Vector3d();

  return Ogre2Conversions::Convert(this->ogreNode->getScale());
}

//////////////////////////////////////////////////
bool Ogre2Node::InheritScale() const
{
  if (nullptr == this->ogreNode)
    return false;

  return this->ogreNode->getInheritScale();
}

//////////////////////////////////////////////////
void Ogre2Node::SetInheritScale(bool _inherit)
{
  if (nullptr == this->ogreNode)
    return;

  this->ogreNode->setInheritScale(_inherit);
}

//////////////////////////////////////////////////
void Ogre2Node::SetLocalScaleImpl(const math::Vector3d &_scale)
{
  if (nullptr == this->ogreNode)
    return;

  this->ogreNode->setScale(Ogre2Conversions::Convert(_scale));
}
