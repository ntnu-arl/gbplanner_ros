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
#include "gz/rendering/optix/OptixNode.hh"
#include "gz/rendering/optix/OptixConversions.hh"
#include "gz/rendering/optix/OptixIncludes.hh"
#include "gz/rendering/optix/OptixScene.hh"
#include "gz/rendering/optix/OptixStorage.hh"

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
OptixNode::OptixNode() :
  poseDirty(true)
{
  this->pose = math::Pose3d::Zero;
}

//////////////////////////////////////////////////
OptixNode::~OptixNode()
{
}

//////////////////////////////////////////////////
bool OptixNode::HasParent() const
{
  return this->parent != nullptr;
}

//////////////////////////////////////////////////
NodePtr OptixNode::Parent() const
{
  return this->parent;
}

//////////////////////////////////////////////////
void OptixNode::PreRender()
{
  BaseNode::PreRender();
  this->WritePoseToDevice();
  this->optixAccel->markDirty();
}

//////////////////////////////////////////////////
optix::Transform OptixNode::OptixTransform() const
{
  return this->optixTransform;
}

//////////////////////////////////////////////////
math::Pose3d OptixNode::RawLocalPose() const
{
  return this->pose;
}

//////////////////////////////////////////////////
void OptixNode::SetRawLocalPose(const math::Pose3d &_pose)
{
  this->pose = _pose;
  this->poseDirty = true;
}

//////////////////////////////////////////////////
void OptixNode::WritePoseToDevice()
{
  if (this->poseDirty)
  {
    this->WritePoseToDeviceImpl();
    this->poseDirty = false;
  }
}

//////////////////////////////////////////////////
void OptixNode::WritePoseToDeviceImpl()
{
  float matrix[16];
  math::Vector3d position = this->pose.Pos();
  math::Matrix3d rotation(this->pose.Rot());

  // assign row 1
  matrix[ 0] = rotation(0, 0);
  matrix[ 1] = rotation(0, 1);
  matrix[ 2] = rotation(0, 2);
  matrix[ 3] = position.X();

  // assign row 2
  matrix[ 4] = rotation(1, 0);
  matrix[ 5] = rotation(1, 1);
  matrix[ 6] = rotation(1, 2);
  matrix[ 7] = position.Y();

  // assign row 3
  matrix[ 8] = rotation(2, 0);
  matrix[ 9] = rotation(2, 1);
  matrix[10] = rotation(2, 2);
  matrix[11] = position.Z();

  // assign row 4
  matrix[12] = 0;
  matrix[13] = 0;
  matrix[14] = 0;
  matrix[15] = 1;

  this->optixTransform->setMatrix(0, matrix, 0);
}

//////////////////////////////////////////////////
void OptixNode::SetParent(OptixNodePtr _parent)
{
  this->parent = _parent;
}

//////////////////////////////////////////////////
void OptixNode::Init()
{
  optix::Context optixContext = this->scene->OptixContext();
  this->optixTransform = optixContext->createTransform();
  // this->optixAccel = optixContext->createAcceleration("MedianBvh", "Bvh");
  // this->optixAccel = optixContext->createAcceleration("Lbvh", "Bvh");
  this->optixAccel = optixContext->createAcceleration("Sbvh", "Bvh");
  this->optixGroup = optixContext->createGroup();
  this->optixGroup->setAcceleration(this->optixAccel);
  this->optixTransform->setChild(this->optixGroup);
  this->children = OptixNodeStorePtr(new OptixNodeStore);
}

//////////////////////////////////////////////////
NodeStorePtr OptixNode::Children() const
{
  return this->children;
}

//////////////////////////////////////////////////
bool OptixNode::AttachChild(NodePtr _child)
{
  OptixNodePtr derived = std::dynamic_pointer_cast<OptixNode>(_child);

  if (!derived)
  {
    ignerr << "Cannot attach node created by another render-engine"
        << std::endl;
    return false;
  }

  derived->SetParent(this->SharedThis());
  optix::Transform childTransform = derived->OptixTransform();
  this->optixGroup->addChild(childTransform);
  this->optixAccel->markDirty();
  return true;
}

//////////////////////////////////////////////////
bool OptixNode::DetachChild(NodePtr _child)
{
  OptixNodePtr derived = std::dynamic_pointer_cast<OptixNode>(_child);

  if (!derived)
  {
    return false;
  }

  this->optixGroup->removeChild(derived->OptixTransform());
  this->optixAccel->markDirty();
  return true;
}

//////////////////////////////////////////////////
OptixNodePtr OptixNode::SharedThis()
{
  return std::dynamic_pointer_cast<OptixNode>(shared_from_this());
}

//////////////////////////////////////////////////
math::Vector3d OptixNode::LocalScale() const
{
  return this->scale;
}

//////////////////////////////////////////////////
bool OptixNode::InheritScale() const
{
  return this->inheritScale;
}

//////////////////////////////////////////////////
void OptixNode::SetInheritScale(bool _inherit)
{
  this->inheritScale = _inherit;
}

//////////////////////////////////////////////////
void OptixNode::SetLocalScaleImpl(const math::Vector3d &_scale)
{
  this->scale = _scale;
}
