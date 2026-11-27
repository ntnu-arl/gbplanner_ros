/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "gz/common/Console.hh"
#include "gz/rendering/ogre/OgreDynamicRenderable.hh"

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
OgreDynamicRenderable::OgreDynamicRenderable()
{
}

//////////////////////////////////////////////////
OgreDynamicRenderable::~OgreDynamicRenderable()
{
  if (!Ogre::HardwareBufferManager::getSingletonPtr())
    return;

  if (this->mRenderOp.vertexData)
  {
    delete this->mRenderOp.vertexData;
    this->mRenderOp.vertexData = nullptr;
  }

  if (this->mRenderOp.indexData)
  {
    delete this->mRenderOp.indexData;
    this->mRenderOp.indexData = nullptr;
  }
}

//////////////////////////////////////////////////
void OgreDynamicRenderable::Init(MarkerType _opType, bool useIndices)
{
  this->SetOperationType(_opType);

  // Initialize render operation
  this->mRenderOp.useIndexes = useIndices;
  this->mRenderOp.vertexData = new Ogre::VertexData;

  if (this->mRenderOp.useIndexes)
    this->mRenderOp.indexData = new Ogre::IndexData;

  // Reset buffer capacities
  this->vertexBufferCapacity = 0;
  this->indexBufferCapacity = 0;

  // Create vertex declaration
  this->CreateVertexDeclaration();
}

//////////////////////////////////////////////////
void OgreDynamicRenderable::SetOperationType(MarkerType _opType)
{
  switch (_opType)
  {
    case MT_POINTS:
      this->mRenderOp.operationType = Ogre::RenderOperation::OT_POINT_LIST;
      break;

    case MT_LINE_LIST:
      this->mRenderOp.operationType = Ogre::RenderOperation::OT_LINE_LIST;
      break;

    case MT_LINE_STRIP:
      this->mRenderOp.operationType = Ogre::RenderOperation::OT_LINE_STRIP;
      break;

    case MT_TRIANGLE_LIST:
      this->mRenderOp.operationType = Ogre::RenderOperation::OT_TRIANGLE_LIST;
      break;

    case MT_TRIANGLE_STRIP:
      this->mRenderOp.operationType = Ogre::RenderOperation::OT_TRIANGLE_STRIP;
      break;

    case MT_TRIANGLE_FAN:
      this->mRenderOp.operationType = Ogre::RenderOperation::OT_TRIANGLE_FAN;
      break;

    default:
      ignerr << "Unknown render operation type[" << _opType << "]\n";
      break;
  }
}

//////////////////////////////////////////////////
MarkerType OgreDynamicRenderable::OperationType() const
{
  MarkerType opType;
  switch (this->mRenderOp.operationType)
  {
    case Ogre::RenderOperation::OT_LINE_LIST:
      opType = MT_LINE_LIST;
      break;

    case Ogre::RenderOperation::OT_LINE_STRIP:
      opType = MT_LINE_STRIP;
      break;

    case Ogre::RenderOperation::OT_TRIANGLE_LIST:
      opType = MT_TRIANGLE_LIST;
      break;

    case Ogre::RenderOperation::OT_TRIANGLE_STRIP:
      opType = MT_TRIANGLE_STRIP;
      break;

    case Ogre::RenderOperation::OT_TRIANGLE_FAN:
      opType = MT_TRIANGLE_FAN;
      break;

    default:
    case Ogre::RenderOperation::OT_POINT_LIST:
      opType = MT_POINTS;
      break;
  }

  return opType;
}

//////////////////////////////////////////////////
void OgreDynamicRenderable::PrepareHardwareBuffers(size_t vertexCount,
                                               size_t indexCount)
{
  // Prepare vertex buffer
  size_t newVertCapacity = this->vertexBufferCapacity;

  if ((vertexCount > this->vertexBufferCapacity) ||
      (!this->vertexBufferCapacity))
  {
    // vertexCount exceeds current capacity!
    // It is necessary to reallocate the buffer.

    // Check if this is the first call
    if (!newVertCapacity)
      newVertCapacity = 1;

    // Make capacity the next power of two
    while (newVertCapacity < vertexCount)
      newVertCapacity <<= 1;
  }
  else if (vertexCount < this->vertexBufferCapacity>>1)
  {
    // Make capacity the previous power of two
    while (vertexCount < newVertCapacity>>1)
      newVertCapacity >>= 1;
  }

  if (newVertCapacity != this->vertexBufferCapacity)
  {
    this->vertexBufferCapacity = newVertCapacity;

    // Create new vertex buffer
    Ogre::HardwareVertexBufferSharedPtr vbuf =
      Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
        this->mRenderOp.vertexData->vertexDeclaration->getVertexSize(0),
        this->vertexBufferCapacity,
        Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY);

    Ogre::HardwareVertexBufferSharedPtr cbuf =
      Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
        Ogre::VertexElement::getTypeSize(Ogre::VET_COLOUR),
        this->vertexBufferCapacity,
        Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY);

    // TODO(anyone): Custom HBU_?

    // Bind buffer
    this->mRenderOp.vertexData->vertexBufferBinding->setBinding(0, vbuf);

    this->mRenderOp.vertexData->vertexBufferBinding->setBinding(1, cbuf);
  }

  // Update vertex count in the render operation
  this->mRenderOp.vertexData->vertexCount = vertexCount;

  if (this->mRenderOp.useIndexes)
  {
    OgreAssert(indexCount <= std::numeric_limits<uint16_t>::max(),
        "indexCount exceeds 16 bit");

    size_t newIndexCapacity = this->indexBufferCapacity;

    // Prepare index buffer
    if ((indexCount > newIndexCapacity) || (!newIndexCapacity))
    {
      // indexCount exceeds current capacity!
      // It is necessary to reallocate the buffer.

      // Check if this is the first call
      if (!newIndexCapacity)
        newIndexCapacity = 1;

      // Make capacity the next power of two
      while (newIndexCapacity < indexCount)
        newIndexCapacity <<= 1;
    }
    else if (indexCount < newIndexCapacity>>1)
    {
      // Make capacity the previous power of two
      while (indexCount < newIndexCapacity>>1)
        newIndexCapacity >>= 1;
    }

    if (newIndexCapacity != this->indexBufferCapacity)
    {
      this->indexBufferCapacity = newIndexCapacity;

      // Create new index buffer
      this->mRenderOp.indexData->indexBuffer =
        Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
          Ogre::HardwareIndexBuffer::IT_16BIT,
          this->indexBufferCapacity,
          Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY);
      // TODO(anyone): Custom HBU_?
    }

    // Update index count in the render operation
    this->mRenderOp.indexData->indexCount = indexCount;
  }
}

//////////////////////////////////////////////////
Ogre::Real OgreDynamicRenderable::getBoundingRadius() const
{
  return Ogre::Math::Sqrt(std::max(mBox.getMaximum().squaredLength(),
                                   mBox.getMinimum().squaredLength()));
}

//////////////////////////////////////////////////
Ogre::Real OgreDynamicRenderable::getSquaredViewDepth(
          const Ogre::Camera *cam) const
{
  Ogre::Vector3 vMin, vMax, vMid, vDist;
  vMin = mBox.getMinimum();
  vMax = mBox.getMaximum();
  vMid = ((vMax - vMin) * 0.5) + vMin;
  vDist = cam->getDerivedPosition() - vMid;
  return vDist.squaredLength();
}
