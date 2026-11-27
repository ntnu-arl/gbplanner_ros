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
#include <math.h>

#include <cmath>
#include <sstream>
#include <gz/math/Color.hh>

#include "gz/common/Console.hh"
#include "gz/rendering/ogre/OgreDynamicLines.hh"

using namespace gz;
using namespace rendering;

enum {POSITION_BINDING, TEXCOORD_BINDING};


/// \brief Private implementation
class gz::rendering::OgreDynamicLinesPrivate
{
  /// \brief list of colors at each point
  public: std::vector<math::Color> colors;

  /// \brief List of points for the line
  public: std::vector<math::Vector3d> points;

  /// \brief Used to indicate if the lines require an update
  public: bool dirty = false;
};

/////////////////////////////////////////////////
OgreDynamicLines::OgreDynamicLines(MarkerType _opType)
  : dataPtr(new OgreDynamicLinesPrivate)
{
  this->Init(_opType, false);
  this->setCastShadows(false);
  this->dataPtr->dirty = true;
}

/////////////////////////////////////////////////
OgreDynamicLines::~OgreDynamicLines()
{
}

/////////////////////////////////////////////////
void OgreDynamicLines::AddPoint(const math::Vector3d &_pt,
                            const math::Color &_color)
{
  this->dataPtr->points.push_back(_pt);
  this->dataPtr->colors.push_back(_color);
  this->dataPtr->dirty = true;
}

/////////////////////////////////////////////////
void OgreDynamicLines::AddPoint(double _x, double _y,
                double _z, const math::Color &_color)
{
  this->AddPoint(math::Vector3d(_x, _y, _z), _color);
}

/////////////////////////////////////////////////
void OgreDynamicLines::SetPoint(unsigned int _index,
                            const math::Vector3d &_value)
{
  if (_index >= this->dataPtr->points.size())
  {
    ignerr << "Point index[" << _index << "] is out of bounds[0-"
           << this->dataPtr->points.size()-1 << "]\n";
    return;
  }

  this->dataPtr->points[_index] = _value;

  this->dataPtr->dirty = true;
}

/////////////////////////////////////////////////
void OgreDynamicLines::SetColor(unsigned int _index,
                            const math::Color &_color)
{
  this->dataPtr->colors[_index] = _color;
  this->dataPtr->dirty = true;
}

/////////////////////////////////////////////////
math::Vector3d OgreDynamicLines::Point(
    const unsigned int _index) const
{
  if (_index >= this->dataPtr->points.size())
  {
    ignerr << "Point index[" << _index << "] is out of bounds[0-"
           << this->dataPtr->points.size()-1 << "]\n";

    return math::Vector3d(math::INF_D,
                                    math::INF_D,
                                    math::INF_D);
  }

  return this->dataPtr->points[_index];
}

/////////////////////////////////////////////////
unsigned int OgreDynamicLines::PointCount() const
{
  return this->dataPtr->points.size();
}

/////////////////////////////////////////////////
void OgreDynamicLines::Clear()
{
  this->dataPtr->points.clear();
  this->dataPtr->dirty = true;
}

/////////////////////////////////////////////////
void OgreDynamicLines::Update()
{
  if (this->dataPtr->dirty && this->dataPtr->points.size() > 1)
    this->FillHardwareBuffers();
}

/////////////////////////////////////////////////
void OgreDynamicLines::CreateVertexDeclaration()
{
  Ogre::VertexDeclaration *decl =
    this->mRenderOp.vertexData->vertexDeclaration;

  decl->addElement(POSITION_BINDING, 0, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
  decl->addElement(1, 0, Ogre::VET_COLOUR, Ogre::VES_DIFFUSE);
}

/////////////////////////////////////////////////
void OgreDynamicLines::FillHardwareBuffers()
{
  int size = this->dataPtr->points.size();
  this->PrepareHardwareBuffers(size, 0);

  if (!size)
  {
    this->mBox.setExtents(Ogre::Vector3::ZERO, Ogre::Vector3::ZERO);
    this->dataPtr->dirty = false;
  }

  Ogre::HardwareVertexBufferSharedPtr vbuf =
    this->mRenderOp.vertexData->vertexBufferBinding->getBuffer(0);

  Ogre::Real *prPos =
    static_cast<Ogre::Real*>(vbuf->lock(Ogre::HardwareBuffer::HBL_NORMAL));
  {
    for (int i = 0; i < size; i++)
    {
      *prPos++ = this->dataPtr->points[i].X();
      *prPos++ = this->dataPtr->points[i].Y();
      *prPos++ = this->dataPtr->points[i].Z();

      this->mBox.merge(OgreConversions::Convert(this->dataPtr->points[i]));
    }
  }
  vbuf->unlock();

  // Update the colors
  Ogre::HardwareVertexBufferSharedPtr cbuf =
    this->mRenderOp.vertexData->vertexBufferBinding->getBuffer(1);

  Ogre::RGBA *colorArrayBuffer =
        static_cast<Ogre::RGBA *>(
        cbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));
  Ogre::RenderSystem *renderSystemForVertex =
        Ogre::Root::getSingleton().getRenderSystem();
  for (int i = 0; i < size; ++i)
  {
    Ogre::ColourValue color =
            OgreConversions::Convert(this->dataPtr->colors[i]);
    renderSystemForVertex->convertColourValue(color, &colorArrayBuffer[i]);
  }
  cbuf->unlock();

  // need to update after mBox change, otherwise the lines goes in and out
  // of scope based on old mBox
  this->getParentSceneNode()->needUpdate();

  this->dataPtr->dirty = false;
}
