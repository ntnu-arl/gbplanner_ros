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
#include "gz/rendering/ogre/OgreConversions.hh"

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
const Ogre::PixelFormat OgreConversions::ogrePixelFormats[PF_COUNT] =
    {
      // UKNOWN
      Ogre::PF_UNKNOWN,
      // L8
      Ogre::PF_L8,
      // R8G8B8
      Ogre::PF_BYTE_RGB,
      // B8G8R8
      Ogre::PF_BYTE_BGR,
      // BAYER_RGGB8
      Ogre::PF_BYTE_RGB,
      // BAYER_BGGR8
      Ogre::PF_BYTE_RGB,
      // BAYER_GBGR8
      Ogre::PF_BYTE_RGB,
      // BAYER_GRGB8
      Ogre::PF_BYTE_RGB,
      // PF_FLOAT32_R
      Ogre::PF_FLOAT32_R,
      // PF_FLOAT32_RGBA
      Ogre::PF_FLOAT32_RGBA,
      // PF_FLOAT32_RGB
      Ogre::PF_FLOAT32_RGB,
      // PF_L16
      Ogre::PF_L16
    };

//////////////////////////////////////////////////
Ogre::ColourValue OgreConversions::Convert(const math::Color &_color)
{
  return Ogre::ColourValue(_color.R(), _color.G(), _color.B(), _color.A());
}

//////////////////////////////////////////////////
math::Color OgreConversions::Convert(const Ogre::ColourValue &_color)
{
  return math::Color(_color.r, _color.g, _color.b, _color.a);
}

//////////////////////////////////////////////////
Ogre::Vector3 OgreConversions::Convert(const math::Vector3d &_vector)
{
  return Ogre::Vector3(_vector.X(), _vector.Y(), _vector.Z());
}

//////////////////////////////////////////////////
math::Vector3d OgreConversions::Convert(const Ogre::Vector3 &_vector)
{
  return math::Vector3d(_vector.x, _vector.y, _vector.z);
}

//////////////////////////////////////////////////
Ogre::Quaternion OgreConversions::Convert(const math::Quaterniond &_quat)
{
  return Ogre::Quaternion(_quat.W(), _quat.X(), _quat.Y(), _quat.Z());
}

//////////////////////////////////////////////////
math::Quaterniond OgreConversions::Convert(const Ogre::Quaternion &_quat)
{
  return math::Quaterniond(_quat.w, _quat.x, _quat.y, _quat.z);
}

//////////////////////////////////////////////////
Ogre::Radian OgreConversions::Convert(const math::Angle &_angle)
{
  return Ogre::Radian(_angle.Radian());
}

//////////////////////////////////////////////////
math::Angle OgreConversions::Convert(const Ogre::Radian &_angle)
{
  return math::Angle(_angle.valueRadians());
}

//////////////////////////////////////////////////
Ogre::PixelFormat OgreConversions::Convert(PixelFormat _format)
{
  return OgreConversions::ogrePixelFormats[_format];
}

//////////////////////////////////////////////////
math::Matrix4d OgreConversions::Convert(const Ogre::Matrix4 &_m)
{
  return math::Matrix4d(_m[0][0], _m[0][1], _m[0][2], _m[0][3],
                        _m[1][0], _m[1][1], _m[1][2], _m[1][3],
                        _m[2][0], _m[2][1], _m[2][2], _m[2][3],
                        _m[3][0], _m[3][1], _m[3][2], _m[3][3]);
}

//////////////////////////////////////////////////
Ogre::Matrix4 OgreConversions::Convert(const math::Matrix4d &_m)
{
  return Ogre::Matrix4(_m(0, 0), _m(0, 1), _m(0, 2), _m(0, 3),
                       _m(1, 0), _m(1, 1), _m(1, 2), _m(1, 3),
                       _m(2, 0), _m(2, 1), _m(2, 2), _m(2, 3),
                       _m(3, 0), _m(3, 1), _m(3, 2), _m(3, 3));
}

