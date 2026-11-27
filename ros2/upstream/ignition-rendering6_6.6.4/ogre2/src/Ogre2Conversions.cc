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
#include "gz/rendering/ogre2/Ogre2Conversions.hh"

#ifdef _MSC_VER
  #pragma warning(push, 0)
#endif
#include <OgreMatrix4.h>
#ifdef _MSC_VER
  #pragma warning(pop)
#endif

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
const Ogre::PixelFormatGpu Ogre2Conversions::ogrePixelFormats[PF_COUNT] =
    {
      // UKNOWN
      Ogre::PFG_UNKNOWN,
      // L8
      Ogre::PFG_R8_UNORM,
      // R8G8B8
      Ogre::PFG_RGB8_UNORM,
      // B8G8R8
      Ogre::PFG_BGR8_UNORM,
      // BAYER_RGGB8
      Ogre::PFG_RGB8_UNORM,
      // BAYER_BGGR8
      Ogre::PFG_RGB8_UNORM,
      // BAYER_GBGR8
      Ogre::PFG_RGB8_UNORM,
      // BAYER_GRGB8
      Ogre::PFG_RGB8_UNORM,
      // PF_FLOAT32_R
      Ogre::PFG_R32_FLOAT,
      // PF_FLOAT32_RGBA
      Ogre::PFG_RGBA32_FLOAT,
      // PF_FLOAT32_RGB
      Ogre::PFG_RGB32_FLOAT,
      // PF_L16
      Ogre::PFG_R16_UNORM,
      // PF_R8G8B8A8
      Ogre::PFG_RGBA8_UNORM,
    };

//////////////////////////////////////////////////
Ogre::ColourValue Ogre2Conversions::Convert(const math::Color &_color)
{
  return Ogre::ColourValue(_color.R(), _color.G(), _color.B(), _color.A());
}

//////////////////////////////////////////////////
math::Color Ogre2Conversions::Convert(const Ogre::ColourValue &_color)
{
  return math::Color(_color.r, _color.g, _color.b, _color.a);
}

//////////////////////////////////////////////////
Ogre::Vector3 Ogre2Conversions::Convert(const math::Vector3d &_vector)
{
  return Ogre::Vector3(_vector.X(), _vector.Y(), _vector.Z());
}

//////////////////////////////////////////////////
math::Vector3d Ogre2Conversions::Convert(const Ogre::Vector3 &_vector)
{
  return math::Vector3d(_vector.x, _vector.y, _vector.z);
}

//////////////////////////////////////////////////
Ogre::Quaternion Ogre2Conversions::Convert(const math::Quaterniond &_quat)
{
  return Ogre::Quaternion(_quat.W(), _quat.X(), _quat.Y(), _quat.Z());
}

//////////////////////////////////////////////////
math::Quaterniond Ogre2Conversions::Convert(const Ogre::Quaternion &_quat)
{
  return math::Quaterniond(_quat.w, _quat.x, _quat.y, _quat.z);
}

//////////////////////////////////////////////////
Ogre::Radian Ogre2Conversions::Convert(const math::Angle &_angle)
{
  return Ogre::Radian(_angle.Radian());
}

//////////////////////////////////////////////////
math::Angle Ogre2Conversions::Convert(const Ogre::Radian &_angle)
{
  return math::Angle(_angle.valueRadians());
}

//////////////////////////////////////////////////
Ogre::PixelFormatGpu Ogre2Conversions::Convert(PixelFormat _format)
{
  return Ogre2Conversions::ogrePixelFormats[_format];
}

//////////////////////////////////////////////////
math::Matrix4d Ogre2Conversions::Convert(const Ogre::Matrix4 &_m)
{
  return math::Matrix4d(_m[0][0], _m[0][1], _m[0][2], _m[0][3],
                        _m[1][0], _m[1][1], _m[1][2], _m[1][3],
                        _m[2][0], _m[2][1], _m[2][2], _m[2][3],
                        _m[3][0], _m[3][1], _m[3][2], _m[3][3]);
}

//////////////////////////////////////////////////
Ogre::Matrix4 Ogre2Conversions::Convert(const math::Matrix4d &_m)
{
  return Ogre::Matrix4(_m(0, 0), _m(0, 1), _m(0, 2), _m(0, 3),
                       _m(1, 0), _m(1, 1), _m(1, 2), _m(1, 3),
                       _m(2, 0), _m(2, 1), _m(2, 2), _m(2, 3),
                       _m(3, 0), _m(3, 1), _m(3, 2), _m(3, 3));
}

