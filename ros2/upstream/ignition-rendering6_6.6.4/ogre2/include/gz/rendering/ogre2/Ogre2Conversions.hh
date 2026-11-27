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
#ifndef GZ_RENDERING_OGRE2_OGRE2CONVERSIONS_HH_
#define GZ_RENDERING_OGRE2_OGRE2CONVERSIONS_HH_

#include <gz/math/Color.hh>
#include <gz/math/Matrix4.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

#include "gz/rendering/config.hh"
#include "gz/rendering/PixelFormat.hh"
#include "gz/rendering/ogre2/Export.hh"

#ifdef _MSC_VER
  #pragma warning(push, 0)
#endif

#include <OgreColourValue.h>
#include <OgreVector3.h>
#include <OgrePixelFormatGpu.h>

#ifdef _MSC_VER
  #pragma warning(pop)
#endif

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \addtogroup ign_rendering
    /// \{

    /// \brief Conversions Conversions.hh rendering/Conversions.hh
    /// \brief A set of utility function to convert between Ignition and Ogre
    /// data types
    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2Conversions
    {
      /// \brief Return the equivalent ogre color
      /// \param[in] _color ign-math color to convert
      /// \return Ogre color value
      public: static Ogre::ColourValue Convert(
          const math::Color &_color);

      /// \brief Return the equivalent ign-math color
      /// \param[in] _color Ogre color to convert
      /// \return ign-math color value
      public: static math::Color Convert(
          const Ogre::ColourValue &_color);

      /// \brief return Ogre Vector from ign-math Vector3
      /// \param[in] _vector ign-math vector
      /// \return Ogre vector
      public: static Ogre::Vector3 Convert(const math::Vector3d &_vector);

      /// \brief return ign-math Vector from ogre Vector3
      /// \param[in] _vector Ogre vector
      /// \return ign-math vector
      public: static math::Vector3d Convert(const Ogre::Vector3 &_vector);

      /// \brief Ign-math quaternion to Ogre quaternion
      /// \param[in] _quat ign-math quaternion
      /// \return Ogre quaternion
      public: static Ogre::Quaternion Convert(const math::Quaterniond &_quat);

      /// \brief Ogre quaternion to ign-math quaternion
      /// \param[in] _quat Ogre quaternion
      /// return ign-math quaternion
      public: static math::Quaterniond Convert(const Ogre::Quaternion &_quat);

      /// \brief Ign-math angle to Ogre angle
      /// \param[in] _angle ign-math angle
      /// \return Ogre angle
      public: static Ogre::Radian Convert(const math::Angle &_angle);

      /// \brief Ogre angle to ign-math angle
      /// \param[in] _angle Ogre angle
      /// return Ign-math angle
      public: static math::Angle Convert(const Ogre::Radian &_angle);

      /// \brief Ogre Matrix4 to ignition math Matrix4d
      /// \param[in] _m Ogre Matrix4
      /// \return ignition math Matrix4d
      public: static math::Matrix4d Convert(const Ogre::Matrix4 &_m);

      /// \brief Ignition math Matrix4d to Ogre Matrix4
      /// \param[in] _m ignition math Matrix4d
      /// \return Ogre Matrix4
      public: static Ogre::Matrix4 Convert(const math::Matrix4d &_m);

      /// \brief Ign-rendering PixelFormat to Ogre PixelFormat
      /// \param[in] _format Ign-rendering PixelFormat
      /// return Ogre PixelFormat
      public: static Ogre::PixelFormatGpu Convert(PixelFormat _format);

      /// \brief A list of ogre pixel formats
      private: static const Ogre::PixelFormatGpu ogrePixelFormats[PF_COUNT];
    };
    /// \}
    }
  }
}
#endif
