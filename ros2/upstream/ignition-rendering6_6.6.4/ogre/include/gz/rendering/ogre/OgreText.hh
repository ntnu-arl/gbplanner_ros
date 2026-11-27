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

#ifndef GZ_RENDERING_OGRE_OGRETEXT_HH_
#define GZ_RENDERING_OGRE_OGRETEXT_HH_

#include <memory>
#include <string>

#include <gz/math/AxisAlignedBox.hh>
#include <gz/math/Color.hh>

#include "gz/rendering/base/BaseText.hh"
#include "gz/rendering/ogre/OgreGeometry.hh"
#include "gz/rendering/ogre/OgreIncludes.hh"
#include "gz/rendering/ogre/Export.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    class OgreMovableText;
    class OgreTextPrivate;

    /// \brief Ogre implementation of text geometry
    class IGNITION_RENDERING_OGRE_VISIBLE OgreText
        : public BaseText<OgreGeometry>
    {
      /// \brief Constructor
      protected: OgreText();

      /// \brief Destructor
      public: virtual ~OgreText();

      // Documentation inherited
      public: virtual void Init() override;

      // Documentation inherited
      public: virtual void PreRender() override;

      // Documentation inherited
      public: virtual Ogre::MovableObject *OgreObject() const override;

      // Documentation inherited.
      public: virtual MaterialPtr Material() const override;

      // Documentation inherited.
      public: virtual void SetMaterial(MaterialPtr _material, bool _unique)
          override;

      // Documentation inherited.
      public: virtual void SetFontName(const std::string &_font) override;

      // Documentation inherited.
      public: virtual void SetTextString(const std::string &_text) override;

      // Documentation inherited.
      public: virtual void SetColor(const gz::math::Color &_color)
          override;

      // Documentation inherited.
      public: virtual void SetCharHeight(const float _height) override;

      // Documentation inherited.
      public: virtual void SetSpaceWidth(const float _width) override;

      // Documentation inherited.
      public: virtual void SetTextAlignment(
                  const TextHorizontalAlign &_horizAlign,
                  const TextVerticalAlign &_vertAlign) override;
      // Documentation inherited.
      public: virtual void SetBaseline(const float _baseline) override;

      // Documentation inherited.
      public: virtual void SetShowOnTop(const bool _onTop) override;

      // Documentation inherited.
      public: virtual gz::math::AxisAlignedBox AABB() const override;

      /// \brief Set material to text geometry.
      /// \param[in] _material Ogre material.
      protected: virtual void SetMaterialImpl(OgreMaterialPtr _material);

      /// \brief Text should only be created by scene.
      private: friend class OgreScene;

      /// \internal
      /// \brief Private data pointer
      private: std::unique_ptr<OgreTextPrivate> dataPtr;
    };
    }
  }
}
#endif
