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
#ifndef GZ_RENDERING_TEXT_HH_
#define GZ_RENDERING_TEXT_HH_

#include <string>

#include <gz/math/AxisAlignedBox.hh>
#include <gz/math/Color.hh>

#include "gz/rendering/config.hh"
#include "gz/rendering/Geometry.hh"
#include "gz/rendering/Object.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \enum TextHorizontalAlign
    /// \brief Text Horizontal alignment
    enum class TextHorizontalAlign
    {
      /// \brief Left alignment
      LEFT = 0,
      /// \brief Center alignment
      CENTER = 1,
      /// \brief Right alignment
      RIGHT = 2
    };

    /// \enum TextVerticalAlign
    /// \brief Text vertical alignment
    enum class TextVerticalAlign
    {
      /// \brief Align bottom
      BOTTOM = 0,
      /// \brief Align center
      CENTER = 1,
      /// \brief Align top
      TOP = 2
    };

    /// \class Text Text.hh ignition/rendering/Text
    /// \brief Represents a billboard text geometry that is always facing the
    /// camera.
    class IGNITION_RENDERING_VISIBLE Text :
      public virtual Geometry
    {
      /// \brief Constructor
      public: Text() = default;

      /// \brief Destructor
      public: virtual ~Text() = default;

      /// \brief Set the font.
      /// \param[in] _font Name of the font
      /// \sa FontName()
      public: virtual void SetFontName(const std::string &_font) = 0;

      /// \brief Get the font name.
      /// \return The font name.
      /// \sa SetFontName()
      public: virtual std::string FontName() const = 0;

      /// \brief Set the text to display.
      /// \param[in] _text The text to display.
      /// \sa Text()
      public: virtual void SetTextString(const std::string &_text) = 0;

      /// \brief Get the displayed text.
      /// \return The displayed text.
      /// \sa SetText()
      public: virtual std::string TextString() const = 0;

      /// \brief Set the text color.
      /// \param[in] _color Text color.
      /// \sa Color()
      public: virtual void SetColor(const gz::math::Color &_color) = 0;

      /// \brief Get the text color.
      /// \return Text color.
      /// \sa SetColor()
      public: virtual gz::math::Color Color() const = 0;

      /// \brief Set the height of the character in meters.
      /// \param[in] _height Height of the characters.
      /// \sa CharHeight()
      public: virtual void SetCharHeight(const float _height) = 0;

      /// \brief Get the height of the characters in meters
      /// return Height of the characters.
      /// \sa SetCharHeight()
      public: virtual float CharHeight() const = 0;

      /// \brief Set the width of spaces between words.
      /// \param[in] _width Space width
      /// \sa SpaceWidth()
      public: virtual void SetSpaceWidth(const float _width) = 0;

      /// \brief Get the width of spaces between words.
      /// \return Space width
      /// \sa SetSpaceWidth()
      public: virtual float SpaceWidth() const = 0;

      /// \brief Set the alignment of the text
      /// \param[in] _hAlign Horizontal alignment
      /// \param[in] _vAlign Vertical alignment
      public: virtual void SetTextAlignment(const TextHorizontalAlign &_hAlign,
                  const TextVerticalAlign &_vAlign) = 0;

      /// \brief Get the horizontal alignment of the text
      /// \return Text horizontal alignment
      public: virtual TextHorizontalAlign HorizontalAlignment() const = 0;

      /// \brief Get the vertical alignment of the text
      /// \return Text verical alignment
      public: virtual TextVerticalAlign VerticalAlignment() const = 0;

      /// \brief Set the baseline height of the text
      /// \param[in] _baseline Baseline height
      /// \sa Baseline()
      public: virtual void SetBaseline(const float _baseline) = 0;

      /// \brief Get the baseline height in meters.
      /// \return Baseline height
      /// \sa SetBaseline()
      public: virtual float Baseline() const = 0;

      /// \brief True = text always is displayed ontop.
      /// \param[in] _onTop Set to true to render the text on top of
      /// all other drawables.
      /// \sa ShowOnTop()
      public: virtual void SetShowOnTop(const bool _onTop) = 0;

      /// \brief Get whether the is displayed above other objects.
      /// \return True if it is on top.
      /// \sa SetShowOnTop()
      public: virtual bool ShowOnTop() const = 0;

      /// \brief Get the axis aligned bounding box of the text.
      /// \return The axis aligned bounding box.
      public: virtual gz::math::AxisAlignedBox AABB() const = 0;
    };
    }
  }
}

#endif
