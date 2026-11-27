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
#ifndef GZ_RENDERING_BASE_BASETEXT_HH_
#define GZ_RENDERING_BASE_BASETEXT_HH_

#include <string>
#include "gz/rendering/Text.hh"
#include "gz/rendering/base/BaseObject.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \brief Base implementation of a text geometry
    template <class T>
    class BaseText :
      public virtual Text,
      public virtual T
    {
      /// \brief Constructor
      protected: BaseText();

      /// \brief Destructor
      public: virtual ~BaseText();

      // Documentation inherited.
      public: virtual void PreRender() override;

      // Documentation inherited.
      public: virtual void Destroy() override;

      // Documentation inherited.
      public: void SetFontName(const std::string &_font) override;

      // Documentation inherited.
      public: std::string FontName() const override;

      // Documentation inherited.
      public: virtual void SetTextString(const std::string &_text) override;

      // Documentation inherited.
      public: virtual std::string TextString() const override;

      // Documentation inherited.
      public: virtual void SetColor(const gz::math::Color &_color)
          override;

      // Documentation inherited.
      public: virtual gz::math::Color Color() const override;

      // Documentation inherited.
      public: virtual void SetCharHeight(const float _height) override;

      // Documentation inherited.
      public: virtual float CharHeight() const override;

      // Documentation inherited.
      public: virtual void SetSpaceWidth(const float _width) override;

      // Documentation inherited.
      public: virtual float SpaceWidth() const override;

      // Documentation inherited.
      public: virtual void SetTextAlignment(const TextHorizontalAlign &_hAlign,
                  const TextVerticalAlign &_vAlign) override;
      // Documentation inherited.
      public: virtual TextHorizontalAlign HorizontalAlignment() const override;

      // Documentation inherited.
      public: virtual TextVerticalAlign VerticalAlignment() const override;

      // Documentation inherited.
      public: virtual void SetBaseline(const float _baseline) override;

      // Documentation inherited.
      public: virtual float Baseline() const override;

      // Documentation inherited.
      public: void SetShowOnTop(const bool _onTop) override;

      // Documentation inherited.
      public: virtual bool ShowOnTop() const override;

      // Documentation inherited.
      public: virtual gz::math::AxisAlignedBox AABB() const override;

      /// \brief Font name, such as "Liberation Sans"
      protected: std::string fontName = "Liberation Sans";

      /// \brief Text being displayed
      protected: std::string text;

      /// \brief Text color
      protected: gz::math::Color color = gz::math::Color::White;

      /// \brief Character height in meters
      protected: float charHeight = 1.0;

      /// \brief Width of space between letters
      protected: float spaceWidth = 0;

      /// \brief Horizontal alignment
      protected: TextHorizontalAlign horizontalAlign =
                 TextHorizontalAlign::LEFT;

      /// \brief Vertical alignment
      protected: TextVerticalAlign verticalAlign = TextVerticalAlign::BOTTOM;

      /// \brief Baseline height in meters.
      protected: float baseline = 0;

      /// \brief True for text to be displayed on top of other objects in the
      /// scene.
      protected: bool onTop = false;

      /// \brief Flag to indicate text properties have changed
      protected: bool textDirty = false;
    };

    //////////////////////////////////////////////////
    // BaseText
    //////////////////////////////////////////////////
    template <class T>
    BaseText<T>::BaseText()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    BaseText<T>::~BaseText()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    std::string BaseText<T>::FontName() const
    {
      return this->fontName;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseText<T>::SetFontName(const std::string &_font)
    {
      this->fontName = _font;
      this->textDirty = true;
    }

    //////////////////////////////////////////////////
    template <class T>
    std::string BaseText<T>::TextString() const
    {
      return this->text;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseText<T>::SetTextString(const std::string &_text)
    {
      this->text = _text;
      this->textDirty = true;
    }

    //////////////////////////////////////////////////
    template <class T>
    gz::math::Color BaseText<T>::Color() const
    {
      return this->color;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseText<T>::SetColor(const gz::math::Color &_color)
    {
      this->color = _color;
      this->textDirty = true;
    }

    //////////////////////////////////////////////////
    template <class T>
    float BaseText<T>::CharHeight() const
    {
      return this->charHeight;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseText<T>::SetCharHeight(const float _height)
    {
      this->charHeight = _height;
      this->textDirty = true;
    }

    //////////////////////////////////////////////////
    template <class T>
    float BaseText<T>::SpaceWidth() const
    {
      return this->spaceWidth;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseText<T>::SetSpaceWidth(const float _width)
    {
      this->spaceWidth = _width;
      this->textDirty = true;
    }

    //////////////////////////////////////////////////
    template <class T>
    TextHorizontalAlign BaseText<T>::HorizontalAlignment() const
    {
      return this->horizontalAlign;
    }

    //////////////////////////////////////////////////
    template <class T>
    TextVerticalAlign BaseText<T>::VerticalAlignment() const
    {
      return this->verticalAlign;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseText<T>::SetTextAlignment(const TextHorizontalAlign &_horzAlign,
                                       const TextVerticalAlign &_vertAlign)
    {
      this->horizontalAlign = _horzAlign;
      this->verticalAlign = _vertAlign;
      this->textDirty = true;
    }

    //////////////////////////////////////////////////
    template <class T>
    float BaseText<T>::Baseline() const
    {
      return this->baseline;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseText<T>::SetBaseline(const float _baseline)
    {
      this->baseline = _baseline;
      this->textDirty = true;
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseText<T>::ShowOnTop() const
    {
      return this->onTop;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseText<T>::SetShowOnTop(const bool _onTop)
    {
      this->onTop = _onTop;
      this->textDirty = true;
    }

    //////////////////////////////////////////////////
    template <class T>
    gz::math::AxisAlignedBox BaseText<T>::AABB() const
    {
      math::AxisAlignedBox box;
      return box;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseText<T>::PreRender()
    {
      T::PreRender();
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseText<T>::Destroy()
    {
      T::Destroy();
    }
    }
  }
}
#endif
