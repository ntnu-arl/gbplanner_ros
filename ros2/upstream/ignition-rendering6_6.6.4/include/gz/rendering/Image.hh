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
#ifndef GZ_RENDERING_IMAGE_HH_
#define GZ_RENDERING_IMAGE_HH_

#include <memory>

#include <gz/common/SuppressWarning.hh>

#include "gz/rendering/config.hh"
#include "gz/rendering/PixelFormat.hh"
#include "gz/rendering/Export.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \class Image Image.hh ignition/rendering/Image.hh
    /// \brief Encapsulates a raw image buffer and relevant properties
    class IGNITION_RENDERING_VISIBLE Image
    {
      /// \brief Shared pointer to raw image buffer
      typedef std::shared_ptr<unsigned char> DataPtr;

      /// \brief Default constructor
      public: Image() = default;

      /// \brief Constructor
      /// \param[in] _width Image width in pixels
      /// \param[in] _height Image height in pixels
      /// \param[in] _format Image pixel format
      public: Image(unsigned int _width, unsigned int _height,
                  PixelFormat _format);

      /// \brief Deconstructor
      public: ~Image();

      /// \brief Get image width in pixels
      /// \return The image width in pixels
      public: unsigned int Width() const;

      /// \brief Get image height in pixels
      /// \return The image height in pixels
      public: unsigned int Height() const;

      /// \brief Get image pixel format
      /// \return The image pixel format
      public: PixelFormat Format() const;

      /// \brief Get image channel depth
      /// \return The image channel depth
      public: unsigned int Depth() const;

      /// \brief Get image channel depth
      /// \return The image channel depth
      public: unsigned int MemorySize() const;

      /// \brief Get a const pointer to image data
      /// \return The const pointer to image data
      public: const void *Data() const;

      /// \brief Get a pointer to image data
      /// \return The pointer to image data
      public: void *Data();

      /// \brief Get a const pointer to image data. The returned image buffer
      /// will be statically cast to the given template type T.
      /// \return The const pointer to image data
      public: template <typename T>
              const T *Data() const;

      /// \brief Get a pointer to image data. The returned image buffer will be
      /// statically cast to the given template type T.
      /// \return The pointer to image data
      public: template <typename T>
              T *Data();

      /// \brief Image width in pixels
      private: unsigned int width = 0;

      /// \brief Image height in pixels
      private: unsigned int height = 0;

      /// \brief Image pixel format
      private: PixelFormat format = PF_UNKNOWN;

      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      /// \brief Pointer to the image data
      private: DataPtr data = nullptr;
      IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING
    };

    //////////////////////////////////////////////////
    template <typename T>
    const T *Image::Data() const
    {
      return static_cast<const T *>(this->data.get());
    }

    //////////////////////////////////////////////////
    template <typename T>
    T *Image::Data()
    {
      return static_cast<T *>(this->data.get());
    }
    }
  }
}
#endif
