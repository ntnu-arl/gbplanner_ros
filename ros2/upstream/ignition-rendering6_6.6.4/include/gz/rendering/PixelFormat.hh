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
#ifndef GZ_RENDERING_PIXELFORMAT_HH_
#define GZ_RENDERING_PIXELFORMAT_HH_

#include <string>
#include "gz/rendering/config.hh"
#include "gz/rendering/Export.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \enum PixelFormat PixelFormat.hh ignition/rendering/PixelFormat.hh
    /// \brief Image pixel format types
    enum IGNITION_RENDERING_VISIBLE PixelFormat
    {
      /// < Unknown or errant type
      PF_UNKNOWN      = 0,
      /// < Grayscale, 1-byte per channel
      PF_L8           = 1,
      /// < RGB, 1-byte per channel
      PF_R8G8B8       = 2,
      /// < BGR, 1-byte per channel
      PF_B8G8R8       = 3,
      /// < Bayer RGGB, 1-byte per channel
      PF_BAYER_RGGB8  = 4,
      /// < Bayer BGGR, 1-byte per channel
      PF_BAYER_BGGR8  = 5,
      /// < Bayer GBGR, 1-byte per channel
      PF_BAYER_GBGR8  = 6,
      /// < Bayer GRGB, 1-byte per channel
      PF_BAYER_GRGB8  = 7,
      // Float32 format one channel
      PF_FLOAT32_R    = 8,
      // Float32 format and RGB
      PF_FLOAT32_RGBA = 9,
      // Float32 format and RGB
      PF_FLOAT32_RGB  = 10,
      // 16 bit single channel
      PF_L16          = 11,
      /// < RGBA, 1-byte per channel
      PF_R8G8B8A8     = 12,
      /// < Number of pixel format types
      PF_COUNT        = 13
    };

    /// \class PixelUtil PixelFormat.hh ignition/rendering/PixelFormat.hh
    /// \brief Provides supporting functions for PixelFormat enum
    class IGNITION_RENDERING_VISIBLE PixelUtil
    {
      /// \brief Determine if given format is valid PixelFormat enum
      /// \param[in] _format Enum value to be evaluated
      public: static bool IsValid(PixelFormat _format);

      /// \brief Sanitize given format. If the given value is invalid,
      /// PF_UNKNOWN will be returned, otherwise input will be returned
      /// unchanged.
      /// \param[in] _format Image pixel format to be sanitized
      /// \return The sanitized pixel format
      public: static PixelFormat Sanitize(PixelFormat _format);

      /// \brief Get human-readable name for pixel format value.
      /// \param[in] _format Image pixel format
      /// \return The format name
      public: static std::string Name(PixelFormat _format);

      /// \brief Get number of channels for given format. If an invalid format
      /// is given, 0 will be returned.
      /// \param[in] _format Image pixel format
      /// \return The channel count
      public: static unsigned int ChannelCount(PixelFormat _format);

      /// \brief Get number of bytes per channel for given format. If an invalid
      /// format is given, 0 will be returned.
      /// \param[in] _format Image pixel format
      /// \return The number of bytes per channel
      public: static unsigned int BytesPerChannel(PixelFormat _format);

      /// \brief Get number of bytes per pixel for given format. If an invalid
      /// format is given, 0 will be returned. This is simply the product of
      /// GetChannelCount and GetBytesPerChannel.
      /// \return The number of bytes per pixel
      public: static unsigned int BytesPerPixel(PixelFormat _format);

      /// \brief Get total memory size in bytes for an image with the given
      /// format and dimensions. If an invalid format is given, 0 will be
      /// returned. This is simply the product of GetBytesPerPixel, _width,
      /// and, _height.
      /// \param[in] _format Image pixel format
      /// \param[in] _width Image width in pixels
      /// \param[in] _height Image height in pixels
      /// \return The number of bytes per pixel
      public: static unsigned int MemorySize(PixelFormat _format,
                  unsigned int _width, unsigned int _height);

      /// \brief Get enum value by human-readable name. The given string should
      /// match watch is returned by GetName. If an invalid name is given,
      /// PF_UNKNOWN will be returned.
      /// \param[in] _name Name of the pixel format to be retrieved
      /// \return The specified PixelFormat enum value
      public: static PixelFormat Enum(const std::string &_name);

      /// \brief Array of human-readable names for each PixelFormat
      private: static const char *names[PF_COUNT];

      /// \brief Array of channel counts for each PixelFormat
      private: static const unsigned char channelCounts[PF_COUNT];

      /// \brief Array of bytes per channel for each PixelFormat
      private: static const unsigned char channelByteCounts[PF_COUNT];
    };
    }
  }
}
#endif
