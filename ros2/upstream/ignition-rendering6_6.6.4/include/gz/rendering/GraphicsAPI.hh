/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#ifndef IGNITION_RENDERING_GRAPHICSAPI_HH_
#define IGNITION_RENDERING_GRAPHICSAPI_HH_

#include <cstdint>
#include <string>
#include "ignition/rendering/config.hh"
#include "ignition/rendering/Export.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {

     /// \brief The graphics API used by the render engine
    enum class IGNITION_RENDERING_VISIBLE GraphicsAPI : uint16_t
    {
      /// \internal
      /// \brief Indicator used to create an iterator over the
      /// enum. Do not use this.
      GRAPHICS_API_BEGIN = 0,

      /// \brief Unknown graphics interface
      UNKNOWN = GRAPHICS_API_BEGIN,

      /// \brief OpenGL graphics interface
      OPENGL = 1,

      /// \brief Direct3D11 graphics interface
      DIRECT3D11 = 2,

      /// \brief Vulkan graphics interface
      VULKAN = 3,

      /// \brief Metal graphics interface
      METAL = 4,

      /// \internal
      /// \brief Indicator used to create an iterator over the
      /// enum. Do not use this.
      GRAPHICS_API_END
    };

    /// \brief Utils to convert GraphicsAPI to and from strings
    class IGNITION_RENDERING_VISIBLE GraphicsAPIUtils
    {
      /// \brief Convert enum value to string.
      /// \param[in] _e Enum value to convert.
      /// \return String representation of the enum. An empty string is
      /// returned if _e is invalid, or the names for the enum have not been
      /// set.
      public: static std::string Str(GraphicsAPI _e);

      /// \brief Set an enum from a string. This function requires a valid
      /// string, and an array of names for the enum must exist.
      /// The string is not case sensitive.
      /// \param[in] _str String value to convert to enum value.
      /// \return GraphicsAPI enum
      public: static GraphicsAPI Set(const std::string &_str);
    };
    }
  }
}
#endif
