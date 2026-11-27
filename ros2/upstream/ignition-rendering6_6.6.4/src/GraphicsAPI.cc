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

#include "ignition/rendering/config.hh"
#include "ignition/rendering/GraphicsAPI.hh"
#include <ignition/common/EnumIface.hh>

#include <algorithm>
#include <string>

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
IGN_ENUM(graphicsAPIIface, GraphicsAPI,
    GraphicsAPI::GRAPHICS_API_BEGIN,
    GraphicsAPI::GRAPHICS_API_END,
    "UNKNOWN", "OPENGL", "DIRECT3D11", "VULKAN", "METAL")

//////////////////////////////////////////////////
std::string GraphicsAPIUtils::Str(GraphicsAPI _e)
{
  return graphicsAPIIface.Str(_e);
}

//////////////////////////////////////////////////
GraphicsAPI GraphicsAPIUtils::Set(const std::string &_str)
{
  // Convert to upper case
  std::string str(_str);
  std::transform(str.begin(), str.end(), str.begin(), ::toupper);

  // Set the enum
  GraphicsAPI e{GraphicsAPI::UNKNOWN};
  graphicsAPIIface.Set(e, str);
  return e;
}
