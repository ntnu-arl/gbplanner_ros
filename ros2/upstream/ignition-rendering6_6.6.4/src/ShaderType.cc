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

#include <gz/common/Console.hh>

#include "gz/rendering/ShaderType.hh"

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
const char *ShaderUtil::names[ST_COUNT] =
    {
      "UNKNOWN",
      "pixel",
      "vertex",
      "normal_map_object_space",
      "normal_map_tangent_space",
    };

//////////////////////////////////////////////////
bool ShaderUtil::IsValid(ShaderType _type)
{
  return _type > 0 && _type < ST_COUNT;
}

//////////////////////////////////////////////////
ShaderType ShaderUtil::Sanitize(ShaderType _type)
{
  // check if value within enum bounds
  if (!ShaderUtil::IsValid(_type))
  {
    ignerr << "Invalid ShaderType value: " << _type << std::endl;
    return ST_UNKNOWN;
  }

  return _type;
}

//////////////////////////////////////////////////
std::string ShaderUtil::Name(ShaderType _type)
{
  _type = ShaderUtil::Sanitize(_type);
  return ShaderUtil::names[_type];
}

//////////////////////////////////////////////////
ShaderType ShaderUtil::Enum(const std::string &_name)
{
  // search over all enum elements
  for (unsigned int i = 0; i < ST_COUNT; ++i)
  {
    ShaderType format = static_cast<ShaderType>(i);

    // check if names match
    if (ShaderUtil::Name(format) == _name)
    {
      return format;
    }
  }

  // no match found
  return ST_UNKNOWN;
}
