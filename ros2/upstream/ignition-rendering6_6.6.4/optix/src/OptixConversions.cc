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
#include "gz/rendering/optix/OptixConversions.hh"

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
float4 OptixConversions::ConvertColor(const math::Color &_color)
{
  return make_float4(_color.R(), _color.G(), _color.B(), _color.A());
}

//////////////////////////////////////////////////
math::Color OptixConversions::ConvertColor(const float4 &_color)
{
  return math::Color(_color.x, _color.y, _color.z, _color.w);
}

//////////////////////////////////////////////////
float3 OptixConversions::ConvertVector(const math::Vector3d &_vector)
{
  return make_float3(_vector.X(), _vector.Y(), _vector.Z());
}

//////////////////////////////////////////////////
math::Vector3d OptixConversions::ConvertVector(const float3 &_vector)
{
  return math::Vector3d(_vector.x, _vector.y, _vector.z);
}
