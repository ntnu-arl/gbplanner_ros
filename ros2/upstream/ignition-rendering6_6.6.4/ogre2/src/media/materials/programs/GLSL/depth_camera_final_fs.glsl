/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#version 330

in block
{
  vec2 uv0;
} inPs;

uniform usampler2D inputTexture;

out uvec4 fragColor;

uniform float near;
uniform float far;
uniform float min;
uniform float max;

uniform vec4 texResolution;

void main()
{
  float tolerance = 1e-6;

  // Note: We use PFG_RGBA32_UINT because p.a contains an uint32 and sampling
  // (even w/ point filtering) causes p.a to loss information (e.g.
  // values close to 0 get rounded to 0)
  //
  // See https://github.com/gazebosim/gz-rendering/issues/332
  uvec4 p = texelFetch(inputTexture, ivec2(inPs.uv0 *texResolution.xy), 0);

  vec3 point = uintBitsToFloat(p.xyz);

  // Clamp again in case render passes changed depth values
  // to be outside of min/max range

  // clamp xyz
  if (!isinf(point.x) && length(point) > far - tolerance)
  {
    if (isinf(max))
    {
      point = vec3(max);
    }
    else
    {
      point.x = max;
    }
  }
  else if (point.x < near + tolerance)
  {
    if (isinf(min))
    {
      point = vec3(min);
    }
    else
    {
      point.x = min;
    }
  }

  fragColor = uvec4(floatBitsToUint(point), p.a);
}
