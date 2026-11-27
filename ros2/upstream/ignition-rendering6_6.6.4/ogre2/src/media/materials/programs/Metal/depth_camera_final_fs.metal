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
 
// For details and documentation see: depth_camera_final_fs.glsl

#include <metal_stdlib>
using namespace metal;

struct PS_INPUT
{
  float2 uv0;
};

struct Params
{
  float near;
  float far;
  float min;
  float max;
  float4 texResolution;
};

fragment uint4 main_metal
(
  PS_INPUT inPs [[stage_in]],
  texture2d<uint> inputTexture [[texture(0)]],
  constant Params &params [[buffer(PARAMETER_SLOT)]]
)
{
  float tolerance = 1e-6;

  // Note: We use PFG_RGBA32_UINT because p.a contains an uint32 and sampling
  // (even w/ point filtering) causes p.a to loss information (e.g.
  // values close to 0 get rounded to 0)
  //
  // See https://github.com/ignitionrobotics/ign-rendering/issues/332
  // Either: Metal equivalent of texelFetch
  uint4 p = inputTexture.read(uint2(inPs.uv0 * params.texResolution.xy), 0);

  float3 point = as_type<float3>(p.xyz);

  // Clamp again in case render passes changed depth values
  // to be outside of min/max range

  // clamp xyz
  if (!isinf(point.x) && length(point) > params.far - tolerance)
  {
    if (isinf(params.max))
    {
      point = float3(params.max);
    }
    else
    {
      point.x = params.max;
    }
  }
  else if (point.x < params.near + tolerance)
  {
    if (isinf(params.min))
    {
      point = float3(params.min);
    }
    else
    {
      point.x = params.min;
    }
  }

  uint4 fragColor(as_type<uint3>(point), p.a);
  return fragColor;
}
