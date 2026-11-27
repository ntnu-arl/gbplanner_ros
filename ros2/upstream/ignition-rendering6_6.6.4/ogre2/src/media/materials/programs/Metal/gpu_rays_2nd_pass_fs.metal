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
 
// For details and documentation see: gpu_rays_2nd_pass_fs.glsl

#include <metal_stdlib>
using namespace metal;

struct PS_INPUT
{
  float2 uv0;
};

struct Params
{
};

float2 getRange(float2 uv, texture2d<float> tex, sampler texSampler)
{
  float2 range = tex.sample(texSampler, uv).xy;
  return range;
}

fragment float4 main_metal
(
  PS_INPUT inPs [[stage_in]],
  texture2d<float>  cubeUVTex [[texture(0)]],
  texture2d<float>  tex0      [[texture(1)]],
  texture2d<float>  tex1      [[texture(2)]],
  texture2d<float>  tex2      [[texture(3)]],
  texture2d<float>  tex3      [[texture(4)]],
  texture2d<float>  tex4      [[texture(5)]],
  texture2d<float>  tex5      [[texture(6)]],
  sampler cubeUVTexSampler    [[sampler(0)]],
  sampler tex0Sampler         [[sampler(1)]],
  sampler tex1Sampler         [[sampler(2)]],
  sampler tex2Sampler         [[sampler(3)]],
  sampler tex3Sampler         [[sampler(4)]],
  sampler tex4Sampler         [[sampler(5)]],
  sampler tex5Sampler         [[sampler(6)]],
  constant Params &p [[buffer(PARAMETER_SLOT)]]
)
{
  // get face index and uv coorodate data
  float3 data = cubeUVTex.sample(cubeUVTexSampler, inPs.uv0).xyz;

  // which face to sample range data from
  float faceIdx = data.z;

  // uv coordinates on texture that stores the range data
  float2 uv = data.xy;

  float2 d;
  d.x = 0;
  d.y = 0;
  if (faceIdx == 0)
    d = getRange(uv, tex0, tex0Sampler);
  else if (faceIdx == 1)
    d = getRange(uv, tex1, tex1Sampler);
  else if (faceIdx == 2)
    d = getRange(uv, tex2, tex2Sampler);
  else if (faceIdx == 3)
    d = getRange(uv, tex3, tex3Sampler);
  else if (faceIdx == 4)
    d = getRange(uv, tex4, tex4Sampler);
  else if (faceIdx == 5)
    d = getRange(uv, tex5, tex5Sampler);

  float range = d.x;
  float retro = d.y;

  float4 fragColor(range, retro, 0, 1.0);
  return fragColor;
}
