/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
 
// Simple vertex shader; just setting things up for the real work to be done in
// gaussian_noise_fs.metal.

#include <metal_stdlib>
using namespace metal;

struct VS_INPUT
{
  float4 position [[attribute(VES_POSITION)]];
  float2 uv0      [[attribute(VES_TEXTURE_COORDINATES0)]];
};

struct PS_INPUT
{
  float4 gl_Position  [[position]];
  float2 uv0;
};

struct Params
{
  float4x4 worldViewProj;
};

vertex PS_INPUT main_metal
(
  VS_INPUT input [[stage_in]],
  constant Params &p [[buffer(PARAMETER_SLOT)]]
)
{
  PS_INPUT outVs;

  outVs.gl_Position = ( p.worldViewProj * input.position ).xyzw;
  outVs.uv0 = input.uv0;

  return outVs;
}
