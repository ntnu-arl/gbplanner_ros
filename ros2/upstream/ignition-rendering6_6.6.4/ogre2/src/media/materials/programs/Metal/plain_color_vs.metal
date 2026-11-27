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
 
#include <metal_stdlib>
using namespace metal;

struct VS_INPUT
{
  float4 position [[attribute(VES_POSITION)]];
};

struct PS_INPUT
{
  float4 gl_Position [[position]];
  float gl_ClipDistance [[clip_distance]] [1];
};

struct Params
{
  float4x4 worldViewProj;
  float4x4 worldView;
  float ignMinClipDistance;
};

vertex PS_INPUT main_metal
(
  VS_INPUT input [[stage_in]],
  constant Params &p [[buffer(PARAMETER_SLOT)]]
)
{
  PS_INPUT outVs;

  outVs.gl_Position = ( p.worldViewProj * input.position ).xyzw;

  if( p.ignMinClipDistance > 0.0f )
  {
    // Frustum is rectangular; but the minimum lidar distance is spherical,
    // so we can't rely on near plane to clip geometry that is too close,
    // we have to do it by hand
    float3 viewSpacePos = (p.worldView * input.position).xyz;
    outVs.gl_ClipDistance[0] = length( viewSpacePos.xyz ) - p.ignMinClipDistance;
  }
  else
  {
  	outVs.gl_ClipDistance[0] = 1.0f;
  }

  return outVs;
}
