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
 
// For details and documentation see: thermal_camera_fs.glsl

#include <metal_stdlib>
using namespace metal;

struct PS_INPUT
{
  float2 uv0;
};

struct Params
{
  // The minimum and maximum temprature values (in Kelvin) that the
  // heat signature texture should be normalized to
  // (users can override these defaults)
  float minTemp = 0.0;
  float maxTemp = 100.0;

  int bitDepth;
  float resolution;
};

// map a temperature from the [min, max] range to the user defined
// [minTemp, maxTemp] range
float mapNormalized(
  float num,
  float minTemp,
  float maxTemp,
  float resolution,
  int bitDepth)
{
  float mappedKelvin = ((maxTemp - minTemp) * num) + minTemp;
  return mappedKelvin / (((1 << bitDepth) - 1.0) * resolution);
}

fragment float4 main_metal
(
  PS_INPUT inPs [[stage_in]],
  texture2d<float> renderTexture [[texture(0)]],
  sampler renderTextureSampler [[sampler(0)]],
  constant Params &p [[buffer(PARAMETER_SLOT)]]
)
{
  float heat = renderTexture.sample(renderTextureSampler, inPs.uv0.xy).x;

  // set g, b, a to 0. This will be used by thermal_camera_fs.glsl to determine
  // if a particular fragment is a heat source or not
  float normalisedHeat = mapNormalized(
    heat, p.minTemp, p.maxTemp, p.resolution, p.bitDepth);

  float4 fragColor(normalisedHeat, 0, 0, 0.0);
  return fragColor;
}
