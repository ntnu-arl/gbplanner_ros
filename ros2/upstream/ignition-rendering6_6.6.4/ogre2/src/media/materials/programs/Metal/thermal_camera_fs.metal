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

// For details and documentation see: thermal_camera_fs.glsl

#include <metal_stdlib>
using namespace metal;

struct PS_INPUT
{
  float2 uv0;
  float3 cameraDir;
};

struct Params
{
  float2 projectionParams;
  float near;
  float far;
  float min;
  float max;
  float range;
  float resolution;
  float heatSourceTempRange;
  float ambient;
  int rgbToTemp;
  int bitDepth;
};

float getDepth(
  float2 uv,
  texture2d<float> depthTexture,
  sampler depthSampler,
  float2 projectionParams)
{
  float fDepth = depthTexture.sample(depthSampler, uv).x;
  float linearDepth = projectionParams.y / (fDepth - projectionParams.x);
  return linearDepth;
}

fragment float4 main_metal
(
  PS_INPUT inPs [[stage_in]],
  texture2d<float>  depthTexture [[texture(0)]],
  texture2d<float>  colorTexture [[texture(1)]],
  sampler           depthSampler [[sampler(0)]],
  sampler           colorSampler [[sampler(1)]],
  constant Params &p [[buffer(PARAMETER_SLOT)]]
)
{
  // temperature defaults to ambient
  float temp = p.ambient;
  float heatRange = p.range;
  int bitMaxValue = (1 << p.bitDepth) - 1;

  // dNorm value is between [0, 1]. It is used to for varying temperature
  // value of a fragment.
  // When dNorm = 1, temp = temp + heatRange*0.5.
  // When dNorm = 0, temp = temp - heatRange*0.5.
  float dNorm = 0.5;

  // check for heat source
  // heat source are objects with uniform temperature or heat signature
  // The custom heat source / signature shaders stores heat data in
  // a vec4 of [heat, 0, 0, 0] so we test to see if it is a heat
  // source by checking gba == 0. This is more of a hack but the idea is to
  // avoid having to render an extra pass to create a mask of heat source
  // objects
  float4 rgba = colorTexture.sample(colorSampler, inPs.uv0).rgba;
  bool isHeatSource = (rgba.g == 0.0 && rgba.b == 0.0 && rgba.a == 0.0);
  float heat = rgba.r;

  if (isHeatSource)
  {
    // heat is normalized so convert back to work in kelvin
    // for 16 bit camera and 0.01 resolution:
    //     ((1 << bitDepth) - 1) * resolution = 655.35
    temp = heat * bitMaxValue * p.resolution;

    // set temperature variation for heat source
    heatRange = p.heatSourceTempRange;
  }
  else
  {
    // other non-heat source objects are assigned ambient temperature
    temp = p.ambient;
  }

  // add temperature variation, either as a function of color or depth
  if (p.rgbToTemp == 1)
  {
    if (!isHeatSource)
    {
      // convert to grayscale: darker = warmer
      // (https://docs.opencv.org/3.4/de/d25/imgproc_color_conversions.html)
      float gray = rgba.r * 0.299 + rgba.g * 0.587 + rgba.b*0.114;
      dNorm = 1.0 - gray;
    }
  }
  else
  {
    // get depth
    float d = getDepth(inPs.uv0, depthTexture, depthSampler, p.projectionParams);
    // reconstruct 3d viewspace pos from depth
    float3 viewSpacePos = inPs.cameraDir * d;
    d = -viewSpacePos.z;
    dNorm = (d-p.near) / (p.far-p.near);
  }

  // simulate temp variations
  float delta = (1.0 - dNorm) * heatRange;
  temp = temp - heatRange / 2.0 + delta;
  clamp(temp, p.min, p.max);

  // apply resolution factor
  temp /= p.resolution;
  // normalize
  float denorm = float(bitMaxValue);
  temp /= denorm;

  float4 fragColor(temp, 0, 0, 1.0);
  return fragColor;
}
