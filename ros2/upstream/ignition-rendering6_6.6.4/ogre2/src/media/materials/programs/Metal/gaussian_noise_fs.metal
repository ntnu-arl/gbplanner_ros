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
 
// For details and documentation see: gaussian_noise_fs.glsl

#include <metal_stdlib>
using namespace metal;

struct PS_INPUT
{
  float2 uv0;
};

struct Params
{
  // Random values sampled on the CPU, which we'll use as offsets into our 2-D
  // pseudo-random sampler here.
  float3 offsets;
  // Mean of the Gaussian distribution that we want to sample from.
  float mean;
  // Standard deviation of the Gaussian distribution that we want to sample from.
  float stddev;
};

#define PI 3.14159265358979323846264

float rand(float2 co)
{
  // This one-liner can be found in many places, including:
  // http://stackoverflow.com/questions/4200224/random-noise-functions-for-glsl
  // I can't find any explanation for it, but experimentally it does seem to
  // produce approximately uniformly distributed values in the interval [0,1].
  float r = fract(sin(dot(co.xy, float2(12.9898,78.233))) * 43758.5453);

  // Make sure that we don't return 0.0
  if(r == 0.0)
    return 0.000000000001;
  else
    return r;
}

float4 gaussrand(float2 co, float3 offsets, float mean, float stddev)
{
  // Box-Muller method for sampling from the normal distribution
  // http://en.wikipedia.org/wiki/Normal_distribution#Generating_values_from_normal_distribution
  // This method requires 2 uniform random inputs and produces 2
  // Gaussian random outputs.  We'll take a 3rd random variable and use it to
  // switch between the two outputs.

  float U, V, R, Z;
  // Add in the CPU-supplied random offsets to generate the 3 random values that
  // we'll use.
  U = rand(co + float2(offsets.x, offsets.x));
  V = rand(co + float2(offsets.y, offsets.y));
  R = rand(co + float2(offsets.z, offsets.z));
  // Switch between the two random outputs.
  if(R < 0.5)
    Z = sqrt(-2.0 * log(U)) * sin(2.0 * PI * V);
  else
    Z = sqrt(-2.0 * log(U)) * cos(2.0 * PI * V);

  // Apply the stddev and mean.
  Z = Z * stddev + mean;

  // Return it as a vec4, to be added to the input ("true") color.
  return float4(Z, Z, Z, 0.0);
}

fragment float4 main_metal
(
  PS_INPUT inPs [[stage_in]],
  texture2d<float> RT [[texture(0)]],
  sampler rtSampler [[sampler(0)]],
  constant Params &p [[buffer(PARAMETER_SLOT)]]
)
{
  // Add the sampled noise to the input color and clamp the result to a valid
  // range.
  // note that an exponent is added to sampled noise, i.e. pow(noise, x),
  // which produces more consistent result with ogre1.x
  float z = gaussrand(inPs.uv0.xy, p.offsets, p.mean, p.stddev).x;
  float n = pow(abs(z), 2.1);
  if (z < 0)
    n = -n;

  float4 texColor = RT.sample(rtSampler, inPs.uv0.xy);
  float4 fragColor = clamp(texColor + float4(n, n, n, 0.0), 0.0, 1.0);

  return fragColor;
}
