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
 
// For details and documentation see: gpu_rays_1st_pass_fs.glsl

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
  float particleStddev;
  float particleScatterRatio;
  // rnd is a random number in the range of [0-1]
  float rnd;
};

// see gaussian_noise_fs.metal for documentation on the rand and gaussrand
// functions

#define PI 3.14159265358979323846264

float rand(float2 co)
{
  float r = fract(sin(dot(co.xy, float2(12.9898,78.233))) * 43758.5453);
  if(r == 0.0)
    return 0.000000000001;
  else
    return r;
}

float4 gaussrand(float2 co, float3 offsets, float mean, float stddev)
{
  float U, V, R, Z;
  U = rand(co + float2(offsets.x, offsets.x));
  V = rand(co + float2(offsets.y, offsets.y));
  R = rand(co + float2(offsets.z, offsets.z));
  if(R < 0.5)
    Z = sqrt(-2.0 * log(U)) * sin(2.0 * PI * V);
  else
    Z = sqrt(-2.0 * log(U)) * cos(2.0 * PI * V);
  Z = Z * stddev + mean;
  return float4(Z, Z, Z, 0.0);
}

fragment float4 main_metal
(
  PS_INPUT inPs [[stage_in]],
  texture2d<float>  depthTexture [[texture(0)]],
  texture2d<float>  colorTexture [[texture(1)]],
  texture2d<float>  particleDepthTexture [[texture(2)]],
  texture2d<float>  particleTexture [[texture(3)]],
  sampler           depthSampler [[sampler(0)]],
  sampler           colorSampler [[sampler(1)]],
  sampler           particleDepthSampler [[sampler(2)]],
  sampler           particleSampler [[sampler(3)]],
  constant Params &p [[buffer(PARAMETER_SLOT)]]
)
{
  // get linear depth
  float fDepth = depthTexture.sample(depthSampler, inPs.uv0).x;
  float d = p.projectionParams.y / (fDepth - p.projectionParams.x);

  // get retro
  float retro = colorTexture.sample(colorSampler, inPs.uv0).x * 2000.0;

  // reconstruct 3d viewspace pos from depth
  float3 viewSpacePos = inPs.cameraDir * d;

  // get length of 3d point, i.e.range
  float l = length(viewSpacePos);

  // particle mask - color and depth
  float4 particle = particleTexture.sample(particleSampler, inPs.uv0);
  float particleDepth = particleDepthTexture.sample(particleDepthSampler, inPs.uv0).x;
  float pd = p.projectionParams.y / (particleDepth - p.projectionParams.x);

  // check if need to apply scatter effect
  if (particle.x > 0.0 && pd < d)
  {
    // apply scatter effect so that only some of the smoke pixels are visible
    float r = rand(inPs.uv0 + float2(p.rnd, p.rnd));
    if (r < p.particleScatterRatio)
    {
      float3 point = inPs.cameraDir * pd;

      float rr = rand(inPs.uv0 + float2(p.rnd, p.rnd)) - 0.5;

      // apply gaussian noise to particle range data
      // With large particles, the range returned are all from the first large
      // particle. So add noise with some mean values so that all the points are
      // shifted further out. This gives depth readings beyond the first few
      // particles and avoid too many early returns
      float3 noise = gaussrand(inPs.uv0, float3(p.rnd, p.rnd, p.rnd),
           p.particleStddev, rr*rr*p.particleStddev*0.5).xyz;
      float noiseLength = length(noise);

      // apply gaussian noise to particle depth data
      float newLength = length(point) + noiseLength;

      // make sure we do not produce values larger than the range of the first
      // non-particle obstacle, e.g. a box behind particle should still return
      // a hit
      if (newLength < l)
        l = newLength;
    }
  }

  if (l > p.far)
    l = p.max;
  else if (l < p.near)
    l = p.min;

  float4 fragColor(l, retro, 0.0, 1.0);
  return fragColor;
}
