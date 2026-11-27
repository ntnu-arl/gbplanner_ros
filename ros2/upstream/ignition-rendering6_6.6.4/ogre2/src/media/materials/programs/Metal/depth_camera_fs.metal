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

// For details and documentation see: depth_camera_fs.glsl

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
  float3 backgroundColor;
  int hasBackground;

  float particleStddev;
  float particleScatterRatio;
  // rnd is a random number in the range of [0-1]
  float rnd;
};

uint packUnorm4x8Gz(float4 color)
{
  uint rgba = (uint(round(color.x * 255.0)) << 24u) +
              (uint(round(color.y * 255.0)) << 16u) +
              (uint(round(color.z * 255.0)) << 8u) +
              uint(round(color.w * 255.0));
  return rgba;
}

inline float toSRGB( float x )
{
  return (x < 0.0031308 ? x * 12.92 : 1.055 * pow( x, 0.41666 ) - 0.055 );
}

inline float4 toSRGB( float4 x )
{
  return float4( toSRGB( x.x ), toSRGB( x.y ), toSRGB( x.z ), x.w );
}

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

fragment uint4 main_metal
(
  PS_INPUT inPs [[stage_in]],
  texture2d<float>  depthTexture [[texture(0)]],
  texture2d<float>  colorTexture [[texture(1)]],
  texture2d<float>  particleTexture [[texture(2)]],
  texture2d<float>  particleDepthTexture [[texture(3)]],
  sampler           depthSampler [[sampler(0)]],
  sampler           colorSampler [[sampler(1)]],
  sampler           particleSampler [[sampler(2)]],
  sampler           particleDepthSampler [[sampler(3)]],
  constant Params &p [[buffer(PARAMETER_SLOT)]]
)
{
  float tolerance = 1e-6;

  // get linear depth
  float fDepth = depthTexture.sample(depthSampler, inPs.uv0).x;
  float d = p.projectionParams.y / (fDepth - p.projectionParams.x);

  // reconstruct 3d viewspace pos from depth
  float3 viewSpacePos = inPs.cameraDir * d;

  // convert to z up
  float3 point = float3(-viewSpacePos.z, -viewSpacePos.x, viewSpacePos.y);

  // color
  float4 color = colorTexture.sample(colorSampler, inPs.uv0);

  // particle mask - color and depth
  float4 particle = particleTexture.sample(particleSampler, inPs.uv0);
  float particleDepth = particleDepthTexture.sample(particleDepthSampler, inPs.uv0).x;
  float pd = p.projectionParams.y / (particleDepth - p.projectionParams.x);

  // return particle depth if it can be seen by the camera and not obstructed
  // by other objects in the camera view
  if (particle.x > 0 && pd < d)
  {
    // apply scatter effect so that only some of the smoke pixels are visible
    float r = rand(inPs.uv0 + float2(p.rnd, p.rnd));
    if (r < p.particleScatterRatio)
    {
      // set point to 3d pos of particle pixel
      float3 particleViewSpacePos = inPs.cameraDir * pd;
      float3 particlePoint = float3(
          -particleViewSpacePos.z,
          -particleViewSpacePos.x,
          particleViewSpacePos.y);

      float rr = rand(inPs.uv0 + float2(p.rnd, p.rnd)) - 0.5;

      // apply gaussian noise to particle point cloud data
      // With large particles, the range returned are all from the first large
      // particle. So add noise with some mean values so that all the points are
      // shifted further out. This gives depth readings beyond the first few
      // particles and avoid too many early returns
      float3 noise = gaussrand(inPs.uv0, float3(p.rnd, p.rnd, p.rnd),
          p.particleStddev, rr*rr*p.particleStddev*0.5).xyz;
      float noiseLength = length(noise);
      float particlePointLength = length(particlePoint);
      float newLength = particlePointLength + noiseLength;
      float3 newPoint = particlePoint * (newLength / particlePointLength);

      // make sure we do not produce depth values larger than depth of first
      // non-particle obstacle, e.g. a box behind particle should still return
      // a hit
      float pointLength = length(point);
      if (newLength < pointLength)
        point = newPoint;
    }
  }

  // clamp xyz and set rgb to background color
  if (length(point) > p.far - tolerance)
  {
    if (isinf(p.max))
    {
      point = float3(p.max);
    }
    else
    {
      point.x = p.max;
    }
    // clamp to background color only if it is not a particle pixel
    // this is because point.x may have been set to background depth value
    // due to the scatter effect. We should still render particles in the color
    // image
    // todo(iche033) handle case when background is a cubemap
    if (p.hasBackground == 0 && particle.x < 1e-6)
    {
      color = float4(p.backgroundColor, 1.0);
    }
  }
  else if (point.x < p.near + tolerance)
  {
    if (isinf(p.min))
    {
      point = float3(p.min);
    }
    else
    {
      point.x = p.min;
    }

    // clamp to background color only if it is not a particle pixel
    // todo(iche033) handle case when background is a cubemap
    if (p.hasBackground == 0 && particle.x < 1e-6)
    {
      color = float4(p.backgroundColor, 1.0);
    }
  }

  // gamma correct
  color = toSRGB(color);

  uint4 fragColor(as_type<uint3>(point), packUnorm4x8Gz(color));
  return fragColor;
}
