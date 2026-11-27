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

#version 330

in block
{
  vec2 uv0;
  vec3 cameraDir;
} inPs;

uniform sampler2D depthTexture;
uniform sampler2D colorTexture;
uniform sampler2D particleTexture;
uniform sampler2D particleDepthTexture;

out uvec4 fragColor;

uniform vec2 projectionParams;
uniform float near;
uniform float far;
uniform float min;
uniform float max;
uniform vec3 backgroundColor;
uniform int hasBackground;

uniform float particleStddev;
uniform float particleScatterRatio;
// rnd is a random number in the range of [0-1]
uniform float rnd;

uint packUnorm4x8Gz(vec4 color)
{
  uint rgba = (uint(round(color.x * 255.0)) << 24u) +
              (uint(round(color.y * 255.0)) << 16u) +
              (uint(round(color.z * 255.0)) << 8u) +
              uint(round(color.w * 255.0));
  return rgba;
}

float toSRGB( float x )
{
  return (x < 0.0031308 ? x * 12.92 : 1.055 * pow( x, 0.41666 ) - 0.055 );
}

vec4 toSRGB( vec4 x )
{
  return vec4( toSRGB( x.x ), toSRGB( x.y ), toSRGB( x.z ), x.w );
}


// see gaussian_noise_fs.glsl for documentation on the rand and gaussrand
// functions

#define PI 3.14159265358979323846264

float rand(vec2 co)
{
  float r = fract(sin(dot(co.xy, vec2(12.9898,78.233))) * 43758.5453);
  // Make sure that we don't return 0.0
  if(r == 0.0)
    return 0.000000000001;
  else
    return r;
}

vec4 gaussrand(vec2 co, vec3 offsets, float stddev, float mean)
{
  float U, V, R, Z;
  U = rand(co + vec2(offsets.x, offsets.x));
  V = rand(co + vec2(offsets.y, offsets.y));
  R = rand(co + vec2(offsets.z, offsets.z));
  if(R < 0.5)
    Z = sqrt(-2.0 * log(U)) * sin(2.0 * PI * V);
  else
    Z = sqrt(-2.0 * log(U)) * cos(2.0 * PI * V);
  Z = Z * stddev + mean;
  return vec4(Z, Z, Z, 0.0);
}

void main()
{
  float tolerance = 1e-6;

  // get linear depth
  float fDepth = texture(depthTexture, inPs.uv0).x;
  float d = projectionParams.y / (fDepth - projectionParams.x);

  // reconstruct 3d viewspace pos from depth
  vec3 viewSpacePos = inPs.cameraDir * d;

  // convert to z up
  vec3 point = vec3(-viewSpacePos.z, -viewSpacePos.x, viewSpacePos.y);

  // color
  vec4 color = texture(colorTexture, inPs.uv0);

  // particle mask - color and depth
  vec4 particle = texture(particleTexture, inPs.uv0);
  float particleDepth = texture(particleDepthTexture, inPs.uv0).x;
  float pd = projectionParams.y / (particleDepth - projectionParams.x);

  // return particle depth if it can be seen by the camera and not obstructed
  // by other objects in the camera view
  if (particle.x > 0 && pd < d)
  {
    // apply scatter effect so that only some of the smoke pixels are visible
    float r = rand(inPs.uv0 + vec2(rnd, rnd));
    if (r < particleScatterRatio)
    {
      // set point to 3d pos of particle pixel
      vec3 particleViewSpacePos = inPs.cameraDir * pd;
      vec3 particlePoint = vec3(-particleViewSpacePos.z, -particleViewSpacePos.x,
          particleViewSpacePos.y);

      float rr = rand(inPs.uv0 + vec2(rnd, rnd)) - 0.5;

      // apply gaussian noise to particle point cloud data
      // With large particles, the range returned are all from the first large
      // particle. So add noise with some mean values so that all the points are
      // shifted further out. This gives depth readings beyond the first few
      // particles and avoid too many early returns
      vec3 noise = gaussrand(inPs.uv0, vec3(rnd, rnd, rnd),
          particleStddev, rr*rr*particleStddev*0.5).xyz;
      float noiseLength = length(noise);
      float particlePointLength = length(particlePoint);
      float newLength = particlePointLength + noiseLength;
      vec3 newPoint = particlePoint * (newLength / particlePointLength);

      // make sure we do not produce depth values larger than depth of first
      // non-particle obstacle, e.g. a box behind particle should still return
      // a hit
      float pointLength = length(point);
      if (newLength < pointLength)
        point = newPoint;
    }
  }

  // clamp xyz and set rgb to background color
  if (length(point) > far - tolerance)
  {
    if (isinf(max))
    {
      point = vec3(max);
    }
    else
    {
      point.x = max;
    }
    // clamp to background color only if it is not a particle pixel
    // this is because point.x may have been set to background depth value
    // due to the scatter effect. We should still render particles in the color
    // image
    // todo(iche033) handle case when background is a cubemap
    if (hasBackground == 0 && particle.x < 1e-6)
    {
      color = vec4(backgroundColor, 1.0);
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

    // clamp to background color only if it is not a particle pixel
    // todo(iche033) handle case when background is a cubemap
    if (hasBackground == 0 && particle.x < 1e-6)
    {
      color = vec4(backgroundColor, 1.0);
    }
  }

  // gamma correct
  color = toSRGB(color);

  fragColor = uvec4(floatBitsToUint(point), packUnorm4x8Gz(color));
}
