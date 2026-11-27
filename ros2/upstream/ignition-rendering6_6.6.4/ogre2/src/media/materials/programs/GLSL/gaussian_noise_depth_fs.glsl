/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

// This fragment shader will add Gaussian noise to an rgbd image.
// The implementation is adapted from gaussian_noise_fs.glsl to work with
// a float32 rgba texture that consists of [x, y, z, rgba] values

// The input texture, which is set up by the Ogre Compositor infrastructure.
uniform sampler2D RT;

// Random values sampled on the CPU, which we'll use as offsets into our 2-D
// pseudo-random sampler here.
uniform vec3 offsets;
// Mean of the Gaussian distribution that we want to sample from.
uniform float mean;
// Standard deviation of the Gaussian distribution that we want to sample from.
uniform float stddev;


// input params from vertex shader
in block
{
  vec2 uv0;
} inPs;

// final output color
out vec4 fragColor;

#define PI 3.14159265358979323846264

float rand(vec2 co)
{
  // This one-liner can be found in many places, including:
  // http://stackoverflow.com/questions/4200224/random-noise-functions-for-glsl
  // I can't find any explanation for it, but experimentally it does seem to
  // produce approximately uniformly distributed values in the interval [0,1].
  float r = fract(sin(dot(co.xy, vec2(12.9898,78.233))) * 43758.5453);

  // Make sure that we don't return 0.0
  return clamp(r, 0.001, 1.0);
}

vec4 gaussrand(vec2 co)
{
  // Box-Muller method for sampling from the normal distribution
  // http://en.wikipedia.org/wiki/Normal_distribution#Generating_values_from_normal_distribution
  // This method requires 2 uniform random inputs and produces 2
  // Gaussian random outputs.  We'll take a 3rd random variable and use it to
  // switch between the two outputs.

  float U, V, R, Z;
  // Add in the CPU-supplied random offsets to generate the 3 random values that
  // we'll use.
  U = rand(co + vec2(offsets.x, offsets.x));
  V = rand(co + vec2(offsets.y, offsets.y));
  R = rand(co + vec2(offsets.z, offsets.z));
  // Switch between the two random outputs.
  if(R < 0.5)
    Z = sqrt(-2.0 * log(U)) * sin(2.0 * PI * V);
  else
    Z = sqrt(-2.0 * log(U)) * cos(2.0 * PI * V);

  float oldZ = Z;

  // Apply the stddev and mean.
  Z = Z * stddev + mean;

  // Return it as a vec4, to be added to the input ("true") color.
  return vec4(Z, Z, oldZ, 0.0);
}

float pack(vec4 color)
{
  int rgba = (int(color.x * 255.0) << 24) +
             (int(color.y * 255.0) << 16) +
             (int(color.z * 255.0) << 8) +
             int(color.w * 255.0);
  return intBitsToFloat(rgba);
}

vec4 unpack(float color)
{
  int rgba = floatBitsToInt(color);
  int r = rgba >> 24 & 0xFF;
  int g = rgba >> 16 & 0xFF;
  int b = rgba >> 8 & 0xFF;
  int a = rgba & 0xFF;
  return vec4(r/255.0, g/255.0, b/255.0, a/255.0);
}

void main()
{
  // Add the sampled noise to the input x, y, z, rgba values

  // original value
  vec4 p = texture(RT, inPs.uv0.xy);

  // gaussian noise
  float z = gaussrand(inPs.uv0.xy).x;

  // apply noise to xyz
  vec3 xyz =  p.xyz + vec3(z, z, z);

  // apply noise to color
  float n = pow(abs(z), 2.1);
  if (z < 0)
    n = -n;
  vec4 color = unpack(p.a);
  color = clamp(color + vec4(n, n, n, 0.0), 0.0, 1.0);

  float rgba = pack(color);

  fragColor = vec4(xyz, rgba);
}
