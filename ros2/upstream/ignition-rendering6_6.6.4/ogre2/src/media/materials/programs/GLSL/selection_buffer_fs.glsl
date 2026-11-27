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

#version 330

in block
{
  vec2 uv0;
  vec3 cameraDir;
} inPs;

uniform sampler2D colorTexture;
uniform sampler2D depthTexture;
uniform vec4 colorTexResolution;

out vec4 fragColor;

uniform vec2 projectionParams;
uniform float far;
uniform float inf;

float packFloat(vec4 color)
{
  int rgba = (int(color.x * 255.0) << 24) +
             (int(color.y * 255.0) << 16) +
             (int(color.z * 255.0) << 8) +
             int(color.w * 255.0);
  return intBitsToFloat(rgba);
}

void main()
{
  // get linear depth
  float fDepth = texture(depthTexture, inPs.uv0).x;
  float d = projectionParams.y / (fDepth - projectionParams.x);

  // reconstruct 3d viewspace pos from depth
  vec3 viewSpacePos = inPs.cameraDir * d;

  // convert to z up
  vec3 point = vec3(-viewSpacePos.z, -viewSpacePos.x, viewSpacePos.y);

  // set to inf if point is at far clip plane
  if (point.x > far - 1e-4)
    point = vec3(inf);

  // color
  vec4 color = texelFetch(colorTexture,
      ivec2(inPs.uv0 * colorTexResolution.xy), 0);

  float rgba = packFloat(color);

  fragColor = vec4(point.xyz, rgba);
}
