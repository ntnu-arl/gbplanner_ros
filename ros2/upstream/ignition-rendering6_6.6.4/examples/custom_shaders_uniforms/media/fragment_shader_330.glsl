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

#define M_PI 3.1415926535897932384626433832795

uniform int u_seed;
uniform vec2 u_resolution;
uniform vec3 u_color;
uniform mat4 u_adjustments;

in vec4 interpolatedPosition;

out vec4 fragColor;

float random(vec2 uv, float seed) {
  return fract(sin(mod(dot(uv, vec2(12.9898, 78.233)) + 1113.1 * seed, M_PI)) * 43758.5453);;
}

void main()
{
  vec3 a = vec3(u_adjustments[0][0], u_adjustments[1][0], u_adjustments[2][0]);
  vec2 b = vec2(distance(vec3(interpolatedPosition.xyw), a)) * u_adjustments[3][0];
  vec2 normalizedFragCoord = b / u_resolution;

  vec3 color = vec3(random(normalizedFragCoord, float(u_seed)));
  fragColor = vec4(color * u_color, 1.0);
}
