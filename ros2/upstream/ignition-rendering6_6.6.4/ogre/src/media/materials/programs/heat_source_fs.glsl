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


uniform vec4 inColor;
uniform float near;
uniform float far;

varying vec4 eyePos;

void main()
{
  // get depth, convert to z up
  float d = -eyePos.z;
  d = (d-near) / (far-near);
  vec4 c = inColor;
  // set y to indicate valid depth
  c.y = 1.0;
  c.z = d;

  gl_FragColor = c;
}
