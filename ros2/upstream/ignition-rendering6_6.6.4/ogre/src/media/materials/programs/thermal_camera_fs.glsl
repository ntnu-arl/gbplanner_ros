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

uniform float min;
uniform float max;
uniform float ambient;
uniform float range;
uniform float resolution;
uniform float heatSourceTempRange;
uniform sampler2D heatTexture;

varying vec4 eyePos;

void main()
{
  // temperature defaults to ambient
  float temp = ambient;
  float heatRange = range;

  // check for heat source
  vec4 sample = texture2D(heatTexture, gl_TexCoord[0].xy);
  float heat = sample.x;
  float valid = sample.y;
  float d = 1.0;
  if (valid == 1.0)
    d = sample.z;
  if (heat > 0.0)
  {
    // heat is normalized so convert back to work in kelvin
    temp = heat * 655.35;

    // set temperature variation for heat source
    heatRange = heatSourceTempRange;
  }

  // simulate temp variation as a function of depth
  float delta = (1.0 - d) * heatRange;
  temp = temp - heatRange / 2.0 + delta;
  clamp(temp, min, max);

  // apply resolution factor
  temp /= resolution;
  // normalize
  temp /= 65535.0;

  gl_FragColor = vec4(temp, 0, 0, 1.0);
}
