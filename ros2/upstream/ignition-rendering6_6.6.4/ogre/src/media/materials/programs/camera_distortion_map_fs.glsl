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

// The input texture, which is set up by the Ogre Compositor infrastructure.
uniform sampler2D RT;

// Mapping of undistorted to distorted uv coordinates.
uniform sampler2D distortionMap;

// Scale the input texture if necessary to crop black border
uniform vec3 scale;

void main()
{
  vec2 scaleCenter = vec2(0.5, 0.5);
  vec2 inputUV = (gl_TexCoord[0].xy - scaleCenter) / scale.xy + scaleCenter;
  vec4 mapUV = texture2D(distortionMap, inputUV);

  if (mapUV.x < 0.0 || mapUV.y < 0.0)
    gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);
  else
    gl_FragColor = texture2D(RT, mapUV.xy);
}

