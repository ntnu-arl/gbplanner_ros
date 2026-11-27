/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef GZ_RENDERING_EXAMPLES_CAMERAWINDOW_HH_
#define GZ_RENDERING_EXAMPLES_CAMERAWINDOW_HH_

#include <vector>
#include "gz/rendering/RenderTypes.hh"

void GlutRun(std::vector<gz::rendering::CameraPtr> _cameras);

void GlutDisplay();

void GlutIdle();

void GlutKeyboard(unsigned char _key, int _x, int _y);

void GlutReshape(int _width, int _height);

void GlutInitCamera(gz::rendering::CameraPtr _camera);

void GlutInitContext();

void GlutPrintUsage();

#endif
