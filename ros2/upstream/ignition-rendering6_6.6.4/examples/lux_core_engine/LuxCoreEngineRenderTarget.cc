/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#include <ignition/common/Console.hh>

#include "LuxCoreEngineRenderTarget.hh"

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
LuxCoreEngineRenderTarget::LuxCoreEngineRenderTarget() : hostDataBuffer(0)
{
}

//////////////////////////////////////////////////
LuxCoreEngineRenderTarget::~LuxCoreEngineRenderTarget()
{
}

//////////////////////////////////////////////////
void LuxCoreEngineRenderTarget::Copy(Image &_image) const
{
  if (_image.Width() != this->width || _image.Height() != this->height)
  {
    ignerr << "Invalid image dimensions" << std::endl;
    return;
  }

  if (this->hostDataBuffer == NULL)
  {
    ignerr << "Host buffer is NULL" << std::endl;
    return;
  }

  void *imageData = _image.Data<void>();
  memcpy(imageData, this->hostDataBuffer, this->width * this->height * 3);
}

//////////////////////////////////////////////////
void *LuxCoreEngineRenderTarget::HostDataBuffer()
{
  return this->hostDataBuffer;
}

//////////////////////////////////////////////////
void LuxCoreEngineRenderTarget::ResizeHostDataBuffer(unsigned int size)
{
  this->hostDataBuffer = malloc(size);
}

//////////////////////////////////////////////////
unsigned int LuxCoreEngineRenderTarget::MemorySize() const
{
  /// \todo(anyone) Implement this function
  return 0u;
}

//////////////////////////////////////////////////
void LuxCoreEngineRenderTarget::RebuildImpl()
{
  ///TODO Implement this functiom
}
