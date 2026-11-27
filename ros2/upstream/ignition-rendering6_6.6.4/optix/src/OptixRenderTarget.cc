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

#include <gz/common/Console.hh>

#include "gz/rendering/optix/OptixRenderTarget.hh"

#include "gz/rendering/optix/OptixConversions.hh"
#include "gz/rendering/optix/OptixScene.hh"

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
// OptixRenderTarget
//////////////////////////////////////////////////
OptixRenderTarget::OptixRenderTarget() :
  hostData(0)
{
}

//////////////////////////////////////////////////
OptixRenderTarget::~OptixRenderTarget()
{
}

//////////////////////////////////////////////////
void OptixRenderTarget::Copy(Image &_image) const
{
  // TODO: handle Bayer conversions
  // TODO: move shared code to base

  if (_image.Width() != this->width || _image.Height() != this->height)
  {
    ignerr << "Invalid image dimensions" << std::endl;
    return;
  }

  float3 *deviceData = static_cast<float3 *>(this->OptixBuffer()->map());
  unsigned char *imageData = _image.Data<unsigned char>();
  unsigned int count = this->width * this->height;
  unsigned int index = 0;

  for (unsigned int i = 0; i < count; ++i)
  {
    imageData[index++] =
        (unsigned char)fminf(fmaxf(255 * deviceData[i].x, 0), 255);
    imageData[index++] =
        (unsigned char)fminf(fmaxf(255 * deviceData[i].y, 0), 255);
    imageData[index++] =
        (unsigned char)fminf(fmaxf(255 * deviceData[i].z, 0), 255);
  }

  this->OptixBuffer()->unmap();
}

//////////////////////////////////////////////////
unsigned int OptixRenderTarget::MemorySize() const
{
  unsigned int elementSize = this->OptixBuffer()->getElementSize();
  return elementSize * this->Width() * this->Height();
}

//////////////////////////////////////////////////
// OptixRenderTexture
//////////////////////////////////////////////////
OptixRenderTexture::OptixRenderTexture()
{
}

//////////////////////////////////////////////////
OptixRenderTexture::~OptixRenderTexture()
{
}

//////////////////////////////////////////////////
void OptixRenderTexture::Destroy()
{
}

//////////////////////////////////////////////////
optix::Buffer OptixRenderTexture::OptixBuffer() const
{
  return this->optixBuffer;
}

//////////////////////////////////////////////////
void OptixRenderTexture::RebuildImpl()
{
  // TODO: determine pixel format

  delete this->hostData;
  unsigned int count = this->width * this->height * 3;
  this->hostData = new float[count];

  this->optixBuffer->setSize(this->width, this->height);
}

//////////////////////////////////////////////////
void OptixRenderTexture::Init()
{
  BaseRenderTarget::Init();
  optix::Context optixContext = this->scene->OptixContext();
  this->optixBuffer = optixContext->createBuffer(RT_BUFFER_OUTPUT);
  // this->optixBuffer->setFormat(RT_FORMAT_UNSIGNED_BYTE3);
  this->optixBuffer->setFormat(RT_FORMAT_FLOAT3);
}

//////////////////////////////////////////////////
// OptixRenderWindow
//////////////////////////////////////////////////
OptixRenderWindow::OptixRenderWindow()
{
}

//////////////////////////////////////////////////
OptixRenderWindow::~OptixRenderWindow()
{
}


