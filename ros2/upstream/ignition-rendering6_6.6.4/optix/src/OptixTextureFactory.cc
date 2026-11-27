/* * Copyright (C) 2015 Open Source Robotics Foundation
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

#include <FreeImage.h>
#include <gz/common/Console.hh>

#include "gz/rendering/config.hh"
#include "gz/rendering/optix/OptixTextureFactory.hh"
#include "gz/rendering/optix/OptixScene.hh"

using namespace gz;
using namespace rendering;

// TODO: clone texture sampler, reuse texture buffers

OptixTextureFactory::OptixTextureFactory(OptixScenePtr _scene) :
  scene(_scene)
{
}

//////////////////////////////////////////////////
OptixTextureFactory::~OptixTextureFactory()
{
}

//////////////////////////////////////////////////
optix::TextureSampler OptixTextureFactory::Create(const std::string &_filename)
{
  optix::Buffer buffer = this->CreateBuffer(_filename);
  return this->CreateSampler(buffer);
}

//////////////////////////////////////////////////
optix::TextureSampler OptixTextureFactory::Create()
{
  optix::Buffer buffer = this->CreateBuffer();
  return this->CreateSampler(buffer);
}

//////////////////////////////////////////////////
optix::Buffer OptixTextureFactory::CreateBuffer(const std::string &_filename)
{
  if (_filename.empty())
  {
    ignerr << "Cannot load texture from empty filename" << std::endl;
    return this->CreateBuffer();
  }

  FREE_IMAGE_FORMAT format = FreeImage_GetFileType(_filename.c_str(), 0);
  FIBITMAP *image = FreeImage_Load(format, _filename.c_str());

  if (!image)
  {
    ignerr << "Unable to load texture: " << _filename << std::endl;
    return this->CreateBuffer();
  }

  FIBITMAP *temp = image;
  image = FreeImage_ConvertTo32Bits(image);

  unsigned w = FreeImage_GetWidth(image);
  unsigned h = FreeImage_GetHeight(image);

  // freeimage stores data as BGR[A] on little endian architecture
  // reverse pixel values if needed
#if FREEIMAGE_COLORORDER == FREEIMAGE_COLORORDER_BGR
  unsigned p = FreeImage_GetPitch(image);
  unsigned bpp = FreeImage_GetBPP(image) / 8;
  unsigned lineSize = FreeImage_GetLine(image);
  BYTE* line = FreeImage_GetBits(image);
  for (unsigned y = 0; y < h; ++y, line += p)
  {
    for (BYTE* pixel = line; pixel < line + lineSize; pixel += bpp)
    {
      // in- place swap
      pixel[0] ^= pixel[2]; pixel[2] ^= pixel[0]; pixel[0] ^= pixel[2];
    }
  }
#endif

  FreeImage_Unload(temp);

  optix::Context optixContext = this->scene->OptixContext();

  optix::Buffer buffer = optixContext->createBuffer(RT_BUFFER_INPUT);
  buffer->setFormat(RT_FORMAT_UNSIGNED_BYTE4);
  buffer->setSize(w, h);
  // get raw bits after flipping vertical axis (last bool arg)
  // as free image stores data upside down in memory
  FreeImage_ConvertToRawBits(reinterpret_cast<BYTE *>(buffer->map()),
      image, FreeImage_GetLine(image), FreeImage_GetBPP(image),
      FI_RGBA_RED_MASK, FI_RGBA_GREEN_MASK, FI_RGBA_BLUE_MASK, true);
  buffer->unmap();
  FreeImage_Unload(image);

  return buffer;
}

//////////////////////////////////////////////////
optix::Buffer OptixTextureFactory::CreateBuffer()
{
  unsigned char data[4] = { 0, 0, 0, 0 };
  unsigned int memSize = sizeof(data);

  optix::Context optixContext = this->scene->OptixContext();

  optix::Buffer buffer = optixContext->createBuffer(RT_BUFFER_INPUT);
  buffer->setFormat(RT_FORMAT_UNSIGNED_BYTE4);
  buffer->setSize(1, 1);

  std::memcpy(buffer->map(), &data[0], memSize);
  buffer->unmap();

  return buffer;
}

//////////////////////////////////////////////////
optix::TextureSampler OptixTextureFactory::CreateSampler(optix::Buffer _buffer)
{
  optix::Context optixContext = this->scene->OptixContext();
  optix::TextureSampler sampler = optixContext->createTextureSampler();

  sampler->setWrapMode(0, RT_WRAP_REPEAT);
  sampler->setWrapMode(1, RT_WRAP_REPEAT);
  sampler->setWrapMode(2, RT_WRAP_REPEAT);

  sampler->setIndexingMode(RT_TEXTURE_INDEX_NORMALIZED_COORDINATES);
  sampler->setReadMode(RT_TEXTURE_READ_NORMALIZED_FLOAT);
  sampler->setMaxAnisotropy(1.0);
  sampler->setMipLevelCount(1.0);
  sampler->setArraySize(1);
  sampler->setBuffer(0, 0, _buffer);

  sampler->setFilteringModes(RT_FILTER_LINEAR, RT_FILTER_LINEAR,
      RT_FILTER_NONE);

  return sampler;
}
