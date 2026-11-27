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
#include "gz/rendering/Image.hh"

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
template <class T>
struct ArrayDeleter
{
  void operator () (T const * p)
  {
    delete [] p;
  }
};

//////////////////////////////////////////////////
Image::Image(unsigned int _width, unsigned int _height,
  PixelFormat _format) :
  width(_width),
  height(_height)
{
  this->format = PixelUtil::Sanitize(_format);
  unsigned int size = this->MemorySize();
  this->data = DataPtr(new unsigned char[size], ArrayDeleter<unsigned char>());
}

//////////////////////////////////////////////////
Image::~Image()
{
}

//////////////////////////////////////////////////
unsigned int Image::Width() const
{
  return this->width;
}

//////////////////////////////////////////////////
unsigned int Image::Height() const
{
  return this->height;
}

//////////////////////////////////////////////////
PixelFormat Image::Format() const
{
  return this->format;
}

//////////////////////////////////////////////////
unsigned int Image::Depth() const
{
  return PixelUtil::ChannelCount(this->format);
}

//////////////////////////////////////////////////
unsigned int Image::MemorySize() const
{
  return PixelUtil::MemorySize(this->format, this->width, this->height);
}

//////////////////////////////////////////////////
const void *Image::Data() const
{
  return this->data.get();
}

//////////////////////////////////////////////////
void *Image::Data()
{
  return this->data.get();
}
