/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include "gz/rendering/ShaderParams.hh"

#include <unordered_map>

using namespace gz::rendering;


class gz::rendering::ShaderParamsPrivate
{
  /// \brief collection of parameters
  public: std::unordered_map<std::string, ShaderParam> parameters;

  /// \brief true if the parameters have been modified since last cleared
  public: bool isDirty = false;
};


class gz::rendering::ShaderParams::IteratorPrivate
{
  /// \brief Iterator from the map in ShaderParamsPrivate
  public: std::unordered_map<std::string, ShaderParam>::const_iterator iter;
};


//////////////////////////////////////////////////
ShaderParams::Iterator::Iterator()
  : dataPtr(new ShaderParams::IteratorPrivate)
{
}

//////////////////////////////////////////////////
ShaderParams::Iterator::Iterator(std::unique_ptr<IteratorPrivate> _dataPtr)
  : dataPtr(std::move(_dataPtr))
{
}

//////////////////////////////////////////////////
ShaderParams::Iterator::Iterator(const ShaderParams::Iterator &_iter)
  : dataPtr(new ShaderParams::IteratorPrivate)
{
  this->dataPtr->iter = _iter.dataPtr->iter;
}

//////////////////////////////////////////////////
ShaderParams::Iterator& ShaderParams::Iterator::operator=(
    const ShaderParams::Iterator &_iter)
{
  this->dataPtr->iter = _iter.dataPtr->iter;
  return *this;
}

//////////////////////////////////////////////////
ShaderParams::Iterator::~Iterator()
{
}

//////////////////////////////////////////////////
bool ShaderParams::Iterator::operator==(const ShaderParams::Iterator &_iter)
{
  return this->dataPtr->iter == _iter.dataPtr->iter;
}

//////////////////////////////////////////////////
bool ShaderParams::Iterator::operator!=(const ShaderParams::Iterator &_iter)
{
  return this->dataPtr->iter != _iter.dataPtr->iter;
}

//////////////////////////////////////////////////
const std::pair<const std::string, ShaderParam> &
ShaderParams::Iterator::operator*()
{
  return *(this->dataPtr->iter);
}

//////////////////////////////////////////////////
const std::pair<const std::string, ShaderParam> *
ShaderParams::Iterator::operator->()
{
  return &*(this->dataPtr->iter);
}

//////////////////////////////////////////////////
ShaderParams::Iterator &ShaderParams::Iterator::operator++()
{
  ++(this->dataPtr->iter);
  return *this;
}

//////////////////////////////////////////////////
ShaderParams::Iterator
ShaderParams::Iterator::operator++(int)  // NOLINT(readability/casting)
{
  ShaderParams::Iterator copy(*this);
  ++(this->dataPtr->iter);
  return copy;
}

//////////////////////////////////////////////////
ShaderParams::ShaderParams() :
  dataPtr(new ShaderParamsPrivate)
{
}

//////////////////////////////////////////////////
ShaderParams::~ShaderParams()
{
}

//////////////////////////////////////////////////
ShaderParam &ShaderParams::operator[](const std::string &_name)
{
  this->dataPtr->isDirty = true;
  return this->dataPtr->parameters[_name];
}

//////////////////////////////////////////////////
const ShaderParam &ShaderParams::operator[](const std::string &_name) const
{
  return this->dataPtr->parameters.at(_name);
}

//////////////////////////////////////////////////
ShaderParams::Iterator ShaderParams::begin() const
{
  auto iterPrivatePtr = std::make_unique<ShaderParams::IteratorPrivate>();
  iterPrivatePtr->iter = this->dataPtr->parameters.begin();
  return ShaderParams::Iterator(std::move(iterPrivatePtr));
}

//////////////////////////////////////////////////
ShaderParams::Iterator ShaderParams::end() const
{
  auto iterPrivatePtr = std::make_unique<ShaderParams::IteratorPrivate>();
  iterPrivatePtr->iter = this->dataPtr->parameters.end();
  return ShaderParams::Iterator(std::move(iterPrivatePtr));
}

//////////////////////////////////////////////////
bool ShaderParams::IsDirty() const
{
  return this->dataPtr->isDirty;
}

//////////////////////////////////////////////////
void ShaderParams::ClearDirty()
{
  this->dataPtr->isDirty = false;
}
