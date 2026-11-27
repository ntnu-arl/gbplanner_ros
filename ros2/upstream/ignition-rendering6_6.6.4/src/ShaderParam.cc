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

#include "gz/rendering/ShaderParam.hh"

using namespace gz::rendering;


class gz::rendering::ShaderParamPrivate
{
  /// \brief Type of parameter held
  public: ShaderParam::ParamType type = ShaderParam::PARAM_NONE;

  /// \brief Union for the type held in this param
  public: union
    {
      float vFloat;
      int vInt;
    } paramValue;

  /// \brief buffer of parameter held
  //   Currently only 32-bit elements will be supported for buffers
  public: std::shared_ptr<void> buffer;

  /// \brief Count of elements in buffer of parameter held
  public: uint32_t count = 0u;

  /// \brief Path to texture
  public: std::string texture;

  /// \brief Texture coord set index
  public: uint32_t uvSetIndex = 0u;
};


//////////////////////////////////////////////////
ShaderParam::ShaderParam() :
  dataPtr(new ShaderParamPrivate)
{
  this->dataPtr->count = 0u;
  this->dataPtr->buffer.reset();
}

//////////////////////////////////////////////////
ShaderParam::ShaderParam(const ShaderParam &_other)
  : dataPtr(new ShaderParamPrivate)
{
  // Avoid incorrect cppcheck error about dataPtr being assigned in constructor
  ShaderParamPrivate &dp = *(this->dataPtr);
  dp = *(_other.dataPtr);
}

//////////////////////////////////////////////////
ShaderParam::~ShaderParam()
{
  this->dataPtr->buffer.reset();
}

//////////////////////////////////////////////////
ShaderParam::ParamType ShaderParam::Type() const
{
  return this->dataPtr->type;
}

//////////////////////////////////////////////////
uint32_t ShaderParam::Count() const
{
  return this->dataPtr->count;
}

//////////////////////////////////////////////////
ShaderParam &ShaderParam::operator=(const ShaderParam &_other)
{
  *(this->dataPtr) = *(_other.dataPtr);
  return *this;
}

//////////////////////////////////////////////////
void ShaderParam::operator=(const float _value)
{
  this->dataPtr->type = PARAM_FLOAT;
  this->dataPtr->paramValue.vFloat = _value;
}

//////////////////////////////////////////////////
void ShaderParam::operator=(const int _value)
{
  this->dataPtr->type = PARAM_INT;
  this->dataPtr->paramValue.vInt = _value;
}

//////////////////////////////////////////////////
void ShaderParam::SetTexture(const std::string &_value,
    ShaderParam::ParamType _type, unsigned int _uvSetIndex)
{
  this->dataPtr->type = _type;
  this->dataPtr->texture = _value;
  this->dataPtr->uvSetIndex = _uvSetIndex;
}

//////////////////////////////////////////////////
void ShaderParam::InitializeBuffer(uint32_t _count)
{
  this->dataPtr->count = _count;
  this->dataPtr->buffer.reset(new float[_count],
      std::default_delete<float[]>());
}

//////////////////////////////////////////////////
void ShaderParam::UpdateBuffer(float *_floatBuffer)
{
  this->dataPtr->type = PARAM_FLOAT_BUFFER;
  memcpy(this->dataPtr->buffer.get(), _floatBuffer,
      4 * this->dataPtr->count);
}

//////////////////////////////////////////////////
void ShaderParam::UpdateBuffer(int *_intBuffer)
{
  this->dataPtr->type = PARAM_INT_BUFFER;
  memcpy(this->dataPtr->buffer.get(), _intBuffer,
      4 * this->dataPtr->count);
}

//////////////////////////////////////////////////
bool ShaderParam::Value(float *_value) const
{
  if (PARAM_FLOAT == this->dataPtr->type)
  {
    *_value = this->dataPtr->paramValue.vFloat;
    return true;
  }
  return false;
}

//////////////////////////////////////////////////
bool ShaderParam::Value(int *_value) const
{
  if (PARAM_INT == this->dataPtr->type)
  {
    *_value = this->dataPtr->paramValue.vInt;
    return true;
  }
  return false;
}

//////////////////////////////////////////////////
bool ShaderParam::Buffer(std::shared_ptr<void> &_buffer) const
{
  if (PARAM_FLOAT_BUFFER == this->dataPtr->type ||
      PARAM_INT_BUFFER == this->dataPtr->type)
  {
    _buffer = this->dataPtr->buffer;
    return true;
  }
  return false;
}

//////////////////////////////////////////////////
bool ShaderParam::Value(std::string &_value, uint32_t &_uvSetIndex) const
{
  _value = this->dataPtr->texture;
  _uvSetIndex = this->dataPtr->uvSetIndex;
  return false;
}
