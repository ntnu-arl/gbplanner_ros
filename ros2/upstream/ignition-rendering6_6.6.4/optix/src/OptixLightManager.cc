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
#include "gz/rendering/optix/OptixLightManager.hh"

#include "gz/rendering/optix/OptixLight.hh"
#include "gz/rendering/optix/OptixScene.hh"
#include "gz/rendering/optix/OptixVisual.hh"

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
OptixLightManager::OptixLightManager(OptixScenePtr _scene) :
  scene(_scene)
{
  this->CreateBuffers();
}

//////////////////////////////////////////////////
OptixLightManager::~OptixLightManager()
{
}

//////////////////////////////////////////////////
void OptixLightManager::AddDirectionalLight(OptixDirectionalLightPtr _light)
{
  this->directionalData.push_back(_light->Data());
}

//////////////////////////////////////////////////
void OptixLightManager::AddPointLight(OptixPointLightPtr _light)
{
  this->pointData.push_back(_light->Data());
}

//////////////////////////////////////////////////
void OptixLightManager::AddSpotLight(OptixSpotLightPtr _light)
{
  this->spotData.push_back(_light->Data());
}

//////////////////////////////////////////////////
void OptixLightManager::PreRender()
{
  this->WriteDirectionalBuffer();
  this->WritePointBuffer();
  this->WriteSpotBuffer();
}

//////////////////////////////////////////////////
void OptixLightManager::Clear()
{
  directionalData.clear();
  pointData.clear();
  spotData.clear();
}

//////////////////////////////////////////////////
void OptixLightManager::WriteDirectionalBuffer()
{
  this->WriteBuffer<OptixDirectionalLightData>(this->directionalBuffer,
      this->directionalData);
}

//////////////////////////////////////////////////
void OptixLightManager::WritePointBuffer()
{
  this->WriteBuffer<OptixPointLightData>(this->pointBuffer, this->pointData);
}

//////////////////////////////////////////////////
void OptixLightManager::WriteSpotBuffer()
{
  this->WriteBuffer<OptixSpotLightData>(this->spotBuffer, this->spotData);
}

//////////////////////////////////////////////////
template <class T>
void OptixLightManager::WriteBuffer(optix::Buffer _buffer,
    const std::vector<T> &_data)
{
  _buffer->setSize(_data.size());
  unsigned int memSize = sizeof(T) * _data.size();
  std::memcpy(_buffer->map(), &_data[0], memSize);
  _buffer->unmap();
}

//////////////////////////////////////////////////
void OptixLightManager::CreateBuffers()
{
  this->directionalBuffer =
      this->CreateBuffer<OptixDirectionalLightData>("directionalLights");

  this->pointBuffer = this->CreateBuffer<OptixPointLightData>("pointLights");
  this->spotBuffer = this->CreateBuffer<OptixSpotLightData>("spotLights");
}

//////////////////////////////////////////////////
template <class T>
optix::Buffer OptixLightManager::CreateBuffer(const std::string &_name)
{
  optix::Context optixContext = this->scene->OptixContext();
  optix::Buffer buffer = optixContext->createBuffer(RT_BUFFER_INPUT);
  optixContext[_name]->setBuffer(buffer);
  buffer->setFormat(RT_FORMAT_USER);
  buffer->setElementSize(sizeof(T));
  return buffer;
}
