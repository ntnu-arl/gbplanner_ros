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
#include "gz/rendering/optix/OptixCamera.hh"

#include <gz/math/Matrix3.hh>
#include "gz/rendering/optix/OptixIncludes.hh"
#include "gz/rendering/optix/OptixRenderTarget.hh"
#include "gz/rendering/optix/OptixScene.hh"

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////

const std::string OptixCamera::PTX_BASE_NAME("OptixCamera");

const std::string OptixCamera::PTX_RENDER_FUNCTION("Render");

const std::string OptixCamera::PTX_CLEAR_FUNCTION("Clear");

//////////////////////////////////////////////////
OptixCamera::OptixCamera() :
  optixRenderProgram(nullptr),
  optixClearProgram(nullptr),
  optixErrorProgram(nullptr),
  renderTexture(nullptr),
  cameraDirty(true),
  traceId(0)
{
}

//////////////////////////////////////////////////
OptixCamera::~OptixCamera()
{
}

//////////////////////////////////////////////////
void OptixCamera::SetHFOV(const math::Angle &_angle)
{
  BaseCamera::SetHFOV(_angle);
  this->poseDirty = true;
}

//////////////////////////////////////////////////
void OptixCamera::SetAspectRatio(const double _ratio)
{
  BaseCamera::SetAspectRatio(_ratio);
  this->poseDirty = true;
}

//////////////////////////////////////////////////
void OptixCamera::SetAntiAliasing(const unsigned int _aa)
{
  BaseCamera::SetAntiAliasing(_aa);
  this->cameraDirty = true;
}

//////////////////////////////////////////////////
void OptixCamera::PreRender()
{
  BaseCamera::PreRender();
  this->WriteCameraToDevice();
}

//////////////////////////////////////////////////
void OptixCamera::Render()
{
  unsigned int width = this->ImageWidth();
  unsigned int height = this->ImageHeight();
  optix::Context optixContext = this->scene->OptixContext();
  optixContext->launch(this->clearId, width, height);
  optixContext->launch(this->traceId, width, height);
}

//////////////////////////////////////////////////
RenderTargetPtr OptixCamera::RenderTarget() const
{
  return this->renderTexture;
}

//////////////////////////////////////////////////
void OptixCamera::WriteCameraToDevice()
{
  if (this->cameraDirty)
  {
    this->WriteCameraToDeviceImpl();
    this->cameraDirty = false;
  }
}

//////////////////////////////////////////////////
void OptixCamera::WriteCameraToDeviceImpl()
{
  this->optixRenderProgram["aa"]->setUint(this->AntiAliasing() + 1u);
}

//////////////////////////////////////////////////
void OptixCamera::WritePoseToDeviceImpl()
{
  BaseCamera::WritePoseToDeviceImpl();

  math::Pose3d worldPose = this->WorldPose();
  math::Vector3d pos = worldPose.Pos();
  math::Matrix3d rot(worldPose.Rot());

  float3 eye = make_float3(pos.X(), pos.Y(), pos.Z());
  float3   u = make_float3(-rot(0, 1), -rot(1, 1), -rot(2, 1));
  float3   v = make_float3(-rot(0, 2), -rot(1, 2), -rot(2, 2));
  float3   w = make_float3(rot(0, 0),  rot(1, 0),  rot(2, 0));

  // TODO: handle auto and manual aspect-ratio
  // v *= 1 / this->aspectRatio;
  v *= static_cast<float>(this->ImageHeight()) / this->ImageWidth();
  w *= 1 / (2 * tan(this->HFOV().Radian() / 2));

  this->optixRenderProgram["eye"]->setFloat(eye);
  this->optixRenderProgram["u"]->setFloat(u);
  this->optixRenderProgram["v"]->setFloat(v);
  this->optixRenderProgram["w"]->setFloat(w);
}

//////////////////////////////////////////////////
void OptixCamera::Init()
{
  BaseCamera::Init();
  this->CreateRenderTexture();
  this->CreateRenderProgram();
  this->CreateClearProgram();
  this->CreateErrorProgram();
  this->Reset();
}

//////////////////////////////////////////////////
void OptixCamera::CreateRenderTexture()
{
  RenderTexturePtr base = this->scene->CreateRenderTexture();
  this->renderTexture = std::dynamic_pointer_cast<OptixRenderTexture>(base);
  this->renderTexture->SetFormat(PF_R8G8B8);
  this->SetAntiAliasing(1);
}

//////////////////////////////////////////////////
RenderWindowPtr OptixCamera::CreateRenderWindow()
{
  RenderWindowPtr base = this->scene->CreateRenderWindow();
  this->renderTexture = std::dynamic_pointer_cast<OptixRenderWindow>(base);
  this->renderTexture->SetFormat(PF_R8G8B8);
  this->SetAntiAliasing(1);
  return base;
}

//////////////////////////////////////////////////
void OptixCamera::CreateRenderProgram()
{
  this->optixRenderProgram =
      this->scene->CreateOptixProgram(PTX_BASE_NAME, PTX_RENDER_FUNCTION);

  optix::Context optixContext = this->scene->OptixContext();

  optixContext->setRayGenerationProgram(this->traceId,
      this->optixRenderProgram);

  optix::Buffer optixBuffer = this->renderTexture->OptixBuffer();
  this->optixRenderProgram["buffer"]->setBuffer(optixBuffer);
}

//////////////////////////////////////////////////
void OptixCamera::CreateClearProgram()
{
  this->optixClearProgram =
      this->scene->CreateOptixProgram(PTX_BASE_NAME, PTX_CLEAR_FUNCTION);

  optix::Context optixContext = this->scene->OptixContext();

  optixContext->setRayGenerationProgram(this->clearId,
      this->optixClearProgram);

  optix::Buffer optixBuffer = this->renderTexture->OptixBuffer();
  this->optixClearProgram["buffer"]->setBuffer(optixBuffer);
}

//////////////////////////////////////////////////
void OptixCamera::CreateErrorProgram()
{
  // TODO: clean up hard-coded name
  this->optixErrorProgram =
      this->scene->CreateOptixProgram("OptixErrorProgram", "Error");

  optix::Context optixContext = this->scene->OptixContext();
  optix::Buffer optixBuffer = this->renderTexture->OptixBuffer();
  this->optixErrorProgram["buffer"]->setBuffer(optixBuffer);
  optixContext->setExceptionProgram(this->traceId, this->optixErrorProgram);
}
