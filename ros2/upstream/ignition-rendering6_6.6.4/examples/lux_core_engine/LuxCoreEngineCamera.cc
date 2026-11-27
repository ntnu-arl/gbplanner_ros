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
#include "LuxCoreEngineCamera.hh"
#include "LuxCoreEngineScene.hh"

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
LuxCoreEngineCamera::LuxCoreEngineCamera()
{
  this->renderTarget =
      LuxCoreEngineRenderTargetPtr(new LuxCoreEngineRenderTarget);
  this->renderTarget->SetFormat(PF_R8G8B8);
}

//////////////////////////////////////////////////
LuxCoreEngineCamera::~LuxCoreEngineCamera()
{
}

//////////////////////////////////////////////////
void LuxCoreEngineCamera::Render()
{
  if (!this->renderSessionLux)
  {
    luxrays::Properties props;

    std::string renderEngineType = "RTPATHOCL";
    if (this->HasUserData("renderengine.type"))
    {
      std::string type =
          std::get<std::string>(this->UserData("renderengine.type"));
      if (!type.empty())
        renderEngineType = type;
    }
    props.Set(luxrays::Property("renderengine.type")(renderEngineType));
    props.Set(luxrays::Property("rtpath.resolutionreduction.preview")("4"));
    props.Set(
        luxrays::Property("rtpath.resolutionreduction.preview.step")("8"));
    props.Set(luxrays::Property("rtpath.resolutionreduction")("4"));

    std::string samplerType;
    if ((renderEngineType == "TILEPATHCPU")
        || (renderEngineType == "TILEPATHOCL")
        || (renderEngineType == "RTPATHOCL"))
      samplerType = "TILEPATHSAMPLER";
    else if (renderEngineType == "RTPATHCPU")
      samplerType = "RTPATHCPUSAMPLER";
    else
      samplerType = "SOBOL";
    props.Set(luxrays::Property("sampler.type")(samplerType));
    props.Set(luxrays::Property("accelerator.type")("BVH"));
    props.Set(luxrays::Property("accelerator.instances.enable")("0"));

    props.Set(luxrays::Property("film.width")(this->ImageWidth()));
    props.Set(luxrays::Property("film.height")(this->ImageHeight()));

    props.Set(luxrays::Property("film.imagepipeline.0.type")("TONEMAP_LINEAR"));
    props.Set(luxrays::Property("film.imagepipeline.0.scale")("1.0"));

    props.Set(
        luxrays::Property("film.imagepipeline.1.type")("GAMMA_CORRECTION"));
    props.Set(luxrays::Property("film.imagepipeline.1.value")("2.2"));
    props.Set(luxrays::Property("film.imagepipeline.1.table.size")("4096"));

    luxcore::RenderConfig *config =
        luxcore::RenderConfig::Create(props, scene->SceneLux());

    this->renderSessionLux = luxcore::RenderSession::Create(config);
    this->renderSessionLux->Start();

    ignmsg << scene->SceneLux()->ToProperties() << std::endl;
    ignmsg << props << std::endl;
  }

  this->renderSessionLux->WaitNewFrame();

  luxcore::Film &film = this->renderSessionLux->GetFilm();

  float *luxcoreBuffer =
      (float *)malloc(film.GetWidth() * film.GetHeight() * 3 * sizeof(float));

  film.GetOutput(luxcore::Film::OUTPUT_RGB_IMAGEPIPELINE, luxcoreBuffer);

  if (this->renderTarget->HostDataBuffer() == NULL)
  {
    this->renderTarget->ResizeHostDataBuffer(this->ImageWidth() *
                                             this->ImageHeight() * 3);
  }

  unsigned char *buffer = (unsigned char *)this->renderTarget->HostDataBuffer();
  for (unsigned int x = 0; x < this->ImageHeight() * this->ImageWidth() * 3;
       x++)
  {
    buffer[x] = luxcoreBuffer[x] * 255;
  }

  free(luxcoreBuffer);
};

//////////////////////////////////////////////////
void LuxCoreEngineCamera::Update()
{
  this->Render();
};

//////////////////////////////////////////////////
void LuxCoreEngineCamera::SetLocalPosition(double _x, double _y, double _z)
{
  this->localPositionX = _x;
  this->localPositionY = _y;
  this->localPositionZ = _z;

  if (renderSessionLux && renderSessionLux->IsStarted())
  {
    renderSessionLux->BeginSceneEdit();
  }

  float targetX = 0;
  float targetY = 0;
  float targetZ = 0;

  if (trackNode)
  {
    targetX = trackNode->LocalPosition()[0];
    targetY = trackNode->LocalPosition()[1];
    targetZ = trackNode->LocalPosition()[2];
  }

  scene->SceneLux()->Parse(
      luxrays::Property("scene.camera.type")("perspective")
      << luxrays::Property("scene.camera.up")(0, 0, -1)
      << luxrays::Property("scene.camera.lookat.orig")(_x, _y, _z)
      << luxrays::Property("scene.camera.lookat.target")(targetX, targetY,
                                                         targetZ));

  if (renderSessionLux && renderSessionLux->IsStarted())
  {
    renderSessionLux->EndSceneEdit();
  }
}

//////////////////////////////////////////////////
void LuxCoreEngineCamera::SetLocalRotation(double /*_r*/, double /*_p*/,
    double /*_y*/)
{
  // \todo(anyone) implement this function
}

//////////////////////////////////////////////////
void LuxCoreEngineCamera::SetHFOV(const math::Angle &_hfov)
{
}

//////////////////////////////////////////////////
RenderTargetPtr LuxCoreEngineCamera::RenderTarget() const
{
  return this->renderTarget;
}
