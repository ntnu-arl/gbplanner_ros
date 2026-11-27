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
#ifndef GZ_RENDERING_OPTIX_OPTIXLIGHTTYPES_HH_
#define GZ_RENDERING_OPTIX_OPTIXLIGHTTYPES_HH_

#include <optix_math.h>

#ifndef __CUDA_ARCH__
namespace ignition
{
  namespace rendering
  {
  inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
  //
#endif

  typedef enum OptixLightType_t
  {
    LT_DIRECTIONAL = 0,
    LT_POINT       = 1,
    LT_SPOT        = 2,
    LT_COUNT       = 3,
  } OptixLightType;

  struct OptixLightColor
  {
    float4 diffuse;
    float4 specular;
  };

  struct OptixLightAttenuation
  {
    // cppcheck-suppress unusedStructMember
    float range;
    // cppcheck-suppress unusedStructMember
    float constant;
    // cppcheck-suppress unusedStructMember
    float linear;
    // cppcheck-suppress unusedStructMember
    float quadratic;
  };

  struct OptixLightSpot
  {
    // cppcheck-suppress unusedStructMember
    float innerAngle;
    // cppcheck-suppress unusedStructMember
    float outerAngle;
    // cppcheck-suppress unusedStructMember
    float falloff;
  };

  struct OptixCommonLightData
  {
    float3 position;
    OptixLightColor color;
    OptixLightAttenuation atten;
    // cppcheck-suppress unusedStructMember
    bool castShadows;
  };

  struct OptixDirectionalLightData
  {
    OptixCommonLightData common;
    float3 direction;
  };

  struct OptixPointLightData
  {
    OptixCommonLightData common;
  };

  struct OptixSpotLightData
  {
    OptixCommonLightData common;
    float3 direction;
    OptixLightSpot spot;
  };

#ifndef __CUDA_ARCH__
  }
  }
}
#endif

#endif
