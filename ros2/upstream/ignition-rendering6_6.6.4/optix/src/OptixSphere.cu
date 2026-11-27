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
#include <optix.h>
#include <optixu/optixu_math.h>
#include <optixu/optixu_aabb.h>

rtDeclareVariable(float3, scale, , );
rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );
rtDeclareVariable(float3, geometricNormal, attribute geometricNormal, );
rtDeclareVariable(float3, shadingNormal, attribute shadingNormal, );
rtDeclareVariable(float3, shadingTangent, attribute shadingTangent, );
rtDeclareVariable(float2, texCoord, attribute texCoord, );

static __inline__ __device__ bool ReportPotentialIntersect(float _t)
{
  if (rtPotentialIntersection(_t))
  {
    float radius = scale.x / 2; // TODO: handle scale.y
    float3 normal = (_t * ray.direction + ray.origin) / radius;
    shadingNormal = geometricNormal = normal;

    float xt = (shadingNormal.y > 0) ? -shadingNormal.y :  shadingNormal.y;
    float yt = (shadingNormal.x > 0) ?  shadingNormal.x : -shadingNormal.x;
    shadingTangent = normalize(make_float3(xt, yt, 0));

    float u = atan2(normal.y, normal.x) / M_PI;
    float v = acos(normal.z) / M_PI;
    texCoord = make_float2(u, v) + 0.5;

    return rtReportIntersection(0);
  }

  return false;
}

RT_PROGRAM void Intersect(int)
{
  float3 origin = ray.origin;
  float3 direction = ray.direction;

  float radius = scale.x / 2; // TODO: handle scale.y
  float zzz = dot(origin, direction);
  float aaa = dot(origin, origin) - radius * radius;
  float squaredDistance = zzz * zzz - aaa;

  if (squaredDistance > 0.0f)
  {
    float distance = sqrtf(squaredDistance);
    float t = -zzz - distance;

    if (!ReportPotentialIntersect(t))
    {
      float t = -zzz + distance;
      ReportPotentialIntersect(t);
    }
  }
}

RT_PROGRAM void Bounds(int, float _result[6])
{
  float3 ex = scale / 2;
  optix::Aabb* aabb = (optix::Aabb*)_result;
  aabb->set(-ex, ex);
}
