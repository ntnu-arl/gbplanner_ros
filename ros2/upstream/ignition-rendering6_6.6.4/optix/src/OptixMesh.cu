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

rtBuffer<float3> vertexBuffer;
rtBuffer<float3> normalBuffer;
rtBuffer<float2> texCoordBuffer;
rtBuffer<int3>   indexBuffer;
rtDeclareVariable(float3, scale, , );

rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );
rtDeclareVariable(float3, geometricNormal, attribute geometricNormal, );
rtDeclareVariable(float3, shadingNormal, attribute shadingNormal, );
rtDeclareVariable(float3, shadingTangent, attribute shadingTangent, );
rtDeclareVariable(float2, texCoord, attribute texCoord, );

RT_PROGRAM void Intersect(int primIndex)
{
  int3 indices = indexBuffer[primIndex];

  float3 p0 = scale * vertexBuffer[ indices.x ];
  float3 p1 = scale * vertexBuffer[ indices.y ];
  float3 p2 = scale * vertexBuffer[ indices.z ];

  float3 n;
  float t, beta, gamma;

  if (intersect_triangle(ray, p0, p1, p2, n, t, beta, gamma))
  {

    if (rtPotentialIntersection(t))
    {
      if (normalBuffer.size() == 0 || indices.x < 0 || indices.y < 0 ||
          indices.z < 0 )
      {
        shadingNormal = normalize(n);
      }
      else
      {
        float3 n0 = normalBuffer[ indices.x ] / scale;
        float3 n1 = normalBuffer[ indices.y ] / scale;
        float3 n2 = normalBuffer[ indices.z ] / scale;

        shadingNormal = normalize(n1 * beta + n2 * gamma + n0 *
          (1.0f - beta - gamma));
      }

      geometricNormal = normalize(n);

      if ( texCoordBuffer.size() == 0 || indices.x < 0 || indices.y < 0 ||
        indices.z < 0 )
      {
        texCoord = make_float2( 0.0f, 0.0f );
        shadingTangent = make_float3(0);
      }
      else
      {
        float2 t0 = texCoordBuffer[ indices.x ];
        float2 t1 = texCoordBuffer[ indices.y ];
        float2 t2 = texCoordBuffer[ indices.z ];

        texCoord = t1 * beta + t2 * gamma + t0 * (1.0f - beta - gamma);
        shadingTangent = make_float3(0); // TODO: implement
      }

      rtReportIntersection(0);
    }

  }
}

RT_PROGRAM void Bounds(int _primIndex, float _result[6])
{
  optix::Aabb* aabb = (optix::Aabb*)_result;
  const int3 indices = indexBuffer[_primIndex];

  const float3 v0   = vertexBuffer[ indices.x ];
  const float3 v1   = vertexBuffer[ indices.y ];
  const float3 v2   = vertexBuffer[ indices.z ];
  const float  area = length(cross(v1 - v0, v2 - v0));

  if(area > 0.0f && !isinf(area))
  {
    aabb->m_min = scale * fminf(fminf(v0, v1), v2 );
    aabb->m_max = scale * fmaxf(fmaxf(v0, v1), v2 );
  }
  else
  {
    aabb->invalidate();
  }
}
