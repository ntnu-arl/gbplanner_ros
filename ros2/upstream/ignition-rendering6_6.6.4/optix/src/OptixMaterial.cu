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
#include <optix_math.h>
#include <optixu/optixu_matrix_namespace.h>
#include "gz/rendering/optix/OptixRayTypes.hh"
#include "gz/rendering/optix/OptixLightTypes.hh"

// scene variables
rtDeclareVariable(float3, ambientLightColor, , );
rtDeclareVariable(int, maxReflectionDepth, , );
rtDeclareVariable(int, maxRefractionDepth, , );
rtDeclareVariable(float, importanceCutoff, , );
rtDeclareVariable(float, sceneEpsilon, , );
rtDeclareVariable(rtObject, rootGroup, , );
rtBuffer<OptixDirectionalLightData> directionalLights;
rtBuffer<OptixPointLightData> pointLights;
rtTextureSampler<float4, 2> texSampler;
rtTextureSampler<float4, 2> normSampler;
rtDeclareVariable(bool, normWorldSpace, , );

// material variables
rtDeclareVariable(float3, ambient, , );
rtDeclareVariable(float3, diffuse, , );
rtDeclareVariable(float3, emissive, , );
rtDeclareVariable(float, reflectivity, , );
rtDeclareVariable(float, transparency, , );
rtDeclareVariable(uint, lightingEnabled, , );
rtDeclareVariable(uint, castShadows, , );
rtDeclareVariable(uint, receiveShadows, , );

// ray variables
rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );
rtDeclareVariable(OptixRadianceRayData, radianceData, rtPayload, );
rtDeclareVariable(OptixShadowRayData, shadowData, rtPayload, );

// intersect variables
rtDeclareVariable(float, hitDist, rtIntersectionDistance, );
rtDeclareVariable(float3, geometricNormal, attribute geometricNormal, );
rtDeclareVariable(float3, shadingNormal, attribute shadingNormal, );
rtDeclareVariable(float3, shadingTangent, attribute shadingTangent, );
rtDeclareVariable(float2, texCoord, attribute texCoord, );

static __device__ __inline__ float3 Exp(const float3 &_x)
{
  return make_float3(exp(_x.x), exp(_x.y), exp(_x.z));
}

RT_PROGRAM void AnyHit()
{
  float3 shadowAtten   = diffuse;

  if (!castShadows)
  {
    rtIgnoreIntersection();
  }
  else if (transparency > 0)
  {
    float3 worldNormal = normalize(rtTransformNormal(RT_OBJECT_TO_WORLD,
          shadingNormal));

    float ndi = fabs(dot(worldNormal, ray.direction));

    shadowData.attenuation *= 1.0 - fresnel_schlick(ndi, 5, 1 - shadowAtten,
        make_float3(1));

    // float3 ones = make_float3(1);
    // shadowData.attenuation *= 1 - transparency * (ones - ambient) *
    //     (2 - fresnel_schlick(ndi, 5, 1 - shadowAtten, make_float3(1)));

    rtIgnoreIntersection();
  }
  else
  {
    shadowData.attenuation = make_float3(0);
    rtTerminateRay();
  }
}

RT_PROGRAM void ClosestHit()
{
  float  fresnelExp    = 3.0;
  float  fresnelMin    = 0.1;
  float  fresnelMax    = 1.0;
  float  refractIndex  = 1.4;
  float3 refractColor  = diffuse;
  float3 extinctConst  = diffuse;
  float3 cutoffColor   = diffuse;

  float3 color = ambient;
  if (lightingEnabled) color *= ambientLightColor;

  if (emissive.x > 0 || emissive.y > 0 || emissive.z > 0)
    color = emissive;

  float3 worldGeomNorm = normalize(
      rtTransformNormal(RT_OBJECT_TO_WORLD, geometricNormal));

  float3 worldShadeNorm = normalize(
      rtTransformNormal(RT_OBJECT_TO_WORLD, shadingNormal));

  float3 worldShadeTang = normalize(
      rtTransformNormal(RT_OBJECT_TO_WORLD, shadingTangent));

  float3 matNorm = make_float3(tex2D(normSampler, texCoord.x, texCoord.y));

  if ((matNorm.x > 0 || matNorm.y > 0 || matNorm.z > 0) &&
      (shadingTangent.x != 0 || shadingTangent.y != 0 || shadingTangent.z != 0))
  {
    optix::Matrix<3, 3> matrix;
    matrix.setCol(0, worldShadeNorm);
    matrix.setCol(1, worldShadeTang);
    matrix.setCol(2, cross(worldShadeNorm, worldShadeTang));

    matNorm = normalize(matNorm - 0.5);
    worldShadeNorm = matrix * matNorm;
  }

  float3 forwardNormal = faceforward(worldShadeNorm, -ray.direction,
      worldGeomNorm);

  float3 hitPoint = ray.origin + hitDist * ray.direction;

  float reflection = 1.0;
  float3 result = make_float3(0);
  float3 beerAtten = make_float3(1);

  if (transparency > 0)
  {
    float3 beerAtten = (dot(worldShadeNorm, ray.direction) > 0) ?
      Exp(extinctConst * hitDist) : make_float3(1);

    if (radianceData.depth < maxRefractionDepth)
    {
      float3 t;

      if (refract(t, ray.direction, worldShadeNorm, refractIndex))
      {
        float cosTheta = dot(ray.direction, worldGeomNorm);
        cosTheta = (cosTheta < 0) ? -cosTheta : dot(t, worldShadeNorm);

        reflection = fresnel_schlick(cosTheta, fresnelExp, fresnelMin,
            fresnelMax);

        float importance = radianceData.importance * (1 - reflection) *
          optix::luminance(refractColor * beerAtten);

        if (importance > importanceCutoff)
        {
          optix::Ray ray(hitPoint, t, RT_RADIANCE, sceneEpsilon);
          OptixRadianceRayData refrData;
          refrData.depth = radianceData.depth + 1;
          refrData.importance = importance;
          rtTrace(rootGroup, ray, refrData);
          result += (1 - reflectivity) * refractColor * refrData.color;
        }
        else
        {
          result += (1 - reflectivity) * refractColor * cutoffColor;
        }
      }
    }
  }

  // TODO: clean up
  for (int i = 0; i < directionalLights.size() && lightingEnabled; ++i)
  {
    OptixDirectionalLightData light = directionalLights[i];
    float3 l = normalize(-light.direction);
    float ndl = dot(forwardNormal, l);

    if (ndl > 0)
    {
      OptixShadowRayData data;
      data.attenuation = make_float3(1);
      optix::Ray shadowRay(hitPoint, l, RT_SHADOW, sceneEpsilon, 1000);
      rtTrace(rootGroup, shadowRay, data);
      float3 attenuation = data.attenuation;

      if (fmaxf(attenuation) > 0)
      {
        OptixLightAttenuation att = light.common.atten;
        float attp = 1 - fminf(hitDist, att.range) / att.range;

        float attf = att.constant + attp * att.linear + attp *
          attp * att.quadratic;

        // rtPrintf("1 - (%f / %f) = %f\n", hitDist, att.range, attp);

        // rtPrintf("%f + %f * %f + %f * %f * %f = %f\n", att.constant, attp,
        //     att.linear, attp, attp, att.quadratic, attf);

        float4 ld4 = light.common.color.diffuse;
        float3 Lc = make_float3(ld4.x, ld4.y, ld4.z) * attenuation;
        color += diffuse * ndl * Lc * attf;

        float3 H = normalize(l - ray.direction);
        float nDh = dot( forwardNormal, H );

        if(nDh > 0)
        {
          // TODO: include material specular
          // float4 ks4 = light.common.color.specular;
          // float3 Ks = make_float3(ks4.x, ks4.y, ks4.z) * attenuation;
          float3 Ks = make_float3(0.5, 0.5, 0.5);
          float phong_exp = 50;
          color += Ks * Lc * pow(nDh, phong_exp);
        }
      }
    }
  }

  for (int i = 0; i < pointLights.size() && lightingEnabled; ++i)
  {
    OptixPointLightData light = pointLights[i];
    float3 l = normalize(light.common.position - hitPoint);
    float ndl = dot(forwardNormal, l);

    if (ndl > 0)
    {
      OptixShadowRayData data;
      data.attenuation = make_float3(1);
      float dist = length(light.common.position - hitPoint);
      optix::Ray shadowRay(hitPoint, l, RT_SHADOW, sceneEpsilon, dist);
      rtTrace(rootGroup, shadowRay, data);
      float3 attenuation = data.attenuation;

      if (fmaxf(attenuation) > 0)
      {
        // TODO: add light's attenuation
        float4 ld4 = light.common.color.diffuse;
        float3 Lc = make_float3(ld4.x, ld4.y, ld4.z) * attenuation;
        color += diffuse * ndl * Lc;

        float3 H = normalize(l - ray.direction);
        float nDh = dot( forwardNormal, H );

        if(nDh > 0)
        {
          // TODO: include material specular
          // float4 ks4 = light.common.color.specular;
          // float3 Ks = make_float3(ks4.x, ks4.y, ks4.z) * attenuation;
          float3 Ks = make_float3(0.5, 0.5, 0.5);
          float phong_exp = 50;
          color += Ks * Lc * pow(nDh, phong_exp);
        }
      }
    }
  }

  if (reflectivity > 0 && radianceData.depth < maxReflectionDepth)
  {
    OptixRadianceRayData refData;
    refData.depth = radianceData.depth + 1;
    float3 R = reflect(ray.direction, forwardNormal);
    optix::Ray refRay(hitPoint, R, RT_RADIANCE, sceneEpsilon);
    rtTrace(rootGroup, refRay, refData);

    // TODO: determine the actual root of the problem
    if (refData.color.x < 1 || refData.color.y < 1 || refData.color.z < 1)
      color += reflectivity * refData.color;
  }

  float3 tcolor = make_float3(tex2D(texSampler, texCoord.x, texCoord.y));
  float3 finalColor = color + color * tcolor * tcolor * tcolor;

  radianceData.color = (1 - transparency) * finalColor +
      (transparency * result * beerAtten);
}
