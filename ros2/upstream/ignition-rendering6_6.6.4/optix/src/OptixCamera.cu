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
#include <gz/rendering/optix/OptixRayTypes.hh>

// camera variables
rtDeclareVariable(float3, eye, , );
rtDeclareVariable(float3,   u, , );
rtDeclareVariable(float3,   v, , );
rtDeclareVariable(float3,   w, , );
rtDeclareVariable(uint,    aa, , );
rtBuffer<float3, 2> buffer;

// current ray variables
rtDeclareVariable(uint2, launchIndex, rtLaunchIndex, );
rtDeclareVariable(uint2, launchDim, rtLaunchDim, );

// scene variables
rtDeclareVariable(rtObject, rootGroup, , );
rtDeclareVariable(float, sceneEpsilon, , );

typedef struct Context_t
{

  uint2 subpixel;
  float subpixelWeight;

} Context;

static __inline__ __device__ void AddColor(const uint2 &_index,
    const float3 &_color)
{
  float *channels = (float *)&buffer[_index];
  atomicAdd(&channels[0], _color.x);
  atomicAdd(&channels[1], _color.y);
  atomicAdd(&channels[2], _color.z);
}

static __inline__ __device__ void AddColor(const Context &_context,
    const float3 &_color)
{
  // TODO: clean up

  float3 weightedColor = _color * _context.subpixelWeight;
  AddColor(launchIndex, weightedColor);

  bool addX = _context.subpixel.x == aa - 1 && launchIndex.x < launchDim.x;
  bool addY = _context.subpixel.y == aa - 1 && launchIndex.y < launchDim.y;

  if (addX)
  {
    uint2 index = launchIndex + make_uint2(1, 0);
    AddColor(index, weightedColor);
  }

  if (addY)
  {
    uint2 index = launchIndex + make_uint2(0, 1);
    AddColor(index, weightedColor);
  }

  if (addX && addY)
  {
    uint2 index = launchIndex + make_uint2(1, 1);
    AddColor(index, weightedColor);
  }
}

static __inline__ __device__ void TraceRay(const Context &_context)
{
  float2 offset = make_float2(_context.subpixel) / aa;

  // get image plane intersect point
  float2 pixel = make_float2(launchIndex) + offset;
  float2 size  = make_float2(launchDim);
  float2 ratio = pixel / size - 0.5;

  // create ray that traverses through image plane point
  float3 direction = normalize(ratio.x * u + ratio.y * v + w);
  optix::Ray ray(eye, direction, RT_RADIANCE, sceneEpsilon);

  // initialize ray payload
  OptixRadianceRayData data;
  data.color = make_float3(0, 0, 0);
  data.importance = 1;
  data.depth = 0;

  // trace ray and update buffer
  rtTrace(rootGroup, ray, data);
  AddColor(_context, data.color);
}

static __inline__ __device__ void RenderAA()
{
  Context context;
  context.subpixelWeight = 1.0 / (aa * aa);
  uint &x = context.subpixel.x;
  uint &y = context.subpixel.y;

  for (x = 1; x < aa; ++x)
  {
    for (y = 1; y < aa; ++y)
    {
      TraceRay(context);
    }
  }
}

static __inline__ __device__ void RenderNoAA()
{
  // get image plane intersect point
  float2 pixel = make_float2(launchIndex) + 0.5;
  float2 size  = make_float2(launchDim);
  float2 ratio = pixel / size - 0.5;

  // create ray that traverses through image plane point
  float3 direction = normalize(ratio.x * u + ratio.y * v + w);
  optix::Ray ray(eye, direction, RT_RADIANCE, sceneEpsilon);

  // initialize ray payload
  OptixRadianceRayData data;
  data.color = make_float3(0, 0, 0);
  data.importance = 1;
  data.depth = 0;

  // trace ray and update buffer
  rtTrace(rootGroup, ray, data);
  buffer[launchIndex] = data.color;
}

RT_PROGRAM void Render()
{
  if (aa > 1)
  {
    RenderAA();
  }
  else
  {
    RenderNoAA();
  }
}

RT_PROGRAM void Clear()
{
  buffer[launchIndex] = make_float3(0);
}
