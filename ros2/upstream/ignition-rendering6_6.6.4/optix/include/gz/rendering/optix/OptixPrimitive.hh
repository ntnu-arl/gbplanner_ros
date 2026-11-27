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
#ifndef GZ_RENDERING_OPTIX_OPTIXPRIMITIVE_HH_
#define GZ_RENDERING_OPTIX_OPTIXPRIMITIVE_HH_

#include <string>
#include "gz/rendering/optix/OptixGeometry.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    class IGNITION_RENDERING_OPTIX_VISIBLE OptixPrimitive :
      public OptixGeometry
    {
      protected: OptixPrimitive();

      public: virtual ~OptixPrimitive();

      public: virtual MaterialPtr Material() const;

      public: virtual void SetMaterial(MaterialPtr _material,
                  bool _unique = true);

      public: virtual optix::GeometryGroup OptixGeometryGroup() const;

      public: virtual optix::Acceleration OptixAccel() const;

      public: virtual optix::GeometryInstance OptixGeometryInstance() const;

      public: virtual optix::Geometry OptixGeometry() const;

      public: virtual void PreRender();

      protected: virtual void SetMaterialImpl(OptixMaterialPtr _material);

      protected: virtual void Init();

      protected: static optix::Geometry CreateOptixGeometry(
                     OptixScenePtr _scene, const std::string &_ptxFile);

      protected: MaterialPtr material;

      protected: optix::GeometryGroup optixGeomGroup;

      protected: optix::Acceleration optixAccel;

      protected: optix::GeometryInstance optixGeomInstance;

      protected: optix::Geometry optixGeometry;

      protected: static const std::string PTX_INTERSECT_FUNC;

      protected: static const std::string PTX_BOUNDS_FUNC;
    };
    }
  }
}
#endif
