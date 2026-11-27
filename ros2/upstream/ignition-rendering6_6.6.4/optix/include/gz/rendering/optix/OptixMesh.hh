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
#ifndef GZ_RENDERING_OPTIX_OPTIXMESH_HH_
#define GZ_RENDERING_OPTIX_OPTIXMESH_HH_

#include <string>
#include <vector>
#include "gz/rendering/base/BaseMesh.hh"
#include "gz/rendering/optix/OptixGeometry.hh"
#include "gz/rendering/optix/OptixObject.hh"
#include "gz/rendering/optix/OptixRenderTypes.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    class IGNITION_RENDERING_OPTIX_VISIBLE OptixMesh :
      public BaseMesh<OptixGeometry>
    {
      typedef std::vector<std::string> NameList;

      protected: OptixMesh();

      public: virtual ~OptixMesh();

      public: virtual optix::GeometryGroup OptixGeometryGroup() const;

      public: virtual optix::Acceleration OptixAccel() const;

      protected: virtual SubMeshStorePtr SubMeshes() const;

      protected: OptixSubMeshStorePtr subMeshes;

      protected: optix::GeometryGroup optixGeomGroup;

      protected: optix::Acceleration optixAccel;

      private: friend class OptixScene;

      private: friend class OptixMeshFactory;
    };

    class IGNITION_RENDERING_OPTIX_VISIBLE OptixSubMesh :
      public BaseSubMesh<OptixObject>
    {
      protected: OptixSubMesh();

      public: virtual ~OptixSubMesh();

      public: virtual optix::GeometryInstance OptixGeometryInstance() const;

      protected: virtual void SetMaterialImpl(MaterialPtr _material) override;

      protected: optix::GeometryInstance optixGeomInstance;

      protected: optix::Geometry optixGeometry;

      protected: optix::Buffer optixVertexBuffer;

      protected: optix::Buffer optixNormalBuffer;

      protected: optix::Buffer optixTexCoordBuffer;

      protected: optix::Buffer optixIndexBuffer;

      private: friend class OptixScene;

      private: friend class OptixSubMeshStoreFactory;
    };
    }
  }
}
#endif
