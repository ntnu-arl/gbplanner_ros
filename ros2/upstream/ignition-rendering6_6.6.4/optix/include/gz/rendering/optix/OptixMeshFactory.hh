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
#ifndef GZ_RENDERING_OPTIX_OPTIXMESHFACTORY_HH_
#define GZ_RENDERING_OPTIX_OPTIXMESHFACTORY_HH_

#include <map>
#include <string>
#include <gz/common/Mesh.hh>

#include "gz/rendering/MeshDescriptor.hh"
#include "gz/rendering/optix/OptixRenderTypes.hh"
#include "gz/rendering/optix/OptixMesh.hh"
#include "gz/rendering/optix/OptixIncludes.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    class IGNITION_RENDERING_OPTIX_VISIBLE OptixSubMeshStoreFactory
    {
      // cppcheck-suppress noExplicitConstructor
      public: OptixSubMeshStoreFactory(OptixScenePtr _scene);

      public: virtual ~OptixSubMeshStoreFactory();

      public: virtual OptixSubMeshStorePtr Create(const MeshDescriptor &_desc);

      protected: virtual optix::Geometry Geometry(
                     const MeshDescriptor &_desc, unsigned int _subMeshIndex);

      protected: virtual std::string KeyName(const MeshDescriptor &_desc,
                  unsigned int _subMeshIndex);

      protected: std::map<std::string, optix::Geometry> geometries;

      protected: OptixScenePtr scene;
    };

    class IGNITION_RENDERING_OPTIX_VISIBLE OptixMeshFactory
    {
      // cppcheck-suppress noExplicitConstructor
      public: OptixMeshFactory(OptixScenePtr _scene);

      public: virtual ~OptixMeshFactory();

      public: virtual OptixMeshPtr Create(const MeshDescriptor &_desc);

      protected: virtual OptixMeshPtr Create(OptixSubMeshStorePtr _subMeshes);

      protected: OptixSubMeshStoreFactory subMeshStoreFactory;

      protected: OptixScenePtr scene;
    };

    class IGNITION_RENDERING_OPTIX_VISIBLE OptixMeshGeometryFactory
    {
      public: OptixMeshGeometryFactory(OptixScenePtr _scene,
                  const common::SubMesh &_subMesh);

      public: virtual ~OptixMeshGeometryFactory();

      public: virtual optix::Geometry Create();

      protected: virtual void CreateGeometry();

      protected: virtual optix::Buffer CreateVertexBuffer();

      protected: virtual optix::Buffer CreateNormalBuffer();

      protected: virtual optix::Buffer CreateTexCoordBuffer();

      protected: virtual optix::Buffer CreateIndexBuffer();

      protected: OptixScenePtr scene;

      protected: const common::SubMesh &subMesh;

      protected: optix::Geometry optixGeometry;
    };
    }
  }
}
#endif
