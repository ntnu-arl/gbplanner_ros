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
#include "gz/rendering/optix/OptixMeshFactory.hh"

#include <sstream>
#include <gz/common/Mesh.hh>
#include <gz/common/SubMesh.hh>
#include "gz/rendering/optix/OptixMesh.hh"
#include "gz/rendering/optix/OptixStorage.hh"

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
// OptixMeshFactory
//////////////////////////////////////////////////
OptixMeshFactory::OptixMeshFactory(OptixScenePtr _scene) :
  subMeshStoreFactory(_scene),
  scene(_scene)
{
}

//////////////////////////////////////////////////
OptixMeshFactory::~OptixMeshFactory()
{
}

//////////////////////////////////////////////////
OptixMeshPtr OptixMeshFactory::Create(const MeshDescriptor &_desc)
{
  OptixSubMeshStorePtr subMeshStore;
  MeshDescriptor normDesc = _desc;
  normDesc.Load();
  subMeshStore = this->subMeshStoreFactory.Create(normDesc);

  if (!subMeshStore)
  {
    return nullptr;
  }

  return this->Create(subMeshStore);
}

//////////////////////////////////////////////////
OptixMeshPtr OptixMeshFactory::Create(OptixSubMeshStorePtr _subMeshes)
{
  optix::Context optixContext = this->scene->OptixContext();

  OptixMeshPtr mesh(new OptixMesh);
  mesh->optixGeomGroup = optixContext->createGeometryGroup();
  // mesh->optixAccel =
  //    optixContext->createAcceleration("TriangleKdTree", "KdTree");
  // mesh->optixAccel =
  //    optixContext->createAcceleration("MedianBvh", "Bvh");
  // mesh->optixAccel =
  //    optixContext->createAcceleration("Lbvh", "Bvh");
  mesh->optixAccel = optixContext->createAcceleration("Sbvh", "Bvh");
  mesh->optixAccel->markDirty();

  mesh->optixGeomGroup->setAcceleration(mesh->optixAccel);
  mesh->subMeshes = _subMeshes;

  unsigned int count = _subMeshes->Size();

  for (unsigned int i = 0; i < count; ++i)
  {
    OptixSubMeshPtr subMesh = _subMeshes->DerivedByIndex(i);
    mesh->optixGeomGroup->addChild(subMesh->OptixGeometryInstance());
  }

  return mesh;
}

//////////////////////////////////////////////////
// OptixSubMeshStoreFactory
//////////////////////////////////////////////////
OptixSubMeshStoreFactory::OptixSubMeshStoreFactory(OptixScenePtr _scene) :
  scene(_scene)
{
}

//////////////////////////////////////////////////
OptixSubMeshStoreFactory::~OptixSubMeshStoreFactory()
{
}

//////////////////////////////////////////////////
OptixSubMeshStorePtr OptixSubMeshStoreFactory::Create(
    const MeshDescriptor &_desc)
{
  optix::Context optixContext = this->scene->OptixContext();
  unsigned int count = _desc.mesh->SubMeshCount();
  const std::string searchName = _desc.subMeshName;

  OptixSubMeshStorePtr store(new OptixSubMeshStore);

  for (unsigned int i = 0; i < count; ++i)
  {
    auto subMesh = _desc.mesh->SubMeshByIndex(i).lock();
    const std::string foundName = subMesh->Name();

    if (searchName.empty() || foundName == searchName)
    {
      optix::Geometry optixGeometry = this->Geometry(_desc, i);
      OptixSubMeshPtr sm(new OptixSubMesh);
      sm->id = i;
      sm->name = foundName;
      sm->scene = this->scene;
      sm->optixGeometry = optixGeometry;
      sm->optixGeomInstance = optixContext->createGeometryInstance();
      sm->optixGeomInstance->setGeometry(optixGeometry);

      common::MaterialPtr material;
      material = _desc.mesh->MaterialByIndex(subMesh->MaterialIndex());
      MaterialPtr mat = this->scene->CreateMaterial();
      if (material)
      {
        mat->CopyFrom(*material);
      }
      else
      {
        MaterialPtr defaultMat = this->scene->Material("Default/White");
        if (defaultMat != nullptr)
          mat->CopyFrom(defaultMat);
      }
      // assign material to submesh who will make a copy of this material
      sm->SetMaterial(mat);

      // clean up the material created by factory
      this->scene->DestroyMaterial(mat);

      store->Add(sm);
    }
  }

  return store;
}

//////////////////////////////////////////////////
optix::Geometry OptixSubMeshStoreFactory::Geometry(
    const MeshDescriptor &_desc, unsigned int _subMeshIndex)
{
  const std::string keyName = this->KeyName(_desc, _subMeshIndex);
  auto iter = this->geometries.find(keyName);

  auto subMesh = _desc.mesh->SubMeshByIndex(_subMeshIndex).lock();
  if (iter == this->geometries.end() && subMesh)
  {
    OptixMeshGeometryFactory factory(this->scene, *subMesh.get());
    this->geometries[keyName] = factory.Create();
    iter = this->geometries.find(keyName);
  }

  return iter->second;
}

//////////////////////////////////////////////////
std::string OptixSubMeshStoreFactory::KeyName(const MeshDescriptor &_desc,
    unsigned int _subMeshIndex)
{
  const std::string tail = (_desc.centerSubMesh) ? "_centered" : "_original";

  std::stringstream ss;
  auto subMesh = _desc.mesh->SubMeshByIndex(_subMeshIndex).lock();
  ss << _desc.meshName << "::" << subMesh->Name() << tail;
  return ss.str();
}

//////////////////////////////////////////////////
// OptixMeshGeometryFactory
//////////////////////////////////////////////////
OptixMeshGeometryFactory::OptixMeshGeometryFactory(OptixScenePtr _scene,
    const common::SubMesh &_subMesh) :
  scene(_scene),
  subMesh(_subMesh),
  optixGeometry(nullptr)
{
}

//////////////////////////////////////////////////
OptixMeshGeometryFactory::~OptixMeshGeometryFactory()
{
}

//////////////////////////////////////////////////
optix::Geometry OptixMeshGeometryFactory::Create()
{
  if (!this->optixGeometry)
  {
    this->CreateGeometry();
  }

  return this->optixGeometry;
}

//////////////////////////////////////////////////
void OptixMeshGeometryFactory::CreateGeometry()
{
  optix::Context optixContext = this->scene->OptixContext();
  this->optixGeometry = optixContext->createGeometry();

  optix::Program intersectProgram, boundsProgram;
  intersectProgram = this->scene->CreateOptixProgram("OptixMesh", "Intersect");
  boundsProgram = this->scene->CreateOptixProgram("OptixMesh", "Bounds");

  this->optixGeometry->setIntersectionProgram(intersectProgram);
  this->optixGeometry->setBoundingBoxProgram(boundsProgram);

  this->optixGeometry["vertexBuffer"]->setBuffer(this->CreateVertexBuffer());
  this->optixGeometry["normalBuffer"]->setBuffer(this->CreateNormalBuffer());
  this->optixGeometry["texCoordBuffer"]->setBuffer(
      this->CreateTexCoordBuffer());
  this->optixGeometry["indexBuffer"]->setBuffer(this->CreateIndexBuffer());

  unsigned int count = this->subMesh.IndexCount() / 3;
  this->optixGeometry->setPrimitiveCount(count);
}

//////////////////////////////////////////////////
optix::Buffer OptixMeshGeometryFactory::CreateVertexBuffer()
{
  // create new buffer
  optix::Context optixContext = this->scene->OptixContext();
  optix::Buffer buffer = optixContext->createBuffer(RT_BUFFER_INPUT);
  buffer->setFormat(RT_FORMAT_FLOAT3);

  // update buffer size
  unsigned int count = this->subMesh.VertexCount();
  buffer->setSize(count);

  // create host buffer from device buffer
  float3 *array = static_cast<float3 *>(buffer->map());

  // add each vertex to array
  for (unsigned int i = 0; i < count; ++i)
  {
    // copy vertex to host buffer
    const math::Vector3d &vertex = this->subMesh.Vertex(i);
    array[i].x = vertex.X();
    array[i].y = vertex.Y();
    array[i].z = vertex.Z();
  }

  // copy host buffer to device
  buffer->unmap();
  return buffer;
}

//////////////////////////////////////////////////
optix::Buffer OptixMeshGeometryFactory::CreateNormalBuffer()
{
  // create new buffer
  optix::Context optixContext = this->scene->OptixContext();
  optix::Buffer buffer = optixContext->createBuffer(RT_BUFFER_INPUT);
  buffer->setFormat(RT_FORMAT_FLOAT3);

  // update buffer size
  unsigned int count = this->subMesh.NormalCount();
  buffer->setSize(count);

  // create host buffer from device buffer
  float3 *array = static_cast<float3 *>(buffer->map());

  // add each vertex to array
  for (unsigned int i = 0; i < count; ++i)
  {
    // copy normal to host buffer
    const math::Vector3d &normal = this->subMesh.Normal(i);
    array[i].x = normal.X();
    array[i].y = normal.Y();
    array[i].z = normal.Z();
  }

  // copy host buffer to device
  buffer->unmap();
  return buffer;
}

//////////////////////////////////////////////////
optix::Buffer OptixMeshGeometryFactory::CreateTexCoordBuffer()
{
  // create new buffer
  optix::Context optixContext = this->scene->OptixContext();
  optix::Buffer buffer = optixContext->createBuffer(RT_BUFFER_INPUT);
  buffer->setFormat(RT_FORMAT_FLOAT2);

  // update buffer size
  unsigned int count = this->subMesh.TexCoordCount();
  buffer->setSize(count);

  // create host buffer from device buffer
  float2 *array = static_cast<float2 *>(buffer->map());

  // add each texcoord to array
  for (unsigned int i = 0; i < count; ++i)
  {
    // copy texcoord to host buffer
    const math::Vector2d &texcoord = this->subMesh.TexCoord(i);
    array[i].x = texcoord.X();
    array[i].y = texcoord.Y();
  }

  // copy host buffer to device
  buffer->unmap();
  return buffer;
}

//////////////////////////////////////////////////
optix::Buffer OptixMeshGeometryFactory::CreateIndexBuffer()
{
  // create new buffer
  optix::Context optixContext = this->scene->OptixContext();
  optix::Buffer buffer = optixContext->createBuffer(RT_BUFFER_INPUT);
  buffer->setFormat(RT_FORMAT_INT3);

  // TODO: handle quads

  // update buffer size
  unsigned int count = this->subMesh.IndexCount() / 3;
  buffer->setSize(count);

  // create host buffer from device buffer
  int3 *array = static_cast<int3 *>(buffer->map());
  int index = 0;

  // add each index to array
  for (unsigned int i = 0; i < count; ++i)
  {
    // copy vertex to host buffer
    array[i].x = this->subMesh.Index(index++);
    array[i].y = this->subMesh.Index(index++);
    array[i].z = this->subMesh.Index(index++);
  }

  // copy host buffer to device
  buffer->unmap();
  return buffer;
}
