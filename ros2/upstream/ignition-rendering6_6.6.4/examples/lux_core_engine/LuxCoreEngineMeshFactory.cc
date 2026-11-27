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
#include <ignition/common/Image.hh>
#include <ignition/common/Mesh.hh>
#include <ignition/common/SubMesh.hh>

#include "LuxCoreEngineMeshFactory.hh"
#include "LuxCoreEngineScene.hh"

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
LuxCoreEngineMeshFactory::LuxCoreEngineMeshFactory(LuxCoreEngineScenePtr _scene)
    : scene(_scene)
{
}

//////////////////////////////////////////////////
LuxCoreEngineMeshPtr
LuxCoreEngineMeshFactory::Create(const MeshDescriptor &_desc,
                                 const std::string &_name)
{
  LuxCoreEngineMeshPtr mesh(new LuxCoreEngineMesh);

  std::string meshName = _name;
  meshName.erase(std::remove(meshName.begin(), meshName.end(), '.'),
                 meshName.end());
  mesh->SetName(meshName);

  struct Coordinate
  {
    float x, y;

    Coordinate(float x, float y) : x(x), y(y) {}
    Coordinate() {}
  };

  struct Vertex
  {
    float x, y, z;

    Vertex(float x, float y, float z) : x(x), y(y), z(z) {}
    Vertex() {}
  };

  struct VertexTriangle
  {
    unsigned int v1, v2, v3;

    VertexTriangle(unsigned int v1, unsigned int v2, unsigned int v3)
        : v1(v1), v2(v2), v3(v3) {}
    VertexTriangle() {}
  };

  if (_desc.meshName == "unit_box")
  {
    Vertex *p = (Vertex *)luxcore::Scene::AllocVerticesBuffer(24);

    float minX = -0.5f, minY = -0.5f, minZ = -0.5f;
    float maxX = 0.5f, maxY = 0.5f, maxZ = 0.5f;

    // Bottom face
    p[0] = Vertex(minX, minY, minZ);
    p[1] = Vertex(minX, maxY, minZ);
    p[2] = Vertex(maxX, maxY, minZ);
    p[3] = Vertex(maxX, minY, minZ);
    // Top face
    p[4] = Vertex(minX, minY, maxZ);
    p[5] = Vertex(maxX, minY, maxZ);
    p[6] = Vertex(maxX, maxY, maxZ);
    p[7] = Vertex(minX, maxY, maxZ);
    // Side left
    p[8] = Vertex(minX, minY, minZ);
    p[9] = Vertex(minX, minY, maxZ);
    p[10] = Vertex(minX, maxY, maxZ);
    p[11] = Vertex(minX, maxY, minZ);
    // Side right
    p[12] = Vertex(maxX, minY, minZ);
    p[13] = Vertex(maxX, maxY, minZ);
    p[14] = Vertex(maxX, maxY, maxZ);
    p[15] = Vertex(maxX, minY, maxZ);
    // Side back
    p[16] = Vertex(minX, minY, minZ);
    p[17] = Vertex(maxX, minY, minZ);
    p[18] = Vertex(maxX, minY, maxZ);
    p[19] = Vertex(minX, minY, maxZ);
    // Side front
    p[20] = Vertex(minX, maxY, minZ);
    p[21] = Vertex(minX, maxY, maxZ);
    p[22] = Vertex(maxX, maxY, maxZ);
    p[23] = Vertex(maxX, maxY, minZ);

    VertexTriangle *vi =
        (VertexTriangle *)luxcore::Scene::AllocTrianglesBuffer(12);

    vi[0] = VertexTriangle(0, 1, 2);
    vi[1] = VertexTriangle(2, 3, 0);
    // Top face
    vi[2] = VertexTriangle(4, 5, 6);
    vi[3] = VertexTriangle(6, 7, 4);
    // Side left
    vi[4] = VertexTriangle(8, 9, 10);
    vi[5] = VertexTriangle(10, 11, 8);
    // Side right
    vi[6] = VertexTriangle(12, 13, 14);
    vi[7] = VertexTriangle(14, 15, 12);
    // Side back
    vi[8] = VertexTriangle(16, 17, 18);
    vi[9] = VertexTriangle(18, 19, 16);
    // Side back
    vi[10] = VertexTriangle(20, 21, 22);
    vi[11] = VertexTriangle(22, 23, 20);

    scene->SceneLux()->DefineMesh(meshName + "-mesh", 24, 12, (float *)p,
                                  (unsigned int *)vi, NULL, NULL, NULL, NULL);
  }
  else if (_desc.meshName == "unit_plane")
  {
    Vertex *p = (Vertex *)luxcore::Scene::AllocVerticesBuffer(4);

    float minX = -0.5f, minY = -0.5f, z = 0.0f;
    float maxX = 0.5f, maxY = 0.5f;

    p[0] = Vertex(minX, minY, z);
    p[1] = Vertex(minX, maxY, z);
    p[2] = Vertex(maxX, maxY, z);
    p[3] = Vertex(maxX, minY, z);

    VertexTriangle *vi =
        (VertexTriangle *)luxcore::Scene::AllocTrianglesBuffer(2);

    vi[0] = VertexTriangle(0, 1, 2);
    vi[1] = VertexTriangle(2, 3, 0);

    scene->SceneLux()->DefineMesh(meshName + "-mesh", 4, 2, (float *)p,
                                  (unsigned int *)vi, NULL, NULL, NULL, NULL);
  }
  else
  {
    for (unsigned int i = 0; i < _desc.mesh->SubMeshCount(); i++)
    {
      LuxCoreEngineSubMeshPtr submesh(new LuxCoreEngineSubMesh);

      auto submeshCommon = _desc.mesh->SubMeshByIndex(i).lock();
      std::string submeshName = _name + "-mesh" + "-" + submeshCommon->Name();
      submeshName.erase(
          std::remove(submeshName.begin(), submeshName.end(), '.'),
          submeshName.end());

      submesh->SetName(submeshName);

      // Vertices
      Vertex *p = (Vertex *)luxcore::Scene::AllocVerticesBuffer(
          submeshCommon->VertexCount());

      for (unsigned int x = 0; x < submeshCommon->VertexCount(); x++)
      {
        p[x] = Vertex(submeshCommon->Vertex(x)[0], submeshCommon->Vertex(x)[1],
                      submeshCommon->Vertex(x)[2]);
      }

      // Indices
      unsigned int *vi = (unsigned int *)luxcore::Scene::AllocTrianglesBuffer(
          submeshCommon->IndexCount() / 3);

      for (unsigned int x = 0; x < submeshCommon->IndexCount(); x++)
      {
        vi[x] = submeshCommon->Index(x);
      }

      // Normals
      Vertex *n = (Vertex *)luxcore::Scene::AllocVerticesBuffer(
          submeshCommon->NormalCount());

      for (unsigned int x = 0; x < submeshCommon->NormalCount(); x++)
      {
        n[x] = Vertex(submeshCommon->Normal(x)[0], submeshCommon->Normal(x)[1],
                      submeshCommon->Normal(x)[2]);
      }

      // UV Coordinates
      Coordinate *uv = new Coordinate[submeshCommon->TexCoordCount()];

      for (unsigned int x = 0; x < submeshCommon->TexCoordCount(); x++)
      {
        uv[x] = Coordinate(submeshCommon->TexCoord(x)[0],
                           submeshCommon->TexCoord(x)[1]);
      }

      if (!scene->SceneLux()->IsMeshDefined(submeshName + "-submesh"))
      {
        scene->SceneLux()->DefineMesh(
            submeshName + "-submesh", submeshCommon->VertexCount(),
            submeshCommon->IndexCount() / 3, (float *)p, (unsigned int *)vi,
            (float *)n, (float *)uv, NULL, NULL);
      }

      mesh->AddSubMesh(submesh);

      ignition::common::Image image(
          _desc.mesh->MaterialByIndex(submeshCommon->MaterialIndex())->
              TextureImage());

      if (image.Valid())
      {
        unsigned char* imageData = NULL;
        unsigned int imageDataSize = 0;
        image.Data(&imageData, imageDataSize);

        scene->SceneLux()->DefineImageMap<unsigned char>(
            submeshName + "-texmap", imageData, 1.f, 3, image.Width(),
            image.Height());

        scene->SceneLux()->Parse(
            luxrays::Property("scene.textures." + submeshName + "-texmap" +
                        ".type")("imagemap")
         << luxrays::Property("scene.textures." + submeshName + "-texmap" +
                        ".file")(submeshName + "-texmap")
         << luxrays::Property("scene.textures." + submeshName + "-texmap" +
                        ".gamma")(1.f));

        scene->SceneLux()->Parse(
            luxrays::Property("scene.materials." + submeshName + "-texmap" +
                        ".type")("matte")
         << luxrays::Property("scene.materials." + submeshName + "-texmap" +
                        ".kd")(submeshName + "-texmap"));
      }
    }
  }

  return mesh;
}
