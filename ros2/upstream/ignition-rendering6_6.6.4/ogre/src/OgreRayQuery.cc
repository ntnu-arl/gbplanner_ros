/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include <typeinfo>

#include <gz/common/Console.hh>

#include "gz/rendering/ogre/OgreIncludes.hh"
#include "gz/rendering/ogre/OgreCamera.hh"
#include "gz/rendering/ogre/OgreConversions.hh"
#include "gz/rendering/ogre/OgreRayQuery.hh"
#include "gz/rendering/ogre/OgreScene.hh"

class gz::rendering::OgreRayQueryPrivate
{
  /// \brief Ogre ray scene query object for computing intersection.
  public: Ogre::RaySceneQuery *rayQuery = nullptr;
};

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
OgreRayQuery::OgreRayQuery()
    : dataPtr(new OgreRayQueryPrivate)
{
}

//////////////////////////////////////////////////
OgreRayQuery::~OgreRayQuery()
{
}

//////////////////////////////////////////////////
void OgreRayQuery::SetFromCamera(const CameraPtr &_camera,
    const math::Vector2d &_coord)
{
  // convert to nomalized screen pos for ogre
  math::Vector2d screenPos((_coord.X() + 1.0) / 2.0, (_coord.Y() - 1.0) / -2.0);
  OgreCameraPtr camera = std::dynamic_pointer_cast<OgreCamera>(_camera);
  Ogre::Ray ray =
      camera->ogreCamera->getCameraToViewportRay(screenPos.X(), screenPos.Y());

  this->origin = OgreConversions::Convert(ray.getOrigin());
  this->direction = OgreConversions::Convert(ray.getDirection());
}

//////////////////////////////////////////////////
RayQueryResult OgreRayQuery::ClosestPoint()
{
  RayQueryResult result;
  OgreScenePtr ogreScene = std::dynamic_pointer_cast<OgreScene>(this->Scene());
  if (!ogreScene)
    return result;

  Ogre::Ray mouseRay(OgreConversions::Convert(this->origin),
      OgreConversions::Convert(this->direction));

  if (!this->dataPtr->rayQuery)
  {
    this->dataPtr->rayQuery =
        ogreScene->OgreSceneManager()->createRayQuery(mouseRay);
  }
  this->dataPtr->rayQuery->setSortByDistance(true);
  this->dataPtr->rayQuery->setRay(mouseRay);

  // Perform the scene query
  Ogre::RaySceneQueryResult &ogreResult = this->dataPtr->rayQuery->execute();

  double distance = -1.0;

  // Iterate over all the results.
  for (auto iter = ogreResult.begin(); iter != ogreResult.end(); ++iter)
  {
    if (iter->distance <= 0.0)
      continue;

    if (iter->movable && iter->movable->getVisible())
    {
      auto userAny = iter->movable->getUserObjectBindings().getUserAny();
      if (!userAny.isEmpty() && userAny.getType() == typeid(unsigned int) &&
          iter->movable->getMovableType() == "Entity")
      {
        Ogre::Entity *ogreEntity = static_cast<Ogre::Entity*>(iter->movable);

        // mesh data to retrieve
        size_t vertexCount;
        size_t indexCount;
        Ogre::Vector3 *vertices;
        uint64_t *indices;

       // Get the mesh information
        this->MeshInformation(ogreEntity->getMesh().get(), vertexCount,
            vertices, indexCount, indices,
            OgreConversions::Convert(
              ogreEntity->getParentNode()->_getDerivedPosition()),
            OgreConversions::Convert(
            ogreEntity->getParentNode()->_getDerivedOrientation()),
            OgreConversions::Convert(
            ogreEntity->getParentNode()->_getDerivedScale()));

        for (unsigned int i = 0; i < indexCount; i += 3)
        {
          // when indices size is not divisible by 3
          if (i+2 >= indexCount)
            break;

          // check for a hit against this triangle
          std::pair<bool, Ogre::Real> hit = Ogre::Math::intersects(mouseRay,
              vertices[indices[i]],
              vertices[indices[i+1]],
              vertices[indices[i+2]],
              true, false);

          // if it was a hit check if its the closest
          if (hit.first)
          {
            if ((distance < 0.0f) || (hit.second < distance))
            {
              // this is the closest so far, save it off
              distance = hit.second;
              result.distance = distance;
              result.point =
                  OgreConversions::Convert(mouseRay.getPoint(distance));
              result.objectId = Ogre::any_cast<unsigned int>(userAny);
            }
          }
        }
        delete [] vertices;
        delete [] indices;
      }
    }
  }

  return result;
}

//////////////////////////////////////////////////
void OgreRayQuery::MeshInformation(const Ogre::Mesh *_mesh,
                                   size_t &_vertex_count,
                                   Ogre::Vector3* &_vertices,
                                   size_t &_index_count,
                                   uint64_t* &_indices,
                                   const math::Vector3d &_position,
                                   const math::Quaterniond &_orient,
                                   const math::Vector3d &_scale)
{
  bool added_shared = false;
  size_t current_offset = 0;
  size_t next_offset = 0;
  size_t index_offset = 0;

  _vertex_count = _index_count = 0;

  // Calculate how many vertices and indices we're going to need
  for (uint16_t i = 0; i < _mesh->getNumSubMeshes(); ++i)
  {
    Ogre::SubMesh* submesh = _mesh->getSubMesh(i);

    // We only need to add the shared vertices once
    if (submesh->useSharedVertices)
    {
      if (!added_shared)
      {
        _vertex_count += _mesh->sharedVertexData->vertexCount;
        added_shared = true;
      }
    }
    else
    {
      _vertex_count += submesh->vertexData->vertexCount;
    }

    // Add the indices
    _index_count += submesh->indexData->indexCount;
  }

  // Allocate space for the vertices and indices
  _vertices = new Ogre::Vector3[_vertex_count];
  _indices = new uint64_t[_index_count];

  added_shared = false;

  // Run through the submeshes again, adding the data into the arrays
  for (uint16_t i = 0; i < _mesh->getNumSubMeshes(); ++i)
  {
    Ogre::SubMesh* submesh = _mesh->getSubMesh(i);

    Ogre::VertexData* vertex_data = submesh->useSharedVertices ?
        _mesh->sharedVertexData : submesh->vertexData;

    if (!submesh->useSharedVertices || !added_shared)
    {
      if (submesh->useSharedVertices)
      {
        added_shared = true;
      }

      const Ogre::VertexElement* posElem =
        vertex_data->vertexDeclaration->findElementBySemantic(
            Ogre::VES_POSITION);

      Ogre::HardwareVertexBufferSharedPtr vbuf =
        vertex_data->vertexBufferBinding->getBuffer(posElem->getSource());

      unsigned char *vertex =
        static_cast<unsigned char*>(
            vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

      // There is _no_ baseVertexPointerToElement() which takes an
      // Ogre::Real or a double as second argument. So make it float,
      // to avoid trouble when Ogre::Real will be comiled/typedefed as double:
      //      Ogre::Real* pReal;
      float *pReal;

      for (size_t j = 0; j < vertex_data->vertexCount;
           ++j, vertex += vbuf->getVertexSize())
     {
        posElem->baseVertexPointerToElement(vertex, &pReal);
        math::Vector3d pt(pReal[0], pReal[1], pReal[2]);
        _vertices[current_offset + j] =
            OgreConversions::Convert((_orient * (pt * _scale)) + _position);
      }

      vbuf->unlock();
      next_offset += vertex_data->vertexCount;
    }

    Ogre::IndexData* index_data = submesh->indexData;
    Ogre::HardwareIndexBufferSharedPtr ibuf = index_data->indexBuffer;

    if ((ibuf->getType() == Ogre::HardwareIndexBuffer::IT_32BIT))
    {
      uint32_t*  pLong = static_cast<uint32_t*>(
          ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

      for (size_t k = 0; k < index_data->indexCount; k++)
      {
        _indices[index_offset++] = pLong[k];
      }
    }
    else
    {
      uint64_t*  pLong = static_cast<uint64_t*>(
          ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

      uint16_t* pShort = reinterpret_cast<uint16_t*>(pLong);
      for (size_t k = 0; k < index_data->indexCount; k++)
      {
        _indices[index_offset++] = static_cast<uint64_t>(pShort[k]);
      }
    }

    ibuf->unlock();
    current_offset = next_offset;
  }
}
