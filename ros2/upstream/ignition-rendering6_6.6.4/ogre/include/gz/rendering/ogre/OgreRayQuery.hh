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
#ifndef GZ_RENDERING_OGRE_OGRERAYQUERY_HH_
#define GZ_RENDERING_OGRE_OGRERAYQUERY_HH_

#include <memory>

#include "gz/rendering/base/BaseRayQuery.hh"
#include "gz/rendering/ogre/OgreIncludes.hh"
#include "gz/rendering/ogre/OgreObject.hh"
#include "gz/rendering/ogre/OgreRenderTypes.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    // forward declaration
    class OgreRayQueryPrivate;

    /// \class OgreRayQuery OgreRayQuery.hh
    /// ignition/rendering/base/OgreRayQuery.hh
    /// \brief A Ray Query class used for computing ray object intersections
    class IGNITION_RENDERING_OGRE_VISIBLE OgreRayQuery :
        public BaseRayQuery<OgreObject>
    {
      /// \brief Constructor
      protected: OgreRayQuery();

      /// \brief Destructor
      public: virtual ~OgreRayQuery();

      // Documentation inherited
      public: virtual void SetFromCamera(const CameraPtr &_camera,
                const math::Vector2d &_coord);

      // Documentation inherited
      public: virtual RayQueryResult ClosestPoint();

      /// \brief Get the mesh information for the given mesh.
      /// \param[in] _mesh Mesh to get info about.
      /// \param[out] _vertexCount Number of vertices in the mesh.
      /// \param[out] _vertices Array of the vertices.
      /// \param[out] _indexCount Number if indices.
      /// \param[out] _indices Array of the indices.
      /// \param[in] _position Position of the mesh.
      /// \param[in] _orient Orientation of the mesh.
      /// \param[in] _scale Scale of the mesh
      // Code found in Wiki: www.ogre3d.org/wiki/index.php/RetrieveVertexData
      private: void MeshInformation(const Ogre::Mesh *_mesh,
                                    size_t &_vertexCount,
                                    Ogre::Vector3* &_vertices,
                                    size_t &_indexCount,
                                    uint64_t* &_indices,
                                    const math::Vector3d &_position,
                                    const math::Quaterniond &_orient,
                                    const math::Vector3d &_scale);

      /// \brief Private data pointer
      private: std::unique_ptr<OgreRayQueryPrivate> dataPtr;

      /// \brief Pointer to friend scene class for creating ray query
      private: friend class OgreScene;
    };
    }
  }
}
#endif
