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
#ifndef GZ_RENDERING_MESHDESCRIPTOR_HH_
#define GZ_RENDERING_MESHDESCRIPTOR_HH_

#include <string>

#include <gz/common/SuppressWarning.hh>

#include "gz/rendering/config.hh"
#include "gz/rendering/Export.hh"

namespace ignition
{
  namespace common
  {
    class Mesh;
  }
}

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \struct MeshDescriptor MeshDescriptor.hh
    /// ignition/rendering/MeshDescriptor.hh
    /// \brief Describes how a Mesh should be loaded
    struct IGNITION_RENDERING_VISIBLE MeshDescriptor
    {
      /// \brief Constructor
      public: MeshDescriptor();

      /// \brief Constructor. A common::Mesh will be retrieved from the
      /// MeshManager by the given name upon a call to Normalize.
      /// \param[in] _meshName Name of the mesh to load
      public: explicit MeshDescriptor(const std::string &_meshName);

      /// \brief Constructor
      /// \param[in] _mesh Mesh to load
      public: explicit MeshDescriptor(const common::Mesh *_mesh);

      /// \brief Ensures both the meshName and mesh member variables have been
      /// assigned. If mesh is not null, it will be used to override the value
      /// of meshName.
      public: void Load();

      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      /// \brief common::Mesh object
      public: const common::Mesh *mesh = nullptr;

      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      /// \brief Name of the registered Mesh
      public: std::string meshName;
      IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING

      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      /// \brief Name of the sub-mesh to be loaded. An empty string signifies
      /// all sub-meshes should be loaded.
      public: std::string subMeshName;
      IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING

      /// \brief Denotes if the loaded sub-mesh vertices should be centered
      public: bool centerSubMesh = false;
    };
    }
  }
}
#endif
