/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#ifndef GZ_RENDERING_OGRE2_OGRE2MESHFACTORY_HH_
#define GZ_RENDERING_OGRE2_OGRE2MESHFACTORY_HH_

#include <memory>
#include <string>
#include <vector>

#include "gz/rendering/config.hh"
#include "gz/rendering/MeshDescriptor.hh"
#include "gz/rendering/ogre2/Ogre2Mesh.hh"
#include "gz/rendering/ogre2/Ogre2RenderTypes.hh"
#include "gz/rendering/ogre2/Export.hh"

namespace Ogre
{
  class Item;
}

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    // forward declaration
    class Ogre2MeshFactoryPrivate;
    class Ogre2SubMeshStoreFactoryPrivate;

    /// \brief Ogre2.x implementation of the mesh factory class
    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2MeshFactory
    {
      /// \brief Constructor
      /// \param[in] _scene Pointer to the scene
      public: explicit Ogre2MeshFactory(Ogre2ScenePtr _scene);

      /// \brief Destructor
      public: virtual ~Ogre2MeshFactory();

      /// \brief Create a mesh from a descriptor
      /// \param[in] _desc Mesh descriptor containing data needed to create a
      /// mesh
      public: virtual Ogre2MeshPtr Create(const MeshDescriptor &_desc);

      /// \brief Cleanup and clear all internal ogre v2 meshes created by this
      /// factory
      public: virtual void Clear();

      /// \brief Get the ogre item based on the mesh descriptor
      /// \param[in] _desc Descriptor describing the target mesh
      protected: virtual Ogre::Item *OgreItem(
                     const MeshDescriptor &_desc);

      /// \brief Load a mesh using a mesh descriptor
      /// \param[in] _desc Mesh descriptor
      protected: virtual bool Load(const MeshDescriptor &_desc);

      /// \brief Check if the mesh is loaded using a mesh descriptor
      /// \param[in] _desc Mesh descriptor containing the mesh name used
      /// by this function for checking the loaded state
      protected: virtual bool IsLoaded(const MeshDescriptor &_desc);

      /// \brief Helper function to load the mesh from the input mesh descriptor
      /// \param[in] _desc Input mesh descriptor
      protected: virtual bool LoadImpl(const MeshDescriptor &_desc);

      /// \brief Get the mesh name from the mesh descriptor
      /// \param[in] _desc Mesh descriptor containing the mesh name
      protected: virtual std::string MeshName(const MeshDescriptor &_desc);

      /// \brief Validate the mesh descriptor to make sure it contains all the
      /// needed information to create a mesh
      /// \param[in] _desc Mesh descriptor to be validated
      protected: virtual bool Validate(const MeshDescriptor &_desc);

      /// \brief A list of ogre meshes created by this factory
      protected: std::vector<std::string> ogreMeshes;

      /// \brief Pointer to the scene object
      protected: Ogre2ScenePtr scene;

      /// \brief Remove internal material cache for a specific material
      public: void ClearMaterialsCache(const std::string &_name);

      /// \brief Pointer to private data class
      private: std::unique_ptr<Ogre2MeshFactoryPrivate> dataPtr;
    };

    /// \brief Ogre2.x implementation of a submesh store factory class
    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2SubMeshStoreFactory
    {
      /// \brief Constructor
      /// \param[in] _scene Pointer to the scene object
      /// \param[in] _item Parent ogre item
      public: Ogre2SubMeshStoreFactory(Ogre2ScenePtr _scene,
                  Ogre::Item *_item);

      /// \brief Destructor
      public: virtual ~Ogre2SubMeshStoreFactory();

      /// \brief Create the submeshes
      /// \return A store containing all the submeshes
      public: virtual Ogre2SubMeshStorePtr Create();

      /// \brief Helper function to create submesh at the given index
      /// \param[in] _index Index of the ogre subitem. The subitem is then used
      /// to create the submesh.
      protected: virtual Ogre2SubMeshPtr CreateSubMesh(unsigned int _index);

      /// \brief Create a list of names and the corresponding submesh object
      protected: virtual void CreateNameList();

      /// \brief Populate the name list with default generated names
      protected: virtual void PopulateDefaultNames();

      /// \brief Populate the name list with names associated with each ogre
      /// subitem
      protected: virtual void PopulateGivenNames();

      /// \brief Pointer to the scene object
      protected: Ogre2ScenePtr scene;

      /// \brief Pointer to the parent ogre item
      protected: Ogre::Item *ogreItem = nullptr;

      /// \brief A list of names associated with each ogre subitem / submesh
      protected: std::vector<std::string> names;

      /// \brief Pointer to private data class
      private: std::unique_ptr<Ogre2SubMeshStoreFactoryPrivate> dataPtr;
    };
    }
  }
}
#endif
