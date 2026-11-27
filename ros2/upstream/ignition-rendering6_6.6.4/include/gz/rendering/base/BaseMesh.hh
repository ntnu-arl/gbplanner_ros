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
#ifndef GZ_RENDERING_BASE_BASEMESH_HH_
#define GZ_RENDERING_BASE_BASEMESH_HH_

#include <map>
#include <string>
#include <unordered_map>
#include "gz/rendering/Mesh.hh"
#include "gz/rendering/RenderEngine.hh"
#include "gz/rendering/Storage.hh"
#include "gz/rendering/base/BaseObject.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    //////////////////////////////////////////////////
    template <class T>
    class BaseMesh :
      public virtual Mesh,
      public virtual T
    {
      protected: BaseMesh();

      public: virtual ~BaseMesh();

      // Documentation inherited.
      public: virtual bool HasSkeleton() const override;

      // Documentation inherited.
      public: virtual std::map<std::string, math::Matrix4d>
                      SkeletonLocalTransforms() const override;

      // Documentation inherited.
      public: virtual void SetSkeletonLocalTransforms(
                      const std::map<std::string, math::Matrix4d> &_tfs)
                      override;

      // Documentation inherited.
      public: virtual std::unordered_map<std::string, float> SkeletonWeights()
                      const override;

      // Documentation inherited.
      public: virtual void SetSkeletonWeights(
                      const std::unordered_map<std::string, float> &_weights)
                      override;

      // Documentation inherited.
      public: virtual void SetSkeletonAnimationEnabled(const std::string &_name,
            bool _enabled, bool _loop = true, float _weight = 1.0) override;

      // Documentation inherited.
      public: virtual bool SkeletonAnimationEnabled(const std::string &_name)
            const override;

      // Documentation inherited.
      public: virtual void UpdateSkeletonAnimation(
            std::chrono::steady_clock::duration _time) override;

      public: virtual unsigned int SubMeshCount() const override;

      public: virtual bool HasSubMesh(ConstSubMeshPtr _subMesh) const override;

      public: virtual bool HasSubMeshName(const std::string &_name) const
                      override;

      public: virtual SubMeshPtr SubMeshByName(
                  const std::string &_name) const override;

      public: virtual SubMeshPtr SubMeshByIndex(unsigned int _index) const
                      override;

      // Documentation inherited.
      public: virtual MaterialPtr Material() const override;

      // Documentation inherited.
      public: virtual void SetMaterial(const std::string &_name,
                  bool _unique = true) override;

      // Documentation inherited.
      public: virtual void SetMaterial(MaterialPtr _material,
                  bool _unique = true) override;

      public: virtual void PreRender() override;

      // Documentation inherited.
      public: virtual GeometryPtr Clone() const override;

      // Documentation inherited.
      public: void SetDescriptor(const MeshDescriptor &_desc) override;

      // Documentation inherited.
      public: const MeshDescriptor &Descriptor() const override;

      // Documentation inherited
      public: virtual void Destroy() override;

      protected: virtual SubMeshStorePtr SubMeshes() const = 0;

      /// \brief Flag to indicate whether or not this mesh should be
      /// responsible for destroying the material
      protected: bool ownsMaterial = false;

      /// \brief Pointer to currently assigned material
      protected: MaterialPtr material;

      /// \brief MeshDescriptor for this mesh
      protected: MeshDescriptor meshDescriptor;
    };

    //////////////////////////////////////////////////
    template <class T>
    class BaseSubMesh :
      public virtual SubMesh,
      public virtual T
    {
      protected: BaseSubMesh();

      public: virtual ~BaseSubMesh();

      // Documentation inherited
      public: virtual MaterialPtr Material() const override;

      // Documentation inherited
      public: virtual void SetMaterial(const std::string &_name,
                  bool _unique = true) override;

      // Documentation inherited
      public: virtual void SetMaterial(MaterialPtr _material,
                  bool _unique = true) override;

      /// \brief Engine implementation for setting the material of this SubMesh.
      /// \param[in] _material New Material to be assigned
      public: virtual void SetMaterialImpl(MaterialPtr _material) = 0;

      public: virtual void PreRender() override;

      // Documentation inherited
      public: virtual void Destroy() override;

      /// \brief Flag to indicate whether or not this submesh should be
      /// responsible for destroying the material
      protected: bool ownsMaterial = false;

      /// \brief Pointer to currently assigned material
      protected: MaterialPtr material;
    };

    //////////////////////////////////////////////////
    // BaseMesh
    //////////////////////////////////////////////////
    template <class T>
    BaseMesh<T>::BaseMesh()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    BaseMesh<T>::~BaseMesh()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseMesh<T>::HasSkeleton() const
    {
      return false;
    }

    //////////////////////////////////////////////////
    template <class T>
    std::map<std::string, math::Matrix4d>
          BaseMesh<T>::SkeletonLocalTransforms() const
    {
      std::map<std::string, gz::math::Matrix4d> tmpMap;
      return tmpMap;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMesh<T>::SetSkeletonLocalTransforms(
          const std::map<std::string, math::Matrix4d> &)
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    std::unordered_map<std::string, float> BaseMesh<T>::SkeletonWeights() const
    {
      std::unordered_map<std::string, float> tmpMap;
      return tmpMap;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMesh<T>::SetSkeletonWeights(
          const std::unordered_map<std::string, float> &)
    {
      ignerr << "SetSkeletonWeights not supported for render engine: "
             << this->Scene()->Engine()->Name() << std::endl;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMesh<T>::SetSkeletonAnimationEnabled(const std::string &, bool,
        bool, float)
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseMesh<T>::SkeletonAnimationEnabled(const std::string &) const
    {
      return false;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMesh<T>::UpdateSkeletonAnimation(
        std::chrono::steady_clock::duration)
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    unsigned int BaseMesh<T>::SubMeshCount() const
    {
      return this->SubMeshes()->Size();
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseMesh<T>::HasSubMesh(ConstSubMeshPtr _subMesh) const
    {
      return this->SubMeshes()->Contains(_subMesh);
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseMesh<T>::HasSubMeshName(const std::string &_name) const
    {
      return this->SubMeshes()->ContainsName(_name);
    }

    //////////////////////////////////////////////////
    template <class T>
    SubMeshPtr BaseMesh<T>::SubMeshByName(const std::string &_name) const
    {
      return this->SubMeshes()->GetByName(_name);
    }

    //////////////////////////////////////////////////
    template <class T>
    SubMeshPtr BaseMesh<T>::SubMeshByIndex(unsigned int _index) const
    {
      return this->SubMeshes()->GetByIndex(_index);
    }

    //////////////////////////////////////////////////
    template <class T>
    MaterialPtr BaseMesh<T>::Material() const
    {
      unsigned int count = this->SubMeshCount();
      return (count > 0) ? this->SubMeshByIndex(0)->Material() :
          MaterialPtr();
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMesh<T>::SetMaterial(const std::string &_name, bool _unique)
    {
      MaterialPtr mat = this->Scene()->Material(_name);
      if (mat) this->SetMaterial(mat, _unique);
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMesh<T>::SetMaterial(MaterialPtr _material, bool _unique)
    {
      // todo(anyone) take ownership of reference _material if _unique
      // and destroy the reference material when the mesh is destroyed
      unsigned int count = this->SubMeshCount();
      _material = (_unique && count > 0) ? _material->Clone() : _material;

      for (unsigned int i = 0; i < count; ++i)
      {
        SubMeshPtr subMesh = this->SubMeshByIndex(i);
        subMesh->SetMaterial(_material, false);
      }

      if (this->material && this->ownsMaterial)
        this->Scene()->DestroyMaterial(this->material);

      this->ownsMaterial = _unique;
      this->material = _material;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMesh<T>::PreRender()
    {
      unsigned int count = this->SubMeshCount();

      for (unsigned int i = 0; i < count; ++i)
      {
        SubMeshPtr subMesh =  this->SubMeshByIndex(i);
        subMesh->PreRender();
      }

      T::PreRender();
    }

    //////////////////////////////////////////////////
    template <class T>
    GeometryPtr BaseMesh<T>::Clone() const
    {
      if (!this->Scene())
      {
        ignerr << "Cloning a mesh failed because the mesh to be "
          << "cloned does not belong to a scene.\n";
        return nullptr;
      }
      else if (this->meshDescriptor.meshName.empty())
      {
        ignerr << "Cloning a geometry failed because the name of the mesh is "
          << "missing.\n";
        return nullptr;
      }

      auto result = this->Scene()->CreateMesh(this->meshDescriptor);
      if (result)
      {
        if (this->Material())
        {
          // this call will set the material for the mesh and its submeshes
          result->SetMaterial(this->Material());
        }
        else
        {
          // if the mesh doesn't have a material, clone any existing submesh
          // materials
          for (unsigned int i = 0; i < this->SubMeshCount(); ++i)
          {
            auto existingSubMeshMaterial = this->SubMeshByIndex(i)->Material();
            if (existingSubMeshMaterial)
              result->SubMeshByIndex(i)->SetMaterial(existingSubMeshMaterial);
          }
        }
      }

      return result;
    }

    //////////////////////////////////////////////////
    template <class T>
    const MeshDescriptor &BaseMesh<T>::Descriptor() const
    {
      return this->meshDescriptor;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMesh<T>::SetDescriptor(const MeshDescriptor &_desc)
    {
      this->meshDescriptor = _desc;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMesh<T>::Destroy()
    {
      T::Destroy();
      this->SubMeshes()->DestroyAll();
      if (this->material && this->ownsMaterial)
        this->Scene()->DestroyMaterial(this->material);
      this->material.reset();
      this->meshDescriptor = MeshDescriptor();
    }

    //////////////////////////////////////////////////
    // BaseSubMesh
    //////////////////////////////////////////////////
    template <class T>
    BaseSubMesh<T>::BaseSubMesh()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    BaseSubMesh<T>::~BaseSubMesh()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseSubMesh<T>::Destroy()
    {
      T::Destroy();
      if (this->material && this->ownsMaterial)
        this->Scene()->DestroyMaterial(this->material);
      this->material.reset();
    }


    //////////////////////////////////////////////////
    template <class T>
    void BaseSubMesh<T>::SetMaterial(const std::string &_name, bool _unique)
    {
      MaterialPtr mat = this->Scene()->Material(_name);
      if (mat) this->SetMaterial(mat, _unique);
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseSubMesh<T>::SetMaterial(MaterialPtr _material, bool _unique)
    {
      _material = (_unique) ? _material->Clone() : _material;

      MaterialPtr origMaterial = this->material;
      bool origUnique = this->ownsMaterial;

      this->SetMaterialImpl(_material);

      if (origMaterial && origUnique)
        this->Scene()->DestroyMaterial(origMaterial);

      this->material = _material;
      this->ownsMaterial = _unique;
    }

    //////////////////////////////////////////////////
    template <class T>
    MaterialPtr BaseSubMesh<T>::Material() const
    {
      return this->material;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseSubMesh<T>::PreRender()
    {
      T::PreRender();
      if (this->Material())
        this->Material()->PreRender();
    }
    }
  }
}
#endif
