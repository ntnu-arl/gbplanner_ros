/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <cmath>

#include <ignition/common/Mesh.hh>
#include <ignition/common/MeshManager.hh>

#include "ignition/rendering/ogre/OgreCapsule.hh"
#include "ignition/rendering/ogre/OgreMaterial.hh"
#include "ignition/rendering/ogre/OgreScene.hh"
#include "ignition/rendering/ogre/OgreMesh.hh"
#include "ignition/rendering/ogre/OgreVisual.hh"

class ignition::rendering::OgreCapsulePrivate
{
  /// \brief Capsule materal
  public: OgreMaterialPtr material{nullptr};

  /// \brief Mesh Object for capsule shape
  public: OgreMeshPtr ogreMesh{nullptr};
};

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
OgreCapsule::OgreCapsule()
  : dataPtr(new OgreCapsulePrivate)
{
}

//////////////////////////////////////////////////
OgreCapsule::~OgreCapsule() = default;

//////////////////////////////////////////////////
Ogre::MovableObject *OgreCapsule::OgreObject() const
{
  if (this->dataPtr->ogreMesh)
    return this->dataPtr->ogreMesh->OgreObject();
  else
    return nullptr;
}

//////////////////////////////////////////////////
void OgreCapsule::PreRender()
{
  if (this->capsuleDirty)
  {
    this->Update();
    this->capsuleDirty = false;
  }
}

//////////////////////////////////////////////////
void OgreCapsule::Init()
{
  this->Update();
}

//////////////////////////////////////////////////
void OgreCapsule::Destroy()
{
  if (this->dataPtr->ogreMesh)
  {
    this->dataPtr->ogreMesh->Destroy();
    this->dataPtr->ogreMesh.reset();
  }

  if (this->dataPtr->material && this->Scene())
  {
    this->Scene()->DestroyMaterial(this->dataPtr->material);
    this->dataPtr->material.reset();
  }
}

//////////////////////////////////////////////////
void OgreCapsule::Update()
{
  common::MeshManager *meshMgr = common::MeshManager::Instance();
  std::string capsuleMeshName = this->Name() + "_capsule_mesh"
    + "_" + std::to_string(this->radius)
    + "_" + std::to_string(this->length);

  // Create new mesh if needed
  if (!meshMgr->HasMesh(capsuleMeshName))
  {
    meshMgr->CreateCapsule(capsuleMeshName, this->radius, this->length, 32, 32);
  }

  MeshDescriptor meshDescriptor;
  meshDescriptor.mesh = meshMgr->MeshByName(capsuleMeshName);
  if (meshDescriptor.mesh == nullptr)
  {
    ignerr << "Capsule mesh is unavailable in the Mesh Manager" << std::endl;
    return;
  }

  auto visual = std::dynamic_pointer_cast<OgreVisual>(this->Parent());

  // clear geom if needed
  if (this->dataPtr->ogreMesh)
  {
    if (visual)
    {
      visual->RemoveGeometry(
          std::dynamic_pointer_cast<Geometry>(shared_from_this()));
    }
    this->dataPtr->ogreMesh->Destroy();
  }
  this->dataPtr->ogreMesh = std::dynamic_pointer_cast<OgreMesh>(
      this->Scene()->CreateMesh(meshDescriptor));
  if (this->dataPtr->material != nullptr)
  {
    this->dataPtr->ogreMesh->SetMaterial(this->dataPtr->material, false);
  }
  if (visual)
  {
    visual->AddGeometry(
        std::dynamic_pointer_cast<Geometry>(shared_from_this()));
  }
}

//////////////////////////////////////////////////
void OgreCapsule::SetMaterial(MaterialPtr _material, bool _unique)
{
  _material = (_unique) ? _material->Clone() : _material;

  OgreMaterialPtr derived =
      std::dynamic_pointer_cast<OgreMaterial>(_material);

  if (!derived)
  {
    ignerr << "Cannot assign material created by another render-engine"
        << std::endl;

    return;
  }

  // Set material for the underlying dynamic renderable
  this->dataPtr->ogreMesh->SetMaterial(derived, false);
  this->dataPtr->material = derived;
}

//////////////////////////////////////////////////
MaterialPtr OgreCapsule::Material() const
{
  return this->dataPtr->material;
}
