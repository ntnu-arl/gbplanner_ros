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

// Note this include is placed in the src file because
// otherwise ogre produces compile errors
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable: 4005)  // Macro redefinition
#pragma warning(disable: 5033)  // 'register' is no longer supported
#endif
#include <Animation/OgreSkeletonInstance.h>
#include <Hlms/Pbs/OgreHlmsPbsDatablock.h>
#include <OgreItem.h>
#include <OgreSceneManager.h>
#include <OgreMeshManager.h>
#include <OgreMeshManager2.h>
#include <OgreMaterialManager.h>
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#include <gz/common/Console.hh>

#include "gz/rendering/ogre2/Ogre2Conversions.hh"
#include "gz/rendering/ogre2/Ogre2Mesh.hh"
#include "gz/rendering/ogre2/Ogre2Material.hh"
#include "gz/rendering/ogre2/Ogre2Storage.hh"

/// brief Private implementation of the Ogre2Mesh class
class gz::rendering::Ogre2MeshPrivate
{
};

/// brief Private implementation of the Ogre2SubMesh class
class gz::rendering::Ogre2SubMeshPrivate
{
  /// \brief name of the mesh inside the mesh manager to be able to
  /// remove it
  public: std::string subMeshName;
};

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
Ogre2Mesh::Ogre2Mesh()
  : dataPtr(new Ogre2MeshPrivate)
{
}

//////////////////////////////////////////////////
Ogre2Mesh::~Ogre2Mesh()
{
  this->Destroy();
}

//////////////////////////////////////////////////
void Ogre2Mesh::Destroy()
{
  if (!this->ogreItem || !this->Scene()->IsInitialized())
    return;

  // We need to override BaseMesh::Destroy for ogre2 implementation to control
  // the order in which ogre items and materials are destroyed.
  // Items must be destroyed before materials otherwise ogre throws an exception
  // when unlinking an renderable from a hlms datablock

  // Remove this object from parent
  BaseGeometry::Destroy();

  // destroy mesh (ogre item)
  auto ogreScene = std::dynamic_pointer_cast<Ogre2Scene>(this->Scene());
  ogreScene->OgreSceneManager()->destroyItem(this->ogreItem);
  this->ogreItem = nullptr;

  // destroy submeshes (ogre subitems)
  this->SubMeshes()->DestroyAll();

  // destroy material (ogre hlms datablock) - this needs to be done last!
  if (this->material && this->ownsMaterial)
    this->Scene()->DestroyMaterial(this->material);
  this->material.reset();
}

//////////////////////////////////////////////////
bool Ogre2Mesh::HasSkeleton() const
{
  return this->ogreItem->hasSkeleton();
}

//////////////////////////////////////////////////
std::map<std::string, math::Matrix4d> Ogre2Mesh::SkeletonLocalTransforms() const
{
  std::map<std::string, math::Matrix4d> mapTfs;
  if (this->ogreItem->hasSkeleton())
  {
    auto skel = this->ogreItem->getSkeletonInstance();
    for (unsigned int i = 0; i < skel->getNumBones(); ++i)
    {
      auto bone = skel->getBone(i);

      math::Matrix4d tf(Ogre2Conversions::Convert(bone->getOrientation()));
      tf.SetTranslation(Ogre2Conversions::Convert(bone->getPosition()));

      mapTfs[bone->getName()] = tf;
    }
  }

  return mapTfs;
}

//////////////////////////////////////////////////
void Ogre2Mesh::SetSkeletonLocalTransforms(
          const std::map<std::string, math::Matrix4d> &_tfs)
{
  if (!this->ogreItem->hasSkeleton())
  {
    return;
  }
  auto skel = this->ogreItem->getSkeletonInstance();

  for (auto const &[boneName, tf] : _tfs)
  {
    auto bone = skel->getBone(boneName);
    if (bone)
    {
      skel->setManualBone(bone, true);
      bone->setPosition(Ogre2Conversions::Convert(tf.Translation()));
      bone->setOrientation(Ogre2Conversions::Convert(tf.Rotation()));
    }
  }
}

//////////////////////////////////////////////////
std::unordered_map<std::string, float> Ogre2Mesh::SkeletonWeights() const
{
  std::unordered_map<std::string, float> mapWeights;
  if (!this->ogreItem->hasSkeleton())
    return mapWeights;

  auto skel = this->ogreItem->getSkeletonInstance();

  auto animations = skel->getAnimations();
  if (animations.empty())
    return mapWeights;

  // todo(anyone) support different bone weight per animation?
  // currently assume all skeletal animations have same bone weights
  Ogre::SkeletonAnimation &anim = *animations.begin();
  for (unsigned int i = 0; i < skel->getNumBones(); ++i)
  {
    auto bone = skel->getBone(i);
    float weight = anim.getBoneWeight(bone->getName());
    mapWeights[bone->getName()] = weight;
  }

  return mapWeights;
}

//////////////////////////////////////////////////
void Ogre2Mesh::SetSkeletonWeights(
    const std::unordered_map<std::string, float> &_weights)
{
  if (!this->ogreItem->hasSkeleton())
    return;

  auto skel = this->ogreItem->getSkeletonInstance();

  // set bone weights for all animations
  auto &animations = skel->getAnimations();
  for (auto &anim : animations)
  {
    for (auto const &[boneName, weight] : _weights)
    {
      if (skel->getBone(boneName))
      {
        Ogre::SkeletonAnimation *animPtr = skel->getAnimation(anim.getName());
        if (animPtr)
          animPtr->setBoneWeight(boneName, weight);
      }
    }
  }
}

//////////////////////////////////////////////////
void Ogre2Mesh::SetSkeletonAnimationEnabled(const std::string &_name,
    bool _enabled, bool _loop, float _weight)
{
  if (!this->ogreItem->hasSkeleton())
  {
    return;
  }

  Ogre::SkeletonInstance *skel = this->ogreItem->getSkeletonInstance();

  if (!skel->hasAnimation(_name))
  {
    ignerr << "Skeleton animation name not found: " << _name << std::endl;
    return;
  }

  if (_enabled)
  {
    for (unsigned int i = 0; i < skel->getNumBones(); ++i)
    {
      auto bone = skel->getBone(i);
      skel->setManualBone(bone, false);
    }
  }

  Ogre::SkeletonAnimation *anim = skel->getAnimation(_name);
  anim->setEnabled(_enabled);
  anim->setLoop(_loop);
  anim->mWeight = _weight;
}

//////////////////////////////////////////////////
void Ogre2Mesh::UpdateSkeletonAnimation(
    std::chrono::steady_clock::duration _time)
{
  if (!this->ogreItem->hasSkeleton())
  {
    return;
  }

  Ogre::SkeletonInstance *skel = this->ogreItem->getSkeletonInstance();
  auto animations = skel->getAnimations();
  for (auto &anim : animations)
  {
    Ogre::SkeletonAnimation *sa = skel->getAnimation(anim.getName());
    if (sa->getEnabled())
    {
      auto seconds =
          std::chrono::duration_cast<std::chrono::milliseconds>(_time).count() /
          1000.0;
      sa->setTime(seconds);
    }
  }
}

//////////////////////////////////////////////////
bool Ogre2Mesh::SkeletonAnimationEnabled(const std::string &_name) const
{
  if (!this->ogreItem->hasSkeleton())
  {
    return false;
  }

  Ogre::SkeletonInstance *skel = this->ogreItem->getSkeletonInstance();
  if (!skel->hasAnimation(_name))
  {
    ignerr << "Skeleton animation name not found: " << _name << std::endl;
    return false;
  }

  Ogre::SkeletonAnimation *anim = skel->getAnimation(_name);
  return anim->getEnabled();
}

//////////////////////////////////////////////////
Ogre::MovableObject *Ogre2Mesh::OgreObject() const
{
  return this->ogreItem;
}

//////////////////////////////////////////////////
SubMeshStorePtr Ogre2Mesh::SubMeshes() const
{
  return this->subMeshes;
}

//////////////////////////////////////////////////
Ogre2SubMesh::Ogre2SubMesh()
  : dataPtr(new Ogre2SubMeshPrivate)
{
}

//////////////////////////////////////////////////
Ogre2SubMesh::~Ogre2SubMesh()
{
  this->Destroy();
}

//////////////////////////////////////////////////
void Ogre2SubMesh::SetMeshName(const std::string &_name)
{
  this->dataPtr->subMeshName = _name;
}

//////////////////////////////////////////////////
void Ogre2SubMesh::Destroy()
{
  auto meshManager = Ogre::MeshManager::getSingletonPtr();
  if (meshManager)
  {
    auto iend = meshManager->getResourceIterator().end();
    for (auto i = meshManager->getResourceIterator().begin(); i != iend;)
    {
      // A use count of 3 means that only RGM and RM have
      // references RGM has one (this one) and RM has 2 (by name and by handle)
      Ogre::Resource* res = i->second.get();
      if (i->second.useCount() == 3)
      {
        if (res->getName() == this->dataPtr->subMeshName)
        {
          Ogre::v1::MeshManager::getSingleton().remove(
            this->dataPtr->subMeshName);
          Ogre::MeshManager::getSingleton().remove(this->dataPtr->subMeshName);
          break;
        }
      }
      ++i;
    }
  }
  BaseSubMesh::Destroy();
}

//////////////////////////////////////////////////
Ogre::SubItem *Ogre2SubMesh::Ogre2SubItem() const
{
  return this->ogreSubItem;
}

//////////////////////////////////////////////////
void Ogre2SubMesh::SetMaterialImpl(MaterialPtr _material)
{
  Ogre2MaterialPtr derived =
      std::dynamic_pointer_cast<Ogre2Material>(_material);

  if (!derived)
  {
    ignerr << "Cannot assign material created by another render-engine"
        << std::endl;

    return;
  }

  // low level material with custom shaders
  if (!derived->FragmentShader().empty() && !derived->VertexShader().empty())
  {
    this->ogreSubItem->setMaterial(derived->Material());
  }
  // Pbs Hlms material
  else
  {
    auto datablock =
        static_cast<Ogre::HlmsPbsDatablock *>(derived->Datablock());
    if (datablock)
    {
      this->ogreSubItem->setDatablock(datablock);

      // update render queue group based on material transparency setting
      if (datablock->getTransparencyMode() == Ogre::HlmsPbsDatablock::None)
      {
        // by default, ogre items are in render queue 10
        // these are hardcoded in ogre-next and there does not seem to be
        // an enum of function to retrieve this default render queue group
        this->ogreSubItem->getParent()->setRenderQueueGroup(10);
      }
      else
      {
        // put in render queue group 200
        // v2 entities can be placed in groups 0-99 or 200-224
        this->ogreSubItem->getParent()->setRenderQueueGroup(200);
      }
    }
  }

  // set cast shadows
  this->ogreSubItem->getParent()->setCastShadows(_material->CastShadows());
}

//////////////////////////////////////////////////
void Ogre2SubMesh::Init()
{
  BaseSubMesh::Init();
}

//////////////////////////////////////////////////
