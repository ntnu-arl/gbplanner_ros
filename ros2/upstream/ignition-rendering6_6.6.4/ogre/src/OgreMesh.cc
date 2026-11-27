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

#include <gz/common/Console.hh>

#include "gz/rendering/ogre/OgreConversions.hh"
#include "gz/rendering/ogre/OgreMesh.hh"
#include "gz/rendering/ogre/OgreIncludes.hh"
#include "gz/rendering/ogre/OgreMaterial.hh"
#include "gz/rendering/ogre/OgreStorage.hh"
#include "gz/rendering/ogre/OgreRTShaderSystem.hh"


/// brief Private implementation of the OgreMesh class
class gz::rendering::OgreMeshPrivate
{
};

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
OgreMesh::OgreMesh()
  : dataPtr(new OgreMeshPrivate)
{
}

//////////////////////////////////////////////////
OgreMesh::~OgreMesh()
{
  this->Destroy();
}

//////////////////////////////////////////////////
void OgreMesh::Destroy()
{
  if (!this->ogreEntity)
    return;

  if (!this->Scene()->IsInitialized())
    return;

  BaseMesh::Destroy();

  auto ogreScene = std::dynamic_pointer_cast<OgreScene>(this->Scene());

  std::string ogreMeshName = this->ogreEntity->getMesh()->getName();

  ogreScene->OgreSceneManager()->destroyEntity(this->ogreEntity);
  this->ogreEntity = nullptr;

  auto &meshManager = Ogre::MeshManager::getSingleton();
  auto iend = meshManager.getResourceIterator().end();
  for (auto i = meshManager.getResourceIterator().begin(); i != iend;)
  {
    // A use count of 3 means that only RGM and RM have references
    // RGM has one (this one) and RM has 2 (by name and by handle)
    Ogre::Resource* res = i->second.get();
    if (i->second.useCount() == 3)
    {
      if (res->getName() == ogreMeshName)
      {
        Ogre::MeshManager::getSingleton().remove(ogreMeshName);
        break;
      }
    }
    i++;
  }
}

//////////////////////////////////////////////////
bool OgreMesh::HasSkeleton() const
{
  return this->ogreEntity->hasSkeleton();
}

//////////////////////////////////////////////////
std::map<std::string, math::Matrix4d>
        OgreMesh::SkeletonLocalTransforms() const
{
  std::map<std::string, math::Matrix4d> mapTfs;
  if (this->ogreEntity->hasSkeleton())
  {
    Ogre::SkeletonInstance *skel = this->ogreEntity->getSkeleton();
    for (unsigned int i = 0; i < skel->getNumBones(); ++i)
    {
      Ogre::Bone *bone = skel->getBone(i);
      Ogre::Quaternion quat(bone->getOrientation());
      Ogre::Vector3 p(bone->getPosition());

      math::Quaterniond tfQuat(quat.w, quat.x, quat.y, quat.z);
      math::Vector3d tfTrans(p.x, p.y, p.z);

      math::Matrix4d tf(tfQuat);
      tf.SetTranslation(tfTrans);

      mapTfs[bone->getName()] = tf;
    }
  }

  return mapTfs;
}

//////////////////////////////////////////////////
void OgreMesh::SetSkeletonLocalTransforms(
          const std::map<std::string, math::Matrix4d> &_tfs)
{
  if (!this->ogreEntity->hasSkeleton())
  {
    return;
  }

  Ogre::SkeletonInstance *skel = this->ogreEntity->getSkeleton();

  for (auto const &[boneName, tf] : _tfs)
  {
    if (skel->hasBone(boneName))
    {
      Ogre::Bone *bone = skel->getBone(boneName);
      bone->setManuallyControlled(true);
      bone->setPosition(OgreConversions::Convert(tf.Translation()));
      bone->setOrientation(OgreConversions::Convert(tf.Rotation()));
    }
  }
}

//////////////////////////////////////////////////
void OgreMesh::SetSkeletonAnimationEnabled(const std::string &_name,
    bool _enabled, bool _loop, float _weight)
{
  if (!this->ogreEntity->hasAnimationState(_name))
  {
    ignerr << "Skeleton animation name not found: " << _name << std::endl;
    return;
  }

  // disable manual control
  if (_enabled)
  {
    Ogre::SkeletonInstance *skel = this->ogreEntity->getSkeleton();
    Ogre::Skeleton::BoneIterator iter = skel->getBoneIterator();
    while (iter.hasMoreElements())
    {
      Ogre::Bone* bone = iter.getNext();
      bone->setManuallyControlled(false);
    }
  }

  // update animation state
  Ogre::AnimationState *anim = this->ogreEntity->getAnimationState(_name);
  anim->setEnabled(_enabled);
  anim->setLoop(_loop);
  anim->setWeight(_weight);
}

//////////////////////////////////////////////////
std::unordered_map<std::string, float> OgreMesh::SkeletonWeights() const
{
  std::unordered_map<std::string, float> mapWeights;
  Ogre::SkeletonInstance *skel = this->ogreEntity->getSkeleton();
  if (!skel)
    return mapWeights;

  auto animations = this->ogreEntity->getAllAnimationStates();
  if (!animations)
    return mapWeights;

  auto animIt = animations->getAnimationStateIterator();

  // todo(anyone) support different bone weight per animation?
  // currently assume all skeletal animations have same bone weights
  if (animIt.hasMoreElements())
  {
    Ogre::AnimationState *anim = animIt.getNext();
    if (!anim->hasBlendMask())
      anim->createBlendMask(skel->getNumBones());

    Ogre::Skeleton::BoneIterator iter = skel->getBoneIterator();
    while (iter.hasMoreElements())
    {
      Ogre::Bone* bone = iter.getNext();
      mapWeights[bone->getName()] =
          anim->getBlendMaskEntry(bone->getHandle());
    }
  }

  return mapWeights;
}

//////////////////////////////////////////////////
void OgreMesh::SetSkeletonWeights(
    const std::unordered_map<std::string, float> &_weights)
{
  Ogre::SkeletonInstance *skel = this->ogreEntity->getSkeleton();
  if (!skel)
    return;

  auto animations = this->ogreEntity->getAllAnimationStates();
  if (!animations)
    return;

  // set bone weights for all animations
  auto animIt = animations->getAnimationStateIterator();
  while (animIt.hasMoreElements())
  {
    Ogre::AnimationState *anim = animIt.getNext();
    if (!anim->hasBlendMask())
      anim->createBlendMask(skel->getNumBones());

    for (auto const &[boneName, weight] : _weights)
    {
      Ogre::Bone *bone = skel->getBone(boneName);
      if (bone)
        anim->setBlendMaskEntry(bone->getHandle(), weight);
    }
  }
}

//////////////////////////////////////////////////
bool OgreMesh::SkeletonAnimationEnabled(const std::string &_name) const
{
  if (!this->ogreEntity->hasAnimationState(_name))
  {
    ignerr << "Skeleton animation name not found: " << _name << std::endl;
    return false;
  }

  Ogre::AnimationState *anim = this->ogreEntity->getAnimationState(_name);
  return anim->getEnabled();
}


//////////////////////////////////////////////////
void OgreMesh::UpdateSkeletonAnimation(
    std::chrono::steady_clock::duration _time)
{
  Ogre::AnimationStateSet *animationStateSet =
      this->ogreEntity->getAllAnimationStates();

  auto it = animationStateSet->getAnimationStateIterator();
  while (it.hasMoreElements())
  {
    Ogre::AnimationState *anim = it.getNext();
    if (anim->getEnabled())
    {
      auto seconds =
          std::chrono::duration_cast<std::chrono::milliseconds>(_time).count() /
          1000.0;
      anim->setTimePosition(seconds);
    }
  }

  // this workaround is needed for ogre 1.x because we are doing manual
  // render updates.
  // see https://forums.ogre3d.org/viewtopic.php?t=33448
  Ogre::SkeletonInstance *skel = this->ogreEntity->getSkeleton();
  skel->setAnimationState(*this->ogreEntity->getAllAnimationStates());
  skel->_notifyManualBonesDirty();
}

//////////////////////////////////////////////////
Ogre::MovableObject *OgreMesh::OgreObject() const
{
  return this->ogreEntity;
}

//////////////////////////////////////////////////
SubMeshStorePtr OgreMesh::SubMeshes() const
{
  return this->subMeshes;
}

//////////////////////////////////////////////////
OgreSubMesh::OgreSubMesh()
{
}

//////////////////////////////////////////////////
OgreSubMesh::~OgreSubMesh()
{
  this->Destroy();
}

//////////////////////////////////////////////////
Ogre::SubEntity *OgreSubMesh::OgreSubEntity() const
{
  return this->ogreSubEntity;
}

//////////////////////////////////////////////////
void OgreSubMesh::Destroy()
{
  // this causes several tests to crash on exit because MaterialManager
  // is already deleted before this call
  // todo(anyone) check if ogre root is deleted before removing material?
  // Ogre::MaterialManager::getSingleton().remove(
  //       this->ogreSubEntity->getMaterialName());
  OgreRTShaderSystem::Instance()->DetachEntity(this);

  BaseSubMesh::Destroy();
}

//////////////////////////////////////////////////
void OgreSubMesh::SetMaterialImpl(MaterialPtr _material)
{
  OgreMaterialPtr derived =
      std::dynamic_pointer_cast<OgreMaterial>(_material);

  if (!derived)
  {
    ignerr << "Cannot assign material created by another render-engine"
        << std::endl;

    return;
  }

  std::string materialName = derived->Name();
  Ogre::MaterialPtr ogreMaterial = derived->Material();
  this->ogreSubEntity->setMaterialName(materialName);

  // set cast shadows
  this->ogreSubEntity->getParent()->setCastShadows(_material->CastShadows());
}

//////////////////////////////////////////////////
void OgreSubMesh::Init()
{
  BaseSubMesh::Init();
  OgreRTShaderSystem::Instance()->AttachEntity(this);
}

//////////////////////////////////////////////////
