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


#include <sstream>

#include <gz/common/Console.hh>
#include <gz/common/Material.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/Skeleton.hh>
#include <gz/common/SkeletonAnimation.hh>
#include <gz/common/SubMesh.hh>

#include <gz/math/Matrix4.hh>

#include "gz/rendering/ogre/OgreConversions.hh"
#include "gz/rendering/ogre/OgreIncludes.hh"
#include "gz/rendering/ogre/OgreMesh.hh"
#include "gz/rendering/ogre/OgreMeshFactory.hh"
#include "gz/rendering/ogre/OgreRenderEngine.hh"
#include "gz/rendering/ogre/OgreScene.hh"
#include "gz/rendering/ogre/OgreStorage.hh"

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
OgreMeshFactory::OgreMeshFactory(OgreScenePtr _scene) :
  scene(_scene)
{
}

//////////////////////////////////////////////////
OgreMeshFactory::~OgreMeshFactory()
{
}

//////////////////////////////////////////////////
OgreMeshPtr OgreMeshFactory::Create(const MeshDescriptor &_desc)
{
  // create ogre entity
  OgreMeshPtr mesh(new OgreMesh);
  MeshDescriptor normDesc = _desc;
  normDesc.Load();
  mesh->ogreEntity = this->OgreEntity(normDesc);

  // check if invalid mesh
  if (!mesh->ogreEntity)
  {
    return nullptr;
  }

  // create sub-mesh store
  OgreSubMeshStoreFactory subMeshFactory(this->scene, mesh->ogreEntity);
  mesh->subMeshes = subMeshFactory.Create();
  return mesh;
}

//////////////////////////////////////////////////
Ogre::Entity *OgreMeshFactory::OgreEntity(const MeshDescriptor &_desc)
{
  if (!this->Load(_desc))
  {
    return nullptr;
  }

  std::string name = this->MeshName(_desc);
  Ogre::SceneManager *sceneManager = this->scene->OgreSceneManager();
  return sceneManager->createEntity(name);
}

//////////////////////////////////////////////////
bool OgreMeshFactory::Load(const MeshDescriptor &_desc)
{
  if (!this->Validate(_desc))
  {
    return false;
  }

  if (this->IsLoaded(_desc))
  {
    return true;
  }

  return this->LoadImpl(_desc);
}

//////////////////////////////////////////////////
bool OgreMeshFactory::IsLoaded(const MeshDescriptor &_desc)
{
  std::string name = this->MeshName(_desc);
  return Ogre::MeshManager::getSingleton().resourceExists(name);
}

//////////////////////////////////////////////////
void OgreMeshFactory::ClearMaterialsCache(const std::string &_name)
{
  auto it = this->materialCache.begin();
  for (auto &mat : this->materialCache)
  {
    std::string matName = mat->Name();
    std::string textureName = mat->Texture();
    if (textureName == _name)
    {
      this->scene->UnregisterMaterial(matName);
      break;
    }
    ++it;
  }
  if (it != this->materialCache.end())
    this->materialCache.erase(it);
}

//////////////////////////////////////////////////
bool OgreMeshFactory::LoadImpl(const MeshDescriptor &_desc)
{
  Ogre::MeshPtr ogreMesh;
  std::string name;
  std::string group;

  OgreRenderEngine::Instance()->AddResourcePath(_desc.mesh->Path());

  try
  {
    name = this->MeshName(_desc);
    group = Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME;
    ogreMesh = Ogre::MeshManager::getSingleton().createManual(name, group);

    // load skeleton
    Ogre::SkeletonPtr ogreSkeleton;
    if (_desc.mesh->HasSkeleton())
    {
      common::SkeletonPtr skel = _desc.mesh->MeshSkeleton();
      ogreSkeleton = Ogre::SkeletonManager::getSingleton().create(
        _desc.mesh->Name() + "_skeleton",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        true);

      // load bones
      for (unsigned int i = 0; i < skel->NodeCount(); i++)
      {
        common::SkeletonNode *node = skel->NodeByHandle(i);
        Ogre::Bone *bone = ogreSkeleton->createBone(node->Name());

        if (node->Parent())
          ogreSkeleton->getBone(node->Parent()->Name())->addChild(bone);

        math::Matrix4d trans = node->Transform();
        math::Vector3d pos = trans.Translation();
        math::Quaterniond q = trans.Rotation();
        bone->setPosition(Ogre::Vector3(pos.X(), pos.Y(), pos.Z()));
        bone->setOrientation(Ogre::Quaternion(q.W(), q.X(), q.Y(), q.Z()));
        bone->setInheritOrientation(true);
        bone->setManuallyControlled(true);
        bone->setInitialState();
      }

      // load skeletal animations
      for (unsigned int i = 0; i < skel->AnimationCount(); ++i)
      {
        common::SkeletonAnimation *skelAnim = skel->Animation(i);
        if (ogreSkeleton->hasAnimation(skelAnim->Name()))
        {
          continue;
        }

        Ogre::Animation *ogreAnim = ogreSkeleton->createAnimation(
            skelAnim->Name(), skelAnim->Length());

        for (unsigned int j = 0; j < skel->NodeCount(); ++j)
        {
          common::SkeletonNode *node = skel->NodeByHandle(j);
          common::NodeAnimation *nodeAnim = skelAnim->NodeAnimationByName(
              node->Name());
          if (!nodeAnim)
          {
            continue;
          }
          Ogre::Bone *bone = ogreSkeleton->getBone(node->Name());
          Ogre::NodeAnimationTrack *ogreNodeAnimTrack =
              ogreAnim->createNodeTrack(j, bone);

          // set up transform (needed for bvh)
          math::Matrix4d alignTrans = skel->AlignTranslation(i, node->Name());
          math::Matrix4d alignRot = skel->AlignRotation(i, node->Name());

          for (unsigned int k = 0; k < nodeAnim->FrameCount(); ++k)
          {
            std::pair<double, math::Matrix4d> keyFrame = nodeAnim->KeyFrame(k);
            Ogre::TransformKeyFrame *kf =
                ogreNodeAnimTrack->createNodeKeyFrame(keyFrame.first);

            math::Matrix4d p = keyFrame.second;

            // apply anim-skin transform
            p = alignTrans * p * alignRot;

            auto rot = bone->getOrientation().Inverse() *
                OgreConversions::Convert(p.Rotation());
            auto pos = OgreConversions::Convert(p.Translation()) -
                bone->getPosition();
            kf->setRotation(rot);
            kf->setTranslate(pos);
          }
        }
      }

      ogreMesh->setSkeletonName(_desc.mesh->Name() + "_skeleton");
    }

    // load submeshes
    for (unsigned int i = 0; i < _desc.mesh->SubMeshCount(); i++)
    {
      auto s = _desc.mesh->SubMeshByIndex(i).lock();
      if (!_desc.subMeshName.empty() && s &&
          s->Name() != _desc.subMeshName)
      {
        continue;
      }

      Ogre::SubMesh *ogreSubMesh;
      Ogre::VertexData *vertexData;
      Ogre::VertexDeclaration* vertexDecl;
      Ogre::HardwareVertexBufferSharedPtr vBuf;
      Ogre::HardwareIndexBufferSharedPtr iBuf;
      Ogre::HardwareVertexBufferSharedPtr texBuf;
      float *vertices{nullptr};
      float *texMappings{nullptr};
      uint32_t *indices{nullptr};

      size_t currOffset = 0;

      // Copy the original submesh. We may need to modify the vertices, and
      // we don't want to change the original.
      common::SubMesh subMesh(*s.get());

      // Recenter the vertices if requested.
      if (_desc.centerSubMesh)
        subMesh.Center(math::Vector3d::Zero);

      ogreSubMesh = ogreMesh->createSubMesh();
      ogreSubMesh->useSharedVertices = false;
      if (subMesh.SubMeshPrimitiveType() == common::SubMesh::TRIANGLES)
        ogreSubMesh->operationType = Ogre::RenderOperation::OT_TRIANGLE_LIST;
      else if (subMesh.SubMeshPrimitiveType() == common::SubMesh::LINES)
        ogreSubMesh->operationType = Ogre::RenderOperation::OT_LINE_LIST;
      else if (subMesh.SubMeshPrimitiveType() == common::SubMesh::LINESTRIPS)
        ogreSubMesh->operationType = Ogre::RenderOperation::OT_LINE_STRIP;
      else if (subMesh.SubMeshPrimitiveType() == common::SubMesh::TRIFANS)
        ogreSubMesh->operationType = Ogre::RenderOperation::OT_TRIANGLE_FAN;
      else if (subMesh.SubMeshPrimitiveType() == common::SubMesh::TRISTRIPS)
        ogreSubMesh->operationType = Ogre::RenderOperation::OT_TRIANGLE_STRIP;
      else if (subMesh.SubMeshPrimitiveType() == common::SubMesh::POINTS)
        ogreSubMesh->operationType = Ogre::RenderOperation::OT_POINT_LIST;
      else
        ignerr << "Unknown primitive type["
              << subMesh.SubMeshPrimitiveType() << "]\n";

      ogreSubMesh->vertexData = new Ogre::VertexData();
      vertexData = ogreSubMesh->vertexData;
      vertexDecl = vertexData->vertexDeclaration;

      // The vertexDecl should contain positions, blending weights, normals,
      // diffiuse colors, specular colors, tex coords. In that order.
      vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3,
                             Ogre::VES_POSITION);
      currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

      // TODO(anyone): blending weights

      // normals
      if (subMesh.NormalCount() > 0)
      {
        vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3,
                               Ogre::VES_NORMAL);
        currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
      }

      // TODO(anyone): diffuse colors

      // TODO(anyone): specular colors

      // two dimensional texture coordinates
      // allocate buffer for texture mapping, when doing animations, OGRE
      // requires the vertex position and normals reside in their own buffer,
      // see `https://ogrecave.github.io/ogre/api/1.11/_animation.html` under,
      // `Vertex buffer arrangements`.
      currOffset = 0;
      if (subMesh.TexCoordCountBySet(0) > 0)
      {
        vertexDecl->addElement(1, currOffset, Ogre::VET_FLOAT2,
            Ogre::VES_TEXTURE_COORDINATES, 0);
        currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT2);
      }

      // allocate the vertex buffer
      vertexData->vertexCount = subMesh.VertexCount();

      vBuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
                 vertexDecl->getVertexSize(0),
                 vertexData->vertexCount,
                 Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
                 false);

      if (subMesh.TexCoordCountBySet(0) > 0)
      {
        texBuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
            vertexDecl->getVertexSize(1),
            vertexData->vertexCount,
            Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
            false);
      }

      vertexData->vertexBufferBinding->setBinding(0, vBuf);
      vertices = static_cast<float *>(vBuf->lock(
                      Ogre::HardwareBuffer::HBL_DISCARD));

      if (subMesh.TexCoordCountBySet(0) > 0)
      {
        vertexData->vertexBufferBinding->setBinding(1, texBuf);
        texMappings = static_cast<float *>(texBuf->lock(
                        Ogre::HardwareBuffer::HBL_DISCARD));
      }

      if (_desc.mesh->HasSkeleton())
      {
        if (subMesh.NodeAssignmentsCount())
        {
          common::SkeletonPtr skel = _desc.mesh->MeshSkeleton();
          for (unsigned int j = 0; j < subMesh.NodeAssignmentsCount(); j++)
          {
            common::NodeAssignment na = subMesh.NodeAssignmentByIndex(j);
            Ogre::VertexBoneAssignment vba;
            vba.vertexIndex = na.vertexIndex;
            vba.boneIndex = ogreSkeleton->getBone(skel->NodeByHandle(
                                na.nodeIndex)->Name())->getHandle();
            vba.weight = na.weight;
            ogreSubMesh->addBoneAssignment(vba);
          }
        }
        else
        {
          // When there is a skeleton associated with the mesh,
          // Ogre requires at least 1 bone assignment to compile the blend
          // weights.
          // The submeshes laoded from COLLADA may not have weights so we need
          // to add a dummy bone assignment for OGRE
          Ogre::VertexBoneAssignment vba;
          vba.vertexIndex = 0;
          vba.boneIndex = 0;
          vba.weight = 0;
          ogreSubMesh->addBoneAssignment(vba);
        }
      }

      // allocate index buffer
      ogreSubMesh->indexData->indexCount = subMesh.IndexCount();

      ogreSubMesh->indexData->indexBuffer =
        Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
            Ogre::HardwareIndexBuffer::IT_32BIT,
            ogreSubMesh->indexData->indexCount,
            Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
            false);

      iBuf = ogreSubMesh->indexData->indexBuffer;
      indices = static_cast<uint32_t*>(
          iBuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

      unsigned int j;

      // Add all the vertices
      for (j = 0; j < subMesh.VertexCount(); j++)
      {
        *vertices++ = subMesh.Vertex(j).X();
        *vertices++ = subMesh.Vertex(j).Y();
        *vertices++ = subMesh.Vertex(j).Z();

        if (subMesh.NormalCount() > 0)
        {
          *vertices++ = subMesh.Normal(j).X();
          *vertices++ = subMesh.Normal(j).Y();
          *vertices++ = subMesh.Normal(j).Z();
        }

        if (subMesh.TexCoordCountBySet(0) > 0)
        {
          *texMappings++ = subMesh.TexCoordBySet(j, 0).X();
          *texMappings++ = subMesh.TexCoordBySet(j, 0).Y();
        }
      }

      // Add all the indices
      for (j = 0; j < subMesh.IndexCount(); j++)
        *indices++ = subMesh.Index(j);

      common::MaterialPtr material;
      material = _desc.mesh->MaterialByIndex(subMesh.MaterialIndex());

      MaterialPtr mat = this->scene->CreateMaterial();
      if (material)
      {
        mat->CopyFrom(*material);
        materialCache.push_back(mat);
      }
      else
      {
        MaterialPtr defaultMat = this->scene->Material("Default/White");
        if (defaultMat != nullptr)
          mat->CopyFrom(defaultMat);
      }
      ogreSubMesh->setMaterialName(mat->Name());

      // Unlock
      vBuf->unlock();
      iBuf->unlock();
      if (subMesh.TexCoordCount() > 0)
      {
        texBuf->unlock();
      }
    }

    math::Vector3d max = _desc.mesh->Max();
    math::Vector3d min = _desc.mesh->Min();

    if (_desc.mesh->HasSkeleton())
    {
      min = math::Vector3d(-1, -1, -1);
      max = math::Vector3d(1, 1, 1);
    }

    if (!max.IsFinite())
    {
      ignerr << "Max bounding box is not finite[" << max << "]" << std::endl;
      return false;
    }

    if (!min.IsFinite())
    {
      ignerr << "Min bounding box is not finite[" << min << "]" << std::endl;
      return false;
    }

    ogreMesh->_setBounds(Ogre::AxisAlignedBox(
          Ogre::Vector3(min.X(), min.Y(), min.Z()),
          Ogre::Vector3(max.X(), max.Y(), max.Z())),
          false);

    // this line makes clear the mesh is loaded (avoids memory leaks)
    ogreMesh->load();
  }
  catch(Ogre::Exception &e)
  {
    ignerr << "Unable to insert mesh[" << e.getDescription() << "]"
        << std::endl;
    return false;
  }

  if (ogreMesh->getNumSubMeshes() == 0u)
  {
    std::string msg = "Unable to load mesh: '" + _desc.meshName + "'";
    if (!_desc.subMeshName.empty())
      msg += ", submesh: '" + _desc.subMeshName + "'";
    msg += ". Mesh will be empty.";
    ignwarn << msg << std::endl;
  }

  return true;
}

//////////////////////////////////////////////////
std::string OgreMeshFactory::MeshName(const MeshDescriptor &_desc)
{
  std::stringstream ss;
  ss << _desc.meshName << "::";
  ss << _desc.subMeshName << "::";
  ss << ((_desc.centerSubMesh) ? "CENTERED" : "ORIGINAL");
  return ss.str();
}

//////////////////////////////////////////////////
bool OgreMeshFactory::Validate(const MeshDescriptor &_desc)
{
  if (!_desc.mesh && _desc.meshName.empty())
  {
    ignerr << "Invalid mesh-descriptor, no mesh specified" << std::endl;
    return false;
  }

  if (!_desc.mesh)
  {
    ignerr << "Cannot load null mesh" << std::endl;
    return false;
  }

  if (_desc.mesh->SubMeshCount() == 0)
  {
    ignerr << "Cannot load mesh with zero sub-meshes" << std::endl;
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
OgreSubMeshStoreFactory::OgreSubMeshStoreFactory(OgreScenePtr _scene,
    Ogre::Entity *_entity) :
  scene(_scene),
  ogreEntity(_entity)
{
  this->CreateNameList();
}

//////////////////////////////////////////////////
OgreSubMeshStoreFactory::~OgreSubMeshStoreFactory()
{
}

//////////////////////////////////////////////////
OgreSubMeshStorePtr OgreSubMeshStoreFactory::Create()
{
  OgreSubMeshStorePtr subMeshes(new OgreSubMeshStore);
  unsigned int count = this->ogreEntity->getNumSubEntities();

  for (unsigned int i = 0; i < count; ++i)
  {
    OgreSubMeshPtr subMesh = this->CreateSubMesh(i);
    subMeshes->Add(subMesh);
  }

  return subMeshes;
}

//////////////////////////////////////////////////
OgreSubMeshPtr OgreSubMeshStoreFactory::CreateSubMesh(unsigned int _index)
{
  OgreSubMeshPtr subMesh(new OgreSubMesh);

  subMesh->id = _index;
  subMesh->name = this->names[_index];
  subMesh->scene = this->scene;
  subMesh->ogreSubEntity = this->ogreEntity->getSubEntity(_index);

  MaterialPtr mat = this->scene->Material(
      subMesh->ogreSubEntity->getMaterialName());
  if (mat)
  {
    // assign material to submesh who will make a copy of this material
    subMesh->SetMaterial(mat);
  }

  subMesh->Load();
  subMesh->Init();

  return subMesh;
}

//////////////////////////////////////////////////
void OgreSubMeshStoreFactory::CreateNameList()
{
  this->PopulateDefaultNames();
  this->PopulateGivenNames();
}

//////////////////////////////////////////////////
void OgreSubMeshStoreFactory::PopulateDefaultNames()
{
  unsigned int count = this->ogreEntity->getNumSubEntities();
  this->names.reserve(count);

  for (unsigned int i = 0; i < count; ++i)
  {
    this->names.push_back("SubMesh(" + std::to_string(i) + ")");
  }
}

//////////////////////////////////////////////////
void OgreSubMeshStoreFactory::PopulateGivenNames()
{
  const Ogre::MeshPtr ogreMesh = this->ogreEntity->getMesh();
  const Ogre::Mesh::SubMeshNameMap &ogreMap = ogreMesh->getSubMeshNameMap();

  for (auto pair : ogreMap)
  {
    std::string name = pair.first;
    unsigned int index = pair.second;
    this->names[index] = name;
  }
}
