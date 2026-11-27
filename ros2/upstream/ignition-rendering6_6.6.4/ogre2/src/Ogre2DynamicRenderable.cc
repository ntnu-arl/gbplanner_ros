/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#pragma warning(disable: 5033)
#endif
#include <Hlms/Pbs/OgreHlmsPbsDatablock.h>
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#include "gz/common/Console.hh"
#include "gz/rendering/ogre2/Ogre2Conversions.hh"
#include "gz/rendering/ogre2/Ogre2DynamicRenderable.hh"
#include "gz/rendering/ogre2/Ogre2Material.hh"
#include "gz/rendering/ogre2/Ogre2RenderEngine.hh"
#include "gz/rendering/ogre2/Ogre2Scene.hh"

#ifdef _MSC_VER
  #pragma warning(push, 0)
#endif
#include <OgreItem.h>
#include <OgreMesh2.h>
#include <OgreMeshManager2.h>
#include <OgreSceneManager.h>
#include <OgreSubMesh2.h>
#include <Vao/OgreVaoManager.h>
#ifdef _MSC_VER
  #pragma warning(pop)
#endif

/// \brief Private implementation
class gz::rendering::Ogre2DynamicRenderablePrivate
{
  /// \brief list of colors at each point
  public: std::vector<gz::math::Color> colors;

  /// \brief List of vertices for the mesh
  public: std::vector<gz::math::Vector3d> vertices;

  /// \brief Used to indicate if the lines require an update
  public: bool dirty = false;

  /// \brief Render operation type
  public: Ogre::OperationType operationType;

  /// \brief Ogre submesh
  public: Ogre::SubMesh *subMesh = nullptr;

  /// \brief Ogre vertex buffer data structure
  public: Ogre::VertexBufferPacked *vertexBuffer = nullptr;

  /// \brief Ogre vertex array object which binds the index and vertex buffers
  public: Ogre::VertexArrayObject *vao = nullptr;

  /// \brief Ogre item created from the dynamic geometry
  public: Ogre::Item *ogreItem = nullptr;

  /// \brief raw vertex buffer
  public: float *vbuffer = nullptr;

  /// \brief Maximum capacity of the currently allocated vertex buffer.
  public: size_t vertexBufferCapacity = 0;

  /// \brief Pointer to the dynamic renderable's material
  public: Ogre2MaterialPtr material;

  /// \brief Flag to indicate whether or not this mesh should be
  /// responsible for destroying the material
  public: bool ownsMaterial = false;

  /// \brief Pointer to scene
  public: ScenePtr scene;

  /// \brief Pointer to the ogre scene manager
  public: Ogre::SceneManager *sceneManager = nullptr;
};


using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
Ogre2DynamicRenderable::Ogre2DynamicRenderable(
    ScenePtr _scene)
    : dataPtr(new Ogre2DynamicRenderablePrivate)
{
  this->dataPtr->scene = _scene;

  Ogre2ScenePtr s = std::dynamic_pointer_cast<Ogre2Scene>(this->dataPtr->scene);
  this->dataPtr->sceneManager = s->OgreSceneManager();

  this->SetOperationType(MT_LINE_STRIP);
  this->CreateDynamicMesh();
}

//////////////////////////////////////////////////
Ogre2DynamicRenderable::~Ogre2DynamicRenderable()
{
  this->Destroy();
}

//////////////////////////////////////////////////
void Ogre2DynamicRenderable::Destroy()
{
  if (!this->dataPtr->scene->IsInitialized())
    return;

  if (!this->dataPtr->ogreItem)
    return;

  this->DestroyBuffer();

  // destroy ogre item
  this->dataPtr->sceneManager->destroyItem(this->dataPtr->ogreItem);
  this->dataPtr->ogreItem = nullptr;

  // remove mesh from mesh manager
  if (this->dataPtr->subMesh &&
      Ogre::MeshManager::getSingleton().resourceExists(
      this->dataPtr->subMesh->mParent->getName()))
  {
    Ogre::MeshManager::getSingleton().remove(
        this->dataPtr->subMesh->mParent->getName());
    this->dataPtr->subMesh = nullptr;
  }

  if (this->dataPtr->material && this->dataPtr->ownsMaterial)
  {
    this->dataPtr->scene->DestroyMaterial(this->dataPtr->material);
    this->dataPtr->material.reset();
  }
}

//////////////////////////////////////////////////
void Ogre2DynamicRenderable::DestroyBuffer()
{
  if (this->dataPtr->vbuffer)
    delete [] this->dataPtr->vbuffer;

  Ogre::RenderSystem *renderSystem =
      this->dataPtr->sceneManager->getDestinationRenderSystem();
  Ogre::VaoManager *vaoManager = renderSystem->getVaoManager();

  if (!vaoManager)
    return;

  if (this->dataPtr->subMesh)
  {
    if (!this->dataPtr->subMesh->mVao[Ogre::VpNormal].empty())
    {
      this->dataPtr->subMesh->destroyVaos(
          this->dataPtr->subMesh->mVao[Ogre::VpNormal], vaoManager);
    }
    if (!this->dataPtr->subMesh->mVao[Ogre::VpShadow].empty())
      this->dataPtr->subMesh->mVao[Ogre::VpShadow].clear();
  }

  this->dataPtr->vertexBuffer = nullptr;
  this->dataPtr->vao = nullptr;
  this->dataPtr->vbuffer = nullptr;
}

//////////////////////////////////////////////////
Ogre::MovableObject *Ogre2DynamicRenderable::OgreObject() const
{
  return this->dataPtr->ogreItem;
}

//////////////////////////////////////////////////
void Ogre2DynamicRenderable::Update()
{
  this->UpdateBuffer();
}

//////////////////////////////////////////////////
void Ogre2DynamicRenderable::CreateDynamicMesh()
{
  if (this->dataPtr->ogreItem)
    return;

  static int dynamicRenderableId = 0;
  Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(
              "dynamic_renderable_" + std::to_string(dynamicRenderableId++),
              Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  this->dataPtr->subMesh = mesh->createSubMesh();
  this->dataPtr->dirty = true;

  // this creates the ogre2 dynamic geometry buffer
  this->UpdateBuffer();

  this->dataPtr->ogreItem =
      this->dataPtr->sceneManager->createItem(mesh, Ogre::SCENE_DYNAMIC);
}

//////////////////////////////////////////////////
void Ogre2DynamicRenderable::UpdateBuffer()
{
  if (!this->dataPtr->dirty)
    return;

  Ogre::RenderSystem *renderSystem =
      this->dataPtr->sceneManager->getDestinationRenderSystem();

  Ogre::VaoManager *vaoManager = renderSystem->getVaoManager();
  if (!vaoManager)
    return;

  // Prepare vertex buffer
  unsigned int newVertCapacity = this->dataPtr->vertexBufferCapacity;

  unsigned int vertexCount = this->dataPtr->vertices.size();
  if ((vertexCount > this->dataPtr->vertexBufferCapacity) ||
      (!this->dataPtr->vertexBufferCapacity))
  {
    // vertexCount exceeds current capacity!
    // It is necessary to reallocate the buffer.

    // Check if this is the first call
    if (!newVertCapacity)
      newVertCapacity = 1;

    // Make capacity the next power of two
    while (newVertCapacity < vertexCount)
      newVertCapacity <<= 1;
  }
  else if (vertexCount < this->dataPtr->vertexBufferCapacity>>1)
  {
    // Make capacity the previous power of two
    unsigned int newCapacity = newVertCapacity >>1;
    while (vertexCount < newCapacity)
    {
      newVertCapacity = newCapacity;
      newCapacity >>= 1;
    }
  }

  // recreate vao if needed
  if (newVertCapacity != this->dataPtr->vertexBufferCapacity)
  {
    this->dataPtr->vertexBufferCapacity = newVertCapacity;

    this->DestroyBuffer();

    unsigned int size = this->dataPtr->vertexBufferCapacity * 6;
    this->dataPtr->vbuffer = new float[size];
    memset(this->dataPtr->vbuffer, 0, size * sizeof(float));

    this->dataPtr->subMesh->mVao[Ogre::VpNormal].clear();
    this->dataPtr->subMesh->mVao[Ogre::VpShadow].clear();

    // recreate the vao data structures
    Ogre::VertexElement2Vec vertexElements;
    vertexElements.push_back(
        Ogre::VertexElement2(Ogre::VET_FLOAT3, Ogre::VES_POSITION));
    vertexElements.push_back(
        Ogre::VertexElement2(Ogre::VET_FLOAT3, Ogre::VES_NORMAL));

    // create vertex buffer
    this->dataPtr->vertexBuffer = vaoManager->createVertexBuffer(
        vertexElements, this->dataPtr->vertexBufferCapacity,
        Ogre::BT_DYNAMIC_PERSISTENT, this->dataPtr->vbuffer, false);

    Ogre::VertexBufferPackedVec vertexBuffers;
    vertexBuffers.push_back(this->dataPtr->vertexBuffer);

    // it is ok to use null index buffer
    Ogre::IndexBufferPacked *indexBuffer = nullptr;

    this->dataPtr->vao = vaoManager->createVertexArrayObject(vertexBuffers,
        indexBuffer, this->dataPtr->operationType);

    this->dataPtr->subMesh->mVao[Ogre::VpNormal].push_back(this->dataPtr->vao);
    // Use the same geometry for shadow casting.
    this->dataPtr->subMesh->mVao[Ogre::VpShadow].push_back(this->dataPtr->vao);
  }

  // map buffer and update the geometry
  Ogre::Aabb bbox;
  float * RESTRICT_ALIAS vertices = reinterpret_cast<float * RESTRICT_ALIAS>(
      this->dataPtr->vertexBuffer->map(
      0, this->dataPtr->vertexBuffer->getNumElements()));

  // fill vertices
  for (unsigned int i = 0; i < vertexCount; ++i)
  {
    unsigned int idx = i*6;
    Ogre::Vector3 v = Ogre2Conversions::Convert(this->dataPtr->vertices[i]);
    vertices[idx] = v.x;
    vertices[idx+1] = v.y;
    vertices[idx+2] = v.z;

    bbox.merge(v);
  }

  // fill the rest of the buffer with the position of the last vertex to avoid
  // the geometry connecting back to 0, 0, 0
  if (vertexCount > 0 && vertexCount < this->dataPtr->vertexBufferCapacity)
  {
    math::Vector3d lastVertex = this->dataPtr->vertices[vertexCount-1];
    for (unsigned int i = vertexCount; i < this->dataPtr->vertexBufferCapacity;
        ++i)
    {
      unsigned int idx = i * 6;
      vertices[idx] = lastVertex.X();
      vertices[idx+1] = lastVertex.Y();
      vertices[idx+2] = lastVertex.Z();

      vertices[idx+3] = 0;
      vertices[idx+4] = 0;
      vertices[idx+5] = 1;
    }
  }

  // fill normals
  this->GenerateNormals(this->dataPtr->operationType, this->dataPtr->vertices,
      vertices);

  // unmap buffer
  this->dataPtr->vertexBuffer->unmap(Ogre::UO_KEEP_PERSISTENT);

  // Set the bounds to get frustum culling and LOD to work correctly.
  Ogre::Mesh *mesh = this->dataPtr->subMesh->mParent;
  mesh->_setBounds(bbox, true);

  // update item aabb
  if (this->dataPtr->ogreItem)
  {
    bool castShadows = this->dataPtr->ogreItem->getCastShadows();
    auto lowLevelMat = this->dataPtr->ogreItem->getSubItem(0)->getMaterial();

    // need to rebuild ogre [sub]item because the vao was destroyed
    // this updates the item's bounding box and fixes occasional crashes
    // from invalid access to old vao
    this->dataPtr->ogreItem->_initialise(true);

    // set material
    if (this->dataPtr->material)
    {
      this->dataPtr->ogreItem->getSubItem(0)->setDatablock(
          static_cast<Ogre::HlmsPbsDatablock *>(
          this->dataPtr->material->Datablock()));
      this->dataPtr->ogreItem->setCastShadows(
          this->dataPtr->material->CastShadows());
    }
    else if (lowLevelMat)
    {
      // the _initialise call above resets the ogre item properties so set
      // them again
      this->dataPtr->ogreItem->getSubItem(0)->setMaterial(lowLevelMat);
      this->dataPtr->ogreItem->setCastShadows(castShadows);
    }
  }

  this->dataPtr->dirty = false;
}

//////////////////////////////////////////////////
void Ogre2DynamicRenderable::SetOperationType(MarkerType _opType)
{
  switch (_opType)
  {
    case MT_POINTS:
      this->dataPtr->operationType = Ogre::OperationType::OT_POINT_LIST;
      break;

    case MT_LINE_LIST:
      this->dataPtr->operationType = Ogre::OperationType::OT_LINE_LIST;
      break;

    case MT_LINE_STRIP:
      this->dataPtr->operationType = Ogre::OperationType::OT_LINE_STRIP;
      break;

    case MT_TRIANGLE_LIST:
      this->dataPtr->operationType = Ogre::OperationType::OT_TRIANGLE_LIST;
      break;

    case MT_TRIANGLE_STRIP:
      this->dataPtr->operationType = Ogre::OperationType::OT_TRIANGLE_STRIP;
      break;

    case MT_TRIANGLE_FAN:
      this->dataPtr->operationType = Ogre::OperationType::OT_TRIANGLE_FAN;
      break;

    default:
      ignerr << "Unknown render operation type[" << _opType << "]\n";
      return;
  }
}

//////////////////////////////////////////////////
MarkerType Ogre2DynamicRenderable::OperationType() const
{
  MarkerType opType;
  switch (this->dataPtr->operationType)
  {
    case Ogre::OperationType::OT_LINE_LIST:
      opType = MT_LINE_LIST;
      break;

    case Ogre::OperationType::OT_LINE_STRIP:
      opType = MT_LINE_STRIP;
      break;

    case Ogre::OperationType::OT_TRIANGLE_LIST:
      opType = MT_TRIANGLE_LIST;
      break;

    case Ogre::OperationType::OT_TRIANGLE_STRIP:
      opType = MT_TRIANGLE_STRIP;
      break;

    case Ogre::OperationType::OT_TRIANGLE_FAN:
      opType = MT_TRIANGLE_FAN;
      break;

    default:
    case Ogre::OperationType::OT_POINT_LIST:
      opType = MT_POINTS;
      break;
  }

  return opType;
}

/////////////////////////////////////////////////
void Ogre2DynamicRenderable::AddPoint(const math::Vector3d &_pt,
                                      const math::Color &_color)
{
  this->dataPtr->vertices.push_back(_pt);

  // todo(anyone)
  // setting material works but vertex coloring does not work yet.
  // It requires using an unlit datablock:
  // https://forums.ogre3d.org/viewtopic.php?t=93627#p539276
  this->dataPtr->colors.push_back(_color);

  this->dataPtr->dirty = true;
}

/////////////////////////////////////////////////
void Ogre2DynamicRenderable::AddPoint(double _x, double _y, double _z,
                                      const math::Color &_color)
{
  this->AddPoint(math::Vector3d(_x, _y, _z), _color);
}

/////////////////////////////////////////////////
void Ogre2DynamicRenderable::SetPoint(unsigned int _index,
                                      const math::Vector3d &_value)
{
  if (_index >= this->dataPtr->vertices.size())
  {
    ignerr << "Point index[" << _index << "] is out of bounds[0-"
           << this->dataPtr->vertices.size()-1 << "]\n";
    return;
  }

  this->dataPtr->vertices[_index] = _value;

  this->dataPtr->dirty = true;
}

/////////////////////////////////////////////////
void Ogre2DynamicRenderable::SetColor(unsigned int _index,
                                      const math::Color &_color)
{
  if (_index >= this->dataPtr->colors.size())
  {
    ignerr << "Point color index[" << _index << "] is out of bounds[0-"
           << this->dataPtr->colors.size()-1 << "]\n";
    return;
  }


  // todo(anyone)
  // vertex coloring does not work yet. It requires using an unlit datablock:
  // https://forums.ogre3d.org/viewtopic.php?t=93627#p539276
  this->dataPtr->colors[_index] = _color;

  // uncomment this line when colors are working
  // this->dataPtr->dirty = true;
}

/////////////////////////////////////////////////
math::Vector3d Ogre2DynamicRenderable::Point(
    const unsigned int _index) const
{
  if (_index >= this->dataPtr->vertices.size())
  {
    ignerr << "Point index[" << _index << "] is out of bounds[0-"
           << this->dataPtr->vertices.size()-1 << "]\n";

    return math::Vector3d(math::INF_D,
                                    math::INF_D,
                                    math::INF_D);
  }

  return this->dataPtr->vertices[_index];
}

/////////////////////////////////////////////////
unsigned int Ogre2DynamicRenderable::PointCount() const
{
  return this->dataPtr->vertices.size();
}

/////////////////////////////////////////////////
void Ogre2DynamicRenderable::Clear()
{
  if (this->dataPtr->vertices.empty() && this->dataPtr->colors.empty())
    return;

  this->dataPtr->vertices.clear();
  this->dataPtr->colors.clear();
  this->dataPtr->dirty = true;
}

//////////////////////////////////////////////////
void Ogre2DynamicRenderable::SetMaterial(MaterialPtr _material, bool _unique)
{
  _material = (_unique) ? _material->Clone() : _material;

  Ogre2MaterialPtr derived =
      std::dynamic_pointer_cast<Ogre2Material>(_material);

  if (!derived)
  {
    ignerr << "Cannot assign material created by another render-engine"
        << std::endl;

    return;
  }

  if (this->dataPtr->material && this->dataPtr->ownsMaterial)
    this->dataPtr->scene->DestroyMaterial(this->dataPtr->material);

  this->dataPtr->ownsMaterial = _unique;

  this->dataPtr->material = derived;

  this->dataPtr->ogreItem->getSubItem(0)->setDatablock(
      static_cast<Ogre::HlmsPbsDatablock *>(derived->Datablock()));

  // set cast shadows
  this->dataPtr->ogreItem->setCastShadows(_material->CastShadows());
}

//////////////////////////////////////////////////
void Ogre2DynamicRenderable::GenerateNormals(Ogre::OperationType _opType,
  const std::vector<math::Vector3d> &_vertices, float *_vbuffer)
{
  unsigned int vertexCount = _vertices.size();
  // Each vertex occupies 6 elements in the vbuffer float array:
  // vbuffer[i]   : position x
  // vbuffer[i+1] : position y
  // vbuffer[i+2] : position z
  // vbuffer[i+3] : normal x
  // vbuffer[i+4] : normal y
  // vbuffer[i+5] : normal z
  switch (_opType)
  {
    case Ogre::OperationType::OT_POINT_LIST:
    case Ogre::OperationType::OT_LINE_LIST:
    case Ogre::OperationType::OT_LINE_STRIP:
      return;
    case Ogre::OperationType::OT_TRIANGLE_LIST:
    {
      if (vertexCount < 3)
        return;

      for (unsigned int i = 0; i < vertexCount / 3; ++i)
      {
        unsigned int idx = i*3;
        unsigned int idx1 = idx * 6;
        unsigned int idx2 = idx1 + 6;
        unsigned int idx3 = idx2 + 6;
        math::Vector3d v1 = _vertices[idx];
        math::Vector3d v2 = _vertices[idx+1];
        math::Vector3d v3 = _vertices[idx+2];
        math::Vector3d n = (v1 - v2).Cross((v1 - v3));

        _vbuffer[idx1+3] = n.X();
        _vbuffer[idx1+4] = n.Y();
        _vbuffer[idx1+5] = n.Z();
        _vbuffer[idx2+3] = n.X();
        _vbuffer[idx2+4] = n.Y();
        _vbuffer[idx2+5] = n.Z();
        _vbuffer[idx3+3] = n.X();
        _vbuffer[idx3+4] = n.Y();
        _vbuffer[idx3+5] = n.Z();
      }

      break;
    }
    case Ogre::OperationType::OT_TRIANGLE_STRIP:
    {
      if (vertexCount < 3)
        return;

      bool even = false;
      for (unsigned int i = 0; i < vertexCount - 2; ++i)
      {
        math::Vector3d v1;
        math::Vector3d v2;
        math::Vector3d v3 = _vertices[i+2];

        // For odd n, vertices n, n+1, and n+2 define triangle n.
        // For even n, vertices n+1, n, and n+2 define triangle n.
        unsigned int idx1;
        unsigned int idx2;
        unsigned int idx3 = (i+2) * 6;
        if (even)
        {
          v1 = _vertices[i+1];
          v2 = _vertices[i];
          idx1 = (i+1) * 6;
          idx2 = i*6;
        }
        else
        {
          v1 = _vertices[i];
          v2 = _vertices[i+1];
          idx1 = i*6;
          idx2 = (i+1) * 6;
        }
        even = !even;

        math::Vector3d n = (v1 - v2).Cross((v1 - v3));
        math::Vector3d n1(_vbuffer[idx1+3], _vbuffer[idx1+4], _vbuffer[idx1+5]);
        math::Vector3d n2(_vbuffer[idx2+3], _vbuffer[idx2+4], _vbuffer[idx2+5]);
        math::Vector3d n3(_vbuffer[idx3+3], _vbuffer[idx3+4], _vbuffer[idx3+5]);

        math::Vector3d n1a = ((n1 + n)/2);
        n1a.Normalize();
        math::Vector3d n2a = ((n2 + n)/2);
        n2a.Normalize();
        math::Vector3d n3a = ((n3 + n)/2);
        n3a.Normalize();

        _vbuffer[idx1+3] = n1a.X();
        _vbuffer[idx1+4] = n1a.Y();
        _vbuffer[idx1+5] = n1a.Z();
        _vbuffer[idx2+3] = n2a.X();
        _vbuffer[idx2+4] = n2a.Y();
        _vbuffer[idx2+5] = n2a.Z();
        _vbuffer[idx3+3] = n3a.X();
        _vbuffer[idx3+4] = n3a.Y();
        _vbuffer[idx3+5] = n3a.Z();
      }

      break;
    }
    case Ogre::OperationType::OT_TRIANGLE_FAN:
    {
      if (vertexCount < 3)
        return;

      unsigned int idx1 = 0;
      math::Vector3d v1 = _vertices[0];

      for (unsigned int i = 0; i < vertexCount - 2; ++i)
      {
        unsigned int idx2 = (i+1) * 6;
        unsigned int idx3 = idx2 + 6;
        math::Vector3d v2 = _vertices[i+1];
        math::Vector3d v3 = _vertices[i+2];
        math::Vector3d n = (v1 - v2).Cross((v1 - v3));

        math::Vector3d n1(_vbuffer[idx1+3], _vbuffer[idx1+4], _vbuffer[idx1+5]);
        math::Vector3d n2(_vbuffer[idx2+3], _vbuffer[idx2+4], _vbuffer[idx2+5]);
        math::Vector3d n3(_vbuffer[idx3+3], _vbuffer[idx3+4], _vbuffer[idx3+5]);

        math::Vector3d n1a = ((n1 + n)/2);
        n1a.Normalize();
        math::Vector3d n2a = ((n2 + n)/2);
        n2a.Normalize();
        math::Vector3d n3a = ((n3 + n)/2);
        n3a.Normalize();

        _vbuffer[idx1+3] = n1a.X();
        _vbuffer[idx1+4] = n1a.Y();
        _vbuffer[idx1+5] = n1a.Z();
        _vbuffer[idx2+3] = n2a.X();
        _vbuffer[idx2+4] = n2a.Y();
        _vbuffer[idx2+5] = n2a.Z();
        _vbuffer[idx3+3] = n3a.X();
        _vbuffer[idx3+4] = n3a.Y();
        _vbuffer[idx3+5] = n3a.Z();
      }

      break;
    }
    default:
      break;
  }
}
