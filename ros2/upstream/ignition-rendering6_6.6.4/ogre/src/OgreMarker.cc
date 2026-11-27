/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "gz/rendering/ogre/OgreCapsule.hh"
#include "gz/rendering/ogre/OgreDynamicLines.hh"
#include "gz/rendering/ogre/OgreMarker.hh"
#include "gz/rendering/ogre/OgreMaterial.hh"
#include "gz/rendering/ogre/OgreMesh.hh"
#include "gz/rendering/ogre/OgreScene.hh"

class gz::rendering::OgreMarkerPrivate
{
  /// \brief Marker material
  public: OgreMaterialPtr material = nullptr;

  /// \brief DynamicLines Object to display
  public: std::shared_ptr<OgreDynamicLines> dynamicRenderable;

  /// \brief Geometry Object for primitive shapes
  public: OgreGeometryPtr geom{nullptr};
};

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
OgreMarker::OgreMarker()
  : dataPtr(new OgreMarkerPrivate)
{
}

//////////////////////////////////////////////////
OgreMarker::~OgreMarker()
{
  this->Destroy();
}

//////////////////////////////////////////////////
void OgreMarker::PreRender()
{
  if (this->markerType == MarkerType::MT_POINTS &&
      this->dataPtr->dynamicRenderable &&
      this->dataPtr->dynamicRenderable->PointCount() > 0u)
  {
    std::string pointsMatName = "PointCloudPoint";
    if (this->dataPtr->dynamicRenderable->getMaterial().isNull() ||
        this->dataPtr->dynamicRenderable->getMaterial()->getName()
        != pointsMatName)
    {
#if (OGRE_VERSION <= ((1 << 16) | (10 << 8) | 7))
      this->dataPtr->dynamicRenderable->setMaterial(pointsMatName);
#else
      Ogre::MaterialPtr pointsMat =
          Ogre::MaterialManager::getSingleton().getByName(
          pointsMatName);
      this->dataPtr->dynamicRenderable->setMaterial(pointsMat);
#endif
    }
    auto pass = this->dataPtr->dynamicRenderable->getMaterial()->getTechnique(
        0)->getPass(0);
    auto vertParams = pass->getVertexProgramParameters();
    vertParams->setNamedConstant("size", static_cast<Ogre::Real>(this->size));
  }

  this->dataPtr->dynamicRenderable->Update();
}

//////////////////////////////////////////////////
void OgreMarker::Destroy()
{
  if (this->dataPtr->geom)
  {
    this->dataPtr->geom->Destroy();
    this->dataPtr->geom.reset();
  }

  if (this->dataPtr->material && this->Scene())
  {
    this->Scene()->DestroyMaterial(this->dataPtr->material);
    this->dataPtr->material.reset();
  }
  this->dataPtr->dynamicRenderable.reset();
}

//////////////////////////////////////////////////
Ogre::MovableObject *OgreMarker::OgreObject() const
{
  switch (this->markerType)
  {
    case MT_NONE:
      return nullptr;
    case MT_BOX:
    case MT_CAPSULE:
    case MT_CYLINDER:
    case MT_SPHERE:
    {
      if (nullptr != this->dataPtr->geom)
      {
        return this->dataPtr->geom->OgreObject();
      }
      return nullptr;
    }
    case MT_LINE_STRIP:
    case MT_LINE_LIST:
    case MT_POINTS:
    case MT_TRIANGLE_FAN:
    case MT_TRIANGLE_LIST:
    case MT_TRIANGLE_STRIP:
      return std::dynamic_pointer_cast<Ogre::MovableObject>
        (this->dataPtr->dynamicRenderable).get();
    default:
      ignerr << "Invalid Marker type " << this->markerType << "\n";
      return nullptr;
  }
}

//////////////////////////////////////////////////
void OgreMarker::Init()
{
  this->Create();
}

//////////////////////////////////////////////////
void OgreMarker::Create()
{
  this->markerType = MT_NONE;
  this->dataPtr->dynamicRenderable.reset(new OgreDynamicLines(MT_LINE_STRIP));

  if (!this->dataPtr->geom)
  {
    this->dataPtr->geom =
      std::dynamic_pointer_cast<OgreGeometry>(this->scene->CreateBox());
  }
}

//////////////////////////////////////////////////
void OgreMarker::SetMaterial(MaterialPtr _material, bool _unique)
{
  if (nullptr == _material)
  {
    ignerr << "Cannot assign null material" << std::endl;
    return;
  }

  _material = (_unique) ? _material->Clone() : _material;

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
  this->dataPtr->material = derived;

  this->dataPtr->material->SetReceiveShadows(false);
  this->dataPtr->material->SetCastShadows(false);
  this->dataPtr->material->SetLightingEnabled(false);

  switch (this->markerType)
  {
    case MT_NONE:
      break;
    case MT_BOX:
    case MT_CAPSULE:
    case MT_CYLINDER:
    case MT_SPHERE:
    {
      if (nullptr != this->dataPtr->geom)
      {
        this->dataPtr->geom->SetMaterial(derived, false);
      }
      else
      {
        ignerr << "Failed to set material, null geometry." << std::endl;
      }
      break;
    }
    case MT_LINE_STRIP:
    case MT_LINE_LIST:
    case MT_POINTS:
    case MT_TRIANGLE_FAN:
    case MT_TRIANGLE_LIST:
    case MT_TRIANGLE_STRIP:
#if (OGRE_VERSION <= ((1 << 16) | (10 << 8) | 7))
      this->dataPtr->dynamicRenderable->setMaterial(materialName);
#else
      this->dataPtr->dynamicRenderable->setMaterial(ogreMaterial);
#endif
      break;
    default:
      ignerr << "Invalid Marker type " << this->markerType << "\n";
      break;
  }
}

//////////////////////////////////////////////////
MaterialPtr OgreMarker::Material() const
{
  return this->dataPtr->material;
}

//////////////////////////////////////////////////
void OgreMarker::SetPoint(unsigned int _index,
    const math::Vector3d &_value)
{
  this->dataPtr->dynamicRenderable->SetPoint(_index, _value);
}

//////////////////////////////////////////////////
void OgreMarker::AddPoint(const math::Vector3d &_pt,
    const math::Color &_color)
{
  this->dataPtr->dynamicRenderable->AddPoint(_pt, _color);
}

//////////////////////////////////////////////////
void OgreMarker::ClearPoints()
{
  this->dataPtr->dynamicRenderable->Clear();
}

//////////////////////////////////////////////////
void OgreMarker::SetType(MarkerType _markerType)
{
  this->markerType = _markerType;
  switch (_markerType)
  {
    case MT_NONE:
      break;
    case MT_BOX:
      this->dataPtr->geom =
        std::dynamic_pointer_cast<OgreGeometry>(this->scene->CreateBox());
      break;
    case MT_CAPSULE:
      this->dataPtr->geom =
        std::dynamic_pointer_cast<OgreGeometry>(this->scene->CreateCapsule());
      break;
    case MT_CYLINDER:
      this->dataPtr->geom =
        std::dynamic_pointer_cast<OgreGeometry>(this->scene->CreateCylinder());
      break;
    case MT_SPHERE:
      this->dataPtr->geom =
        std::dynamic_pointer_cast<OgreGeometry>(this->scene->CreateSphere());
      break;
    case MT_LINE_STRIP:
    case MT_LINE_LIST:
    case MT_POINTS:
    case MT_TRIANGLE_FAN:
    case MT_TRIANGLE_LIST:
    case MT_TRIANGLE_STRIP:
      this->dataPtr->dynamicRenderable->SetOperationType(_markerType);
      break;
    default:
      ignerr << "Invalid Marker type\n";
      break;
  }
}

//////////////////////////////////////////////////
MarkerType OgreMarker::Type() const
{
  return this->markerType;
}
