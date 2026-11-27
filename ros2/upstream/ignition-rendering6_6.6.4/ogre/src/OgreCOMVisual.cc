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

#include "ignition/rendering/ogre/OgreCOMVisual.hh"
#include "ignition/rendering/ogre/OgreDynamicLines.hh"

#include "ignition/rendering/ogre/OgreScene.hh"

class ignition::rendering::OgreCOMVisualPrivate
{
  /// \brief Grid materal
  public: OgreMaterialPtr material = nullptr;

  /// \brief Lines that make the cross marking the center of mass.
  public: std::shared_ptr<OgreDynamicLines> crossLines = nullptr;

  /// \brief Sphere visual marking the center of mass
  public: VisualPtr sphereVis = nullptr;
};

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
OgreCOMVisual::OgreCOMVisual()
  : dataPtr(new OgreCOMVisualPrivate)
{
}

//////////////////////////////////////////////////
OgreCOMVisual::~OgreCOMVisual()
{
}

//////////////////////////////////////////////////
void OgreCOMVisual::PreRender()
{
  if (this->HasParent() && this->parentName.empty())
    this->parentName = this->Parent()->Name();

  if (this->dirtyCOMVisual &&
      !this->parentName.empty())
  {
    this->parentName = this->Parent()->Name();
    this->CreateVisual();
    this->dirtyCOMVisual = false;
  }
}

//////////////////////////////////////////////////
void OgreCOMVisual::Init()
{
  BaseCOMVisual::Init();
}

//////////////////////////////////////////////////
Ogre::MovableObject *OgreCOMVisual::OgreObject() const
{
  std::shared_ptr<Ogre::MovableObject> mv =
    std::dynamic_pointer_cast<Ogre::MovableObject>(this->dataPtr->crossLines);
  return mv.get();
}

//////////////////////////////////////////////////
void OgreCOMVisual::CreateVisual()
{
  if (!this->dataPtr->crossLines)
  {
    this->dataPtr->crossLines = std::shared_ptr<OgreDynamicLines>(
      new OgreDynamicLines(MT_LINE_LIST));
    this->ogreNode->attachObject(this->OgreObject());
    MaterialPtr COMVisualMaterial =
      this->Scene()->Material("Default/TransGreen");
    this->SetMaterial(COMVisualMaterial, false);
  }

  if (!this->dataPtr->sphereVis)
  {
    this->dataPtr->sphereVis = this->Scene()->CreateVisual();
    this->dataPtr->sphereVis->AddGeometry(this->Scene()->CreateSphere());
    this->dataPtr->sphereVis->SetMaterial("Default/CoM");
    this->dataPtr->sphereVis->SetInheritScale(false);
    this->AddChild(this->dataPtr->sphereVis);
  }

  double sphereRadius = this->SphereRadius();
  this->dataPtr->sphereVis->SetLocalScale(ignition::math::Vector3d(
      sphereRadius*2, sphereRadius*2, sphereRadius*2));
  this->dataPtr->sphereVis->SetLocalPosition(this->InertiaPose().Pos());
  this->dataPtr->sphereVis->SetLocalRotation(this->InertiaPose().Rot());

  // Get the bounding box of the parent visual
  VisualPtr vis = this->Scene()->VisualByName(this->parentName);
  ignition::math::AxisAlignedBox box;
  if (vis)
    box = vis->LocalBoundingBox();

  // Clear any previous data from the grid and update
  this->dataPtr->crossLines->Clear();
  this->dataPtr->crossLines->Update();

  // CoM position indicator
  ignition::math::Vector3d p1(0, 0,
      box.Min().Z() - this->InertiaPose().Pos().Z());
  ignition::math::Vector3d p2(0, 0,
      box.Max().Z() - this->InertiaPose().Pos().Z());

  ignition::math::Vector3d p3(0,
      box.Min().Y() - this->InertiaPose().Pos().Y(), 0);
  ignition::math::Vector3d p4(0,
      box.Max().Y() - this->InertiaPose().Pos().Y(), 0);

  ignition::math::Vector3d p5(
      box.Min().X() - this->InertiaPose().Pos().X(), 0, 0);
  ignition::math::Vector3d p6(
      box.Max().X() - this->InertiaPose().Pos().X(), 0, 0);

  p1 += this->InertiaPose().Pos();
  p2 += this->InertiaPose().Pos();
  p3 += this->InertiaPose().Pos();
  p4 += this->InertiaPose().Pos();
  p5 += this->InertiaPose().Pos();
  p6 += this->InertiaPose().Pos();

  this->dataPtr->crossLines->AddPoint(p1);
  this->dataPtr->crossLines->AddPoint(p2);
  this->dataPtr->crossLines->AddPoint(p3);
  this->dataPtr->crossLines->AddPoint(p4);
  this->dataPtr->crossLines->AddPoint(p5);
  this->dataPtr->crossLines->AddPoint(p6);

  this->dataPtr->crossLines->Update();
  this->ogreNode->setVisible(true);
}

//////////////////////////////////////////////////
void OgreCOMVisual::SetMaterial(MaterialPtr _material, bool _unique)
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

  this->SetMaterialImpl(derived);
}

//////////////////////////////////////////////////
void OgreCOMVisual::SetMaterialImpl(OgreMaterialPtr _material)
{
  std::string materialName = _material->Name();
  Ogre::MaterialPtr ogreMaterial = _material->Material();

// OGRE 1.10.7
#if (OGRE_VERSION <= ((1 << 16) | (10 << 8) | 7))
  this->dataPtr->crossLines->setMaterial(materialName);
#else
  this->dataPtr->crossLines->setMaterial(ogreMaterial);
#endif
  this->dataPtr->material = _material;

  this->dataPtr->material->SetReceiveShadows(false);
  this->dataPtr->material->SetLightingEnabled(false);
}

//////////////////////////////////////////////////
MaterialPtr OgreCOMVisual::Material() const
{
  return this->dataPtr->material;
}

//////////////////////////////////////////////////
VisualPtr OgreCOMVisual::SphereVisual() const
{
  return this->dataPtr->sphereVis;
}
