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

#include "ignition/rendering/ogre/OgreInertiaVisual.hh"
#include "ignition/rendering/ogre/OgreDynamicLines.hh"

#include "ignition/rendering/ogre/OgreScene.hh"

class ignition::rendering::OgreInertiaVisualPrivate
{
  /// \brief Grid materal
  public: OgreMaterialPtr material = nullptr;

  /// \brief Ogre renderable used to render the cross lines.
  public: std::shared_ptr<OgreDynamicLines> crossLines = nullptr;

  /// \brief Box visual
  public: VisualPtr boxVis = nullptr;
};

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
OgreInertiaVisual::OgreInertiaVisual()
  : dataPtr(new OgreInertiaVisualPrivate)
{
}

//////////////////////////////////////////////////
OgreInertiaVisual::~OgreInertiaVisual()
{
}

//////////////////////////////////////////////////
void OgreInertiaVisual::PreRender()
{
}

//////////////////////////////////////////////////
void OgreInertiaVisual::Init()
{
  BaseInertiaVisual::Init();
}

//////////////////////////////////////////////////
Ogre::MovableObject *OgreInertiaVisual::OgreObject() const
{
  std::shared_ptr<Ogre::MovableObject> mv =
    std::dynamic_pointer_cast<Ogre::MovableObject>(this->dataPtr->crossLines);
  return mv.get();
}

//////////////////////////////////////////////////
void OgreInertiaVisual::Load(const ignition::math::Pose3d &_pose,
                             const ignition::math::Vector3d &_scale)
{
  if (!this->dataPtr->crossLines)
  {
    this->dataPtr->crossLines = std::shared_ptr<OgreDynamicLines>(
      new OgreDynamicLines(MT_LINE_LIST));
    this->ogreNode->attachObject(this->OgreObject());
    MaterialPtr inertiaVisualMaterial =
      this->Scene()->Material("Default/TransGreen");
    this->SetMaterial(inertiaVisualMaterial, false);
  }

  if (!this->dataPtr->boxVis)
  {
    this->dataPtr->boxVis = this->Scene()->CreateVisual();
    this->dataPtr->boxVis->AddGeometry(this->Scene()->CreateBox());
    this->dataPtr->boxVis->SetMaterial("Default/TransPurple");
    this->AddChild(this->dataPtr->boxVis);
  }

  // Inertia position indicator
  ignition::math::Vector3d p1(0, 0, -2*_scale.Z());
  ignition::math::Vector3d p2(0, 0, 2*_scale.Z());
  ignition::math::Vector3d p3(0, -2*_scale.Y(), 0);
  ignition::math::Vector3d p4(0, 2*_scale.Y(), 0);
  ignition::math::Vector3d p5(-2*_scale.X(), 0, 0);
  ignition::math::Vector3d p6(2*_scale.X(), 0, 0);
  p1 = _pose.Rot().RotateVector(p1);
  p2 = _pose.Rot().RotateVector(p2);
  p3 = _pose.Rot().RotateVector(p3);
  p4 = _pose.Rot().RotateVector(p4);
  p5 = _pose.Rot().RotateVector(p5);
  p6 = _pose.Rot().RotateVector(p6);
  p1 += _pose.Pos();
  p2 += _pose.Pos();
  p3 += _pose.Pos();
  p4 += _pose.Pos();
  p5 += _pose.Pos();
  p6 += _pose.Pos();

  this->dataPtr->crossLines->AddPoint(p1);
  this->dataPtr->crossLines->AddPoint(p2);
  this->dataPtr->crossLines->AddPoint(p3);
  this->dataPtr->crossLines->AddPoint(p4);
  this->dataPtr->crossLines->AddPoint(p5);
  this->dataPtr->crossLines->AddPoint(p6);

  this->dataPtr->crossLines->Update();
  this->ogreNode->setVisible(true);

  this->dataPtr->boxVis->SetLocalScale(_scale);
  this->dataPtr->boxVis->SetLocalPosition(_pose.Pos());
  this->dataPtr->boxVis->SetLocalRotation(_pose.Rot());
}

//////////////////////////////////////////////////
VisualPtr OgreInertiaVisual::BoxVisual() const
{
  return this->dataPtr->boxVis;
}

//////////////////////////////////////////////////
void OgreInertiaVisual::SetMaterial(MaterialPtr _material, bool _unique)
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
void OgreInertiaVisual::SetMaterialImpl(OgreMaterialPtr _material)
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
MaterialPtr OgreInertiaVisual::Material() const
{
  return this->dataPtr->material;
}
