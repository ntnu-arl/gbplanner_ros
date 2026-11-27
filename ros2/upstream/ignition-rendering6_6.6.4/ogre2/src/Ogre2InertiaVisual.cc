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
#include "ignition/rendering/ogre2/Ogre2InertiaVisual.hh"
#include "ignition/rendering/ogre2/Ogre2Material.hh"
#include "ignition/rendering/ogre2/Ogre2DynamicRenderable.hh"

#ifdef _MSC_VER
  #pragma warning(push, 0)
#endif
#include <OgreSceneNode.h>
#ifdef _MSC_VER
  #pragma warning(pop)
#endif

using namespace ignition;
using namespace rendering;

class ignition::rendering::Ogre2InertiaVisualPrivate
{
  /// \brief inertia visual materal
  public: Ogre2MaterialPtr material = nullptr;

  /// \brief Ogre renderable used to render the cross lines.
  public: std::shared_ptr<Ogre2DynamicRenderable> crossLines = nullptr;

  /// \brief Box visual
  public: VisualPtr boxVis = nullptr;
};

//////////////////////////////////////////////////
Ogre2InertiaVisual::Ogre2InertiaVisual()
  : dataPtr(new Ogre2InertiaVisualPrivate)
{
}

//////////////////////////////////////////////////
Ogre2InertiaVisual::~Ogre2InertiaVisual()
{
}

//////////////////////////////////////////////////
void Ogre2InertiaVisual::PreRender()
{
}

//////////////////////////////////////////////////
void Ogre2InertiaVisual::Init()
{
  BaseInertiaVisual::Init();
}

//////////////////////////////////////////////////
void Ogre2InertiaVisual::Destroy()
{
  if (this->dataPtr->boxVis != nullptr)
  {
    this->dataPtr->boxVis->Destroy();
    this->dataPtr->boxVis.reset();
  }

  if (this->dataPtr->crossLines)
  {
    this->dataPtr->crossLines->Destroy();
    this->dataPtr->crossLines.reset();
  }

  if (this->dataPtr->material && this->Scene())
  {
    this->Scene()->DestroyMaterial(this->dataPtr->material);
    this->dataPtr->material.reset();
  }
}

//////////////////////////////////////////////////
void Ogre2InertiaVisual::Load(const ignition::math::Pose3d &_pose,
                              const ignition::math::Vector3d &_scale)
{
  if (!this->dataPtr->crossLines)
  {
    this->dataPtr->crossLines.reset(
      new Ogre2DynamicRenderable(this->Scene()));
    this->ogreNode->attachObject(this->dataPtr->crossLines->OgreObject());
  }

  if (!this->dataPtr->boxVis)
  {
    this->dataPtr->boxVis = this->Scene()->CreateVisual();
    this->dataPtr->boxVis->AddGeometry(this->Scene()->CreateBox());
    this->dataPtr->boxVis->SetMaterial("Default/TransPurple");
    this->AddChild(this->dataPtr->boxVis);
  }

  // Clear any previous data from the grid and update
  this->dataPtr->crossLines->Clear();
  this->dataPtr->crossLines->Update();

  this->dataPtr->crossLines->SetOperationType(MarkerType::MT_LINE_LIST);
  if (this->dataPtr->material == nullptr)
  {
    MaterialPtr defaultMat =
        this->Scene()->Material("Default/TransGreen")->Clone();
    this->SetMaterial(defaultMat, false);
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

  this->dataPtr->boxVis->SetLocalScale(_scale);
  this->dataPtr->boxVis->SetLocalPosition(_pose.Pos());
  this->dataPtr->boxVis->SetLocalRotation(_pose.Rot());
}

//////////////////////////////////////////////////
VisualPtr Ogre2InertiaVisual::BoxVisual() const
{
  return this->dataPtr->boxVis;
}

//////////////////////////////////////////////////
void Ogre2InertiaVisual::SetMaterial(MaterialPtr _material, bool _unique)
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

  this->dataPtr->crossLines->SetMaterial(_material, false);
  this->SetMaterialImpl(derived);
}

//////////////////////////////////////////////////
void Ogre2InertiaVisual::SetMaterialImpl(Ogre2MaterialPtr _material)
{
  Ogre::MaterialPtr ogreMaterial = _material->Material();
  this->dataPtr->material = _material;
}

//////////////////////////////////////////////////
MaterialPtr Ogre2InertiaVisual::Material() const
{
  return this->dataPtr->material;
}
