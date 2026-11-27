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

#include <gz/common/Console.hh>

#include "gz/rendering/ogre2/Ogre2Conversions.hh"
#include "gz/rendering/ogre2/Ogre2Geometry.hh"
#include "gz/rendering/ogre2/Ogre2ParticleEmitter.hh"
#include "gz/rendering/ogre2/Ogre2RenderTypes.hh"
#include "gz/rendering/ogre2/Ogre2Storage.hh"
#include "gz/rendering/ogre2/Ogre2Visual.hh"
#include "gz/rendering/Utils.hh"

#ifdef _MSC_VER
  #pragma warning(push, 0)
#endif
#include <OgreItem.h>
#ifdef _MSC_VER
  #pragma warning(pop)
#endif

using namespace gz;
using namespace rendering;

/// \brief Private data for the Ogre2Visual class
class ignition::rendering::Ogre2VisualPrivate
{
  /// \brief True if wireframe mode is enabled
  public: bool wireframe;
};

//////////////////////////////////////////////////
Ogre2Visual::Ogre2Visual()
  : dataPtr(new Ogre2VisualPrivate)
{
  this->dataPtr->wireframe = false;
}

//////////////////////////////////////////////////
Ogre2Visual::~Ogre2Visual()
{
}

//////////////////////////////////////////////////
void Ogre2Visual::SetWireframe(bool _show)
{
  if (this->dataPtr->wireframe == _show)
    return;

  if (!this->ogreNode)
    return;

  this->dataPtr->wireframe = _show;
  for (unsigned int i = 0; i < this->ogreNode->numAttachedObjects();
      i++)
  {
    Ogre::MovableObject *obj = this->ogreNode->getAttachedObject(i);
    Ogre::Item *item = dynamic_cast<Ogre::Item *>(obj);

    if (!item)
      continue;

    for (unsigned int j = 0; j < item->getNumSubItems(); j++)
    {
      Ogre::SubItem *subItem = item->getSubItem(j);
      auto datablock = subItem->getDatablock();
      auto macroblock = *(datablock->getMacroblock());

      if (_show)
        macroblock.mPolygonMode = Ogre::PM_WIREFRAME;
      else
        macroblock.mPolygonMode = Ogre::PM_SOLID;

      datablock->setMacroblock(macroblock);
    }
  }
}

//////////////////////////////////////////////////
bool Ogre2Visual::Wireframe() const
{
  return this->dataPtr->wireframe;
}

//////////////////////////////////////////////////
void Ogre2Visual::SetVisible(bool _visible)
{
  if (!this->ogreNode)
    return;

  this->ogreNode->setVisible(_visible);
}

//////////////////////////////////////////////////
void Ogre2Visual::SetVisibilityFlags(uint32_t _flags)
{
  BaseVisual::SetVisibilityFlags(_flags);

  if (!this->ogreNode)
    return;

  for (unsigned int i = 0; i < this->ogreNode->numAttachedObjects(); ++i)
  {
    this->ogreNode->getAttachedObject(i)->setVisibilityFlags(_flags
      & ~Ogre2ParticleEmitter::kParticleVisibilityFlags);
  }
}

//////////////////////////////////////////////////
GeometryStorePtr Ogre2Visual::Geometries() const
{
  return this->geometries;
}

//////////////////////////////////////////////////
bool Ogre2Visual::AttachGeometry(GeometryPtr _geometry)
{
  if (!_geometry)
  {
    ignerr << "Cannot attach null geometry." << std::endl;

    return false;
  }

  if (!this->ogreNode)
  {
    ignerr << "Cannot attach geometry, null Ogre node." << std::endl;
    return false;
  }

  Ogre2GeometryPtr derived =
      std::dynamic_pointer_cast<Ogre2Geometry>(_geometry);

  if (!derived)
  {
    ignerr << "Cannot attach geometry created by another render-engine"
          << std::endl;

    return false;
  }

  Ogre::MovableObject *ogreObj = derived->OgreObject();
  if (!ogreObj)
  {
    ignerr << "Cannot attach a null geometry object" << std::endl;
    return false;
  }

  // set user data for mouse queries
  ogreObj->getUserObjectBindings().setUserAny(
      Ogre::Any(this->Id()));
  ogreObj->setName(this->Name() + "_" + _geometry->Name());
  ogreObj->setVisibilityFlags(this->visibilityFlags
      & ~Ogre2ParticleEmitter::kParticleVisibilityFlags);

  derived->SetParent(this->SharedThis());
  this->ogreNode->attachObject(ogreObj);

  return true;
}

//////////////////////////////////////////////////
bool Ogre2Visual::DetachGeometry(GeometryPtr _geometry)
{
  if (!this->ogreNode)
  {
    ignerr << "Cannot detach geometry, null Ogre node." << std::endl;
    return false;
  }

  Ogre2GeometryPtr derived =
      std::dynamic_pointer_cast<Ogre2Geometry>(_geometry);

  if (!derived)
  {
    ignerr << "Cannot detach geometry created by another render-engine"
          << std::endl;

    return false;
  }

  if (nullptr != derived->OgreObject())
    this->ogreNode->detachObject(derived->OgreObject());
  derived->SetParent(nullptr);
  return true;
}

//////////////////////////////////////////////////
ignition::math::AxisAlignedBox Ogre2Visual::LocalBoundingBox() const
{
  ignition::math::AxisAlignedBox box;
  this->BoundsHelper(box, true /* local frame */);
  return box;
}

//////////////////////////////////////////////////
ignition::math::AxisAlignedBox Ogre2Visual::BoundingBox() const
{
  ignition::math::AxisAlignedBox box;
  this->BoundsHelper(box, false /* world frame */);
  return box;
}

//////////////////////////////////////////////////
void Ogre2Visual::BoundsHelper(ignition::math::AxisAlignedBox &_box,
    bool _local) const
{
  this->BoundsHelper(_box, _local, this->WorldPose());
}

//////////////////////////////////////////////////
void Ogre2Visual::BoundsHelper(ignition::math::AxisAlignedBox &_box,
    bool _local, const ignition::math::Pose3d &_pose) const
{
  if (!this->ogreNode)
    return;

  ignition::math::Vector3d scale = this->WorldScale();

  for (size_t i = 0; i < this->ogreNode->numAttachedObjects(); i++)
  {
    Ogre::MovableObject *obj = this->ogreNode->getAttachedObject(i);

    if (obj->isVisible() && obj->getVisibilityFlags() != IGN_VISIBILITY_GUI)
    {
      Ogre::Aabb bb = obj->getLocalAabb();

      ignition::math::Vector3d min(0, 0, 0);
      ignition::math::Vector3d max(0, 0, 0);
      ignition::math::AxisAlignedBox box(min, max);

      // Ogre does not return a valid bounding box for lights.
      if (obj->getMovableType() == Ogre::LightFactory::FACTORY_TYPE_NAME)
      {
        box.Min() = ignition::math::Vector3d(-0.5, -0.5, -0.5);
        box.Max()  = ignition::math::Vector3d(0.5, 0.5, 0.5);
      }
      else
      {
        Ogre::Vector3 ogreMin = bb.getMinimum();
        Ogre::Vector3 ogreMax = bb.getMaximum();

        // Get ogre bounding boxes and size to object's scale
        min = scale * ignition::math::Vector3d(ogreMin.x, ogreMin.y, ogreMin.z);
        max = scale * ignition::math::Vector3d(ogreMax.x, ogreMax.y, ogreMax.z);
        box.Min() = min,
        box.Max() = max;

        // Assume world transform
        ignition::math::Pose3d transform = _pose;

        // If local frame, calculate transform matrix and set
        if (_local)
        {
          ignition::math::Pose3d worldPose = this->WorldPose();
          ignition::math::Vector3d parentPos = _pose.Pos();
          ignition::math::Quaternion parentRotInv = _pose.Rot().Inverse();
          ignition::math::Pose3d localTransform =
            ignition::math::Pose3d(
                (parentRotInv * (worldPose.Pos() - parentPos)),
                (parentRotInv * worldPose.Rot()));
          transform = localTransform;
        }

        // Transform to world or local space
        box = transformAxisAlignedBox(box, transform);
      }
      _box.Merge(box);
    }
  }

  auto childNodes = std::dynamic_pointer_cast<Ogre2NodeStore>(this->Children());
  if (!childNodes)
    return;

  for (auto it = childNodes->Begin(); it != childNodes->End(); ++it)
  {
    NodePtr child = it->second;
    Ogre2VisualPtr visual = std::dynamic_pointer_cast<Ogre2Visual>(child);
    if (visual)
      visual->BoundsHelper(_box, _local, _pose);
  }
}

//////////////////////////////////////////////////
void Ogre2Visual::Init()
{
  BaseVisual::Init();
  this->geometries = Ogre2GeometryStorePtr(new Ogre2GeometryStore);
}

//////////////////////////////////////////////////
Ogre2VisualPtr Ogre2Visual::SharedThis()
{
  ObjectPtr object = shared_from_this();
  return std::dynamic_pointer_cast<Ogre2Visual>(object);
}
