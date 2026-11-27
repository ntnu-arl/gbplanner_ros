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

#ifndef IGNITION_RENDERING_OGRE_OGREINERTIAVISUAL_HH_
#define IGNITION_RENDERING_OGRE_OGREINERTIAVISUAL_HH_

#include <memory>

#include "ignition/rendering/base/BaseInertiaVisual.hh"
#include "ignition/rendering/ogre/OgreIncludes.hh"
#include "ignition/rendering/ogre/OgreMaterial.hh"
#include "ignition/rendering/ogre/OgreVisual.hh"

namespace Ogre
{
  class MovableObject;
}

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {

    // Forward declaration
    class OgreInertiaVisualPrivate;

    class IGNITION_RENDERING_OGRE_VISIBLE OgreInertiaVisual :
      public BaseInertiaVisual<OgreVisual>
    {
      /// \brief Constructor
      protected: OgreInertiaVisual();

      /// \brief Destructor
      public: virtual ~OgreInertiaVisual();

      // Documentation inherited.
      public: virtual void Init() override;

      // Documentation inherited.
      public: virtual void PreRender() override;

      // Documentation inherited.
      public: Ogre::MovableObject *OgreObject() const;

      /// \brief Load the Inertia visual from its pose and scale
      /// \param[in] _pose Pose of the Inertia visual
      /// \param[in] _scale Scale factor of the box visual
      public: void Load(const ignition::math::Pose3d &_pose,
          const ignition::math::Vector3d &_scale) override;

      /// \brief Get the box visual
      /// \return Pointer to the box visual
      public: VisualPtr BoxVisual() const override;

      // Documentation inherited.
      public: virtual MaterialPtr Material() const override;

      // Documentation inherited.
      public: virtual void SetMaterial(
        MaterialPtr _material, bool _unique) override;

      /// \brief Set material to line geometry.
      /// \param[in] _material Ogre material.
      protected: virtual void SetMaterialImpl(OgreMaterialPtr _material);

      private: friend class OgreScene;

      /// \brief Private data class
      private: std::unique_ptr<OgreInertiaVisualPrivate> dataPtr;
    };
    }
  }
}
#endif
