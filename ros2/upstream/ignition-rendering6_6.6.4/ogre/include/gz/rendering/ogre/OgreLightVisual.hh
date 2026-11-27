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

#ifndef IGNITION_RENDERING_OGRE_OGRELIGHTVISUAL_HH_
#define IGNITION_RENDERING_OGRE_OGRELIGHTVISUAL_HH_

#include <memory>

#include "ignition/rendering/base/BaseLightVisual.hh"
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
    class OgreLightVisualPrivate;

    class IGNITION_RENDERING_OGRE_VISIBLE OgreLightVisual :
      public BaseLightVisual<OgreVisual>
    {
      /// \brief Constructor
      protected: OgreLightVisual();

      /// \brief Destructor
      public: virtual ~OgreLightVisual();

      // Documentation inherited.
      public: virtual void Init() override;

      // Documentation inherited.
      public: virtual void PreRender() override;

      // Documentation inherited.
      public: Ogre::MovableObject *OgreObject() const;

      /// \brief Create the Light Visual in Ogre
      public: void CreateVisual();

      // Documentation inherited.
      public: virtual MaterialPtr Material() const override;

      // Documentation inherited.
      public: virtual void SetMaterial(
        MaterialPtr _material, bool _unique) override;

      /// \brief Set material to grid geometry.
      /// \param[in] _material Ogre material.
      protected: virtual void SetMaterialImpl(OgreMaterialPtr _material);

      private: friend class OgreScene;

      /// \brief Private data class
      private: std::unique_ptr<OgreLightVisualPrivate> dataPtr;
    };
    }
  }
}
#endif
