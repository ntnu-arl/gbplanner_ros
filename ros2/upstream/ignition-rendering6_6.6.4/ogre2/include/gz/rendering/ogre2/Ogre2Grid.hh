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

#ifndef GZ_RENDERING_OGRE2_OGRE2GRID_HH_
#define GZ_RENDERING_OGRE2_OGRE2GRID_HH_

#include <memory>
#include "gz/rendering/base/BaseGrid.hh"
#include "gz/rendering/ogre2/Ogre2Geometry.hh"

namespace Ogre
{
  class MovableObject;
}

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    // Forward declaration
    class Ogre2GridPrivate;

    /// \brief Ogre2 implementation of a grid geometry.
    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2Grid
      : public BaseGrid<Ogre2Geometry>
    {
      /// \brief Constructor
      protected: Ogre2Grid();

      /// \brief Destructor
      public: virtual ~Ogre2Grid();

      // Documentation inherited.
      public: virtual void Init();

      // Documentation inherited.
      public: virtual Ogre::MovableObject *OgreObject() const;

      // Documentation inherited.
      public: virtual void PreRender();

      // Documentation inherited.
      public: virtual MaterialPtr Material() const;

      // Documentation inherited.
      public: virtual void SetMaterial(MaterialPtr _material, bool _unique);

      /// \brief Set material to grid geometry.
      /// \param[in] _material Ogre material.
      protected: virtual void SetMaterialImpl(Ogre2MaterialPtr _material);

      /// \brief Create the grid geometry in ogre
      private: void Create();

      /// \brief Grid should only be created by scene.
      private: friend class Ogre2Scene;

      /// \brief Private data class
      private: std::unique_ptr<Ogre2GridPrivate> dataPtr;
    };
    }
  }
}
#endif
