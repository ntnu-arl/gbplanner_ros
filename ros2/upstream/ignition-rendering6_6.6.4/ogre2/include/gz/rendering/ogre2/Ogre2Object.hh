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
#ifndef GZ_RENDERING_OGRE2_OGRE2OBJECT_HH_
#define GZ_RENDERING_OGRE2_OGRE2OBJECT_HH_

#include "gz/rendering/config.hh"
#include "gz/rendering/base/BaseObject.hh"
#include "gz/rendering/ogre2/Ogre2RenderTypes.hh"
#include "gz/rendering/ogre2/Export.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \brief Ogre2.x implementation of the Object class
    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2Object :
      public BaseObject
    {
      /// \brief Constructor
      protected: Ogre2Object();

      /// \brief Destructor
      public: virtual ~Ogre2Object();

      // Documentation inherited
      public: virtual ScenePtr Scene() const override;

      /// \brief Pointer to the ogre scene
      protected: Ogre2ScenePtr scene;

      /// \brief Make ogre scene our friend so it is able to create objects
      private: friend class Ogre2Scene;
    };
    }
  }
}
#endif
