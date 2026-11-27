/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef GZ_RENDERING_OGRE_OGREOBJECT_HH_
#define GZ_RENDERING_OGRE_OGREOBJECT_HH_

#include <gz/common/SuppressWarning.hh>

#include "gz/rendering/base/BaseObject.hh"
#include "gz/rendering/ogre/OgreRenderTypes.hh"
#include "gz/rendering/ogre/Export.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    class IGNITION_RENDERING_OGRE_VISIBLE OgreObject :
      public BaseObject
    {
      protected: OgreObject();

      public: virtual ~OgreObject();

      public: virtual ScenePtr Scene() const;

      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      protected: OgreScenePtr scene;
      IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING

      private: friend class OgreScene;
    };
    }
  }
}
#endif
