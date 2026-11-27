/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#ifndef GZ_RENDERING_OGRE2_OGRE2ARROWVISUAL_HH_
#define GZ_RENDERING_OGRE2_OGRE2ARROWVISUAL_HH_

#include "gz/rendering/base/BaseArrowVisual.hh"
#include "gz/rendering/ogre2/Ogre2Visual.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \brief Ogre2.x implementation of the arrow visual class
    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2ArrowVisual :
      public BaseArrowVisual<Ogre2Visual>
    {
      /// \brief Constructor
      protected: Ogre2ArrowVisual();

      /// \brief Destructor
      public: virtual ~Ogre2ArrowVisual();

      /// \brief Only scene can instantiate an arrow visual
      private: friend class Ogre2Scene;
    };
    }
  }
}
#endif
