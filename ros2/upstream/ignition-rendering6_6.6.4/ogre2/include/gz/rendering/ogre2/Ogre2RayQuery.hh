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
#ifndef GZ_RENDERING_OGRE2_OGRE2RAYQUERY_HH_
#define GZ_RENDERING_OGRE2_OGRE2RAYQUERY_HH_

#include <memory>

#include "gz/rendering/base/BaseRayQuery.hh"
#include "gz/rendering/ogre2/Ogre2Object.hh"
#include "gz/rendering/ogre2/Ogre2RenderTypes.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    // forward declaration
    class Ogre2RayQueryPrivate;

    /// \class Ogre2RayQuery Ogre2RayQuery.hh
    /// ignition/rendering/ogre2/Ogre2RayQuery.hh
    /// \brief A Ray Query class used for computing ray object intersections
    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2RayQuery :
        public BaseRayQuery<Ogre2Object>
    {
      /// \brief Constructor
      protected: Ogre2RayQuery();

      /// \brief Destructor
      public: virtual ~Ogre2RayQuery();

      // Documentation inherited
      public: virtual void SetFromCamera(const CameraPtr &_camera,
                const math::Vector2d &_coord);

      // Documentation inherited
      public: virtual RayQueryResult ClosestPoint();

      /// \brief Get closest point by selection buffer.
      /// This is executed on the GPU.
      private: RayQueryResult ClosestPointBySelectionBuffer();

      /// \brief Get closest point by ray triangle intersection test.
      /// This is executed on the CPU.
      private: RayQueryResult ClosestPointByIntersection();

      /// \brief Private data pointer
      private: std::unique_ptr<Ogre2RayQueryPrivate> dataPtr;

      /// \brief Pointer to friend scene class for creating ray query
      private: friend class Ogre2Scene;
    };
    }
  }
}
#endif
