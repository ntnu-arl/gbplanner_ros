/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#ifndef GZ_RENDERING_RAYQUERY_HH_
#define GZ_RENDERING_RAYQUERY_HH_

#include <gz/common/SuppressWarning.hh>
#include <gz/math/Vector3.hh>

#include "gz/rendering/config.hh"
#include "gz/rendering/Camera.hh"
#include "gz/rendering/Scene.hh"
#include "gz/rendering/Visual.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \brief A class that stores ray query intersection results.
    class IGNITION_RENDERING_VISIBLE RayQueryResult
    {
      /// \brief Intersection distance
      public: double distance = -1;

      /// \brief Intersection point in 3d space
      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      public: math::Vector3d point;
      IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING

      /// \brief Intersected object id
      public: unsigned int objectId = 0;

      /// \brief Returns false if result is not valid
      public: operator bool() const
              {
                return distance > 0;
              }

      /// \brief Returns false if result is not valid
      public: operator bool()
              {
                return distance > 0;
              }
    };

    /// \class RayQuery RayQuery.hh ignition/rendering/RayQuery.hh
    /// \brief A Ray Query class used for computing ray object intersections
    class IGNITION_RENDERING_VISIBLE RayQuery
        : public virtual Object
    {
      /// \brief Destructor
      public: virtual ~RayQuery() { }

      /// \brief Set ray origin
      /// \param[in] _origin Ray origin
      public: virtual void SetOrigin(const math::Vector3d &_origin) = 0;

      /// \brief Get ray origin
      /// \return Ray origin
      public: virtual math::Vector3d Origin() const = 0;

      /// \brief Set ray direction
      /// \param[in] _dir Ray origin
      public: virtual void SetDirection(const math::Vector3d &_dir) = 0;

      /// \brief Get ray direction
      /// \return Ray direction.
      public: virtual math::Vector3d Direction() const = 0;

      /// \brief Create the ray query from camera
      /// \param[in] _camera Camera to construct ray
      /// \param[in] _coord normalized device coords [-1, +1]
      public: virtual void SetFromCamera(const CameraPtr &_camera,
                const math::Vector2d &_coord) = 0;

      /// \brief Compute intersections
      /// \return A vector of intersection results
      public: virtual RayQueryResult ClosestPoint() = 0;
    };
    }
  }
}
#endif
