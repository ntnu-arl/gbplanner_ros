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
#ifndef GZ_RENDERING_BASE_BASERAYQUERY_HH_
#define GZ_RENDERING_BASE_BASERAYQUERY_HH_

#include <gz/math/Matrix4.hh>
#include <gz/math/Vector3.hh>

#include "gz/rendering/RayQuery.hh"
#include "gz/rendering/Scene.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \class BaseRayQuery BaseRayQuery.hh
    /// ignition/rendering/base/BaseRayQuery.hh
    /// \brief A Ray Query class used for computing ray object intersections
    template <class T>
    class BaseRayQuery :
        public virtual RayQuery,
        public T
    {
      /// \brief Constructor
      protected: BaseRayQuery();

      /// \brief Destructor
      public: virtual ~BaseRayQuery() override;

      // Documentation inherited
      public: virtual void SetOrigin(const math::Vector3d &_origin) override;

      // Documentation inherited
      public: virtual math::Vector3d Origin() const override;

      // Documentation inherited
      public: virtual void SetDirection(const math::Vector3d &_dir) override;

      // Documentation inherited
      public: virtual math::Vector3d Direction() const override;

      // Documentation inherited
      public: virtual void SetFromCamera(const CameraPtr &_camera,
                const math::Vector2d &_coord) override;

      // Documentation inherited
      public: virtual RayQueryResult ClosestPoint() override;

      /// \brief Ray origin
      protected: math::Vector3d origin;

      /// \brief Ray direction
      protected: math::Vector3d direction;
    };

    //////////////////////////////////////////////////
    template <class T>
    BaseRayQuery<T>::BaseRayQuery()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    BaseRayQuery<T>::~BaseRayQuery()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseRayQuery<T>::SetOrigin(const math::Vector3d &_origin)
    {
      this->origin = _origin;
    }

    //////////////////////////////////////////////////
    template <class T>
    gz::math::Vector3d BaseRayQuery<T>::Origin() const
    {
      return this->origin;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseRayQuery<T>::SetDirection(const math::Vector3d &_dir)
    {
      this->direction = _dir;
    }

    //////////////////////////////////////////////////
    template <class T>
    gz::math::Vector3d BaseRayQuery<T>::Direction() const
    {
      return this->direction;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseRayQuery<T>::SetFromCamera(const CameraPtr &_camera,
        const gz::math::Vector2d &_coord)
    {
      math::Matrix4d projectionMatrix = _camera->ProjectionMatrix();
      math::Matrix4d viewMatrix = _camera->ViewMatrix();
      math::Vector3d start(_coord.X(), _coord.Y(), -1.0);
      math::Vector3d end(_coord.X(), _coord.Y(), 0.0);
      math::Matrix4d viewProjInv = (projectionMatrix * viewMatrix).Inverse();

      // rotate start and end
      // ign math does not support matrix4 * vec4
      // so calc homogeneous coordinate w ourselves
      double startw = viewProjInv(3, 0) * start[0] +
                      viewProjInv(3, 1) * start[1] +
                      viewProjInv(3, 2) * start[2] + viewProjInv(3, 3);
      double endw = viewProjInv(3, 0) * end[0] +
                    viewProjInv(3, 1) * end[1] +
                    viewProjInv(3, 2) * end[2] + viewProjInv(3, 3);
      start = viewProjInv * start;
      end = viewProjInv * end;
      // normalize
      start = start / startw;
      end = end / endw;
      math::Vector3d dir = (end - start).Normalize();

      this->origin = start;
      this->direction = dir;
    }

    //////////////////////////////////////////////////
    template <class T>
    RayQueryResult BaseRayQuery<T>::ClosestPoint()
    {
      // TODO(anyone): implement a generic ray query here?
      RayQueryResult result;
      result.distance = -1;
      return result;
    }
    }
  }
}
#endif
