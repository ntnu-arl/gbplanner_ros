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
#ifndef GZ_RENDERING_OPTIX_OPTIXGEOMETRY_HH_
#define GZ_RENDERING_OPTIX_OPTIXGEOMETRY_HH_

#include "gz/rendering/base/BaseGeometry.hh"
#include "gz/rendering/optix/OptixIncludes.hh"
#include "gz/rendering/optix/OptixObject.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    class IGNITION_RENDERING_OPTIX_VISIBLE OptixGeometry :
      public BaseGeometry<OptixObject>
    {
      protected: OptixGeometry();

      public: virtual ~OptixGeometry();

      public: virtual bool HasParent() const;

      public: virtual VisualPtr Parent() const;

      public: virtual optix::GeometryGroup OptixGeometryGroup() const = 0;

      protected: virtual void SetParent(OptixVisualPtr _parent);

      protected: virtual void SetScale(math::Vector3d _scale);

      protected: OptixVisualPtr parent;

      private: friend class OptixVisual;
    };
    }
  }
}
#endif
