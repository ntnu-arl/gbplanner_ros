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
#ifndef GZ_RENDERING_OPTIX_OPTIXVISUAL_HH_
#define GZ_RENDERING_OPTIX_OPTIXVISUAL_HH_

#include "gz/rendering/base/BaseVisual.hh"
#include "gz/rendering/optix/OptixNode.hh"
#include "gz/rendering/optix/OptixRenderTypes.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    class IGNITION_RENDERING_OPTIX_VISIBLE OptixVisual :
      public BaseVisual<OptixNode>
    {
      protected: OptixVisual();

      public: virtual ~OptixVisual();

      public: virtual optix::Group OptixGroup() const;

      public: virtual optix::Acceleration OptixAccel() const;

      public: virtual void PreRender();

      protected: virtual GeometryStorePtr Geometries() const;

      protected: virtual bool AttachGeometry(GeometryPtr _geometry);

      protected: virtual bool DetachGeometry(GeometryPtr _geometry);

      protected: virtual void Init();

      private: void CreateStorage();

      private: OptixVisualPtr SharedThis();

      protected: OptixGeometryStorePtr geometries;

      private: friend class OptixScene;
    };
    }
  }
}
#endif
