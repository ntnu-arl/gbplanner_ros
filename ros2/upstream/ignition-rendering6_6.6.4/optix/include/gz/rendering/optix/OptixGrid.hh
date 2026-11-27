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
#ifndef GZ_RENDERING_OPTIX_OPTIXGRID_HH_
#define GZ_RENDERING_OPTIX_OPTIXGRID_HH_

#include <string>
#include <vector>
#include "gz/rendering/base/BaseGrid.hh"
#include "gz/rendering/optix/OptixPrimitive.hh"
#include "gz/rendering/optix/OptixObject.hh"
#include "gz/rendering/optix/OptixRenderTypes.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// TODO Not implemented yet!
    /// \brief Optix implementation of a grid geometry
    class IGNITION_RENDERING_OPTIX_VISIBLE OptixGrid
      : public BaseGrid<OptixPrimitive>
    {
      /// \brief Constructor
      protected: OptixGrid();

      /// \brief Destructor
      public: virtual ~OptixGrid();

      // Documentation inherited.
      public: virtual void PreRender();

      /// \brief Grids can only be created by scene
      private: friend class OptixScene;
    };
    }
  }
}
#endif
