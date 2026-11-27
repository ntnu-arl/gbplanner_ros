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
#ifndef GZ_RENDERING_OPTIX_OPTIXARROWVISUAL_HH_
#define GZ_RENDERING_OPTIX_OPTIXARROWVISUAL_HH_

#include "gz/rendering/base/BaseArrowVisual.hh"
#include "gz/rendering/optix/OptixVisual.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    class IGNITION_RENDERING_OPTIX_VISIBLE OptixArrowVisual :
      public BaseArrowVisual<OptixVisual>
    {
      protected: OptixArrowVisual();

      public: virtual ~OptixArrowVisual();

      private: friend class OptixScene;
    };
    }
  }
}
#endif
