/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#ifndef IGNITION_RENDERING_HEIGHTMAP_HH_
#define IGNITION_RENDERING_HEIGHTMAP_HH_

#include "ignition/rendering/config.hh"
#include "ignition/rendering/Geometry.hh"
#include "ignition/rendering/HeightmapDescriptor.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \class Heightmap Heightmap.hh ignition/rendering/Heightmap
    /// \brief A terrain defined by a heightfield.
    class Heightmap :
      public virtual Geometry
    {
      /// \brief Get the immutable heightmap descriptor.
      /// \return Descriptor with heightmap information.
      public: virtual const HeightmapDescriptor &Descriptor() = 0;
    };
    }
  }
}
#endif
