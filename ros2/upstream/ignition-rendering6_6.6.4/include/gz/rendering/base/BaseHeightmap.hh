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
#ifndef IGNITION_RENDERING_BASE_BASEHEIGHTMAP_HH_
#define IGNITION_RENDERING_BASE_BASEHEIGHTMAP_HH_

#include "ignition/rendering/Heightmap.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    //////////////////////////////////////////////////
    template <class T>
    class BaseHeightmap :
      public virtual Heightmap,
      public virtual T
    {
      /// \brief Constructor
      /// \param[in] _desc Descriptor containing heightmap information.
      protected: explicit BaseHeightmap(const HeightmapDescriptor &_desc);

      // Documentation inherited
      public: virtual void PreRender() override;

      // Documentation inherited
      public: virtual void Destroy() override;

      // Documentation inherited
      public: virtual const HeightmapDescriptor &Descriptor() override;

      /// \brief Descriptor containing heightmap information
      public: HeightmapDescriptor descriptor;
    };

    //////////////////////////////////////////////////
    template <class T>
    BaseHeightmap<T>::BaseHeightmap(const HeightmapDescriptor &_desc)
        : descriptor{_desc}
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseHeightmap<T>::PreRender()
    {
      T::PreRender();
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseHeightmap<T>::Destroy()
    {
      T::Destroy();
    }

    //////////////////////////////////////////////////
    template <class T>
    const HeightmapDescriptor &BaseHeightmap<T>::Descriptor()
    {
      return this->descriptor;
    }
    }
  }
}
#endif
