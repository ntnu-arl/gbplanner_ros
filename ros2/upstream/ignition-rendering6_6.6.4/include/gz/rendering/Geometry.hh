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
#ifndef GZ_RENDERING_GEOMETRY_HH_
#define GZ_RENDERING_GEOMETRY_HH_

#include <string>
#include "gz/rendering/config.hh"
#include "gz/rendering/RenderTypes.hh"
#include "gz/rendering/Object.hh"
#include "gz/rendering/Export.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \class Geometry Geometry.hh ignition/rendering/Geometry.hh
    /// \brief Represents a geometric shape to be rendered
    class IGNITION_RENDERING_VISIBLE Geometry :
      public virtual Object
    {
      /// \brief Deconstructor
      public: virtual ~Geometry() { }

      /// \brief Determine if this Geometry is attached to a Visual
      /// \return True if this Geometry has a parent Visual
      public: virtual bool HasParent() const = 0;

      /// \brief Get the parent Visual
      /// \return the parent Visual
      public: virtual VisualPtr Parent() const = 0;

      /// \brief Detach this Geometry from its parent Visual. If this
      /// Geometry does not have a parent, no work will be done.
      public: virtual void RemoveParent() = 0;

      /// \brief Set the materials of this Geometry. The specified material
      /// will be retrieved from the parent Scene. If no material is registered
      /// by the given name, no work will be done.
      /// \param[in] _name Name of registered Material
      /// \param[in] _unique True if the specified material should be cloned
      public: virtual void SetMaterial(const std::string &_name,
                  bool _unique = true) = 0;

      /// \brief Set the materials of this Geometry
      /// \param[in] _material New Material to be assigned
      /// \param[in] _unique True if the given material should be cloned
      public: virtual void SetMaterial(MaterialPtr _material,
                  bool _unique = true) = 0;

      /// \brief Get the material of this geometry
      /// \return Material used by this geometry
      public: virtual MaterialPtr Material() const = 0;

      /// \brief Clone the geometry.
      /// \return The cloned geometry.
      public: virtual GeometryPtr Clone() const = 0;
    };
    }
  }
}
#endif
