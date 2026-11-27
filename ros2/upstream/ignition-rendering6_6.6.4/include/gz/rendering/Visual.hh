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
#ifndef GZ_RENDERING_VISUAL_HH_
#define GZ_RENDERING_VISUAL_HH_

#include <string>
#include <gz/math/AxisAlignedBox.hh>
#include "gz/rendering/config.hh"
#include "gz/rendering/Node.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    /// \class Visual Visual.hh ignition/rendering/Visual.hh
    /// \brief Represents a visual node in a scene graph. A Visual is the only
    /// node that can have Geometry and other Visual children.
    class IGNITION_RENDERING_VISIBLE Visual :
      public virtual Node
    {
      /// \brief Deconstructor
      public: virtual ~Visual() { }

      /// \brief Get the number of geometries attached to this visual
      /// \return The number of geometries attached to this visual
      public: virtual unsigned int GeometryCount() const = 0;

      /// \brief Determine if given geometry is attached to this visual
      /// \param[in] _geometry Geometry in question
      /// \return True if given node is attached to this visual
      public: virtual bool HasGeometry(ConstGeometryPtr _geometry) const = 0;

      /// \brief Get geometry at given index. If no geometry exists at given
      /// index, NULL will be returned.
      /// \param[in] _index Index of the desired geometry
      /// \return The specified geometry
      public: virtual GeometryPtr GeometryByIndex(
                  unsigned int _index) const = 0;

      /// \brief Add the given geometry to this visual. If the given node is
      /// already attached, no work will be done.
      /// \param[in] _geometry Geometry to be added
      public: virtual void AddGeometry(GeometryPtr _geometry) = 0;

      /// \brief Remove the given geometry from this visual. If the given node
      /// is not a child of this visual, no work will be done.
      /// \param[in] _geometry Geometry to be removed
      /// \return The removed geometry
      public: virtual GeometryPtr RemoveGeometry(GeometryPtr _geometry) = 0;

      /// \brief Remove the geometry at the given index from this visual. If the
      /// specified node is not attached this visual, no work will be done.
      /// \param[in] _index Index of the geometry to be removed
      /// \return The removed geometry
      public: virtual GeometryPtr RemoveGeometryByIndex(
                  unsigned int _index) = 0;

      /// \brief Remove all attached geometries from this visual
      public: virtual void RemoveGeometries() = 0;

      /// \brief Set the material for all attached visuals and geometries. The
      /// specified material will be retrieved from Scene::Material using
      /// the given material name. If no material is registered with the given
      /// name, no work will be done.
      /// \param[in] _name Name of the material to be assigned
      /// \param[in] _unique True if the specified material should be cloned
      public: virtual void SetMaterial(const std::string &_name,
                  bool _unique = true) = 0;

      /// \brief Set the material for all attached visuals and geometries
      /// \param[in] _material Name of the material to be assigned
      /// \param[in] _unique True if the specified material should be cloned
      public: virtual void SetMaterial(MaterialPtr _material,
                  bool _unique = true) = 0;

      /// \brief Set the material for all attached visuals only
      /// \param[in] _material Name of the material to be assigned
      /// \param[in] _unique True if the specified material should be cloned
      public: virtual void SetChildMaterial(MaterialPtr _material,
                  bool _unique = true) = 0;

      /// \brief Set the material for all attached geometries only
      /// \param[in] _material Name of the material to be assigned
      /// \param[in] _unique True if the specified material should be cloned
      public: virtual void SetGeometryMaterial(MaterialPtr _material,
                  bool _unique = true) = 0;

      /// \brief Get the material assigned to attached visuals and geometries.
      /// \return the Pointer to the material assigned to this visual. If the
      /// material is cloned at the time it is set to this visual, the cloned
      /// material will be returned.
      public: virtual MaterialPtr Material() const = 0;

      /// \brief Enable or disable wireframe
      /// \param[in] _show True to enable wireframe
      public: virtual void SetWireframe(bool _show) = 0;

      /// \brief Get whether wireframe is enabled for this visual.
      /// \return True if wireframe is enabled for this visual.
      public: virtual bool Wireframe() const = 0;

      /// \brief Specify if this visual is visible
      /// \param[in] _visible True if this visual should be made visible
      public: virtual void SetVisible(bool _visible) = 0;

      /// \brief Set visibility flags
      /// \param[in] _flags Visibility flags
      public: virtual void SetVisibilityFlags(uint32_t _flags) = 0;

      /// \brief Get visibility flags
      /// \return visibility flags
      public: virtual uint32_t VisibilityFlags() const = 0;

      /// \brief Add visibility flags
      /// \param[in] _flags Visibility flags
      public: virtual void AddVisibilityFlags(uint32_t _flags) = 0;

      /// \brief Remove visibility flags
      /// \param[in] _flags Visibility flags
      public: virtual void RemoveVisibilityFlags(uint32_t _flags) = 0;

      /// \brief Get the bounding box in world frame coordinates.
      /// \return The axis aligned bounding box
      public: virtual ignition::math::AxisAlignedBox BoundingBox() const = 0;

      /// \brief Get the local bounding box of the visual.
      /// \return The local bounding box
      public: virtual ignition::math::AxisAlignedBox LocalBoundingBox()
              const = 0;

      /// \brief Clone the visual (and its children) with a new name.
      /// \param[in] _name Name of the cloned Visual. Set this to an empty
      /// string to auto-generate a unique name for the cloned visual.
      /// \param[in] _newParent Parent of the cloned Visual. Set to nullptr if
      /// the cloned visual should have no parent.
      /// \return The visual. nullptr is returned if cloning failed.
      public: virtual VisualPtr Clone(const std::string &_name,
                  NodePtr _newParent) const = 0;
    };
    }
  }
}
#endif
