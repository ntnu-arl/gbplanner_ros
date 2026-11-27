/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#ifndef IGNITION_RENDERING_BOUNDINGBOX_HH_
#define IGNITION_RENDERING_BOUNDINGBOX_HH_

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include <ignition/common/SuppressWarning.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

#include "ignition/rendering/config.hh"
#include "ignition/rendering/Export.hh"

namespace ignition
{
namespace rendering
{
inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
  class BoundingBoxPrivate;

  /// \brief 2D or 3D Bounding box. It stores the
  /// position / orientation / size info of the box and its label
  class IGNITION_RENDERING_VISIBLE BoundingBox
  {
    /// \brief Constructor
    public: BoundingBox();

    /// \brief Copy constructor
    /// \param[in] _box BoundingBox to copy.
    public: BoundingBox(const BoundingBox &_box);

    /// \brief Move constructor
    /// \param[in] _box BoundingBox to move.
    public: BoundingBox(BoundingBox &&_box) noexcept;

    /// \brief Destructor
    public: virtual ~BoundingBox();

    /// \brief Move assignment operator.
    /// \param[in] _box Heightmap box to move.
    /// \return Reference to this.
    public: BoundingBox &operator=(BoundingBox &&_box);

    /// \brief Copy Assignment operator.
    /// \param[in] _box The heightmap box to set values from.
    /// \return *this
    public: BoundingBox &operator=(const BoundingBox &_box);

    /// \brief Get the center of the bounding box.
    /// \return The center of the bounding box, in (x,y,z) representation.
    /// If this is a 2D bounding box, the z component should be ignored.
    public: const math::Vector3d &Center() const;

    /// \brief Set the center of the bounding box.
    /// \param[in] _center The center of the bounding box, in (x,y,z)
    /// representation. If this is a 2D bounding box, the z component should be
    /// ignored.
    public: void SetCenter(const math::Vector3d &_center);

    /// \brief Get the size of the bounding box.
    /// \return The size of the box (width, height, depth). If this is a 2D
    /// bounding box, the depth should be ignored.
    public: const math::Vector3d &Size() const;

    /// \brief Set the size of the bounding box.
    /// \param[in] _size The size of the box (width, height, depth). If this is
    /// a 2D bounding box, the depth should be ignored.
    public: void SetSize(const math::Vector3d &_size);

    /// \brief Get the orientation of the bounding box.
    /// \return The orientation of the bounding box, in camera coordinates.
    public: const math::Quaterniond &Orientation() const;

    /// \brief Set the orientation of the bounding box.
    /// \param[in] _orientation The orientation of the bounding box, in camera
    /// coordinates.
    public: void SetOrientation(const math::Quaterniond &_orientation);

    /// \brief Get the vertices of the 3D bounding box representation.
    /// \return The vertices, in the following order:
    ///
    ///    1 -------- 0
    ///   /|         /|
    ///  2 -------- 3 .
    ///  | |        | |
    ///  . 5 -------- 4
    ///  |/         |/
    ///  6 -------- 7
    public: const std::vector<math::Vector3d> &Vertices3D() const;

    /// \brief Get the label of the bounding box.
    /// \return The label of the bounding box.
    public: uint32_t Label() const;

    /// \brief Set the label of the bounding box.
    /// \param[in] _label The label of the bounding box.
    public: void SetLabel(uint32_t _label);

    /// \internal
    /// \brief Private data
    IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
    private: std::unique_ptr<BoundingBoxPrivate> dataPtr;
    IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING
  };
}
}
}
#endif
