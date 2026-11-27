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
#ifndef GZ_RENDERING_GRID_HH_
#define GZ_RENDERING_GRID_HH_

#include <string>
#include "gz/rendering/config.hh"
#include "gz/rendering/Geometry.hh"
#include "gz/rendering/Object.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \class Grid Grid.hh ignition/rendering/Grid
    /// \brief Represents a grid geometry drawn along the XY plane.
    /// If vertical cell count is specified then the grid becomes 3D.
    class IGNITION_RENDERING_VISIBLE Grid :
      public virtual Geometry
    {
      /// \brief Destructor
      public: virtual ~Grid() { }

      /// \brief Set the number of cells on a planar grid
      /// \param[in] _count The number of cells
      public: virtual void SetCellCount(const unsigned int _count) = 0;

      /// \brief Get the number of cells on a planar grid
      /// \return Number of cells
      public: virtual unsigned int CellCount() const = 0;

      /// \brief Set the cell length in one direction.
      /// The cells are assumed to be square.
      /// \param[in] _len The cell length
      public: virtual void SetCellLength(const double _len) = 0;

      /// \brief Get the cell length
      /// \return The cell length
      public: virtual double CellLength() const = 0;

      /// \brief Set the vertical cell count of the grid. This makes the grid 3D
      /// \param[in] _count Number of cells in vertical direction.
      public: virtual void SetVerticalCellCount(const unsigned int _count) = 0;

      /// \brief Get the vertical cell count
      /// \return The vertical cell count.
      public: virtual unsigned int VerticalCellCount() const = 0;
    };
    }
  }
}

#endif
