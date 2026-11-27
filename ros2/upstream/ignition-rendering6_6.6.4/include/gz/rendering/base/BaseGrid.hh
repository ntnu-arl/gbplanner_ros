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
#ifndef GZ_RENDERING_BASE_BASEGRID_HH_
#define GZ_RENDERING_BASE_BASEGRID_HH_

#include <string>
#include "gz/rendering/Grid.hh"
#include "gz/rendering/base/BaseObject.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \brief Base implementation of a grid geometry
    template <class T>
    class BaseGrid :
      public virtual Grid,
      public virtual T
    {
      /// \brief Constructor
      protected: BaseGrid();

      /// \brief Destructor
      public: virtual ~BaseGrid();

      // Documentation inherited.
      public: virtual void PreRender();

      // Documentation inherited.
      public: virtual void Destroy();

      // Documentation inherited.
      public: virtual unsigned int CellCount() const;

      // Documentation inherited.
      public: virtual void SetCellCount(const unsigned int _count);

      // Documentation inherited.
      public: virtual double CellLength() const;

      // Documentation inherited.
      public: virtual void SetCellLength(const double _len);

      // Documentation inherited.
      public: virtual unsigned int VerticalCellCount() const;

      // Documentation inherited.
      public: virtual void SetVerticalCellCount(const unsigned int _count);

      /// \brief Number of cells in grid
      protected: unsigned int cellCount = 10u;

      /// \brief Length of a single cell
      protected: double cellLength = 1.0;

      /// \brief Number of cells in vertical direction
      protected: unsigned int verticalCellCount = 0;

      /// \brief vertical offset of the XY plane from origin
      protected: double heightOffset = 0.0;

      /// \brief Flag to indicate grid properties have changed
      protected: bool gridDirty = false;
    };

    //////////////////////////////////////////////////
    // BaseGrid
    //////////////////////////////////////////////////
    template <class T>
    BaseGrid<T>::BaseGrid()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    BaseGrid<T>::~BaseGrid()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    unsigned int BaseGrid<T>::CellCount() const
    {
      return this->cellCount;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseGrid<T>::SetCellCount(const unsigned int _count)
    {
      this->cellCount = _count;
      this->gridDirty = true;
    }

    //////////////////////////////////////////////////
    template <class T>
    double BaseGrid<T>::CellLength() const
    {
      return this->cellLength;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseGrid<T>::SetCellLength(const double _len)
    {
      this->cellLength = _len;
      this->gridDirty = true;
    }

    //////////////////////////////////////////////////
    template <class T>
    unsigned int BaseGrid<T>::VerticalCellCount() const
    {
      return this->verticalCellCount;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseGrid<T>::SetVerticalCellCount(const unsigned int _count)
    {
      this->verticalCellCount = _count;
      this->gridDirty = true;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseGrid<T>::PreRender()
    {
      T::PreRender();
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseGrid<T>::Destroy()
    {
      T::Destroy();
    }
    }
  }
}
#endif
