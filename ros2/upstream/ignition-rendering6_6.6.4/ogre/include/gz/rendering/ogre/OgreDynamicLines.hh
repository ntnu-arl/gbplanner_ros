/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#ifndef GZ_RENDERING_OGRE_OGREDYNAMICLINES_HH_
#define GZ_RENDERING_OGRE_OGREDYNAMICLINES_HH_

#include <memory>
#include <vector>
#include <string>
#include <gz/math/Color.hh>

#include "gz/rendering/ogre/Export.hh"
#include "gz/rendering/ogre/OgreConversions.hh"
#include "gz/rendering/ogre/OgreDynamicRenderable.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    /// \brief Forward declaration
    class OgreDynamicLinesPrivate;

    /*  \class OgreDynamicLines OgreDynamicLines.hh \
     *  ignition/rendering/ogre/OgreDynamicLines.hh
     */
    /// \brief Class for drawing lines that can change
    class IGNITION_RENDERING_OGRE_VISIBLE OgreDynamicLines :
      public OgreDynamicRenderable
    {
      /// \brief Constructor
      /// \param[in] _opType The type of Line
      public: explicit OgreDynamicLines(
                  MarkerType _opType = MT_LINE_STRIP);

      /// \brief Destructor
      public: virtual ~OgreDynamicLines();

      /// \brief Add a point to the point list
      /// \param[in] _pt gz::math::Vector3d point
      /// \param[in] _color gz::math::Color Point color
      public: void AddPoint(const gz::math::Vector3d &_pt,
            const gz::math::Color &_color = gz::math::Color::White);

      /// \brief Add a point to the point list.
      /// \param[in] _x X position
      /// \param[in] _y Y position
      /// \param[in] _z Z position
      /// \param[in] _color gz::math::Color Point color
      public: void AddPoint(const double _x, const double _y, const double _z,
            const gz::math::Color &_color = gz::math::Color::White);

      /// \brief Change the location of an existing point in the point list
      /// \param[in] _index Index of the point to set
      /// \param[in] _value gz::math::Vector3d value to set the point to
      public: void SetPoint(unsigned int _index,
                  const gz::math::Vector3d &_value);

      /// \brief Change the color of an existing point in the point list
      /// \param[in] _index Index of the point to set
      /// \param[in] _color gz::math::Color Pixelcolor color to set the
      /// point to
      public: void SetColor(unsigned int _index,
                            const gz::math::Color &_color);

      /// \brief Return the location of an existing point in the point list
      /// \param[in] _index Number of the point to return
      /// \return gz::math::Vector3d value of the point. A vector of
      /// [gz::math::INF_D, gz::math::INF_D, gz::math::INF_D]
      /// is returned when then the _index is out of bounds.
      /// gz::math::INF_D==std::numeric_limits<double>::infinity()
      public: gz::math::Vector3d Point(unsigned int _index) const;

      /// \brief Return the total number of points in the point list
      /// \return Number of points
      public: unsigned int PointCount() const;

      /// \brief Remove all points from the point list
      public: void Clear();

      /// \brief Call this to update the hardware buffer after making changes.
      public: void Update();

      /// \brief Implementation DynamicRenderable,
      /// creates a simple vertex-only decl
      private: virtual void CreateVertexDeclaration();

      /// \brief Implementation DynamicRenderable, pushes point
      /// list out to hardware memory
      private: virtual void FillHardwareBuffers();

      /// \brief private implementation
      private: std::unique_ptr<OgreDynamicLinesPrivate> dataPtr;
    };
    }
  }
}
#endif
