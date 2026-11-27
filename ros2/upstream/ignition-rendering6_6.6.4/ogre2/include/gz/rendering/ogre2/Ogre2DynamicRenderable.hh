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
#ifndef GZ_RENDERING_OGRE2_OGRE2DYNAMICRENDERABLE_HH_
#define GZ_RENDERING_OGRE2_OGRE2DYNAMICRENDERABLE_HH_

#include <memory>
#include <string>
#include <vector>

#include "gz/rendering/ogre2/Export.hh"
#include "gz/rendering/ogre2/Ogre2RenderTypes.hh"
#include "gz/rendering/Marker.hh"

#ifdef _MSC_VER
  #pragma warning(push, 0)
#endif
#include <OgreHlmsPso.h>
#ifdef _MSC_VER
  #pragma warning(pop)
#endif

namespace Ogre
{
  class MovableObject;
}

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    // forward declarations
    class Ogre2DynamicRenderablePrivate;

    /*  \class Ogre2DynamicRenderable Ogre2DynamicRenderable.hh \
     *  ignition/rendering/ogre2/Ogre2DynamicRenderable.hh
     */
    /// \brief Dynamic renderable class that manages hardware buffers for
    /// a dynamic geometry
    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2DynamicRenderable
    {
      /// \brief Constructor
      /// \param[in] _scene Pointer to scene
      public: explicit Ogre2DynamicRenderable(ScenePtr _scene);

      /// \brief Virtual destructor
      public: virtual ~Ogre2DynamicRenderable();

      /// \brief Set the render operation type
      /// \param[in] _opType The type of render operation to perform.
      public: void SetOperationType(MarkerType _opType);

      /// \brief Get the render operation type
      /// \return The render operation type.
      public: MarkerType OperationType() const;

      /// \brief Update the dynamic renderable
      public: void Update();

      /// \brief Get the ogre object associated with this dynamic renderable
      public: Ogre::MovableObject *OgreObject() const;

      /// \brief Add a point to the point list
      /// \param[in] _pt gz::math::Vector3d point
      /// \param[in] _color gz::math::Color Point color
      public: void AddPoint(const gz::math::Vector3d &_pt,
            const gz::math::Color &_color = gz::math::Color::White);

      /// \brief Add a point to the point list.
      /// \param[in] _x X position
      /// \param[in] _y Y position
      /// \param[in] _z Z position
      /// \param[in] _color Point color
      public: void AddPoint(const double _x, const double _y, const double _z,
            const gz::math::Color &_color = gz::math::Color::White);

      /// \brief Change the location of an existing point in the point list
      /// \param[in] _index Index of the point to set
      /// \param[in] _value Position of the point
      public: void SetPoint(unsigned int _index,
                            const gz::math::Vector3d &_value);

      /// \brief Change the color of an existing point in the point list
      /// \param[in] _index Index of the point to set
      /// \param[in] _color color to set the point to
      public: void SetColor(unsigned int _index,
                            const gz::math::Color &_color);

      /// \brief Return the position of an existing point in the point list
      /// \param[in] _index Get the point at this index
      /// \return position of point. A vector of
      /// [gz::math::INF_D, gz::math::INF_D, gz::math::INF_D]
      /// is returned when then the _index is out of bounds.
      /// gz::math::INF_D==std::numeric_limits<double>::infinity()
      public: gz::math::Vector3d Point(unsigned int _index) const;

      /// \brief Return the total number of points in the point list
      /// \return Number of points
      public: unsigned int PointCount() const;

      /// \brief Remove all points from the point list
      public: void Clear();

      /// \brief Destroy the dynamic renderable
      public: void Destroy();

      /// \brief Set the material for this dynamic renderable
      /// \param[in] _material New Material to be assigned
      /// \param[in] _unique True if the given material should be cloned
      public: void SetMaterial(MaterialPtr _material, bool _unique = true);

      /// \brief Create the dynamic mesh
      private: void CreateDynamicMesh();

      /// \brief Update vertex buffer if vertices have changes
      private: void UpdateBuffer();

      /// \brief Helper function to generate normals
      /// \param[in] _opType Ogre render operation type
      /// \param[in] _vertices a list of vertices
      /// \param[in,out] _vbuffer vertex buffer to be filled
      private: void GenerateNormals(Ogre::OperationType _opType,
          const std::vector<math::Vector3d> &_vertices, float *_vbuffer);

      /// \brief Destroy the vertex buffer
      private: void DestroyBuffer();

      /// \brief Pointer to private data
      private: std::unique_ptr<Ogre2DynamicRenderablePrivate> dataPtr;
    };
    }
  }
}
#endif
