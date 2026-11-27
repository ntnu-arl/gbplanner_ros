/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#ifndef GZ_RENDERING_OGRE2_OGRE2VISUAL_HH_
#define GZ_RENDERING_OGRE2_OGRE2VISUAL_HH_

#include <memory>
#include <vector>

#include "gz/rendering/base/BaseVisual.hh"
#include "gz/rendering/ogre2/Ogre2Node.hh"
#include "gz/rendering/ogre2/Ogre2RenderTypes.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    // forward declaration
    class Ogre2VisualPrivate;

    /// \brief Ogre2.x implementation of the visual class
    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2Visual :
      public BaseVisual<Ogre2Node>
    {
      /// \brief Constructor
      protected: Ogre2Visual();

      /// \brief Destructor
      public: virtual ~Ogre2Visual();

      // Documentation inherited
      public: virtual void SetWireframe(bool _show) override;

      // Documentation inherited
      public: virtual bool Wireframe() const override;

      // Documentation inherited.
      public: virtual void SetVisible(bool _visible) override;

      // Documentation inherited.
      public: virtual void SetVisibilityFlags(uint32_t _flags) override;

      // Documentation inherited.
      public: virtual ignition::math::AxisAlignedBox BoundingBox()
                  const override;

      // Documentation inherited.
      public: virtual ignition::math::AxisAlignedBox LocalBoundingBox()
                  const override;

      /// \brief Recursively loop through this visual's children
      /// to obtain the bounding box.
      /// \param[in,out] _box The bounding box.
      /// \param[in] _local A flag indicating if the local bounding box is to
      /// be calculated.
      /// \param[in] _pose World pose of the visual
      private: virtual void BoundsHelper(
                     ignition::math::AxisAlignedBox &_box, bool _local,
                     const ignition::math::Pose3d &_pose) const;

      /// \brief Wrapper function for BoundsHelper to reduce redundant
      /// world pose access
      /// \param[in,out] _box The bounding box.
      /// \param[in] _local A flag indicating if the local bounding box is to
      /// be calculated.
      private: virtual void BoundsHelper(
                     ignition::math::AxisAlignedBox &_box, bool _local) const;

      // Documentation inherited.
      protected: virtual GeometryStorePtr Geometries() const override;

      // Documentation inherited.
      protected: virtual bool AttachGeometry(GeometryPtr _geometry) override;

      // Documentation inherited.
      protected: virtual bool DetachGeometry(GeometryPtr _geometry) override;

      /// \brief Initialize the visual
      protected: virtual void Init() override;

      /// \brief Get a shared pointer to this.
      /// \return Shared pointer to this
      private: Ogre2VisualPtr SharedThis();

      /// \brief Pointer to the attached geometries
      protected: Ogre2GeometryStorePtr geometries;

      /// \brief Pointer to private data class
      private: std::unique_ptr<Ogre2VisualPrivate> dataPtr;

      /// \brief Make scene our friend so it can create ogre2 visuals
      private: friend class Ogre2Scene;
    };
    }
  }
}
#endif
