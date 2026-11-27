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

#ifndef IGNITION_RENDERING_OGRE_OGRELIDARVISUAL_HH_
#define IGNITION_RENDERING_OGRE_OGRELIDARVISUAL_HH_

#include <vector>
#include <memory>
#include "ignition/rendering/base/BaseLidarVisual.hh"
#include "ignition/rendering/ogre/OgreVisual.hh"
#include "ignition/rendering/ogre/OgreIncludes.hh"
#include "ignition/rendering/ogre/OgreScene.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    // Forward declaration
    class OgreLidarVisualPrivate;

    /// \brief Ogre implementation of a Lidar Visual.
    class IGNITION_RENDERING_OGRE_VISIBLE OgreLidarVisual
      : public BaseLidarVisual<OgreVisual>
    {
      /// \brief Constructor
      protected: OgreLidarVisual();

      /// \brief Destructor
      public: virtual ~OgreLidarVisual();

      // Documentation inherited.
      public: virtual void Init() override;

      // Documentation inherited.
      public: virtual void PreRender() override;

      // Documentation inherited.
      public: virtual void Destroy() override;

      // Documentation inherited
      public: virtual void Update() override;

      // Documentation inherited
      public: virtual void SetPoints(
              const std::vector<double> &_points) override;

      // Documentation inherited
      // This only affects lidar visuals with type LVT_POINTS
      public: virtual void SetPoints(const std::vector<double> &_points,
                        const std::vector<ignition::math::Color> &_colors)
                                    override;

      // Documentation inherited
      public: virtual void ClearPoints() override;

      // Documentation inherited
      public: virtual unsigned int PointCount() const override;

      // Documentation inherited
      public: virtual std::vector<double> Points() const override;

      /// \brief Create the Lidar Visual in ogre
      private: void Create();

      /// \brief Clear data stored by dynamiclines
      private: void ClearVisualData();

      // Documentation inherited
      public: virtual void SetVisible(bool _visible) override;

      /// \brief Lidar Visual should only be created by scene.
      private: friend class OgreScene;

      /// \brief Private data class
      private: std::unique_ptr<OgreLidarVisualPrivate> dataPtr;
    };
    }
  }
}
#endif
