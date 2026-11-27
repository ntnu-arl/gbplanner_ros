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

#ifndef IGNITION_RENDERING_OGRE2_OGRELIDARVISUAL_HH_
#define IGNITION_RENDERING_OGRE2_OGRELIDARVISUAL_HH_

#include <memory>
#include <vector>
#include "ignition/rendering/base/BaseLidarVisual.hh"
#include "ignition/rendering/ogre2/Ogre2Visual.hh"
#include "ignition/rendering/ogre2/Ogre2Scene.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    // Forward declaration
    class Ogre2LidarVisualPrivate;

    /// \brief Ogre 2.x implementation of a Lidar Visual.
    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2LidarVisual
      : public BaseLidarVisual<Ogre2Visual>
    {
      /// \brief Constructor
      protected: Ogre2LidarVisual();

      /// \brief Destructor
      public: virtual ~Ogre2LidarVisual();

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
      private: friend class Ogre2Scene;

      /// \brief Private data class
      private: std::unique_ptr<Ogre2LidarVisualPrivate> dataPtr;
    };
    }
  }
}
#endif
