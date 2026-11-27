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

#ifndef IGNITION_RENDERING_OGRE2_OGRE2SEGMENTATIONCAMERA_HH_
#define IGNITION_RENDERING_OGRE2_OGRE2SEGMENTATIONCAMERA_HH_

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <memory>
#include <string>

#include <ignition/common/Console.hh>
#include <ignition/common/Event.hh>

#include "ignition/rendering/base/BaseSegmentationCamera.hh"
#include "ignition/rendering/ogre2/Ogre2Includes.hh"
#include "ignition/rendering/ogre2/Ogre2Sensor.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    // Forward declaration
    class Ogre2SegmentationCameraPrivate;

    /// \brief Segmentation camera used to label each pixel with a label id.
    /// Supports Semantic / Panoptic Segmentation
    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2SegmentationCamera :
      public BaseSegmentationCamera<Ogre2Sensor>
    {
      /// \brief Constructor
      protected: Ogre2SegmentationCamera();

      /// \brief Destructor
      public: virtual ~Ogre2SegmentationCamera();

      // Documentation inherited
      public: virtual void Init() override;

      // Documentation inherited
      public: virtual void Destroy() override;

      // Documentation inherited
      public: virtual void PreRender() override;

      // Documentation inherited
      public: virtual void PostRender() override;

      // Documentation inherited
      public: virtual ignition::common::ConnectionPtr
        ConnectNewSegmentationFrame(
        std::function<void(const uint8_t *, unsigned int, unsigned int,
        unsigned int, const std::string &)>  _subscriber) override;

      // Documentation inherited
      public: virtual void Render() override;

      /// \brief Get a pointer to the render target.
      /// \return Pointer to the render target
      protected: virtual RenderTargetPtr RenderTarget() const override;

      // Documentation inherited
      public: void SetBackgroundLabel(int _label) override;

      // Documentation inherited
      public: void LabelMapFromColoredBuffer(
                  uint8_t * _labelBuffer) const override;

      /// \brief Create the camera.
      protected: void CreateCamera();

      /// \brief Create render texture
      protected: virtual void CreateRenderTexture();

      // Documentation inherited
      protected: virtual void CreateSegmentationTexture() override;

      /// \brief Pointer to the ogre camera
      protected: Ogre::Camera *ogreCamera = nullptr;

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<Ogre2SegmentationCameraPrivate> dataPtr;

      /// \brief Make scene our friend so it can create a camera
      private: friend class Ogre2Scene;
    };
    }
  }
}
#endif
