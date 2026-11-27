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

#ifndef GZ_RENDERING_OGRE2_OGRE2THERMALCAMERA_HH_
#define GZ_RENDERING_OGRE2_OGRE2THERMALCAMERA_HH_

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <memory>
#include <string>

#include "gz/rendering/base/BaseThermalCamera.hh"
#include "gz/rendering/ogre2/Export.hh"
#include "gz/rendering/ogre2/Ogre2Sensor.hh"

#include "gz/common/Event.hh"
#include "gz/common/Console.hh"

namespace Ogre
{
  class Material;
  class RenderTarget;
  class Texture;
  class Viewport;
}

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    // Forward declaration
    class Ogre2ThermalCameraPrivate;

    /// \brief Thermal camera used to render thermal data into an image buffer
    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2ThermalCamera :
      public BaseThermalCamera<Ogre2Sensor>
    {
      /// \brief Constructor
      protected: Ogre2ThermalCamera();

      /// \brief Destructor
      public: virtual ~Ogre2ThermalCamera();

      /// \brief Initialize the camera
      public: virtual void Init() override;

      // Documentation inherited
      public: virtual void Destroy() override;

      // Documentation inherited
      public: virtual void PreRender() override;

      /// \brief Render the camera
      public: virtual void PostRender() override;

      /// \brief Connect to the new thermal image event
      /// \param[in] _subscriber Subscriber callback function
      /// \return Pointer to the new Connection. This must be kept in scope
      public: virtual gz::common::ConnectionPtr ConnectNewThermalFrame(
          std::function<void(const uint16_t *, unsigned int, unsigned int,
          unsigned int, const std::string &)>  _subscriber) override;

      /// \brief Implementation of the render call
      public: virtual void Render() override;

      /// \brief Get a pointer to the render target.
      /// \return Pointer to the render target
      protected: virtual RenderTargetPtr RenderTarget() const override;

      /// \brief Create the camera.
      protected: void CreateCamera();

      /// \brief Create dummy render texture. Needed to satisfy inheritance
      protected: virtual void CreateRenderTexture();

      /// \brief Create thermal texture
      protected: virtual void CreateThermalTexture();

      /// \brief Pointer to the ogre camera
      protected: Ogre::Camera *ogreCamera = nullptr;

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<Ogre2ThermalCameraPrivate> dataPtr;

      private: friend class Ogre2Scene;
    };
    }
  }
}
#endif
