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

#ifndef GZ_RENDERING_OGRE_OGRETHERMALCAMERA_HH_
#define GZ_RENDERING_OGRE_OGRETHERMALCAMERA_HH_

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <memory>
#include <string>

#include "gz/rendering/RenderTypes.hh"
#include "gz/rendering/base/BaseThermalCamera.hh"
#include "gz/rendering/ogre/Export.hh"
#include "gz/rendering/ogre/OgreConversions.hh"
#include "gz/rendering/ogre/OgreIncludes.hh"
#include "gz/rendering/ogre/OgreRenderTarget.hh"
#include "gz/rendering/ogre/OgreRenderTypes.hh"
#include "gz/rendering/ogre/OgreScene.hh"
#include "gz/rendering/ogre/OgreSensor.hh"

#include "gz/common/Event.hh"
#include "gz/common/Console.hh"


namespace Ogre
{
  class Material;
  class Camera;
}

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    // forward declaration
    class OgreThermalCameraPrivate;

    /** \class OgreThermalCamera OgreThermalCamera.hh\
     * rendering/ogre/OgreThermalCamera.hh
    **/
    /// \brief Depth camera used to render thermal data into an image buffer
    class IGNITION_RENDERING_OGRE_VISIBLE OgreThermalCamera :
      public BaseThermalCamera<OgreSensor>
    {
      /// \brief Constructor
      protected: OgreThermalCamera();

      /// \brief Destructor
      public: virtual ~OgreThermalCamera();

      /// \brief Initialize the camera
      public: virtual void Init() override;

      /// \brief Create a texture
      public: virtual void CreateRenderTexture();

      /// \brief Destroy render texture created by CreateRenderTexture()
      /// Note: It's not virtual.
      protected: void DestroyRenderTexture();

      /// \brief Render the camera
      public: virtual void PostRender() override;

      /// \brief Connect to the new thermal image signal
      /// \param[in] _subscriber Subscriber callback function
      /// \return Pointer to the new Connection. This must be kept in scope
      public: virtual gz::common::ConnectionPtr ConnectNewThermalFrame(
          std::function<void(const uint16_t *, unsigned int, unsigned int,
          unsigned int, const std::string &)>  _subscriber) override;

      // Documentation inherited.
      public: virtual void PreRender() override;

      /// \brief Implementation of the render call
      public: virtual void Render() override;

      // Documentation inherited
      public: virtual void Destroy() override;

      /// \brief Get a pointer to the render target.
      /// \return Pointer to the render target
      protected: virtual RenderTargetPtr RenderTarget() const override;

      /// \brief Create the camera.
      protected: void CreateCamera();

      /// \brief Create thermal texture. This stores temperature data
      private: void CreateThermalTexture();

      /// \brief Pointer to the ogre camera
      protected: Ogre::Camera *ogreCamera = nullptr;

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<OgreThermalCameraPrivate> dataPtr;

      private: friend class OgreScene;
    };
    }
  }
}
#endif
