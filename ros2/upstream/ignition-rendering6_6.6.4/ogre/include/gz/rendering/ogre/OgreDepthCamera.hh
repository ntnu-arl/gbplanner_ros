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

#ifndef GZ_RENDERING_OGRE_OGREDEPTHCAMERA_HH_
#define GZ_RENDERING_OGRE_OGREDEPTHCAMERA_HH_

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <memory>
#include <string>

#include "gz/rendering/RenderTypes.hh"
#include "gz/rendering/base/BaseDepthCamera.hh"
#include "gz/rendering/ogre/OgreConversions.hh"
#include "gz/rendering/ogre/OgreIncludes.hh"
#include "gz/rendering/ogre/OgreRenderTarget.hh"
#include "gz/rendering/ogre/OgreRenderTypes.hh"
#include "gz/rendering/ogre/OgreScene.hh"
#include "gz/rendering/ogre/OgreSensor.hh"
#include "gz/rendering/ogre/OgreSelectionBuffer.hh"

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
    // forward declaration
    class OgreDepthCameraPrivate;

    /** \class OgreDepthCamera OgreDepthCamera.hh\
     * rendering/ogre/OgreDepthCamera.hh
    **/
    /// \brief Depth camera used to render depth data into an image buffer
    class IGNITION_RENDERING_OGRE_VISIBLE OgreDepthCamera :
      public BaseDepthCamera<OgreSensor>
    {
      /// \brief Constructor
      protected: OgreDepthCamera();

      /// \brief Destructor
      public: virtual ~OgreDepthCamera();

      /// \brief Initialize the camera
      public: virtual void Init() override;

      /// \brief Create a texture which will hold the depth data
      public: virtual void CreateDepthTexture() override;

      /// \brief Destroy render texture created by CreateDepthTexture()
      /// Note: It's not virtual.
      protected: void DestroyDepthTexture();

      /// \brief Render the camera
      public: virtual void PostRender() override;

      /// \brief All things needed to get back z buffer for depth data
      /// \return The z-buffer as a float array
      public: virtual const float *DepthData() const override;

      /// \brief Connect a to the new depth image signal
      /// \param[in] _subscriber Subscriber callback function
      /// \return Pointer to the new Connection. This must be kept in scope
      public: virtual gz::common::ConnectionPtr ConnectNewDepthFrame(
          std::function<void(const float *, unsigned int, unsigned int,
          unsigned int, const std::string &)>  _subscriber) override;

      /// \brief Connect a to the new rgb point cloud signal
      /// \param[in] _subscriber Subscriber callback function
      /// \return Pointer to the new Connection. This must be kept in scope
      public: virtual gz::common::ConnectionPtr ConnectNewRgbPointCloud(
          std::function<void(const float *, unsigned int, unsigned int,
          unsigned int, const std::string &)>  _subscriber) override;

      // Documentation inherited.
      public: virtual void PreRender() override;

      /// \brief Implementation of the render call
      public: virtual void Render() override;

      /// \brief Set the far clip distance
      /// \param[in] _far far clip distance
      public: virtual void SetFarClipPlane(const double _far) override;

      /// \brief Set the near clip distance
      /// \param[in] _near Near clip distance
      public: virtual void SetNearClipPlane(const double _near) override;

      /// \brief Get the near clip distance
      /// \return Near clip distance. A value of zero is returned if the
      /// ogre camera has not been created.
      public: double NearClipPlane() const override;

      /// \brief Get the far clip distance
      /// \return Far clip distance. A value of zero is returned if the
      /// ogre camera has not been created.
      public: double FarClipPlane() const override;

      // Documentation inherited
      public: virtual void Destroy() override;

      /// \brief Update a render target
      /// \param[in] _target Render target to update
      /// \param[in] _material Material to use
      /// \param[in] _matName Material name
      protected: void UpdateRenderTarget(OgreRenderTexturePtr _target,
                                       Ogre::Material *_material,
                                       const std::string &_matName);

      /// \brief Get a pointer to the render target.
      /// \return Pointer to the render target
      protected: virtual RenderTargetPtr RenderTarget() const override;

      /// \brief Limit field of view taking care of using a valid value for
      /// an OGRE camera.
      /// \param[in] _fov expected field of view
      /// \return valid field of view
      protected: static double LimitFOV(const double _fov);

      /// \brief Create the camera.
      protected: void CreateCamera();

      /// \brief Create point cloud texture. This stores xyz rgb data
      private: void CreatePointCloudTexture();

      /// \brief Destroy render texture created by CreatePointCloudTexture()
      /// Note: It's not virtual.
      protected: void DestroyPointCloudTexture();

      /// \brief Communicates that a frams was rendered
      protected: bool newData = false;

      protected: bool captureData = false;

      /// \brief Pointer to the depth texture
      protected: OgreRenderTexturePtr depthTexture;

      /// \brief Pointer to the depth viewport
      protected: Ogre::Viewport *depthViewport = nullptr;

      /// \brief Pointer to the ogre camera
      protected: Ogre::Camera *ogreCamera;

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<OgreDepthCameraPrivate> dataPtr;

      private: friend class OgreScene;
      private: friend class OgreRayQuery;
    };
    }
  }
}
#endif
