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

#ifndef GZ_RENDERING_OGRE2_OGRE2DEPTHCAMERA_HH_
#define GZ_RENDERING_OGRE2_OGRE2DEPTHCAMERA_HH_

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <memory>
#include <string>

#include "gz/rendering/base/BaseDepthCamera.hh"
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
    class Ogre2DepthCameraPrivate;

    /// \brief Depth camera used to render depth data into an image buffer
    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2DepthCamera :
      public BaseDepthCamera<Ogre2Sensor>
    {
      /// \brief Constructor
      protected: Ogre2DepthCamera();

      /// \brief Destructor
      public: virtual ~Ogre2DepthCamera();

      /// \brief Initialize the camera
      public: virtual void Init() override;

      // Documentation inherited
      public: virtual void Destroy() override;

      /// \brief Create dummy render texture. Needed to satisfy inheritance
      public: virtual void CreateRenderTexture();

      /// \brief Create a texture which will hold the depth data
      /// \brief Set up 1st pass material, texture, and compositor
      public: virtual void CreateDepthTexture() override;

      /// \brief Creates an Ogre Workspace instance. Assumes the definition
      /// already and the depth texture have already been created
      private: void CreateWorkspaceInstance();

      // Documentation inherited
      public: virtual void PreRender() override;

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

      // Documentation inherited.
      public: void SetShadowsDirty() override;

      // Documentation inherited.
      public: void AddRenderPass(const RenderPassPtr &_pass) override;

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

      /// \brief Notifies us that the shadow node definition is about to be
      /// updated. This means our compositor workspace must be destroyed
      /// because the shadow node definition it's using will become a
      /// dangling pointer otherwise
      /// \sa SetShadowsDirty
      private: void SetShadowsNodeDefDirty();

      /// \brief Pointer to the ogre camera
      protected: Ogre::Camera *ogreCamera;

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<Ogre2DepthCameraPrivate> dataPtr;

      private: friend class Ogre2Scene;
    };
    }
  }
}
#endif
