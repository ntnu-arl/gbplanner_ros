/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef GZ_RENDERING_OGRE_OGRECAMERA_HH_
#define GZ_RENDERING_OGRE_OGRECAMERA_HH_

#include <string>

#include "gz/rendering/base/BaseCamera.hh"
#include "gz/rendering/ogre/OgreRenderTypes.hh"
#include "gz/rendering/ogre/OgreSensor.hh"
#include "gz/rendering/ogre/OgreSelectionBuffer.hh"

namespace Ogre
{
  class Camera;
}

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    // forward declaration
    class OgreSelectionBuffer;

    class IGNITION_RENDERING_OGRE_VISIBLE OgreCamera :
      public BaseCamera<OgreSensor>
    {
      protected: OgreCamera();

      public: virtual ~OgreCamera();

      // Documentation inherited.
      public: virtual math::Angle HFOV() const override;

      // Documentation inherited.
      public: virtual void SetHFOV(const math::Angle &_hfov) override;

      // Documentation inherited.
      public: virtual double AspectRatio() const override;

      // Documentation inherited.
      public: virtual void SetAspectRatio(const double _ratio) override;

      // Documentation inherited.
      public: virtual unsigned int AntiAliasing() const override;

      // Documentation inherited.
      public: virtual void SetAntiAliasing(const unsigned int _aa) override;

      // Documentation inherited.
      public: virtual void SetFarClipPlane(const double _far) override;

      // Documentation inherited.
      public: virtual void SetNearClipPlane(const double _near) override;

      public: virtual math::Color BackgroundColor() const;

      public: virtual void SetBackgroundColor(const math::Color &_color);

      // Documentation inherited.
      public: virtual void Render() override;

      // Documentation inherited.
      public: virtual RenderWindowPtr CreateRenderWindow() override;

      // Documentation inherited.
      public: virtual math::Matrix4d ProjectionMatrix() const override;

      // Documentation inherited.
      public: virtual void SetProjectionMatrix(
          const math::Matrix4d &_matrix) override;

      // Documentation inherited.
      public: virtual math::Matrix4d ViewMatrix() const override;

      // Documentation inherited.
      public: virtual void SetProjectionType(CameraProjectionType _type)
          override;

      public: void SetVFOV(double cameraVFOV) const;

      /// \brief Get the near clip distance
      /// \return Near clip distance
      public: double NearClip() const;

      /// \brief Get the far clip distance
      /// \return Far clip distance
      public: double FarClip() const;

      // Documentation inherited
      public: virtual VisualPtr VisualAt(const gz::math::Vector2i
                  &_mousePos) override;

      // Documentation Inherited.
      // \sa Camera::SetMaterial(const MaterialPtr &)
      public: virtual void SetMaterial(
                  const MaterialPtr &_material) override;

      // Documentation inherited.
      public: virtual unsigned int RenderTextureGLId() const override;

      // Documentation inherited.
      public: virtual void Destroy() override;

      // Documentation inherited.
      public: virtual void SetVisibilityMask(uint32_t _mask) override;

      /// \brief Get underlying Ogre camera
      public: Ogre::Camera *Camera() const;

      // Documentation inherited.
      // public: virtual uint32_t VisibilityMask() const override;

      protected: virtual RenderTargetPtr RenderTarget() const override;

      protected: virtual void Init() override;

      protected: virtual void SetSelectionBuffer();

      private: void CreateCamera();

      protected: virtual void CreateRenderTexture();

      /// \brief Destroy render texture created by CreateRenderTexture()
      /// Note: It's not virtual.
      protected: void DestroyRenderTexture();

      protected: Ogre::Camera *ogreCamera = nullptr;

      protected: OgreSelectionBuffer *selectionBuffer = nullptr;

      protected: OgreRenderTargetPtr renderTexture;

      protected: math::Color backgroundColor;

      private: friend class OgreScene;
      private: friend class OgreRayQuery;
    };
    }
  }
}
#endif
