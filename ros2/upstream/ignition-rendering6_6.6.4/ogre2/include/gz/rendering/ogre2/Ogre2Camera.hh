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
#ifndef GZ_RENDERING_OGRE2_OGRE2CAMERA_HH_
#define GZ_RENDERING_OGRE2_OGRE2CAMERA_HH_

#include <memory>

#include "gz/rendering/base/BaseCamera.hh"
#include "gz/rendering/ogre2/Ogre2RenderTypes.hh"
#include "gz/rendering/ogre2/Ogre2Includes.hh"
#include "gz/rendering/ogre2/Ogre2Sensor.hh"

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
    class Ogre2CameraPrivate;
    class Ogre2SelectionBuffer;

    /// \brief Ogre2.x implementation of the camera class
    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2Camera :
      public BaseCamera<Ogre2Sensor>
    {
      /// \brief Constructor
      protected: Ogre2Camera();

      /// \brief Destructor
      public: virtual ~Ogre2Camera();

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

      /// \brief Get the background material of this camera
      /// \return Background  material of this camera
      public: virtual MaterialPtr BackgroundMaterial() const;

      /// \brief Set the background material of this camera
      /// \param[in] _material Material to set the background to
      public: virtual void SetBackgroundMaterial(MaterialPtr _material);

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

      // Documentation inherited
      public: virtual VisualPtr VisualAt(const ignition::math::Vector2i
                  &_mousePos) override;

      // Documentation Inherited.
      // \sa Camera::SetMaterial(const MaterialPtr &)
      public: virtual void SetMaterial(
                  const MaterialPtr &_material) override;

      // Documentation inherited.
      public: virtual unsigned int RenderTextureGLId() const override;

      // Documentation inherited.
      public: void SetShadowsDirty() override;

      // Documentation inherited.
      public: virtual void Destroy() override;

      public: Ogre::Camera *OgreCamera() const;

      // Documentation inherited.
      public: virtual void SetVisibilityMask(uint32_t _mask) override;

      /// \brief Get the selection buffer object
      /// \return the selection buffer object
      public: Ogre2SelectionBuffer *SelectionBuffer() const;

      // Documentation inherited.
      protected: virtual RenderTargetPtr RenderTarget() const override;

      // Documentation inherited.
      protected: virtual void Init() override;

      /// \brief Create a render texture for the camera for offscreen rendering
      protected: virtual void CreateRenderTexture();

      /// \brief Destroy render texture created by CreateRenderTexture()
      /// Note: It's not virtual.
      protected: void DestroyRenderTexture();

      /// \brief Create and set selection buffer object
      /// TODO(anyone) to be implemented
      protected: virtual void SetSelectionBuffer();

      /// \brief Create internal camera object
      private: void CreateCamera();

      /// \brief Notifies us that the shadow node definition is about to be
      /// updated. This means our compositor workspace must be destroyed
      /// because the shadow node definition it's using will become a
      /// dangling pointer otherwise
      /// \sa SetShadowsDirty
      private: void SetShadowsNodeDefDirty();

      /// \brief Pointer to ogre camera object
      protected: Ogre::Camera *ogreCamera = nullptr;

      /// \brief Selection buffer object for entity picking
      protected: Ogre2SelectionBuffer *selectionBuffer = nullptr;

      /// \brief Pointer to render texture
      protected: Ogre2RenderTargetPtr renderTexture;

      /// \brief Color of background
      protected: math::Color backgroundColor;

      /// \brief Pointer to private data class
      private: std::unique_ptr<Ogre2CameraPrivate> dataPtr;

      /// \brief Make scene our friend so it can create a camera
      private: friend class Ogre2Scene;

      /// \brief Make ray query our friend so it can use the internal ogre
      /// camera to execute queries
      private: friend class Ogre2RayQuery;
    };
    }
  }
}
#endif
