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
#ifndef GZ_RENDERING_OGRE_OGRERENDERTARGET_HH_
#define GZ_RENDERING_OGRE_OGRERENDERTARGET_HH_

#include <gz/math/Color.hh>

#include "gz/rendering/base/BaseRenderTypes.hh"
#include "gz/rendering/base/BaseRenderTarget.hh"
#include "gz/rendering/ogre/OgreRenderTypes.hh"
#include "gz/rendering/ogre/OgreIncludes.hh"
#include "gz/rendering/ogre/OgreObject.hh"
#include "gz/rendering/ogre/OgreRenderTargetMaterial.hh"

namespace Ogre
{
  class Camera;
  class RenderTarget;
  class Texture;
}

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    class IGNITION_RENDERING_OGRE_VISIBLE OgreRenderTarget :
      public virtual BaseRenderTarget<OgreObject>
    {
      protected: OgreRenderTarget();

      public: virtual ~OgreRenderTarget();

      public: virtual unsigned int AntiAliasing() const;

      public: virtual void SetAntiAliasing(unsigned int _aa);

      public: virtual void Copy(Image &_image) const override;

      public: virtual Ogre::Camera *Camera() const;

      public: virtual void SetCamera(Ogre::Camera *_camera);

      // Documentation inherited
      public: virtual math::Color BackgroundColor() const override;

      public: virtual void SetBackgroundColor(math::Color _color);

      // Documentation inherited.
      public: virtual void PreRender() override;

      // Documentation inherited.
      public: virtual void PostRender() override;

      public: virtual void Render();

      public: virtual void Destroy() override = 0;

      /// \brief Set a material to render on every object. This method is used
      /// for special cases like the render target of a depth camera.
      /// \param[in] _material The material to render
      public: void SetMaterial(MaterialPtr _material);

      public: virtual Ogre::RenderTarget *RenderTarget() const = 0;

      public: Ogre::Viewport *AddViewport(Ogre::Camera *_viewport);

      public: Ogre::Viewport *Viewport(const int _viewportId) const;

      public: void SetUpdate(const bool _value);

      public: void SetAutoUpdated(const bool _value);

      /// \brief Set visibility mask for the viewport associated with this
      /// render target
      /// \param[in] _mask Visibility mask
      public: virtual void SetVisibilityMask(uint32_t _mask);

      protected: virtual void UpdateBackgroundColor();

      /// \brief Update render pass chain if changes were made
      protected: virtual void UpdateRenderPassChain();

      protected: virtual void RebuildImpl() override;

      protected: virtual void RebuildTarget() = 0;

      protected: virtual void RebuildViewport();

      /// \brief Re-initializes render target material to apply a material to
      /// everything in the scene. Does nothing if no material has been set
      /// \sa OgreRenderTarget::RebuildImpl()
      /// \sa BaseRenderTarget::Rebuild()
      protected: void RebuildMaterial();

      protected: Ogre::Camera *ogreCamera = nullptr;

      protected: Ogre::Viewport *ogreViewport = nullptr;

      protected: Ogre::ColourValue ogreBackgroundColor;

      /// \brief a material used by for the render target
      protected: MaterialPtr material;

      /// \brief Helper class that applies the material to the render target
      protected: OgreRenderTargetMaterialPtr materialApplicator;

      protected: bool colorDirty = true;

      protected: unsigned int antiAliasing = 4;

      /// \brief visibility mask associated with this render target
      protected: uint32_t visibilityMask = IGN_VISIBILITY_ALL;
    };

    class IGNITION_RENDERING_OGRE_VISIBLE OgreRenderTexture :
      public virtual BaseRenderTexture<OgreRenderTarget>
    {
      protected: OgreRenderTexture();

      public: virtual ~OgreRenderTexture();

      public: virtual void Destroy() override;

      // Documentation inherited.
      public: virtual void PreRender() override;

      // Documentation inherited.
      public: virtual void PostRender() override;

      // Documentation inherited.
      public: virtual unsigned int GLId();

      public: virtual void Buffer(float *buffer);

      public: virtual Ogre::RenderTarget *RenderTarget() const override;

      protected: virtual void RebuildTarget() override;

      protected: virtual void DestroyTarget();

      protected: virtual void BuildTarget();

      protected: Ogre::Texture *ogreTexture = nullptr;

      private: friend class OgreScene;
    };

    class IGNITION_RENDERING_OGRE_VISIBLE OgreRenderWindow :
      public virtual BaseRenderWindow<OgreRenderTarget>
    {
      protected: OgreRenderWindow();

      public: virtual ~OgreRenderWindow();

      public: virtual void Destroy();

      protected: virtual Ogre::RenderTarget *RenderTarget() const;

      protected: virtual void RebuildTarget();

      protected: virtual void BuildTarget();

      protected: Ogre::RenderTarget *ogreRenderWindow = nullptr;

      private: friend class OgreScene;
    };
    }
  }
}
#endif
