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
#ifndef GZ_RENDERING_OGRE2_OGRE2RENDERTARGET_HH_
#define GZ_RENDERING_OGRE2_OGRE2RENDERTARGET_HH_

#include <memory>
#include <string>
#include <vector>
#include <gz/math/Color.hh>

#include "gz/rendering/base/BaseRenderTypes.hh"
#include "gz/rendering/base/BaseRenderTarget.hh"
#include "gz/rendering/ogre2/Ogre2Object.hh"
#include "gz/rendering/ogre2/Ogre2RenderTargetMaterial.hh"

#ifdef _MSC_VER
  #pragma warning(push, 0)
#endif
#include <Compositor/OgreCompositorShadowNode.h>
#ifdef _MSC_VER
  #pragma warning(pop)
#endif

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
    // forward declaration
    class Ogre2RenderTargetPrivate;

    /// \brief Ogre2.x implementation of the render target class
    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2RenderTarget :
      public virtual BaseRenderTarget<Ogre2Object>
    {
      /// \brief Constructor
      protected: Ogre2RenderTarget();

      /// \brief Destructor
      public: virtual ~Ogre2RenderTarget();

      /// \brief Get the anti-aliasing level
      public: virtual unsigned int AntiAliasing() const;

      /// \brief set the anti-aliasing level
      /// \param[in] _aa Anti-aliasing level
      public: virtual void SetAntiAliasing(unsigned int _aa);

      /// \brief Copy the render target buffer data to an image
      /// \param[in] _image Image to copy the data to
      public: virtual void Copy(Image &_image) const override;

      /// \brief Get a pointer to the internal ogre camera
      /// \return Pointer to ogre camera
      public: virtual Ogre::Camera *Camera() const;

      /// \brief Set the ogre camera to use for this render target
      /// \param[in] _camera Pointer to ogre camera
      public: virtual void SetCamera(Ogre::Camera *_camera);

      // Documentation inherited
      public: virtual math::Color BackgroundColor() const override;

      /// \brief Set the background color of the render target
      /// \param[in] _color Color to set the background to
      public: virtual void SetBackgroundColor(math::Color _color);

      /// \brief Set the background material of this camera
      /// \param[in] _material Material to set the background to
      public: virtual void SetBackgroundMaterial(MaterialPtr _material);

      /// \brief Get the background material of this camera
      /// \return background material
      public: virtual MaterialPtr BackgroundMaterial() const;

      // Documentation inherited
      public: virtual void PreRender() override;

      // Documentation inherited
      public: virtual void PostRender() override;

      /// \brief Main render call
      public: virtual void Render();

      /// \brief Destroy the render target
      public: virtual void Destroy() override = 0;

      /// \brief Set a material to render on every object. This method is used
      /// for special cases like the render target of a depth camera.
      /// \param[in] _material The material to render
      public: void SetMaterial(MaterialPtr _material);

      /// \see Camera::SetShadowsNodeDefDirty
      public: void SetShadowsNodeDefDirty();

      /// \brief Returns the FSAA to use based on supported specs by HW
      /// and value specified in Ogre2RenderTarget::AntiAliasing
      /// \return Value in range [1; 256). 1 means no antialiasing.
      protected: uint8_t TargetFSAA() const;

      /// \brief Get a pointer to the ogre render target containing
      /// the results of the render (implemented separately
      /// to avoid breaking ABI of the pure virtual function)
      protected: Ogre::TextureGpu *RenderTargetImpl() const;

      /// \brief Get a pointer to the ogre render target containing
      /// the results of the render
      public: virtual Ogre::TextureGpu *RenderTarget() const = 0;

      /// \brief Returns true if this is a render window
      /// \return True if this render target is a render window
      public: virtual bool IsRenderWindow() const;

      // Documentation inherited
      public: unsigned int GLIdImpl() const;

      /// \brief Destroy the render texture
      protected: void DestroyTargetImpl();

      /// \brief Build the render texture
      protected: void BuildTargetImpl();

      /// \brief Get visibility mask for the viewport associated with this
      /// render target
      /// \return Visibility mask
      public: virtual uint32_t VisibilityMask() const;

      /// \brief Set visibility mask for the viewport associated with this
      /// render target
      /// \param[in] _mask Visibility mask
      public: virtual void SetVisibilityMask(uint32_t _mask);

      /// \brief Update the render pass chain
      public: static void UpdateRenderPassChain(
          Ogre::CompositorWorkspace *_workspace,
          const std::string &_workspaceDefName,
          const std::string &_baseNode, const std::string &_finalNode,
          const std::vector<RenderPassPtr> &_renderPasses,
          bool _recreateNodes,
          Ogre::TextureGpu *(*_ogreTextures)[2],
          bool _isRenderWindow);

      /// \brief Update the background color
      protected: virtual void UpdateBackgroundColor();

      /// \brief Update the background material
      protected: virtual void UpdateBackgroundMaterial();

      /// \brief Update the render pass chain
      protected: virtual void UpdateRenderPassChain();

      /// \brief Implementation of the Rebuild function
      protected: virtual void RebuildImpl() override;

      /// \brief Rebuild the render target
      protected: virtual void RebuildTarget() = 0;

      /// \brief Rebuild the compositor
      protected: virtual void RebuildCompositor();

      /// \brief Build the compositor
      protected: virtual void BuildCompositor();

      /// \brief Destroy the compositor
      protected: virtual void DestroyCompositor();

      /// \brief Re-initializes render target material to apply a material to
      /// everything in the scene. Does nothing if no material has been set
      /// \sa Ogre2RenderTarget::RebuildImpl()
      /// \sa BaseRenderTarget::Rebuild()
      protected: void RebuildMaterial();

      /// \brief Pointer to the internal ogre camera
      protected: Ogre::Camera *ogreCamera = nullptr;

      /// \brief Ogre's compositor workspace - the main interface to render
      /// into a render target or render texture.
      protected: Ogre::CompositorWorkspace *ogreCompositorWorkspace = nullptr;

      /// \brief Ogre's compositor workspace definition name
      protected: std::string ogreCompositorWorkspaceDefName;

      /// \brief Stores the background color of the render target
      protected: Ogre::ColourValue ogreBackgroundColor;

      /// \brief Background material of the render target
      protected: MaterialPtr backgroundMaterial;

      /// \brief a material used by for the render target
      protected: MaterialPtr material;

      /// \brief Helper class that applies the material to the render target
      protected: Ogre2RenderTargetMaterialPtr materialApplicator;

      /// \brief Flag to indicate if the render target color has changed
      protected: bool colorDirty = true;

      /// \brief Flag to indicate if the render target background material has
      /// changed
      protected: bool backgroundMaterialDirty = false;

      /// \brief Anti-aliasing level
      protected: unsigned int antiAliasing = 4;

      /// \brief visibility mask associated with this render target
      protected: uint32_t visibilityMask = IGN_VISIBILITY_ALL;

      /// \brief Pointer to private data
      private: std::unique_ptr<Ogre2RenderTargetPrivate> dataPtr;
    };

    /// \brief Ogre2.x implementation of the render texture class
    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2RenderTexture :
      public virtual BaseRenderTexture<Ogre2RenderTarget>
    {
      /// \brief Constructor
      protected: Ogre2RenderTexture();

      /// \brief Destructor
      public: virtual ~Ogre2RenderTexture();

      // Documentation inherited.
      public: virtual void Destroy() override;

      // Documentation inherited
      public: virtual void PreRender() override;

      // Documentation inherited
      public: virtual void PostRender() override;

      // Documentation inherited
      public: virtual unsigned int GLId() const override;

      // Documentation inherited
      // TODO(anyone): this function should be removed.
      // We didn't do it to preserve ABI.
      public: virtual Ogre::TextureGpu *RenderTarget() const override;

      // Documentation inherited.
      protected: virtual void RebuildTarget() override;

      /// \brief Destroy the render texture
      protected: virtual void DestroyTarget();

      /// \brief Build the render texture
      protected: virtual void BuildTarget();

      /// \brief Make scene our friend so it can create a ogre2 render texture
      private: friend class Ogre2Scene;
    };

    /// \brief Ogre2.x implementation of the render window class
    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2RenderWindow :
      public virtual BaseRenderWindow<Ogre2RenderTarget>
    {
      /// \brief Constructor
      protected: Ogre2RenderWindow();

      /// \brief Destructor
      public: virtual ~Ogre2RenderWindow();

      // Documentation inherited.
      public: virtual void Destroy() override;

      // Documentation inherited.
      public: virtual bool IsRenderWindow() const override;

      // Documentation inherited.
      public: virtual Ogre::TextureGpu *RenderTarget() const override;

      // Documentation inherited.
      protected: virtual void RebuildTarget() override;

      /// \brief Build the render window
      protected: virtual void BuildTarget();

      /// \brief Pointer to the internal ogre render target object
      protected: Ogre::TextureGpu *ogreRenderWindow = nullptr;

      /// \brief Make scene our friend so it can create a ogre2 render window
      private: friend class Ogre2Scene;
    };
    }
  }
}
#endif
