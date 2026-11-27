/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License") = 0;
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
#ifndef GZ_RENDERING_RENDERTARGET_HH_
#define GZ_RENDERING_RENDERTARGET_HH_

#include <string>

#include <gz/math/Color.hh>

#include "gz/rendering/config.hh"
#include "gz/rendering/RenderTypes.hh"
#include "gz/rendering/Image.hh"
#include "gz/rendering/Object.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \class RenderTarget RenderTarget.hh ignition/rendering/RenderTarget.hh
    /// \brief Represents a render-target to which cameras can render images.
    class IGNITION_RENDERING_VISIBLE RenderTarget :
      public virtual Object
    {
      /// \brief Destructor
      public: virtual ~RenderTarget() { }

      /// \brief Get render target width in pixels
      /// \return The render target width in pixels
      public: virtual unsigned int Width() const = 0;

      /// \brief Get render target height in pixels
      /// \return The Render target height in pixels
      public: virtual unsigned int Height() const = 0;

      /// \brief Set the render target width in pixels
      /// \param[in] _width New render target width in pixels
      public: virtual void SetWidth(const unsigned int _width) = 0;

      /// \brief Set the render target height in pixels
      /// \param[in] _height New render target height in pixels
      public: virtual void SetHeight(const unsigned int _height) = 0;

      /// \brief Set the render target image format
      /// \return Render target format
      public: virtual PixelFormat Format() const = 0;

      /// \brief Set the render target image format
      /// \param[in] _format New target format
      public: virtual void SetFormat(PixelFormat _format) = 0;

      /// \brief Write rendered image to given Image. The RenderTarget will
      /// convert the underlying image to the specified format listed in the
      /// given Image. However if the given image is not of the correct size no
      /// work will be done. Calling this function before an image has been
      /// rendered will result in undefined behavior.
      /// \param[out] _image Image to which output will be written
      public: virtual void Copy(Image &_image) const = 0;

      /// \brief Get the background color of the render target.
      /// This should be the same as the scene background color.
      /// \return Render target background color.
      public: virtual math::Color BackgroundColor() const = 0;

      /// \brief Add a render pass to the render target
      /// \param[in] _pass New render pass to add
      public: virtual void AddRenderPass(const RenderPassPtr &_pass) = 0;

      /// \brief Remove a render pass from the render target
      /// \param[in] _pass render pass to remove
      public: virtual void RemoveRenderPass(const RenderPassPtr &_pass) = 0;

      /// \brief Get the number of render passes applied to the render target
      /// \return Number of render passes applied
      public: virtual unsigned int RenderPassCount() const = 0;

      /// \brief Get a render pass by index
      /// \return Render pass at the specified index
      public: virtual RenderPassPtr RenderPassByIndex(unsigned int _index)
          const = 0;
    };

    /* \class RenderTexture RenderTexture.hh \
     * ignition/rendering/RenderTexture.hh
     */
    /// \brief Represents a off-screen render-texture to which cameras can
    /// render images.
    class IGNITION_RENDERING_VISIBLE RenderTexture :
      public virtual RenderTarget
    {
      /// \brief Deconstructor
      public: virtual ~RenderTexture() { }

      /// \brief Returns the OpenGL texture Id. A valid Id is returned only
      // if this is an OpenGL render texture
      public: virtual unsigned int GLId() const = 0;
    };

    /* \class RenderWindow RenderWindow.hh \
     * ignition/rendering/RenderWindow.hh
     */
    /// \brief Represents a on-screen render-window to which cameras can
    /// render images.
    class IGNITION_RENDERING_VISIBLE RenderWindow :
      public virtual RenderTarget
    {
      /// \brief Deconstructor
      public: virtual ~RenderWindow() { }

      /// \brief Get the window handle that the render window is attached to.
      /// \return Window handle
      public: virtual std::string Handle() const = 0;

      /// \brief Set the window handle to attach the render window to
      /// \param[in] _handle Window handle
      public: virtual void SetHandle(const std::string &_handle) = 0;

      /// \brief Get the device to pixel ratio
      /// \return Device to pixel ratio
      public: virtual double DevicePixelRatio() const = 0;

      /// \brief Set the device to pixel ratio
      /// \param[in] _ratio Device to pixel ratio
      public: virtual void SetDevicePixelRatio(const double _ratio) = 0;

      /// \brief Alert the window of a window resize event
      /// \param[in] _width New window width in pixels
      /// \param[in] _height New window height in pixels
      public: virtual void OnResize(unsigned int _width,
                  unsigned int _height) = 0;

      /// \brief Alert the window of a window move event
      public: virtual void OnMove() = 0;
    };
    }
  }
}
#endif
