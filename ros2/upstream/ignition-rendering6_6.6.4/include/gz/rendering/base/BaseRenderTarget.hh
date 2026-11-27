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
#ifndef GZ_RENDERING_BASE_BASERENDERTARGET_HH_
#define GZ_RENDERING_BASE_BASERENDERTARGET_HH_

#include <string>
#include <vector>

#include "gz/rendering/RenderPass.hh"
#include "gz/rendering/RenderTarget.hh"
#include "gz/rendering/Scene.hh"
#include "gz/rendering/base/BaseRenderTypes.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    template <class T>
    class BaseRenderTarget :
      public virtual RenderTarget,
      public virtual T
    {
      public: BaseRenderTarget();

      public: virtual ~BaseRenderTarget();

      // Documentation inherited.
      public: virtual void PreRender() override;

      // Documentation inherited.
      public: virtual void PostRender() override;

      public: virtual unsigned int Width() const override;

      public: virtual void SetWidth(const unsigned int _width) override;

      public: virtual unsigned int Height() const override;

      public: virtual void SetHeight(const unsigned int _height) override;

      public: virtual PixelFormat Format() const override;

      public: virtual void SetFormat(PixelFormat _format) override;

      // Documentation inherited
      public: virtual math::Color BackgroundColor() const override;

      // Documentation inherited
      public: virtual void AddRenderPass(const RenderPassPtr &_pass) override;

      // Documentation inherited
      public: virtual void RemoveRenderPass(const RenderPassPtr &_pass)
          override;

      // Documentation inherited
      public: virtual unsigned int RenderPassCount() const override;

      // Documentation inherited
      public: virtual RenderPassPtr RenderPassByIndex(unsigned int _index)
          const override;

      protected: virtual void Rebuild();

      protected: virtual void RebuildImpl() = 0;

      protected: PixelFormat format = PF_UNKNOWN;

      protected: bool targetDirty = true;

      /// \brief Flag to indicate if render pass need to be rebuilt
      protected: bool renderPassDirty = false;

      protected: unsigned int width = 0u;

      protected: unsigned int height = 0u;

      /// \brief A chain of render passes applied to the render target
      protected: std::vector<RenderPassPtr> renderPasses;
    };

    template <class T>
    class BaseRenderTexture :
      public virtual RenderTexture,
      public virtual T
    {
      public: BaseRenderTexture();

      public: virtual ~BaseRenderTexture();

      // Documentation inherited.
      public: virtual unsigned int GLId() const override;
    };

    template <class T>
    class BaseRenderWindow :
      public virtual RenderWindow,
      public virtual T
    {
      public: BaseRenderWindow();

      public: virtual ~BaseRenderWindow();

      public: virtual std::string Handle() const;

      public: virtual void SetHandle(const std::string &_handle);

      public: virtual double DevicePixelRatio() const;

      public: virtual void SetDevicePixelRatio(const double _ratio);

      public: virtual void OnResize(const unsigned int _width,
                  const unsigned int _height);

      public: virtual void OnMove();

      protected: std::string handle;

      protected: double ratio = 1.0;
    };

    //////////////////////////////////////////////////
    // BaseRenderTarget
    //////////////////////////////////////////////////
    template <class T>
    BaseRenderTarget<T>::BaseRenderTarget()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    BaseRenderTarget<T>::~BaseRenderTarget()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseRenderTarget<T>::PreRender()
    {
      T::PreRender();
      this->Rebuild();
      for (auto &pass : this->renderPasses)
        pass->PreRender();
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseRenderTarget<T>::PostRender()
    {
      T::PostRender();
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseRenderTarget<T>::Rebuild()
    {
      if (this->targetDirty)
      {
        this->RebuildImpl();
        this->targetDirty = false;
      }
    }

    //////////////////////////////////////////////////
    template <class T>
    unsigned int BaseRenderTarget<T>::Width() const
    {
      return this->width;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseRenderTarget<T>::SetWidth(const unsigned int _width)
    {
      this->width = _width;
      this->targetDirty = true;
    }

    //////////////////////////////////////////////////
    template <class T>
    unsigned int BaseRenderTarget<T>::Height() const
    {
      return this->height;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseRenderTarget<T>::SetHeight(const unsigned int _height)
    {
      this->height = _height;
      this->targetDirty = true;
    }

    //////////////////////////////////////////////////
    template <class T>
    PixelFormat BaseRenderTarget<T>::Format() const
    {
      return this->format;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseRenderTarget<T>::SetFormat(PixelFormat _format)
    {
      this->format = PixelUtil::Sanitize(_format);
      this->targetDirty = true;
    }

    //////////////////////////////////////////////////
    template <class T>
    math::Color BaseRenderTarget<T>::BackgroundColor() const
    {
      return this->Scene()->BackgroundColor();
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseRenderTarget<T>::AddRenderPass(const RenderPassPtr &_pass)
    {
      this->renderPasses.push_back(_pass);
      this->renderPassDirty = true;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseRenderTarget<T>::RemoveRenderPass(const RenderPassPtr &_pass)
    {
      auto it = std::find(this->renderPasses.begin(), this->renderPasses.end(),
          _pass);
      if (it != this->renderPasses.end())
      {
        (*it)->Destroy();
        this->renderPasses.erase(it);
        this->renderPassDirty = true;
      }
    }

    //////////////////////////////////////////////////
    template <class T>
    unsigned int BaseRenderTarget<T>::RenderPassCount() const
    {
      return this->renderPasses.size();
    }

    //////////////////////////////////////////////////
    template <class T>
    RenderPassPtr BaseRenderTarget<T>::RenderPassByIndex(unsigned int _index)
        const
    {
      if (_index > this->renderPasses.size())
      {
        ignerr << "RenderPass index out of range: " << _index << std::endl;
        return RenderPassPtr();
      }
      return this->renderPasses[_index];
    }

    //////////////////////////////////////////////////
    // BaseRenderTexture
    //////////////////////////////////////////////////
    template <class T>
    BaseRenderTexture<T>::BaseRenderTexture()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    BaseRenderTexture<T>::~BaseRenderTexture()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    unsigned int BaseRenderTexture<T>::GLId() const
    {
      return 0u;
    }

    //////////////////////////////////////////////////
    // BaseRenderWindow
    //////////////////////////////////////////////////
    template <class T>
    BaseRenderWindow<T>::BaseRenderWindow()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    BaseRenderWindow<T>::~BaseRenderWindow()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    std::string BaseRenderWindow<T>::Handle() const
    {
      return this->handle;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseRenderWindow<T>::SetHandle(const std::string &_handle)
    {
      this->handle = _handle;
      this->targetDirty = true;
    }

    //////////////////////////////////////////////////
    template <class T>
    double BaseRenderWindow<T>::DevicePixelRatio() const
    {
      return this->ratio;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseRenderWindow<T>::SetDevicePixelRatio(const double _ratio)
    {
      this->ratio = _ratio;
      this->targetDirty = true;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseRenderWindow<T>::OnResize(const unsigned int _width,
        const unsigned int _height)
    {
      this->width = _width;
      this->height = _height;
      this->targetDirty = true;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseRenderWindow<T>::OnMove()
    {
      this->targetDirty = true;
    }
    }
  }
}
#endif
