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

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
#else
# pragma warning(push)
# pragma warning(disable: 4005)
# pragma warning(disable: 4275)
#endif
// leave this out of OgreIncludes as it conflicts with other files requiring
// gl.h
#include <OgreGLFBORenderTexture.h>
#ifndef _WIN32
# pragma GCC diagnostic pop
#else
# pragma warning(pop)
#endif


#include <gz/common/Console.hh>

#include "gz/rendering/Material.hh"

#include "gz/rendering/ogre/OgreRenderEngine.hh"
#include "gz/rendering/ogre/OgreRenderPass.hh"
#include "gz/rendering/ogre/OgreConversions.hh"
#include "gz/rendering/ogre/OgreMaterial.hh"
#include "gz/rendering/ogre/OgreRenderTarget.hh"
#include "gz/rendering/ogre/OgreRTShaderSystem.hh"
#include "gz/rendering/ogre/OgreScene.hh"
#include "gz/rendering/ogre/OgreCamera.hh"
#include "gz/rendering/ogre/OgreIncludes.hh"

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
// OgreRenderTarget
//////////////////////////////////////////////////
OgreRenderTarget::OgreRenderTarget()
{
  this->ogreBackgroundColor = Ogre::ColourValue::Black;
}

//////////////////////////////////////////////////
OgreRenderTarget::~OgreRenderTarget()
{
  IGN_ASSERT(this->ogreViewport == nullptr,
             "OgreRenderTarget::Destroy not called!");
}

//////////////////////////////////////////////////
void OgreRenderTarget::Copy(Image &_image) const
{
  if (nullptr == this->RenderTarget())
    return;

  // TODO(anyone): handle Bayer conversions
  // TODO(anyone): handle ogre version differences

  if (_image.Width() != this->width || _image.Height() != this->height)
  {
    ignerr << "Invalid image dimensions" << std::endl;
    return;
  }

  void* data = _image.Data();
  Ogre::PixelFormat imageFormat = OgreConversions::Convert(_image.Format());
  Ogre::PixelBox ogrePixelBox(this->width, this->height, 1, imageFormat, data);
  this->RenderTarget()->copyContentsToMemory(ogrePixelBox);
}

//////////////////////////////////////////////////
Ogre::Camera *OgreRenderTarget::Camera() const
{
  return this->ogreCamera;
}

//////////////////////////////////////////////////
void OgreRenderTarget::SetCamera(Ogre::Camera *_camera)
{
  this->ogreCamera = _camera;
  this->targetDirty = true;
}

//////////////////////////////////////////////////
math::Color OgreRenderTarget::BackgroundColor() const
{
  return OgreConversions::Convert(this->ogreBackgroundColor);
}

//////////////////////////////////////////////////
void OgreRenderTarget::SetBackgroundColor(math::Color _color)
{
  this->ogreBackgroundColor = OgreConversions::Convert(_color);
  this->colorDirty = true;
}

//////////////////////////////////////////////////
unsigned int OgreRenderTarget::AntiAliasing() const
{
  return this->antiAliasing;
}

//////////////////////////////////////////////////
void OgreRenderTarget::SetAntiAliasing(unsigned int _aa)
{
  this->antiAliasing = _aa;
  this->targetDirty = true;
}

//////////////////////////////////////////////////
void OgreRenderTarget::PreRender()
{
  BaseRenderTarget::PreRender();
  this->UpdateBackgroundColor();

  if (this->material)
  {
    this->material->PreRender();
  }

  this->UpdateRenderPassChain();
}

//////////////////////////////////////////////////
void OgreRenderTarget::PostRender()
{
  // do nothing by default
}

//////////////////////////////////////////////////
void OgreRenderTarget::Render()
{
  if (nullptr == this->RenderTarget())
    return;

  this->RenderTarget()->update();
}

//////////////////////////////////////////////////
void OgreRenderTarget::SetVisibilityMask(uint32_t _mask)
{
  this->visibilityMask = _mask;
  if (this->ogreViewport)
    this->ogreViewport->setVisibilityMask(this->visibilityMask);
}

//////////////////////////////////////////////////
void OgreRenderTarget::UpdateBackgroundColor()
{
  if (this->colorDirty && this->ogreViewport)
  {
    this->ogreViewport->setBackgroundColour(this->ogreBackgroundColor);
    this->colorDirty = false;
  }
}

//////////////////////////////////////////////////
void OgreRenderTarget::RebuildImpl()
{
  this->RebuildTarget();
  this->RebuildViewport();
  this->RebuildMaterial();
}

//////////////////////////////////////////////////
void OgreRenderTarget::RebuildViewport()
{
  if (nullptr == this->RenderTarget())
    return;

  Ogre::RenderTarget *ogreRenderTarget = this->RenderTarget();
  ogreRenderTarget->removeAllViewports();
  ogreRenderTarget->removeAllListeners();

  this->ogreViewport = ogreRenderTarget->addViewport(this->ogreCamera);
  this->ogreViewport->setBackgroundColour(this->ogreBackgroundColor);
  this->ogreViewport->setClearEveryFrame(true);
  this->ogreViewport->setShadowsEnabled(true);
  this->ogreViewport->setOverlaysEnabled(false);
  this->ogreViewport->setVisibilityMask(this->visibilityMask);

  OgreRTShaderSystem::Instance()->AttachViewport(this->ogreViewport,
      this->scene);
}

//////////////////////////////////////////////////
void OgreRenderTarget::SetMaterial(MaterialPtr _material)
{
  this->material = _material;

  // Have to rebuild the target so there is something to apply the applicator to
  this->targetDirty = true;
}

//////////////////////////////////////////////////
void OgreRenderTarget::RebuildMaterial()
{
  if (this->material && this->RenderTarget())
  {
    OgreMaterial *ogreMaterial = dynamic_cast<OgreMaterial*>(
        this->material.get());
    Ogre::MaterialPtr matPtr = ogreMaterial->Material();

    Ogre::RenderTarget *target = this->RenderTarget();
    this->materialApplicator.reset(new OgreRenderTargetMaterial(
        this->scene, target, matPtr.get()));
  }
}

//////////////////////////////////////////////////
void OgreRenderTarget::UpdateRenderPassChain()
{
  if (!this->renderPassDirty)
    return;

  for (auto pass : this->renderPasses)
  {
    OgreRenderPass *ogreRenderPass =
        dynamic_cast<OgreRenderPass *>(pass.get());
    ogreRenderPass->SetCamera(this->ogreCamera);
    ogreRenderPass->CreateRenderPass();
  }
  this->renderPassDirty = false;
}

////////////////////////////////////////////////////
Ogre::Viewport *OgreRenderTarget::Viewport(const int _viewportId) const
{
  Ogre::RenderTarget *ogreRenderTarget = this->RenderTarget();

  if (nullptr == ogreRenderTarget)
  {
    ignerr << "Failed to get viewport: null render target" << std::endl;
    return nullptr;
  }

  return ogreRenderTarget->getViewport(_viewportId);
}

////////////////////////////////////////////////////
Ogre::Viewport *OgreRenderTarget::AddViewport(Ogre::Camera *_camera)
{
  Ogre::RenderTarget *ogreRenderTarget = this->RenderTarget();

  if (nullptr == ogreRenderTarget)
  {
    ignerr << "Failed to add viewport: null render target" << std::endl;
    return nullptr;
  }

  return ogreRenderTarget->addViewport(_camera);
}

////////////////////////////////////////////////////
void OgreRenderTarget::SetAutoUpdated(const bool _value)
{
  Ogre::RenderTarget *ogreRenderTarget = this->RenderTarget();

  if (nullptr == ogreRenderTarget)
  {
    ignerr << "Failed to set auto update: null render target" << std::endl;
    return;
  }

  ogreRenderTarget->setAutoUpdated(_value);
}

////////////////////////////////////////////////////
void OgreRenderTarget::SetUpdate(const bool _value)
{
  Ogre::RenderTarget *ogreRenderTarget = this->RenderTarget();

  if (nullptr == ogreRenderTarget)
  {
    ignerr << "Failed to set update: null render target" << std::endl;
    return;
  }

  ogreRenderTarget->update(_value);
}

//////////////////////////////////////////////////
// OgreRenderTexture
//////////////////////////////////////////////////
OgreRenderTexture::OgreRenderTexture()
{
}

//////////////////////////////////////////////////
OgreRenderTexture::~OgreRenderTexture()
{
  IGN_ASSERT(this->ogreTexture == nullptr,
             "OgreRenderTexture::Destroy not called!");
}

//////////////////////////////////////////////////
void OgreRenderTexture::Destroy()
{
  this->DestroyTarget();
}

//////////////////////////////////////////////////
Ogre::RenderTarget *OgreRenderTexture::RenderTarget() const
{
  if (nullptr == this->ogreTexture)
    return nullptr;

  return this->ogreTexture->getBuffer()->getRenderTarget();
}

//////////////////////////////////////////////////
void OgreRenderTexture::RebuildTarget()
{
  this->DestroyTarget();
  this->BuildTarget();
}

//////////////////////////////////////////////////
void OgreRenderTexture::DestroyTarget()
{
  if (nullptr == this->ogreTexture)
    return;

  this->materialApplicator.reset();

  OgreRTShaderSystem::Instance()->DetachViewport(this->ogreViewport,
      this->scene);

  auto &manager = Ogre::TextureManager::getSingleton();
  manager.unload(this->ogreTexture->getName());
  manager.remove(this->ogreTexture->getName());

  // clean up OGRE depth buffers on RTT resize.
  // see https://forums.ogre3d.org/viewtopic.php?t=92111#p535220
  auto engine = OgreRenderEngine::Instance();
  engine->OgreRoot()->getRenderSystem()->_cleanupDepthBuffers(false);

  this->ogreTexture = nullptr;
  this->ogreViewport = nullptr;
}

//////////////////////////////////////////////////
void OgreRenderTexture::BuildTarget()
{
  Ogre::TextureManager &manager = Ogre::TextureManager::getSingleton();
  Ogre::PixelFormat ogreFormat = OgreConversions::Convert(this->format);

  // check if target fsaa is supported
  unsigned int fsaa = 0;
  std::vector<unsigned int> fsaaLevels =
      OgreRenderEngine::Instance()->FSAALevels();
  unsigned int targetFSAA = this->antiAliasing;
  auto const it = std::find(fsaaLevels.begin(), fsaaLevels.end(), targetFSAA);
  if (it != fsaaLevels.end())
  {
    fsaa = targetFSAA;
  }
  else
  {
    // output warning but only do it once
    static bool ogreFSAAWarn = false;
    if (ogreFSAAWarn)
    {
      ignwarn << "Anti-aliasing level of '" << this->antiAliasing << "' "
              << "is not supported. Setting to 0" << std::endl;
      ogreFSAAWarn = true;
    }
  }

  this->ogreTexture = (manager.createManual(this->name, "General",
      Ogre::TEX_TYPE_2D, this->width, this->height, 0, ogreFormat,
      Ogre::TU_RENDERTARGET, 0, false, fsaa)).get();
}

//////////////////////////////////////////////////
unsigned int OgreRenderTexture::GLId()
{
  if (!this->ogreTexture)
    return 0u;

  GLuint texId;
  this->ogreTexture->getCustomAttribute("GLID", &texId);

  return static_cast<unsigned int>(texId);
}

//////////////////////////////////////////////////
void OgreRenderTexture::PreRender()
{
  OgreRenderTarget::PreRender();
}

//////////////////////////////////////////////////
void OgreRenderTexture::PostRender()
{
  OgreRenderTarget::PostRender();
}

//////////////////////////////////////////////////
void OgreRenderTexture::Buffer(float *_buffer)
{
  Ogre::RenderTarget *ogreRenderTarget = this->RenderTarget();

  if (nullptr == ogreRenderTarget)
  {
    ignerr << "Failed to set buffer: null render target" << std::endl;
    return;
  }

  ogreRenderTarget->swapBuffers();

  Ogre::HardwarePixelBufferSharedPtr pcdPixelBuffer;
  pcdPixelBuffer = this->ogreTexture->getBuffer();

  Ogre::PixelFormat imageFormat = OgreConversions::Convert(
      this->Format());

  Ogre::PixelBox ogrePixelBox(this->width, this->height, 1,
      imageFormat, _buffer);
  this->RenderTarget()->copyContentsToMemory(ogrePixelBox);
}


//////////////////////////////////////////////////
// OgreRenderWindow
//////////////////////////////////////////////////
OgreRenderWindow::OgreRenderWindow()
{
}

//////////////////////////////////////////////////
OgreRenderWindow::~OgreRenderWindow()
{
}

//////////////////////////////////////////////////
Ogre::RenderTarget *OgreRenderWindow::RenderTarget() const
{
  return this->ogreRenderWindow;
}

//////////////////////////////////////////////////
void OgreRenderWindow::Destroy()
{
  // if (this->ogreRenderWindow)
  //  this->ogreRenderWindow->destroy();
}

//////////////////////////////////////////////////
void OgreRenderWindow::RebuildTarget()
{
  // TODO(anyone) determine when to rebuild
  // ie. only when ratio or handle changes!
  // e.g. sizeDirty?
  if (!this->ogreRenderWindow)
    this->BuildTarget();

  Ogre::RenderWindow *window =
      dynamic_cast<Ogre::RenderWindow *>(this->ogreRenderWindow);

  if (nullptr == window)
  {
    ignerr << "Failed to cast render window." << std::endl;
    return;
  }

  window->resize(this->width, this->height);
  window->windowMovedOrResized();
}

//////////////////////////////////////////////////
void OgreRenderWindow::BuildTarget()
{
  auto engine = OgreRenderEngine::Instance();
  std::string renderTargetName =
      engine->CreateRenderWindow(this->handle,
          this->width,
          this->height,
          this->ratio,
          this->antiAliasing);

  if (renderTargetName.empty())
  {
    ignerr << "Failed to build target." << std::endl;
    return;
  }

  this->ogreRenderWindow =
      engine->OgreRoot()->getRenderTarget(renderTargetName);
}
