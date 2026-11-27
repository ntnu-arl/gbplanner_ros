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

#include <memory>
#include <gz/math/Color.hh>

#include "gz/common/Console.hh"
#include "gz/rendering/RenderTypes.hh"
#include "gz/rendering/ogre/OgreIncludes.hh"
#include "gz/rendering/ogre/OgreMaterialSwitcher.hh"
#include "gz/rendering/ogre/OgreSelectionBuffer.hh"

using namespace gz;
using namespace rendering;

class gz::rendering::OgreSelectionBufferPrivate
{
  /// \brief This is a material listener and a RenderTargetListener.
  /// The material switcher is applied to only the selection camera
  /// and not applied globally to all targets. The class associates a
  /// color to an ogre entity
  public: std::unique_ptr<OgreMaterialSwitcher> materialSwitcher;

  /// \brief Ogre scene manager
  public: Ogre::SceneManager *sceneMgr = nullptr;

  /// \brief Pointer to the camera that will be used as the reference
  /// for selection
  public: Ogre::Camera *camera = nullptr;

  /// \brief Selection buffer's render to texture camera
  public: Ogre::Camera *selectionCamera  = nullptr;

  /// \brief Ogre texture
  public: Ogre::TexturePtr texture;

  /// \brief Ogre render texture
  public: Ogre::RenderTexture *renderTexture  = nullptr;

  /// \brief Render texture data buffer
  public: uint8_t *buffer = nullptr;

  /// \brief Ogre pixel box that contains description of the data buffer
  public: Ogre::PixelBox *pixelBox = nullptr;

  /// \brief A 2D overlay used for debugging the selection buffer. It
  /// is hidden by default.
  public: Ogre::Overlay *selectionDebugOverlay = nullptr;
};

/////////////////////////////////////////////////
OgreSelectionBuffer::OgreSelectionBuffer(const std::string &_cameraName,
    Ogre::SceneManager *_mgr): dataPtr(new OgreSelectionBufferPrivate)
{
  this->dataPtr->sceneMgr = _mgr;

  this->dataPtr->camera = this->dataPtr->sceneMgr->getCamera(_cameraName);

  this->dataPtr->selectionCamera =
      this->dataPtr->sceneMgr->createCamera(_cameraName + "_selection_buffer");

  this->dataPtr->materialSwitcher.reset(new OgreMaterialSwitcher());
  this->CreateRTTBuffer();
  this->CreateRTTOverlays();
}

/////////////////////////////////////////////////
OgreSelectionBuffer::~OgreSelectionBuffer()
{
  this->DeleteRTTBuffer();

  // remove selection buffer camera
  this->dataPtr->sceneMgr->destroyCamera(this->dataPtr->selectionCamera);
}

/////////////////////////////////////////////////
void OgreSelectionBuffer::Update()
{
  if (!this->dataPtr->renderTexture)
    return;

  this->dataPtr->materialSwitcher->Reset();

  // FIXME: added try-catch block to prevent crash in deferred rendering mode.
  // RTT does not like VPL.material as it references a texture in the compositor
  // pipeline. A possible workaround is to add the deferred rendering
  // compositors to the renderTexture
  try
  {
    this->dataPtr->renderTexture->update();
  }
  catch(...)
  {
  }

  this->dataPtr->renderTexture->copyContentsToMemory(*this->dataPtr->pixelBox,
      Ogre::RenderTarget::FB_FRONT);
}

/////////////////////////////////////////////////
void OgreSelectionBuffer::DeleteRTTBuffer()
{
  auto &manager = Ogre::TextureManager::getSingleton();
  manager.unload(this->dataPtr->texture->getName());
  manager.remove(this->dataPtr->texture->getName());

  if (this->dataPtr->buffer)
  {
    delete [] this->dataPtr->buffer;
    this->dataPtr->buffer = nullptr;
  }
  if (this->dataPtr->pixelBox)
    delete this->dataPtr->pixelBox;
}

/////////////////////////////////////////////////
void OgreSelectionBuffer::CreateRTTBuffer()
{
  try
  {
    // 1x1 pixel buffer
    unsigned int width = 1;
    unsigned int height = 1;

    this->dataPtr->texture = Ogre::TextureManager::getSingleton().createManual(
        "SelectionPassTex",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D, width, height, 0, Ogre::PF_R8G8B8,
        Ogre::TU_RENDERTARGET);
  }
  catch(...)
  {
    this->dataPtr->renderTexture = nullptr;
    ignerr << "Unable to create selection buffer.\n";
    return;
  }

  this->dataPtr->renderTexture =
    this->dataPtr->texture->getBuffer()->getRenderTarget();
  this->dataPtr->renderTexture->setAutoUpdated(false);
  this->dataPtr->renderTexture->setPriority(0);
  Ogre::Viewport *vp =
      this->dataPtr->renderTexture->addViewport(this->dataPtr->selectionCamera);
  vp->setOverlaysEnabled(false);
  vp->setShadowsEnabled(false);
  vp->setClearEveryFrame(true);
  this->dataPtr->renderTexture->addListener(
      this->dataPtr->materialSwitcher.get());
  vp->setMaterialScheme("selection");
  vp->setVisibilityMask(
      IGN_VISIBILITY_SELECTABLE);
  Ogre::HardwarePixelBufferSharedPtr pixelBuffer =
    this->dataPtr->texture->getBuffer();
  size_t bufferSize = pixelBuffer->getSizeInBytes();

  this->dataPtr->buffer = new uint8_t[bufferSize];
  this->dataPtr->pixelBox = new Ogre::PixelBox(pixelBuffer->getWidth(),
      pixelBuffer->getHeight(), pixelBuffer->getDepth(),
      pixelBuffer->getFormat(), this->dataPtr->buffer);
}

/////////////////////////////////////////////////
Ogre::Entity *OgreSelectionBuffer::OnSelectionClick(const int _x, const int _y)
{
  if (!this->dataPtr->renderTexture)
    return nullptr;

  if (!this->dataPtr->camera)
    return nullptr;

  Ogre::Viewport *vp = this->dataPtr->camera->getViewport();

  if (!vp)
    return nullptr;

  Ogre::RenderTarget *rt = vp->getTarget();

  if (!rt)
    return nullptr;

  const unsigned int targetWidth = rt->getWidth();
  const unsigned int targetHeight = rt->getHeight();

  if (_x < 0 || _y < 0 || _x >= static_cast<int>(targetWidth)
      || _y >= static_cast<int>(targetHeight))
    return nullptr;

  // 1x1 selection buffer, adapted from rviz
  // http://docs.ros.org/indigo/api/rviz/html/c++/selection__manager_8cpp.html
  float width = 1.0f;
  float height = 1.0f;
  float x1 = static_cast<float>(_x) /
      static_cast<float>(targetWidth - 1) - 0.5f;
  float y1 = static_cast<float>(_y) /
      static_cast<float>(targetHeight - 1) - 0.5f;
  float x2 = static_cast<float>(_x+width) /
      static_cast<float>(targetWidth - 1) - 0.5f;
  float y2 = static_cast<float>(_y+height) /
      static_cast<float>(targetHeight - 1) - 0.5f;
  Ogre::Matrix4 scaleMatrix = Ogre::Matrix4::IDENTITY;
  Ogre::Matrix4 transMatrix = Ogre::Matrix4::IDENTITY;
  scaleMatrix[0][0] = 1.0 / (x2-x1);
  scaleMatrix[1][1] = 1.0 / (y2-y1);
  transMatrix[0][3] -= x1+x2;
  transMatrix[1][3] += y1+y2;
  this->dataPtr->selectionCamera->setCustomProjectionMatrix(true,
      scaleMatrix * transMatrix * this->dataPtr->camera->getProjectionMatrix());
  this->dataPtr->selectionCamera->setPosition(
      this->dataPtr->camera->getDerivedPosition());
  this->dataPtr->selectionCamera->setOrientation(
      this->dataPtr->camera->getDerivedOrientation());
  Ogre::Viewport* renderViewport = this->dataPtr->renderTexture->getViewport(0);
  renderViewport->setDimensions(0, 0, width, height);

  // update render texture
  this->Update();

  size_t posInStream = 0;

  math::Color::BGRA color(0);
  if (!this->dataPtr->buffer)
  {
    ignerr << "Selection buffer is null.\n";
    return nullptr;
  }
  memcpy(static_cast<void *>(&color), this->dataPtr->buffer + posInStream, 4);
  math::Color cv;
  cv.SetFromARGB(color);
  cv.A(1.0);
  const std::string &entName =
    this->dataPtr->materialSwitcher->EntityName(cv);

  if (entName.empty())
  {
    return 0;
  }
  else
    return this->dataPtr->sceneMgr->getEntity(entName);
}

/////////////////////////////////////////////////
void OgreSelectionBuffer::CreateRTTOverlays()
{
  Ogre::OverlayManager *mgr = Ogre::OverlayManager::getSingletonPtr();

  if (mgr && mgr->getByName("SelectionDebugOverlay"))
    return;

  Ogre::MaterialPtr baseWhite =
    Ogre::MaterialManager::getSingleton().getDefaultSettings();

  Ogre::MaterialPtr selectionBufferTexture =
    baseWhite->clone("SelectionDebugMaterial");
  Ogre::TextureUnitState *textureUnit =
    selectionBufferTexture->getTechnique(0)->getPass(0)->
    createTextureUnitState();
  textureUnit->setTextureName("SelectionPassTex");

  this->dataPtr->selectionDebugOverlay = mgr->create("SelectionDebugOverlay");

  Ogre::OverlayContainer *panel =
    static_cast<Ogre::OverlayContainer *>(
        mgr->createOverlayElement("Panel", "SelectionDebugPanel"));

  if (panel)
  {
    panel->setMetricsMode(Ogre::GMM_PIXELS);
    panel->setPosition(10, 10);
    panel->setDimensions(400, 280);
    panel->setMaterialName("SelectionDebugMaterial");
#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR  <= 9
    this->dataPtr->selectionDebugOverlay->add2D(panel);
    this->dataPtr->selectionDebugOverlay->hide();
#endif
  }
  else
  {
    ignlog << "Unable to create selection buffer overlay. "
      "This will not effect ignition-rendering unless you're trying to debug "
      "the selection buffer.\n";
  }
}

/////////////////////////////////////////////////
#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR  <= 9
void OgreSelectionBuffer::ShowOverlay(const bool _show)
{
  if (_show)
    this->dataPtr->selectionDebugOverlay->show();
  else
    this->dataPtr->selectionDebugOverlay->hide();
}
#else
void OgreSelectionBuffer::ShowOverlay(const bool _show)
{
  ignerr << "Selection debug overlay disabled for Ogre > 1.9\n";
}
#endif
