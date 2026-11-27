/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <string>

#include <ignition/common/Console.hh>
#include <ignition/math/Color.hh>

#include "ignition/rendering/ogre2/Ogre2Camera.hh"
#include "ignition/rendering/ogre2/Ogre2Conversions.hh"
#include "ignition/rendering/ogre2/Ogre2Includes.hh"
#include "ignition/rendering/ogre2/Ogre2ParticleEmitter.hh"
#include "ignition/rendering/ogre2/Ogre2RenderTarget.hh"
#include "ignition/rendering/ogre2/Ogre2RenderTypes.hh"
#include "ignition/rendering/ogre2/Ogre2RenderEngine.hh"
#include "ignition/rendering/ogre2/Ogre2Scene.hh"
#include "ignition/rendering/ogre2/Ogre2SegmentationCamera.hh"
#include "ignition/rendering/RenderTypes.hh"
#include "ignition/rendering/Utils.hh"

#include "Ogre2SegmentationMaterialSwitcher.hh"

/// \brief Private data for the Ogre2SegmentationCamera class
class ignition::rendering::Ogre2SegmentationCameraPrivate
{
  /// \brief buffer to store render texture data & to be sent to listeners
  public: uint8_t *buffer {nullptr};

  /// \brief Workspace Definition
  public: std::string ogreCompositorWorkspaceDef;

  /// \brief Final pass compositor node definition
  public: std::string ogreCompositorNodeDef;

  /// \brief 1st pass compositor workspace
  public: Ogre::CompositorWorkspace *ogreCompositorWorkspace {nullptr};

  /// \brief Output texture
  public: Ogre::TextureGpu *ogreSegmentationTexture {nullptr};

  /// \brief Dummy render texture for the depth data
  public: RenderTexturePtr segmentationTexture {nullptr};

  /// \brief New Segmentation Frame Event to notify listeners with new data
  /// \param[in] _data Segmentation buffer data
  /// \param[in] _width Width of the image
  /// \param[in] _height Height of the image
  /// \param[in] _channels Number of channels
  /// \param[in] _format Image Format
  public: ignition::common::EventT<void(const uint8_t *_data,
    unsigned int _width, unsigned int _height, unsigned int _channels,
    const std::string &_format)> newSegmentationFrame;

  /// \brief Material Switcher to switch item's material
  /// with colored version for segmentation
  public: std::unique_ptr<Ogre2SegmentationMaterialSwitcher>
          materialSwitcher {nullptr};
};

using namespace ignition;
using namespace rendering;

/////////////////////////////////////////////////
Ogre2SegmentationCamera::Ogre2SegmentationCamera() :
  dataPtr(new Ogre2SegmentationCameraPrivate())
{
}

/////////////////////////////////////////////////
Ogre2SegmentationCamera::~Ogre2SegmentationCamera()
{
  this->Destroy();
}

/////////////////////////////////////////////////
void Ogre2SegmentationCamera::Init()
{
  BaseCamera::Init();

  this->CreateCamera();

  this->CreateRenderTexture();

  this->dataPtr->materialSwitcher.reset(
      new Ogre2SegmentationMaterialSwitcher(this->scene, this));
}

/////////////////////////////////////////////////
void Ogre2SegmentationCamera::Destroy()
{
  if (this->dataPtr->buffer)
  {
    delete [] this->dataPtr->buffer;
    this->dataPtr->buffer = nullptr;
  }

  if (!this->ogreCamera)
    return;

  auto engine = Ogre2RenderEngine::Instance();
  auto ogreRoot = engine->OgreRoot();
  auto ogreCompMgr = ogreRoot->getCompositorManager2();

  if (this->dataPtr->ogreSegmentationTexture)
  {
    ogreRoot->getRenderSystem()->getTextureGpuManager()->destroyTexture(
      this->dataPtr->ogreSegmentationTexture);
    this->dataPtr->ogreSegmentationTexture = nullptr;

  }
  if (this->dataPtr->ogreCompositorWorkspace)
  {
    ogreCompMgr->removeWorkspace(
        this->dataPtr->ogreCompositorWorkspace);
    this->dataPtr->ogreCompositorWorkspace = nullptr;
  }

  if (!this->dataPtr->ogreCompositorWorkspaceDef.empty())
  {
    ogreCompMgr->removeWorkspaceDefinition(
        this->dataPtr->ogreCompositorWorkspaceDef);
    ogreCompMgr->removeNodeDefinition(
        this->dataPtr->ogreCompositorNodeDef);
  }

  Ogre::SceneManager *ogreSceneManager;
  ogreSceneManager = this->scene->OgreSceneManager();
  if (ogreSceneManager == nullptr)
  {
    ignerr << "Scene manager cannot be obtained" << std::endl;
  }
  else
  {
    if (ogreSceneManager->findCameraNoThrow(this->name) != nullptr)
    {
      ogreSceneManager->destroyCamera(this->ogreCamera);
      this->ogreCamera = nullptr;
    }
  }

  this->dataPtr->materialSwitcher.reset();
}

/////////////////////////////////////////////////
void Ogre2SegmentationCamera::PreRender()
{
  if (!this->dataPtr->ogreSegmentationTexture)
    this->CreateSegmentationTexture();
}

/////////////////////////////////////////////////
void Ogre2SegmentationCamera::CreateCamera()
{
  auto ogreSceneManager = this->scene->OgreSceneManager();
  if (ogreSceneManager == nullptr)
  {
    ignerr << "Scene manager cannot be obtained" << std::endl;
    return;
  }

  this->ogreCamera = ogreSceneManager->createCamera(this->Name());
  if (this->ogreCamera == nullptr)
  {
    ignerr << "Ogre camera cannot be created" << std::endl;
    return;
  }

  this->ogreCamera->detachFromParent();
  this->ogreNode->attachObject(this->ogreCamera);

  // rotate to ignition gazebo coord.
  this->ogreCamera->yaw(Ogre::Degree(-90));
  this->ogreCamera->roll(Ogre::Degree(-90));
  this->ogreCamera->setFixedYawAxis(false);

  this->ogreCamera->setAutoAspectRatio(true);
  this->ogreCamera->setRenderingDistance(100);
  this->ogreCamera->setProjectionType(Ogre::ProjectionType::PT_PERSPECTIVE);
  this->ogreCamera->setCustomProjectionMatrix(false);
}

/////////////////////////////////////////////////
void Ogre2SegmentationCamera::CreateSegmentationTexture()
{
  // Camera Parameters
  this->ogreCamera->setNearClipDistance(this->NearClipPlane());
  this->ogreCamera->setFarClipDistance(this->FarClipPlane());
  this->ogreCamera->setAspectRatio(this->AspectRatio());
  double vfov = 2.0 * atan(tan(this->HFOV().Radian() / 2.0) /
    this->AspectRatio());
  this->ogreCamera->setFOVy(Ogre::Radian(vfov));

  auto engine = Ogre2RenderEngine::Instance();
  auto ogreRoot = engine->OgreRoot();
  Ogre::CompositorManager2 *ogreCompMgr = ogreRoot->getCompositorManager2();

  this->SetImageFormat(PixelFormat::PF_R8G8B8);
  Ogre::PixelFormatGpu ogrePF = Ogre::PFG_RGBA8_UNORM;

  auto backgroundColor_ = Ogre2Conversions::Convert(
      this->backgroundColor);

  std::string wsDefName = "SegmentationCameraWorkspace_" + this->Name();
  ogreCompMgr->createBasicWorkspaceDef(wsDefName, backgroundColor_);

  Ogre::TextureGpuManager *textureMgr =
    ogreRoot->getRenderSystem()->getTextureGpuManager();
  // create render texture
  this->dataPtr->ogreSegmentationTexture =
    textureMgr->createOrRetrieveTexture(this->Name() + "_segmentation",
      Ogre::GpuPageOutStrategy::SaveToSystemRam,
      Ogre::TextureFlags::RenderToTexture,
      Ogre::TextureTypes::Type2D);

  this->dataPtr->ogreSegmentationTexture->setResolution(
      this->ImageWidth(), this->ImageHeight());
  this->dataPtr->ogreSegmentationTexture->setNumMipmaps(1u);
  this->dataPtr->ogreSegmentationTexture->setPixelFormat(ogrePF);
  this->dataPtr->ogreSegmentationTexture->scheduleTransitionTo(
    Ogre::GpuResidency::Resident);

  // create compositor worksspace
  this->dataPtr->ogreCompositorWorkspace =
      ogreCompMgr->addWorkspace(
        this->scene->OgreSceneManager(),
        this->dataPtr->ogreSegmentationTexture,
        this->ogreCamera,
        wsDefName,
        false);

  this->ogreCamera->addListener(
    this->dataPtr->materialSwitcher.get());
}

/////////////////////////////////////////////////
void Ogre2SegmentationCamera::PostRender()
{
  // return if no one is listening to the new frame
  if (this->dataPtr->newSegmentationFrame.ConnectionCount() == 0)
    return;

  const auto width = this->ImageWidth();
  const auto height = this->ImageHeight();
  PixelFormat format = this->ImageFormat();

  const auto len = width * height;
  const auto channelCount = PixelUtil::ChannelCount(format);
  const auto bytesPerChannel = PixelUtil::BytesPerChannel(format);
  const auto bufferSize = len * channelCount * bytesPerChannel;

  Ogre::Image2 image;
  image.convertFromTexture(this->dataPtr->ogreSegmentationTexture, 0u, 0u);
  Ogre::TextureBox box = image.getData(0);

  if (!this->dataPtr->buffer)
  {
    this->dataPtr->buffer = new uint8_t[bufferSize];
  }

  uint8_t *bufferTmp = static_cast<uint8_t*>(box.data);

  auto rawChannelCount = 4u;

  for (unsigned int row = 0; row < height; ++row)
  {
    unsigned int rawDataRowIdx = row * box.bytesPerRow / bytesPerChannel;
    for (unsigned int column = 0; column < width; ++column)
    {
      unsigned int idx = (row * width * channelCount) +
          column * channelCount;
      unsigned int rawIdx = rawDataRowIdx +
          column * rawChannelCount;

      this->dataPtr->buffer[idx] = bufferTmp[rawIdx];
      this->dataPtr->buffer[idx + 1] = bufferTmp[rawIdx + 1];
      this->dataPtr->buffer[idx + 2] = bufferTmp[rawIdx + 2];
    }
  }

  this->dataPtr->newSegmentationFrame(
    this->dataPtr->buffer,
    width, height, channelCount,
    PixelUtil::Name(format));
}

/////////////////////////////////////////////////
ignition::common::ConnectionPtr
  Ogre2SegmentationCamera::ConnectNewSegmentationFrame(
  std::function<void(const uint8_t *, unsigned int, unsigned int,
  unsigned int, const std::string &)>  _subscriber)
{
  return this->dataPtr->newSegmentationFrame.Connect(_subscriber);
}

/////////////////////////////////////////////////
void Ogre2SegmentationCamera::Render()
{
  // update the compositors
  this->scene->StartRendering(nullptr);

  this->dataPtr->ogreCompositorWorkspace->_validateFinalTarget();
  this->dataPtr->ogreCompositorWorkspace->_beginUpdate(false);
  this->dataPtr->ogreCompositorWorkspace->_update();
  this->dataPtr->ogreCompositorWorkspace->_endUpdate(false);

  Ogre::vector<Ogre::TextureGpu*>::type swappedTargets;
  swappedTargets.reserve(2u);
  this->dataPtr->ogreCompositorWorkspace->_swapFinalTarget(swappedTargets);

  this->scene->FlushGpuCommandsAndStartNewFrame(1u, false);
}

/////////////////////////////////////////////////
RenderTargetPtr Ogre2SegmentationCamera::RenderTarget() const
{
  return this->dataPtr->segmentationTexture;
}

/////////////////////////////////////////////////
void Ogre2SegmentationCamera::CreateRenderTexture()
{
  RenderTexturePtr base = this->scene->CreateRenderTexture();
  this->dataPtr->segmentationTexture =
    std::dynamic_pointer_cast<Ogre2RenderTexture>(base);
  this->dataPtr->segmentationTexture->SetWidth(1);
  this->dataPtr->segmentationTexture->SetHeight(1);
}

/////////////////////////////////////////////////
void Ogre2SegmentationCamera::SetBackgroundLabel(int _label)
{
  this->backgroundLabel = _label;
  this->SetBackgroundColor(
    math::Color(_label / 255.0, _label / 255.0, _label / 255.0));
}

/////////////////////////////////////////////////
void Ogre2SegmentationCamera::LabelMapFromColoredBuffer(
  uint8_t * _labelBuffer) const
{
  if (!this->isColoredMap)
    return;

  if (!this->dataPtr->buffer)
    return;

  auto colorToLabel = this->dataPtr->materialSwitcher->ColorToLabel();

  auto width = this->ImageWidth();
  auto height = this->ImageHeight();

  for (uint32_t i = 0; i < height; ++i)
  {
    for (uint32_t j = 0; j < width; ++j)
    {
      auto index = (i * width + j) * 3;
      auto r = this->dataPtr->buffer[index];
      auto g = this->dataPtr->buffer[index + 1];
      auto b = this->dataPtr->buffer[index + 2];

      // get color 24 bit unique id, we don't multiply it by 255 like before
      // as they are not normalized we read it from the buffer in
      // range [0-255] already
      int64_t colorId = r * 256 * 256 + g * 256 + b;

      // initialize the pixel with the background label value
      _labelBuffer[index] = this->backgroundLabel;
      _labelBuffer[index + 1] = this->backgroundLabel;
      _labelBuffer[index + 2] = this->backgroundLabel;

      // skip if not exist
      auto it = colorToLabel.find(colorId);
      if (it == colorToLabel.end())
        continue;

      int64_t label = it->second;

      if (this->type == SegmentationType::ST_SEMANTIC)
      {
        uint8_t label8bit = label % 256;

        _labelBuffer[index] = label8bit;
        _labelBuffer[index + 1] = label8bit;
        _labelBuffer[index + 2] = label8bit;
      }
      else if (this->type == SegmentationType::ST_PANOPTIC)
      {
        // get the label and instance counts from the composite label id
        uint8_t label8bit = label / (256 * 256);
        // get the rest 16 bit
        uint16_t instanceCount = label % (256 * 256);
        // composite that 16 bit count to two 8 bit channels
        uint8_t instanceCount1 = instanceCount / 256;
        uint8_t instanceCount2 = instanceCount % 256;

        _labelBuffer[index + 2] = label8bit;
        _labelBuffer[index + 1] = instanceCount1;
        _labelBuffer[index] = instanceCount2;
      }
    }
  }
}
