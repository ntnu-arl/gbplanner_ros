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

#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/SubMesh.hh>

#include <gz/math/Color.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Vector3.hh>

#include "gz/rendering/RenderTypes.hh"
#include "gz/rendering/ogre/OgreCamera.hh"
#include "gz/rendering/ogre/OgreGpuRays.hh"

/// \internal
/// \brief Private data for the OgreGpuRays class
class gz::rendering::OgreGpuRaysPrivate
{
  /// \brief Event triggered when new gpu rays range data are available.
  /// \param[in] _frame New frame containing raw gpu rays data.
  /// \param[in] _width Width of frame.
  /// \param[in] _height Height of frame.
  /// \param[in] _channels Number of channels
  /// \param[in] _format Format of frame.
  public: gz::common::EventT<void(const float *,
               unsigned int, unsigned int, unsigned int,
               const std::string &)> newGpuRaysFrame;

  /// \brief Raw buffer of gpu rays data.
  public: float *gpuRaysBuffer = nullptr;

  /// \brief Outgoing gpu rays data, used by newGpuRaysFrame event.
  public: float *gpuRaysScan = nullptr;

  /// \brief Pointer to Ogre material for the first rendering pass.
  public: Ogre::Material *matFirstPass = nullptr;

  /// \brief Pointer to Ogre material for the second rendering pass.
  public: Ogre::Material *matSecondPass = nullptr;

  /// \brief Temporary pointer to the current material.
  public: Ogre::Material *currentMat = nullptr;

  /// \brief An array of first pass textures.
  public: Ogre::Texture *firstPassTextures[3];

  /// \brief Second pass texture.
  public: Ogre::Texture *secondPassTexture = nullptr;

  /// \brief Temporary pointer to the current render target.
  public: Ogre::Texture *currentTexture = nullptr;

  /// \brief Ogre orthorgraphic camera used in the second pass for
  /// undistortion.
  public: Ogre::Camera *orthoCam = nullptr;

  /// \brief Pointer to the ogre camera
  public: Ogre::Camera *ogreCamera = nullptr;

  /// \brief Ogre scenenode where the orthorgraphic camera is attached to.
  public: Ogre::SceneNode *pitchNodeOrtho = nullptr;

  /// \brief Ogre mesh used to create a canvas for undistorting range values
  /// in the second rendering pass.
  public: common::Mesh *undistMesh = nullptr;

  /// \brief Pointer to visual that holds the canvas.
  public: VisualPtr visual;

  /// \brief Number of first pass textures.
  public: unsigned int textureCount = 0u;

  /// \brief A list of camera angles for first pass rendering.
  public: double cameraYaws[4];

  /// \brief Image width of first pass.
  public: unsigned int w1st = 0u;

  /// \brief Image height of first pass.
  public: unsigned int h1st = 0u;

  /// \brief Image width of second pass.
  public: unsigned int w2nd = 0u;

  /// \brief Image height of second pass.
  public: unsigned int h2nd = 0u;

  /// \brief List of texture unit indices used during the second
  /// rendering pass.
  public: std::vector<int> texIdx;

  /// Number of second pass texture units created.
  public: unsigned int texCount = 0;

  /// \brief Dummy render texture for the gpu rays
  public: RenderTexturePtr renderTexture;

  /// \brief Cos horizontal field-of-view.
  public: double chfov = 0;

  /// \brief Cos vertical field-of-view.
  public: double cvfov = 0;

  /// \brief Horizontal half angle.
  public: double horzHalfAngle = 0;

  /// \brief Vertical half angle.
  public: double vertHalfAngle = 0;

  /// \brief Number of cameras needed to generate the rays.
  public: unsigned int cameraCount = 1;

  /// \brief Min allowed angle in radians;
  public: const math::Angle kMinAllowedAngle = 1e-4;
};

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
OgreGpuRays::OgreGpuRays()
  : dataPtr(new OgreGpuRaysPrivate)
{
  // r = depth, g = retro, and b = n/a
  this->channels = 3u;
}

//////////////////////////////////////////////////
OgreGpuRays::~OgreGpuRays()
{
  this->Destroy();
}

//////////////////////////////////////////////////
void OgreGpuRays::Init()
{
  BaseGpuRays::Init();

  // create internal camera for 1st pass
  this->CreateCamera();

  // create dummy render texture
  this->CreateRenderTexture();
}

//////////////////////////////////////////////////
void OgreGpuRays::Destroy()
{
  if (this->dataPtr->gpuRaysBuffer)
  {
    delete [] this->dataPtr->gpuRaysBuffer;
    this->dataPtr->gpuRaysBuffer = nullptr;
  }

  if (this->dataPtr->gpuRaysScan)
  {
    delete [] this->dataPtr->gpuRaysScan;
    this->dataPtr->gpuRaysScan = nullptr;
  }

  for (unsigned int i = 0; i < this->dataPtr->textureCount; ++i)
  {
    if (this->dataPtr->firstPassTextures[i])
    {
      Ogre::TextureManager::getSingleton().remove(
          this->dataPtr->firstPassTextures[i]->getName());
      this->dataPtr->firstPassTextures[i] = nullptr;
    }
  }

  if (this->dataPtr->secondPassTexture)
  {
    Ogre::TextureManager::getSingleton().remove(
        this->dataPtr->secondPassTexture->getName());
    this->dataPtr->secondPassTexture = nullptr;
  }

  if (this->dataPtr->matSecondPass)
  {
    Ogre::MaterialManager::getSingleton().remove(
        this->dataPtr->matSecondPass->getName());
    this->dataPtr->matSecondPass = nullptr;
  }

  if (this->scene && this->dataPtr->orthoCam)
  {
    this->scene->OgreSceneManager()->destroyCamera(this->dataPtr->orthoCam);
    this->dataPtr->orthoCam = nullptr;
  }

  this->dataPtr->visual.reset();
  this->dataPtr->texIdx.clear();
  this->dataPtr->texCount = 0u;
}

/////////////////////////////////////////////////
void OgreGpuRays::CreateRenderTexture()
{
  RenderTexturePtr base = this->scene->CreateRenderTexture();
  this->dataPtr->renderTexture =
      std::dynamic_pointer_cast<OgreRenderTexture>(base);
  this->dataPtr->renderTexture->SetWidth(1);
  this->dataPtr->renderTexture->SetHeight(1);
}

/////////////////////////////////////////////////
void OgreGpuRays::CreateCamera()
{
  // Create ogre camera object
  Ogre::SceneManager *ogreSceneManager  = this->scene->OgreSceneManager();
  if (ogreSceneManager == nullptr)
  {
    ignerr << "Scene manager cannot be obtained" << std::endl;
    return;
  }

  this->dataPtr->ogreCamera = ogreSceneManager->createCamera(
      this->Name() + "_Camera");
  if (this->dataPtr->ogreCamera == nullptr)
  {
    ignerr << "Ogre camera cannot be created" << std::endl;
    return;
  }

  this->ogreNode->attachObject(this->dataPtr->ogreCamera);
  this->dataPtr->ogreCamera->setFixedYawAxis(false);
  this->dataPtr->ogreCamera->yaw(Ogre::Degree(-90));
  this->dataPtr->ogreCamera->roll(Ogre::Degree(-90));
  this->dataPtr->ogreCamera->setAutoAspectRatio(true);
}

/////////////////////////////////////////////////
void OgreGpuRays::ConfigureCameras()
{
  // horizontal gpu rays setup
  auto hfovAngle = this->AngleMax() - this->AngleMin();
  hfovAngle = std::max(this->dataPtr->kMinAllowedAngle, hfovAngle);
  this->SetHFOV(hfovAngle);

  if (this->HFOV().Radian() > 2.0 * IGN_PI)
  {
    this->SetHFOV(2.0 * IGN_PI);
    ignwarn << "Horizontal FOV for GPU rays is capped at 180 degrees.\n";
  }

  this->SetHorzHalfAngle((this->AngleMax() + this->AngleMin()).Radian() / 2.0);

  // determine number of cameras to use
  if (this->HFOV().Radian() > 2.8)
  {
    if (this->HFOV().Radian()  > 5.6)
    {
      this->dataPtr->cameraCount = 3;
    }
    else
    {
      this->dataPtr->cameraCount = 2;
    }
  }
  else
  {
    this->dataPtr->cameraCount = 1;
  }

  // horizontal fov of single frame
  this->SetHFOV(this->HFOV().Radian() / this->dataPtr->cameraCount);
  this->SetCosHorzFOV(this->HFOV().Radian());

  // Fixed minimum resolution of texture to reduce steps in ranges
  // when hitting surfaces where the angle between ray and surface is small.
  // Also have to keep in mind the GPU's max. texture size
  unsigned int horzRangeCountPerCamera =
      std::max(2048U, this->RangeCount() / this->dataPtr->cameraCount);
  unsigned int vertRangeCountPerCamera = this->VerticalRangeCount();

  // vertical laser setup
  double vfovAngle;

  if (this->VerticalRangeCount() > 1)
  {
    vfovAngle = std::max(this->dataPtr->kMinAllowedAngle.Radian(),
        (this->VerticalAngleMax() - this->VerticalAngleMin()).Radian());
  }
  else
  {
    vfovAngle = 0;

    if (this->VerticalAngleMax() != this->VerticalAngleMin())
    {
      ignwarn << "Only one vertical ray but vertical min. and max. angle "
          "are not equal. Min. angle is used.\n";
      this->SetVerticalAngleMax(this->VerticalAngleMin().Radian());
    }
  }

  if (vfovAngle > IGN_PI / 2.0)
  {
    vfovAngle = IGN_PI / 2.0;
    ignwarn << "Vertical FOV for GPU laser is capped at 90 degrees.\n";
  }

  this->SetVFOV(vfovAngle);
  this->SetVertHalfAngle((this->VerticalAngleMax()
                   + this->VerticalAngleMin()).Radian() / 2.0);

  this->SetVerticalAngleMin(this->VertHalfAngle() - (vfovAngle / 2));
  this->SetVerticalAngleMax(this->VertHalfAngle() + (vfovAngle / 2));

  // Assume camera always stays horizontally even if vert. half angle of
  // laser is not 0. Add padding to camera vfov.
  double vfovCamera = vfovAngle + 2.0 * std::abs(this->VertHalfAngle());

  // Add padding to vertical camera FOV to cover all possible rays
  // for given laser vert. and horiz. FOV
  vfovCamera = 2.0 * atan(tan(vfovCamera / 2.0) / cos(
        this->HFOV().Radian() / 2.0));

  if (vfovCamera > 2.8)
  {
    ignerr << "Vertical FOV of internal camera exceeds 2.8 radians.\n";
  }

  this->SetCosVertFOV(vfovCamera);

  // If vertical ray is not 1 adjust horizontal and vertical
  // ray count to maintain aspect ratio
  if (this->vSamples > 1)
  {
    double cameraAspectRatio =
      tan(this->HFOV().Radian() / 2.0) / tan(vfovCamera / 2.0);

    this->SetRayCountRatio(cameraAspectRatio);
    this->rangeCountRatio = cameraAspectRatio;

    if ((horzRangeCountPerCamera / this->rangeCountRatio) >
         vertRangeCountPerCamera)
    {
      vertRangeCountPerCamera = static_cast<unsigned int>(
          std::round(horzRangeCountPerCamera / this->rangeCountRatio));
    }
    else
    {
      horzRangeCountPerCamera = static_cast<unsigned int>(std::round(
          vertRangeCountPerCamera * this->rangeCountRatio));
    }
  }
  else
  {
    // In case of 1 vert. ray, set a very small vertical FOV for camera
    this->SetRayCountRatio(horzRangeCountPerCamera);
    double camVFOV = 2.0 * atan(tan(
          this->HFOV().Radian() / 2.0) / this->RayCountRatio());
    this->SetCosVertFOV(camVFOV);
  }

  // Configure first pass texture size
  this->Set1stTextureSize(horzRangeCountPerCamera, vertRangeCountPerCamera);
  // Configure second pass texture size
  this->SetRangeCount(this->RangeCount(), this->VerticalRangeCount());

  // Set ogre cam properties
  this->dataPtr->ogreCamera->setAspectRatio(this->RayCountRatio());
  this->dataPtr->ogreCamera->setFOVy(Ogre::Radian((this->CosVertFOV())));
  this->dataPtr->ogreCamera->setNearClipDistance(this->NearClipPlane());
  this->dataPtr->ogreCamera->setFarClipDistance(this->FarClipPlane());
  this->dataPtr->ogreCamera->setRenderingDistance(this->FarClipPlane());
  this->dataPtr->ogreCamera->yaw(Ogre::Radian(this->HorzHalfAngle()));
}

/////////////////////////////////////////////////////////
void OgreGpuRays::CreateGpuRaysTextures()
{
  this->ConfigureCameras();

  this->CreateOrthoCam();

  this->dataPtr->textureCount = this->dataPtr->cameraCount;

  if (this->dataPtr->textureCount == 2)
  {
    this->dataPtr->cameraYaws[0] = -this->HFOV().Radian() / 2.0;
    this->dataPtr->cameraYaws[1] = +this->HFOV().Radian();
    this->dataPtr->cameraYaws[2] = 0;
    this->dataPtr->cameraYaws[3] = -this->HFOV().Radian() / 2.0;
  }
  else
  {
    this->dataPtr->cameraYaws[0] = -this->HFOV().Radian();
    this->dataPtr->cameraYaws[1] = +this->HFOV().Radian();
    this->dataPtr->cameraYaws[2] = +this->HFOV().Radian();
    this->dataPtr->cameraYaws[3] = -this->HFOV().Radian();
  }

  // Configure first pass textures that are not yet configured properly
  for (unsigned int i = 0; i < this->dataPtr->textureCount; ++i)
  {
    std::stringstream texName;
    texName << this->Name() << "_first_pass_" << i;
    this->dataPtr->firstPassTextures[i] =
      Ogre::TextureManager::getSingleton().createManual(
      texName.str(), "General", Ogre::TEX_TYPE_2D,
      this->dataPtr->w1st, this->dataPtr->h1st, 0,
#if OGRE_VERSION_LT_1_11_0
      Ogre::PF_FLOAT32_RGB, Ogre::TU_RENDERTARGET).getPointer();
#else
      Ogre::PF_FLOAT32_RGB, Ogre::TU_RENDERTARGET).get();
#endif

    Ogre::RenderTarget *rt =
        this->dataPtr->firstPassTextures[i]->getBuffer()->getRenderTarget();
    rt->setAutoUpdated(false);

    // Setup the viewport to use the texture
    Ogre::Viewport *vp =
        rt->addViewport(this->dataPtr->ogreCamera);
    vp->setClearEveryFrame(true);
    vp->setOverlaysEnabled(false);
    vp->setShadowsEnabled(false);
    vp->setSkiesEnabled(false);
    vp->setBackgroundColour(
        Ogre::ColourValue(this->dataMaxVal, 0.0, 1.0));
    vp->setVisibilityMask(this->VisibilityMask() &
        ~(IGN_VISIBILITY_GUI | IGN_VISIBILITY_SELECTABLE));
  }

  this->dataPtr->matFirstPass = dynamic_cast<Ogre::Material *>(
      Ogre::MaterialManager::getSingleton().getByName("GpuRaysScan1st").get());
  this->dataPtr->matFirstPass->load();
  this->dataPtr->matFirstPass->setCullingMode(Ogre::CULL_NONE);

  // Configure second pass texture
  this->dataPtr->secondPassTexture =
      Ogre::TextureManager::getSingleton().createManual(
      this->Name() + "_second_pass",
      "General",
      Ogre::TEX_TYPE_2D,
      this->dataPtr->w2nd, this->dataPtr->h2nd, 0,
      Ogre::PF_FLOAT32_RGB,
#if OGRE_VERSION_LT_1_11_0
      Ogre::TU_RENDERTARGET).getPointer();
#else
      Ogre::TU_RENDERTARGET).get();
#endif

  Ogre::RenderTarget *rt =
      this->dataPtr->secondPassTexture->getBuffer()->getRenderTarget();
  rt->setAutoUpdated(false);

  // Setup the viewport to use the texture
  Ogre::Viewport *vp = rt->addViewport(this->dataPtr->orthoCam);
  vp->setClearEveryFrame(true);
  vp->setOverlaysEnabled(false);
  vp->setShadowsEnabled(false);
  vp->setSkiesEnabled(false);
  vp->setBackgroundColour(Ogre::ColourValue(0.0, 1.0, 0.0));
  vp->setVisibilityMask(
      IGN_VISIBILITY_ALL & ~(IGN_VISIBILITY_GUI | IGN_VISIBILITY_SELECTABLE));

  Ogre::Matrix4 p = this->BuildScaledOrthoMatrix(
      0, static_cast<float>(this->dataPtr->w2nd / 10.0),
      0, static_cast<float>(this->dataPtr->h2nd / 10.0),
      0.01f, 0.02f);

  this->dataPtr->orthoCam->setCustomProjectionMatrix(true, p);

  // Set GpuRaysScan2nd material
  this->dataPtr->matSecondPass = dynamic_cast<Ogre::Material *>(
      Ogre::MaterialManager::getSingleton().getByName("GpuRaysScan2nd").get());
  // clone the material since we're modifying it's definitions
  this->dataPtr->matSecondPass = this->dataPtr->matSecondPass->clone(
      this->Name() + "_" + this->dataPtr->matSecondPass->getName()).get();
  this->dataPtr->matSecondPass->load();

  Ogre::Technique *technique =
    this->dataPtr->matSecondPass->getTechnique(0);
  IGN_ASSERT(technique,
      "OgreGpuRays material script error: technique not found");

  Ogre::Pass *pass = technique->getPass(0);
  IGN_ASSERT(pass,
      "OgreGpuRays material script error: pass not found");
  pass->removeAllTextureUnitStates();
  Ogre::TextureUnitState *texUnit = nullptr;
  for (unsigned int i = 0; i < this->dataPtr->textureCount; ++i)
  {
    unsigned int texIndex = this->dataPtr->texCount++;
    {
      texUnit = pass->createTextureUnitState(
            this->dataPtr->firstPassTextures[i]->getName(), texIndex);

      this->dataPtr->texIdx.push_back(texIndex);

      texUnit->setTextureFiltering(Ogre::TFO_NONE);
      texUnit->setTextureAddressingMode(Ogre::TextureUnitState::TAM_MIRROR);
    }
  }

  this->CreateCanvas();
}

/////////////////////////////////////////////////
void OgreGpuRays::UpdateRenderTarget(Ogre::RenderTarget *_target,
                   Ogre::Material *_material, Ogre::Camera *_cam,
                   const bool _updateTex)
{
  Ogre::SceneManager *sceneMgr = this->scene->OgreSceneManager();
  Ogre::RenderSystem *renderSys = sceneMgr->getDestinationRenderSystem();
  // Get pointer to the material pass
  Ogre::Pass *pass = _material->getBestTechnique()->getPass(0);

  // Render the depth texture
  // OgreSceneManager::_render function automatically sets farClip to 0.
  // Which normally equates to infinite distance. We don't want this. So
  // we have to set the distance every time.
  _cam->setFarClipDistance(this->FarClipPlane());

  Ogre::AutoParamDataSource autoParamDataSource;

  Ogre::Viewport *vp = _target->getViewport(0);

  // Need this line to render the ground plane. No idea why it's necessary.
  renderSys->_setViewport(vp);
  sceneMgr->_setPass(pass, true, false);
  autoParamDataSource.setCurrentPass(pass);
  autoParamDataSource.setCurrentViewport(vp);
  autoParamDataSource.setCurrentRenderTarget(_target);
  autoParamDataSource.setCurrentSceneManager(sceneMgr);
  autoParamDataSource.setCurrentCamera(_cam, true);

  renderSys->setLightingEnabled(false);
  renderSys->_setFog(Ogre::FOG_NONE);

  pass->_updateAutoParams(&autoParamDataSource, 1);

  if (_updateTex)
  {
    pass->getFragmentProgramParameters()->setNamedConstant("tex1",
      this->dataPtr->texIdx[0]);
    if (this->dataPtr->texIdx.size() > 1)
    {
      pass->getFragmentProgramParameters()->setNamedConstant("tex2",
        this->dataPtr->texIdx[1]);
      if (this->dataPtr->texIdx.size() > 2)
        pass->getFragmentProgramParameters()->setNamedConstant("tex3",
          this->dataPtr->texIdx[2]);
    }
  }

  // NOTE: We MUST bind parameters AFTER updating the autos
  if (pass->hasVertexProgram())
  {
    renderSys->bindGpuProgram(
        pass->getVertexProgram()->_getBindingDelegate());

    renderSys->bindGpuProgramParameters(Ogre::GPT_VERTEX_PROGRAM,
    pass->getVertexProgramParameters(), 1);
  }

  if (pass->hasFragmentProgram())
  {
    renderSys->bindGpuProgram(
    pass->getFragmentProgram()->_getBindingDelegate());

    renderSys->bindGpuProgramParameters(Ogre::GPT_FRAGMENT_PROGRAM,
    pass->getFragmentProgramParameters(), 1);
  }

  _target->update(false);
}

//////////////////////////////////////////////////
void OgreGpuRays::Render()
{
  Ogre::SceneManager *sceneMgr = this->scene->OgreSceneManager();

  sceneMgr->_suppressRenderStateChanges(true);
  sceneMgr->addRenderObjectListener(this);

  for (unsigned int i = 0; i < this->dataPtr->textureCount; ++i)
  {
    if (this->dataPtr->textureCount > 1)
    {
      // Cannot call Camera::RotateYaw because it rotates in world frame,
      // but we need rotation in camera local frame
      this->Node()->roll(Ogre::Radian(this->dataPtr->cameraYaws[i]));
    }

    this->dataPtr->currentMat = this->dataPtr->matFirstPass;
    this->dataPtr->currentTexture = this->dataPtr->firstPassTextures[i];

    this->UpdateRenderTarget(
        this->dataPtr->firstPassTextures[i]->getBuffer()->getRenderTarget(),
        this->dataPtr->matFirstPass,
        this->dataPtr->ogreCamera, false);
  }

  if (this->dataPtr->textureCount > 1)
      this->Node()->roll(Ogre::Radian(this->dataPtr->cameraYaws[3]));

  sceneMgr->removeRenderObjectListener(this);

  this->dataPtr->visual->SetVisible(true);

  this->UpdateRenderTarget(
      this->dataPtr->secondPassTexture->getBuffer()->getRenderTarget(),
      this->dataPtr->matSecondPass,
      this->dataPtr->orthoCam, true);

  this->dataPtr->visual->SetVisible(false);

  sceneMgr->_suppressRenderStateChanges(false);
}

//////////////////////////////////////////////////
void OgreGpuRays::PreRender()
{
  if (this->dataPtr->textureCount == 0)
    this->CreateGpuRaysTextures();
}

//////////////////////////////////////////////////
void OgreGpuRays::PostRender()
{
  for (unsigned int i = 0; i < this->dataPtr->textureCount; ++i)
  {
    auto rt =
        this->dataPtr->firstPassTextures[i]->getBuffer()->getRenderTarget();
    rt->swapBuffers();
  }
  auto rt = this->dataPtr->secondPassTexture->getBuffer()->getRenderTarget();
  rt->swapBuffers();

  const Ogre::Viewport *secondPassViewport = rt->getViewport(0);
  unsigned int width = secondPassViewport->getActualWidth();
  unsigned int height = secondPassViewport->getActualHeight();

  size_t size = Ogre::PixelUtil::getMemorySize(
    width, height, 1, Ogre::PF_FLOAT32_RGB);
  int len = width * height * this->Channels();

  if (!this->dataPtr->gpuRaysBuffer)
  {
    this->dataPtr->gpuRaysBuffer = new float[len];
  }

  Ogre::PixelBox dstBox(width, height,
        1, Ogre::PF_FLOAT32_RGB, this->dataPtr->gpuRaysBuffer);

  auto pixelBuffer = this->dataPtr->secondPassTexture->getBuffer();
  pixelBuffer->blitToMemory(dstBox);

  if (!this->dataPtr->gpuRaysScan)
  {
    this->dataPtr->gpuRaysScan = new float[len];
  }

  memcpy(this->dataPtr->gpuRaysScan, this->dataPtr->gpuRaysBuffer, size);

  this->dataPtr->newGpuRaysFrame(this->dataPtr->gpuRaysScan,
      width, height, this->Channels(), "PF_FLOAT32_RGB");
}

//////////////////////////////////////////////////
const float* OgreGpuRays::Data() const
{
  return this->dataPtr->gpuRaysScan;
}

//////////////////////////////////////////////////
void OgreGpuRays::Copy(float *_dataDest)
{
  auto rt = this->dataPtr->secondPassTexture->getBuffer()->getRenderTarget();
  const Ogre::Viewport *secondPassViewport = rt->getViewport(0);
  unsigned int width = secondPassViewport->getActualWidth();
  unsigned int height = secondPassViewport->getActualHeight();

  size_t size = Ogre::PixelUtil::getMemorySize(
    width, height, 1, Ogre::PF_FLOAT32_RGB);

  memcpy(_dataDest, this->dataPtr->gpuRaysScan, size);
}

/////////////////////////////////////////////////
void OgreGpuRays::CreateOrthoCam()
{
  // create ogre camera object
  Ogre::SceneManager *ogreSceneManager;
  ogreSceneManager = this->scene->OgreSceneManager();
  if (ogreSceneManager == nullptr)
  {
    ignerr << "Scene manager cannot be obtained" << std::endl;
    return;
  }

  this->dataPtr->orthoCam = ogreSceneManager->createCamera(
      this->Name() + "_Ortho_Camera");
  if (this->dataPtr->orthoCam == nullptr)
  {
    ignerr << "Ogre camera cannot be created" << std::endl;
    return;
  }

  Ogre::SceneNode *rootSceneNode = std::dynamic_pointer_cast<
      OgreNode>(this->scene->RootVisual())->Node();
  this->dataPtr->pitchNodeOrtho = rootSceneNode->createChildSceneNode();
  this->dataPtr->pitchNodeOrtho->attachObject(this->dataPtr->orthoCam);

  // Use X/Y as horizon, Z up
  this->dataPtr->orthoCam->pitch(Ogre::Degree(90));

  // Don't yaw along variable axis, causes leaning
  this->dataPtr->orthoCam->setFixedYawAxis(true, Ogre::Vector3::UNIT_Z);
  this->dataPtr->orthoCam->setDirection(1, 0, 0);
  this->dataPtr->orthoCam->setAutoAspectRatio(true);

  if (this->dataPtr->orthoCam)
  {
    this->dataPtr->orthoCam->setNearClipDistance(0.01f);
    this->dataPtr->orthoCam->setFarClipDistance(0.02f);
    this->dataPtr->orthoCam->setRenderingDistance(0.02f);

    this->dataPtr->orthoCam->setProjectionType(Ogre::PT_ORTHOGRAPHIC);
  }
}

/////////////////////////////////////////////////
Ogre::Matrix4 OgreGpuRays::BuildScaledOrthoMatrix(const float _left,
    const float _right, const float _bottom, const float _top,
    const float _near, const float _far)
{
  float invw = 1 / (_right - _left);
  float invh = 1 / (_top - _bottom);
  float invd = 1 / (_far - _near);

  Ogre::Matrix4 proj = Ogre::Matrix4::ZERO;
  proj[0][0] = 2 * invw;
  proj[0][3] = -(_right + _left) * invw;
  proj[1][1] = 2 * invh;
  proj[1][3] = -(_top + _bottom) * invh;
  proj[2][2] = -2 * invd;
  proj[2][3] = -(_far + _near) * invd;
  proj[3][3] = 1;

  return proj;
}

/////////////////////////////////////////////////
void OgreGpuRays::Set1stTextureSize(
    const unsigned int _w, const unsigned int _h)
{
  this->dataPtr->w1st = _w;
  this->dataPtr->h1st = _h;
}

/////////////////////////////////////////////////
void OgreGpuRays::SetRangeCount(
    const unsigned int _w, const unsigned int _h)
{
  this->dataPtr->w2nd = _w;
  this->dataPtr->h2nd = _h;
}

/////////////////////////////////////////////////
void OgreGpuRays::CreateMesh()
{
  std::string meshName = this->Name() + "_undistortion_mesh";

  common::Mesh *mesh = new common::Mesh();
  mesh->SetName(meshName);

  common::SubMesh *submesh = new common::SubMesh();

  double dx, dy;
  submesh->SetPrimitiveType(common::SubMesh::POINTS);

  if (this->dataPtr->h2nd == 1)
  {
    dy = 0;
  }
  else
  {
    dy = 0.1;
  }

  dx = 0.1;

  // startX ranges from 0 to -(w2nd/10) at dx=0.1 increments
  // startY ranges from h2nd/10 to 0 at dy=0.1 decrements
  // see OgreGpuRays::Set2ndPassTarget() on how the ortho cam is set up
  double startX = dx;
  double startY = this->dataPtr->h2nd/10.0;

  // half of actual camera vertical FOV without padding
  double phi = this->VFOV().Radian() / 2.0;
  double phiCamera = phi + std::abs(this->VertHalfAngle());
  double theta = this->CosHorzFOV() / 2.0;

  if (this->dataPtr->h2nd == 1)
  {
    phi = 0;
  }

  // index of ray
  unsigned int ptsOnLine = 0;

  // total gpu rays hfov
  double thfov = this->dataPtr->textureCount * this->CosHorzFOV();
  double hstep = thfov / (this->dataPtr->w2nd - 1);
  double vstep = 2.0 * phi / (this->dataPtr->h2nd - 1);

  if (this->dataPtr->h2nd == 1)
  {
    vstep = 0;
  }

  for (unsigned int j = 0; j < this->dataPtr->h2nd; ++j)
  {
    double gamma = 0;
    if (this->dataPtr->h2nd != 1)
    {
      // gamma: current vertical angle w.r.t. camera
      gamma = vstep * j - phi + this->VertHalfAngle();
    }

    for (unsigned int i = 0; i < this->dataPtr->w2nd; ++i)
    {
      // current horizontal angle from start of gpu rays scan
      double delta = hstep * i;

      // index of texture that contains the depth value
      unsigned int texture = static_cast<unsigned int>(
          delta / this->CosHorzFOV());

      // cap texture index and horizontal angle
      if (texture > this->dataPtr->textureCount-1)
      {
        texture -= 1;
        delta -= hstep;
      }

      startX -= dx;
      if (ptsOnLine == this->dataPtr->w2nd)
      {
        ptsOnLine = 0;
        startX = 0;
        startY -= dy;
      }
      ptsOnLine++;

      // the texture/1000.0 value is used in the gpu rays_2nd_pass.frag shader
      // as a trick to determine which camera texture to use when stitching
      // together the final depth image.
      submesh->AddVertex(texture/1000.0, startX, startY);

      // first compute angle from the start of current camera's horizontal
      // min angle, then set delta to be angle from center of current camera.
      delta = delta - (texture * this->CosHorzFOV());
      delta = delta - theta;

      // adjust uv coordinates of depth texture to match projection of current
      // gpu ray the depth image plane.
      double u = 0.5 - tan(delta) / (2.0 * tan(theta));
      double v = 0.5 - (tan(gamma) * cos(theta)) /
          (2.0 * tan(phiCamera) * cos(delta));

      submesh->AddTexCoord(u, v);
      submesh->AddIndex(this->dataPtr->w2nd * j + i);
    }
  }

  mesh->AddSubMesh(*submesh);

  this->dataPtr->undistMesh = mesh;

  common::MeshManager::Instance()->AddMesh(this->dataPtr->undistMesh);
}

/////////////////////////////////////////////////
void OgreGpuRays::CreateCanvas()
{
  this->CreateMesh();

  this->dataPtr->visual = this->scene->CreateVisual(
      this->Name() + "second_pass_canvas");

  Ogre::SceneNode *visualSceneNode =  std::dynamic_pointer_cast<
    OgreNode>(this->dataPtr->visual)->Node();

  Ogre::Node *visualParent = visualSceneNode->getParent();
  if (visualParent != nullptr)
  {
    visualParent->removeChild(visualSceneNode);
  }
  this->dataPtr->pitchNodeOrtho->addChild(visualSceneNode);

  // Convert mesh from common::Mesh to mesh and add it to
  // the canvas visual
  MeshPtr renderingMesh = this->scene->CreateMesh(
      this->dataPtr->undistMesh);
  this->dataPtr->visual->AddGeometry(renderingMesh);

  this->dataPtr->visual->SetLocalPosition(0.01, 0, 0);
  this->dataPtr->visual->SetLocalRotation(0, 0, 0);

  MaterialPtr canvasMaterial =
    this->scene->CreateMaterial(this->Name() + "_green");
  canvasMaterial->SetAmbient(math::Color(0, 1, 0, 1));
  this->dataPtr->visual->SetMaterial(canvasMaterial);

  this->dataPtr->visual->SetVisible(true);
}

/////////////////////////////////////////////////
void OgreGpuRays::notifyRenderSingleObject(Ogre::Renderable *_rend,
      const Ogre::Pass* /*pass*/, const Ogre::AutoParamDataSource* /*source*/,
      const Ogre::LightList* /*lights*/, bool /*supp*/)
{
  // TODO(anyone): this function sets the retro for each obj
  // but currently just sets it to 0

  Ogre::Vector4 retro = Ogre::Vector4(0, 0, 0, 0);
  if (!_rend->hasCustomParameter(1))
  {
    _rend->setCustomParameter(1, Ogre::Vector4(0, 0, 0, 0));
  }

  Ogre::Pass *pass =
    this->dataPtr->currentMat->getBestTechnique()->getPass(0);
  Ogre::RenderSystem *renderSys =
    this->scene->OgreSceneManager()->getDestinationRenderSystem();

  Ogre::AutoParamDataSource autoParamDataSource;

  Ogre::RenderTarget *rt =
    this->dataPtr->currentTexture->getBuffer()->getRenderTarget();
  Ogre::Viewport *vp = rt->getViewport(0);

  renderSys->_setViewport(vp);
  autoParamDataSource.setCurrentRenderable(_rend);
  autoParamDataSource.setCurrentPass(pass);
  autoParamDataSource.setCurrentViewport(vp);
  autoParamDataSource.setCurrentRenderTarget(rt);
  autoParamDataSource.setCurrentSceneManager(this->scene->OgreSceneManager());
  autoParamDataSource.setCurrentCamera(this->dataPtr->ogreCamera, true);

  pass->_updateAutoParams(&autoParamDataSource,
      Ogre::GPV_GLOBAL | Ogre::GPV_PER_OBJECT);
  pass->getFragmentProgramParameters()->setNamedConstant("retro", retro[0]);
  pass->getFragmentProgramParameters()->setNamedConstant(
      "max", static_cast<float>(this->dataMaxVal));
  pass->getFragmentProgramParameters()->setNamedConstant(
      "min", static_cast<float>(this->dataMinVal));
  renderSys->bindGpuProgram(
      pass->getVertexProgram()->_getBindingDelegate());

  renderSys->bindGpuProgramParameters(Ogre::GPT_VERTEX_PROGRAM,
      pass->getVertexProgramParameters(),
      Ogre::GPV_GLOBAL | Ogre::GPV_PER_OBJECT);

  renderSys->bindGpuProgram(
      pass->getFragmentProgram()->_getBindingDelegate());

  renderSys->bindGpuProgramParameters(Ogre::GPT_FRAGMENT_PROGRAM,
      pass->getFragmentProgramParameters(),
      (Ogre::GPV_GLOBAL | Ogre::GPV_PER_OBJECT));
}

//////////////////////////////////////////////////
common::ConnectionPtr OgreGpuRays::ConnectNewGpuRaysFrame(
    std::function<void(const float *_frame, unsigned int _width,
    unsigned int _height, unsigned int _channels,
    const std::string &/*_format*/)> _subscriber)
{
  return this->dataPtr->newGpuRaysFrame.Connect(_subscriber);
}

//////////////////////////////////////////////////
RenderTargetPtr OgreGpuRays::RenderTarget() const
{
  return this->dataPtr->renderTexture;
}

//////////////////////////////////////////////////
double OgreGpuRays::CosHorzFOV() const
{
  return this->dataPtr->chfov;
}

//////////////////////////////////////////////////
void OgreGpuRays::SetCosHorzFOV(const double _chfov)
{
  this->dataPtr->chfov = _chfov;
}

//////////////////////////////////////////////////
double OgreGpuRays::CosVertFOV() const
{
  return this->dataPtr->cvfov;
}

//////////////////////////////////////////////////
void OgreGpuRays::SetCosVertFOV(const double _cvfov)
{
  this->dataPtr->cvfov = _cvfov;
}

//////////////////////////////////////////////////
void OgreGpuRays::SetHorzHalfAngle(const double _angle)
{
  this->dataPtr->horzHalfAngle = _angle;
}

//////////////////////////////////////////////////
void OgreGpuRays::SetVertHalfAngle(const double _angle)
{
  this->dataPtr->vertHalfAngle = _angle;
}

//////////////////////////////////////////////////
double OgreGpuRays::HorzHalfAngle() const
{
  return this->dataPtr->horzHalfAngle;
}

//////////////////////////////////////////////////
double OgreGpuRays::VertHalfAngle() const
{
  return this->dataPtr->vertHalfAngle;
}
