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
#include "gz/rendering/ogre2/Ogre2Conversions.hh"
#include "gz/rendering/ogre2/Ogre2MaterialSwitcher.hh"
#include "gz/rendering/ogre2/Ogre2RenderEngine.hh"
#include "gz/rendering/ogre2/Ogre2RenderTarget.hh"
#include "gz/rendering/ogre2/Ogre2Scene.hh"
#include "gz/rendering/ogre2/Ogre2SelectionBuffer.hh"

#ifdef _MSC_VER
  #pragma warning(push, 0)
#endif
#include <Compositor/OgreCompositorManager2.h>
#include <Compositor/OgreCompositorWorkspace.h>
#include <Compositor/Pass/PassClear/OgreCompositorPassClearDef.h>
#include <Compositor/Pass/PassQuad/OgreCompositorPassQuadDef.h>
#include <Compositor/Pass/PassScene/OgreCompositorPassSceneDef.h>
#include <OgreCamera.h>
#include <OgreDepthBuffer.h>
#include <OgreItem.h>
#include <OgrePass.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreTechnique.h>
#include <OgreTextureGpuManager.h>
#include <OgreViewport.h>
#ifdef _MSC_VER
  #pragma warning(pop)
#endif

using namespace gz;
using namespace rendering;

struct gz::rendering::Ogre2SelectionBufferPrivate
{
  /// \brief This is a material listener and a RenderTargetListener.
  /// The material switcher is applied to only the selection camera
  /// and not applied globally to all targets. The class associates a
  /// color to an ogre entity
  public: std::unique_ptr<Ogre2MaterialSwitcher> materialSwitcher;

  /// \brief Ogre scene manager
  public: Ogre2ScenePtr scene;

  /// \brief Ogre scene manager
  public: Ogre::SceneManager *sceneMgr = nullptr;

  /// \brief Pointer to the camera that will be used as the reference
  /// for selection
  public: Ogre::Camera *camera = nullptr;

  /// \brief Selection buffer's render to texture camera
  public: Ogre::Camera *selectionCamera  = nullptr;

  /// \brief Ogre render texture
  public: Ogre::TextureGpu *renderTexture = nullptr;

  /// \brief Ogre compositor manager
  public: Ogre::CompositorManager2 *ogreCompMgr = nullptr;

  /// \brief selection buffer width
  public: unsigned int width = 0;

  /// \brief selection buffer height
  public: unsigned int height = 0;

  /// \brief Ogre's compositor workspace - the main interface to render
  /// into a render target or render texture.
  public: Ogre::CompositorWorkspace *ogreCompositorWorkspace = nullptr;

  /// \brief Name of the compositor workspace definition
  public: std::string ogreCompWorkspaceDefName;

  /// \brief The selection buffer material
  public: Ogre::MaterialPtr selectionMaterial;
};

/////////////////////////////////////////////////
Ogre2SelectionBuffer::Ogre2SelectionBuffer(const std::string &_cameraName,
    Ogre2ScenePtr _scene, unsigned int _width, unsigned int _height):
    dataPtr(new Ogre2SelectionBufferPrivate)
{
  this->dataPtr->scene = _scene;
  this->dataPtr->sceneMgr = _scene->OgreSceneManager();

  auto engine = Ogre2RenderEngine::Instance();
  auto ogreRoot = engine->OgreRoot();
  this->dataPtr->ogreCompMgr = ogreRoot->getCompositorManager2();

  this->dataPtr->width = _width;
  this->dataPtr->height = _height;

  this->dataPtr->camera = this->dataPtr->sceneMgr->findCameraNoThrow(
      _cameraName);
  if (!this->dataPtr->camera)
  {
    ignerr << "No camera found. Unable to create Ogre 2 selection buffer "
           << std::endl;
    return;
  }

  std::string selectionCameraName = _cameraName + "_selection_buffer";
  this->dataPtr->selectionCamera =
      this->dataPtr->sceneMgr->createCamera(selectionCameraName);

  this->dataPtr->selectionCamera->detachFromParent();
  this->dataPtr->sceneMgr->getRootSceneNode()->attachObject(
    this->dataPtr->selectionCamera);

  this->dataPtr->selectionCamera->setFOVy(this->dataPtr->camera->getFOVy());
  this->dataPtr->selectionCamera->setNearClipDistance(
    this->dataPtr->camera->getNearClipDistance());
  this->dataPtr->selectionCamera->setFarClipDistance(
     this->dataPtr->camera->getFarClipDistance());
  this->dataPtr->selectionCamera->setAspectRatio(
    this->dataPtr->camera->getAspectRatio());

  this->dataPtr->materialSwitcher.reset(
      new Ogre2MaterialSwitcher(this->dataPtr->scene));
  this->CreateRTTBuffer();
}

/////////////////////////////////////////////////
Ogre2SelectionBuffer::~Ogre2SelectionBuffer()
{
  this->DeleteRTTBuffer();

  // remove selectionMaterial in destructor
  // this does not need to be done in DeleteRTTBuffer as we do not need to
  // reload the same material every time
  if (!this->dataPtr->selectionMaterial.isNull())
  {
    Ogre::MaterialManager::getSingleton().remove(
        this->dataPtr->selectionMaterial->getName());
    this->dataPtr->selectionMaterial.setNull();
  }

  // remove selection buffer camera
  this->dataPtr->sceneMgr->destroyCamera(this->dataPtr->selectionCamera);
}

/////////////////////////////////////////////////
void Ogre2SelectionBuffer::Update()
{
  if (!this->dataPtr->renderTexture)
    return;

  this->dataPtr->materialSwitcher->Reset();

  this->dataPtr->scene->StartForcedRender();

  // manual update
  // this->dataPtr->ogreCompositorWorkspace->setEnabled(true);
  // auto engine = Ogre2RenderEngine::Instance();
  // engine->OgreRoot()->renderOneFrame();
  // this->dataPtr->ogreCompositorWorkspace->setEnabled(false);
  this->dataPtr->ogreCompositorWorkspace->_validateFinalTarget();
  this->dataPtr->ogreCompositorWorkspace->_beginUpdate(false);
  this->dataPtr->ogreCompositorWorkspace->_update();
  this->dataPtr->ogreCompositorWorkspace->_endUpdate(false);

  Ogre::vector<Ogre::TextureGpu *>::type swappedTargets;
  swappedTargets.reserve(2u);
  this->dataPtr->ogreCompositorWorkspace->_swapFinalTarget(swappedTargets);

  this->dataPtr->scene->FlushGpuCommandsAndStartNewFrame(1u, false);

  this->dataPtr->scene->EndForcedRender();

  // this->dataPtr->renderTexture->copyContentsToMemory(*this->dataPtr->pixelBox,
  //     Ogre::RenderTarget::FB_FRONT);
}

/////////////////////////////////////////////////
void Ogre2SelectionBuffer::DeleteRTTBuffer()
{
  if (this->dataPtr->ogreCompositorWorkspace)
  {
    // TODO(ahcorde): Remove the workspace. Potential leak here
    this->dataPtr->ogreCompMgr->removeWorkspace(
        this->dataPtr->ogreCompositorWorkspace);

    this->dataPtr->ogreCompMgr->removeWorkspaceDefinition(
        this->dataPtr->ogreCompWorkspaceDefName);
    this->dataPtr->ogreCompMgr->removeNodeDefinition(
        this->dataPtr->ogreCompWorkspaceDefName + "/Node");
    this->dataPtr->ogreCompositorWorkspace = nullptr;
  }

  auto engine = Ogre2RenderEngine::Instance();
  auto ogreRoot = engine->OgreRoot();
  ogreRoot->getRenderSystem()->getTextureGpuManager()->destroyTexture(
    this->dataPtr->renderTexture);
  this->dataPtr->renderTexture = nullptr;
}

/////////////////////////////////////////////////
void Ogre2SelectionBuffer::CreateRTTBuffer()
{
  auto engine = Ogre2RenderEngine::Instance();
  auto ogreRoot = engine->OgreRoot();

  Ogre::TextureGpuManager *textureMgr =
    ogreRoot->getRenderSystem()->getTextureGpuManager();
  const std::string selectionTextureName = "SelectionPassTex";
  bool hasSelectionTexture =
      textureMgr->findTextureNoThrow(selectionTextureName);
  this->dataPtr->renderTexture =
      textureMgr->createOrRetrieveTexture(
        selectionTextureName,
        Ogre::GpuPageOutStrategy::SaveToSystemRam,
        Ogre::TextureFlags::RenderToTexture,
        Ogre::TextureTypes::Type2D);
  if (!hasSelectionTexture)
  {
    this->dataPtr->renderTexture->setResolution(1, 1);
    this->dataPtr->renderTexture->setNumMipmaps(1u);
    this->dataPtr->renderTexture->setPixelFormat(Ogre::PFG_RGBA32_FLOAT);

    // we are reusing the same render texture so schedule transition only
    // if it is not resident yet otherwise it may throw an exception
    this->dataPtr->renderTexture->scheduleTransitionTo(
      Ogre::GpuResidency::Resident);
  }

  this->dataPtr->selectionCamera->addListener(
      this->dataPtr->materialSwitcher.get());

  // Load selection material
  // The SelectionBuffer material is defined in script
  // (selection_buffer.material).
  std::string matSelectionName = "SelectionBuffer";
  std::string matSelectionCloneName =
      this->dataPtr->camera->getName() + "_" + matSelectionName;
  if (this->dataPtr->selectionMaterial.isNull())
  {
    Ogre::MaterialPtr matSelection =
        Ogre::MaterialManager::getSingleton().getByName(matSelectionName);
    this->dataPtr->selectionMaterial = matSelection->clone(
        matSelectionCloneName);
    this->dataPtr->selectionMaterial->load();
  }
  Ogre::Pass *p = this->dataPtr->selectionMaterial->getTechnique(0)->getPass(0);
  Ogre::GpuProgramParametersSharedPtr psParams =
      p->getFragmentProgramParameters();

  // Set the uniform variables (selection_buffer_fs.glsl).
  // The projectParams is used to linearize depth buffer data
  double nearPlane = this->dataPtr->camera->getNearClipDistance();
  double farPlane = this->dataPtr->camera->getFarClipDistance();
  this->dataPtr->selectionCamera->setNearClipDistance(nearPlane);
  this->dataPtr->selectionCamera->setFarClipDistance(farPlane);

  Ogre::Vector2 projectionAB =
    this->dataPtr->selectionCamera->getProjectionParamsAB();
  double projectionA = projectionAB.x;
  double projectionB = projectionAB.y;
  projectionB /= farPlane;
  psParams->setNamedConstant("projectionParams",
      Ogre::Vector2(projectionA, projectionB));
  psParams->setNamedConstant("far",
      static_cast<float>(farPlane));
  psParams->setNamedConstant("inf",
      static_cast<float>(math::INF_F));

  // create compositor workspace for rendering
  // Setup the selection buffer compositor.
  this->dataPtr->ogreCompWorkspaceDefName = "SelectionBufferWorkspace" +
      this->dataPtr->camera->getName();

  std::string nodeSpaceDefName =
      this->dataPtr->ogreCompWorkspaceDefName + "/Node";

  Ogre::CompositorNodeDef *nodeDef =
      this->dataPtr->ogreCompMgr->addNodeDefinition(
      nodeSpaceDefName);
  Ogre::TextureDefinitionBase::TextureDefinition *depthTexDef =
      nodeDef->addTextureDefinition("depthTexture");
  depthTexDef->textureType = Ogre::TextureTypes::Type2D;
  depthTexDef->width = 0;
  depthTexDef->height = 0;
  depthTexDef->depthOrSlices = 1;
  depthTexDef->numMipmaps = 0;
  depthTexDef->widthFactor = 1;
  depthTexDef->heightFactor = 1;
  depthTexDef->format = Ogre::PFG_D32_FLOAT;
  depthTexDef->textureFlags &= ~Ogre::TextureFlags::Uav;
  depthTexDef->depthBufferId = Ogre::DepthBuffer::POOL_DEFAULT;
  depthTexDef->depthBufferFormat = Ogre::PFG_UNKNOWN;
  depthTexDef->fsaa = "0";

  Ogre::TextureDefinitionBase::TextureDefinition *colorTexDef =
      nodeDef->addTextureDefinition("colorTexture");
  colorTexDef->textureType = Ogre::TextureTypes::Type2D;
  colorTexDef->width = 0;
  colorTexDef->height = 0;
  colorTexDef->depthOrSlices = 1;
  colorTexDef->numMipmaps = 0;
  colorTexDef->widthFactor = 1;
  colorTexDef->heightFactor = 1;
  colorTexDef->format = Ogre::PFG_RGBA8_UNORM;
  colorTexDef->textureFlags &= ~Ogre::TextureFlags::Uav;
  colorTexDef->depthBufferId = Ogre::DepthBuffer::POOL_DEFAULT;
  colorTexDef->depthBufferFormat = Ogre::PFG_D32_FLOAT;
  colorTexDef->preferDepthTexture = true;
  colorTexDef->fsaa = "0";

  // Auto setup the RTV then manually override the depth buffer so
  // it uses the one we created (and thus we can sample from it later)
  Ogre::RenderTargetViewDef *rtvColor =
    nodeDef->addRenderTextureView("colorTexture");
  rtvColor->setForTextureDefinition("colorTexture", colorTexDef);
  rtvColor->depthAttachment.textureName = "depthTexture";

  // Input texture
  nodeDef->addTextureSourceName("rt", 0,
      Ogre::TextureDefinitionBase::TEXTURE_INPUT);

  nodeDef->setNumTargetPass(2);
  Ogre::CompositorTargetDef *colorTargetDef =
      nodeDef->addTargetPass("colorTexture");
  colorTargetDef->setNumPasses(1);
  {
    // scene pass
    Ogre::CompositorPassSceneDef *passScene =
        static_cast<Ogre::CompositorPassSceneDef *>(
        colorTargetDef->addPass(Ogre::PASS_SCENE));
    passScene->setAllLoadActions(Ogre::LoadAction::Clear);
    passScene->setAllClearColours(Ogre::ColourValue::Black);
    passScene->setVisibilityMask(IGN_VISIBILITY_SELECTABLE);
  }

  Ogre::CompositorTargetDef *targetDef = nodeDef->addTargetPass("rt");
  targetDef->setNumPasses(1);
  {
    // quad pass
    Ogre::CompositorPassQuadDef *passQuad =
        static_cast<Ogre::CompositorPassQuadDef *>(
        targetDef->addPass(Ogre::PASS_QUAD));
    passQuad->setAllLoadActions(Ogre::LoadAction::Clear);
    passQuad->setAllClearColours(Ogre::ColourValue::Black);
    passQuad->mMaterialName = this->dataPtr->selectionMaterial->getName();
    passQuad->addQuadTextureSource(0, "colorTexture");
    passQuad->addQuadTextureSource(1, "depthTexture");
    passQuad->mFrustumCorners =
        Ogre::CompositorPassQuadDef::VIEW_SPACE_CORNERS;
  }

  Ogre::CompositorWorkspaceDef *workDef =
      this->dataPtr->ogreCompMgr->addWorkspaceDefinition(
      this->dataPtr->ogreCompWorkspaceDefName);
  workDef->connectExternal(0, nodeDef->getName(), 0);

  this->dataPtr->ogreCompositorWorkspace =
      this->dataPtr->ogreCompMgr->addWorkspace(
        this->dataPtr->scene->OgreSceneManager(),
        this->dataPtr->renderTexture,
        this->dataPtr->selectionCamera,
        this->dataPtr->ogreCompWorkspaceDefName,
        false);
}

/////////////////////////////////////////////////
void Ogre2SelectionBuffer::SetDimensions(
  unsigned int _width, unsigned int _height)
{
  if (this->dataPtr->width == _width && this->dataPtr->height == _height)
    return;

  this->dataPtr->width = _width;
  this->dataPtr->height = _height;

  this->DeleteRTTBuffer();
  this->CreateRTTBuffer();
}
/////////////////////////////////////////////////
Ogre::Item *Ogre2SelectionBuffer::OnSelectionClick(const int _x, const int _y)
{
  Ogre::Item *item = nullptr;
  math::Vector3d point;
  this->ExecuteQuery(_x, _y, item, point);
  return item;
}

/////////////////////////////////////////////////
bool Ogre2SelectionBuffer::ExecuteQuery(const int _x, const int _y,
    Ogre::Item *&_item, math::Vector3d &_point)
{
  if (!this->dataPtr->renderTexture)
    return false;

  if (!this->dataPtr->camera)
    return false;

  // check camera has valid projection matrix
  // There could be nan values if camera was resized
  Ogre::Matrix4 projectionMatrix =
      this->dataPtr->camera->getProjectionMatrix();
  if (projectionMatrix.getTrans().isNaN() ||
      projectionMatrix.extractQuaternion().isNaN())
    return false;

   const unsigned int targetWidth = this->dataPtr->width;
   const unsigned int targetHeight = this->dataPtr->height;

   if (_x < 0 || _y < 0 || _x >= static_cast<int>(targetWidth)
       || _y >= static_cast<int>(targetHeight))
     return false;

  // 1x1 selection buffer, adapted from rviz
  // http://docs.ros.org/indigo/api/rviz/html/c++/selection__manager_8cpp.html
  unsigned int width = 1;
  unsigned int height = 1;
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
  Ogre::Matrix4 customProjectionMatrix =
      scaleMatrix * transMatrix *
      this->dataPtr->camera->getProjectionMatrix();
  this->dataPtr->selectionCamera->setCustomProjectionMatrix(true,
      customProjectionMatrix);

  this->dataPtr->selectionCamera->setPosition(
      this->dataPtr->camera->getDerivedPosition());
  this->dataPtr->selectionCamera->setOrientation(
      this->dataPtr->camera->getDerivedOrientation());

  // update render texture
  this->Update();

  Ogre::Image2 image;
  image.convertFromTexture(this->dataPtr->renderTexture, 0, 0);
  Ogre::ColourValue pixel = image.getColourAt(0, 0, 0, 0);

  float color = pixel[3];
  uint32_t *rgba = reinterpret_cast<uint32_t *>(&color);
  unsigned int r = *rgba >> 24 & 0xFF;
  unsigned int g = *rgba >> 16 & 0xFF;
  unsigned int b = *rgba >> 8 & 0xFF;

  math::Vector3d point(pixel[0], pixel[1], pixel[2]);

  auto rot = Ogre2Conversions::Convert(
      this->dataPtr->camera->getParentSceneNode()->_getDerivedOrientation());
  auto pos = Ogre2Conversions::Convert(
      this->dataPtr->camera->getParentSceneNode()->_getDerivedPosition());
  math::Pose3d p(pos, rot);
  point = rot * point + pos;

  ignition::math::Color cv;
  cv.A(1.0);
  cv.R(r / 255.0);
  cv.G(g / 255.0);
  cv.B(b / 255.0);

  const std::string &entName =
    this->dataPtr->materialSwitcher->EntityName(cv);

  if (entName.empty())
  {
    return false;
  }
  else
  {
    auto collection = this->dataPtr->sceneMgr->findMovableObjects(
        Ogre::ItemFactory::FACTORY_TYPE_NAME, entName);
    if (collection.empty())
      return false;
    else
    {
      _item = dynamic_cast<Ogre::Item *>(collection[0]);
      _point = point;
      return true;
    }
  }
}
